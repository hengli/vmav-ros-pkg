#include "VRmagicCamera.h"

#include <ndds/ndds_cpp.h>

#include <dds_ros/utils.h>
#include <px_comm/DDSImage.h>
#include <px_comm/DDSImageSupport.h>

namespace px
{

VRmagicCamera::VRmagicCamera(const std::string& ns,
                             ImageTransportType imageTransportType,
                             float fpsExpected,
                             const std::string& cameraName)
 : m_imageTransportType(imageTransportType)
 , m_cameraName(cameraName)
 , m_nodeHandle(ns)
 , m_imageTransport(m_nodeHandle)
 , m_cameraInfoPublisher(m_nodeHandle.advertise<px_comm::CameraInfo>("camera_info", 1))
 , m_cameraInfo(boost::make_shared<px_comm::CameraInfo>())
 , m_rosImage(boost::make_shared<sensor_msgs::Image>())
 , m_isFramePublished(true)
 , m_sensorPort(-1)
{
    if (imageTransportType == ROS)
    {
        m_imagePublisher = m_imageTransport.advertise("image_raw", 1);
    }

    m_diagnostics.setHardwareID(cameraName);

    m_minFreq = fpsExpected;
    m_maxFreq = fpsExpected;
    ROS_INFO("[%s] Expected frequency: %.2f (Hz)", cameraName.c_str(), fpsExpected);

    using namespace diagnostic_updater;
    m_diagnosticTopic = boost::make_shared<TopicDiagnostic>("vrmagic_frames",
                                                            boost::ref(m_diagnostics),
                                                            FrequencyStatusParam(&m_minFreq,
                                                                                 &m_maxFreq,
                                                                                 0.1, 100),
                                                            TimeStampStatusParam());
}

bool
VRmagicCamera::setup(void)
{
    if (m_imageTransportType == DDS)
    {
        // set up DDS
        m_ddsParticipant = createDDSParticipant(0, DDS_TRANSPORTBUILTIN_UDPv4);
        if (m_ddsParticipant == 0)
        {
            return false;
        }

        DDSPublisher* ddsPublisher =
            m_ddsParticipant->create_publisher(DDS_PUBLISHER_QOS_DEFAULT, 0, DDS_STATUS_MASK_NONE);

        if (ddsPublisher == 0)
        {
            ROS_ERROR("create_publisher error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        DDS_DataWriterQos writer_qos;
        DDS_ReturnCode_t retcode = ddsPublisher->get_default_datawriter_qos(writer_qos);

        if (retcode != DDS_RETCODE_OK)
        {
            ROS_ERROR("get_default_datawriter_qos error %d", retcode);
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        // use flow controller
        DDS_FlowControllerProperty_t property;
        retcode = m_ddsParticipant->get_default_flowcontroller_property(property);

        if (retcode != DDS_RETCODE_OK)
        {
            ROS_ERROR("get_default_flowcontroller_property error %d", retcode);
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        property.scheduling_policy = DDS_RR_FLOW_CONTROLLER_SCHED_POLICY;
        property.token_bucket.period.sec = 0;
        property.token_bucket.period.nanosec = 10000000;
        property.token_bucket.max_tokens = 100;
        property.token_bucket.tokens_added_per_period = 40;
        property.token_bucket.tokens_leaked_per_period = 0;
        property.token_bucket.bytes_per_token = 66000;

        DDSFlowController* controller = m_ddsParticipant->create_flowcontroller("image_flowcontroller", property);
        if (controller == 0)
        {
            ROS_ERROR("create_flowcontroller error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        // use non-strict reliability model
        writer_qos.publish_mode.kind = DDS_ASYNCHRONOUS_PUBLISH_MODE_QOS;
//        writer_qos.publish_mode.flow_controller_name = DDS_String_dup("image_flowcontroller");
        writer_qos.liveliness.lease_duration.sec = 1;
        writer_qos.liveliness.lease_duration.nanosec = 0;

        writer_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
        writer_qos.history.kind = DDS_KEEP_LAST_HISTORY_QOS;

        writer_qos.history.depth = 1;

        writer_qos.resource_limits.max_samples =
                writer_qos.resource_limits.initial_samples = 1;
        writer_qos.resource_limits.max_samples_per_instance =
                writer_qos.resource_limits.max_samples;

        writer_qos.protocol.rtps_reliable_writer.heartbeats_per_max_samples = 1;

        writer_qos.protocol.rtps_reliable_writer.high_watermark = 1;
        writer_qos.protocol.rtps_reliable_writer.low_watermark = 0;

        retcode = ddsPublisher->set_default_datawriter_qos(writer_qos);

        if (retcode != DDS_RETCODE_OK)
        {
            ROS_ERROR("set_default_datawriter_qos error %d", retcode);
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        // register type before creating topic
        const char* type_name = px_comm::DDSImageTypeSupport::get_type_name();
        retcode = px_comm::DDSImageTypeSupport::register_type(m_ddsParticipant, type_name);

        if (retcode != DDS_RETCODE_OK)
        {
            ROS_ERROR("register_type error %d", retcode);
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        std::ostringstream oss;
        oss << "cam" << (m_sensorPort - 1);

        DDSTopic* topic =
            m_ddsParticipant->create_topic(oss.str().c_str(), type_name,
                                           DDS_TOPIC_QOS_DEFAULT, 0, DDS_STATUS_MASK_NONE);

        if (topic == 0)
        {
            ROS_ERROR("create_topic error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        DDSDataWriter* writer =
            ddsPublisher->create_datawriter(topic, DDS_DATAWRITER_QOS_DEFAULT, 0,
                                            DDS_STATUS_MASK_NONE);
        if (writer == 0)
        {
            ROS_ERROR("create_datawriter error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        m_ddsImageWriter = px_comm::DDSImageDataWriter::narrow(writer);
        if (m_ddsImageWriter == 0)
        {
            ROS_ERROR("DataWriter narrow error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        m_ddsImage = px_comm::DDSImageTypeSupport::create_data();
        if (m_ddsImage == 0)
        {
            ROS_ERROR("create_data error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        ROS_INFO("[cam%d] Publishing to DDS topic: %s", m_sensorPort - 1, oss.str().c_str());
    }

    return true;
}

px_comm::CameraInfoPtr&
VRmagicCamera::cameraInfo(void)
{
    return m_cameraInfo;
}

std::string&
VRmagicCamera::cameraName(void)
{
    return m_cameraName;
}

sensor_msgs::ImagePtr&
VRmagicCamera::image(void)
{
    return m_rosImage;
}

void
VRmagicCamera::grabFrame(const ros::Time& stamp, const char* const imageData)
{
    m_cameraInfo->header.stamp = stamp;
    m_rosImage->header.stamp = stamp;

    if (m_imageTransportType == DDS)
    {
        m_ddsImage->seq = m_rosImage->header.seq;
        m_ddsImage->stamp_sec = m_rosImage->header.stamp.sec;
        m_ddsImage->stamp_nsec = m_rosImage->header.stamp.nsec;
        strncpy(m_ddsImage->frame_id, m_rosImage->header.frame_id.c_str(), 255);
        m_ddsImage->height = m_rosImage->height;
        m_ddsImage->width = m_rosImage->width;
        strncpy(m_ddsImage->encoding, m_rosImage->encoding.c_str(), 255);
        m_ddsImage->is_bigendian = m_rosImage->is_bigendian;
        m_ddsImage->step = m_rosImage->step;

        int imageSize = m_rosImage->step * m_rosImage->height;
        if (m_ddsImage->data.length() != imageSize)
        {
            m_ddsImage->data.ensure_length(imageSize, imageSize);
        }
        memcpy(m_ddsImage->data.get_contiguous_buffer(), imageData, imageSize);
    }
    else
    {
        memcpy(&m_rosImage->data.at(0), imageData, m_rosImage->data.size());
    }

    m_isFramePublished = false;
}

ros::NodeHandle&
VRmagicCamera::nodeHandle(void)
{
    return m_nodeHandle;
}

int&
VRmagicCamera::sensorPort(void)
{
    return m_sensorPort;
}

ros::ServiceServer&
VRmagicCamera::serviceServer(void)
{
    return m_serviceServer;
}

void
VRmagicCamera::publishFrame(void)
{
    if (m_isFramePublished)
    {
        return;
    }

    if (m_imageTransportType == DDS)
    {
        DDS_ReturnCode_t retcode;

        retcode = m_ddsImageWriter->write(*m_ddsImage, DDS_HANDLE_NIL);
        if (retcode != DDS_RETCODE_OK)
        {
            ROS_ERROR("write error %d", retcode);
        }
    }
    else
    {
        m_imagePublisher.publish(m_rosImage);
    }

    m_cameraInfoPublisher.publish(m_cameraInfo);

    m_diagnosticTopic->tick(m_rosImage->header.stamp);
    m_diagnostics.update();

    m_isFramePublished = true;
}

}
