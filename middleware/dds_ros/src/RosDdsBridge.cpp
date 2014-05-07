#include "RosDdsBridge.h"

// NDDS
#include <ndds/ndds_cpp.h>

#include <px_comm/DDSImage.h>
#include <px_comm/DDSImageSupport.h>

#include "dds_ros/utils.h"

namespace px
{

RosDdsBridge::RosDdsBridge()
 : k_nCameras(4)
 , m_nh("vrmagic")
 , m_imageTransport(m_nh)
{

}

bool
RosDdsBridge::start(void)
{
    // set up DDS
    m_ddsParticipant = createDDSParticipant(0);
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

    // use non-strict reliability model
    writer_qos.publish_mode.kind = DDS_ASYNCHRONOUS_PUBLISH_MODE_QOS;
    writer_qos.liveliness.lease_duration.sec = 1;
    writer_qos.liveliness.lease_duration.nanosec = 0;

    writer_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
    writer_qos.history.kind = DDS_KEEP_LAST_HISTORY_QOS;
    writer_qos.history.depth = 2;

    writer_qos.resource_limits.max_samples =
            writer_qos.resource_limits.initial_samples = 2;
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

    m_ddsImageWriter.resize(k_nCameras);
    m_ddsImage.resize(k_nCameras);
    for (int i = 0; i < k_nCameras; ++i)
    {
        std::ostringstream oss;
        oss << "cam" << i;

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

        m_ddsImageWriter.at(i) = px_comm::DDSImageDataWriter::narrow(writer);
        if (m_ddsImageWriter.at(i) == 0)
        {
            ROS_ERROR("DataWriter narrow error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        m_ddsImage.at(i) = px_comm::DDSImageTypeSupport::create_data();
        if (m_ddsImage.at(i) == 0)
        {
            ROS_ERROR("create_data error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        ROS_INFO("Publishing to DDS topic: %s", oss.str().c_str());
    }

    // set up ROS
    m_rosImageSubscriber.resize(k_nCameras);
    for (int i = 0; i < k_nCameras; ++i)
    {
        std::ostringstream oss;
        oss << "cam" << i << "/image_raw";

        m_rosImageSubscriber.at(i) = m_imageTransport.subscribe(oss.str(), 1, boost::bind(&RosDdsBridge::rosImageCallback, this, _1, m_ddsImageWriter.at(i), m_ddsImage.at(i)));

        ROS_INFO("Subscribed to ROS topic: %s", m_rosImageSubscriber.at(i).getTopic().c_str());
    }

    return true;
}

void
RosDdsBridge::stop(void)
{
    ddsShutdown(m_ddsParticipant);
}

void
RosDdsBridge::poll(void)
{

}

void
RosDdsBridge::rosImageCallback(const sensor_msgs::ImageConstPtr& rosImage,
                               px_comm::DDSImageDataWriter* ddsImageWriter,
                               px_comm::DDSImage* ddsImage) const
{
    ddsImage->seq = rosImage->header.seq;
    ddsImage->stamp_sec = rosImage->header.stamp.sec;
    ddsImage->stamp_nsec = rosImage->header.stamp.nsec;
    strncpy(ddsImage->frame_id, rosImage->header.frame_id.c_str(), 255);
    ddsImage->height = rosImage->height;
    ddsImage->width = rosImage->width;
    strncpy(ddsImage->encoding, rosImage->encoding.c_str(), 255);
    ddsImage->is_bigendian = rosImage->is_bigendian;
    ddsImage->step = rosImage->step;

    int imageSize = rosImage->step * rosImage->height;
    if (imageSize != ddsImage->data.length())
    {
        ddsImage->data.ensure_length(imageSize, imageSize);
    }

    memcpy(ddsImage->data.get_contiguous_buffer(), &rosImage->data[0], imageSize);

    DDS_ReturnCode_t retcode;

    retcode = ddsImageWriter->write(*ddsImage, DDS_HANDLE_NIL);
    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("write error %d", retcode);
    }
}

}
