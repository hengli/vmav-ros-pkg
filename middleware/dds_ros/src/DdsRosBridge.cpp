#include "DdsRosBridge.h"

// NDDS
#include <ndds/ndds_cpp.h>

#include <px_comm/DDSImage.h>
#include <px_comm/DDSImageSupport.h>

#include "dds_ros/utils.h"

namespace px
{

DdsRosBridge::DdsRosBridge()
 : k_nCameras(4)
 , m_nh("vrmagic")
 , m_imageTransport(m_nh)
{

}

bool
DdsRosBridge::start(void)
{
    // set up DDS
    m_ddsParticipant = createDDSParticipant(0, DDS_TRANSPORTBUILTIN_UDPv4);
    if (m_ddsParticipant == 0)
    {
        return false;
    }

    DDSSubscriber* ddsSubscriber =
        m_ddsParticipant->create_subscriber(DDS_SUBSCRIBER_QOS_DEFAULT, 0, DDS_STATUS_MASK_NONE);
    if (ddsSubscriber == 0)
    {
        ROS_ERROR("create_subscriber error");
        ddsShutdown(m_ddsParticipant);
        return false;
    }

    DDS_DataReaderQos reader_qos;
    DDS_ReturnCode_t retcode = ddsSubscriber->get_default_datareader_qos(reader_qos);

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("get_default_datareader_qos error %d", retcode);
        ddsShutdown(m_ddsParticipant);
        return false;
    }

    // use non-strict reliability model
    reader_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
    reader_qos.history.kind = DDS_KEEP_ALL_HISTORY_QOS;

    retcode = ddsSubscriber->set_default_datareader_qos(reader_qos);

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("set_default_datareader_qos error %d", retcode);
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

    m_ddsImageReader.resize(k_nCameras);
    m_waitset = new DDSWaitSet();
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

        DDSDataReader* ddsReader =
            ddsSubscriber->create_datareader(topic, DDS_DATAREADER_QOS_DEFAULT, 0,
                                             DDS_STATUS_MASK_NONE);
        if (ddsReader == 0)
        {
            ROS_ERROR("create_datareader error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        DDSStatusCondition* statusCondition = ddsReader->get_statuscondition();
        if (statusCondition == 0)
        {
            ROS_ERROR("get_statuscondition error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        m_ddsConditionMap.insert(std::make_pair(statusCondition, i));

        retcode = statusCondition->set_enabled_statuses(DDS_DATA_AVAILABLE_STATUS);
        if (retcode != DDS_RETCODE_OK)
        {
            ROS_ERROR("set_enabled_statuses error %d", retcode);
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        retcode = m_waitset->attach_condition(statusCondition);
        if (retcode != DDS_RETCODE_OK)
        {
            ROS_ERROR("attach_condition error %d", retcode);
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        m_ddsImageReader.at(i) = px_comm::DDSImageDataReader::narrow(ddsReader);
        if (m_ddsImageReader.at(i) == 0)
        {
            ROS_ERROR("DataReader narrow error");
            ddsShutdown(m_ddsParticipant);
            return false;
        }

        ROS_INFO("Subscribed to DDS topic: %s", oss.str().c_str());
    }

    // set up ROS
    m_rosImagePublisher.resize(k_nCameras);
    m_rosImage.resize(k_nCameras);

    for (int i = 0; i < k_nCameras; ++i)
    {
        std::ostringstream oss;
        oss << "cam" << i << "/image_raw";

        m_rosImagePublisher.at(i) = m_imageTransport.advertise(oss.str(), 1);
        m_rosImage.at(i) = boost::make_shared<sensor_msgs::Image>();

        ROS_INFO("Publishing to ROS topic: %s", m_rosImagePublisher.at(i).getTopic().c_str());
    }

    return true;
}

void
DdsRosBridge::stop(void)
{
    ddsShutdown(m_ddsParticipant);

    delete m_waitset;
}

void
DdsRosBridge::poll(void)
{
    struct DDS_Duration_t wait_timeout = {2,0};

    DDSConditionSeq activeConditionsSeq;
    DDS_ReturnCode_t retcode = m_waitset->wait(activeConditionsSeq, wait_timeout);

    if (retcode == DDS_RETCODE_TIMEOUT)
    {
        return;
    }

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("wait error: %d", retcode);
        return;
    }

    for (size_t i = 0; i < activeConditionsSeq.length(); ++i)
    {
        int cameraId = m_ddsConditionMap[activeConditionsSeq[i]];

        px_comm::DDSImageDataReader* reader = m_ddsImageReader.at(cameraId);
        px_comm::DDSImageSeq dataSeq;
        DDS_SampleInfoSeq infoSeq;

        retcode = reader->take(dataSeq, infoSeq,
                               DDS_LENGTH_UNLIMITED, DDS_ANY_SAMPLE_STATE,
                               DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE);

        if (retcode == DDS_RETCODE_NO_DATA)
        {
            continue;
        }
        else if (retcode != DDS_RETCODE_OK)
        {
            ROS_WARN("take error %d\n", retcode);
            continue;
        }

        for (int j = 0; j < dataSeq.length(); ++j)
        {
            if (!infoSeq[j].valid_data)
            {
                continue;
            }

            ddsImageCallback(&dataSeq[j], &m_rosImagePublisher.at(cameraId),
                             m_rosImage.at(cameraId));
        }

        retcode = reader->return_loan(dataSeq, infoSeq);
        if (retcode != DDS_RETCODE_OK)
        {
            ROS_WARN("return_loan error %d\n", retcode);
            continue;
        }
    }
}

void
DdsRosBridge::ddsImageCallback(px_comm::DDSImage* ddsImage,
                               image_transport::Publisher* rosPublisher,
                               sensor_msgs::ImagePtr& rosImage) const
{
    rosImage->header.seq = ddsImage->seq;
    rosImage->header.stamp.sec = ddsImage->stamp_sec;
    rosImage->header.stamp.nsec = ddsImage->stamp_nsec;
    rosImage->header.frame_id = std::string(ddsImage->frame_id);
    rosImage->height = ddsImage->height;
    rosImage->width = ddsImage->width;
    rosImage->encoding = std::string(ddsImage->encoding);
    rosImage->is_bigendian = ddsImage->is_bigendian;
    rosImage->step = ddsImage->step;

    unsigned int imageSize = rosImage->step * rosImage->height;
    if (rosImage->data.size() != imageSize)
    {
        rosImage->data.resize(imageSize);
    }

    memcpy(&rosImage->data[0], ddsImage->data.get_contiguous_buffer(), imageSize);

    rosPublisher->publish(rosImage);
}

}
