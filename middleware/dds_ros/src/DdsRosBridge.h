#ifndef DDSROSBRIDGE_H
#define DDSROSBRIDGE_H

#include <boost/unordered_map.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

// forward declarations
class DDSCondition;
class DDSDomainParticipant;
class DDSWaitSet;

namespace px_comm
{
class DDSImageDataReader;
class DDSImage;
}

namespace px
{

class DdsRosBridge
{
public:
    DdsRosBridge();

    bool start(void);
    void stop(void);

    void poll(void);

private:
    void ddsImageCallback(px_comm::DDSImage* ddsImage,
                          image_transport::Publisher* rosPublisher,
                          sensor_msgs::ImagePtr& rosImage) const;

    const int k_nCameras;
    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_imageTransport;
    std::vector<image_transport::Publisher> m_rosImagePublisher;
    std::vector<sensor_msgs::ImagePtr> m_rosImage;

    DDSDomainParticipant* m_ddsParticipant;
    std::vector<px_comm::DDSImageDataReader*> m_ddsImageReader;
    boost::unordered_map<DDSCondition*, int> m_ddsConditionMap;
    DDSWaitSet* m_waitset;
};

}

#endif
