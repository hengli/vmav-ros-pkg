#ifndef ROSDDSBRIDGE_H
#define ROSDDSBRIDGE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

// forward declarations
class DDSDomainParticipant;

namespace px_comm
{
class DDSImageDataWriter;
class DDSImage;
}

namespace px
{

class RosDdsBridge
{
public:
    RosDdsBridge();

    bool start(void);
    void stop(void);

    void poll(void);

private:
    void rosImageCallback(const sensor_msgs::ImageConstPtr& rosImage,
                          px_comm::DDSImageDataWriter* ddsImageWriter,
                          px_comm::DDSImage* ddsImage) const;

    const int k_nCameras;
    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_imageTransport;
    std::vector<image_transport::Subscriber> m_rosImageSubscriber;

    DDSDomainParticipant* m_ddsParticipant;
    std::vector<px_comm::DDSImageDataWriter*> m_ddsImageWriter;
    std::vector<px_comm::DDSImage*> m_ddsImage;
};

}

#endif
