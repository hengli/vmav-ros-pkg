#ifndef VRMAGICAMERA_H
#define VRMAGICAMERA_H

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <image_transport/image_transport.h>
#include <px_comm/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>

// forward declarations
class DDSDomainParticipant;

namespace px_comm
{
class DDSImageDataWriter;
class DDSImage;
}

namespace px
{

class VRmagicCamera
{
public:
    enum ImageTransportType
    {
        DDS,
        ROS
    };

    VRmagicCamera(const std::string& ns,
                  ImageTransportType imageTransportType,
                  float fpsExpected,
                  const std::string& cameraName = std::string());

    bool setup(void);

    px_comm::CameraInfoPtr& cameraInfo(void);
    std::string& cameraName(void);
    sensor_msgs::ImagePtr& image(void);

    void grabFrame(const ros::Time& stamp, const char* const imageData);

    ros::NodeHandle& nodeHandle(void);
    int& sensorPort(void);
    ros::ServiceServer& serviceServer(void);

    void publishFrame(void);

private:
    ImageTransportType m_imageTransportType;
    std::string m_cameraName;

    // ROS
    ros::NodeHandle m_nodeHandle;
    ros::ServiceServer m_serviceServer;
    image_transport::ImageTransport m_imageTransport;
    image_transport::Publisher m_imagePublisher;
    ros::Publisher m_cameraInfoPublisher;
    px_comm::CameraInfoPtr m_cameraInfo;
    sensor_msgs::ImagePtr m_rosImage;

    // DDS
    DDSDomainParticipant* m_ddsParticipant;
    px_comm::DDSImageDataWriter* m_ddsImageWriter;
    px_comm::DDSImage* m_ddsImage;

    bool m_isFramePublished;

    diagnostic_updater::Updater m_diagnostics;
    double m_minFreq;
    double m_maxFreq;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> m_diagnosticTopic;

    int m_sensorPort;
};

typedef boost::shared_ptr<VRmagicCamera> VRmagicCameraPtr;

}

#endif
