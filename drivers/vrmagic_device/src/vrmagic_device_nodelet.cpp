#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "VRmagicDeviceDriver.h"

class VRmagicDeviceNodelet: public nodelet::Nodelet
{
public:
    VRmagicDeviceNodelet();
    virtual ~VRmagicDeviceNodelet();

    virtual void onInit(void);

private:
    virtual void devicePoll(void);

    bool m_isRunning;
    boost::shared_ptr<px::VRmagicDeviceDriver> m_driver;
    boost::shared_ptr<boost::thread> m_deviceThread;
};

PLUGINLIB_DECLARE_CLASS(vrmagic_device, driver, VRmagicDeviceNodelet, nodelet::Nodelet)

VRmagicDeviceNodelet::VRmagicDeviceNodelet()
 : m_isRunning(false)
{

}

VRmagicDeviceNodelet::~VRmagicDeviceNodelet()
{
    if (m_isRunning)
    {
        m_isRunning = false;
        m_deviceThread->join();
        NODELET_INFO("Stopped driver thread.");
    }

    m_driver->stop();
}

void
VRmagicDeviceNodelet::onInit(void)
{
    ros::NodeHandle node;
    m_driver = boost::make_shared<px::VRmagicDeviceDriver>(node);
    if (!m_driver->start(0))
    {
        NODELET_ERROR("Failed to start driver.");
        return;
    }

    NODELET_INFO("Starting driver thread.");

    m_isRunning = true;
    m_deviceThread = boost::make_shared<boost::thread>(boost::bind(&VRmagicDeviceNodelet::devicePoll, this));
}

void
VRmagicDeviceNodelet::devicePoll(void)
{
    while (m_isRunning)
    {
        m_driver->poll();
    }
}
