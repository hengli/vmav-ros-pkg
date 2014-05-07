#include <boost/thread.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "DdsRosBridge.h"

class DdsRosBridgeNodelet: public nodelet::Nodelet
{
public:
    DdsRosBridgeNodelet();
    virtual ~DdsRosBridgeNodelet();

    virtual void onInit(void);

private:
    virtual void bridgePoll(void);

    bool m_isRunning;
    boost::shared_ptr<px::DdsRosBridge> m_bridge;
    boost::shared_ptr<boost::thread> m_bridgeThread;
};

PLUGINLIB_DECLARE_CLASS(dds_to_ros, bridge, DdsRosBridgeNodelet, nodelet::Nodelet)

DdsRosBridgeNodelet::DdsRosBridgeNodelet()
 : m_isRunning(false)
{

}

DdsRosBridgeNodelet::~DdsRosBridgeNodelet()
{
    if (m_isRunning)
    {
        m_isRunning = false;
        m_bridgeThread->join();
        NODELET_INFO("Stopped bridge thread.");
    }

    m_bridge->stop();
}

void
DdsRosBridgeNodelet::onInit(void)
{
    m_bridge = boost::make_shared<px::DdsRosBridge>();
    if (!m_bridge->start())
    {
        NODELET_ERROR("Failed to start bridge.");
        return;
    }

    NODELET_INFO("Starting bridge thread.");

    m_isRunning = true;
    m_bridgeThread = boost::make_shared<boost::thread>(boost::bind(&DdsRosBridgeNodelet::bridgePoll, this));
}

void
DdsRosBridgeNodelet::bridgePoll(void)
{
    while (m_isRunning)
    {
        m_bridge->poll();
    }
}
