#ifndef VICONCLIENT_H
#define VICONCLIENT_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>

#include "Client.h"

namespace px
{

class ViconClient
{
public:
    ViconClient();

    bool connect(const std::string& viconHostName);
    bool disconnect(void);

    bool isConnected(void);

    void setSubject(const std::string& subjectName);
    void waitForFrame(void);

    bool getPose(geometry_msgs::PoseStamped &pose);

    void printFrame(void);

private:
    std::string toString(const bool value) const;
    std::string toString(const ViconDataStreamSDK::CPP::Direction::Enum direction) const;
    std::string toString(const ViconDataStreamSDK::CPP::DeviceType::Enum deviceType) const;
    std::string toString(const ViconDataStreamSDK::CPP::Unit::Enum unit) const;

    boost::shared_ptr<ViconDataStreamSDK::CPP::Client> m_client;
    boost::mutex m_clientMutex;

    ros::Time m_frameStamp;
    std::string m_subjectName;
};

}

#endif
