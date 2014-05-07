#include <driver_base/SensorLevels.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "vicon_client/ViconClientConfig.h"
#include "ViconClient.h"

void reconfigure(vicon_client::ViconClientConfig& config,
                 uint32_t level,
                 px::ViconClient* viconClient)
{
    if (level & driver_base::SensorLevels::RECONFIGURE_CLOSE)
    {
        if (viconClient->isConnected())
        {
            viconClient->disconnect();
        }
    }

    if (!viconClient->isConnected())
    {
        if (!viconClient->connect(config.hostname))
        {
            ROS_WARN("Failed to connect to Vicon DataStream server at %s.", config.hostname.c_str());
        }
        else
        {
            viconClient->setSubject(config.subject_name);
        }
    }

    if (level == driver_base::SensorLevels::RECONFIGURE_RUNNING)
    {
        viconClient->setSubject(config.subject_name);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vicon_client_node");

    ros::NodeHandle nh("vicon");
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);

    dynamic_reconfigure::Server<vicon_client::ViconClientConfig> server(nh);

    px::ViconClient viconClient;
    server.setCallback(boost::bind(&reconfigure, _1, _2, &viconClient));

    ros::NodeHandle pnh("~vicon");
    std::string frame_id;
    pnh.param("frame_id", frame_id, std::string("vicon"));

    while (nh.ok())
    {
        if (!viconClient.isConnected())
        {
            continue;
        }

        viconClient.waitForFrame();

        geometry_msgs::PoseStamped pose;
        if (viconClient.getPose(pose))
        {
            pose.header.frame_id = frame_id;
            pub.publish(pose);
        }
    }

    if (viconClient.isConnected())
    {
        viconClient.disconnect();
    }

    return 0;
}
