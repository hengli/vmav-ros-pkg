#include "RosDdsBridge.h"

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_to_dds_bridge");

    px::RosDdsBridge bridge;
    if (!bridge.start())
    {
        ROS_ERROR("Failed to start bridge.");
        return -1;
    }

    while (ros::ok())
    {
        bridge.poll();

        ros::spinOnce();
    }

    bridge.stop();

    return 0;
}
