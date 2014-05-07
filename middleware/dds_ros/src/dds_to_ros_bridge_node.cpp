#include "DdsRosBridge.h"

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "dds_to_ros_bridge");

    px::DdsRosBridge bridge;
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
