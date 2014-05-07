#include "VRmagicDeviceDriver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vrmagic_device_node");

    ros::NodeHandle node;
    px::VRmagicDeviceDriver driver(node);
    if (!driver.start(0))
    {
        return -1;
    }

    ROS_INFO("Image publication is about to start...");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (node.ok() && driver.poll())
    {
    }

    spinner.stop();
    driver.stop();

    return 0;
}
