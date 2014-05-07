#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "camera_systems/CameraSystem.h"
#include "cauldron/EigenUtils.h"

int main(int argc, char** argv)
{
    std::string inputFilename;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input", boost::program_options::value<std::string>(&inputFilename)->default_value("calib/camera_system_extrinsics.txt"), "Extrinsic calibration file.")
        ;

    boost::program_options::positional_options_description pdesc;
    pdesc.add("input", 1);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(pdesc).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    ros::init(argc, argv, "visualize_camera_system");

    px::CameraSystem cameraSystem(4);
    if (!cameraSystem.readPosesFromTextFile(inputFilename))
    {
        ROS_ERROR("Failed to read extrinsic file.");
        return 1;
    }

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("cam_system_marker", 2);

    visualization_msgs::Marker lineMarker;

    lineMarker.header.frame_id = "vmav";
    lineMarker.header.stamp = ros::Time::now();
    lineMarker.ns = "cam_system_lines";
    lineMarker.id = 0;

    lineMarker.type = visualization_msgs::Marker::LINE_LIST;

    lineMarker.action = visualization_msgs::Marker::ADD;

    lineMarker.pose.position.x = 0.0;
    lineMarker.pose.position.y = 0.0;
    lineMarker.pose.position.z = 0.0;
    lineMarker.pose.orientation.x = 0.0;
    lineMarker.pose.orientation.y = 0.0;
    lineMarker.pose.orientation.z = 0.0;
    lineMarker.pose.orientation.w = 1.0;

    lineMarker.scale.x = 0.002;
    lineMarker.scale.y = 0.0;
    lineMarker.scale.z = 0.0;

    lineMarker.color.r = 1.0f;
    lineMarker.color.g = 1.0f;
    lineMarker.color.b = 1.0f;
    lineMarker.color.a = 1.f;

    lineMarker.lifetime = ros::Duration();

    visualization_msgs::Marker triangleMarker;

    triangleMarker.header.frame_id = "vmav";
    triangleMarker.header.stamp = ros::Time::now();
    triangleMarker.ns = "cam_system_triangles";
    triangleMarker.id = 0;

    triangleMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;

    triangleMarker.action = visualization_msgs::Marker::ADD;

    triangleMarker.pose.position.x = 0.0;
    triangleMarker.pose.position.y = 0.0;
    triangleMarker.pose.position.z = 0.0;
    triangleMarker.pose.orientation.x = 0.0;
    triangleMarker.pose.orientation.y = 0.0;
    triangleMarker.pose.orientation.z = 0.0;
    triangleMarker.pose.orientation.w = 1.0;

    triangleMarker.scale.x = 1.0;
    triangleMarker.scale.y = 1.0;
    triangleMarker.scale.z = 1.0;

    triangleMarker.color.r = 0.0f;
    triangleMarker.color.g = 1.0f;
    triangleMarker.color.b = 0.0f;
    triangleMarker.color.a = 1.f;

    triangleMarker.lifetime = ros::Duration();

    std_msgs::ColorRGBA axisColors[3];

    // x-axis
    axisColors[0].r = 1.0f;
    axisColors[0].g = 0.0f;
    axisColors[0].b = 0.0f;

    // y-axis
    axisColors[1].r = 0.0f;
    axisColors[1].g = 1.0f;
    axisColors[1].b = 0.0f;

    // z-axis
    axisColors[2].r = 0.0f;
    axisColors[2].g = 0.0f;
    axisColors[2].b = 1.0f;

    for (int i = 0; i < cameraSystem.cameraCount(); ++i)
    {
        Eigen::Matrix4d H_cam = cameraSystem.getGlobalCameraPose(i);

        double xBound = 0.06;
        double yBound = 0.06;
        double zFar = 0.08;

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > frustum;
        frustum.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        frustum.push_back(Eigen::Vector3d(-xBound, -yBound, zFar));
        frustum.push_back(Eigen::Vector3d(xBound, -yBound, zFar));
        frustum.push_back(Eigen::Vector3d(xBound, yBound, zFar));
        frustum.push_back(Eigen::Vector3d(-xBound, yBound, zFar));

        for (size_t k = 0; k < frustum.size(); ++k)
        {
            frustum.at(k) = px::transformPoint(H_cam, frustum.at(k));
        }

        for (int k = 1; k < 5; ++k)
        {
            geometry_msgs::Point p;

            p.x = frustum.at(0)(0);
            p.y = frustum.at(0)(1);
            p.z = frustum.at(0)(2);

            lineMarker.points.push_back(p);

            p.x = frustum.at(k)(0);
            p.y = frustum.at(k)(1);
            p.z = frustum.at(k)(2);

            lineMarker.points.push_back(p);
        }

        std_msgs::ColorRGBA color;

        switch (i)
        {
        case 0:
            color.r = 1.0f;
            color.g = 0.0f;
            color.b = 0.0f;
            color.a = 0.5f;
            break;
        case 1:
            color.r = 0.0f;
            color.g = 1.0f;
            color.b = 0.0f;
            color.a = 0.5f;
            break;
        case 2:
            color.r = 0.0f;
            color.g = 1.0f;
            color.b = 1.0f;
            color.a = 0.5f;
            break;
        case 3:
            color.r = 1.0f;
            color.g = 1.0f;
            color.b = 0.0f;
            color.a = 0.5f;
            break;
        default:
            color.r = 1.0f;
            color.g = 1.0f;
            color.b = 1.0f;
            color.a = 0.5f;
        }

        geometry_msgs::Point p0;

        p0.x = frustum.at(1)(0);
        p0.y = frustum.at(1)(1);
        p0.z = frustum.at(1)(2);

        triangleMarker.points.push_back(p0);
        triangleMarker.colors.push_back(color);

        for (int k = 2; k < 4; ++k)
        {
            geometry_msgs::Point p;

            p.x = frustum.at(k)(0);
            p.y = frustum.at(k)(1);
            p.z = frustum.at(k)(2);

            triangleMarker.points.push_back(p);
            triangleMarker.colors.push_back(color);
        }

        triangleMarker.points.push_back(p0);
        triangleMarker.colors.push_back(color);

        for (int k = 3; k < 5; ++k)
        {
            geometry_msgs::Point p;

            p.x = frustum.at(k)(0);
            p.y = frustum.at(k)(1);
            p.z = frustum.at(k)(2);

            triangleMarker.points.push_back(p);
            triangleMarker.colors.push_back(color);
        }
    }

    ros::Time tsStart = ros::Time::now();
    ros::Rate r(1);
    while (pub.getNumSubscribers() == 0 && (ros::Time::now() - tsStart).toSec() < 10.0)
    {
        r.sleep();
    }

    if (pub.getNumSubscribers() == 0)
    {
        ROS_ERROR("No subscribers.");
        return 1;
    }

    pub.publish(lineMarker);
    pub.publish(triangleMarker);

    return 0;
}
