#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "dynocmap/DynocMap.h"
#include "dynocmap_msgs/DynocMap.h"
#include "sensor_models/LaserSensorModel.h"

double k_maxRange = 5.0;

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg,
                        sensor_msgs::CameraInfoPtr& cameraInfo)
{
    if (!cameraInfo)
    {
        cameraInfo = boost::make_shared<sensor_msgs::CameraInfo>();
    }

    *cameraInfo = *msg;
}

void callback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg,
              const sensor_msgs::PointCloud2::ConstPtr& cloudMsg,
              px::DynocMapPtr& map,
              const Eigen::Matrix3d& cameraMatrix,
              const Eigen::Matrix4d& H_sensor_body,
              ros::Publisher& mapPub)
{
    float rangeThresh = k_maxRange - 1e-2;

    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(poseMsg->pose.orientation, q);

    Eigen::Vector3d t;
    tf::pointMsgToEigen(poseMsg->pose.position, t);

    map->recenter(t);

    Eigen::Matrix4d H_body_world = Eigen::Matrix4d::Identity();
    H_body_world.block<3,3>(0,0) = q.toRotationMatrix();
    H_body_world.block<3,1>(0,3) = t;

    Eigen::Matrix4d H_sensor_world = H_body_world * H_sensor_body;

    cv::Mat depthImage(cloudMsg->height, cloudMsg->width, CV_32F);
    const unsigned char* data = cloudMsg->data.data();
    size_t nPoints = 0;
    for (size_t r = 0; r < cloudMsg->height; ++r)
    {
        for (size_t c = 0; c < cloudMsg->width; ++c)
        {
            const float* P_data = reinterpret_cast<const float*>(data + nPoints * cloudMsg->point_step);

            if (P_data[2] > rangeThresh)
            {
                depthImage.at<float>(r, cloudMsg->width - c - 1) = k_maxRange + 1.0;
            }
            else
            {
                depthImage.at<float>(r, cloudMsg->width - c - 1) = P_data[2];
            }

            ++nPoints;
        }
    }

    map->castRays(H_sensor_world, depthImage, cameraMatrix, true);

    dynocmap_msgs::DynocMap msg;
    map->write(msg, "world");

    mapPub.publish(msg);
}

int main(int argc, char** argv)
{
    px::SensorModelPtr sensorModel = boost::make_shared<px::LaserSensorModel>(0.1, 0.9, k_maxRange, 0.02);
    px::DynocMapPtr map = boost::make_shared<px::DynocMap>(0.1, sensorModel, "mapcache");

    ros::init(argc, argv, "dynocmap_mapping");

    ros::NodeHandle nh;

    sensor_msgs::CameraInfoPtr cameraInfo;
    ros::Subscriber cameraInfoSub = nh.subscribe<sensor_msgs::CameraInfo>("/vrep/rgbd/info", 1,
                                                                          boost::bind(&cameraInfoCallback, _1, boost::ref(cameraInfo)));

    ROS_INFO("Waiting for camera information...");

    ros::Rate r(50);
    while (nh.ok() && !cameraInfo)
    {
        ros::spinOnce();
        r.sleep();
    }

    cameraInfoSub.shutdown();

    if (!nh.ok())
    {
        ROS_INFO("Aborting...");
        return 1;
    }

    Eigen::Matrix3d cameraMatrix = Eigen::Matrix3d::Identity();
    cameraMatrix(0,0) = cameraInfo->K[0];
    cameraMatrix(0,2) = cameraInfo->K[2];
    cameraMatrix(1,1) = cameraInfo->K[4];
    cameraMatrix(1,2) = cameraInfo->K[5];

    ROS_INFO_STREAM("Camera matrix:" << std::endl << cameraMatrix << std::endl);

    ROS_INFO("Waiting for sensor-body transform...");

    tf::TransformListener tfListener(nh);

    while (nh.ok() &&
           !tfListener.waitForTransform("/Quadricopter_rgbdSensor", "/Quadricopter_base", ros::Time(0), ros::Duration(1.0)))
    {

    }

    if (!nh.ok())
    {
        ROS_INFO("Aborting...");
        return 1;
    }

    tf::StampedTransform transform;
    try
    {
        tfListener.lookupTransform("/Quadricopter_rgbdSensor", "/Quadricopter_base", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return 1;
    }

    Eigen::Affine3d e;
    tf::transformTFToEigen(transform, e);

    Eigen::Matrix4d H_sensor_body = e.matrix().inverse();

    ros::Publisher mapPub = nh.advertise<dynocmap_msgs::DynocMap>("map", 1);

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub(nh, "/vrep/rgbd/cloud", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> poseSub(nh, "/vrep/pose", 1);

    message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> sync(poseSub, cloudSub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, boost::ref(map), boost::cref(cameraMatrix), boost::cref(H_sensor_body), boost::ref(mapPub)));

    ROS_INFO("Initialized!");

    ros::spin();

    return 0;
}
