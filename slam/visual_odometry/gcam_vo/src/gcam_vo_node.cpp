#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "camera_models/CameraFactory.h"
#include "camera_systems/CameraSystem.h"
#include "cauldron/DataBuffer.h"
#include "sparse_graph/SparseGraphViz.h"
#include "gcam_vo/GCamVO.h"

class Container
{
public:
    Container(std::vector<cv::Mat>& _imageVec,
              px::DataBuffer<sensor_msgs::ImuConstPtr>& _imuBuffer,
              px::GCamVO& _gvo,
              px::SparseGraphPtr& _sparseGraph,
              px::SparseGraphViz& _sgv,
              ros::Publisher& _posePub)
     : imageVec(_imageVec)
     , imuBuffer(_imuBuffer)
     , gvo(_gvo)
     , sparseGraph(_sparseGraph)
     , sgv(_sgv)
     , posePub(_posePub)
    {

    }

    std::vector<cv::Mat>& imageVec;
    px::DataBuffer<sensor_msgs::ImuConstPtr>& imuBuffer;
    px::GCamVO& gvo;
    px::SparseGraphPtr& sparseGraph;
    px::SparseGraphViz& sgv;
    ros::Publisher& posePub;
};

void
cameraInfoCallback(const px_comm::CameraInfoConstPtr& msg,
                   px_comm::CameraInfoPtr& cameraInfo)
{
    cameraInfo = boost::make_shared<px_comm::CameraInfo>();

    *cameraInfo = *msg;
}

void
imuCallback(const sensor_msgs::ImuConstPtr& imuMsg,
            px::DataBuffer<sensor_msgs::ImuConstPtr>& imuBuffer)
{
    imuBuffer.push(imuMsg->header.stamp, imuMsg);
}

void
dataCallback(const sensor_msgs::ImageConstPtr& imageMsg0,
             const sensor_msgs::ImageConstPtr& imageMsg1,
             const sensor_msgs::ImageConstPtr& imageMsg2,
             const sensor_msgs::ImageConstPtr& imageMsg3,
             Container& container)
{
    ros::Time stamp = imageMsg0->header.stamp;

    sensor_msgs::ImuConstPtr imuMsg;
    if (!container.imuBuffer.find(stamp, imuMsg))
    {
        ROS_WARN("No IMU message with matching timestamp is found.");
        return;
    }

    std::vector<sensor_msgs::ImageConstPtr> imageMsgs;
    imageMsgs.push_back(imageMsg0);
    imageMsgs.push_back(imageMsg1);
    imageMsgs.push_back(imageMsg2);
    imageMsgs.push_back(imageMsg3);

    try
    {
        for (size_t i = 0; i < imageMsgs.size(); ++i)
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(imageMsgs.at(i));

            cv_ptr->image.copyTo(container.imageVec.at(i));
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    px::FrameSetPtr frameSet;
    if (!container.gvo.processFrames(stamp, container.imageVec, imuMsg, frameSet))
    {
        return;
    }

    Eigen::Quaterniond q = frameSet->systemPose()->rotation().conjugate();
    Eigen::Vector3d t = q * (- frameSet->systemPose()->translation());

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "vmav";
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.position.x = t(0);
    pose.pose.position.y = t(1);
    pose.pose.position.z = t(2);

    container.posePub.publish(pose);

    if (container.gvo.getCurrentCorrespondenceCount() < 50)
    {
        container.gvo.keyCurrentFrameSet();

        container.sparseGraph->frameSetSegment(0).push_back(frameSet);

        container.sgv.visualize(8);
    }
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "gcam_vo");

    ros::NodeHandle nh;

    // get IMU topic name
    std::string imuTopicName;
    if (!nh.getParam("imu_topic", imuTopicName))
    {
        ROS_ERROR("Cannot retrieve parameter: imu_topic");
        return 1;
    }

    // get pose topic name
    std::string poseTopicName;
    if (!nh.getParam("pose_topic", poseTopicName))
    {
        ROS_ERROR("Cannot retrieve parameter: pose_topic");
        return 1;
    }

    // get namespaces of all cameras
    std::vector<std::string> cameraNsVec;
    while (1)
    {
        std::ostringstream oss;
        oss << "camera_ns_" << cameraNsVec.size();

        std::string cameraNs;
        if (!nh.getParam(oss.str(), cameraNs))
        {
            break;
        }

        cameraNsVec.push_back(cameraNs);
    }

    if (cameraNsVec.empty())
    {
        ROS_ERROR("No parameter with the form camera_ns_* exists.");
        return 1;
    }

    // get information about camera system from CameraInfo messages
    px::CameraSystemPtr cameraSystem = boost::make_shared<px::CameraSystem>(cameraNsVec.size());
    for (size_t i = 0; i < cameraNsVec.size(); ++i)
    {
        ros::Subscriber cameraInfoSub;
        px_comm::CameraInfoPtr cameraInfo;
        cameraInfoSub = nh.subscribe<px_comm::CameraInfo>(ros::names::append(cameraNsVec.at(i), "camera_info"), 1,
                                                          boost::bind(cameraInfoCallback, _1, boost::ref(cameraInfo)));

        ROS_INFO("Waiting for information for camera %lu...", i);

        ros::Rate r(50);
        while (ros::ok() && cameraInfo.get() == 0)
        {
            ros::spinOnce();
            r.sleep();
        }

        cameraInfoSub.shutdown();

        if (!cameraInfo)
        {
            ROS_ERROR("Aborted due to missing camera information.");
            return 1;
        }

        px::CameraPtr camera = px::CameraFactory::instance()->generateCamera(cameraInfo);
        if (!camera)
        {
            ROS_ERROR("Factory is unable to generate a camera instance.");
            return 1;
        }

        cameraSystem->setCamera(i, camera);
        cameraSystem->setGlobalCameraPose(i, cameraInfo->pose);

        ROS_INFO("Processed information for camera %lu [%s].",
                 i, cameraInfo->camera_name.c_str());
    }

    px::GCamVO gvo(cameraSystem, false, true);
    if (!gvo.init("STAR", "ORB", "BruteForce-Hamming"))
    {
        ROS_ERROR("Failed to initialize generalized VO.");
        return 1;
    }

    std::vector<cv::Mat> imageVec(cameraSystem->cameraCount());
    px::DataBuffer<sensor_msgs::ImuConstPtr> imuBuffer(50);
    ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>(imuTopicName, 10, boost::bind(imuCallback, _1, boost::ref(imuBuffer)));

    std::vector<boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > > imageSubs(cameraSystem->cameraCount());
    for (size_t i = 0; i < cameraNsVec.size(); ++i)
    {
        imageSubs.at(i) = boost::make_shared<message_filters::Subscriber<sensor_msgs::Image> >(boost::ref(nh),
                                                                                               ros::names::append(cameraNsVec.at(i), "image_raw"),
                                                                                               2);
    }

    ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>(poseTopicName, 2);

    // assume for now that 4 cameras are used
    message_filters::TimeSynchronizer<sensor_msgs::Image,
                                      sensor_msgs::Image,
                                      sensor_msgs::Image,
                                      sensor_msgs::Image> sync(*imageSubs.at(0),
                                                               *imageSubs.at(1),
                                                               *imageSubs.at(2),
                                                               *imageSubs.at(3),
                                                               2);

    px::SparseGraphPtr sparseGraph = boost::make_shared<px::SparseGraph>();
    px::SparseGraphViz sgv(nh, sparseGraph);
    Container container(imageVec, imuBuffer, gvo, sparseGraph, sgv, posePub);

    sync.registerCallback(boost::bind(dataCallback, _1, _2, _3, _4,
                                      boost::ref(container)));

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
