#include <boost/program_options.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "camera_models/CameraFactory.h"
#include "camera_systems/CameraSystem.h"
#include "cauldron/DataBuffer.h"
#include "gcam_slam/GCamSLAM.h"

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
             std::vector<cv::Mat>& imageVec,
             px::DataBuffer<sensor_msgs::ImuConstPtr>& imuBuffer,
             boost::shared_ptr<px::GCamSLAM>& slam)
{
    ros::Time stamp = imageMsg0->header.stamp;

    sensor_msgs::ImuConstPtr imuMsg;
    if (!imuBuffer.find(stamp, imuMsg))
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

            cv_ptr->image.copyTo(imageVec.at(i));
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    slam->processFrames(stamp, imageVec, imuMsg);
}

int main(int argc, char** argv)
{
    std::string calibDir;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("calib,c", boost::program_options::value<std::string>(&calibDir), "Calibration directory.")
        ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    ros::init(argc, argv, "gcam_slam");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    // get IMU topic name
    std::string imuTopicName;
    if (!pnh.getParam("imu_topic", imuTopicName))
    {
        ROS_ERROR("Cannot retrieve parameter: imu_topic");
        return 1;
    }

    // get pose topic name
    std::string poseTopicName;
    if (!pnh.getParam("pose_topic", poseTopicName))
    {
        ROS_ERROR("Cannot retrieve parameter: pose_topic");
        return 1;
    }

    // get vocabulary filename
    std::string vocFilename;
    if (!pnh.getParam("voc_file", vocFilename))
    {
        ROS_ERROR("Cannot retrieve parameter: voc_file");
        return 1;
    }

    // get namespaces of all cameras
    std::vector<std::string> cameraNsVec;
    while (1)
    {
        std::ostringstream oss;
        oss << "camera_ns_" << cameraNsVec.size();

        std::string cameraNs;
        if (!pnh.getParam(oss.str(), cameraNs))
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

    px::CameraSystemPtr cameraSystem;
    cameraSystem = boost::make_shared<px::CameraSystem>(cameraNsVec.size());

    if (calibDir.empty())
    {
        // get information about camera system from CameraInfo messages
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
    }
    else
    {
        ROS_INFO("Reading calibration data from %s", calibDir.c_str());

        if (!cameraSystem->readFromDirectory(calibDir))
        {
            ROS_ERROR("Unable to read calibration data from %s", calibDir.c_str());
            return 1;
        }
    }

    boost::shared_ptr<px::GCamSLAM> slam = boost::make_shared<px::GCamSLAM>(boost::ref(nh), cameraSystem);
    if (!slam->init("STAR", "ORB", "BruteForce-Hamming",
                    poseTopicName, vocFilename))
    {
        ROS_ERROR("Failed to initialize generalized SLAM.");
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
                                                                                               1);
    }

    // assume for now that 4 cameras are used
    message_filters::TimeSynchronizer<sensor_msgs::Image,
                                      sensor_msgs::Image,
                                      sensor_msgs::Image,
                                      sensor_msgs::Image> sync(*imageSubs.at(0),
                                                               *imageSubs.at(1),
                                                               *imageSubs.at(2),
                                                               *imageSubs.at(3),
                                                               1);
    sync.registerCallback(boost::bind(dataCallback, _1, _2, _3, _4,
                                      boost::ref(imageVec),
                                      boost::ref(imuBuffer),
                                      boost::ref(slam)));

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

    ROS_INFO("Shutting down...");

    slam->writeScenePointsToTextFile("vmav_slam_points.txt");
    slam->writePosesToTextFile("vmav_slam_poses.txt", true);

    return 0;
}
