#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <px_comm/CameraInfo.h>
#include <ros/ros.h>

#include "cauldron/AtomicContainer.h"
#include "camera_models/CameraFactory.h"
#include "camera_systems/CameraSystem.h"
#include "sparse_graph/SparseGraphViz.h"
#include "mono_vo/MonoVO.h"

void
imageCallback(const sensor_msgs::ImageConstPtr& msg,
              px::AtomicContainer<cv::Mat>& frame)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame.lockData();

    cv_ptr->image.copyTo(frame.data());

    frame.available() = true;
    frame.timestamp() = msg->header.stamp;

    frame.unlockData();
}

void
monoVOThread(std::vector<px::AtomicContainer<cv::Mat> >& frames,
             px::MonoVO& vo,
             ros::NodeHandle& nh)
{
    px::SparseGraphPtr sparseGraph = boost::make_shared<px::SparseGraph>();

    px::SparseGraphViz sgv(nh, sparseGraph);

    px::AtomicContainer<cv::Mat>& frame = frames.at(0);

    bool firstFrame = true;

    while (ros::ok())
    {
        bool process = false;
        if (frame.available())
        {
            frame.lockData();

            vo.readFrame(frame.timestamp(), frame.data());

            frame.available() = false;

            frame.unlockData();

            process = true;
        }

        if (process)
        {
            px::FrameSetPtr frameSet;
            vo.processFrames(frameSet);

            if (!firstFrame &&
                vo.getCurrentInlierCorrespondenceCount() > 0 &&
                vo.getCurrentInlierCorrespondenceCount() < 40)
            {
                vo.keyCurrentFrameSet();

                sparseGraph->frameSetSegment(0).push_back(frameSet);

                sgv.visualize(10);
            }

            firstFrame = false;
        }
    }
}

void
cameraInfoCallback(const px_comm::CameraInfoConstPtr& msg,
                   px_comm::CameraInfoPtr& cameraInfo)
{
    cameraInfo = boost::make_shared<px_comm::CameraInfo>();

    *cameraInfo = *msg;
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "mono_vo");

    ros::NodeHandle nh;

    // get camera namespace
    std::string cameraNs;
    if (!nh.getParam("camera_ns", cameraNs))
    {
        ROS_ERROR("Cannot retrieve parameter: camera_ns");
        return 1;
    }

    ros::Subscriber cameraInfoSub;
    px_comm::CameraInfoPtr cameraInfo;
    cameraInfoSub = nh.subscribe<px_comm::CameraInfo>(ros::names::append(cameraNs, "camera_info"), 1,
                                                      boost::bind(cameraInfoCallback, _1, boost::ref(cameraInfo)));

    ROS_INFO("Waiting for information for camera...");

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

    ROS_INFO("Received camera information.");
    ROS_INFO("Camera:");
    ROS_INFO("   Camera name: %s", cameraInfo->camera_name.c_str());
    ROS_INFO("   Camera type: %s", cameraInfo->camera_type.c_str());
    ROS_INFO("   Image width: %d", cameraInfo->image_width);
    ROS_INFO("  Image height: %d", cameraInfo->image_height);

    px::CameraPtr camera = px::CameraFactory::instance()->generateCamera(cameraInfo);

    px::CameraSystemPtr cameraSystem = boost::make_shared<px::CameraSystem>(1);
    cameraSystem->setCamera(0, camera);
    cameraSystem->setGlobalCameraPose(0, cameraInfo->pose);

    px::MonoVO mvo(cameraSystem, 0, true);
    if (!mvo.init("STAR", "ORB", "BruteForce-Hamming"))
    {
        ROS_ERROR("Failed to initialize monocular VO.");
        return 1;
    }

    image_transport::ImageTransport it(nh);
    std::vector<px::AtomicContainer<cv::Mat> > frames(1);
    image_transport::Subscriber imageSub;
    imageSub = it.subscribe(ros::names::append(cameraNs, "image_raw"), 1,
                            boost::bind(imageCallback, _1, boost::ref(frames.at(0))));

    boost::thread thread(boost::bind(&monoVOThread, boost::ref(frames),
                                     boost::ref(mvo), boost::ref(nh)));

    ros::spin();

    thread.join();

    return 0;
}
