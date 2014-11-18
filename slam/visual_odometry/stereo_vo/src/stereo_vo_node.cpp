#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <px_comm/CameraInfo.h>
#include <ros/ros.h>

#include "cauldron/AtomicContainer.h"
#include "camera_models/CameraFactory.h"
#include "camera_systems/CameraSystem.h"
#include "sparse_graph/SparseGraphViz.h"
#include "stereo_vo/StereoVO.h"

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
stereoVOThread(std::vector<px::AtomicContainer<cv::Mat> >& frames,
               px::StereoVO& vo,
               ros::NodeHandle& nh)
{
    px::SparseGraphPtr sparseGraph = boost::make_shared<px::SparseGraph>();

    px::SparseGraphViz sgv(nh, sparseGraph);

    px::AtomicContainer<cv::Mat>& frameL = frames.at(0);
    px::AtomicContainer<cv::Mat>& frameR = frames.at(1);

    while (ros::ok())
    {
        bool process = false;
        if (frameL.available() && frameR.available() &&
            frameL.timestamp() == frameR.timestamp())
        {
            frameL.lockData();
            frameR.lockData();

            vo.readFrames(frameL.timestamp(),
                          frameL.data(),
                          frameR.data());

            frameL.available() = false;
            frameR.available() = false;

            frameL.unlockData();
            frameR.unlockData();

            process = true;
        }

        if (process)
        {
            px::FrameSetPtr frameSet;
            vo.processFrames(frameSet);

            if (vo.getCurrent2D3DCorrespondenceCount() < 40)
            {
                vo.keyCurrentFrameSet();

                sparseGraph->frameSetSegment(0).push_back(frameSet);

                sgv.visualize(10);
            }
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
    ros::init(argc, argv, "stereo_vo");

    ros::NodeHandle nh;

    // get namespaces of both cameras
    std::string cameraNs1, cameraNs2;
    if (!nh.getParam("camera_ns_1", cameraNs1))
    {
        ROS_ERROR("Cannot retrieve parameter: camera_ns_1");
        return 1;
    }
    if (!nh.getParam("camera_ns_2", cameraNs2))
    {
        ROS_ERROR("Cannot retrieve parameter: camera_ns_2");
        return 1;
    }

    ros::Subscriber cameraInfoSub;
    px_comm::CameraInfoPtr cameraInfo1;
    cameraInfoSub = nh.subscribe<px_comm::CameraInfo>(ros::names::append(cameraNs1, "camera_info"), 1,
                                                      boost::bind(cameraInfoCallback, _1, boost::ref(cameraInfo1)));

    ROS_INFO("Waiting for information for camera 1...");

    ros::Rate r(50);
    while (ros::ok() && cameraInfo1.get() == 0)
    {
        ros::spinOnce();
        r.sleep();
    }

    cameraInfoSub.shutdown();

    px_comm::CameraInfoPtr cameraInfo2;
    cameraInfoSub = nh.subscribe<px_comm::CameraInfo>(ros::names::append(cameraNs2, "camera_info"), 1,
                                                      boost::bind(cameraInfoCallback, _1, boost::ref(cameraInfo2)));

    ROS_INFO("Waiting for information for camera 2...");

    while (ros::ok() && cameraInfo2.get() == 0)
    {
        ros::spinOnce();
        r.sleep();
    }

    cameraInfoSub.shutdown();

    if (cameraInfo1.get() == 0 || cameraInfo2.get() == 0)
    {
        ROS_ERROR("Aborted due to missing camera information.");
        return 1;
    }

    ROS_INFO("Received stereo camera information.");
    ROS_INFO("Camera 1:");
    ROS_INFO("   Camera name: %s", cameraInfo1->camera_name.c_str());
    ROS_INFO("   Camera type: %s", cameraInfo1->camera_type.c_str());
    ROS_INFO("   Image width: %d", cameraInfo1->image_width);
    ROS_INFO("  Image height: %d", cameraInfo1->image_height);
    ROS_INFO("Camera 2:");
    ROS_INFO("   Camera name: %s", cameraInfo2->camera_name.c_str());
    ROS_INFO("   Camera type: %s", cameraInfo2->camera_type.c_str());
    ROS_INFO("   Image width: %d", cameraInfo2->image_width);
    ROS_INFO("  Image height: %d", cameraInfo2->image_height);

    px::CameraPtr camera1 = px::CameraFactory::instance()->generateCamera(cameraInfo1);
    px::CameraPtr camera2 = px::CameraFactory::instance()->generateCamera(cameraInfo2);

    px::CameraSystemPtr cameraSystem = boost::make_shared<px::CameraSystem>(2);
    cameraSystem->setCamera(0, camera1);
    cameraSystem->setCamera(1, camera2);
    cameraSystem->setGlobalCameraPose(0, cameraInfo1->pose);
    cameraSystem->setGlobalCameraPose(1, cameraInfo2->pose);

    px::StereoVO svo(cameraSystem, 0, 1, true);
    if (!svo.init("STAR", "ORB", "BruteForce-Hamming"))
    {
        ROS_ERROR("Failed to initialize stereo VO.");
        return 1;
    }

    image_transport::ImageTransport it(nh);
    std::vector<px::AtomicContainer<cv::Mat> > frames(2);
    image_transport::Subscriber imageSub1;
    imageSub1 = it.subscribe(ros::names::append(cameraNs1, "image_raw"), 1,
                             boost::bind(imageCallback, _1, boost::ref(frames.at(0))));

    image_transport::Subscriber imageSub2;
    imageSub2 = it.subscribe(ros::names::append(cameraNs2, "image_raw"), 1,
                             boost::bind(imageCallback, _1, boost::ref(frames.at(1))));

    boost::thread thread(boost::bind(&stereoVOThread, boost::ref(frames),
                                     boost::ref(svo), boost::ref(nh)));

    ros::spin();

    thread.join();

    return 0;
}
