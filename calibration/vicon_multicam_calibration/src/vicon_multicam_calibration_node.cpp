#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <fstream>
#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include <px_comm/CameraInfo.h>
#include <ros/ros.h>

#include "camera_calibration/Chessboard.h"
#include "cauldron/AtomicContainer.h"
#include "cauldron/EigenUtils.h"
#include "vicon_multicam_calibration/ViconMultiCamCalibration.h"

bool
parseConfigFile(const std::string& configFilename,
                std::vector<std::string>& cameraNs)
{
    std::ifstream ifs(configFilename.c_str());

    if (!ifs.is_open())
    {
        return false;
    }

    std::vector<std::string> buffer;
    while (ifs.good())
    {
        std::string tmp;
        std::getline(ifs, tmp);

        if (!tmp.empty())
        {
            buffer.push_back(tmp);
        }
    }

    if (buffer.size() < 2)
    {
        return false;
    }

    cameraNs = buffer;

    return true;
}

void
cameraInfoCallback(const px_comm::CameraInfoConstPtr& msg,
                   px_comm::CameraInfoPtr& cameraInfo)
{
    cameraInfo = boost::make_shared<px_comm::CameraInfo>();

    *cameraInfo = *msg;
}

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

    frame.available() = true;
    cv_ptr->image.copyTo(frame.data());
    frame.timestamp() = msg->header.stamp;
}

void
poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg,
             px::PosePtr& poseChessboard,
             px::PosePtr& poseMAV)
{
    px::Pose pose;
    pose.timeStamp() = msg->header.stamp;
    pose.rotation() = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                         msg->pose.pose.orientation.x,
                                         msg->pose.pose.orientation.y,
                                         msg->pose.pose.orientation.z);
    pose.translation() << msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z;

    if (msg->header.frame_id == "chessboard")
    {
        if (!poseChessboard)
        {
            poseChessboard = boost::make_shared<px::Pose>();
        }

        *poseChessboard = pose;
    }
    else
    {
        if (!poseMAV)
        {
            poseMAV = boost::make_shared<px::Pose>();
        }

        *poseMAV = pose;
    }
}

bool
isObjectImmobile(const px::Pose& lastImmobilePose, const px::Pose& currentPose)
{
    Eigen::Matrix4d H = px::invertHomogeneousTransform(lastImmobilePose.toMatrix()) *
                        currentPose.toMatrix();

    if (H.block<3,1>(0,3).norm() > 0.005)
    {
        return false;
    }

    Eigen::Matrix3d R = H.block<3,3>(0,0);

    Eigen::Vector3d attitude;
    px::mat2RPY(R, attitude(0), attitude(1), attitude(2));

    if (attitude.norm() > px::d2r(0.1))
    {
        return false;
    }

    return true;
}

int
main(int argc, char** argv)
{
    std::string configFilename;
    cv::Size boardSize;
    float squareSize;
    double delay;
    std::string cameraModel;
    std::string outputDir;
    bool readIntermediateData = false;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("config,c", boost::program_options::value<std::string>(&configFilename)->default_value("vicon_calib.cfg"), "Configuration file.")
        ("width,w", boost::program_options::value<int>(&boardSize.width)->default_value(9), "Number of inner corners on the chessboard pattern in x direction")
        ("height,h", boost::program_options::value<int>(&boardSize.height)->default_value(6), "Number of inner corners on the chessboard pattern in y direction")
        ("size,s", boost::program_options::value<float>(&squareSize)->default_value(120.f), "Size of one square in mm")
        ("delay", boost::program_options::value<double>(&delay)->default_value(0.5), "Minimum delay in seconds between captured images")
        ("camera-model", boost::program_options::value<std::string>(&cameraModel)->default_value("mei"), "Camera model: kannala-brandt | mei | pinhole")
        ("output,o", boost::program_options::value<std::string>(&outputDir)->default_value("vicon_calib"), "Output directory.")
        ("intermediate", boost::program_options::bool_switch(&readIntermediateData), "Read intermediate data in lieu of camera images.")
        ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    px::Camera::ModelType modelType;
    if (boost::iequals(cameraModel, "kannala-brandt"))
    {
        modelType = px::Camera::KANNALA_BRANDT;
    }
    else if (boost::iequals(cameraModel, "mei"))
    {
        modelType = px::Camera::MEI;
    }
    else if (boost::iequals(cameraModel, "pinhole"))
    {
        modelType = px::Camera::PINHOLE;
    }
    else
    {
        ROS_ERROR("Unknown camera model: %s", cameraModel.c_str());
        return 1;
    }

    switch (modelType)
    {
    case px::Camera::KANNALA_BRANDT:
        std::cout << "# INFO: Camera model: Kannala-Brandt" << std::endl;
        break;
    case px::Camera::MEI:
        std::cout << "# INFO: Camera model: Mei" << std::endl;
        break;
    case px::Camera::PINHOLE:
        std::cout << "# INFO: Camera model: Pinhole" << std::endl;
        break;
    }

    ros::init(argc, argv, "extrinsic_calibration");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    // get Vicon topic name
    std::string viconTopicName("/delta/xbee/vicon");
//    if (!pnh.getParam("vicon_topic", viconTopicName))
//    {
//        ROS_ERROR("Cannot retrieve parameter: vicon_topic");
//        return 1;
//    }

    std::vector<std::string> cameraNs;
    if (!parseConfigFile(configFilename, cameraNs))
    {
        ROS_ERROR("Failed to read configuration file %s", configFilename.c_str());
        return 1;
    }

    std::vector<px_comm::CameraInfoPtr> cameraInfoVec;
    for (size_t i = 0; i < cameraNs.size(); ++i)
    {
        ros::Subscriber cameraInfoSub;
        px_comm::CameraInfoPtr cameraInfo;
        cameraInfoSub = nh.subscribe<px_comm::CameraInfo>(ros::names::append(cameraNs.at(i), "camera_info"), 1,
                                                          boost::bind(cameraInfoCallback, _1, boost::ref(cameraInfo)));

        ROS_INFO("Waiting for information for camera %s...", cameraNs.at(i).c_str());

        ros::Rate r(50);
        while (ros::ok() && !cameraInfo)
        {
            ros::spinOnce();
            r.sleep();
        }

        cameraInfoSub.shutdown();

        if (!cameraInfo)
        {
            ROS_ERROR("Missing camera information for %s.", cameraNs.at(i).c_str());
            return 1;
        }

        cameraInfoVec.push_back(cameraInfo);
    }

    ROS_INFO("Received information for all cameras.");
    for (size_t i = 0; i < cameraInfoVec.size(); ++i)
    {
        ROS_INFO("Camera %lu:", i + 1);
        ROS_INFO("   Camera name: %s", cameraInfoVec.at(i)->camera_name.c_str());
        ROS_INFO("   Image width: %d", cameraInfoVec.at(i)->image_width);
        ROS_INFO("  Image height: %d", cameraInfoVec.at(i)->image_height);
    }

    image_transport::ImageTransport it(nh);
    std::vector<px::AtomicContainer<cv::Mat> > frameVec(cameraInfoVec.size());
    std::vector<boost::shared_ptr<image_transport::Subscriber> > imageSubVec(cameraInfoVec.size());
    for (size_t i = 0; i < cameraInfoVec.size(); ++i)
    {
        imageSubVec.at(i) = boost::make_shared<image_transport::Subscriber>();
        *(imageSubVec.at(i)) = it.subscribe(ros::names::append(cameraNs.at(i), "image_raw"), 1,
                                            boost::bind(imageCallback, _1, boost::ref(frameVec.at(i))));

        cv::namedWindow(cameraInfoVec.at(i)->camera_name);
    }

    px::PosePtr poseChessboard;
    px::PosePtr poseMAV;
    ros::Subscriber viconSub;
    viconSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(viconTopicName, 100,
                                                                      boost::bind(poseCallback, _1,
                                                                                  boost::ref(poseChessboard),
                                                                                  boost::ref(poseMAV)));

    ros::Rate r(50);

    ROS_INFO("Waiting for chessboard pose data...");
    while (ros::ok() && !poseChessboard)
    {
        ros::spinOnce();
        r.sleep();
    }
    if (!poseChessboard)
    {
        return 1;
    }

    ROS_INFO("Waiting for MAV pose data...");
    while (ros::ok() && !poseMAV)
    {
        ros::spinOnce();
        r.sleep();
    }
    if (!poseMAV)
    {
        return 1;
    }

    px::ViconMultiCamCalibration calibration(modelType,
                                             cameraInfoVec,
                                             boardSize,
                                             squareSize / 1000.0f);
    calibration.setVerbose(true);

    if (readIntermediateData)
    {
        if (!calibration.readData("vmc_data"))
        {
            ROS_ERROR("Unable to read internal data from vmc_data.");
            return 1;
        }
    }

    px::Pose lastChessboardPose;
    px::Pose lastChessboardImmobilePose;
    px::Pose lastMAVImmobilePose;
    ros::Time lastFrameStamp;
    std::vector<cv::Mat> imageViewVec(frameVec.size());
    bool startCalibration = false;
    while (ros::ok() && !startCalibration && !readIntermediateData)
    {
        ros::spinOnce();
        r.sleep();

        bool isFrameSetComplete = true;
        for (size_t i = 0; i < frameVec.size(); ++i)
        {
            if (!frameVec.at(i).available())
            {
                isFrameSetComplete = false;

                break;
            }
        }
        if (!isFrameSetComplete)
        {
            continue;
        }

        isFrameSetComplete = true;
        ros::Time timestamp = frameVec.at(0).timestamp();
        for (size_t i = 1; i < frameVec.size(); ++i)
        {
            if (frameVec.at(i).timestamp() != timestamp)
            {
                isFrameSetComplete = false;

                break;
            }
        }
        if (!isFrameSetComplete)
        {
            continue;
        }

        bool isChessboardImmobile = true;
        if (isObjectImmobile(lastChessboardImmobilePose, *poseChessboard))
        {
            if ((poseChessboard->timeStamp() - lastChessboardImmobilePose.timeStamp()).toSec() < 2.0)
            {
                isChessboardImmobile = false;
            }
        }
        else
        {
            lastChessboardImmobilePose = *poseChessboard;

            isChessboardImmobile = false;
        }

        bool isMAVImmobile = true;
        if (isObjectImmobile(lastMAVImmobilePose, *poseMAV))
        {
            if ((poseMAV->timeStamp() - lastMAVImmobilePose.timeStamp()).toSec() < 2.0)
            {
                isMAVImmobile = false;
            }
        }
        else
        {
            lastMAVImmobilePose = *poseMAV;

            isMAVImmobile = false;
        }

        std::vector<boost::shared_ptr<px::Chessboard> > chessboardVec(frameVec.size());
        std::vector<boost::shared_ptr<boost::thread> > threadVec(frameVec.size());
        for (size_t i = 0; i < frameVec.size(); ++i)
        {
            chessboardVec.at(i) = boost::make_shared<px::Chessboard>(boardSize, frameVec.at(i).data());

            threadVec.at(i) = boost::make_shared<boost::thread>(&px::Chessboard::findCorners, chessboardVec.at(i).get(), false);
        }

        for (size_t i = 0; i < frameVec.size(); ++i)
        {
            threadVec.at(i)->join();

            if (chessboardVec.at(i)->cornersFound())
            {
                chessboardVec.at(i)->getSketch().copyTo(imageViewVec.at(i));
            }
            else
            {
                frameVec.at(i).data().copyTo(imageViewVec.at(i));
            }
        }

        Eigen::Matrix4d H_rel_chessboard = px::invertHomogeneousTransform(lastChessboardPose.toMatrix()) *
                                           poseChessboard->toMatrix();
        Eigen::Matrix3d R_rel_chessboard = H_rel_chessboard.block<3,3>(0,0);
        Eigen::Vector3d attitude;
        px::mat2RPY(R_rel_chessboard, attitude(0), attitude(1), attitude(2));

        if (attitude.norm() + H_rel_chessboard.block<3,1>(0,3).norm() > 0.3 &&
            (timestamp - lastFrameStamp).toSec() > delay &&
            isChessboardImmobile && isMAVImmobile)
        {
            bool cornersFound = false;

            for (size_t i = 0; i < frameVec.size(); ++i)
            {
                if (chessboardVec.at(i)->cornersFound())
                {
                    cv::bitwise_not(imageViewVec.at(i), imageViewVec.at(i));
                    calibration.addChessboardData(i, chessboardVec.at(i)->getCorners(),
                                                  *poseChessboard, *poseMAV);

                    cornersFound = true;
                }
            }

            if (cornersFound)
            {
                lastFrameStamp = timestamp;

                lastChessboardPose = *poseChessboard;
            }
        }

        std::vector<int> sampleCount = calibration.sampleCount();
        for (size_t i = 0; i < imageViewVec.size(); ++i)
        {
            std::ostringstream oss;
            oss << sampleCount.at(i);

            if (!isChessboardImmobile)
            {
                oss << " [chessboard moving]";
            }
            if (!isMAVImmobile)
            {
                oss << " [MAV moving]";
            }

            cv::putText(imageViewVec.at(i), oss.str(), cv::Point(10,20),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, CV_AA);

            cv::imshow(cameraInfoVec.at(i)->camera_name, imageViewVec.at(i));
        }

        int key = cv::waitKey(2);
        if (key == static_cast<int>('c'))
        {
            startCalibration = true;
        }

        for (size_t i = 0; i < frameVec.size(); ++i)
        {
            frameVec.at(i).available() = false;
        }
    }

    for (size_t i = 0; i < cameraInfoVec.size(); ++i)
    {
        cv::destroyWindow(cameraInfoVec.at(i)->camera_name);
    }

    if (!ros::ok())
    {
        ROS_ERROR("Aborted calibration.");
        return 1;
    }

    calibration.writeData("vmc_data");

    // We assume that the transform between the chessboard frame and
    // the chessboard's Vicon marker frame is known.
    // However, this transform will be optimized in the calibration
    // as the known transform may not be exact.
    Eigen::Matrix3d R_cb_cbv = px::RPY2mat(-M_PI, -M_PI_2, -1.5 * M_PI_2);
    Eigen::Vector3d t_cb_cbv;
    t_cb_cbv << - squareSize / 1000.0f, 0.0, squareSize / 1000.0f;

    Eigen::Matrix4d H_cb_cbv = Eigen::Matrix4d::Identity();
    H_cb_cbv.block<3,3>(0,0) = R_cb_cbv;
    H_cb_cbv.block<3,1>(0,3) = t_cb_cbv;

    ROS_INFO("Calibrating...");

    ros::Time startTime = ros::Time::now();

    calibration.calibrate(H_cb_cbv);

    if (!boost::filesystem::exists(outputDir))
    {
        boost::filesystem::create_directory(outputDir);
    }

    px::CameraSystemConstPtr cameraSystem = calibration.getCameraSystem();
    cameraSystem->writePosesToTextFile(outputDir + "/" + "camera_system_extrinsics.txt");
    for (int i = 0; i < cameraSystem->cameraCount(); ++i)
    {
        cameraSystem->getCamera(i)->writeParametersToYamlFile(outputDir + "/" + cameraSystem->getCamera(i)->cameraName() + "_camera_calib.yaml");
    }

    ROS_INFO_STREAM("Wrote calibration files to "
                    << boost::filesystem::absolute(boost::filesystem::path(outputDir)).string());

    ROS_INFO("Calibration took a total time of %.1f sec.",
             (ros::Time::now() - startTime).toSec());

    return 0;
}
