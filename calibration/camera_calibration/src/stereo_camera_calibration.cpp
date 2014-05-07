#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <px_comm/CameraInfo.h>
#include <px_comm/SetCameraInfo.h>
#include <ros/ros.h>

#include "camera_calibration/Chessboard.h"
#include "camera_calibration/StereoCameraCalibration.h"
#include "cauldron/AtomicContainer.h"

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
cameraInfoCallback(const px_comm::CameraInfoConstPtr& msg,
                   px_comm::CameraInfoPtr& cameraInfo)
{
    cameraInfo = boost::make_shared<px_comm::CameraInfo>();

    *cameraInfo = *msg;
}

int main(int argc, char** argv)
{
    cv::Size boardSize;
    float squareSize;
    double delay;
    int imageCount;
    std::string outputFilename;
    std::string cameraModel;
    std::string cameraNsL, cameraNsR;
    float minMove;

    //========= Handling Program options =========
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("width,w", boost::program_options::value<int>(&boardSize.width)->default_value(9), "Number of inner corners on the chessboard pattern in x direction")
        ("height,h", boost::program_options::value<int>(&boardSize.height)->default_value(6), "Number of inner corners on the chessboard pattern in y direction")
        ("size,s", boost::program_options::value<float>(&squareSize)->default_value(120.f), "Size of one square in mm")
        ("delay", boost::program_options::value<double>(&delay)->default_value(0.5), "Minimum delay in seconds between captured images")
        ("count", boost::program_options::value<int>(&imageCount)->default_value(50), "Number of images to be taken for the calibration")
        ("output,o", boost::program_options::value<std::string>(&outputFilename)->default_value("stereo_extrinsics.txt"), "Output filename for stereo calibration data")
        ("camera-model", boost::program_options::value<std::string>(&cameraModel)->default_value("mei"), "Camera model: kannala-brandt | mei | pinhole")
        ("camera-ns-l", boost::program_options::value<std::string>(&cameraNsL)->default_value("camL"), "Left camera namespace")
        ("camera-ns-r", boost::program_options::value<std::string>(&cameraNsR)->default_value("camR"), "Right camera namespace")
        ("move,m", boost::program_options::value<float>(&minMove)->default_value(20.f), "Minimal move in px of chessboard to be used")
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

    ros::init(argc, argv, "stereo_camera_calibration");

    ros::NodeHandle nh;

    ros::Subscriber cameraInfoSub;
    px_comm::CameraInfoPtr cameraInfoL;
    cameraInfoSub = nh.subscribe<px_comm::CameraInfo>(ros::names::append(cameraNsL, "camera_info"), 1,
                                                      boost::bind(cameraInfoCallback, _1, boost::ref(cameraInfoL)));

    ROS_INFO("Waiting for information for left camera...");

    ros::Rate r(50);
    while (ros::ok() && cameraInfoL.get() == 0)
    {
        ros::spinOnce();
        r.sleep();
    }

    cameraInfoSub.shutdown();

    px_comm::CameraInfoPtr cameraInfoR;
    cameraInfoSub = nh.subscribe<px_comm::CameraInfo>(ros::names::append(cameraNsR, "camera_info"), 1,
                                                      boost::bind(cameraInfoCallback, _1, boost::ref(cameraInfoR)));

    ROS_INFO("Waiting for information for right camera...");

    while (ros::ok() && cameraInfoR.get() == 0)
    {
        ros::spinOnce();
        r.sleep();
    }

    cameraInfoSub.shutdown();

    if (cameraInfoL.get() == 0 || cameraInfoR.get() == 0)
    {
        ROS_ERROR("Aborted calibration due to missing camera information.");
        return 1;
    }

    if (cameraInfoL->image_width != cameraInfoR->image_width ||
        cameraInfoL->image_height != cameraInfoR->image_height)
    {
        ROS_ERROR("Calibration does not support a stereo configuration with two different camera makes.");
        return 1;
    }

    ROS_INFO("Received stereo camera information.");
    ROS_INFO("Left Camera:");
    ROS_INFO("   Camera name: %s", cameraInfoL->camera_name.c_str());
    ROS_INFO("   Image width: %d", cameraInfoL->image_width);
    ROS_INFO("  Image height: %d", cameraInfoL->image_height);
    ROS_INFO("Right Camera:");
    ROS_INFO("   Camera name: %s", cameraInfoR->camera_name.c_str());
    ROS_INFO("   Image width: %d", cameraInfoR->image_width);
    ROS_INFO("  Image height: %d", cameraInfoR->image_height);

    px::StereoCameraCalibration calibration(modelType,
                                            cameraInfoL->camera_name,
                                            cameraInfoR->camera_name,
                                            cv::Size(cameraInfoL->image_width,
                                                     cameraInfoL->image_height),
                                            boardSize, squareSize / 1000.0f);
    calibration.setVerbose(true);

    cv::namedWindow("Left Image");
    cv::namedWindow("Right Image");

    image_transport::ImageTransport it(nh);
    px::AtomicContainer<cv::Mat> frameL;
    image_transport::Subscriber imageSubL;
    imageSubL = it.subscribe(ros::names::append(cameraNsL, "image_raw"), 1,
                             boost::bind(imageCallback, _1, boost::ref(frameL)));

    px::AtomicContainer<cv::Mat> frameR;
    image_transport::Subscriber imageSubR;
    imageSubR = it.subscribe(ros::names::append(cameraNsR, "image_raw"), 1,
                             boost::bind(imageCallback, _1, boost::ref(frameR)));

    cv::Point2f lastFirstCorner = cv::Point2f(std::numeric_limits<float>::max(),
                                              std::numeric_limits<float>::max());
    ros::Time lastFrameTime;

    cv::Mat imgViewL, imgViewR;
    while (ros::ok() && calibration.sampleCount() < imageCount)
    {
        ros::spinOnce();
        r.sleep();

        if (!(frameL.available() && frameR.available() &&
              frameL.timestamp() == frameR.timestamp()))
        {
            continue;
        }

        px::Chessboard chessboardL(boardSize, frameL.data());
        px::Chessboard chessboardR(boardSize, frameR.data());

        chessboardL.findCorners();
        chessboardR.findCorners();

        if (chessboardL.cornersFound() && chessboardR.cornersFound())
        {
            chessboardL.getSketch().copyTo(imgViewL);
            chessboardR.getSketch().copyTo(imgViewR);
        }
        else
        {
            frameL.data().copyTo(imgViewL);
            frameR.data().copyTo(imgViewR);
        }

        if (chessboardL.cornersFound() && chessboardR.cornersFound() &&
            (frameL.timestamp() - lastFrameTime).toSec() > delay &&
            cv::norm(cv::Mat(lastFirstCorner - chessboardL.getCorners()[0])) > minMove)
        {
            lastFirstCorner = chessboardL.getCorners()[0];
            lastFrameTime = frameL.timestamp();

            cv::bitwise_not(imgViewL, imgViewL);
            cv::bitwise_not(imgViewR, imgViewR);
            calibration.addChessboardData(chessboardL.getCorners(),
                                          chessboardR.getCorners());
        }

        std::ostringstream oss;
        oss << calibration.sampleCount() << " / " << imageCount;

        cv::putText(imgViewL, oss.str(), cv::Point(10,20),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);

        cv::putText(imgViewR, oss.str(), cv::Point(10,20),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);

        cv::imshow("Left Image", imgViewL);
        cv::imshow("Right Image", imgViewR);
        cv::waitKey(2);

        frameL.available() = false;
        frameR.available() = false;
    }

    cv::destroyWindow("Left Image");
    cv::destroyWindow("Right Image");

    if (calibration.sampleCount() < imageCount)
    {
        ROS_ERROR("Aborted calibration due to insufficient number of detected chessboards.");
        return 1;
    }

    ROS_INFO("Calibrating...");

    ros::Time startTime = ros::Time::now();

    calibration.calibrate();
    calibration.cameraLeft()->writeParametersToYamlFile(cameraInfoL->camera_name + "_camera_calib.yaml");
    calibration.cameraRight()->writeParametersToYamlFile(cameraInfoR->camera_name + "_camera_calib.yaml");
    calibration.writeExtrinsicParameters(outputFilename);
    calibration.writeParameters(cameraInfoL, cameraInfoR);
    calibration.writeChessboardData(cameraInfoL->camera_name + "_" +
                                    cameraInfoR->camera_name +
                                    "_chessboard_data.dat");

    ROS_INFO("Calibration took a total time of %.1f sec.",
             (ros::Time::now() - startTime).toSec());

    ROS_INFO("Wrote calibration files to %s and %s",
             (cameraInfoL->camera_name + "_camera_calib.yaml").c_str(),
             (cameraInfoR->camera_name + "_camera_calib.yaml").c_str());

    // send SetCameraInfo request
    ros::ServiceClient cameraInfoClient;
    cameraInfoClient = nh.serviceClient<px_comm::SetCameraInfo>(ros::names::append(cameraNsL, "set_camera_info"));

    px_comm::SetCameraInfo setCameraInfo;
    setCameraInfo.request.camera_info = *cameraInfoL;

    if (cameraInfoClient.call(setCameraInfo))
    {
        ROS_INFO("Received reply to SetCameraInfo request for left camera.");

        if (setCameraInfo.response.success == true)
        {
            ROS_INFO("SetCameraInfo request for left camera was processed successfully: %s.",
                     setCameraInfo.response.status_message.c_str());
        }
        else
        {
            ROS_ERROR("SetCameraInfo request for left camera was processed successfully.");
        }
    }
    else
    {
        ROS_ERROR("Did not receive reply to SetCameraInfo request for left camera.");
    }

    cameraInfoClient = nh.serviceClient<px_comm::SetCameraInfo>(ros::names::append(cameraNsR, "set_camera_info"));

    setCameraInfo.request.camera_info = *cameraInfoR;

    if (cameraInfoClient.call(setCameraInfo))
    {
        ROS_INFO("Received reply to SetCameraInfo request for right camera.");

        if (setCameraInfo.response.success == true)
        {
            ROS_INFO("SetCameraInfo request for right camera was processed successfully: %s.",
                     setCameraInfo.response.status_message.c_str());
        }
        else
        {
            ROS_ERROR("SetCameraInfo request for right camera was processed successfully.");
        }
    }
    else
    {
        ROS_ERROR("Did not receive reply to SetCameraInfo request for right camera.");
    }

    cv::Mat mapXL, mapYL, mapXR, mapYR;
    px::CameraPtr& cameraL = calibration.cameraLeft();
    px::CameraPtr& cameraR = calibration.cameraRight();

    if (cameraL->modelType() == px::Camera::PINHOLE)
    {
        cameraL->initUndistortRectifyMap(mapXL, mapYL);
        cameraR->initUndistortRectifyMap(mapXR, mapYR);
    }
    else
    {
        cameraL->initUndistortRectifyMap(mapXL, mapYL, 300.0f, 300.0f,
                                         cv::Size(cameraL->imageWidth(),
                                                  cameraL->imageHeight()),
                                         -1.0f, -1.0f);
        cameraR->initUndistortRectifyMap(mapXR, mapYR, 300.0f, 300.0f,
                                         cv::Size(cameraR->imageWidth(),
                                                  cameraR->imageHeight()),
                                         -1.0f, -1.0f);
    }

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();

        if (!frameL.available() || !frameR.available())
        {
            continue;
        }

        cv::remap(frameL.data(), imgViewL, mapXL, mapYL, cv::INTER_LINEAR);
        cv::remap(frameR.data(), imgViewR, mapXR, mapYR, cv::INTER_LINEAR);

        cv::putText(imgViewL, "Calibrated", cv::Point(10,20),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);
        cv::putText(imgViewR, "Calibrated", cv::Point(10,20),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);

        cv::imshow("Left Image", imgViewL);
        cv::imshow("Right Image", imgViewR);
        cv::waitKey(2);

        frameL.available() = false;
        frameR.available() = false;
    }

    return 0;
}
