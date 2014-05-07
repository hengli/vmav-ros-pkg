#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <px_comm/CameraInfo.h>
#include <px_comm/SetCameraInfo.h>
#include <ros/ros.h>

#include "camera_calibration/CameraCalibration.h"
#include "camera_calibration/Chessboard.h"
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

int
main(int argc, char** argv)
{
    cv::Size boardSize;
    float squareSize;
    double delay;
    int imageCount;
    std::string cameraModel;
    std::string cameraNs;
    float minMove;

    //========= Handling Program options =========
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("width,w", boost::program_options::value<int>(&boardSize.width)->default_value(8), "Number of inner corners on the chessboard pattern in x direction")
        ("height,h", boost::program_options::value<int>(&boardSize.height)->default_value(5), "Number of inner corners on the chessboard pattern in y direction")
        ("size,s", boost::program_options::value<float>(&squareSize)->default_value(120.f), "Size of one square in mm")
        ("delay", boost::program_options::value<double>(&delay)->default_value(0.5), "Minimum delay in seconds between captured images")
        ("count", boost::program_options::value<int>(&imageCount)->default_value(50), "Number of images to be taken for the calibration")
        ("camera-model", boost::program_options::value<std::string>(&cameraModel)->default_value("mei"), "Camera model: kannala-brandt | mei | pinhole")
        ("camera-ns", boost::program_options::value<std::string>(&cameraNs)->default_value("cam"), "Camera namespace")
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
        ROS_INFO("Camera model: Kannala-Brandt");
        break;
    case px::Camera::MEI:
        ROS_INFO("Camera model: Mei");
        break;
    case px::Camera::PINHOLE:
        ROS_INFO("Camera model: Pinhole");
        break;
    }

    ros::init(argc, argv, "mono_camera_calibration");

    ros::NodeHandle nh;

    ros::Subscriber cameraInfoSub;
    px_comm::CameraInfoPtr cameraInfo;
    cameraInfoSub = nh.subscribe<px_comm::CameraInfo>(ros::names::append(cameraNs, "camera_info"), 1,
                                                      boost::bind(cameraInfoCallback, _1, boost::ref(cameraInfo)));

    ROS_INFO("Waiting for camera information...");

    ros::Rate r(50);
    while (ros::ok() && cameraInfo.get() == 0)
    {
        ros::spinOnce();
        r.sleep();
    }

    cameraInfoSub.shutdown();

    if (cameraInfo.get() == 0)
    {
        ROS_ERROR("Aborted calibration due to missing camera information.");
        return 1;
    }

    ROS_INFO("Received camera information.");
    ROS_INFO("   Camera name: %s", cameraInfo->camera_name.c_str());
    ROS_INFO("   Image width: %d", cameraInfo->image_width);
    ROS_INFO("  Image height: %d", cameraInfo->image_height);

    px::CameraCalibration calibration(modelType, cameraInfo->camera_name,
                                      cv::Size(cameraInfo->image_width,
                                               cameraInfo->image_height),
                                      boardSize, squareSize / 1000.0f);
    calibration.setVerbose(true);

    cv::namedWindow("Image");

    image_transport::ImageTransport it(nh);
    px::AtomicContainer<cv::Mat> frame;
    image_transport::Subscriber imageSub;
    imageSub = it.subscribe(ros::names::append(cameraNs, "image_raw"), 1,
                            boost::bind(imageCallback, _1, boost::ref(frame)));

    cv::Point2f lastFirstCorner = cv::Point2f(std::numeric_limits<float>::max(),
                                              std::numeric_limits<float>::max());
    ros::Time lastFrameTime;

    cv::Mat imgView;
    while (ros::ok() && calibration.sampleCount() < imageCount)
    {
        ros::spinOnce();
        r.sleep();

        if (!frame.available())
        {
            continue;
        }

        px::Chessboard chessboard(boardSize, frame.data());
        chessboard.findCorners();

        if (chessboard.cornersFound())
        {
            chessboard.getSketch().copyTo(imgView);
        }
        else
        {
            frame.data().copyTo(imgView);
        }

        if (chessboard.cornersFound() &&
            (frame.timestamp() - lastFrameTime).toSec() > delay &&
            cv::norm(cv::Mat(lastFirstCorner - chessboard.getCorners()[0])) > minMove)
        {
            lastFirstCorner = chessboard.getCorners()[0];
            lastFrameTime = frame.timestamp();

            cv::bitwise_not(imgView, imgView);
            calibration.addChessboardData(chessboard.getCorners());
        }

        std::ostringstream oss;
        oss << calibration.sampleCount() << " / " << imageCount;

        cv::putText(imgView, oss.str(), cv::Point(10,20),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);

        cv::imshow("Image", imgView);
        cv::waitKey(2);

        frame.available() = false;
    }

    cv::destroyWindow("Image");

    if (calibration.sampleCount() < imageCount)
    {
        ROS_ERROR("Aborted calibration due to insufficient number of detected chessboards.");
        return 1;
    }

    ROS_INFO("Calibrating...");

    ros::Time startTime = ros::Time::now();

    calibration.calibrate();
    calibration.writeParameters(cameraInfo->camera_name + "_camera_calib.yaml");
    calibration.writeParameters(cameraInfo);
    calibration.writeChessboardData(cameraInfo->camera_name + "_chessboard_data.dat");

    ROS_INFO("Calibration took a total time of %.1f sec.",
             (ros::Time::now() - startTime).toSec());

    ROS_INFO("Wrote calibration file to %s",
             (cameraInfo->camera_name + "_camera_calib.yaml").c_str());

    // send SetCameraInfo request
    ros::ServiceClient cameraInfoClient;
    cameraInfoClient = nh.serviceClient<px_comm::SetCameraInfo>(ros::names::append(cameraNs, "set_camera_info"));

    px_comm::SetCameraInfo setCameraInfo;
    setCameraInfo.request.camera_info = *cameraInfo;

    if (cameraInfoClient.call(setCameraInfo))
    {
        ROS_INFO("Received reply to SetCameraInfo request.");

        if (setCameraInfo.response.success == true)
        {
            ROS_INFO("SetCameraInfo request was processed successfully: %s.",
                     setCameraInfo.response.status_message.c_str());
        }
        else
        {
            ROS_ERROR("SetCameraInfo request was processed successfully.");
        }
    }
    else
    {
        ROS_ERROR("Did not receive reply to SetCameraInfo request.");
    }

    cv::Mat mapX, mapY;
    px::CameraPtr& camera = calibration.camera();

    if (camera->modelType() == px::Camera::PINHOLE)
    {
        camera->initUndistortRectifyMap(mapX, mapY);
    }
    else
    {
        camera->initUndistortRectifyMap(mapX, mapY, 300.0f, 300.0f,
                                        cv::Size(camera->imageWidth(),
                                                 camera->imageHeight()),
                                        -1.0f, -1.0f);
    }

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();

        if (!frame.available())
        {
            continue;
        }

        cv::remap(frame.data(), imgView, mapX, mapY, cv::INTER_LINEAR);

        cv::putText(imgView, "Calibrated", cv::Point(10,20),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);

        cv::imshow("Image", imgView);
        cv::waitKey(2);

        frame.available() = false;
    }

    return 0;
}
