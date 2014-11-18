#include <boost/program_options.hpp>
#include <px_comm/CameraInfo.h>
#include <px_comm/SetCameraInfo.h>
#include <ros/ros.h>

#include "camera_models/CameraFactory.h"
#include "camera_systems/CameraSystem.h"

int main(int argc, char** argv)
{
    std::string calibDir;
    std::string cameraSystemNs;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("calib", boost::program_options::value<std::string>(&calibDir)->default_value("calib"), "Directory containing calibration files.")
        ("ns", boost::program_options::value<std::string>(&cameraSystemNs)->default_value("/delta/vrmagic"), "Namespace of camera system.")
        ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    ros::init(argc, argv, "write_camera_info");

    ros::NodeHandle nh;

    px::CameraSystem cameraSystem;
    if (!cameraSystem.readFromDirectory(calibDir))
    {
        ROS_ERROR_STREAM("Failed to read calibration data from " << calibDir);
        return 1;
    }

    std::vector<std::string> camNs;
    for (size_t i = 0; i < cameraSystem.cameraCount(); ++i)
    {
        std::ostringstream oss;
        oss << "cam" << i;

        camNs.push_back(ros::names::append(cameraSystemNs, oss.str()));
    }

    // send SetCameraInfo request
    bool setCameraInfoSuccess = true;
    for (size_t i = 0; i < camNs.size(); ++i)
    {
        ros::ServiceClient cameraInfoClient;
        cameraInfoClient = nh.serviceClient<px_comm::SetCameraInfo>(ros::names::append(camNs.at(i), "set_camera_info"));

        px_comm::CameraInfoPtr cameraInfo = boost::make_shared<px_comm::CameraInfo>();
        cameraSystem.getCamera(i)->writeParameters(cameraInfo);

        Eigen::Matrix4d H = cameraSystem.getGlobalCameraPose(i);
        Eigen::Quaterniond q(H.block<3,3>(0,0));

        cameraInfo->pose.orientation.w = q.w();
        cameraInfo->pose.orientation.x = q.x();
        cameraInfo->pose.orientation.y = q.y();
        cameraInfo->pose.orientation.z = q.z();

        cameraInfo->pose.position.x = H(0,3);
        cameraInfo->pose.position.y = H(1,3);
        cameraInfo->pose.position.z = H(2,3);

        px_comm::SetCameraInfo setCameraInfo;
        setCameraInfo.request.camera_info = *cameraInfo;

        if (cameraInfoClient.call(setCameraInfo))
        {
            ROS_INFO("Received reply to SetCameraInfo request for camera [%s].",
                     cameraInfo->camera_name.c_str());

            if (setCameraInfo.response.success == true)
            {
                ROS_INFO("SetCameraInfo request for camera [%s] was processed successfully: %s.",
                         cameraInfo->camera_name.c_str(),
                         setCameraInfo.response.status_message.c_str());
            }
            else
            {
                setCameraInfoSuccess = false;

                ROS_ERROR("SetCameraInfo request for camera [%s] was not processed: %s.",
                          cameraInfo->camera_name.c_str(),
                          setCameraInfo.response.status_message.c_str());
            }
        }
        else
        {
            setCameraInfoSuccess = false;

            ROS_ERROR("Did not receive reply to SetCameraInfo request for camera [%s].",
                      cameraInfo->camera_name.c_str());
        }

        if (!setCameraInfoSuccess)
        {
            return 1;
        }
    }

    return 0;
}
