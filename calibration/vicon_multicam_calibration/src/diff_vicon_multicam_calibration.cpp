#include <boost/program_options.hpp>
#include <ros/ros.h>

#include "camera_systems/CameraSystem.h"
#include "cauldron/cauldron.h"
#include "cauldron/EigenUtils.h"

int main(int argc, char** argv)
{
    std::string estCalibDir;
    std::string viconCalibDir;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("est,e", boost::program_options::value<std::string>(&estCalibDir), "Directory containing estimated calibration files.")
        ("vicon,v", boost::program_options::value<std::string>(&viconCalibDir), "Directory containing Vicon calibration files.")
        ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    px::CameraSystem estCS;
    if (!estCS.readFromDirectory(estCalibDir))
    {
        ROS_ERROR("Unable to read estimated calibration files from directory %s", estCalibDir.c_str());
        return 1;
    }

    px::CameraSystem viconCS;
    if (!viconCS.readFromDirectory(viconCalibDir))
    {
        ROS_ERROR("Unable to read Vicon calibration files from directory %s", viconCalibDir.c_str());
        return 1;
    }

    if (estCS.cameraCount() != viconCS.cameraCount())
    {
        ROS_ERROR("Camera counts do not match.");
        return 1;
    }

    if (estCS.cameraCount() <= 1)
    {
        ROS_ERROR("Camera count must be at least 2.");
        return 1;
    }

    for (int i = 1; i < viconCS.cameraCount(); ++i)
    {
        Eigen::Matrix4d H_vicon = viconCS.getGlobalCameraPose(0).inverse() * viconCS.getGlobalCameraPose(i);
        Eigen::Matrix3d R_vicon = H_vicon.block<3,3>(0,0);

        double roll_v, pitch_v, yaw_v;
        px::mat2RPY(R_vicon, roll_v, pitch_v, yaw_v);

        Eigen::Matrix4d H_est = estCS.getGlobalCameraPose(0).inverse() * estCS.getGlobalCameraPose(i);
        Eigen::Matrix3d R_est = H_est.block<3,3>(0,0);

        double roll_e, pitch_e, yaw_e;
        px::mat2RPY(R_est, roll_e, pitch_e, yaw_e);

        ROS_INFO("Transform between cameras 1 and %d (Estimated):", i + 1);
        ROS_INFO("  Rotation: r = %.3f (%.3f) | p = %.3f (%.3f) | y = %.3f (%.3f)",
                 px::r2d(roll_e), px::r2d(fabs(roll_e - roll_v)),
                 px::r2d(pitch_e), px::r2d(fabs(pitch_e - pitch_v)),
                 px::r2d(yaw_e), px::r2d(fabs(yaw_e - yaw_v)));
        ROS_INFO("  Translation: x = %.6f (%.6f) | y = %.6f (%.6f) | z = %.6f (%.6f)",
                 H_est(0,3), fabs(H_est(0,3) - H_vicon(0,3)),
                 H_est(1,3), fabs(H_est(1,3) - H_vicon(1,3)),
                 H_est(2,3), fabs(H_est(2,3) - H_vicon(2,3)));
    }

    Eigen::Matrix4d H_vicon = viconCS.getGlobalCameraPose(0);
    Eigen::Matrix3d R_vicon = H_vicon.block<3,3>(0,0);

    double roll_v, pitch_v, yaw_v;
    px::mat2RPY(R_vicon, roll_v, pitch_v, yaw_v);

    Eigen::Matrix4d H_est = estCS.getGlobalCameraPose(0);
    Eigen::Matrix3d R_est = H_est.block<3,3>(0,0);

    double roll_e, pitch_e, yaw_e;
    px::mat2RPY(R_est, roll_e, pitch_e, yaw_e);

    ROS_INFO("Transform between camera system and camera 1 (Estimated):");
    ROS_INFO("  Rotation: r = %.3f (%.3f) | p = %.3f (%.3f) | y = %.3f (%.3f)",
             px::r2d(roll_e), px::r2d(fabs(roll_e - roll_v)),
             px::r2d(pitch_e), px::r2d(fabs(pitch_e - pitch_v)),
             px::r2d(yaw_e), px::r2d(fabs(yaw_e - yaw_v)));

    return 0;
}
