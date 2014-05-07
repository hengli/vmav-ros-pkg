#include <gtest/gtest.h>
#include <iostream>

#include "cauldron/cauldron.h"
#include "cauldron/EigenUtils.h"
#include "pose_imu_calibration/PoseIMUCalibration.h"

TEST(PoseIMUCalibration, MinimalCase)
{
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > q_pose, q_imu;

    Eigen::Matrix3d R_pose_imu;
    R_pose_imu = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q_pose_imu(R_pose_imu);

    for (int i = 0; i < 3; ++i)
    {
        Eigen::Vector3d rvec = Eigen::Vector3d::Random();

        q_imu.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(rvec.norm(), rvec.normalized())));
        q_pose.push_back(q_pose_imu.conjugate() * q_imu.back().conjugate());
    }

    std::vector<px::PoseConstPtr> poseData;
    std::vector<sensor_msgs::ImuConstPtr> imuData;
    for (size_t i = 0; i < q_pose.size(); ++i)
    {
        px::PosePtr pose = boost::make_shared<px::Pose>();
        pose->rotation() = q_pose.at(i);

        poseData.push_back(pose);

        sensor_msgs::ImuPtr imu = boost::make_shared<sensor_msgs::Imu>();

        imu->orientation.w = q_imu.at(i).w();
        imu->orientation.x = q_imu.at(i).x();
        imu->orientation.y = q_imu.at(i).y();
        imu->orientation.z = q_imu.at(i).z();

        imuData.push_back(imu);
    }

    px::PoseIMUCalibration calib;
    Eigen::Quaterniond q_pose_imu_est;
    ASSERT_TRUE(calib.calibrate(poseData, imuData, q_pose_imu_est));

    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(q_pose_imu.coeffs()(i), q_pose_imu_est.coeffs()(i), 1e-10) << "Coefficients differ at index " << i;
    }
}

TEST(PoseIMUCalibration, NoiseCase)
{
    srand(time(NULL));

    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > q_pose, q_imu;

    Eigen::Matrix3d R_pose_imu;
    R_pose_imu = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q_pose_imu(R_pose_imu);

    for (int i = 0; i < 100; ++i)
    {
        double roll = px::random(-M_PI_2, M_PI_2);
        double pitch = px::random(-M_PI_2, M_PI_2);
        double yaw = px::random(-M_PI, M_PI);

        q_imu.push_back(Eigen::Quaterniond(px::RPY2mat(roll, pitch, yaw)));

        roll += px::randomNormal(0.05);
        pitch += px::randomNormal(0.05);
        yaw += px::randomNormal(0.05);

        q_pose.push_back(q_pose_imu.conjugate() * Eigen::Quaterniond(px::RPY2mat(roll, pitch, yaw)).conjugate());
    }

    std::vector<px::PoseConstPtr> poseData;
    std::vector<sensor_msgs::ImuConstPtr> imuData;
    for (size_t i = 0; i < q_pose.size(); ++i)
    {
        px::PosePtr pose = boost::make_shared<px::Pose>();
        pose->rotation() = q_pose.at(i);

        poseData.push_back(pose);

        sensor_msgs::ImuPtr imu = boost::make_shared<sensor_msgs::Imu>();

        imu->orientation.w = q_imu.at(i).w();
        imu->orientation.x = q_imu.at(i).x();
        imu->orientation.y = q_imu.at(i).y();
        imu->orientation.z = q_imu.at(i).z();

        imuData.push_back(imu);
    }

    px::PoseIMUCalibration calib;
    Eigen::Quaterniond q_pose_imu_est;
    ASSERT_TRUE(calib.calibrate(poseData, imuData, q_pose_imu_est));

    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(q_pose_imu.coeffs()(i), q_pose_imu_est.coeffs()(i), 1e-2) << "Coefficients differ at index " << i;
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
