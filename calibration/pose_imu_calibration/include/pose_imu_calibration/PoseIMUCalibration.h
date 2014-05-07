#ifndef POSEIMUCALIBRATION_H
#define POSEIMUCALIBRATION_H

#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <vector>

#include "sparse_graph/Pose.h"

namespace px
{

class PoseIMUCalibration
{
public:
    bool calibrate(const std::vector<PoseConstPtr>& poseData,
                   const std::vector<sensor_msgs::ImuConstPtr>& imuData,
                   Eigen::Quaterniond& q_pose_imu);

    void errorStats(const Eigen::Quaterniond& q_pose_imu,
                    double& avgError, double& maxError) const;

private:
    void estimate(Eigen::Quaterniond& q_pose_imu) const;
    void refine(Eigen::Quaterniond& q_pose_imu) const;

    std::vector<PoseConstPtr> m_poseData;
    std::vector<sensor_msgs::ImuConstPtr> m_imuData;
};

}

#endif
