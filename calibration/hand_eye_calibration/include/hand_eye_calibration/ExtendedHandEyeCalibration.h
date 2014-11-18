#ifndef EXTENDEDHANDEYECALIBRATION_H
#define EXTENDEDHANDEYECALIBRATION_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "hand_eye_calibration/DualQuaternion.h"

namespace px
{

class ExtendedHandEyeCalibration
{
public:
    ExtendedHandEyeCalibration();

    void solve(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H_1,
               const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H_2,
               Eigen::Matrix4d& H_1_2,
               double& s) const;

private:
    // solve ax^2 + bx + c = 0
    bool solveQuadraticEquation(double a, double b, double c, double& x1, double& x2) const;

    void refine(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H_1,
                const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H_2,
                Eigen::Matrix4d& H_12,
                double& s) const;
};

}

#endif
