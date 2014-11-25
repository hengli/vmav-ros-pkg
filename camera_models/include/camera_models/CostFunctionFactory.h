#ifndef COSTFUNCTIONFACTORY_H
#define COSTFUNCTIONFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "camera_models/Camera.h"

namespace ceres
{
    class CostFunction;
}

namespace px
{

enum
{
    CAMERA_INTRINSICS = 0x1,
    SYSTEM_CAMERA_TRANSFORM = 0x2,
    STEREO_TRANSFORM = 0x4,
    CAMERA_POSE = 0x8,
    SYSTEM_POSE = 0x10,
    SCENE_POINT = 0x20
};

class CostFunctionFactory
{
public:
    CostFunctionFactory();

    static boost::shared_ptr<CostFunctionFactory> instance(void);

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Vector2d& observed_p) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Vector3d& observed_P,
                                              const Eigen::Vector2d& observed_p) const;

    ceres::CostFunction* generateCostFunction(const Eigen::Quaterniond& q,
                                              const Eigen::Vector3d& t,
                                              const Eigen::Vector3d& observed_ray,
                                              int variablesToOptimize) const;

    ceres::CostFunction* generateCostFunction(const Eigen::Vector3d& observed_ray) const;

    ceres::CostFunction* generateCostFunction(const Eigen::Matrix4d& H,
                                              const Eigen::Vector3d& observed_ray) const;

    ceres::CostFunction* generateCostFunction(const Eigen::Vector3d& observed_ray_l,
                                              const Eigen::Vector3d& observed_ray_r,
                                              const Eigen::Quaterniond& q_s_l,
                                              const Eigen::Vector3d& t_s_l,
                                              const Eigen::Quaterniond& q_s_r,
                                              const Eigen::Vector3d& t_s_r) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& cameraL,
                                              const Eigen::Vector2d& observed_p_l,
                                              const CameraConstPtr& cameraR,
                                              const Eigen::Vector2d& observed_p_r,
                                              const Eigen::Vector3d& observed_P,
                                              int variablesToOptimize) const;

private:
    static boost::shared_ptr<CostFunctionFactory> m_instance;
};

}

#endif
