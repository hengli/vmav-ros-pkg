#include "camera_models/CostFunctionFactory.h"

#include "camera_models/CataCamera.h"
#include "camera_models/EquidistantCamera.h"
#include "camera_models/PinholeCamera.h"
#include "cauldron/cauldron.h"
#include "ceres/ceres.h"

namespace px
{

template<class CameraT>
class ReprojectionError1
{
public:
    ReprojectionError1(const Eigen::Vector2d& observed_p)
     : m_observed_p(observed_p) {}

    ReprojectionError1(const Eigen::Vector3d& observed_P,
                       const Eigen::Vector2d& observed_p)
     : m_observed_P(observed_P), m_observed_p(observed_p) {}

    // variables: camera intrinsics and camera pose
    template <typename T>
    bool operator()(const T* const intrinsic_params,
                    const T* const q,
                    const T* const t,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P = m_observed_P.cast<T>();

        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params, q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

    // variables: camera intrinsics, system-camera transform, system pose, and scene point
    template <typename T>
    bool operator()(const T* const intrinsic_params,
                    const T* const q_sys_cam,
                    const T* const t_sys_cam,
                    const T* const q_sys,
                    const T* const t_sys,
                    const T* const point,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P(point);

        Eigen::Quaternion<T> q = Eigen::Quaternion<T>(q_sys_cam) * Eigen::Quaternion<T>(q_sys);
        Eigen::Matrix<T,3,1> t = Eigen::Quaternion<T>(q_sys_cam) * Eigen::Matrix<T,3,1>(t_sys) + Eigen::Matrix<T,3,1>(t_sys_cam);

        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params, q.coeffs().data(), t.data(), P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

private:
    // observed 3D point
    Eigen::Vector3d m_observed_P;

    // observed 2D point
    Eigen::Vector2d m_observed_p;
};

class ReprojectionError2
{
public:
    ReprojectionError2(const Eigen::Quaterniond& q_sys_cam,
                       const Eigen::Vector3d& t_sys_cam,
                       const Eigen::Vector3d& observed_ray)
     : m_q_sys_cam(q_sys_cam)
     , m_t_sys_cam(t_sys_cam)
     , m_observed_ray(observed_ray)
    {

    }

    template <typename T>
    bool operator()(const T* const q_sys,
                    const T* const t_sys,
                    const T* const point,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P(point);

        Eigen::Quaternion<T> q_cam = m_q_sys_cam.cast<T>() * Eigen::Quaternion<T>(q_sys);
        Eigen::Matrix<T,3,1> t_cam = m_q_sys_cam.cast<T>() * Eigen::Matrix<T,3,1>(t_sys) + m_t_sys_cam.cast<T>();

        Eigen::Matrix<T,3,1> P_cam = q_cam * P + t_cam;
        Eigen::Matrix<T,3,1> ray_est = P_cam.normalized();

        residuals[0] = T(1) - ray_est.dot(m_observed_ray.cast<T>());

        return true;
    }

private:
    Eigen::Quaterniond m_q_sys_cam;
    Eigen::Vector3d m_t_sys_cam;

    // observed ray
    Eigen::Vector3d m_observed_ray;
};

class ReprojectionError3
{
public:
    ReprojectionError3(const Eigen::Matrix4d& H,
                       const Eigen::Vector3d& observed_ray)
     : m_H(H)
     , m_observed_ray(observed_ray)
    {

    }

    template <typename T>
    bool operator()(const T* const point,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P(point);

        Eigen::Matrix<T,3,1> P_cam = m_H.cast<T>().block(0,0,3,3) * P + m_H.cast<T>().block(0,3,3,1);
        Eigen::Matrix<T,3,1> ray_est = P_cam.normalized();

        residuals[0] = T(1) - ray_est.dot(m_observed_ray.cast<T>());

        return true;
    }

private:
    Eigen::Matrix4d m_H;

    // observed ray
    Eigen::Vector3d m_observed_ray;
};

template<class CameraT>
class StereoReprojectionError1
{
public:
    StereoReprojectionError1(const std::vector<double>& intrinsic_params_l,
                             const Eigen::Vector2d& observed_p_l,
                             const std::vector<double>& intrinsic_params_r,
                             const Eigen::Vector2d& observed_p_r,
                             const Eigen::Quaterniond& q_l_r,
                             const Eigen::Vector3d& t_l_r)
     : m_intrinsic_params_l(intrinsic_params_l)
     , m_observed_p_l(observed_p_l)
     , m_intrinsic_params_r(intrinsic_params_r)
     , m_observed_p_r(observed_p_r)
     , m_q_l_r(q_l_r)
     , m_t_l_r(t_l_r)
    {

    }

    StereoReprojectionError1(const Eigen::Vector2d& observed_p_l,
                             const Eigen::Vector2d& observed_p_r,
                             const Eigen::Vector3d& observed_P)
     : m_observed_p_l(observed_p_l)
     , m_observed_p_r(observed_p_r)
     , m_observed_P(observed_P)
    {

    }

    // variables: camera pose and 3D point
    template <typename T>
    bool operator()(const T* const q_l, const T* const t_l,
                    const T* const point, T* residuals) const
    {
        std::vector<T> intrinsic_params_l(m_intrinsic_params_l.begin(), m_intrinsic_params_l.end());
        std::vector<T> intrinsic_params_r(m_intrinsic_params_r.begin(), m_intrinsic_params_r.end());

        Eigen::Matrix<T,3,1> P(point);

        Eigen::Quaternion<T> q_r = m_q_l_r.cast<T>() * Eigen::Quaternion<T>(q_l);

        Eigen::Matrix<T,3,1> t_r = m_q_l_r.cast<T>() * Eigen::Matrix<T,3,1>(t_l) + m_t_l_r.cast<T>();

        Eigen::Matrix<T,2,1> predicted_p_l;
        CameraT::spaceToPlane(intrinsic_params_l.data(), q_l, t_l, P, predicted_p_l);

        Eigen::Matrix<T,2,1> predicted_p_r;
        CameraT::spaceToPlane(intrinsic_params_r.data(), q_r.coeffs().data(), t_r.data(), P, predicted_p_r);

        residuals[0] = predicted_p_l(0) - T(m_observed_p_l(0));
        residuals[1] = predicted_p_l(1) - T(m_observed_p_l(1));
        residuals[2] = predicted_p_r(0) - T(m_observed_p_r(0));
        residuals[3] = predicted_p_r(1) - T(m_observed_p_r(1));

        return true;
    }

    // variables: camera intrinsics, stereo transform, and camera pose
    template <typename T>
    bool operator()(const T* const intrinsic_params_l,
                    const T* const intrinsic_params_r,
                    const T* const q_l_r,
                    const T* const t_l_r,
                    const T* const q_l,
                    const T* const t_l,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P = m_observed_P.cast<T>();

        Eigen::Matrix<T,2,1> predicted_p_l;
        CameraT::spaceToPlane(intrinsic_params_l, q_l, t_l, P, predicted_p_l);

        Eigen::Quaternion<T> q_r = Eigen::Quaternion<T>(q_l_r) * Eigen::Quaternion<T>(q_l);
        Eigen::Matrix<T,3,1> t_r = Eigen::Quaternion<T>(q_l_r) * Eigen::Matrix<T,3,1>(t_l) + Eigen::Matrix<T,3,1>(t_l_r);

        Eigen::Matrix<T,2,1> predicted_p_r;
        CameraT::spaceToPlane(intrinsic_params_r, q_r.coeffs().data(), t_r.data(), P, predicted_p_r);

        residuals[0] = predicted_p_l(0) - T(m_observed_p_l(0));
        residuals[1] = predicted_p_l(1) - T(m_observed_p_l(1));
        residuals[2] = predicted_p_r(0) - T(m_observed_p_r(0));
        residuals[3] = predicted_p_r(1) - T(m_observed_p_r(1));

        return true;
    }

    // variables: camera intrinsics, system-camera transform, and system pose
    template <typename T>
    bool operator()(const T* const intrinsic_params_l,
                    const T* const intrinsic_params_r,
                    const T* const q_s_l,
                    const T* const t_s_l,
                    const T* const q_s_r,
                    const T* const t_s_r,
                    const T* const q_l,
                    const T* const t_l,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P = m_observed_P.cast<T>();

        Eigen::Quaternion<T> q_l_s = Eigen::Quaternion<T>(q_s_l).conjugate();
        Eigen::Matrix<T,3,1> t_l_s = q_l_s * (- Eigen::Matrix<T,3,1>(t_s_l));

        Eigen::Quaternion<T> q_l_r = Eigen::Quaternion<T>(q_s_r) * q_l_s;
        Eigen::Matrix<T,3,1> t_l_r = Eigen::Quaternion<T>(q_s_r) * t_l_s + Eigen::Matrix<T,3,1>(t_s_r);

        Eigen::Quaternion<T> q_r = q_l_r * Eigen::Quaternion<T>(q_l);
        Eigen::Matrix<T,3,1> t_r = q_l_r * Eigen::Matrix<T,3,1>(t_l) + t_l_r;

        Eigen::Matrix<T,2,1> predicted_p_l;
        CameraT::spaceToPlane(intrinsic_params_l, q_l, t_l, P, predicted_p_l);

        Eigen::Matrix<T,2,1> predicted_p_r;
        CameraT::spaceToPlane(intrinsic_params_r, q_r.coeffs().data(), t_r.data(), P, predicted_p_r);

        residuals[0] = predicted_p_l(0) - T(m_observed_p_l(0));
        residuals[1] = predicted_p_l(1) - T(m_observed_p_l(1));
        residuals[2] = predicted_p_r(0) - T(m_observed_p_r(0));
        residuals[3] = predicted_p_r(1) - T(m_observed_p_r(1));

        return true;
    }

private:
    // camera intrinsics
    std::vector<double> m_intrinsic_params_l;
    std::vector<double> m_intrinsic_params_r;

    // observed stereo transform
    Eigen::Quaterniond m_q_l_r;
    Eigen::Vector3d m_t_l_r;

    // observed 2D point
    Eigen::Vector2d m_observed_p_l;
    Eigen::Vector2d m_observed_p_r;

    // observed 3D point
    Eigen::Vector3d m_observed_P;
};

//class StereoReprojectionError2
//{
//public:
//    StereoReprojectionError2(const Eigen::Vector3d& observed_ray_l,
//                             const Eigen::Vector3d& observed_ray_r,
//                             const Eigen::Quaterniond& q_s_l,
//                             const Eigen::Vector3d& t_s_l,
//                             const Eigen::Quaterniond& q_s_r,
//                             const Eigen::Vector3d& t_s_r)
//     : m_q_s_l(q_s_l)
//     , m_t_s_l(t_s_l)
//     , m_q_s_r(q_s_r)
//     , m_t_s_r(t_s_r)
//     , m_obs_ray_l(observed_ray_l)
//     , m_obs_ray_r(observed_ray_r)
//    {
//
//    }
//
//    // variables: camera poses and 3D points
//    template <typename T>
//    bool operator()(const T* const q_s_coeffs, const T* const t_s_coeffs,
//                    const T* const point, T* residuals) const
//    {
//        Eigen::Quaternion<T> q_s(q_s_coeffs);
//        Eigen::Matrix<T,3,1> t_s(t_s_coeffs);
//
//        Eigen::Quaternion<T> q_l = m_q_s_l.cast<T>() * q_s;
//        Eigen::Matrix<T,3,1> t_l = m_q_s_l.cast<T>() * t_s + m_t_s_l.cast<T>();
//
//        Eigen::Quaternion<T> q_r = m_q_s_r.cast<T>() * q_s;
//        Eigen::Matrix<T,3,1> t_r = m_q_s_r.cast<T>() * t_s + m_t_s_r.cast<T>();
//
//        Eigen::Matrix<T,3,1> P(point);
//
//        Eigen::Matrix<T,3,1> P_l = q_l * P + t_l;
//        Eigen::Matrix<T,3,1> est_ray_l = P_l.normalized();
//
//        Eigen::Matrix<T,3,1> P_r = q_r * P + t_r;
//        Eigen::Matrix<T,3,1> est_ray_r = P_r.normalized();
//
//        residuals[0] = T(1) - est_ray_l.dot(m_obs_ray_l.cast<T>());
//        residuals[1] = T(1) - est_ray_r.dot(m_obs_ray_r.cast<T>());
//
//        return true;
//    }
//
//private:
//    // observed stereo extrinsics
//    Eigen::Quaterniond m_q_s_l;
//    Eigen::Vector3d m_t_s_l;
//    Eigen::Quaterniond m_q_s_r;
//    Eigen::Vector3d m_t_s_r;
//
//    // observed ray
//    Eigen::Vector3d m_obs_ray_l;
//    Eigen::Vector3d m_obs_ray_r;
//};

class StereoReprojectionError2: public ceres::SizedCostFunction<2, 4, 3, 3>
{
public:
    StereoReprojectionError2(const Eigen::Vector3d& observed_ray_l,
                             const Eigen::Vector3d& observed_ray_r,
                             const Eigen::Quaterniond& q_s_l,
                             const Eigen::Vector3d& t_s_l,
                             const Eigen::Quaterniond& q_s_r,
                             const Eigen::Vector3d& t_s_r)
     : m_q_s_l(q_s_l)
     , m_t_s_l(t_s_l)
     , m_q_s_r(q_s_r)
     , m_t_s_r(t_s_r)
     , m_obs_ray_l(observed_ray_l)
     , m_obs_ray_r(observed_ray_r)
    {

    }

    // variables: camera pose and 3D points
    virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const
    {
        Eigen::Quaterniond q_s(parameters[0]);
        Eigen::Vector3d t_s(parameters[1]);

        Eigen::Quaterniond q_l = m_q_s_l * q_s;
        Eigen::Vector3d t_l = m_q_s_l * t_s + m_t_s_l;

        Eigen::Quaterniond q_r = m_q_s_r * q_s;
        Eigen::Vector3d t_r = m_q_s_r * t_s + m_t_s_r;

        Eigen::Vector3d P(parameters[2]);

        Eigen::Vector3d P_l = q_l * P + t_l;
        Eigen::Vector3d est_ray_l = P_l.normalized();

        Eigen::Vector3d P_r = q_r * P + t_r;
        Eigen::Vector3d est_ray_r = P_r.normalized();

        residuals[0] = 1.0 - est_ray_l.dot(m_obs_ray_l);
        residuals[1] = 1.0 - est_ray_r.dot(m_obs_ray_r);

        return true;
    }

private:
    // observed stereo extrinsics
    Eigen::Quaterniond m_q_s_l;
    Eigen::Vector3d m_t_s_l;
    Eigen::Quaterniond m_q_s_r;
    Eigen::Vector3d m_t_s_r;

    // observed ray
    Eigen::Vector3d m_obs_ray_l;
    Eigen::Vector3d m_obs_ray_r;
};

boost::shared_ptr<CostFunctionFactory> CostFunctionFactory::m_instance;

CostFunctionFactory::CostFunctionFactory()
{

}

boost::shared_ptr<CostFunctionFactory>
CostFunctionFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CostFunctionFactory);
    }

    return m_instance;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector2d& observed_p) const
{
    ceres::CostFunction* costFunction = 0;

    switch (camera->modelType())
    {
    case Camera::KANNALA_BRANDT:
        costFunction =
            new ceres::AutoDiffCostFunction<ReprojectionError1<EquidistantCamera>, 2, 8, 4, 3, 4, 3, 3>(
                new ReprojectionError1<EquidistantCamera>(observed_p));
        break;
    case Camera::PINHOLE:
        costFunction =
            new ceres::AutoDiffCostFunction<ReprojectionError1<PinholeCamera>, 2, 8, 4, 3, 4, 3, 3>(
                new ReprojectionError1<PinholeCamera>(observed_p));
        break;
    case Camera::MEI:
        costFunction =
            new ceres::AutoDiffCostFunction<ReprojectionError1<CataCamera>, 2, 9, 4, 3, 4, 3, 3>(
                new ReprojectionError1<CataCamera>(observed_p));
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector3d& observed_P,
                                          const Eigen::Vector2d& observed_p) const
{
    ceres::CostFunction* costFunction = 0;

    switch (camera->modelType())
    {
    case Camera::KANNALA_BRANDT:
        costFunction =
            new ceres::AutoDiffCostFunction<ReprojectionError1<EquidistantCamera>, 2, 8, 4, 3>(
                new ReprojectionError1<EquidistantCamera>(observed_P, observed_p));
        break;
    case Camera::PINHOLE:
        costFunction =
            new ceres::AutoDiffCostFunction<ReprojectionError1<PinholeCamera>, 2, 8, 4, 3>(
                new ReprojectionError1<PinholeCamera>(observed_P, observed_p));
        break;
    case Camera::MEI:
        costFunction =
            new ceres::AutoDiffCostFunction<ReprojectionError1<CataCamera>, 2, 9, 4, 3>(
                new ReprojectionError1<CataCamera>(observed_P, observed_p));
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const Eigen::Quaterniond& q_sys_cam,
                                          const Eigen::Vector3d& t_sys_cam,
                                          const Eigen::Vector3d& observed_ray) const
{
    ceres::CostFunction* costFunction = 0;

    costFunction =
        new ceres::AutoDiffCostFunction<ReprojectionError2, 1, 4, 3, 3>(
            new ReprojectionError2(q_sys_cam, t_sys_cam, observed_ray));

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const Eigen::Matrix4d& H,
                                          const Eigen::Vector3d& observed_ray) const
{
    ceres::CostFunction* costFunction = 0;

    costFunction =
        new ceres::AutoDiffCostFunction<ReprojectionError3, 1, 3>(
            new ReprojectionError3(H, observed_ray));

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const Eigen::Vector3d& observed_ray_l,
                                          const Eigen::Vector3d& observed_ray_r,
                                          const Eigen::Quaterniond& q_s_l,
                                          const Eigen::Vector3d& t_s_l,
                                          const Eigen::Quaterniond& q_s_r,
                                          const Eigen::Vector3d& t_s_r) const
{
    ceres::CostFunction* costFunction = 0;

    costFunction =
        new ceres::NumericDiffCostFunction<StereoReprojectionError2, ceres::FORWARD, 2, 4, 3, 3>(
//        new ceres::AutoDiffCostFunction<StereoReprojectionError2, 2, 4, 3, 3>(
            new StereoReprojectionError2(observed_ray_l, observed_ray_r, q_s_l, t_s_l, q_s_r, t_s_r));

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& cameraL,
                                          const Eigen::Vector2d& observed_p_l,
                                          const CameraConstPtr& cameraR,
                                          const Eigen::Vector2d& observed_p_r,
                                          const Eigen::Vector3d& observed_P,
                                          int flags) const
{
    ceres::CostFunction* costFunction = 0;

    if (cameraL->modelType() != cameraR->modelType())
    {
        return costFunction;
    }

    switch (flags)
    {
    case CAMERA_INTRINSICS | STEREO_TRANSFORM | CAMERA_POSE:
    {
        switch (cameraL->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<StereoReprojectionError1<EquidistantCamera>, 4, 8, 8, 4, 3, 4, 3>(
                    new StereoReprojectionError1<EquidistantCamera>(observed_p_l, observed_p_r, observed_P));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<StereoReprojectionError1<PinholeCamera>, 4, 8, 8, 4, 3, 4, 3>(
                    new StereoReprojectionError1<PinholeCamera>(observed_p_l, observed_p_r, observed_P));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<StereoReprojectionError1<CataCamera>, 4, 9, 9, 4, 3, 4, 3>(
                    new StereoReprojectionError1<CataCamera>(observed_p_l, observed_p_r, observed_P));
            break;
        }
        break;
    }
    case CAMERA_INTRINSICS | SYSTEM_CAMERA_TRANSFORM | SYSTEM_POSE:
    {
        switch (cameraL->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<StereoReprojectionError1<EquidistantCamera>, 4, 8, 8, 4, 3, 4, 3, 4, 3>(
                    new StereoReprojectionError1<EquidistantCamera>(observed_p_l, observed_p_r, observed_P));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<StereoReprojectionError1<PinholeCamera>, 4, 8, 8, 4, 3, 4, 3, 4, 3>(
                    new StereoReprojectionError1<PinholeCamera>(observed_p_l, observed_p_r, observed_P));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<StereoReprojectionError1<CataCamera>, 4, 9, 9, 4, 3, 4, 3, 4, 3>(
                    new StereoReprojectionError1<CataCamera>(observed_p_l, observed_p_r, observed_P));
            break;
        }
        break;
    }
    }

    return costFunction;
}

}

