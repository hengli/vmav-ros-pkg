#include "pose_imu_calibration/PoseIMUCalibration.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "cauldron/EigenUtils.h"

namespace px
{

class AttitudeError
{
public:
    AttitudeError(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
     : m_q1(q1), m_q2(q2)
    {}

    template<typename T>
    bool operator() (const T* const q_coeffs, T* residuals) const
    {
        Eigen::Quaternion<T> q(q_coeffs);

        Eigen::Quaternion<T> q_err = q.conjugate() * m_q2.cast<T>() * q * m_q1.cast<T>().conjugate();

        T q_err_coeffs[4] = {q_err.w(), q_err.x(), q_err.y(), q_err.z()};
        Eigen::Matrix<T,3,3,Eigen::RowMajor> R_err;
        ceres::QuaternionToRotation(q_err_coeffs, R_err.data());

        T r_err, p_err, y_err;
        mat2RPY(Eigen::Matrix<T,3,3>(R_err), r_err, p_err, y_err);

        residuals[0] = r_err;
        residuals[1] = p_err;
        residuals[2] = y_err;

        return true;
    }

private:
    Eigen::Quaterniond m_q1, m_q2;
};

bool
PoseIMUCalibration::calibrate(const std::vector<PoseConstPtr>& poseData,
                              const std::vector<sensor_msgs::ImuConstPtr>& imuData,
                              Eigen::Quaterniond& q_pose_imu)
{
    m_poseData = poseData;
    m_imuData = imuData;

    if (m_poseData.size() != m_imuData.size())
    {
        return false;
    }

    if (m_imuData.size() < 2)
    {
        return false;
    }

    estimate(q_pose_imu);
    refine(q_pose_imu);

    return true;
}

void
PoseIMUCalibration::errorStats(const Eigen::Quaterniond& q_pose_imu,
                               double& avgError, double& maxError) const
{
    double sum = 0.0;
    maxError = 0.0;
    for (size_t i = 0; i < m_poseData.size() - 1; ++i)
    {
        Eigen::Quaterniond q_pose0 = m_poseData.at(i)->rotation();
        Eigen::Quaterniond q_pose1 = m_poseData.at(i+1)->rotation();

        Eigen::Quaterniond q_imu0(m_imuData.at(i)->orientation.w,
                                  m_imuData.at(i)->orientation.x,
                                  m_imuData.at(i)->orientation.y,
                                  m_imuData.at(i)->orientation.z);
        Eigen::Quaterniond q_imu1(m_imuData.at(i+1)->orientation.w,
                                  m_imuData.at(i+1)->orientation.x,
                                  m_imuData.at(i+1)->orientation.y,
                                  m_imuData.at(i+1)->orientation.z);

        Eigen::Quaterniond q_pose = q_pose1 * q_pose0.conjugate();
        Eigen::Quaterniond q_imu = q_imu1.conjugate() * q_imu0;

        Eigen::Quaterniond q_err = q_pose_imu.conjugate() * q_imu * q_pose_imu * q_pose.conjugate();

        double r, p, y;
        px::mat2RPY(q_err.toRotationMatrix(), r, p, y);

        double error = sqrt(r*r + p*p + y*y);

        sum += error;
        if (error > maxError)
        {
            maxError = error;
        }
    }

    avgError = sum / (m_poseData.size() - 1);
}

void
PoseIMUCalibration::estimate(Eigen::Quaterniond& q_pose_imu) const
{
    size_t nMotions = m_poseData.size() - 1;

    Eigen::MatrixXd T(nMotions * 3, 4);
    for (size_t i = 0; i < nMotions; ++i)
    {
        Eigen::Quaterniond q_pose0 = m_poseData.at(i)->rotation();
        Eigen::Quaterniond q_pose1 = m_poseData.at(i + 1)->rotation();

        Eigen::Quaterniond q_imu0(m_imuData.at(i)->orientation.w,
                                  m_imuData.at(i)->orientation.x,
                                  m_imuData.at(i)->orientation.y,
                                  m_imuData.at(i)->orientation.z);
        Eigen::Quaterniond q_imu1(m_imuData.at(i+1)->orientation.w,
                                  m_imuData.at(i+1)->orientation.x,
                                  m_imuData.at(i+1)->orientation.y,
                                  m_imuData.at(i+1)->orientation.z);

        Eigen::Vector3d a = Eigen::AngleAxisd(q_imu1.conjugate() * q_imu0).axis();
        Eigen::Vector3d b = Eigen::AngleAxisd(q_pose1 * q_pose0.conjugate()).axis();

        T.block<3,1>(i * 3, 0) = a - b;
        T.block<3,3>(i * 3, 1) = skew(Eigen::Vector3d(a + b));
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(T, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Vector4d v = svd.matrixV().block<4,1>(0,3);

    q_pose_imu.coeffs() << v(1), v(2), v(3), v(0);
}

void
PoseIMUCalibration::refine(Eigen::Quaterniond& q_pose_imu) const
{
    ceres::Problem problem;

    size_t nMotions = m_poseData.size() - 1;
    for (size_t i = 0; i < nMotions; ++i)
    {
        Eigen::Quaterniond q_pose0 = m_poseData.at(i)->rotation();
        Eigen::Quaterniond q_pose1 = m_poseData.at(i + 1)->rotation();

        Eigen::Quaterniond q_imu0(m_imuData.at(i)->orientation.w,
                                  m_imuData.at(i)->orientation.x,
                                  m_imuData.at(i)->orientation.y,
                                  m_imuData.at(i)->orientation.z);
        Eigen::Quaterniond q_imu1(m_imuData.at(i+1)->orientation.w,
                                  m_imuData.at(i+1)->orientation.x,
                                  m_imuData.at(i+1)->orientation.y,
                                  m_imuData.at(i+1)->orientation.z);

        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<AttitudeError, 3, 4>(
                new AttitudeError(q_pose1 * q_pose0.conjugate(),
                                  q_imu1.conjugate() * q_imu0));

        problem.AddResidualBlock(costFunction, 0, q_pose_imu.coeffs().data());
    }

    ceres::LocalParameterization* quaternionParameterization =
        new ceres::QuaternionParameterization;

    problem.SetParameterization(q_pose_imu.coeffs().data(), quaternionParameterization);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 2500;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

//    std::cout << summary.BriefReport() << std::endl;
}

}
