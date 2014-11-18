#include "hand_eye_calibration/ExtendedHandEyeCalibration.h"

#include <iostream>

#include "cauldron/EigenQuaternionParameterization.h"
#include "cauldron/EigenUtils.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "hand_eye_calibration/DualQuaternion.h"

namespace px
{

class PoseError
{
public:
    PoseError(const Eigen::Matrix4d& H1, const Eigen::Matrix4d& H2)
        : m_H1(H1), m_H2(H2)
    {}

    template<typename T>
    bool operator() (const T* const q_coeffs, const T* const t_coeffs, const T* const s, T* residuals) const
    {
        T q_ceres[4] = {q_coeffs[3], q_coeffs[0], q_coeffs[1], q_coeffs[2]};
        Eigen::Matrix<T,3,3> R;
        ceres::QuaternionToRotation(q_ceres, ceres::ColumnMajorAdapter3x3(R.data()));

        Eigen::Matrix<T,4,4> H = Eigen::Matrix<T,4,4>::Identity();
        H.block(0,0,3,3) = *s * R;
        H(0,3) = t_coeffs[0];
        H(1,3) = t_coeffs[1];
        H(2,3) = t_coeffs[2];

        Eigen::Matrix<T,4,4> pred_H2 = H * m_H1.cast<T>() * H.inverse();

        Eigen::Matrix<T,4,4> err_H = m_H2.cast<T>().inverse() * pred_H2;
        Eigen::Matrix<T,3,3> err_R = err_H.block(0,0,3,3);

        T roll, pitch, yaw;
        mat2RPY(err_R, roll, pitch, yaw);

        residuals[0] = err_H(0,3);
        residuals[1] = err_H(1,3);
        residuals[2] = err_H(2,3);

        residuals[3] = roll;
        residuals[4] = pitch;
        residuals[5] = yaw;

        return true;
    }

private:
    Eigen::Matrix4d m_H1, m_H2;
};

ExtendedHandEyeCalibration::ExtendedHandEyeCalibration()
{

}

void
ExtendedHandEyeCalibration::solve(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H1,
                                  const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H2,
                                  Eigen::Matrix4d& H_12,
                                  double& s) const
{
    int motionCount = H1.size();

    Eigen::MatrixXd T(motionCount * 6, 8);
    T.setZero();
    for (int i = 0; i < motionCount; ++i)
    {
        Eigen::AngleAxisd aa1(H1.at(i).block<3,3>(0,0));
        Eigen::Vector3d rvec1 = aa1.angle() * aa1.axis();
        Eigen::Vector3d tvec1 = H1.at(i).block<3,1>(0,3);

        Eigen::AngleAxisd aa2(H2.at(i).block<3,3>(0,0));
        Eigen::Vector3d rvec2 = aa2.angle() * aa2.axis();
        Eigen::Vector3d tvec2 = H2.at(i).block<3,1>(0,3);

        double theta1, d1;
        Eigen::Vector3d l1, m1;
        AngleAxisAndTranslationToScrew(rvec1, tvec1, theta1, d1, l1, m1);

        double theta2, d2;
        Eigen::Vector3d l2, m2;
        AngleAxisAndTranslationToScrew(rvec2, tvec2, theta2, d2, l2, m2);

        Eigen::Vector3d a = l1;
        Eigen::Vector3d a_prime = m1;
        Eigen::Vector3d b = l2;
        Eigen::Vector3d b_prime = m2;

        T.block<3,1>(i * 6, 0) = a - b;
        T.block<3,3>(i * 6, 1) = skew(Eigen::Vector3d(a + b));
        T.block<3,1>(i * 6 + 3, 0) = a_prime - b_prime;
        T.block<3,3>(i * 6 + 3, 1) = skew(Eigen::Vector3d(a_prime + b_prime));
        T.block<3,1>(i * 6 + 3, 4) = a - b;
        T.block<3,3>(i * 6 + 3, 5) = skew(Eigen::Vector3d(a + b));
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(T, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // v7 and v8 span the null space of T, v6 may also be one
    // if rank = 5. 
    Eigen::Matrix<double, 8, 1> v6 = svd.matrixV().block<8,1>(0, 5);
    Eigen::Matrix<double, 8, 1> v7 = svd.matrixV().block<8,1>(0,6);
    Eigen::Matrix<double, 8, 1> v8 = svd.matrixV().block<8,1>(0,7);

    Eigen::Vector4d u1 = v7.block<4,1>(0,0);
    Eigen::Vector4d v1 = v7.block<4,1>(4,0);
    Eigen::Vector4d u2 = v8.block<4,1>(0,0);
    Eigen::Vector4d v2 = v8.block<4,1>(4,0);

    double lambda1 = 0;
    double lambda2 = 0.0;

    if (u1.dot(v1) == 0.0)
    {
        std::swap(u1, u2); 
        std::swap(v1, v2); 
    }
    if (u1.dot(v1) != 0.0)
    {
        double s[2];
        solveQuadraticEquation(u1.dot(v1), u1.dot(v2) + u2.dot(v1), u2.dot(v2), s[0], s[1]);

        // find better solution for s
        double t[2];
        for (int i = 0; i < 2; ++i)
        {
            t[i] = s[i] * s[i] * u1.dot(u1) + 2 * s[i] * u1.dot(u2) + u2.dot(u2);
        }

        int idx;
        if (t[0] > t[1])
        {
            idx = 0;
        }
        else
        {
            idx = 1;
        }

        lambda2 = sqrt(1.0 / t[idx]);
        lambda1 = s[idx] * lambda2;
    }
    else 
    {
        if (u1.norm() == 0.0 && u2.norm() > 0.0) 
        {
            lambda1 = 0.0;
            lambda2 = 1.0 / u2.norm();
        }
        else if (u2.norm() == 0.0 && u1.norm() > 0.0)
        {
            lambda1 = 1.0 / u1.norm();
            lambda2 = 0.0;
        }
    }

    // rotation
    Eigen::Vector4d q_coeffs = lambda1 * u1 + lambda2 * u2;
    Eigen::Vector4d q_prime_coeffs = lambda1 * v1 + lambda2 * v2;

    Eigen::Quaterniond q(q_coeffs(0), q_coeffs(1), q_coeffs(2), q_coeffs(3));
    Eigen::Quaterniond d(q_prime_coeffs(0), q_prime_coeffs(1), q_prime_coeffs(2), q_prime_coeffs(3));

    DualQuaterniond dq(q, d);

    H_12 = dq.toMatrix().inverse();

    s = 1.0;
    refine(H1, H2, H_12, s);
}

bool
ExtendedHandEyeCalibration::solveQuadraticEquation(double a, double b, double c, double& x1, double& x2) const
{
    double delta2 = b * b - 4.0 * a * c;

    if (delta2 < 0.0)
    {
        return false;
    }

    double delta = sqrt(delta2);

    x1 = (-b + delta) / (2.0 * a);
    x2 = (-b - delta) / (2.0 * a);

    return true;
}

void
ExtendedHandEyeCalibration::refine(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H1,
                                   const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H2,
                                   Eigen::Matrix4d& H_12,
                                   double& s) const
{
    Eigen::Quaterniond q(H_12.block<3,3>(0,0));
    Eigen::Vector3d t = H_12.block<3,1>(0,3);
 
    ceres::Problem problem; 
    for (size_t i = 0; i < H1.size(); i++)
    {
        ceres::CostFunction* costFunction = 
            new ceres::AutoDiffCostFunction<PoseError, 6, 4, 3, 1>(
                new PoseError(H1.at(i), H2.at(i)));

        problem.AddResidualBlock(costFunction, 0, q.coeffs().data(), t.data(), &s); 
    }

    ceres::LocalParameterization* quaternionParameterization = 
        new EigenQuaternionParameterization;

    problem.SetParameterization(q.coeffs().data(), quaternionParameterization);

    ceres::Solver::Options options;
    options.gradient_tolerance = 1e-12;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 500;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    H_12.block<3,3>(0,0) = q.toRotationMatrix();
    H_12.block<3,1>(0,3) = t;
}

}
