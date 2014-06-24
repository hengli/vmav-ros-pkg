#ifndef POSEGRAPHERROR_H
#define POSEGRAPHERROR_H

#include "cauldron/EigenUtils.h"
#include "ceres/rotation.h"
#include "sparse_graph/Transform.h"

namespace px
{

class PoseGraphError
{
public:
    PoseGraphError(Transform& meas_T_01)
     : m_meas_T_01(meas_T_01)
    {
        for (size_t i = 0; i < 6; ++i)
        {
            m_weight[i] = 1.0;
        }
    }

    PoseGraphError(Transform& meas_T_01, const std::vector<double>& weight)
     : m_meas_T_01(meas_T_01)
    {
        for (size_t i = 0; i < 6; ++i)
        {
            m_weight[i] = weight.at(i);
        }
    }

    template <typename T>
    bool operator()(const T* const q0, const T* const t0,
                    const T* const q1, const T* const t1,
                    T* residuals) const
    {
        Eigen::Matrix<T,4,4> meas_H_01 = m_meas_T_01.toMatrix().cast<T>();

        T q0_ceres[4] = {q0[3], q0[0], q0[1], q0[2]};
        Eigen::Matrix<T,3,3> R0;
        ceres::QuaternionToRotation(q0_ceres, ceres::ColumnMajorAdapter3x3(R0.data()));

        Eigen::Matrix<T,4,4> H0 = Eigen::Matrix<T,4,4>::Identity();
        H0.block(0,0,3,3) = R0;
        H0(0,3) = t0[0];
        H0(1,3) = t0[1];
        H0(2,3) = t0[2];

        T q1_ceres[4] = {q1[3], q1[0], q1[1], q1[2]};
        Eigen::Matrix<T,3,3> R1;
        ceres::QuaternionToRotation(q1_ceres, ceres::ColumnMajorAdapter3x3(R1.data()));

        Eigen::Matrix<T,4,4> H1 = Eigen::Matrix<T,4,4>::Identity();
        H1.block(0,0,3,3) = R1;
        H1(0,3) = t1[0];
        H1(1,3) = t1[1];
        H1(2,3) = t1[2];

        Eigen::Matrix<T,4,4> pred_H_01 = H1 * H0.inverse();

        Eigen::Matrix<T,4,4> err_H = meas_H_01 * pred_H_01.inverse();
        Eigen::Matrix<T,3,3> err_R = err_H.block(0,0,3,3);

        T roll, pitch, yaw;
        mat2RPY(err_R, roll, pitch, yaw);

        residuals[0] = err_H(0,3) * T(m_weight[0]);
        residuals[1] = err_H(1,3) * T(m_weight[1]);
        residuals[2] = err_H(2,3) * T(m_weight[2]);

        residuals[3] = roll * T(m_weight[3]);
        residuals[4] = pitch * T(m_weight[4]);
        residuals[5] = yaw * T(m_weight[5]);

        return true;
    }

private:
    Transform m_meas_T_01;
    double m_weight[6];
};

}

#endif
