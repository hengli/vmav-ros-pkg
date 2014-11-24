#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <stdint.h>

#include "cauldron/Enums.h"

namespace px
{

class Transform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Transform(TransformType type = TRANSFORM_SE3);
    Transform(const Eigen::Matrix4d& H, TransformType type = TRANSFORM_SE3);

    void setIdentity(void);

    Eigen::Quaterniond& rotation(void);
    const Eigen::Quaterniond& rotation(void) const;
    double* rotationData(void);
    const double* const rotationData(void) const;

    Eigen::Vector3d& translation(void);
    const Eigen::Vector3d& translation(void) const;
    double* translationData(void);
    const double* const translationData(void) const;

    double& scale(void);
    double scale(void) const;
    double* scaleData(void);
    const double* const scaleData(void) const;

    Eigen::Matrix4d toMatrix(void) const;

private:
    TransformType m_type;
    Eigen::Quaterniond m_q;
    Eigen::Vector3d m_t;
    double m_s;
};

}

#endif
