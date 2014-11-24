#include "sparse_graph/Pose.h"

namespace px
{

Pose::Pose(TransformType type)
 : Transform(type)
 , m_covariance(Eigen::Matrix<double,7,7>::Zero())
{

}

Pose::Pose(const Eigen::Matrix4d& H, TransformType type)
 : Transform(H, type)
 , m_covariance(Eigen::Matrix<double,7,7>::Zero())
{

}

ros::Time&
Pose::timeStamp(void)
{
    return m_timeStamp;
}

const ros::Time&
Pose::timeStamp(void) const
{
    return m_timeStamp;
}

Eigen::Matrix<double,7,7>&
Pose::covariance(void)
{
    return m_covariance;
}

const Eigen::Matrix<double,7,7>&
Pose::covariance(void) const
{
    return m_covariance;
}

double*
Pose::covarianceData(void)
{
    return m_covariance.data();
}

const double* const
Pose::covarianceData(void) const
{
    return m_covariance.data();
}

}
