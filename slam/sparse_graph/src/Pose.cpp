#include "sparse_graph/Pose.h"

namespace px
{

Pose::Pose()
 : Transform()
{
    m_covariance.setZero();
}

Pose::Pose(const Eigen::Matrix4d& H)
 : Transform(H)
{
   m_covariance.setZero();
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
