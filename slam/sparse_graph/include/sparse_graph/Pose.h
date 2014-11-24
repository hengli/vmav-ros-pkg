#ifndef POSE_H
#define POSE_H

#include <ros/time.h>

#include "sparse_graph/Transform.h"

namespace px
{

class Pose: public Transform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose(TransformType type = TRANSFORM_SE3);
    Pose(const Eigen::Matrix4d& H, TransformType type = TRANSFORM_SE3);

    ros::Time& timeStamp(void);
    const ros::Time& timeStamp(void) const;

    Eigen::Matrix<double,7,7>& covariance(void);
    const Eigen::Matrix<double,7,7>& covariance(void) const;
    double* covarianceData(void);
    const double* const covarianceData(void) const;

private:
    ros::Time m_timeStamp;
    Eigen::Matrix<double,7,7> m_covariance;
};

typedef boost::shared_ptr<Pose> PosePtr;
typedef boost::weak_ptr<Pose> PoseWPtr;
typedef boost::shared_ptr<const Pose> PoseConstPtr;
typedef boost::weak_ptr<const Pose> PoseConstWPtr;

}

#endif
