#ifndef PLINE_H
#define PLINE_H

#include <Eigen/Dense>

namespace px
{

class PLine
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PLine();

    PLine(const Eigen::Vector3d& ray,
          const Eigen::Matrix4d& H_cam_sys);
    PLine(const Eigen::Vector3d& directionVector,
          const Eigen::Vector3d& momentVector);

    Eigen::Vector3d& directionVector(void);
    const Eigen::Vector3d& directionVector(void) const;

    Eigen::Vector3d& momentVector(void);
    const Eigen::Vector3d& momentVector(void) const;

private:
    Eigen::Vector3d m_directionVector;
    Eigen::Vector3d m_momentVector;
};

}

#endif
