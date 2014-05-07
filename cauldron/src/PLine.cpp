#include "cauldron/PLine.h"

namespace px
{

PLine::PLine()
{

}

PLine::PLine(const Eigen::Vector3d& ray,
             const Eigen::Matrix4d& H_cam_sys)
{
    Eigen::Vector3d r = H_cam_sys.block<3,3>(0,0) * ray;

    m_directionVector = r;
    m_momentVector = H_cam_sys.block<3,1>(0,3).cross(r);
}

PLine::PLine(const Eigen::Vector3d& directionVector,
             const Eigen::Vector3d& momentVector)
 : m_directionVector(directionVector)
 , m_momentVector(momentVector)
{

}

Eigen::Vector3d&
PLine::directionVector(void)
{
    return m_directionVector;
}

const Eigen::Vector3d&
PLine::directionVector(void) const
{
    return m_directionVector;
}

Eigen::Vector3d&
PLine::momentVector(void)
{
    return m_momentVector;
}

const Eigen::Vector3d&
PLine::momentVector(void) const
{
    return m_momentVector;
}

}
