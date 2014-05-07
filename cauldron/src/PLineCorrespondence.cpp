#include "cauldron/PLineCorrespondence.h"

namespace px
{

PLineCorrespondence::PLineCorrespondence(int cameraId1,
                                         const Eigen::Vector3d& ray1,
                                         const PLine& l1,
                                         int cameraId2,
                                         const Eigen::Vector3d& ray2,
                                         const PLine& l2)
{
    m_cameraId[0] = cameraId1;
    m_ray[0] = ray1;
    m_l[0] = l1;
    m_cameraId[1] = cameraId2;
    m_ray[1] = ray2;
    m_l[1] = l2;
}

PLineCorrespondence::PLineCorrespondence(int cameraId1,
                                         const Eigen::Vector3d& ray1,
                                         const Eigen::Matrix4d& H_cam1_sys,
                                         int cameraId2,
                                         const Eigen::Vector3d& ray2,
                                         const Eigen::Matrix4d& H_cam2_sys)
{
    m_cameraId[0] = cameraId1;
    m_ray[0] = ray1;
    m_cameraId[1] = cameraId2;
    m_ray[1] = ray2;

    m_l[0] = PLine(ray1, H_cam1_sys);
    m_l[1] = PLine(ray2, H_cam2_sys);
}

int&
PLineCorrespondence::cameraId1(void)
{
    return m_cameraId[0];
}

int
PLineCorrespondence::cameraId1(void) const
{
    return m_cameraId[0];
}

Eigen::Vector3d&
PLineCorrespondence::ray1(void)
{
    return m_ray[0];
}

const Eigen::Vector3d&
PLineCorrespondence::ray1(void) const
{
    return m_ray[0];
}

PLine&
PLineCorrespondence::l1(void)
{
    return m_l[0];
}

const PLine&
PLineCorrespondence::l1(void) const
{
    return m_l[0];
}

int&
PLineCorrespondence::cameraId2(void)
{
    return m_cameraId[1];
}

int
PLineCorrespondence::cameraId2(void) const
{
    return m_cameraId[1];
}

Eigen::Vector3d&
PLineCorrespondence::ray2(void)
{
    return m_ray[1];
}

const Eigen::Vector3d&
PLineCorrespondence::ray2(void) const
{
    return m_ray[1];
}

PLine&
PLineCorrespondence::l2(void)
{
    return m_l[1];
}

const PLine&
PLineCorrespondence::l2(void) const
{
    return m_l[1];
}

}
