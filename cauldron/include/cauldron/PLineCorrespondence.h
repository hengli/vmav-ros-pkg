#ifndef PLINECORRESPONDENCE_H
#define PLINECORRESPONDENCE_H

#include "cauldron/PLine.h"

namespace px
{

class PLineCorrespondence
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PLineCorrespondence(int cameraId1,
                        const Eigen::Vector3d& ray1,
                        const PLine& l1,
                        int cameraId2,
                        const Eigen::Vector3d& ray2,
                        const PLine& l2);
    PLineCorrespondence(int cameraId1,
                        const Eigen::Vector3d& ray1,
                        const Eigen::Matrix4d& H_cam1_sys,
                        int cameraId2,
                        const Eigen::Vector3d& ray2,
                        const Eigen::Matrix4d& H_cam2_sys);

    int& cameraId1(void);
    int cameraId1(void) const;

    Eigen::Vector3d& ray1(void);
    const Eigen::Vector3d& ray1(void) const;

    PLine& l1(void);
    const PLine& l1(void) const;

    int& cameraId2(void);
    int cameraId2(void) const;

    Eigen::Vector3d& ray2(void);
    const Eigen::Vector3d& ray2(void) const;

    PLine& l2(void);
    const PLine& l2(void) const;

private:
    int m_cameraId[2];
    Eigen::Vector3d m_ray[2];
    PLine m_l[2];
};

}

#endif
