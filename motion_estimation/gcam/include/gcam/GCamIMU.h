#ifndef GCAMIMU_H
#define GCAMIMU_H

#include <vector>

#include "camera_systems/CameraSystem.h"
#include "cauldron/PLineCorrespondence.h"

namespace px
{

class GCamIMU
{
public:
    GCamIMU(const CameraSystemConstPtr& cameraSystem);

    /**
     * Find translation given:
     * 1) 3 Plucker line correspondences, and
     * 2) known rotation.
     * @param R known rotation matrix.
     * @param t estimated translation.
     * @return Boolean result specifying whether or not solution is found.
     */
    bool estimateT(const std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> >& lcVec,
                   const Eigen::Matrix3d& R, Eigen::Vector3d& t) const;

    bool estimateH(const std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> >& lcVec,
                   const Eigen::Matrix3d& R, Eigen::Matrix4d& H) const;

    bool estimateH(const std::vector<PLineCorrespondence, Eigen::aligned_allocator<PLineCorrespondence> >& lcVec,
                   const Eigen::Matrix3d& R, Eigen::Matrix4d& H,
                   std::vector<size_t>& inliers) const;

    Eigen::Vector3d triangulate3DPoint(const PLineCorrespondence& lc,
                                       const Eigen::Matrix3d& R,
                                       const Eigen::Vector3d& t) const;

private:
    const double k_sphericalErrorThresh;

    CameraSystemConstPtr m_cameraSystem;
};

typedef boost::shared_ptr<GCamIMU> GCamIMUPtr;

}

#endif
