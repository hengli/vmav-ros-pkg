#ifndef GCAMLOCALBA_H
#define GCAMLOCALBA_H

#include <Eigen/Dense>
#include <list>

#include "camera_systems/CameraSystem.h"
#include "sparse_graph/SparseGraph.h"

namespace px
{

class GCamLocalBA
{
public:
    GCamLocalBA(const CameraSystemConstPtr& cameraSystem,
                int N = 8);

    bool addFrameSet(FrameSetPtr& frameSet, bool replaceCurrentFrameSet);

private:
    void optimize(void);

    const int k_N;

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > m_H_cam_sys;
    std::vector<Transform, Eigen::aligned_allocator<Transform> > m_T_sys_cam;

    std::list<FrameSetPtr> m_window;
};

}

#endif
