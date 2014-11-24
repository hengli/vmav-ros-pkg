#ifndef LOCALMONOBA_H
#define LOCALMONOBA_H

#include <Eigen/Dense>
#include <list>

#include "camera_systems/CameraSystem.h"
#include "sparse_graph/SparseGraph.h"

namespace px
{

class LocalMonoBA
{
public:
    LocalMonoBA(const CameraSystemConstPtr& cameraSystem,
                int cameraId, int N = 10);

    bool addFrameSet(FrameSetPtr& frameSet, bool replaceCurrentFrameSet);

private:
    void optimize(void);

    const int k_cameraId;
    const int k_N;

    Eigen::Matrix4d m_H_c_s;
    Eigen::Quaterniond m_q_s_c;
    Eigen::Vector3d m_t_s_c;

    std::list<FrameSetPtr> m_window;
};

}

#endif
