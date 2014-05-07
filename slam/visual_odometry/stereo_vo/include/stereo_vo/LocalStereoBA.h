#ifndef LOCALSTEREOBA_H
#define LOCALSTEREOBA_H

#include <Eigen/Dense>
#include <list>

#include "camera_systems/CameraSystem.h"
#include "sparse_graph/SparseGraph.h"

namespace px
{

class LocalStereoBA
{
public:
    LocalStereoBA(const CameraSystemConstPtr& cameraSystem,
                  int cameraId1, int cameraId2,
                  int N = 10);

    bool addFrameSet(FrameSetPtr& frameSet, bool replaceCurrentFrameSet);

private:
    void optimize(void);

    const int k_cameraId1;
    const int k_cameraId2;
    const int k_N;

    Eigen::Matrix4d m_H_1_s;
    Eigen::Quaterniond m_q_s_1;
    Eigen::Vector3d m_t_s_1;
    Eigen::Matrix4d m_H_2_s;
    Eigen::Quaterniond m_q_s_2;
    Eigen::Vector3d m_t_s_2;

    std::list<FrameSetPtr> m_window;
};

}

#endif
