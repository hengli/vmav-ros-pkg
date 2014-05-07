#ifndef GCAMDWBA_H
#define GCAMDWBA_H

#include <Eigen/Dense>
#include <list>
#include <ros/ros.h>

#include "camera_systems/CameraSystem.h"
#include "sparse_graph/SparseGraph.h"

namespace px
{

class GCamDWBA
{
public:
    GCamDWBA(ros::NodeHandle& nh,
             const CameraSystemConstPtr& cameraSystem,
             int M1 = 15, int M2 = 50);

    void optimize(FrameSetPtr& refFrameSet);

private:
    const int k_M1;
    const int k_M2;

    ros::Publisher m_mapVizPub;
    ros::Publisher m_poseVizPub;

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > m_H_cam_sys;
    std::vector<Transform, Eigen::aligned_allocator<Transform> > m_T_sys_cam;
};

}

#endif
