#ifndef SPARSEGRAPHVIZ_H
#define SPARSEGRAPHVIZ_H

#include <ros/ros.h>

#include "sparse_graph/SparseGraph.h"

namespace px
{

class SparseGraphViz
{
public:
    SparseGraphViz(ros::NodeHandle& nh,
                   const SparseGraphConstPtr& sparseGraph,
                   const std::string& ns = "");

    void visualize(int windowSize = 0);

private:
    void visualizeMap(int windowSize);
    void visualizePoses(void);
    
    ros::NodeHandle m_nh;
    ros::Publisher m_mapVizPub;
    ros::Publisher m_poseVizPub;

    const SparseGraphConstPtr k_sparseGraph;
    const std::string k_ns;
};

}

#endif
