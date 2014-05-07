#ifndef POSEGRAPHVIZ_H
#define POSEGRAPHVIZ_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "pose_graph/PoseGraph.h"

namespace px
{

class PoseGraphViz
{
public:
    PoseGraphViz(ros::NodeHandle& nh, const PoseGraphConstPtr& poseGraph);

    void visualize(const std::string& marker_ns = "");

private:
    void visualizeEdges(const std::string& marker_ns);

    void drawEdges(const std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> >& edges,
                   const std_msgs::ColorRGBA& color,
                   visualization_msgs::Marker& marker) const;

    ros::NodeHandle m_nh;
    ros::Publisher m_edgeVizPub;

    const PoseGraphConstPtr k_poseGraph;
};

}

#endif
