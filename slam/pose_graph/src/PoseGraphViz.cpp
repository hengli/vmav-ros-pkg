#include "pose_graph/PoseGraphViz.h"

namespace px
{

PoseGraphViz::PoseGraphViz(ros::NodeHandle& nh,
                           const PoseGraphConstPtr& poseGraph)
 : m_nh(nh)
 , k_poseGraph(poseGraph)
{
    m_edgeVizPub = nh.advertise<visualization_msgs::Marker>("pose_graph_marker", 1);
}

void
PoseGraphViz::visualize(const std::string& marker_ns)
{
    visualizeEdges(marker_ns);
}

void
PoseGraphViz::visualizeEdges(const std::string& marker_ns)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "vmav";
    marker.header.stamp = ros::Time::now();

    marker.ns = marker_ns;
    marker.id = 0;

    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    std_msgs::ColorRGBA cRed; // this color marks wrong loop closure edges
    std_msgs::ColorRGBA cGreen; // this color marks correct loop closure edges
    std_msgs::ColorRGBA cBlue; // this color marks VO edges

    cRed.r = 1.0f;
    cRed.g = 0.0f;
    cRed.b = 0.0f;

    cGreen.r = 0.0f;
    cGreen.g = 1.0f;
    cGreen.b = 0.0f;

    cBlue.r = 0.0f;
    cBlue.g = 0.0f;
    cBlue.b = 1.0f;

    // Draw correct loop closure edges.
    std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> > cLoopClosureEdges =
        k_poseGraph->getLoopClosureEdges(true);

    drawEdges(cLoopClosureEdges, cGreen, marker);

    // Draw wrong loop closure edges.
    std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> > wLoopClosureEdges =
        k_poseGraph->getLoopClosureEdges(false);

    drawEdges(wLoopClosureEdges, cRed, marker);

    // Draw VO edges.
    std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> > voEdges =
        k_poseGraph->getVOEdges();

    drawEdges(voEdges, cBlue, marker);

    m_edgeVizPub.publish(marker);
}

void
PoseGraphViz::drawEdges(const std::vector<std::pair<PoseConstWPtr, PoseConstWPtr> >& edges,
                        const std_msgs::ColorRGBA& color,
                        visualization_msgs::Marker& marker) const
{
    for (size_t i = 0; i < edges.size(); ++i)
    {
        const std::pair<PoseConstWPtr, PoseConstWPtr>& edge = edges.at(i);

        Eigen::Matrix4d H1 = edge.first.lock()->toMatrix();
        Eigen::Matrix4d H2 = edge.second.lock()->toMatrix();

        Eigen::Vector3d t1_inv = -H1.block<3,3>(0,0).transpose() * H1.block<3,1>(0,3);
        Eigen::Vector3d t2_inv = -H2.block<3,3>(0,0).transpose() * H2.block<3,1>(0,3);

        geometry_msgs::Point p[2];

        p[0].x = t1_inv(0);
        p[0].y = t1_inv(1);
        p[0].z = t1_inv(2);

        p[1].x = t2_inv(0);
        p[1].y = t2_inv(1);
        p[1].z = t2_inv(2);

        marker.points.push_back(p[0]);
        marker.points.push_back(p[1]);

        marker.colors.push_back(color);
        marker.colors.push_back(color);
    }
}

}
