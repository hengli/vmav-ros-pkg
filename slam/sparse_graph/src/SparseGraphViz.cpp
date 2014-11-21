#include "sparse_graph/SparseGraphViz.h"

#include <boost/unordered_set.hpp>
#include <visualization_msgs/Marker.h>

#include "cauldron/EigenUtils.h"

namespace px
{

SparseGraphViz::SparseGraphViz(ros::NodeHandle& nh,
                               const SparseGraphConstPtr& sparseGraph,
                               const std::string& ns)
 : m_nh(nh)
 , k_sparseGraph(sparseGraph)
 , k_ns(ns)
{
    m_mapVizPub = nh.advertise<visualization_msgs::Marker>("map_marker", 1);
    m_poseVizPub = nh.advertise<visualization_msgs::Marker>("pose_marker", 1);
}

void
SparseGraphViz::visualize(int windowSize)
{
    visualizeMap(windowSize);
    visualizePoses();
}

void
SparseGraphViz::visualizeMap(int windowSize)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "vmav";
    marker.header.stamp = ros::Time::now();

    if (k_ns.empty())
    {
        marker.ns = "map";
    }
    else
    {
        marker.ns = k_ns + "_map";
    }
    marker.id = 0;

    marker.type = visualization_msgs::Marker::SPHERE_LIST;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
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
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    const FrameSetSegment& frameSetSegment = k_sparseGraph->frameSetSegment(0);

    boost::unordered_set<Point3DFeatureConstPtr> scenePointMap;
    int nFrameSets = 0;
    FrameSetSegment::const_reverse_iterator itFrameSet = frameSetSegment.rbegin();
    while (itFrameSet != frameSetSegment.rend())
    {
        FrameSet* frameSet = itFrameSet->get();

        for (size_t i = 0; i < frameSet->frames().size(); ++i)
        {
            const std::vector<Point2DFeaturePtr>& features = frameSet->frames().at(i)->features2D();

            for (size_t j = 0; j < features.size(); ++j)
            {
                const Point2DFeatureConstPtr& feature = features.at(j);
                const Point3DFeatureConstPtr& scenePoint = feature->feature3D();

                if (!scenePoint)
                {
                    continue;
                }

                if (scenePoint->features2D().size() <= 2)
                {
                    continue;
                }

                if (scenePointMap.find(scenePoint) == scenePointMap.end())
                {
                    scenePointMap.insert(scenePoint);
                }
            }
        }

        ++nFrameSets;
        ++itFrameSet;

        if (windowSize > 0 && windowSize == nFrameSets)
        {
            break;
        }
    }

    for (boost::unordered_set<Point3DFeatureConstPtr>::iterator it = scenePointMap.begin();
         it != scenePointMap.end(); ++it)
    {
        Eigen::Vector3d P = (*it)->point();

        geometry_msgs::Point p;
        p.x = P(0);
        p.y = P(1);
        p.z = P(2);

        marker.points.push_back(p);
    }

    m_mapVizPub.publish(marker);
}

void
SparseGraphViz::visualizePoses(void)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "vmav";
    marker.header.stamp = ros::Time::now();

    if (k_ns.empty())
    {
        marker.ns = "cam_poses";
    }
    else
    {
        marker.ns = k_ns + "_cam_poses";
    }
    marker.id = 0;

    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.f;

    marker.lifetime = ros::Duration();

    std_msgs::ColorRGBA axisColors[3];

    // x-axis
    axisColors[0].r = 1.0f;
    axisColors[0].g = 0.0f;
    axisColors[0].b = 0.0f;

    // y-axis
    axisColors[1].r = 0.0f;
    axisColors[1].g = 1.0f;
    axisColors[1].b = 0.0f;

    // z-axis
    axisColors[2].r = 0.0f;
    axisColors[2].g = 0.0f;
    axisColors[2].b = 1.0f;

    double axisLength = 0.1;

    const FrameSetSegment& frameSetSegment = k_sparseGraph->frameSetSegment(0);

    for (size_t i = 0; i < frameSetSegment.size(); ++i)
    {
        Eigen::Matrix4d systemPose = frameSetSegment.at(i)->systemPose()->toMatrix();
        Eigen::Matrix4d systemPose_inv = invertHomogeneousTransform(systemPose);

        geometry_msgs::Point p[4];

        Eigen::Vector3d p_o = systemPose_inv.block<3,1>(0,3);
        p[0].x = p_o(0);
        p[0].y = p_o(1);
        p[0].z = p_o(2);

        // point along x-axis
        Eigen::Vector3d p_x = transformPoint(systemPose_inv, Eigen::Vector3d(axisLength, 0.0, 0.0));
        p[1].x = p_x(0);
        p[1].y = p_x(1);
        p[1].z = p_x(2);

        // point along y-axis
        Eigen::Vector3d p_y = transformPoint(systemPose_inv, Eigen::Vector3d(0.0, axisLength, 0.0));
        p[2].x = p_y(0);
        p[2].y = p_y(1);
        p[2].z = p_y(2);

        // point along z-axis
        Eigen::Vector3d p_z = transformPoint(systemPose_inv, Eigen::Vector3d(0.0, 0.0, axisLength));
        p[3].x = p_z(0);
        p[3].y = p_z(1);
        p[3].z = p_z(2);

        marker.points.push_back(p[0]);
        marker.points.push_back(p[1]);
        marker.points.push_back(p[0]);
        marker.points.push_back(p[2]);
        marker.points.push_back(p[0]);
        marker.points.push_back(p[3]);

        marker.colors.push_back(axisColors[0]);
        marker.colors.push_back(axisColors[0]);
        marker.colors.push_back(axisColors[1]);
        marker.colors.push_back(axisColors[1]);
        marker.colors.push_back(axisColors[2]);
        marker.colors.push_back(axisColors[2]);
    }

    // VO edges
    if (frameSetSegment.size() > 1)
    {
        std::vector<geometry_msgs::Point> positions(frameSetSegment.size());
        for (size_t i = 0; i < frameSetSegment.size(); ++i)
        {
            Eigen::Matrix4d systemPose = frameSetSegment.at(i)->systemPose()->toMatrix();
            Eigen::Matrix4d systemPose_inv = invertHomogeneousTransform(systemPose);

            Eigen::Vector3d p = systemPose_inv.block<3,1>(0,3);

            positions.at(i).x = p(0);
            positions.at(i).y = p(1);
            positions.at(i).z = p(2);
        }

        std_msgs::ColorRGBA color;

        color.r = 0.5f;
        color.g = 0.5f;
        color.b = 0.5f;

        for (size_t i = 0; i < positions.size() - 1; ++i)
        {
            marker.points.push_back(positions.at(i));
            marker.points.push_back(positions.at(i+1));

            marker.colors.push_back(color);
            marker.colors.push_back(color);
        }
    }

    // loop closure edges
    std_msgs::ColorRGBA color;
    color.r = 0.0f;
    color.g = 1.0f;
    color.b = 0.0f;

    for (size_t i = 0; i < frameSetSegment.size(); ++i)
    {
        const FrameSetPtr& frameSet1 = frameSetSegment.at(i);

        for (size_t j = 0; j < frameSet1->frames().size(); ++j)
        {
            const FramePtr& frame1 = frameSet1->frame(j);

            if (frame1->loopClosureEdges().empty())
            {
                continue;
            }

            Eigen::Matrix4d systemPose1 = frameSet1->systemPose()->toMatrix();
            Eigen::Matrix4d systemPose1_inv = invertHomogeneousTransform(systemPose1);
            geometry_msgs::Point p1;
            p1.x = systemPose1_inv(0,3);
            p1.y = systemPose1_inv(1,3);
            p1.z = systemPose1_inv(2,3);

            for (size_t k = 0; k < frame1->loopClosureEdges().size(); ++k)
            {
                const LoopClosureEdge& edge = frame1->loopClosureEdges().at(k);

                Frame* frame2 = edge.inFrame();
                FrameSet* frameSet2 = frame2->frameSet();

                Eigen::Matrix4d systemPose2 = frameSet2->systemPose()->toMatrix();
                Eigen::Matrix4d systemPose2_inv = invertHomogeneousTransform(systemPose2);
                geometry_msgs::Point p2;
                p2.x = systemPose2_inv(0,3);
                p2.y = systemPose2_inv(1,3);
                p2.z = systemPose2_inv(2,3);

                marker.points.push_back(p1);
                marker.points.push_back(p2);

                marker.colors.push_back(color);
                marker.colors.push_back(color);
            }
        }
    }

    m_poseVizPub.publish(marker);
}

}
