#ifndef OCTREE_H_
#define OCTREE_H_

#include <boost/multi_array.hpp>
#include <boost/unordered_set.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <opencv2/core/core.hpp>
#include <sensor_models/SensorModel.h>
#include <vector>

#include "dynocmap/OccupancyCell.h"
#include "dynocmap/OcNode.h"
#include "dynocmap_msgs/DynocMapTile.h"

namespace px
{

class OcTree;
typedef boost::shared_ptr<OcTree> OcTreePtr;

class OcTree
{
public:
    enum Frame
    {
        SENSOR_FRAME,
        GLOBAL_FRAME
    };

    OcTree();
    OcTree(double resolution, int depthLevel,
           const Eigen::Vector3d& center);
    OcTree(double resolution, int depthLevel,
           const Eigen::Vector3i& center);
    OcTree(const Eigen::Vector3i& center,
           const std::vector<OcTreePtr>& subTrees);

    OcNodePtr insertNode(double x, double y, double z);
    OcNodePtr insertNode(const Eigen::Vector3d& pos);
    OcNodePtr insertNode(const Eigen::Vector3i& pos);
    OcNodePtr findNode(double x, double y, double z, int maxDepth = -1);
    OcNodePtr findNode(const Eigen::Vector3d& pos, int maxDepth = -1);
    OcNodePtr findNode(const Eigen::Vector3i& pos, int maxDepth = -1);

    size_t maximumLeafCount(void) const;

    std::vector<OccupancyCell> leafs(void) const;
    std::vector<OccupancyCell> obstacles(void) const;

    double width(void) const;
    int gridWidth(void) const;
    double resolution(void) const;
    int treeHeight(void) const;
    Eigen::Vector3d center(void) const;
    Eigen::Vector3i gridCenter(void) const;

    double& logOddsMax(void);
    double logOddsMax(void) const;

    double& logOddsMin(void);
    double logOddsMin(void) const;

    double& logOddsOccThresh(void);
    double logOddsOccThresh(void) const;

    void updateProb(OcNodePtr& node, double prob);
    void updateLogOdds(OcNodePtr& node, double logodds);
    void updateLogOdds(OcNode* node, double logodds);

    void castRay(const geometry_msgs::Pose& sensorPose, const Eigen::Vector3d& endpoint,
                 Frame frame, SensorModelPtr& sensorModel);
    void castRay(const Eigen::Matrix4d& sensorPose, const Eigen::Vector3d& endpoint,
                 Frame frame, SensorModelPtr& sensorModel);
    void castRays(const Eigen::Matrix4d& sensorPose, const cv::Mat& depthImage,
                  const Eigen::Matrix3d& cameraMatrix, SensorModelPtr& sensorModel,
                  bool usePyramid = true);

    bool read(const boost::multi_array<char, 1>& data);
    bool write(boost::multi_array<char, 1>& data) const;

    bool read(const std::string& filename);
    bool write(const std::string& filename) const;

    bool read(const dynocmap_msgs::DynocMapTile& msg);
    bool write(dynocmap_msgs::DynocMapTile& msg) const;

private:
    int childIndex(const Eigen::Vector3i& p,
                   const Eigen::Vector3i& octantCoords) const;
    Eigen::Vector3i childCoords(const Eigen::Vector3i& parentCoords,
                                int childIndex, int parentWidth) const;

    int getFirstIntersectedNode(const Eigen::Vector3d& t0, const Eigen::Vector3d& tm) const;
    int getNextIntersectedNode(const Eigen::Vector3d& tm, int x, int y, int z);

    std::vector<cv::Mat> buildDepthImagePyramid(const cv::Mat& depthImage) const;

    void initBatchUpdate(void);
    void finalizeBatchUpdate(void);

    class LabeledNode
    {
    public:
        LabeledNode(const OcNodePtr& _node, const Eigen::Vector3i& _coords)
         : node(_node.get())
         , coords(_coords)
        {

        }

        OcNode* node;
        Eigen::Vector3i coords;
    };

    class TraversalNode
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TraversalNode(const Eigen::Vector3d& _t0, const Eigen::Vector3d& _t1,
                      const OcNodePtr& _node,
                      const Eigen::Vector3i& _gridCoords)
         : t0(_t0)
         , t1(_t1)
         , node(_node)
         , gridCoords(_gridCoords)
        {

        }

        Eigen::Vector3d t0;
        Eigen::Vector3d t1;
        OcNodePtr node;
        Eigen::Vector3i gridCoords;
    };

    double m_resolution;
    int m_treeHeight;

    Eigen::Vector3i m_center;

    OcNodePtr m_root;

    double m_logOddsMax;
    double m_logOddsMin;
    double m_logOddsOccThresh;

    bool m_batchUpdate;
};

}

#endif
