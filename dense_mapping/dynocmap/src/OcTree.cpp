#include "dynocmap/OcTree.h"

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <cmath>
#include <cstdio>
#include <eigen_conversions/eigen_msg.h>

#include "OcUtils.h"

const size_t kHeaderSize = sizeof(double) * 4 + sizeof(int) * 4;

const double LOGODDS_MAX = 3.5;
const double LOGODDS_MIN = -2.0;
const double LOGODDS_OCC_THRESH = 1.5;

namespace px
{

OcTree::OcTree()
 : m_resolution(0.0)
 , m_treeHeight(1)
 , m_root(boost::make_shared<OcNode>())
 , m_logOddsMax(LOGODDS_MAX)
 , m_logOddsMin(LOGODDS_MIN)
 , m_logOddsOccThresh(LOGODDS_OCC_THRESH)
 , m_batchUpdate(false)
{
    m_center.setZero();
}

OcTree::OcTree(double resolution, int treeHeight,
               const Eigen::Vector3d& center)
 : m_resolution(resolution)
 , m_treeHeight(treeHeight)
 , m_root(boost::make_shared<OcNode>())
 , m_logOddsMax(LOGODDS_MAX)
 , m_logOddsMin(LOGODDS_MIN)
 , m_logOddsOccThresh(LOGODDS_OCC_THRESH)
 , m_batchUpdate(false)
{
    m_center = pointToGridCoords(center, m_resolution);
}

OcTree::OcTree(double resolution, int treeHeight,
               const Eigen::Vector3i& center)
 : m_resolution(resolution)
 , m_treeHeight(treeHeight)
 , m_root(boost::make_shared<OcNode>())
 , m_logOddsMax(LOGODDS_MAX)
 , m_logOddsMin(LOGODDS_MIN)
 , m_logOddsOccThresh(LOGODDS_OCC_THRESH)
 , m_batchUpdate(false)
{
    m_center = center;
}

OcTree::OcTree(const Eigen::Vector3i& center,
               const std::vector<OcTreePtr>& subTrees)
 : m_resolution(subTrees.at(0)->resolution())
 , m_logOddsMax(LOGODDS_MAX)
 , m_logOddsMin(LOGODDS_MIN)
 , m_logOddsOccThresh(LOGODDS_OCC_THRESH)
 , m_batchUpdate(false)
{
    int parentTreeHeight = static_cast<int>(log2(subTrees.size())) / 3 + 1;

    m_treeHeight = parentTreeHeight + subTrees.at(0)->m_treeHeight - 1;

    m_center = center;

    if (subTrees.size() == 1)
    {
        m_root = subTrees.at(0)->m_root;
    }
    else
    {
        m_root = boost::make_shared<OcNode>();

        std::vector<OcNodePtr> tileTrees;
        tileTrees.push_back(m_root);
        for (int i = 0; i < parentTreeHeight - 1; ++i)
        {
            std::vector<OcNodePtr> nodesToSplit = tileTrees;
            tileTrees.clear();

            for (size_t j = 0; j < nodesToSplit.size(); ++j)
            {
                nodesToSplit.at(j)->split();

                std::vector<OcNodePtr>& children = nodesToSplit.at(j)->children();
                tileTrees.insert(tileTrees.end(), children.begin(), children.end());
            }
        }

        for (size_t i = 0; i < subTrees.size(); ++i)
        {
            const OcTreePtr& tile = subTrees.at(i);

            OcNodePtr node = findNode(tile->m_center, parentTreeHeight - 1);
            OcNodePtr parent = node->parent().lock();

            parent->setChild(node->index(), tile->m_root);
        }
    }
}

OcNodePtr
OcTree::insertNode(double x, double y, double z)
{
    return insertNode(Eigen::Vector3d(x,y,z));
}

OcNodePtr
OcTree::insertNode(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i gridCoords = pointToGridCoords(pos, m_resolution);

    return insertNode(gridCoords);
}

OcNodePtr
OcTree::insertNode(const Eigen::Vector3i& pos)
{
    Eigen::Vector3i coords = pos - m_center;

    // check if (x,y,z) is outside boundaries of the octree
    int halfWidth = gridWidth() / 2;
    if (coords(0) < -halfWidth || coords(0) >= halfWidth ||
        coords(1) < -halfWidth || coords(1) >= halfWidth ||
        coords(2) < -halfWidth || coords(2) >= halfWidth)
    {
        return OcNodePtr();
    }

    int depth = 0;
    Eigen::Vector3i nodeCoords = Eigen::Vector3i::Constant(-halfWidth);

    OcNodePtr node = m_root;
    int halfOctantWidth = halfWidth;
    while (depth < m_treeHeight - 1)
    {
        if (node->isLeaf())
        {
            node->split();
        }

        node = node->child(childIndex(coords,
                                      nodeCoords + Eigen::Vector3i::Constant(halfOctantWidth)));

        if (coords(0) >= nodeCoords(0) + halfOctantWidth)
        {
            nodeCoords(0) += halfOctantWidth;
        }
        if (coords(1) >= nodeCoords(1) + halfOctantWidth)
        {
            nodeCoords(1) += halfOctantWidth;
        }
        if (coords(2) >= nodeCoords(2) + halfOctantWidth)
        {
            nodeCoords(2) += halfOctantWidth;
        }

        halfOctantWidth /= 2;
        ++depth;
    }

    return node;
}

OcNodePtr
OcTree::findNode(double x, double y, double z, int maxDepth)
{
    return findNode(Eigen::Vector3d(x,y,z), maxDepth);
}

OcNodePtr
OcTree::findNode(const Eigen::Vector3d& pos, int maxDepth)
{
    Eigen::Vector3i gridCoords = pointToGridCoords(pos, m_resolution);

    return findNode(gridCoords, maxDepth);
}

OcNodePtr
OcTree::findNode(const Eigen::Vector3i& pos, int maxDepth)
{
    Eigen::Vector3i coords = pos - m_center;

    // check if (x,y,z) is outside boundaries of the octree
    int halfWidth = gridWidth() / 2;
    if (coords(0) < -halfWidth || coords(0) >= halfWidth ||
        coords(1) < -halfWidth || coords(1) >= halfWidth ||
        coords(2) < -halfWidth || coords(2) >= halfWidth)
    {
        return OcNodePtr();
    }

    int depth = 0;
    if (maxDepth < 0)
    {
        maxDepth = m_treeHeight - 1;
    }

    int halfOctantWidth = gridWidth() / 2;
    Eigen::Vector3i nodeCoords = Eigen::Vector3i::Constant(-halfOctantWidth);

    OcNodePtr node = m_root;
    while (depth < maxDepth)
    {
        if (node->isLeaf())
        {
            return OcNodePtr();
        }
        else
        {
            node = node->child(childIndex(coords,
                                          nodeCoords + Eigen::Vector3i::Constant(halfOctantWidth)));

            if (coords(0) >= nodeCoords(0) + halfOctantWidth)
            {
                nodeCoords(0) += halfOctantWidth;
            }
            if (coords(1) >= nodeCoords(1) + halfOctantWidth)
            {
                nodeCoords(1) += halfOctantWidth;
            }
            if (coords(2) >= nodeCoords(2) + halfOctantWidth)
            {
                nodeCoords(2) += halfOctantWidth;
            }

            halfOctantWidth /= 2;
            ++depth;
        }
    }

    return node;
}

size_t
OcTree::maximumLeafCount(void) const
{
    size_t count = 1;
    count << (3 * (m_treeHeight - 1));

    return count;
}

std::vector<OccupancyCell, Eigen::aligned_allocator<OccupancyCell> >
OcTree::leafs(void) const
{
    int depth = 0;
    int halfOctantWidth = gridWidth() / 2;

    std::vector<LabeledNode> queue;
    queue.reserve(maximumLeafCount());

    LabeledNode rootNode(m_root, Eigen::Vector3i::Constant(-halfOctantWidth) + m_center);
    queue.push_back(rootNode);

    while (depth < m_treeHeight - 1 && !queue.empty())
    {
        std::vector<LabeledNode> nodes = queue;
        queue.clear();

        for (std::vector<LabeledNode>::iterator it = nodes.begin();
             it != nodes.end(); ++it)
        {
            const LabeledNode& node = *it;
            if (!node.node->isLeaf())
            {
                for (int i = 0; i < 8; ++i)
                {
                    LabeledNode newNode(node.node->child(i), node.coords);

                    if ((i & 0x4) == 0x4)
                    {
                        newNode.coords(0) += halfOctantWidth;
                    }

                    if ((i & 0x2) == 0x2)
                    {
                        newNode.coords(1) += halfOctantWidth;
                    }

                    if ((i & 0x1) == 0x1)
                    {
                        newNode.coords(2) += halfOctantWidth;
                    }

                    queue.push_back(newNode);
                }
            }
        }

        halfOctantWidth /= 2;
        ++depth;
    }

    std::vector<OccupancyCell, Eigen::aligned_allocator<OccupancyCell> > cells;
    cells.reserve(queue.size());

    for (std::vector<LabeledNode>::iterator it = queue.begin();
         it != queue.end(); ++it)
    {
        const LabeledNode& node = *it;

        OccupancyCell cell;
        cell.coords = node.coords;
        cell.width = 1;
        cell.occupancyLogOdds = node.node->getLogOdds();
        cell.occupancyProb = node.node->getProbability();

        cells.push_back(cell);
    }

    return cells;
}

std::vector<OccupancyCell, Eigen::aligned_allocator<OccupancyCell> >
OcTree::obstacles(void) const
{
    int depth = 0;
    int halfOctantWidth = gridWidth() / 2;

    std::vector<LabeledNode> queue;
    queue.reserve(maximumLeafCount());

    LabeledNode rootNode(m_root, Eigen::Vector3i::Constant(-halfOctantWidth) + m_center);

    queue.push_back(rootNode);

    while (depth < m_treeHeight - 1 && !queue.empty())
    {
        std::vector<LabeledNode> nodes = queue;
        queue.clear();

        for (std::vector<LabeledNode>::iterator it = nodes.begin();
             it != nodes.end(); ++it)
        {
            const LabeledNode& node = *it;
            if (!node.node->isLeaf())
            {
                for (int i = 0; i < 8; ++i)
                {
                    LabeledNode newNode(node.node->child(i), node.coords);

                    if ((i & 0x4) == 0x4)
                    {
                        newNode.coords(0) += halfOctantWidth;
                    }

                    if ((i & 0x2) == 0x2)
                    {
                        newNode.coords(1) += halfOctantWidth;
                    }

                    if ((i & 0x1) == 0x1)
                    {
                        newNode.coords(2) += halfOctantWidth;
                    }

                    queue.push_back(newNode);
                }
            }
        }

        halfOctantWidth /= 2;
        ++depth;
    }

    std::vector<OccupancyCell, Eigen::aligned_allocator<OccupancyCell> > obstacles;
    obstacles.reserve(queue.size());

    for (std::vector<LabeledNode>::iterator it = queue.begin();
         it != queue.end(); ++it)
    {
        const LabeledNode& node = *it;

        if (node.node->getLogOdds() > m_logOddsOccThresh)
        {
            OccupancyCell obstacle;
            obstacle.coords = node.coords;
            obstacle.width = 1;
            obstacle.occupancyLogOdds = node.node->getLogOdds();
            obstacle.occupancyProb = node.node->getProbability();

            obstacles.push_back(obstacle);
        }
    }

    return obstacles;
}

double
OcTree::width(void) const
{
    return m_resolution * static_cast<double>(1 << (m_treeHeight - 1));
}

int
OcTree::gridWidth(void) const
{
    return 1 << (m_treeHeight - 1);
}

double
OcTree::resolution(void) const
{
    return m_resolution;
}

int
OcTree::treeHeight(void) const
{
    return m_treeHeight;
}

Eigen::Vector3d
OcTree::center(void) const
{
    return m_center.cast<double>() * m_resolution;
}

Eigen::Vector3i
OcTree::gridCenter(void) const
{
    return m_center;
}

double&
OcTree::logOddsMax(void)
{
    return m_logOddsMax;
}

double
OcTree::logOddsMax(void) const
{
    return m_logOddsMax;
}

double&
OcTree::logOddsMin(void)
{
    return m_logOddsMin;
}

double
OcTree::logOddsMin(void) const
{
    return m_logOddsMin;
}

double&
OcTree::logOddsOccThresh(void)
{
    return m_logOddsOccThresh;
}

double
OcTree::logOddsOccThresh(void) const
{
    return m_logOddsOccThresh;
}

void
OcTree::updateProb(OcNodePtr& node, double prob)
{
    double logOdds = node->getLogOdds();
    logOdds += log(prob / (1.0 - prob));

    if (logOdds > m_logOddsMax)
    {
        logOdds = m_logOddsMax;
    }
    else if (logOdds < m_logOddsMin)
    {
        logOdds = m_logOddsMin;
    }

    node->setLogOdds(logOdds);
}

void
OcTree::updateLogOdds(OcNodePtr& node, double logodds)
{
    node->setLogOdds(fmin(fmax(node->getLogOdds() + logodds, m_logOddsMin), m_logOddsMax));
}

void
OcTree::updateLogOdds(OcNode* node, double logodds)
{
    node->setLogOdds(fmin(fmax(node->getLogOdds() + logodds, m_logOddsMin), m_logOddsMax));
}

void
OcTree::castRay(const geometry_msgs::Pose& sensorPose, const Eigen::Vector3d& endpoint,
                Frame frame, SensorModelPtr& sensorModel)
{
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(sensorPose.orientation, q);

    Eigen::Vector3d t;
    tf::pointMsgToEigen(sensorPose.position, t);

    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H.block<3,3>(0,0) = q.toRotationMatrix();
    H.block<3,1>(0,3) = t;

    castRay(H, endpoint, frame, sensorModel);
}

void
OcTree::castRay(const Eigen::Matrix4d& sensorPose, const Eigen::Vector3d& endpoint,
                Frame frame, SensorModelPtr& sensorModel)
{
    Eigen::Vector3d endpointLocal;
    Eigen::Vector3d endpointGlobal;

    if (frame == GLOBAL_FRAME)
    {
        endpointLocal = sensorPose.block<3,3>(0,0).transpose() * (endpoint - sensorPose.block<3,1>(0,3));
        endpointGlobal = endpoint;
    }
    else
    {
        endpointLocal = endpoint;
        endpointGlobal = sensorPose.block<3,3>(0,0) * endpoint + sensorPose.block<3,1>(0,3);
    }

    bool isEndpointObstacle = false;
    if (endpointLocal.norm() <= sensorModel->maxRange())
    {
        sensorModel->setObstacle(endpointLocal);
        isEndpointObstacle = true;
    }

    // An Efficient Parametric Algorithm for Octree Traversal
    // J. Revelles, C. Urena, M. Lastra
    Eigen::Vector3d o = sensorPose.block<3,1>(0,3);
    Eigen::Vector3d d = endpointGlobal - o;
    d.normalize();

    double maxLength = 0.0;
    if (isEndpointObstacle)
    {
        maxLength = sensorModel->validRange();
    }
    else
    {
        maxLength = sensorModel->maxRange();
    }

    Eigen::Vector3d cornerMin = center() - Eigen::Vector3d::Constant(width() / 2.0);
    Eigen::Vector3d cornerMax = center() + Eigen::Vector3d::Constant(width() / 2.0);

    int a = 0;
    Eigen::Vector3d t0, t1;
    for (int i = 0; i < 3; ++i)
    {
        if (d(i) == 0.0)
        {
            d(i) = 1e-10;
        }

        if (d(i) < 0.0)
        {
            o(i) = cornerMin(i) + cornerMax(i) - o(i);
            d(i) = -d(i);
            a |= (1 << (2 - i));
        }

        t0(i) = (cornerMin(i) - o(i)) / d(i);
        t1(i) = (cornerMax(i) - o(i)) / d(i);
    }

    if (t0.maxCoeff() >= t1.minCoeff())
    {
         return;
    }

    int octantWidth = gridWidth();
    Eigen::Vector3i rootGridCoords = Eigen::Vector3i::Constant(-octantWidth / 2) + m_center;

    std::vector<TraversalNode> queue;
    queue.reserve(1000);
    queue.push_back(TraversalNode(t0, t1, m_root, rootGridCoords));

    int depth = 0;
    while (depth < m_treeHeight - 1)
    {
        std::vector<TraversalNode> nodes = queue;
        queue.clear();

        for (std::vector<TraversalNode>::iterator it = nodes.begin();
             it != nodes.end(); ++it)
        {
            Eigen::Vector3d& t0 = it->t0;
            Eigen::Vector3d& t1 = it->t1;
            OcNodePtr& node = it->node;
            const Eigen::Vector3i& nodeGridCoords = it->gridCoords;

            if (t1.minCoeff() < 0.0 || t0.maxCoeff() > maxLength)
            {
                continue;
            }

            if (node->isLeaf())
            {
                node->split();
            }

            Eigen::Vector3d tm = 0.5 * (t0 + t1);

            int currNode = getFirstIntersectedNode(t0, tm);
            do
            {
                switch (currNode)
                {
                case 0:
                    queue.push_back(TraversalNode(t0, tm,
                                                  node->child(a),
                                                  childCoords(nodeGridCoords, a, octantWidth)));
                    currNode = getNextIntersectedNode(tm, 4, 2, 1);
                    break;
                case 1:
                {
                    Eigen::Vector3d t1_next(tm(0), tm(1), t1(2));

                    queue.push_back(TraversalNode(Eigen::Vector3d(t0(0), t0(1), tm(2)),
                                                  t1_next,
                                                  node->child(1^a),
                                                  childCoords(nodeGridCoords, 1^a, octantWidth)));
                    currNode = getNextIntersectedNode(t1_next, 5, 3, 8);
                    break;
                }
                case 2:
                {
                    Eigen::Vector3d t1_next(tm(0), t1(1), tm(2));

                    queue.push_back(TraversalNode(Eigen::Vector3d(t0(0), tm(1), t0(2)),
                                                  t1_next,
                                                  node->child(2^a),
                                                  childCoords(nodeGridCoords, 2^a, octantWidth)));
                    currNode = getNextIntersectedNode(t1_next, 6, 8, 3);
                    break;
                }
                case 3:
                {
                    Eigen::Vector3d t1_next(tm(0), t1(1), t1(2));

                    queue.push_back(TraversalNode(Eigen::Vector3d(t0(0), tm(1), tm(2)),
                                                  t1_next,
                                                  node->child(3^a),
                                                  childCoords(nodeGridCoords, 3^a, octantWidth)));
                    currNode = getNextIntersectedNode(t1_next, 7, 8, 8);
                    break;
                }
                case 4:
                {
                    Eigen::Vector3d t1_next(t1(0), tm(1), tm(2));

                    queue.push_back(TraversalNode(Eigen::Vector3d(tm(0), t0(1), t0(2)),
                                                  t1_next,
                                                  node->child(4^a),
                                                  childCoords(nodeGridCoords, 4^a, octantWidth)));
                    currNode = getNextIntersectedNode(t1_next, 8, 6, 5);
                    break;
                }
                case 5:
                {
                    Eigen::Vector3d t1_next(t1(0), tm(1), t1(2));

                    queue.push_back(TraversalNode(Eigen::Vector3d(tm(0), t0(1), tm(2)),
                                                  t1_next,
                                                  node->child(5^a),
                                                  childCoords(nodeGridCoords, 5^a, octantWidth)));
                    currNode = getNextIntersectedNode(t1_next, 8, 7, 8);
                    break;
                }
                case 6:
                {
                    Eigen::Vector3d t1_next(t1(0), t1(1), tm(2));

                    queue.push_back(TraversalNode(Eigen::Vector3d(tm(0), tm(1), t0(2)),
                                                  t1_next,
                                                  node->child(6^a),
                                                  childCoords(nodeGridCoords, 6^a, octantWidth)));
                    currNode = getNextIntersectedNode(t1_next, 8, 8, 7);
                    break;
                }
                case 7:
                    queue.push_back(TraversalNode(tm, t1,
                                                  node->child(7^a),
                                                  childCoords(nodeGridCoords, 7^a, octantWidth)));
                    currNode = 8;
                    break;
                }
            }
            while (currNode < 8);
        }

        octantWidth /= 2;

        ++depth;
    }

    for (std::vector<TraversalNode>::iterator it = queue.begin();
         it != queue.end(); ++it)
    {
        double r1 = it->t0.maxCoeff();
        double r2 = it->t1.minCoeff();

        if (r2 < 0.0 || r1 > maxLength)
        {
            continue;
        }

        double logodds = 0.0;
        if (isEndpointObstacle)
        {
            logodds = sensorModel->getLogOddsLikelihood(r1, r2);
        }
        else
        {
            logodds = sensorModel->freeSpaceLogOdds();
        }

        OcNodePtr& node = it->node;

        if (m_batchUpdate)
        {
            if (node->isUpdated())
            {
                if (node->getInterimLogOdds() < logodds)
                {
                    node->setInterimLogOdds(logodds);
                }
            }
            else
            {
                node->setInterimLogOdds(logodds);
                node->setUpdated(true);
            }
        }
        else
        {
            updateLogOdds(it->node, logodds);
        }
    }
}

void
OcTree::castRays(const Eigen::Matrix4d& sensorPose, const cv::Mat& depthImage,
                 const Eigen::Matrix3d& cameraMatrix, SensorModelPtr& sensorModel,
                 bool usePyramid)
{
    double fx = cameraMatrix(0,0);
    double fy = cameraMatrix(1,1);
    double cx = cameraMatrix(0,2);
    double cy = cameraMatrix(1,2);

    initBatchUpdate();

    if (usePyramid)
    {
        std::vector<cv::Mat> pyramid = buildDepthImagePyramid(depthImage);

        std::vector<cv::Point2i> queue;
        queue.reserve(depthImage.rows * depthImage.cols);
        for (int r = 0; r < pyramid.back().rows; ++r)
        {
            const cv::Vec3f* data = pyramid.back().ptr<cv::Vec3f>(r);
            for (int c = 0; c < pyramid.back().cols; ++c)
            {
                if (data[c][0] < 1e-10)
                {
                    continue;
                }

                queue.push_back(cv::Point2i(c,r));
            }
        }

        for (int level = pyramid.size() - 1; level >= 0; --level)
        {
            cv::Mat& image = pyramid.at(level);

            double image_f = fx / (1 << level);
            double image_cx = cx / (1 << level);
            double image_cy = cy / (1 << level);

            std::vector<cv::Point2i> pixels;
            pixels.swap(queue);

            for (std::vector<cv::Point2i>::iterator it = pixels.begin();
                 it != pixels.end(); ++it)
            {
                cv::Vec3f pixel = image.at<cv::Vec3f>(it->y, it->x);
                float z = pixel[0];

                if (z < 1e-10)
                {
                    continue;
                }

                float z_min = pixel[1];
                float z_max = pixel[2];

                double delta_p = z / image_f;

                if ((delta_p < m_resolution &&
                     z - z_min < m_resolution &&
                     z_max - z < m_resolution) ||
                    level == 0)
                {
                    Eigen::Vector3d p;
                    p << (it->x - image_cx) / image_f,
                         (it->y - image_cy) / image_f,
                         1.0;
                    p *= z;

                    castRay(sensorPose, p, SENSOR_FRAME, sensorModel);
                }
                else
                {
                    cv::Point2i p(it->x << 1, it->y << 1);

                    queue.push_back(cv::Point2i(p.x, p.y));
                    queue.push_back(cv::Point2i(p.x + 1, p.y));
                    queue.push_back(cv::Point2i(p.x, p.y + 1));
                    queue.push_back(cv::Point2i(p.x + 1, p.y + 1));
                }
            }
        }
    }
    else
    {
        double z_min = std::numeric_limits<double>::max();

        for (int r = 0; r < depthImage.rows; ++r)
        {
            const float* depth = depthImage.ptr<float>(r);
            for (int c = 0; c < depthImage.cols; ++c)
            {
                float z = depth[c];

                if (z < 1e-10)
                {
                    continue;
                }

                Eigen::Vector3d p;
                p(0) = (c - cx) * z / fx;
                p(1) = (r - cy) * z / fy;
                p(2) = z;

                castRay(sensorPose, p, SENSOR_FRAME, sensorModel);
            }
        }
    }

    finalizeBatchUpdate();
}

bool
OcTree::read(const boost::multi_array<char, 1>& byteArray)
{
    m_root = boost::make_shared<OcNode>();

    if (byteArray.size() <= kHeaderSize)
    {
        return false;
    }

    // read in header data
    m_resolution = *(reinterpret_cast<const double *>(byteArray.data()));
    m_treeHeight = *(reinterpret_cast<const int *>(byteArray.data() + sizeof(double)));

    m_center(0) = *(reinterpret_cast<const int *>(byteArray.data() + sizeof(double) + sizeof(int)));
    m_center(1) = *(reinterpret_cast<const int *>(byteArray.data() + sizeof(double) + sizeof(int) * 2));
    m_center(2) = *(reinterpret_cast<const int *>(byteArray.data() + sizeof(double) + sizeof(int) * 3));

    m_logOddsMax = *(reinterpret_cast<const double *>(byteArray.data() + sizeof(double) + sizeof(int) * 4));
    m_logOddsMin = *(reinterpret_cast<const double *>(byteArray.data() + sizeof(double) * 2 + sizeof(int) * 4));
    m_logOddsOccThresh = *(reinterpret_cast<const double *>(byteArray.data() + sizeof(double) * 3 + sizeof(int) * 4));

    std::vector<OcNode*> queue;
    queue.reserve(maximumLeafCount());

    queue.push_back(m_root.get());

    int depth = 0;
    size_t mark = kHeaderSize;
    int count = 0;

    while (depth < m_treeHeight - 1 && !queue.empty())
    {
        std::vector<OcNode*> nodes;
        nodes.swap(queue);

        BOOST_FOREACH(OcNode* node, nodes)
        {
            if (((byteArray[mark] >> count) & 0x1) == 0x1)
            {
                node->split();

                std::vector<OcNodePtr>& children = node->children();
                for (std::vector<OcNodePtr>::iterator it = children.begin(); it != children.end(); ++it)
                {
                    queue.push_back(it->get());
                }
            }

            ++count;
            if (count == 8)
            {
                ++mark;
                count = 0;
            }
        }

        ++depth;
    }

    if (count > 0)
    {
        ++mark;
        count = 0;
    }

    BOOST_FOREACH(OcNode* node, queue)
    {
        double logodds;
        memcpy(&logodds, &byteArray[mark], sizeof(double));
        node->setLogOdds(logodds);

        mark += sizeof(double);
    }

    return true;
}

bool
OcTree::write(boost::multi_array<char, 1>& byteArray) const
{
    std::vector<char> buffer;
    int depth = 0;

    std::vector<OcNode*> queue;
    queue.reserve(maximumLeafCount());

    queue.push_back(m_root.get());

    char data = 0;
    int count = 0;

    while (depth < m_treeHeight - 1)
    {
        std::vector<OcNode*> nodes;
        nodes.swap(queue);

        BOOST_FOREACH(OcNode* node, nodes)
        {
            if (!node->isLeaf())
            {
                data |= 1 << count;

                std::vector<OcNodePtr>& children = node->children();
                for (std::vector<OcNodePtr>::iterator it = children.begin(); it != children.end(); ++it)
                {
                    queue.push_back(it->get());
                }
            }

            ++count;
            if (count == 8)
            {
                buffer.push_back(data);
                data = 0;
                count = 0;
            }
        }

        ++depth;
    }

    if (count > 0)
    {
        buffer.push_back(data);
    }

    BOOST_FOREACH(OcNode* node, queue)
    {
        double logodds = node->getLogOdds();

        char data[8];
        memcpy(&data[0], &logodds, sizeof(double));
        buffer.insert(buffer.end(), data, data + 8);
    }

    byteArray.resize(boost::extents[kHeaderSize + buffer.size()]);

    // copy tree parameters
    memcpy(byteArray.data(), reinterpret_cast<const char*>(&m_resolution),
           sizeof(double));
    memcpy(byteArray.data() + sizeof(double), &m_treeHeight, sizeof(int));
    memcpy(byteArray.data() + sizeof(double) + sizeof(int),
           reinterpret_cast<const char*>(&m_center(0)), sizeof(int));
    memcpy(byteArray.data() + sizeof(double) + sizeof(int) * 2,
           reinterpret_cast<const char*>(&m_center(1)), sizeof(int));
    memcpy(byteArray.data() + sizeof(double) + sizeof(int) * 3,
           reinterpret_cast<const char*>(&m_center(2)), sizeof(int));
    memcpy(byteArray.data() + sizeof(double) + sizeof(int) * 4,
           reinterpret_cast<const char*>(&m_logOddsMax), sizeof(double));
    memcpy(byteArray.data() + sizeof(double) * 2 + sizeof(int) * 4,
           reinterpret_cast<const char*>(&m_logOddsMin), sizeof(double));
    memcpy(byteArray.data() + sizeof(double) * 3 + sizeof(int) * 4,
           reinterpret_cast<const char*>(&m_logOddsOccThresh), sizeof(double));

    std::copy(buffer.begin(), buffer.end(), byteArray.data() + kHeaderSize);

    return true;
}

bool
OcTree::read(const std::string& filename)
{
    FILE* fp = fopen(filename.c_str(), "rb");

    fseek(fp, 0, SEEK_END);
    size_t size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    boost::multi_array<char, 1> data(boost::extents[size]);
    size_t nBytesRead = fread(data.data(), 1, size, fp);
    if (nBytesRead != size)
    {
        return false;
    }

    fclose(fp);

    if (!read(data))
    {
        return false;
    }

    return true;
}

bool
OcTree::write(const std::string& filename) const
{
    boost::multi_array<char, 1> data;
    if (!write(data))
    {
        return false;
    }

    FILE* fp = fopen(filename.c_str(), "wb");

    if (fwrite(data.data(), 1, data.size(), fp) != data.size())
    {
        return false;
    }

    fclose(fp);

    return true;
}

bool
OcTree::read(const dynocmap_msgs::DynocMapTile& msg)
{
    m_root = boost::make_shared<OcNode>();

    m_resolution = msg.resolution;
    m_treeHeight = msg.tree_height;

    m_center(0) = msg.center_grid_x;
    m_center(1) = msg.center_grid_y;
    m_center(2) = msg.center_grid_z;

    m_logOddsMax = msg.log_odds_max;
    m_logOddsMin = msg.log_odds_min;
    m_logOddsOccThresh = msg.log_odds_occ_thresh;

    std::vector<OcNode*> queue;
    queue.reserve(maximumLeafCount());

    queue.push_back(m_root.get());

    int depth = 0;
    size_t mark = 0;
    int count = 0;

    while (depth < m_treeHeight - 1 && !queue.empty())
    {
        std::vector<OcNode*> nodes;
        nodes.swap(queue);

        BOOST_FOREACH(OcNode* node, nodes)
        {
            if (((msg.data.at(mark) >> count) & 0x1) == 0x1)
            {
                node->split();

                std::vector<OcNodePtr>& children = node->children();
                for (std::vector<OcNodePtr>::iterator it = children.begin(); it != children.end(); ++it)
                {
                    queue.push_back(it->get());
                }
            }

            ++count;
            if (count == 8)
            {
                ++mark;
                count = 0;
            }
        }

        ++depth;
    }

    if (count > 0)
    {
        ++mark;
        count = 0;
    }

    BOOST_FOREACH(OcNode* node, queue)
    {
        double logodds;
        memcpy(&logodds, &msg.data.at(mark), sizeof(double));
        node->setLogOdds(logodds);

        mark += sizeof(double);
    }

    return true;
}

bool
OcTree::write(dynocmap_msgs::DynocMapTile& msg) const
{
    msg.data.clear();

    msg.resolution = m_resolution;
    msg.tree_height = m_treeHeight;
    msg.center_grid_x = m_center(0);
    msg.center_grid_y = m_center(1);
    msg.center_grid_z = m_center(2);
    msg.log_odds_max = m_logOddsMax;
    msg.log_odds_min = m_logOddsMin;
    msg.log_odds_occ_thresh = m_logOddsOccThresh;

    int depth = 0;

    std::vector<OcNode*> queue;
    queue.reserve(maximumLeafCount());

    queue.push_back(m_root.get());

    char data = 0;
    int count = 0;

    while (depth < m_treeHeight - 1 && !queue.empty())
    {
        std::vector<OcNode*> nodes;
        nodes.swap(queue);

        BOOST_FOREACH(OcNode* node, nodes)
        {
            if (!node->isLeaf())
            {
                data |= 1 << count;

                std::vector<OcNodePtr>& children = node->children();
                for (std::vector<OcNodePtr>::iterator it = children.begin(); it != children.end(); ++it)
                {
                    queue.push_back(it->get());
                }
            }

            ++count;
            if (count == 8)
            {
                msg.data.push_back(data);
                data = 0;
                count = 0;
            }
        }

        ++depth;
    }

    if (count > 0)
    {
        msg.data.push_back(data);
    }

    BOOST_FOREACH(OcNode* node, queue)
    {
        double logodds = node->getLogOdds();

        char data[8];
        memcpy(&data[0], &logodds, sizeof(double));
        msg.data.insert(msg.data.end(), data, data + 8);
    }

    return true;
}

int
OcTree::childIndex(const Eigen::Vector3i& p,
                   const Eigen::Vector3i& octantCoords) const
{
    return ((p(0) >= octantCoords(0)) << 2) |
           ((p(1) >= octantCoords(1)) << 1) |
           (p(2) >= octantCoords(2));
}

Eigen::Vector3i
OcTree::childCoords(const Eigen::Vector3i& parentCoords, int childIndex,
                    int parentWidth) const
{
    int halfWidth = parentWidth / 2;

    Eigen::Vector3i coords = parentCoords;

    if ((childIndex & 0x4) == 0x4)
    {
        coords(0) += halfWidth;
    }
    if ((childIndex & 0x2) == 0x2)
    {
        coords(1) += halfWidth;
    }
    if ((childIndex & 0x1) == 0x1)
    {
        coords(2) += halfWidth;
    }

    return coords;
}

int
OcTree::getFirstIntersectedNode(const Eigen::Vector3d& t0,
                                const Eigen::Vector3d& tm) const
{
    int dim;
    t0.maxCoeff(&dim);

    int mask = 0;

    switch (dim)
    {
    case 0:
        // plane YZ
        if (tm(1) < t0(0))
        {
            mask |= 2;
        }
        if (tm(2) < t0(0))
        {
            mask |= 1;
        }
        break;
    case 1:
        // plane XZ
        if (tm(0) < t0(1))
        {
            mask |= 4;
        }
        if (tm(2) < t0(1))
        {
            mask |= 1;
        }
        break;
    case 2:
        // plane XY
        if (tm(0) < t0(2))
        {
            mask |= 4;
        }
        if (tm(1) < t0(2))
        {
            mask |= 2;
        }
        break;
    }

    return mask;
}

int
OcTree::getNextIntersectedNode(const Eigen::Vector3d& tm, int x, int y, int z)
{
    int dim;
    tm.minCoeff(&dim);

    switch (dim)
    {
    case 0:
        return x;
    case 1:
        return y;
    case 2:
    default:
        return z;
    }
}

std::vector<cv::Mat>
OcTree::buildDepthImagePyramid(const cv::Mat& depthImage) const
{
    std::vector<cv::Mat> pyramid;

    cv::Mat image(depthImage.rows, depthImage.cols, CV_32FC3);
    for (int r = 0; r < depthImage.rows; ++r)
    {
        const float* src = depthImage.ptr<float>(r);
        cv::Vec3f* dst = image.ptr<cv::Vec3f>(r);
        for (int c = 0; c < depthImage.cols; ++c)
        {
            dst[c][0] = src[c];
            dst[c][1] = src[c];
            dst[c][2] = src[c];
        }
    }

    pyramid.push_back(image);

    while (pyramid.back().rows % 2 == 0 && pyramid.back().cols % 2 == 0)
    {
        const cv::Mat& prevImage = pyramid.back();
        cv::Mat imageHalf(prevImage.rows / 2, prevImage.cols / 2, CV_32FC3);
        int v = 0;
        for (int r = 0; r < prevImage.rows; r += 2)
        {
            int u = 0;
            const cv::Vec3f* src0 = prevImage.ptr<cv::Vec3f>(r);
            const cv::Vec3f* src1 = prevImage.ptr<cv::Vec3f>(r + 1);
            cv::Vec3f* dst = imageHalf.ptr<cv::Vec3f>(v);
            for (int c = 0; c < prevImage.cols; c += 2)
            {
                Eigen::Vector4f depth;
                depth << src0[c][0], src0[c+1][0], src1[c][0], src1[c+1][0];

                std::vector<float> depthMinVec;
                depthMinVec.reserve(4);
                for (size_t i = 0; i < 2; ++i)
                {
                    if (src0[c+i][1] > 0.0f)
                    {
                        depthMinVec.push_back(src0[c+i][1]);
                    }
                    if (src1[c+i][1] > 0.0f)
                    {
                        depthMinVec.push_back(src1[c+i][1]);
                    }
                }

                std::vector<float> depthMaxVec;
                depthMaxVec.reserve(4);
                for (size_t i = 0; i < 2; ++i)
                {
                    if (src0[c+i][2] > 0.0f)
                    {
                        depthMaxVec.push_back(src0[c+i][2]);
                    }
                    if (src1[c+i][2] > 0.0f)
                    {
                        depthMaxVec.push_back(src1[c+i][2]);
                    }
                }

                dst[u][0] = depth.maxCoeff();
                dst[u][1] = *std::min_element(depthMinVec.begin(), depthMinVec.end());
                dst[u][2] = *std::max_element(depthMaxVec.begin(), depthMaxVec.end());

                ++u;
            }
            ++v;
        }

        pyramid.push_back(imageHalf);
    }

    return pyramid;
}

void
OcTree::initBatchUpdate(void)
{
    m_batchUpdate = true;
}

void
OcTree::finalizeBatchUpdate(void)
{
    std::vector<OcNode*> queue;
    queue.reserve(maximumLeafCount());

    queue.push_back(m_root.get());

    for (int depth = 0; depth < m_treeHeight - 1; ++depth)
    {
        std::vector<OcNode*> nodes;
        nodes.swap(queue);

        BOOST_FOREACH(OcNode* node, nodes)
        {
            if (node->isLeaf())
            {
                continue;
            }

            std::vector<OcNodePtr>& children = node->children();
            for (std::vector<OcNodePtr>::iterator it = children.begin(); it != children.end(); ++it)
            {
                queue.push_back(it->get());
            }
        }
    }

    BOOST_FOREACH(OcNode* node, queue)
    {
        if (node->isUpdated())
        {
            updateLogOdds(node, node->getInterimLogOdds());

            node->setUpdated(false);
        }
    }

    m_batchUpdate = false;
}

}
