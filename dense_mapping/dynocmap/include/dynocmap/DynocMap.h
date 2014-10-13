#ifndef DYNOCMAP_H_
#define DYNOCMAP_H_

#include <boost/multi_array.hpp>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Pose.h>
#include <opencv2/core/core.hpp>
#include <pcl_ros/point_cloud.h>
#include <sensor_models/SensorModel.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

#include "dynocmap/OccupancyTile.h"
#include "dynocmap/OcTree.h"
#include "dynocmap_msgs/DynocMap.h"

namespace px
{

// forward declarations
class OcTreeCache;

class DynocMap
{
public:
    enum Frame
    {
        SENSOR_FRAME,
        GLOBAL_FRAME
    };

    DynocMap();

    DynocMap(double resolution, const SensorModelPtr& sensorModel,
             const std::string& diskCacheDir, bool enableCaching = true);

    void clear(void);
    void recenter(const Eigen::Vector3d& center);

    void insertScan(const geometry_msgs::Pose& sensorPose,
                    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& scanEndpoints,
                    Frame frame);

    template<typename PointT>
    void insertScan(const geometry_msgs::Pose& sensorPose,
                    const pcl::PointCloud<PointT>& cloud,
                    Frame frame);

    void castRay(const geometry_msgs::Pose& sensorPose,
                 const Eigen::Vector3d& endpoint, Frame frame);
    void castRays(const Eigen::Matrix4d& sensorPose, const cv::Mat& depthImage,
                  const Eigen::Matrix3d& cameraMatrix, bool usePyramid = true);

    void updateCell(const Eigen::Vector3d& pos, double prob);
    void updateCell(const Eigen::Vector3i& pos, double prob);

    double resolution(void) const;
    Eigen::Vector3d center(void) const;
    double mapWidth(void) const;
    double tileWidth(void) const;

    std::vector<OccupancyTile, Eigen::aligned_allocator<OccupancyTile> > tiles(void) const;

    bool read(const boost::multi_array<char, 1>& data);
    bool write(boost::multi_array<char, 1>& data) const;

    bool read(const dynocmap_msgs::DynocMap& msg);
    bool write(dynocmap_msgs::DynocMap& msg, const std::string& frameId = "") const;

private:
    int mapGridWidth(void) const;
    int tileGridWidth(void) const;

    void addRowNorth(void);
    void addRowSouth(void);
    void addColumnEast(void);
    void addColumnWest(void);
    void addSliceUp(void);
    void addSliceDown(void);

    void fillEmptyTiles(void);
    void buildMapTree(void);
    void prefetch(void);

    std::string getTileFilename(double resolution, int depthLevel,
                                const Eigen::Vector3i& tileCenter,
                                const std::string& cacheDir) const;
    Eigen::Vector3i gridCoordsToTileCenter(const Eigen::Vector3i& p,
                                           bool addOffset) const;

    void loadTile(OcTreePtr& tile,
                  const Eigen::Vector3i& tileCenterGridCoords) const;
    void unloadTile(OcTreePtr& tile,
                    const Eigen::Vector3i& tileCenterGridCoords) const;

    Eigen::Vector3i m_center;

    double m_resolution;
    int m_tileTreeHeight;
    int m_mapTreeHeight;

    SensorModelPtr m_sensorModel;
    double m_maxSensorRange;

    std::string m_diskCacheDir;

    std::vector<OcTreePtr> m_mapGrid;
    Eigen::Vector3i m_mapGridOffset;

    OcTreePtr m_mapTree;

    boost::shared_ptr<OcTreeCache> m_memoryCache;
};

typedef boost::shared_ptr<DynocMap> DynocMapPtr;

template<typename PointT>
void
DynocMap::insertScan(const geometry_msgs::Pose& sensorPose,
                     const pcl::PointCloud<PointT>& cloud,
                     Frame frame)
{
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        const PointT& point = cloud.at(i);

        castRay(sensorPose, Eigen::Vector3d(point.x, point.y, point.z), frame);
    }
}

}

#endif
