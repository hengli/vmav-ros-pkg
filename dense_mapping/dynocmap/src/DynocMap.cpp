#include "dynocmap/DynocMap.h"

#include <boost/filesystem.hpp>
#include <cmath>
#include <vector>

#include "OcTreeCache.h"
#include "OcUtils.h"

namespace px
{

DynocMap::DynocMap()
 : m_center(Eigen::Vector3i::Zero())
 , m_resolution(0.0)
 , m_tileTreeHeight(0)
 , m_mapTreeHeight(0)
 , m_mapGridOffset(Eigen::Vector3i::Zero())
{

}

DynocMap::DynocMap(double resolution, const SensorModelPtr& sensorModel,
                   const std::string& diskCacheDir, bool enableMemoryCaching)
 : m_center(Eigen::Vector3i::Zero())
 , m_resolution(resolution)
 , m_mapTreeHeight(3)
 , m_sensorModel(sensorModel)
 , m_diskCacheDir(diskCacheDir)
 , m_mapGridOffset(Eigen::Vector3i::Zero())
{
    if (boost::filesystem::exists(diskCacheDir.c_str()) &&
        boost::filesystem::is_directory(diskCacheDir.c_str()))
    {
        boost::filesystem::remove_all(diskCacheDir.c_str());
    }
    boost::filesystem::create_directory(diskCacheDir.c_str());

    m_tileTreeHeight = static_cast<int>(ceilf(log2(sensorModel->maxRange() / resolution))) + 1;

    int nTiles = cube(mapGridWidth());
    m_mapGrid.resize(nTiles);

    if (enableMemoryCaching)
    {
        int cacheTreeHeight = m_mapTreeHeight + 1;
        int cacheGridWidth =  1 << (cacheTreeHeight - 1);

        m_memoryCache = boost::make_shared<OcTreeCache>(cube(cacheGridWidth));
    }

    fillEmptyTiles();
    buildMapTree();
}

void
DynocMap::clear(void)
{
    if (boost::filesystem::exists(m_diskCacheDir.c_str()) &&
        boost::filesystem::is_directory(m_diskCacheDir.c_str()))
    {
        boost::filesystem::remove_all(m_diskCacheDir.c_str());
    }
    boost::filesystem::create_directory(m_diskCacheDir.c_str());

    int width = mapGridWidth();
    int nTiles = width * width * width;
    for (int i = 0; i < nTiles; ++i)
    {
        m_mapGrid.at(i).reset();
    }

    fillEmptyTiles();
    buildMapTree();
}

void
DynocMap::recenter(const Eigen::Vector3d& center)
{
    Eigen::Vector3i centerI = pointToGridCoords(center, m_resolution);

    Eigen::Vector3i d = centerI - m_center;

    for (int i = 0; i < 3; ++i)
    {
        d(i) = static_cast<int>(round(static_cast<double>(d(i)) / static_cast<double>(tileGridWidth())));
    }

    if (d == Eigen::Vector3i::Zero())
    {
        return;
    }

    int width = mapGridWidth();
    if (std::abs(d(0)) >= width ||
        std::abs(d(1)) >= width ||
        std::abs(d(2)) >= width)
    {
        // load all tiles from cache
        int nTiles = width * width * width;

        for (int i = 0; i < nTiles; ++i)
        {
            int c = i % width;
            int r = (i / width) % width;
            int s = i / (width * width);

            Eigen::Vector3i tileCenter;
            tileCenter = gridCoordsToTileCenter(Eigen::Vector3i(c, r, s), true);

            unloadTile(m_mapGrid.at(i), tileCenter);
        }

        m_center += d * tileGridWidth();
    }
    else
    {
        if (d(0) > 0)
        {
            for (int i = 0; i < d(0); i++)
            {
                addColumnWest();
            }
        }
        else if (d(0) < 0)
        {
            for (int i = 0; i < abs(d(0)); i++)
            {
                addColumnEast();
            }
        }
        if (d(1) > 0)
        {
            for (int i = 0; i < d(1); i++)
            {
                addRowNorth();
            }
        }
        else if (d(1) < 0)
        {
            for (int i = 0; i < abs(d(1)); i++)
            {
                addRowSouth();
            }
        }
        if (d(2) > 0)
        {
            for (int i = 0; i < d(2); i++)
            {
                addSliceUp();
            }
        }
        else if (d(2) < 0)
        {
            for (int i = 0; i < abs(d(2)); i++)
            {
                addSliceDown();
            }
        }
    }

    fillEmptyTiles();
    buildMapTree();

    prefetch();
}

void
DynocMap::insertScan(const geometry_msgs::Pose& sensorPose,
                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& scanEndpoints,
                     Frame frame)
{
    for (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >::const_iterator it = scanEndpoints.begin();
         it != scanEndpoints.end(); ++it)
    {
        castRay(sensorPose, *it, frame);
    }
}

void
DynocMap::castRay(const geometry_msgs::Pose& sensorPose, const Eigen::Vector3d& endpoint,
                  Frame frame)
{
    m_mapTree->castRay(sensorPose, endpoint, (OcTree::Frame)frame,
                       m_sensorModel);

//    Eigen::Vector3d endpointLocal;
//    Eigen::Vector3d endpointGlobal;
//
//    if (frame == GLOBAL_FRAME)
//    {
//        endpointLocal = (sensorPose.q() * m_sensorModel->sensorToWorldTransform()).conjugate().toRotationMatrix() * (endpoint - sensorPose.t());
//        endpointGlobal = endpoint;
//    }
//    else
//    {
//        endpointLocal = endpoint;
//        endpointGlobal = (sensorPose.q() * m_sensorModel->sensorToWorldTransform()).toRotationMatrix() * endpoint + sensorPose.t();
//    }
//
//    m_sensorModel->setObstacle(endpointLocal);
//
//    if (m_rayTracingLUT.get() == 0)
//    {
//        Eigen::Vector3d o = sensorPose.t();
//        Eigen::Vector3d d = endpointGlobal - o;
//        double maxLength = fmin(d.norm(), m_sensorModel->maxSensorRange());
//        d.normalize();
//
//        // indicates how far along the ray we must move for each component
//        // of the translation to equal the width of a voxel
//        Eigen::Vector3d tDelta = (d.cwiseInverse() * m_resolution).cwiseAbs();
//
//        // ray length at which the ray crosses the first voxel boundary along the axis
//        // corresponding to the component
//        Eigen::Vector3d tMax;
//
//        Eigen::Vector3i step;
//        for (int i = 0; i < 3; ++i)
//        {
//            step(i) = (d(i) >= 0.0 ? 1 : -1);
//
//            if (fabs(d(i)) > 1e-10)
//            {
//                if (step(i) >= 0)
//                {
//                    tMax(i) = (ceil(o(i) / m_resolution) * m_resolution - o(i)) / d(i);
//                }
//                else
//                {
//                    tMax(i) = (floor(o(i) / m_resolution) * m_resolution - o(i)) / d(i);
//                }
//            }
//            else
//            {
//                tMax(i) = std::numeric_limits<double>::max();
//            }
//        }
//
//        Eigen::Vector3i p = pointToGridCoords(o, m_resolution);
//
//        int last_dim = -1;
//
//        bool quit = false;
//        while (!quit)
//        {
//            int dim;
//            (tMax + tDelta).minCoeff(&dim);
//
//            Eigen::Vector3d tMax_update = tMax;
//            tMax_update(dim) += tDelta(dim);
//
//            double prob = 0.0;
//            if (last_dim == -1)
//            {
//                prob = m_sensorModel->getLikelihood(0.0, tMax_update(dim));
//            }
//            else
//            {
//                prob = m_sensorModel->getLikelihood(tMax(last_dim), tMax_update(dim));
//            }
//
//            updateCell(p, prob);
//
//            if (last_dim != -1 && tMax(last_dim) >= maxLength)
//            {
//                quit = true;
//            }
//
//            p(dim) += step(dim);
//            tMax = tMax_update;
//
//            last_dim = dim;
//        }
//    }
//    else
//    {
//        Eigen::Vector3i o = pointToGridCoords(sensorPose.t(), m_resolution);
//        Eigen::Vector3i p = pointToGridCoords(endpointGlobal, m_resolution);
//
//        RayTracingLUT::RayPtr ray = m_rayTracingLUT->castRay(o, p);
//
//        for (std::vector<RayTracingLUT::Cell>::const_iterator it = ray->cells.begin();
//             it != ray->cells.end(); ++it)
//        {
//            double prob = m_sensorModel->getLikelihood(it->r1() * m_resolution,
//                                                       it->r2() * m_resolution);
//
//            updateCell(Eigen::Vector3i(it->x(), it->y(), it->z()), prob);
//        }
//    }
}

void
DynocMap::castRays(const Eigen::Matrix4d& sensorPose, const cv::Mat& depthImage,
                   const Eigen::Matrix3d& cameraMatrix, bool usePyramid)
{
    m_mapTree->castRays(sensorPose, depthImage, cameraMatrix,
                        m_sensorModel, usePyramid);
}

void
DynocMap::updateCell(const Eigen::Vector3d& pos, double prob)
{
    OcNodePtr node = m_mapTree->findNode(pos);

    if (!node)
    {
        node = m_mapTree->insertNode(pos);
    }

    m_mapTree->updateLogOdds(node, prob);
}

void
DynocMap::updateCell(const Eigen::Vector3i& pos, double prob)
{
    OcNodePtr node = m_mapTree->findNode(pos);

    if (!node)
    {
        node = m_mapTree->insertNode(pos);
    }

    m_mapTree->updateLogOdds(node, prob);
}

double
DynocMap::resolution(void) const
{
    return m_resolution;
}

Eigen::Vector3d
DynocMap::center(void) const
{
    return m_center.cast<double>() * m_resolution;
}

double
DynocMap::mapWidth(void) const
{
    return mapGridWidth() * tileWidth();
}

double
DynocMap::tileWidth(void) const
{
    return tileGridWidth() * m_resolution;
}

std::vector<OccupancyTile, Eigen::aligned_allocator<OccupancyTile> >
DynocMap::tiles(void) const
{
    std::vector<OccupancyTile, Eigen::aligned_allocator<OccupancyTile> > otiles(m_mapGrid.size());

    for (size_t i = 0; i < m_mapGrid.size(); ++i)
    {
        const OcTreePtr& src = m_mapGrid.at(i);

        OccupancyTile& dst = otiles.at(i);

        dst.resolution() = m_resolution;
        dst.tileGridWidth() = src->gridWidth();
        dst.tileGridCenter() = src->gridCenter();
        dst.cells() = src->leafs();
        dst.obstacles() = src->obstacles();
    }

    return otiles;
}

bool
DynocMap::read(const boost::multi_array<char, 1>& data)
{
    return m_mapTree->read(data);
}

bool
DynocMap::write(boost::multi_array<char, 1>& data) const
{
    return m_mapTree->write(data);
}

bool
DynocMap::read(const dynocmap_msgs::DynocMap& msg)
{
    m_resolution = msg.resolution;
    m_tileTreeHeight = msg.tile_tree_height;
    m_mapTreeHeight = msg.map_tree_height;
    m_center(0) = msg.center_grid_x;
    m_center(1) = msg.center_grid_y;
    m_center(2) = msg.center_grid_z;

    int width = mapGridWidth();
    int nTiles = width * width * width;

    m_mapGrid.clear();
    m_mapGrid.resize(nTiles);

    for (int s = 0; s < width; ++s)
    {
        for (int r = 0; r < width; ++r)
        {
            for (int c = 0; c < width; ++c)
            {
                int index = s * width * width +
                            r * width +
                            c;

                m_mapGrid.at(index) = boost::make_shared<OcTree>();
                m_mapGrid.at(index)->read(msg.tiles.at(index));
            }
        }
    }

    m_mapGridOffset.setZero();

    m_mapTree = boost::make_shared<OcTree>(m_center, m_mapGrid);

    m_memoryCache.reset();

    return true;
}

bool
DynocMap::write(dynocmap_msgs::DynocMap& msg, const std::string& frameId) const
{
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frameId;

    msg.resolution = m_resolution;
    msg.tile_tree_height = m_tileTreeHeight;
    msg.map_tree_height = m_mapTreeHeight;
    msg.center_grid_x = m_center(0);
    msg.center_grid_y = m_center(1);
    msg.center_grid_z = m_center(2);

    int width = mapGridWidth();
    int nTiles = width * width * width;

    msg.tiles.resize(nTiles);

    for (int s = 0; s < width; ++s)
    {
        int s_c = s - m_mapGridOffset(2);
        while (s_c < 0)
        {
            s_c += width;
        }

        for (int r = 0; r < width; ++r)
        {
            int r_c = r - m_mapGridOffset(1);
            while (r_c < 0)
            {
                r_c += width;
            }

            for (int c = 0; c < width; ++c)
            {
                int c_c = c - m_mapGridOffset(0);
                while (c_c < 0)
                {
                    c_c += width;
                }

                m_mapGrid.at(s_c * width * width +
                             r_c * width +
                             c_c)->write(msg.tiles.at(s * width * width +
                                                      r * width +
                                                      c));
            }
        }
    }

    return true;
}

int
DynocMap::mapGridWidth(void) const
{
    return 1 << (m_mapTreeHeight - 1);
}

int
DynocMap::tileGridWidth(void) const
{
    return 1 << (m_tileTreeHeight - 1);
}

void
DynocMap::addRowNorth(void)
{
    int width = mapGridWidth();

    int r_offset = m_mapGridOffset(1);

    for (int c = 0; c < width; ++c)
    {
        for (int s = 0; s < width; ++s)
        {
            OcTreePtr& tile = m_mapGrid.at(s * width * width +
                                           r_offset * width +
                                           c);

            Eigen::Vector3i tileCenter;
            tileCenter = gridCoordsToTileCenter(Eigen::Vector3i(c, r_offset, s), true);

            unloadTile(tile, tileCenter);
        }
    }

    ++r_offset;
    if (r_offset == width)
    {
        r_offset = 0;
    }

    m_mapGridOffset(1) = r_offset;
    m_center(1) += tileGridWidth();
}

void
DynocMap::addRowSouth(void)
{
    int width = mapGridWidth();

    int r_offset = m_mapGridOffset(1);
    if (r_offset != 0)
    {
        --r_offset;
    }
    else
    {
        r_offset = width - 1;
    }

    for (int c = 0; c < width; ++c)
    {
        for (int s = 0; s < width; ++s)
        {
            OcTreePtr& tile = m_mapGrid.at(s * width * width +
                                           r_offset * width +
                                           c);

            Eigen::Vector3i tileCenter;
            tileCenter = gridCoordsToTileCenter(Eigen::Vector3i(c, r_offset, s), true);

            unloadTile(tile, tileCenter);
        }
    }

    m_mapGridOffset(1) = r_offset;
    m_center(1) -= tileGridWidth();
}

void
DynocMap::addColumnEast(void)
{
    int width = mapGridWidth();

    int c_offset = m_mapGridOffset(0);
    if (c_offset != 0)
    {
        --c_offset;
    }
    else
    {
        c_offset = width - 1;
    }

    for (int r = 0; r < width; ++r)
    {
        for (int s = 0; s < width; ++s)
        {
            OcTreePtr& tile = m_mapGrid.at(s * width * width +
                                           r * width +
                                           c_offset);

            Eigen::Vector3i tileCenter;
            tileCenter = gridCoordsToTileCenter(Eigen::Vector3i(c_offset, r, s), true);

            unloadTile(tile, tileCenter);
        }
    }

    m_mapGridOffset(0) = c_offset;
    m_center(0) -= tileGridWidth();
}

void
DynocMap::addColumnWest(void)
{
    int width = mapGridWidth();

    int c_offset = m_mapGridOffset(0);

    for (int r = 0; r < width; ++r)
    {
        for (int s = 0; s < width; ++s)
        {
            OcTreePtr& tile = m_mapGrid.at(s * width * width +
                                           r * width +
                                           c_offset);

            Eigen::Vector3i tileCenter;
            tileCenter = gridCoordsToTileCenter(Eigen::Vector3i(c_offset, r, s), true);

            unloadTile(tile, tileCenter);
        }
    }

    ++c_offset;
    if (c_offset == width)
    {
        c_offset = 0;
    }

    m_mapGridOffset(0) = c_offset;
    m_center(0) += tileGridWidth();
}

void
DynocMap::addSliceUp(void)
{
    int width = mapGridWidth();

    int s_offset = m_mapGridOffset(2);

    for (int r = 0; r < width; ++r)
    {
        for (int c = 0; c < width; ++c)
        {
            OcTreePtr& tile = m_mapGrid.at(s_offset * width * width +
                                           r * width +
                                           c);

            Eigen::Vector3i tileCenter;
            tileCenter = gridCoordsToTileCenter(Eigen::Vector3i(c, r, s_offset), true);

            unloadTile(tile, tileCenter);
        }
    }

    ++s_offset;
    if (s_offset == width)
    {
        s_offset = 0;
    }

    m_mapGridOffset(2) = s_offset;
    m_center(2) += tileGridWidth();
}

void
DynocMap::addSliceDown(void)
{
    int width = mapGridWidth();

    double s_offset = m_mapGridOffset(2);
    if (s_offset != 0)
    {
        --s_offset;
    }
    else
    {
        s_offset = width - 1;
    }

    for (int r = 0; r < width; ++r)
    {
        for (int c = 0; c < width; ++c)
        {
            OcTreePtr& tile = m_mapGrid.at(s_offset * width * width +
                                           r * width +
                                           c);

            Eigen::Vector3i tileCenter;
            tileCenter = gridCoordsToTileCenter(Eigen::Vector3i(c, r, s_offset), true);

            unloadTile(tile, tileCenter);
        }
    }

    m_mapGridOffset(2) = s_offset;
    m_center(2) -= tileGridWidth();
}

void
DynocMap::fillEmptyTiles(void)
{
    int width = mapGridWidth();
    int nTiles = width * width * width;

    for (int i = 0; i < nTiles; ++i)
    {
        if (m_mapGrid.at(i))
        {
            continue;
        }

        int c = i % width;
        int r = (i / width) % width;
        int s = i / (width * width);

        Eigen::Vector3i tileCenter;
        tileCenter = gridCoordsToTileCenter(Eigen::Vector3i(c, r, s), true);

        loadTile(m_mapGrid.at(i), tileCenter);
    }
}

void
DynocMap::buildMapTree(void)
{
    m_mapTree = boost::make_shared<OcTree>(m_center, m_mapGrid);
}

void
DynocMap::prefetch(void)
{
    int cacheTreeHeight = m_mapTreeHeight + 1;
    int cacheWidth =  1 << (cacheTreeHeight - 1);

    int mapWidth = mapGridWidth();
    int borderWidth = (cacheWidth - mapWidth) / 2;

    int start = -borderWidth;
    int end = mapWidth + borderWidth;
    for (int i = start; i < end; ++i)
    {
        if (i >= 0 && i < mapWidth)
        {
            continue;
        }

        for (int j = start; j < end; ++j)
        {
            if (j >= 0 && j < mapWidth)
            {
                continue;
            }

            for (int k = start; k < end; ++k)
            {
                if (k >= 0 && k < mapWidth)
                {
                    continue;
                }

                Eigen::Vector3i tileCenter;
                tileCenter = gridCoordsToTileCenter(Eigen::Vector3i(i, j, k), false);

                std::string tileFilename;
                tileFilename = getTileFilename(m_resolution, m_tileTreeHeight,
                                               tileCenter, m_diskCacheDir);

                m_memoryCache->lookup(tileFilename);
            }
        }
    }
}

std::string
DynocMap::getTileFilename(double resolution, int treeHeight,
                          const Eigen::Vector3i& tileCenter,
                          const std::string& cacheDir) const
{
    std::ostringstream oss;
    oss << cacheDir;
    if (cacheDir.at(cacheDir.size() - 1) != '/')
    {
        oss << "/";
    }
    oss.setf(std::ios::fixed, std::ios::floatfield);
    oss.precision(3);
    oss << "node_" << resolution << "_" << treeHeight << "_"
        << tileCenter(0) << "_" << tileCenter(1) << "_" << tileCenter(2) << ".dat";

    return oss.str();
}

Eigen::Vector3i
DynocMap::gridCoordsToTileCenter(const Eigen::Vector3i& p,
                                 bool addOffset) const
{
    int width = mapGridWidth();

    if (width == 1)
    {
        return m_center;
    }

    int width_2 = width / 2;

    int s = p(2);
    int r = p(1);
    int c = p(0);

    if (addOffset)
    {
        s -= m_mapGridOffset(2);
        while (s < 0)
        {
            s += width;
        }

        r -= m_mapGridOffset(1);
        while (r < 0)
        {
            r += width;
        }

        c -= m_mapGridOffset(0);
        while (c < 0)
        {
            c += width;
        }
    }

    Eigen::Vector3i tileCenter;
    tileCenter(0) = (c - width_2) * tileGridWidth() + tileGridWidth() / 2;
    tileCenter(1) = (r - width_2) * tileGridWidth() + tileGridWidth() / 2;
    tileCenter(2) = (s - width_2) * tileGridWidth() + tileGridWidth() / 2;

    tileCenter += m_center;

    return tileCenter;
}

void
DynocMap::loadTile(OcTreePtr& tile,
                   const Eigen::Vector3i& tileCenterGridCoords) const
{
    std::string tileFilename;
    tileFilename = getTileFilename(m_resolution, m_tileTreeHeight,
                                   tileCenterGridCoords, m_diskCacheDir);

    if (!m_memoryCache)
    {
        if (!tile)
        {
            tile = boost::make_shared<OcTree>(m_resolution, m_tileTreeHeight,
                                              tileCenterGridCoords);
        }

        if (boost::filesystem::exists(tileFilename))
        {
            tile->read(tileFilename);
        }
    }
    else
    {
        tile = m_memoryCache->get(tileFilename);

        if (!tile)
        {
            tile = boost::make_shared<OcTree>(m_resolution, m_tileTreeHeight,
                                              tileCenterGridCoords);
        }
    }
}

void
DynocMap::unloadTile(OcTreePtr& tile,
                     const Eigen::Vector3i& tileCenterGridCoords) const
{
    if (!tile)
    {
        return;
    }

    std::string tileFilename;
    tileFilename = getTileFilename(m_resolution, m_tileTreeHeight,
                                   tileCenterGridCoords, m_diskCacheDir);

    m_memoryCache->cache(tileFilename, tile);

    tile.reset();
}

}
