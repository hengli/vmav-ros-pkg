#include "rviz_dynocmap_plugin/dynocmap_visual.h"

#include "cauldron/cauldron.h"
#include "dynocmap/DynocMap.h"

namespace rviz
{

DynocMapVisual::DynocMapVisual(Ogre::SceneManager* scene_manager,
                               Ogre::SceneNode* parent_node)
 : m_color(0, 0, 255)
 , m_alpha(1.0f)
 , m_coloring_mode(DYNOCMAP_FLAT_COLOR)
 , m_enabled(false)
 , m_showBoundary(false)
{
    m_scene_manager = scene_manager;

    m_map_node = parent_node->createChildSceneNode();
}

DynocMapVisual::~DynocMapVisual()
{
    for (TileMap::iterator it = m_tileMap.begin(); it != m_tileMap.end(); ++it)
    {
        TileItem& item = it->second;

        if (m_showBoundary)
        {
            destroyObject(item.boundaryObject);
        }
        destroyObject(item.tileObject);
    }

    m_scene_manager->destroySceneNode(m_map_node);
}

void
DynocMapVisual::reset(void)
{
    for (TileMap::iterator it = m_tileMap.begin(); it != m_tileMap.end(); ++it)
    {
        TileItem& item = it->second;

        if (m_showBoundary)
        {
            destroyObject(item.boundaryObject);
        }
        destroyObject(item.tileObject);
    }

    m_tileMap.clear();
}

void
DynocMapVisual::setMessage(const dynocmap_msgs::DynocMap::ConstPtr& msg)
{
    if (!m_enabled)
    {
        return;
    }

    px::DynocMap map;
    if (!map.read(*msg))
    {
        return;
    }

    for (TileMap::iterator it = m_tileMap.begin(); it != m_tileMap.end(); ++it)
    {
        it->second.dynamic = false;
    }

    double resolution = map.resolution();

    std::vector<px::OccupancyTile> tiles = map.tiles();

    for (size_t i = 0; i < tiles.size(); ++i)
    {
        const px::OccupancyTile& tile = tiles.at(i);

        TileMap::iterator it = m_tileMap.find(tile.tileGridCenter());
        if (it == m_tileMap.end())
        {
            TileItem item;
            if (m_showBoundary)
            {
                item.boundaryObject = createBoundaryObject(tile.tileGridCenter(),
                                                           tile.tileGridWidth(),
                                                           resolution,
                                                           true);
            }
            item.tileData = boost::make_shared<px::OccupancyTile>();
            item.tileObject = createTileObject("tile-" + toString(tile.tileGridCenter()), true);

            std::pair<TileMap::iterator, bool> ret = m_tileMap.insert(std::make_pair(tile.tileGridCenter(), item));
            it = ret.first;
        }
        else
        {
            TileItem& item = it->second;

            if (!item.dynamic)
            {
                destroyObject(item.boundaryObject);
                destroyObject(item.tileObject);
                if (m_showBoundary)
                {
                    item.boundaryObject = createBoundaryObject(item.tileData->tileGridCenter(),
                                                               item.tileData->tileGridWidth(),
                                                               resolution,
                                                               true);
                }
                item.tileObject = createTileObject("tile-" + toString(tile.tileGridCenter()), true);
            }
        }

        TileItem& item = it->second;
        item.dynamic = true;
        *(item.tileData) = tile;

        drawTileData(item);
    }

    for (TileMap::iterator it = m_tileMap.begin(); it != m_tileMap.end(); ++it)
    {
        TileItem& item = it->second;

        if (item.dynamic != item.tileObject->getDynamic())
        {
            destroyObject(item.boundaryObject);
            destroyObject(item.tileObject);
            if (m_showBoundary)
            {
                item.boundaryObject = createBoundaryObject(item.tileData->tileGridCenter(),
                                                           item.tileData->tileGridWidth(),
                                                           resolution,
                                                           false);
            }
            item.tileObject = createTileObject("tile-" + toString(item.tileData->tileGridCenter()), false);

            drawTileData(item);
        }
    }
}

void
DynocMapVisual::setColor(const QColor& color, float alpha)
{
    m_color = color;
    m_alpha = alpha;
}

void
DynocMapVisual::setColoringMode(DynocMapColoringMode mode)
{
    m_coloring_mode = mode;
}

void
DynocMapVisual::setEnabled(bool enable)
{
    m_enabled = enable;

    for (TileMap::iterator it = m_tileMap.begin(); it != m_tileMap.end(); ++it)
    {
        Ogre::ManualObject* boundaryObject = it->second.boundaryObject;
        Ogre::ManualObject* tileObject = it->second.tileObject;

        if (enable)
        {
            if (m_showBoundary)
            {
                m_map_node->attachObject(boundaryObject);
            }
            m_map_node->attachObject(tileObject);
        }
        else
        {
            if (m_showBoundary)
            {
                m_map_node->detachObject(boundaryObject);
            }
            m_map_node->detachObject(tileObject);
        }
    }
}

Ogre::ManualObject*
DynocMapVisual::createBoundaryObject(const Eigen::Vector3i& center,
                                     int width,
                                     double resolution,
                                     bool dynamic) const
{
    std::ostringstream oss;
    oss << "boundary-" + toString(center);

    Ogre::ManualObject* object = m_scene_manager->createManualObject(oss.str());

    Eigen::Vector3d c1 = (center - Eigen::Vector3i::Constant(width / 2)).cast<double>() * resolution;
    Eigen::Vector3d c2 = (center + Eigen::Vector3i::Constant(width / 2)).cast<double>() * resolution;

    object->setDynamic(false);
    object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    if (dynamic)
    {
        object->colour(1.0f, 0.0f, 0.0f, 0.5f);
    }
    else
    {
        object->colour(0.0f, 0.0f, 1.0f, 0.5f);
    }
    object->position(c1(0), c1(1), c1(2));
    object->position(c1(0), c2(1), c1(2));
    object->position(c2(0), c2(1), c1(2));
    object->position(c2(0), c1(1), c1(2));
    object->position(c1(0), c1(1), c2(2));
    object->position(c1(0), c2(1), c2(2));
    object->position(c2(0), c2(1), c2(2));
    object->position(c2(0), c1(1), c2(2));
    object->index(0); object->index(1);
    object->index(1); object->index(2);
    object->index(2); object->index(3);
    object->index(0); object->index(3);
    object->index(4); object->index(5);
    object->index(5); object->index(6);
    object->index(6); object->index(7);
    object->index(4); object->index(7);
    object->index(0); object->index(4);
    object->index(1); object->index(5);
    object->index(2); object->index(6);
    object->index(3); object->index(7);
    object->end();

    m_map_node->attachObject(object);

    return object;
}

Ogre::ManualObject*
DynocMapVisual::createTileObject(const std::string& name, bool dynamic) const
{
    Ogre::ManualObject* object = m_scene_manager->createManualObject(name);

    object->setDynamic(dynamic);
    if (dynamic)
    {
        // Create invisible triangle. Otherwise, the resulting section will
        // be discarded.

        object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        object->position(0.0, 0.0, 0.0);
        object->colour(0.0f, 0.0f, 0.0f, 0.0f);
        object->index(0);
        object->index(0);
        object->index(0);
        object->end();
    }

    m_map_node->attachObject(object);

    return object;
}

void
DynocMapVisual::destroyObject(Ogre::ManualObject* object)
{
    m_map_node->detachObject(object);
    m_scene_manager->destroyManualObject(object);
}

std::string
DynocMapVisual::toString(const Eigen::Vector3i& center) const
{
    std::ostringstream oss;
    oss << center(0) << "-" << center(1) << "-" << center(2);

    return oss.str();
}

void
DynocMapVisual::drawTileData(TileItem& item)
{
    double resolution = item.tileData->resolution();
    const std::vector<px::OccupancyCell>& obstacles = item.tileData->obstacles();

    if (item.dynamic)
    {
        int z_min = std::numeric_limits<int>::max();
        int z_max = std::numeric_limits<int>::min();

        for (size_t i = 0; i < obstacles.size(); ++i)
        {
            const px::OccupancyCell& obstacle = obstacles.at(i);
            const Eigen::Vector3i& coords = obstacle.coords;
            int width = obstacle.width;

            if (coords(2) < z_min)
            {
                z_min = coords(2);
            }
            if (coords(2) + width - 1 > z_max)
            {
                z_max = coords(2) + width - 1;
            }
        }

        item.z_min = z_min;
        item.z_max = z_max;
    }

    Ogre::ManualObject* object = item.tileObject;

    object->estimateVertexCount(obstacles.size() * 8);
    object->estimateIndexCount(obstacles.size() * 36);

    if (item.dynamic)
    {
        object->beginUpdate(0);
    }
    else
    {
        object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }

    size_t nVertices = 0;
    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        const px::OccupancyCell& obstacle = obstacles.at(i);
        const Eigen::Vector3i& coords = obstacle.coords;
        int width = obstacle.width;

        Ogre::ColourValue color;
        if (m_coloring_mode == DYNOCMAP_FLAT_COLOR)
        {
            color = Ogre::ColourValue(m_color.redF(), m_color.greenF(), m_color.blueF(), m_alpha);
        }
        else
        {
            unsigned char idx = static_cast<double>(coords(2) + width / 2 - item.z_min) / static_cast<double>(item.z_max - item.z_min) * 127.0;

            float r, g, b;
            px::colormap("jet", idx, r, g, b);

            color = Ogre::ColourValue(r, g, b, m_alpha);
        }

        Eigen::Vector3d c1 = coords.cast<double>() * resolution;
        Eigen::Vector3d c2 = (coords + Eigen::Vector3i::Constant(width)).cast<double>() * resolution;

        object->position(c2(0), c2(1), c2(2)); // v0
        object->colour(color);
        object->position(c1(0), c2(1), c2(2)); // v1
        object->colour(color);
        object->position(c1(0), c1(1), c2(2)); // v2
        object->colour(color);
        object->position(c2(0), c1(1), c2(2)); // v3
        object->colour(color);
        object->position(c2(0), c1(1), c1(2)); // v4
        object->colour(color);
        object->position(c2(0), c2(1), c1(2)); // v5
        object->colour(color);
        object->position(c1(0), c2(1), c1(2)); // v6
        object->colour(color);
        object->position(c1(0), c1(1), c1(2)); // v7
        object->colour(color);

        // front
        object->index(nVertices + 0);
        object->index(nVertices + 1);
        object->index(nVertices + 2);
        object->index(nVertices + 2);
        object->index(nVertices + 3);
        object->index(nVertices + 0);

        // right
        object->index(nVertices + 0);
        object->index(nVertices + 3);
        object->index(nVertices + 4);
        object->index(nVertices + 4);
        object->index(nVertices + 5);
        object->index(nVertices + 0);

        // top
        object->index(nVertices + 0);
        object->index(nVertices + 5);
        object->index(nVertices + 6);
        object->index(nVertices + 6);
        object->index(nVertices + 1);
        object->index(nVertices + 0);

        // left
        object->index(nVertices + 1);
        object->index(nVertices + 6);
        object->index(nVertices + 7);
        object->index(nVertices + 7);
        object->index(nVertices + 2);
        object->index(nVertices + 1);

        // bottom
        object->index(nVertices + 7);
        object->index(nVertices + 4);
        object->index(nVertices + 3);
        object->index(nVertices + 3);
        object->index(nVertices + 2);
        object->index(nVertices + 7);

        // back
        object->index(nVertices + 4);
        object->index(nVertices + 7);
        object->index(nVertices + 6);
        object->index(nVertices + 6);
        object->index(nVertices + 5);
        object->index(nVertices + 4);

        nVertices += 8;
    }

    object->end();
}

} // end namespace rviz
