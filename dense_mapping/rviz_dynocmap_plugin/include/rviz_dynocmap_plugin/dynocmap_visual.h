#ifndef RVIZ_DYNOCMAP_PLUGIN_DYNOCMAP_VISUAL_H
#define RVIZ_DYNOCMAP_PLUGIN_DYNOCMAP_VISUAL_H

#include <boost/unordered_map.hpp>
#include <Eigen/Dense>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <QColor>

#include "cauldron/EigenUtils.h"
#include "dynocmap_msgs/DynocMap.h"

// forward declaration
namespace px
{
class OccupancyTile;
}

namespace rviz
{

enum DynocMapColoringMode
{
    DYNOCMAP_FLAT_COLOR,
    DYNOCMAP_Z_AXIS_COLOR
};

class DynocMapVisual
{
public:
    DynocMapVisual(Ogre::SceneManager* scene_manager,
                   Ogre::SceneNode* parent_node);

    virtual ~DynocMapVisual();

    void reset(void);

    void setMessage(const dynocmap_msgs::DynocMap::ConstPtr& msg);

    void setColor(const QColor& color, float alpha);
    void setColoringMode(DynocMapColoringMode mode);

    const QColor& getColor(void) { return m_color; }
    DynocMapColoringMode getColoringMode(void) { return m_coloring_mode; }

    void setEnabled(bool enable);

private:
    Ogre::ManualObject* createBoundaryObject(const Eigen::Vector3i& center,
                                             int width,
                                             double resolution,
                                             bool dynamic) const;
    Ogre::ManualObject* createTileObject(const std::string& name, bool dynamic) const;
    void destroyObject(Ogre::ManualObject* object);
    std::string toString(const Eigen::Vector3i& center) const;

    class TileItem
    {
    public:
        TileItem()
         : boundaryObject(0)
         , dynamic(true)
         , tileObject(0)
         , z_min(std::numeric_limits<int>::max())
         , z_max(std::numeric_limits<int>::min())
        {

        }

        Ogre::ManualObject* boundaryObject;
        bool dynamic;
        boost::shared_ptr<px::OccupancyTile> tileData;
        Ogre::ManualObject* tileObject;
        int z_min;
        int z_max;
    };

    void drawTileData(TileItem& item);

    typedef boost::unordered_map<Eigen::Vector3i,
                                 TileItem,
                                 px::Vector3i_hash,
                                 px::Vector3i_equal_to,
                                 Eigen::aligned_allocator<Eigen::Vector3i> > TileMap;
    TileMap m_tileMap;

    QColor m_color;
    float m_alpha;
    DynocMapColoringMode m_coloring_mode;
    bool m_enabled;
    bool m_showBoundary;

    Ogre::SceneNode* m_map_node;
    Ogre::SceneManager* m_scene_manager;
};

} // end namespace rviz

#endif
