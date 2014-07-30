#ifndef RVIZ_DYNOCMAP_PLUGIN_DYNOCMAP_DISPLAY_H
#define RVIZ_DYNOCMAP_PLUGIN_DYNOCMAP_DISPLAY_H

#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>

#include "dynocmap_msgs/DynocMap.h"
#include "rviz_dynocmap_plugin/dynocmap_visual.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{

class DynocMapDisplay: public rviz::MessageFilterDisplay<dynocmap_msgs::DynocMap>
{
    Q_OBJECT
public:

    DynocMapDisplay();
    virtual ~DynocMapDisplay();

    virtual void onInitialize(void);
    virtual void reset(void);
    virtual void createProperties(void);

    void setTopic(const std::string& topic);
    const std::string& getTopic(void) { return m_topic; }

protected:
    virtual void onEnable(void);
    virtual void onDisable(void);

protected Q_SLOTS:
    void updateMap(void);

private:
    // Property objects for user-editable properties.
    rviz::ColorProperty* m_map_color_property;
    rviz::FloatProperty* m_map_alpha_property;
    rviz::EnumProperty* m_map_coloring_property;

    // Different types of visuals
    DynocMapVisual* m_map_visual;

    // User-editable property variables.
    std::string m_topic;

    // A node in the Ogre scene tree to be the parent of all our visuals.
    Ogre::SceneNode* m_scene_node;

    int m_messages_received;

    // Function to handle an incoming ROS message.
    void processMessage(const dynocmap_msgs::DynocMap::ConstPtr& msg);
};

} // end namespace rviz

#endif
