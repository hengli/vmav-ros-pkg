#include "rviz_dynocmap_plugin/dynocmap_display.h"

#include <rviz/properties/status_property.h>

namespace rviz
{

DynocMapDisplay::DynocMapDisplay()
 : m_scene_node(0)
 , m_messages_received(0)
{
    createProperties();
}

DynocMapDisplay::~DynocMapDisplay()
{

}

void
DynocMapDisplay::onInitialize(void)
{
    MFDClass::onInitialize();

    // Make an Ogre::SceneNode to contain all our visuals.
    m_scene_node = scene_manager_->getRootSceneNode()->createChildSceneNode();

    m_map_visual = new DynocMapVisual(context_->getSceneManager(), m_scene_node);
}

void
DynocMapDisplay::reset(void)
{
    MFDClass::reset();
    m_messages_received = 0;
    setStatus(rviz::StatusProperty::Warn, "Topic", "No messages received");

    m_map_visual->reset();
}

void
DynocMapDisplay::createProperties(void)
{
    m_map_color_property = new rviz::ColorProperty("Color",
                                                   Qt::blue,
                                                   "Color for the occupied voxels.",
                                                   this,
                                                   SLOT(updateMap()));

    m_map_alpha_property = new rviz::FloatProperty("Alpha",
                                                   1.0f,
                                                   "0.0 is fully transparent, 1.0 is fully opaque.",
                                                   this,
                                                   SLOT(updateMap()));

    m_map_coloring_property = new rviz::EnumProperty("Coloring Mode",
                                                     "Flat Color",
                                                     "Select a coloring mode for the map.",
                                                     this,
                                                     SLOT(updateMap()));

    m_map_coloring_property->addOption("Flat Color", DYNOCMAP_FLAT_COLOR);
    m_map_coloring_property->addOption("Z-Axis", DYNOCMAP_Z_AXIS_COLOR);
}

void
DynocMapDisplay::onEnable(void)
{
    MFDClass::onEnable();

    m_map_visual->setEnabled(true);
}

void
DynocMapDisplay::onDisable(void)
{
    MFDClass::onDisable();

    m_map_visual->setEnabled(false);
}

void
DynocMapDisplay::updateMap(void)
{
    m_map_visual->setColor(m_map_color_property->getColor(), m_map_alpha_property->getFloat());
    m_map_visual->setColoringMode(static_cast<DynocMapColoringMode>(m_map_coloring_property->getOptionInt()));
}

void
DynocMapDisplay::processMessage(const dynocmap_msgs::DynocMap::ConstPtr& msg)
{
    ++m_messages_received;

    std::stringstream ss;
    ss << m_messages_received << " messages received";
    setStatus(rviz::StatusProperty::Ok, "Topic", ss.str().c_str());

    m_map_visual->setMessage(msg);

    context_->queueRender();
}

} // end namespace rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::DynocMapDisplay, rviz::Display)
