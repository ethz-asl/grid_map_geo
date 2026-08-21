#include "grid_map_geo_rviz_plugin/quadtree_structure_display.hpp"

#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/status_property.hpp>

#include "grid_map_geo_rviz_plugin/quadtree_structure_visual.hpp"

namespace grid_map_geo_rviz_plugin {

QuadtreeStructureDisplay::QuadtreeStructureDisplay() {
  alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 1.0f, "Line transparency, 0 (invisible) to 1 (opaque).", this, SLOT(updateVisualization()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  elevation_offset_property_ = new rviz_common::properties::FloatProperty(
      "Elevation Offset", 0.05f,
      "Added to every cell's height, so the wireframe sits just above the terrain surface instead of "
      "z-fighting with it.",
      this, SLOT(updateVisualization()));

  min_size_color_property_ = new rviz_common::properties::ColorProperty(
      "Min Size Color", QColor(255, 64, 0), "Color for the smallest (finest-resolution) cells.", this,
      SLOT(updateVisualization()));

  max_size_color_property_ = new rviz_common::properties::ColorProperty(
      "Max Size Color", QColor(0, 128, 255), "Color for the largest (coarsest) cells.", this,
      SLOT(updateVisualization()));

  auto_compute_size_bounds_property_ = new rviz_common::properties::BoolProperty(
      "Auto-Compute Size Bounds", true,
      "Normalize each cell's color against the min/max cell size observed in the latest message, instead of "
      "Max Cell Size below.",
      this, SLOT(updateVisualization()));

  max_cell_size_property_ = new rviz_common::properties::FloatProperty(
      "Max Cell Size", 64.0f, "Cell size that maps to Max Size Color, when Auto-Compute Size Bounds is false.",
      this, SLOT(updateVisualization()));
  max_cell_size_property_->setMin(0.0f);

  show_outlines_property_ = new rviz_common::properties::BoolProperty(
      "Show Cell Outlines", true,
      "Draw a solid line around each cell's own boundary, so the multi-resolution structure stays visible "
      "even when neighboring cells happen to share a similar fill color.",
      this, SLOT(updateVisualization()));

  outline_color_property_ = new rviz_common::properties::ColorProperty(
      "Outline Color", QColor(0, 0, 0), "Color of the cell outlines, when Show Cell Outlines is true.", this,
      SLOT(updateVisualization()));
}

QuadtreeStructureDisplay::~QuadtreeStructureDisplay() = default;

void QuadtreeStructureDisplay::onInitialize() {
  MFDClass::onInitialize();
  visual_ = std::make_unique<QuadtreeStructureVisual>(scene_manager_, scene_node_);
}

void QuadtreeStructureDisplay::reset() {
  MFDClass::reset();
  last_msg_.reset();
  visual_ = std::make_unique<QuadtreeStructureVisual>(scene_manager_, scene_node_);
}

void QuadtreeStructureDisplay::updateVisualization() {
  if (last_msg_) {
    processMessage(last_msg_);
  }
}

void QuadtreeStructureDisplay::processMessage(grid_map_geo_msgs::msg::QuadtreeStructure::ConstSharedPtr msg) {
  last_msg_ = msg;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Transform",
              QString("Failed to transform from frame [%1] to fixed frame [%2]")
                  .arg(QString::fromStdString(msg->header.frame_id))
                  .arg(fixed_frame_));
    return;
  }
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);

  visual_->setMessage(*msg, alpha_property_->getFloat(), elevation_offset_property_->getFloat(),
                      min_size_color_property_->getOgreColor(), max_size_color_property_->getOgreColor(),
                      auto_compute_size_bounds_property_->getBool(), max_cell_size_property_->getFloat(),
                      show_outlines_property_->getBool(), outline_color_property_->getOgreColor());

  setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
}

}  // namespace grid_map_geo_rviz_plugin

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(grid_map_geo_rviz_plugin::QuadtreeStructureDisplay, rviz_common::Display)
