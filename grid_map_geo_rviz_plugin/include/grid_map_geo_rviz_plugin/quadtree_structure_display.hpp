#ifndef GRID_MAP_GEO_RVIZ_PLUGIN__QUADTREE_STRUCTURE_DISPLAY_HPP_
#define GRID_MAP_GEO_RVIZ_PLUGIN__QUADTREE_STRUCTURE_DISPLAY_HPP_

#include <memory>

#include <grid_map_geo_msgs/msg/quadtree_structure.hpp>
#include <rviz_common/message_filter_display.hpp>

namespace rviz_common::properties {
class ColorProperty;
class FloatProperty;
class BoolProperty;
}  // namespace rviz_common::properties

namespace grid_map_geo_rviz_plugin {

class QuadtreeStructureVisual;

/**
 * @brief RViz2 Display for grid_map_geo_msgs/QuadtreeStructure messages --
 * visualizes a grid_map_geo HashedWaveletQuadtree's actual multi-resolution
 * leaf cells (see QuadtreeStructureVisual), meant to be layered on top of
 * the corresponding grid_map_rviz_plugin/GridMap elevation surface.
 */
class QuadtreeStructureDisplay
    : public rviz_common::MessageFilterDisplay<grid_map_geo_msgs::msg::QuadtreeStructure> {
  Q_OBJECT

 public:
  QuadtreeStructureDisplay();
  ~QuadtreeStructureDisplay() override;

  void onInitialize() override;
  void reset() override;

 private Q_SLOTS:
  void updateVisualization();

 private:
  void processMessage(grid_map_geo_msgs::msg::QuadtreeStructure::ConstSharedPtr msg) override;

  std::unique_ptr<QuadtreeStructureVisual> visual_;

  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::FloatProperty* elevation_offset_property_;
  rviz_common::properties::ColorProperty* min_size_color_property_;
  rviz_common::properties::ColorProperty* max_size_color_property_;
  rviz_common::properties::BoolProperty* auto_compute_size_bounds_property_;
  rviz_common::properties::FloatProperty* max_cell_size_property_;

  grid_map_geo_msgs::msg::QuadtreeStructure::ConstSharedPtr last_msg_;
};

}  // namespace grid_map_geo_rviz_plugin

#endif  // GRID_MAP_GEO_RVIZ_PLUGIN__QUADTREE_STRUCTURE_DISPLAY_HPP_
