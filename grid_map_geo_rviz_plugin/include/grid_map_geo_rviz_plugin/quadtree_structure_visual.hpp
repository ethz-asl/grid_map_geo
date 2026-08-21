#ifndef GRID_MAP_GEO_RVIZ_PLUGIN__QUADTREE_STRUCTURE_VISUAL_HPP_
#define GRID_MAP_GEO_RVIZ_PLUGIN__QUADTREE_STRUCTURE_VISUAL_HPP_

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreQuaternion.h>
#include <OgreSharedPtr.h>
#include <OgreVector3.h>
#include <grid_map_geo_msgs/msg/quadtree_structure.hpp>

namespace Ogre {
class SceneManager;
class SceneNode;
class ManualObject;
}  // namespace Ogre

namespace grid_map_geo_rviz_plugin {

/**
 * @brief Draws one filled, flat quad per cell of a
 * grid_map_geo_msgs::msg::QuadtreeStructure message, at that cell's own
 * elevation (plus a small configurable Z offset to avoid z-fighting with
 * the terrain surface it's meant to sit on top of) -- unlike a per-block
 * "column" spanning an arbitrary shared vertical range, this directly shows
 * the tree's actual multi-resolution structure: large quads over
 * locally-uniform terrain, small quads wherever full detail is kept. Each
 * cell is colored by its own orthomosaic sample when the publisher provided
 * one (QuadtreeCell::color, see that message), falling back to a size
 * gradient (small/fine = one color, large/coarse = the other) otherwise --
 * either way giving a direct visual signal of the tree's structure. A solid
 * outline around each cell's own boundary (optional, see setMessage) keeps
 * that structure visible even when neighboring cells end up a similar color.
 *
 * Uses a plain Ogre::ManualObject (OT_TRIANGLE_LIST) rather than
 * rviz_rendering::BillboardLine: BillboardLine packs many independent
 * lines into a fixed-capacity chain-container scheme sized from
 * max_points_per_line * num_lines up front, and real terrain easily
 * produces tens of thousands of cells (uncompressible/rugged terrain
 * barely compresses at all -- see HashedWaveletQuadtree's own docs), which
 * hit an "Exceeded max_points_per_line limit" crash in practice. A plain
 * triangle list has no such per-line bookkeeping or up-front capacity to
 * get wrong.
 */
class QuadtreeStructureVisual {
 public:
  QuadtreeStructureVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~QuadtreeStructureVisual();

  QuadtreeStructureVisual(const QuadtreeStructureVisual&) = delete;
  QuadtreeStructureVisual& operator=(const QuadtreeStructureVisual&) = delete;

  /**
   * @brief Rebuild the filled cells from `msg`.
   *
   * @param alpha cell transparency, 0 (invisible) to 1 (opaque). Applied
   * even when using a cell's own orthomosaic color (see setMessage's .cpp).
   * @param elevation_offset added to every cell's Z, so the surface sits
   * just above the terrain mesh instead of z-fighting with it.
   * @param min_size_color color for the smallest (finest) cells.
   * @param max_size_color color for the largest (coarsest) cells.
   * @param auto_compute_size_bounds if true, normalize each cell's color
   * against the min/max cell size observed in `msg` itself; if false,
   * normalize against [0, max_cell_size] (the message doesn't carry the
   * tree's own finest resolution, so there's no fixed lower bound to use).
   * @param max_cell_size fixed normalization max, used only when
   * auto_compute_size_bounds is false.
   * @param show_outlines if true, draw a solid line around each cell's own
   * boundary on top of the fill, so the multi-resolution structure stays
   * visible even where neighboring cells happen to share a similar color.
   * @param outline_color color of those outlines.
   */
  void setMessage(const grid_map_geo_msgs::msg::QuadtreeStructure& msg, float alpha, float elevation_offset,
                  const Ogre::ColourValue& min_size_color, const Ogre::ColourValue& max_size_color,
                  bool auto_compute_size_bounds, float max_cell_size, bool show_outlines,
                  const Ogre::ColourValue& outline_color);

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private:
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* frame_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::MaterialPtr material_;
};

}  // namespace grid_map_geo_rviz_plugin

#endif  // GRID_MAP_GEO_RVIZ_PLUGIN__QUADTREE_STRUCTURE_VISUAL_HPP_
