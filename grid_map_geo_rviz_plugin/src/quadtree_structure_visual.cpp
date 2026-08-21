#include "grid_map_geo_rviz_plugin/quadtree_structure_visual.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <string>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_rendering/material_manager.hpp>

namespace grid_map_geo_rviz_plugin {

namespace {
// Every instance needs its own uniquely-named Ogre material/object (Ogre's
// resource managers are name-keyed and global), including across
// destroy/recreate cycles (e.g. Display::reset()).
std::string nextInstanceName() {
  static std::atomic<uint64_t> counter{0};
  return "grid_map_geo_rviz_plugin/QuadtreeStructure" + std::to_string(counter++);
}
}  // namespace

QuadtreeStructureVisual::QuadtreeStructureVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager) {
  frame_node_ = parent_node->createChildSceneNode();

  const std::string name = nextInstanceName();
  // No setVertexColourTracking() call needed: with lighting disabled (set
  // by createMaterialWithNoLighting itself), per-vertex colour() already
  // becomes the fragment color directly -- matches the same pattern
  // grid_map_rviz_plugin's own GridMapVisual uses for its colored mesh.
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(name + "Material");

  manual_object_ = scene_manager_->createManualObject(name + "Object");
  frame_node_->attachObject(manual_object_);
}

QuadtreeStructureVisual::~QuadtreeStructureVisual() {
  scene_manager_->destroyManualObject(manual_object_);
  Ogre::MaterialManager::getSingleton().remove(material_);
  scene_manager_->destroySceneNode(frame_node_);
}

void QuadtreeStructureVisual::setMessage(const grid_map_geo_msgs::msg::QuadtreeStructure& msg, float alpha,
                                         float elevation_offset, const Ogre::ColourValue& min_size_color,
                                         const Ogre::ColourValue& max_size_color, bool auto_compute_size_bounds,
                                         float max_cell_size, bool show_outlines,
                                         const Ogre::ColourValue& outline_color) {
  manual_object_->clear();
  if (msg.cells.empty()) return;

  float min_size = 0.0f;
  float max_size = max_cell_size;
  if (auto_compute_size_bounds) {
    min_size = msg.cells.front().size;
    max_size = msg.cells.front().size;
    for (const auto& cell : msg.cells) {
      min_size = std::min(min_size, cell.size);
      max_size = std::max(max_size, cell.size);
    }
  }
  const float size_range = std::max(max_size - min_size, 1e-6f);

  rviz_rendering::MaterialManager::enableAlphaBlending(material_, alpha);

  // Filled quads (two triangles each), not a wireframe outline: with real
  // orthomosaic colors available (see cell.color below), a filled surface
  // reads as actual terrain texture, not just a structure diagram.
  //
  // The "rviz_rendering" resource group argument is required: material_ was
  // created via MaterialManager::createMaterialWithNoLighting(), which
  // creates it specifically under that resource group (see
  // grid_map_rviz_plugin's own GridMapVisual, which does the same
  // three-argument begin() for exactly this reason). Omitting it makes
  // Ogre look for the material in the default resource group instead,
  // fail to find it there, and silently fall back to a flat grey
  // placeholder material -- which renders fine geometrically but ignores
  // per-vertex colour() entirely, exactly the "everything is grey"
  // symptom this was fixing.
  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  for (const auto& cell : msg.cells) {
    Ogre::ColourValue color;
    if (cell.color.a > 0.0f) {
      // Real orthomosaic sample: respect the display's own Alpha property
      // for transparency control rather than the message's own alpha
      // (which just marks "color present", see QuadtreeCell.msg).
      color = Ogre::ColourValue(cell.color.r, cell.color.g, cell.color.b, alpha);
    } else {
      const float t = std::clamp((cell.size - min_size) / size_range, 0.0f, 1.0f);
      color = min_size_color * (1.0f - t) + max_size_color * t;
      color.a = alpha;
    }

    const float x0 = cell.min_corner.x;
    const float y0 = cell.min_corner.y;
    const float x1 = x0 + cell.size;
    const float y1 = y0 + cell.size;
    const float z = cell.elevation + elevation_offset;

    const std::array<Ogre::Vector3, 4> corners = {Ogre::Vector3(x0, y0, z), Ogre::Vector3(x1, y0, z),
                                                   Ogre::Vector3(x1, y1, z), Ogre::Vector3(x0, y1, z)};
    constexpr std::array<int, 6> kTriangleIndices = {0, 1, 2, 0, 2, 3};
    for (int index : kTriangleIndices) {
      manual_object_->position(corners[index]);
      manual_object_->colour(color);
    }
  }
  manual_object_->end();

  // Separate OT_LINE_LIST section (a ManualObject can't mix primitive types
  // within one begin()/end()) tracing each cell's own 4 edges, drawn a hair
  // above the fill (kOutlineZBias) to avoid z-fighting -- without this, a
  // solid fill alone makes adjacent same-size/same-color cells (e.g. a
  // large uniformly-colored merged region, or same-size cells sharing a
  // similar orthomosaic sample) visually indistinguishable from one another,
  // hiding the very multi-resolution structure this display exists to show.
  if (show_outlines) {
    constexpr float kOutlineZBias = 0.01f;
    Ogre::ColourValue line_color = outline_color;
    line_color.a = alpha;
    manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_LINE_LIST, "rviz_rendering");
    for (const auto& cell : msg.cells) {
      const float x0 = cell.min_corner.x;
      const float y0 = cell.min_corner.y;
      const float x1 = x0 + cell.size;
      const float y1 = y0 + cell.size;
      const float z = cell.elevation + elevation_offset + kOutlineZBias;

      const std::array<Ogre::Vector3, 4> corners = {Ogre::Vector3(x0, y0, z), Ogre::Vector3(x1, y0, z),
                                                     Ogre::Vector3(x1, y1, z), Ogre::Vector3(x0, y1, z)};
      for (int i = 0; i < 4; ++i) {
        manual_object_->position(corners[i]);
        manual_object_->colour(line_color);
        manual_object_->position(corners[(i + 1) % 4]);
        manual_object_->colour(line_color);
      }
    }
    manual_object_->end();
  }
}

void QuadtreeStructureVisual::setFramePosition(const Ogre::Vector3& position) { frame_node_->setPosition(position); }

void QuadtreeStructureVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

}  // namespace grid_map_geo_rviz_plugin
