#include "grid_map_geo_rviz_plugin/quadtree_structure_visual.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <string>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
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
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(name + "Material");
  // ManualObject::colour() per-vertex calls only affect rendering if the
  // material is set to track vertex color for ambient/diffuse.
  material_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT | Ogre::TVC_DIFFUSE);

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
                                         float max_cell_size) {
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

  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_LINE_LIST);
  for (const auto& cell : msg.cells) {
    const float t = std::clamp((cell.size - min_size) / size_range, 0.0f, 1.0f);
    Ogre::ColourValue color = min_size_color * (1.0f - t) + max_size_color * t;
    color.a = alpha;

    const float x0 = cell.min_corner.x;
    const float y0 = cell.min_corner.y;
    const float x1 = x0 + cell.size;
    const float y1 = y0 + cell.size;
    const float z = cell.elevation + elevation_offset;

    const std::array<Ogre::Vector3, 4> corners = {Ogre::Vector3(x0, y0, z), Ogre::Vector3(x1, y0, z),
                                                   Ogre::Vector3(x1, y1, z), Ogre::Vector3(x0, y1, z)};
    for (int i = 0; i < 4; ++i) {
      manual_object_->position(corners[i]);
      manual_object_->colour(color);
      manual_object_->position(corners[(i + 1) % 4]);
      manual_object_->colour(color);
    }
  }
  manual_object_->end();
}

void QuadtreeStructureVisual::setFramePosition(const Ogre::Vector3& position) { frame_node_->setPosition(position); }

void QuadtreeStructureVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

}  // namespace grid_map_geo_rviz_plugin
