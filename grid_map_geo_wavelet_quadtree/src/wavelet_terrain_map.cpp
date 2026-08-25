#include "grid_map_geo_wavelet_quadtree/wavelet_terrain_map.hpp"

#include <fstream>

namespace {
// The compression error bound is a property of the store, recorded as an
// optional 5th field of extent.txt (see generate_wavelet_quadtree). Read it
// independently of whether extent.txt is also used for center/extent, since
// callers of the bounded-region overload below still need it. Missing file
// or missing 5th field (an older store) both mean lossless, i.e. 0.0.
double readCompressionErrorBound(const std::string &tile_store_dir) {
  std::ifstream extent_file(tile_store_dir + "/extent.txt");
  double center_x = 0.0, center_y = 0.0, extent_x = 0.0, extent_y = 0.0, error_bound = 0.0;
  extent_file >> center_x >> center_y >> extent_x >> extent_y >> error_bound;
  return error_bound;
}
}  // namespace

bool WaveletTerrainMap::LoadFromWaveletQuadtree(const std::string &tile_store_dir, const Eigen::Vector2d &center,
                                                const grid_map::Length &extent, int query_height) {
  compression_error_bound_ = readCompressionErrorBound(tile_store_dir);

  auto elevation_tree = std::make_shared<HashedWaveletQuadtree>();
  if (!elevation_tree->loadFromFile(tile_store_dir + "/elevation.wavelet_quadtree")) {
    return false;
  }
  auto variance_tree = std::make_shared<HashedWaveletQuadtree>();
  if (!variance_tree->loadFromFile(tile_store_dir + "/variance.wavelet_quadtree")) {
    return false;
  }

  const double resolution = elevation_tree->resolutionAtHeight(query_height);
  // Kept locally centered at (0, 0), matching initializeFromGeotiff()'s
  // convention -- the real-world (world-frame) origin is stored separately
  // in maporigin_, not baked into grid_map_'s own frame.
  grid_map_.setGeometry(extent, resolution, Eigen::Vector2d::Zero());
  grid_map_.setFrameId(frame_id_);
  grid_map_.add("elevation");
  grid_map_.add("elevation_variance");

  elevation_tree_ = elevation_tree;
  variance_tree_ = variance_tree;
  wavelet_store_dir_ = tile_store_dir;

  maporigin_.position = Eigen::Vector3d(center.x(), center.y(), 0.0);
  coordinate_name_ = "wavelet_quadtree";

  elevation_tree_->reconstructRegion(grid_map_, "elevation", query_height, center);
  variance_tree_->reconstructRegion(grid_map_, "elevation_variance", query_height, center);
  return true;
}

bool WaveletTerrainMap::LoadFromWaveletQuadtree(const std::string &tile_store_dir, int query_height) {
  std::ifstream extent_file(tile_store_dir + "/extent.txt");
  if (!extent_file) return false;
  double center_x = 0.0, center_y = 0.0, extent_x = 0.0, extent_y = 0.0;
  extent_file >> center_x >> center_y >> extent_x >> extent_y;
  if (!extent_file) return false;
  return LoadFromWaveletQuadtree(tile_store_dir, Eigen::Vector2d(center_x, center_y),
                                 grid_map::Length(extent_x, extent_y), query_height);
}

bool WaveletTerrainMap::updateElevation(const Eigen::Vector2d &position, double measured_elevation,
                                        double measurement_variance) {
  if (!isWaveletBacked()) return false;
  if (!grid_map_.isInside(position)) return false;

  grid_map::Index index;
  grid_map_.getIndex(position, index);
  float &mean = grid_map_.at("elevation", index);
  float &variance = grid_map_.at("elevation_variance", index);

  const double kalman_gain = static_cast<double>(variance) / (static_cast<double>(variance) + measurement_variance);
  const double new_mean = static_cast<double>(mean) + kalman_gain * (measured_elevation - static_cast<double>(mean));
  const double new_variance = (1.0 - kalman_gain) * static_cast<double>(variance);
  mean = static_cast<float>(new_mean);
  variance = static_cast<float>(new_variance);

  const Eigen::Vector2d world_position = position + maporigin_.position.head<2>();
  elevation_tree_->setCellValue(world_position, static_cast<float>(new_mean));
  variance_tree_->setCellValue(world_position, static_cast<float>(new_variance));
  return true;
}

bool WaveletTerrainMap::checkpoint() {
  if (!isWaveletBacked()) return false;
  elevation_tree_->prune();
  variance_tree_->prune();
  const bool elevation_ok = elevation_tree_->saveToFile(wavelet_store_dir_ + "/elevation.wavelet_quadtree");
  const bool variance_ok = variance_tree_->saveToFile(wavelet_store_dir_ + "/variance.wavelet_quadtree");
  return elevation_ok && variance_ok;
}

std::vector<HashedWaveletQuadtree::Cell> WaveletTerrainMap::getElevationCells() const {
  if (!isWaveletBacked()) return {};
  const Eigen::Vector2d center = maporigin_.position.head<2>();
  const Eigen::Vector2d half_extent = grid_map_.getLength() / 2.0;
  std::vector<HashedWaveletQuadtree::Cell> cells = elevation_tree_->getCells(center - half_extent, center + half_extent);
  // The tree is indexed in an absolute/world frame; convert to local-frame
  // coordinates (world minus map center) before returning, matching
  // grid_map_'s own convention of being internally centered at local (0,0)
  // -- callers (e.g. an RViz overlay under the same frame_id/TF as
  // getGridMap()'s "elevation_map" topic) shouldn't need to know about the
  // tree's world frame at all.
  for (HashedWaveletQuadtree::Cell& cell : cells) {
    cell.min_corner -= center;
  }
  return cells;
}
