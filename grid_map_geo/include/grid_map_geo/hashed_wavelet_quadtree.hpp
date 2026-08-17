#ifndef GRID_MAP_GEO_HASHED_WAVELET_QUADTREE_H
#define GRID_MAP_GEO_HASHED_WAVELET_QUADTREE_H

#include <cstddef>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>

/**
 * @brief A Haar-wavelet-compressed, hashed-block-sparse, multi-resolution 2D
 * scalar field, used to store an elevation (or elevation-variance) map at
 * Earth scale with bounded memory. Built on wavemap's (ethz-asl/wavemap,
 * BSD-3-Clause) generic, dimension/value-templated tree and hashing
 * primitives, instantiated here at dim=2 with a plain float value (rather
 * than wavemap's own dim=3 occupancy log-odds maps). wavemap's types are
 * kept out of this header (pimpl) so consumers of grid_map_geo do not need
 * to depend on wavemap directly.
 *
 * Cell updates (getCellValue/setCellValue/addToCellValue) only touch a
 * leaf's ancestor path, i.e. they cost O(tree height), not O(map size).
 */
class HashedWaveletQuadtree {
 public:
  /**
   * @param tree_height height of the wavelet quadtree stored in each hashed
   * block (mirrors wavemap's HashedWaveletOctreeConfig::tree_height). Higher
   * values mean fewer, larger blocks.
   * @param min_cell_width width, in meters, of the finest-resolution (leaf)
   * cell.
   * @param default_value value returned for cells that have never been set.
   */
  explicit HashedWaveletQuadtree(int tree_height = 6,
                                 double min_cell_width = 1.0,
                                 float default_value = 0.0f);
  ~HashedWaveletQuadtree();

  HashedWaveletQuadtree(HashedWaveletQuadtree&& other) noexcept;
  HashedWaveletQuadtree& operator=(HashedWaveletQuadtree&& other) noexcept;
  HashedWaveletQuadtree(const HashedWaveletQuadtree&) = delete;
  HashedWaveletQuadtree& operator=(const HashedWaveletQuadtree&) = delete;

  /// Query the cell value (at the finest resolution) covering world_position.
  float getCellValue(const Eigen::Vector2d& world_position) const;
  /// Query the cell value at a coarser resolution ("height" levels above the
  /// finest resolution) covering world_position.
  float getCellValue(const Eigen::Vector2d& world_position, int height) const;
  /// Overwrite the finest-resolution cell value covering world_position.
  void setCellValue(const Eigen::Vector2d& world_position, float value);
  /// Add `update` to the finest-resolution cell value covering world_position.
  void addToCellValue(const Eigen::Vector2d& world_position, float update);

  /**
   * @brief Reconstruct a dense, axis-aligned region of the map into a named
   * layer of `map`, for bridging into grid_map::GridMap for downstream
   * consumers (e.g. GridMapGeo::LoadFromWaveletQuadtree).
   *
   * @param map destination map; its geometry (position/length/resolution)
   * must already be set by the caller via setGeometry(), and `layer_name`
   * must already exist. Uses grid_map's own GridMapIterator/getPosition() to
   * map cells to world positions, so this shares grid_map's index/orientation
   * convention rather than introducing a second, independent one.
   * @param layer_name name of the (pre-existing) layer to populate.
   * @param query_height resolution level to reconstruct at (0 = finest).
   * @param origin_offset added to each of `map`'s own (local-frame) cell
   * positions before querying this tree (which is indexed in an absolute/
   * world frame). Lets `map` keep grid_map_geo's usual convention of being
   * internally centered at local (0,0) regardless of where the region
   * actually sits in the tree's world frame.
   */
  void reconstructRegion(grid_map::GridMap& map, const std::string& layer_name,
                        int query_height,
                        const Eigen::Vector2d& origin_offset = Eigen::Vector2d::Zero()) const;

  /// Width, in meters, of the finest-resolution (leaf) cell.
  double getMinCellWidth() const;
  /// Width, in meters, of a cell at the given resolution height.
  double resolutionAtHeight(int height) const;
  /// Height of the wavelet quadtree stored in each hashed block.
  int getTreeHeight() const;

  /// Re-compress detail coefficients (call after a batch of updates).
  void threshold();
  /// Free memory used by nodes that collapse to the default value. Calls
  /// threshold() first if needed.
  void prune();

  size_t getMemoryUsage() const;
  bool empty() const;

  /// Serialize the whole map (all allocated blocks) to a binary file.
  bool saveToFile(const std::string& path) const;
  /// Replace the map's contents by deserializing it from a binary file.
  bool loadFromFile(const std::string& path);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

#endif  // GRID_MAP_GEO_HASHED_WAVELET_QUADTREE_H
