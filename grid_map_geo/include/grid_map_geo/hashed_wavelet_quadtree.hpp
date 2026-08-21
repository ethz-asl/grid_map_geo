#ifndef GRID_MAP_GEO_HASHED_WAVELET_QUADTREE_H
#define GRID_MAP_GEO_HASHED_WAVELET_QUADTREE_H

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>

/**
 * @brief A Haar-wavelet-compressed, hashed-block-sparse, multi-resolution 2D
 * scalar field, used to store an elevation (or elevation-variance) map at
 * Earth scale with bounded memory. Self-contained (no external tree
 * library): a 2D Haar wavelet transform over a hashed map of fixed-height
 * quadtree blocks, each block backed by its own pooled, index-addressed node
 * array (rather than per-node heap allocations) to avoid malloc-per-node
 * overhead. Internal types are kept out of this header (pimpl) for a lean
 * public interface.
 *
 * A locally-uniform region produces exactly-zero detail coefficients at
 * every level (not just "small"), which is what lets prune() collapse whole
 * subtrees of flat terrain -- this is the actual compression mechanism.
 *
 * Cell updates (getCellValue/setCellValue/addToCellValue) only touch a
 * leaf's ancestor path, i.e. they cost O(tree height), not O(map size).
 */
class HashedWaveletQuadtree {
 public:
  /// Snapshot of one allocated hashed block, for visualization/diagnostics
  /// (e.g. an RViz display of the tree's block structure).
  struct BlockInfo {
    Eigen::Vector2d min_corner;      ///< World-frame min (south-west) corner.
    double size;                     ///< Side length, in meters, of this (square) block.
    float representative_value;      ///< Block's root scale coefficient (its coarse average).
    size_t num_nodes;                ///< Allocated node count; cheap proxy for local detail retained.
  };

  /// One multi-resolution leaf cell, i.e. one already-collapsed subtree at
  /// whatever level it stopped at -- large for locally-uniform terrain,
  /// down to getMinCellWidth() where full detail is retained. See getCells().
  struct Cell {
    Eigen::Vector2d min_corner;      ///< World-frame min (south-west) corner.
    double size;                     ///< Side length, in meters, of this (square) cell.
    float value;                     ///< This cell's (already-reconstructed) value.
  };

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

  /**
   * @brief Lossy-but-bounded compression: guarantees getCellValue() for any
   * previously-set position is within max_error of its value from just
   * before this call. Unlike threshold()/prune() (exact/lossless, and the
   * only thing setCellValue/addToCellValue/checkpoint's fidelity relies
   * on), this can deliberately coarsen genuinely-varying terrain -- use it
   * only when max_error is a real, physically-meaningful tolerance (e.g. a
   * vehicle's own characteristic size), not as a generic performance knob.
   * The bound is checked directly against every actual leaf value
   * underneath each candidate merge (their true min/max), not inferred
   * from a coefficient magnitude, so it holds regardless of how many tree
   * levels a region ends up collapsing through.
   *
   * @param max_error maximum allowed deviation, in the same units as cell
   * values (meters, for an elevation tree), between any original value and
   * its value after this call.
   */
  void boundedPrune(float max_error);

  size_t getMemoryUsage() const;
  bool empty() const;

  /// Snapshot of all currently-allocated blocks (see BlockInfo).
  std::vector<BlockInfo> getBlockInfo() const;

  /**
   * @brief Decompose the region [region_min, region_max] into its actual
   * multi-resolution leaf cells (see Cell), for visualizing the tree's
   * compression structure directly (e.g. an RViz overlay on top of a
   * reconstructRegion()-derived terrain surface). Cost is proportional to
   * the number of allocated nodes intersecting the region, i.e. to visual
   * complexity, not to the region's area or resolution.
   *
   * @param region_min world-frame min (south-west) corner of the query region.
   * @param region_max world-frame max (north-east) corner of the query region.
   */
  std::vector<Cell> getCells(const Eigen::Vector2d& region_min, const Eigen::Vector2d& region_max) const;

  /// Serialize the whole map (all allocated blocks) to a binary file.
  bool saveToFile(const std::string& path) const;
  /// Replace the map's contents by deserializing it from a binary file.
  bool loadFromFile(const std::string& path);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

#endif  // GRID_MAP_GEO_HASHED_WAVELET_QUADTREE_H
