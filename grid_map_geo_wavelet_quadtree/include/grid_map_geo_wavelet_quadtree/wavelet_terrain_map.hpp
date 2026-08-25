#ifndef GRID_MAP_GEO_WAVELET_QUADTREE_WAVELET_TERRAIN_MAP_H
#define GRID_MAP_GEO_WAVELET_QUADTREE_WAVELET_TERRAIN_MAP_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_geo/grid_map_geo.hpp>

#include "grid_map_geo_wavelet_quadtree/hashed_wavelet_quadtree.hpp"

/**
 * @brief A GridMapGeo materialized from a persistent, Earth-scale
 * Haar-wavelet quadtree elevation store (see HashedWaveletQuadtree) instead
 * of a single monolithic GeoTIFF, so terrain hundreds of km across can be
 * loaded within onboard memory constraints. Supports fusing onboard
 * measurements back into the store, and exposes its actual multi-resolution
 * cell structure for visualization.
 */
class WaveletTerrainMap : public GridMapGeo {
 public:
  using GridMapGeo::GridMapGeo;

  /**
   * @brief Materialize a bounded working-set grid map from a persistent,
   * Earth-scale wavelet quadtree elevation store (see HashedWaveletQuadtree),
   * instead of a single monolithic GeoTIFF. Populates "elevation" and
   * "elevation_variance" layers exactly like GridMapGeo::initializeFromGeotiff()
   * does for "elevation", so downstream consumers (TerrainMap, planners, RViz)
   * need no changes.
   *
   * @param tile_store_dir directory containing "elevation.wavelet_quadtree"
   * and "variance.wavelet_quadtree", as written by generate_wavelet_quadtree
   * or checkpoint().
   * @param center world-frame (local projected meters) center of the region
   * to materialize.
   * @param extent size (x, y), in meters, of the region to materialize.
   * @param query_height resolution level to reconstruct at (0 = finest).
   * @return true Successfully loaded the wavelet quadtree store.
   * @return false Failed to load the wavelet quadtree store.
   */
  bool LoadFromWaveletQuadtree(const std::string& tile_store_dir, const Eigen::Vector2d& center,
                               const grid_map::Length& extent, int query_height = 0);

  /**
   * @brief Materialize the ENTIRE region a wavelet quadtree store covers,
   * using the exact center/extent generate_wavelet_quadtree recorded for
   * the source raster (tile_store_dir/extent.txt) -- unlike the overload
   * above, the caller doesn't need to already know (or guess) the store's
   * size, and there's no boundary of never-written (default-value) cells
   * from an over-estimated extent. Only sensible for a store small enough
   * to materialize in full (e.g. a demo/visualization load); an onboard
   * vehicle should keep using the bounded-region overload above.
   *
   * @param tile_store_dir directory containing "elevation.wavelet_quadtree",
   * "variance.wavelet_quadtree", and "extent.txt".
   * @param query_height resolution level to reconstruct at (0 = finest).
   * @return true Successfully read the extent and loaded the store.
   * @return false extent.txt is missing/malformed (e.g. a store written
   * before this metadata existed -- regenerate it), or the store failed
   * to load.
   */
  bool LoadFromWaveletQuadtree(const std::string& tile_store_dir, int query_height = 0);

  /**
   * @brief Whether the map is currently backed by a wavelet quadtree store
   * (i.e. LoadFromWaveletQuadtree() was used).
   */
  bool isWaveletBacked() const { return static_cast<bool>(elevation_tree_); }

  /**
   * @brief Maximum error, in meters, between the "elevation" layer's
   * reconstructed value and the true source value, as recorded by
   * generate_wavelet_quadtree at compression time (0 if the store is
   * lossless, or not yet loaded). Consumers that need a guaranteed
   * clearance from the true terrain -- not just from the reconstructed
   * value -- should inflate their safety margin by this amount; see
   * HashedWaveletQuadtree::boundedPrune for the guarantee this bounds.
   */
  double getCompressionErrorBound() const { return compression_error_bound_; }

  /**
   * @brief Fuse a single onboard elevation measurement into the map with a
   * per-cell scalar Kalman update, at both the currently materialized
   * grid_map_ (for immediate use by the planner) and the underlying
   * persistent wavelet quadtree (for later checkpoint()ing). Only valid if
   * isWaveletBacked().
   *
   * @param position world-frame (x, y) position of the measurement.
   * @param measured_elevation measured elevation at that position.
   * @param measurement_variance variance of the measurement.
   * @return true Successfully fused the measurement.
   * @return false Not wavelet-backed, or position outside the loaded extent.
   */
  bool updateElevation(const Eigen::Vector2d& position, double measured_elevation, double measurement_variance);

  /**
   * @brief Persist all cells touched by updateElevation() since the last
   * checkpoint() (or since load) back to the wavelet quadtree store on disk,
   * so a later LoadFromWaveletQuadtree() of the same store picks them up.
   * Only valid if isWaveletBacked().
   *
   * @return true Successfully checkpointed.
   * @return false Not wavelet-backed, or the write failed.
   */
  bool checkpoint();

  /**
   * @brief Decompose the currently-materialized region (i.e. the same
   * world-frame extent last passed to LoadFromWaveletQuadtree()) into its
   * actual multi-resolution leaf cells (see HashedWaveletQuadtree::Cell),
   * for visualizing the underlying quadtree's compression structure
   * directly -- e.g. an RViz overlay on top of the elevation surface
   * derived from getGridMap(). Only valid if isWaveletBacked().
   *
   * @return Multi-resolution cells in world-frame coordinates; empty if not
   * wavelet-backed.
   */
  std::vector<HashedWaveletQuadtree::Cell> getElevationCells() const;

 protected:
  std::shared_ptr<HashedWaveletQuadtree> elevation_tree_;
  std::shared_ptr<HashedWaveletQuadtree> variance_tree_;
  std::string wavelet_store_dir_{""};
  double compression_error_bound_{0.0};
};
#endif  // GRID_MAP_GEO_WAVELET_QUADTREE_WAVELET_TERRAIN_MAP_H
