#include <cstdio>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include <grid_map_core/iterators/GridMapIterator.hpp>

#include "grid_map_geo_wavelet_quadtree/wavelet_terrain_map.hpp"
#include "grid_map_geo_wavelet_quadtree/hashed_wavelet_quadtree.hpp"

namespace {

// Builds a small on-disk wavelet quadtree store (elevation + variance) under
// `dir`, with elevation following a known analytic surface z = x + 2*y (in
// *world*/absolute coordinates, i.e. the same frame `center` is given in),
// and a uniform variance. Mirrors the directory/file layout that
// WaveletTerrainMap::LoadFromWaveletQuadtree() and checkpoint() expect.
void buildWaveletQuadtreeStore(const std::string& dir, const Eigen::Vector2d& world_min,
                               int cells_per_side, float variance_value) {
  std::filesystem::create_directories(dir);

  HashedWaveletQuadtree elevation(6, 1.0, 0.0f);
  HashedWaveletQuadtree variance(6, 1.0, variance_value);
  for (int ix = 0; ix < cells_per_side; ++ix) {
    for (int iy = 0; iy < cells_per_side; ++iy) {
      const Eigen::Vector2d world_pos = world_min + Eigen::Vector2d(ix + 0.5, iy + 0.5);
      elevation.setCellValue(world_pos, static_cast<float>(world_pos.x() + 2.0 * world_pos.y()));
      variance.setCellValue(world_pos, variance_value);
    }
  }
  ASSERT_TRUE(elevation.saveToFile(dir + "/elevation.wavelet_quadtree"));
  ASSERT_TRUE(variance.saveToFile(dir + "/variance.wavelet_quadtree"));
}

}  // namespace

TEST(WaveletTerrainMapTest, LoadMaterializesExpectedElevationAndVariance) {
  const std::string dir = "/tmp/test_grid_map_geo_wavelet_store_load";
  std::filesystem::remove_all(dir);
  const Eigen::Vector2d center(1000.0, 2000.0);
  buildWaveletQuadtreeStore(dir, center - Eigen::Vector2d(5.0, 5.0), 10, 25.0f);

  WaveletTerrainMap map;
  ASSERT_TRUE(map.LoadFromWaveletQuadtree(dir, center, grid_map::Length(8.0, 8.0), /*query_height=*/0));
  EXPECT_TRUE(map.isWaveletBacked());

  // Check every actual grid cell (its own true center, per grid_map_'s own
  // indexing) rather than hand-picked local coordinates, which may not land
  // exactly on a cell center and would then legitimately resolve to a
  // neighboring cell.
  grid_map::GridMap& grid_map = map.getGridMap();
  for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
    Eigen::Vector2d local;
    grid_map.getPosition(*it, local);
    const Eigen::Vector2d world = local + center;
    const double expected_elevation = world.x() + 2.0 * world.y();
    EXPECT_NEAR(grid_map.at("elevation", *it), expected_elevation, 1e-1)
        << "at local (" << local.x() << ", " << local.y() << ")";
    EXPECT_NEAR(grid_map.at("elevation_variance", *it), 25.0, 1e-2);
  }
}

TEST(WaveletTerrainMapTest, UpdateElevationBlendsAndPersistsAcrossReload) {
  const std::string dir = "/tmp/test_grid_map_geo_wavelet_store_update";
  std::filesystem::remove_all(dir);
  const Eigen::Vector2d center(500.0, -300.0);
  buildWaveletQuadtreeStore(dir, center - Eigen::Vector2d(5.0, 5.0), 10, 100.0f);

  WaveletTerrainMap map;
  ASSERT_TRUE(map.LoadFromWaveletQuadtree(dir, center, grid_map::Length(8.0, 8.0), 0));

  // Pick an actual grid cell center (rather than a hand-picked coordinate
  // that might not land exactly on one) to update.
  grid_map::Index update_index;
  Eigen::Vector2d local;
  {
    grid_map::GridMapIterator it(map.getGridMap());
    update_index = *it;
    map.getGridMap().getPosition(update_index, local);
  }
  const Eigen::Vector2d world = local + center;
  const double prior_elevation = world.x() + 2.0 * world.y();

  // Fuse a confident measurement (tight variance) against the loose prior
  // (variance 100): result should land close to the measurement, not the
  // prior, and reported variance should shrink.
  const double measurement = prior_elevation + 50.0;
  ASSERT_TRUE(map.updateElevation(local, measurement, /*measurement_variance=*/1.0));

  const double fused_elevation = map.getGridMap().atPosition("elevation", local);
  const double fused_variance = map.getGridMap().atPosition("elevation_variance", local);
  EXPECT_NEAR(fused_elevation, measurement, 5.0) << "fused value should land close to the confident measurement";
  EXPECT_LT(fused_variance, 100.0) << "variance should shrink after a confident measurement";

  ASSERT_TRUE(map.checkpoint());

  // A fresh WaveletTerrainMap reloading the SAME store should see the fused values,
  // not the original prior -- this is the actual point of checkpoint().
  WaveletTerrainMap reloaded;
  ASSERT_TRUE(reloaded.LoadFromWaveletQuadtree(dir, center, grid_map::Length(8.0, 8.0), 0));
  EXPECT_NEAR(reloaded.getGridMap().atPosition("elevation", local), fused_elevation, 1e-1);
  EXPECT_NEAR(reloaded.getGridMap().atPosition("elevation_variance", local), fused_variance, 1e-1);

  // A cell that was never updated should still read back the original prior.
  // Pick an actual cell far from update_index (again via the grid's own
  // indexing, not a hand-picked coordinate) to avoid any off-by-half-cell
  // ambiguity. Wrap with modulo to stay in bounds regardless of where
  // update_index happened to land.
  const grid_map::Size size = reloaded.getGridMap().getSize();
  const grid_map::Index untouched_index((update_index(0) + size(0) / 2) % size(0),
                                        (update_index(1) + size(1) / 2) % size(1));
  Eigen::Vector2d untouched_local;
  reloaded.getGridMap().getPosition(untouched_index, untouched_local);
  const Eigen::Vector2d untouched_world = untouched_local + center;
  EXPECT_NEAR(reloaded.getGridMap().at("elevation", untouched_index),
             untouched_world.x() + 2.0 * untouched_world.y(), 1e-1);
}

TEST(WaveletTerrainMapTest, CompressionErrorBoundIsReadFromExtentFile) {
  const std::string dir = "/tmp/test_grid_map_geo_wavelet_store_error_bound";
  std::filesystem::remove_all(dir);
  const Eigen::Vector2d center(100.0, 200.0);
  buildWaveletQuadtreeStore(dir, center - Eigen::Vector2d(5.0, 5.0), 10, 1.0f);
  {
    std::ofstream extent_file(dir + "/extent.txt");
    extent_file << center.x() << " " << center.y() << " " << 8.0 << " " << 8.0 << " " << 3.5 << "\n";
  }

  WaveletTerrainMap map;
  ASSERT_TRUE(map.LoadFromWaveletQuadtree(dir, /*query_height=*/0));
  EXPECT_NEAR(map.getCompressionErrorBound(), 3.5, 1e-9);

  // The bounded-region overload should pick up the same bound even though
  // it doesn't use extent.txt for center/extent, since the bound is a
  // property of the store, not of the queried region.
  WaveletTerrainMap map_explicit_region;
  ASSERT_TRUE(
      map_explicit_region.LoadFromWaveletQuadtree(dir, center, grid_map::Length(8.0, 8.0), /*query_height=*/0));
  EXPECT_NEAR(map_explicit_region.getCompressionErrorBound(), 3.5, 1e-9);
}

TEST(WaveletTerrainMapTest, CompressionErrorBoundDefaultsToZeroForOlderStoreFormat) {
  const std::string dir = "/tmp/test_grid_map_geo_wavelet_store_no_error_bound_field";
  std::filesystem::remove_all(dir);
  const Eigen::Vector2d center(0.0, 0.0);
  buildWaveletQuadtreeStore(dir, center - Eigen::Vector2d(5.0, 5.0), 10, 1.0f);
  {
    // Older stores (written before this field existed) only have 4 fields.
    std::ofstream extent_file(dir + "/extent.txt");
    extent_file << center.x() << " " << center.y() << " " << 8.0 << " " << 8.0 << "\n";
  }

  WaveletTerrainMap map;
  ASSERT_TRUE(map.LoadFromWaveletQuadtree(dir, /*query_height=*/0));
  EXPECT_NEAR(map.getCompressionErrorBound(), 0.0, 1e-9);
}

TEST(WaveletTerrainMapTest, UpdateElevationFailsWhenNotWaveletBacked) {
  WaveletTerrainMap map;
  EXPECT_FALSE(map.isWaveletBacked());
  EXPECT_FALSE(map.updateElevation(Eigen::Vector2d(0.0, 0.0), 10.0, 1.0));
  EXPECT_FALSE(map.checkpoint());
}
