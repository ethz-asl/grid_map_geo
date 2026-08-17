#include <cmath>
#include <cstdio>
#include <random>

#include <gtest/gtest.h>

#include <grid_map_core/iterators/GridMapIterator.hpp>

#include "grid_map_geo/hashed_wavelet_quadtree.hpp"

TEST(HashedWaveletQuadtree, SetGetRoundTripAtFinestResolution) {
  HashedWaveletQuadtree map(/*tree_height=*/4, /*min_cell_width=*/1.0,
                           /*default_value=*/0.0f);

  EXPECT_TRUE(map.empty());
  EXPECT_FLOAT_EQ(map.getCellValue(Eigen::Vector2d(3.5, -2.2)), 0.0f);

  map.setCellValue(Eigen::Vector2d(3.5, -2.2), 42.0f);
  EXPECT_FALSE(map.empty());
  EXPECT_NEAR(map.getCellValue(Eigen::Vector2d(3.5, -2.2)), 42.0f, 1e-3f);

  // A cell far away should be unaffected (still default).
  EXPECT_NEAR(map.getCellValue(Eigen::Vector2d(500.0, -500.0)), 0.0f, 1e-3f);
}

TEST(HashedWaveletQuadtree, SetOverwritesAndAddAccumulates) {
  HashedWaveletQuadtree map(4, 1.0, 0.0f);
  const Eigen::Vector2d p(10.4, 20.6);

  map.setCellValue(p, 5.0f);
  EXPECT_NEAR(map.getCellValue(p), 5.0f, 1e-3f);

  map.setCellValue(p, -3.0f);
  EXPECT_NEAR(map.getCellValue(p), -3.0f, 1e-3f);

  map.addToCellValue(p, 1.5f);
  map.addToCellValue(p, 1.5f);
  EXPECT_NEAR(map.getCellValue(p), 0.0f, 1e-3f);
}

TEST(HashedWaveletQuadtree, ManyCellsAcrossMultipleBlocksRoundTrip) {
  // tree_height=3 -> 8x8 cells per block at 2m resolution -> 16m per block
  // side. Scatter points across many blocks (positive and negative indices)
  // to exercise the hashed block-map path, not just a single block.
  HashedWaveletQuadtree map(3, 2.0, 0.0f);

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> coord(-500.0, 500.0);
  std::vector<std::pair<Eigen::Vector2d, float>> samples;
  for (int i = 0; i < 500; ++i) {
    Eigen::Vector2d p(coord(rng), coord(rng));
    float value = static_cast<float>(i) - 250.0f;
    samples.emplace_back(p, value);
    map.setCellValue(p, value);
  }
  for (const auto& [p, value] : samples) {
    EXPECT_NEAR(map.getCellValue(p), value, 1e-2f)
        << "at (" << p.x() << ", " << p.y() << ")";
  }
}

TEST(HashedWaveletQuadtree, ReconstructRegionMatchesPointQueries) {
  HashedWaveletQuadtree map(4, 1.0, -1.0f);

  // Fill a known analytic surface z = x + 2*y on a grid of finest-resolution
  // cells, then check that reconstructRegion into a grid_map matches.
  for (int ix = -5; ix <= 5; ++ix) {
    for (int iy = -5; iy <= 5; ++iy) {
      Eigen::Vector2d p(static_cast<double>(ix) + 0.5, static_cast<double>(iy) + 0.5);
      map.setCellValue(p, static_cast<float>(ix + 2 * iy));
    }
  }

  grid_map::GridMap grid_map;
  grid_map.setGeometry(grid_map::Length(10.0, 10.0), 1.0, grid_map::Position(0.0, 0.0));
  grid_map.add("elevation");
  map.reconstructRegion(grid_map, "elevation", /*query_height=*/0);

  for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
    Eigen::Vector2d position;
    grid_map.getPosition(*it, position);
    const int ix = static_cast<int>(std::floor(position.x()));
    const int iy = static_cast<int>(std::floor(position.y()));
    if (ix < -5 || ix > 4 || iy < -5 || iy > 4) continue;  // outside filled region
    const float expected = static_cast<float>(ix + 2 * iy);
    EXPECT_NEAR(grid_map.at("elevation", *it), expected, 1e-2f)
        << "at grid position (" << position.x() << ", " << position.y() << ")";
  }
}

TEST(HashedWaveletQuadtree, SaveLoadRoundTrip) {
  const std::string path = "/tmp/test_hashed_wavelet_quadtree_save_load.bin";

  HashedWaveletQuadtree map(4, 1.5, 0.0f);
  map.setCellValue(Eigen::Vector2d(3.0, 3.0), 10.0f);
  map.setCellValue(Eigen::Vector2d(-40.0, 60.0), -20.0f);
  map.setCellValue(Eigen::Vector2d(1000.0, -1000.0), 99.0f);
  map.prune();

  ASSERT_TRUE(map.saveToFile(path));

  HashedWaveletQuadtree loaded(1, 1.0, -99.0f);  // deliberately different ctor args
  ASSERT_TRUE(loaded.loadFromFile(path));

  EXPECT_DOUBLE_EQ(loaded.getMinCellWidth(), 1.5);
  EXPECT_NEAR(loaded.getCellValue(Eigen::Vector2d(3.0, 3.0)), 10.0f, 1e-3f);
  EXPECT_NEAR(loaded.getCellValue(Eigen::Vector2d(-40.0, 60.0)), -20.0f, 1e-3f);
  EXPECT_NEAR(loaded.getCellValue(Eigen::Vector2d(1000.0, -1000.0)), 99.0f, 1e-3f);
  // A cell that was never set should read back as the *loaded* map's
  // original default value, not the file's implicit zero.
  EXPECT_NEAR(loaded.getCellValue(Eigen::Vector2d(50000.0, 50000.0)), 0.0f, 1e-3f);

  std::remove(path.c_str());
}

TEST(HashedWaveletQuadtree, MemoryUsageMuchSmallerThanDenseArrayForFlatRegion) {
  // A large, uniform (mostly-flat) region should compress to far less memory
  // than the naive dense-array equivalent -- this is the actual point of
  // using a Haar-wavelet-compressed representation.
  HashedWaveletQuadtree map(6, 1.0, 0.0f);  // 64x64 cells/block at 1m res

  constexpr int kSide = 512;  // 512x512 m region, all set to the same value
  for (int ix = 0; ix < kSide; ++ix) {
    for (int iy = 0; iy < kSide; ++iy) {
      map.setCellValue(Eigen::Vector2d(ix + 0.5, iy + 0.5), 7.0f);
    }
  }
  map.prune();

  const size_t dense_bytes = static_cast<size_t>(kSide) * kSide * sizeof(float);
  const size_t wavelet_bytes = map.getMemoryUsage();
  EXPECT_LT(wavelet_bytes, dense_bytes / 10)
      << "wavelet=" << wavelet_bytes << " dense=" << dense_bytes;
}
