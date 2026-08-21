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

TEST(HashedWaveletQuadtree, GetCellsReturnsMultiResolutionTiling) {
  HashedWaveletQuadtree map(6, 1.0, 0.0f);  // 64x64 cells/block

  // Left half (x in [0,32)) is perfectly flat -> should collapse to large
  // cells. Right half (x in [32,64)) has all-distinct values -> no group of
  // sibling cells is ever uniform (a wavelet detail triple is exactly zero
  // iff all 4 values are equal), so it must stay at the finest resolution.
  for (int ix = 0; ix < 64; ++ix) {
    for (int iy = 0; iy < 64; ++iy) {
      const float value = (ix < 32) ? 42.0f : static_cast<float>(ix * 1000 + iy);
      map.setCellValue(Eigen::Vector2d(ix + 0.5, iy + 0.5), value);
    }
  }
  map.prune();

  const auto cells = map.getCells(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(64.0, 64.0));

  // Tiling check: cells must exactly cover the region with no gaps or
  // overlaps, and each cell's value must match a direct point query at its
  // center.
  double total_area = 0.0;
  int flat_large_cells = 0;
  int detailed_leaf_cells = 0;
  for (const auto& cell : cells) {
    total_area += cell.size * cell.size;
    EXPECT_GE(cell.min_corner.x(), 0.0);
    EXPECT_GE(cell.min_corner.y(), 0.0);
    EXPECT_LE(cell.min_corner.x() + cell.size, 64.0);
    EXPECT_LE(cell.min_corner.y() + cell.size, 64.0);

    const Eigen::Vector2d center = cell.min_corner + Eigen::Vector2d(cell.size / 2.0, cell.size / 2.0);
    EXPECT_NEAR(cell.value, map.getCellValue(center), 1e-2f);

    if (cell.min_corner.x() + cell.size <= 32.0) {
      EXPECT_GT(cell.size, 1.0) << "flat region should not stay at finest resolution";
      ++flat_large_cells;
    } else {
      ASSERT_GE(cell.min_corner.x(), 32.0) << "cell straddles the flat/detailed boundary";
      EXPECT_DOUBLE_EQ(cell.size, 1.0) << "all-distinct region must stay at finest resolution";
      ++detailed_leaf_cells;
    }
  }
  EXPECT_NEAR(total_area, 64.0 * 64.0, 1e-6);
  EXPECT_EQ(flat_large_cells, 2);            // the flat half's two top-level quadrants each collapse to one cell
  EXPECT_EQ(detailed_leaf_cells, 32 * 64);   // detailed half stays fully dense at 1m
}

TEST(HashedWaveletQuadtree, BoundedPruneCoarsensOnlyWithinTolerance) {
  HashedWaveletQuadtree map(6, 1.0, 0.0f);  // 64x64 cells/block
  constexpr float kMaxError = 1.0f;

  // Left half (x in [0,32)): small, sub-tolerance variation (alternating
  // 100.0/100.3, range 0.3 << 2*kMaxError) -- should coarsen. Right half
  // (x in [32,64)): large variation (differs by ~1000 between adjacent x,
  // by 1 between adjacent y -- verified below that no 2x2 group's range
  // ever falls within 2*kMaxError) -- must stay at finest resolution.
  for (int ix = 0; ix < 64; ++ix) {
    for (int iy = 0; iy < 64; ++iy) {
      const float value = (ix < 32) ? (100.0f + 0.3f * ((ix + iy) % 2)) : static_cast<float>(ix * 1000 + iy);
      map.setCellValue(Eigen::Vector2d(ix + 0.5, iy + 0.5), value);
    }
  }

  map.boundedPrune(kMaxError);
  const auto cells = map.getCells(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(64.0, 64.0));

  double total_area = 0.0;
  int flat_large_cells = 0;
  int detailed_leaf_cells = 0;
  for (const auto& cell : cells) {
    total_area += cell.size * cell.size;
    if (cell.min_corner.x() + cell.size <= 32.0) {
      EXPECT_GT(cell.size, 1.0) << "sub-tolerance region should coarsen";
      ++flat_large_cells;
    } else {
      ASSERT_GE(cell.min_corner.x(), 32.0) << "cell straddles the coarse/detailed boundary";
      EXPECT_DOUBLE_EQ(cell.size, 1.0) << "over-tolerance region must stay at finest resolution";
      ++detailed_leaf_cells;
    }
  }
  EXPECT_NEAR(total_area, 64.0 * 64.0, 1e-6);
  EXPECT_EQ(flat_large_cells, 2);
  EXPECT_EQ(detailed_leaf_cells, 32 * 64);

  // The actual guarantee: every original value is still within kMaxError.
  for (int ix = 0; ix < 64; ++ix) {
    for (int iy = 0; iy < 64; ++iy) {
      const float original = (ix < 32) ? (100.0f + 0.3f * ((ix + iy) % 2)) : static_cast<float>(ix * 1000 + iy);
      const Eigen::Vector2d p(ix + 0.5, iy + 0.5);
      EXPECT_LE(std::abs(map.getCellValue(p) - original), kMaxError + 1e-3f)
          << "at (" << ix << ", " << iy << ")";
    }
  }
}

TEST(HashedWaveletQuadtree, BoundedPruneNeverCorruptsAnIsolatedPointLikeTheNaiveThresholdDid) {
  // Regression test for the failure mode that motivated boundedPrune():
  // raising the plain wavelet-detail-coefficient threshold (kDetailEpsilon)
  // to a large value zeroed out an isolated point update's own detail
  // coefficient (which can be much smaller than the update itself -- e.g.
  // a single 10.0 update produces a detail coefficient of just 2.5 at its
  // own leaf split), silently deleting it rather than bounding its error.
  // boundedPrune must not reproduce that: it checks true leaf values
  // directly, not a transformed coefficient.
  HashedWaveletQuadtree map(4, 1.0, 0.0f);
  const Eigen::Vector2d p(3.5, 3.5);
  map.setCellValue(p, 10.0f);
  ASSERT_NEAR(map.getCellValue(p), 10.0f, 1e-3f);

  map.boundedPrune(3.0f);

  EXPECT_LE(std::abs(map.getCellValue(p) - 10.0f), 3.0f)
      << "got " << map.getCellValue(p) << ", expected within 3.0 of 10.0";
}

TEST(HashedWaveletQuadtree, BoundedPruneGuaranteesMaxErrorAcrossManyPoints) {
  HashedWaveletQuadtree map(4, 1.0, 0.0f);
  constexpr float kMaxError = 5.0f;

  std::vector<std::pair<Eigen::Vector2d, float>> samples;
  for (int ix = -10; ix <= 10; ++ix) {
    for (int iy = -10; iy <= 10; ++iy) {
      const Eigen::Vector2d p(ix + 0.5, iy + 0.5);
      const float value = static_cast<float>((ix * 7 + iy * 13) % 37) - 18.0f;
      samples.emplace_back(p, value);
      map.setCellValue(p, value);
    }
  }

  map.boundedPrune(kMaxError);

  for (const auto& [p, value] : samples) {
    EXPECT_LE(std::abs(map.getCellValue(p) - value), kMaxError + 1e-3f)
        << "at (" << p.x() << ", " << p.y() << ")";
  }
}

TEST(HashedWaveletQuadtree, BoundedPruneFullyCollapsesDespiteFloatingPointResidue) {
  // Regression test: rewriting many DISTINCT (but all within max_error of
  // each other) values via independent setCellValue() calls, each
  // propagating its own delta through a SHARED ancestor chain via
  // non-associative float addition, leaves the leaves merely extremely
  // close rather than bit-identical -- residuals of a few millimeters were
  // observed on real terrain (a near-perfectly-flat 640m playa region that
  // failed to fully collapse at a 30m tolerance despite a true elevation
  // range of only a few centimeters). The cleanup epsilon must be loose
  // enough to absorb that residue, not just kDetailEpsilon's near-bit-exact
  // default (see kBoundedPruneCleanupEpsilon).
  HashedWaveletQuadtree map(6, 1.0, 0.0f);  // 64x64 cells/block
  constexpr float kBase = 1000.0f;  // large base value stresses float32 precision
  for (int ix = 0; ix < 64; ++ix) {
    for (int iy = 0; iy < 64; ++iy) {
      const float value = kBase + 0.001f * static_cast<float>((ix * 67 + iy * 131) % 50);
      map.setCellValue(Eigen::Vector2d(ix + 0.5, iy + 0.5), value);
    }
  }

  map.boundedPrune(1.0f);  // true range here is < 0.05m, comfortably within 2*1.0

  const auto cells = map.getCells(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(64.0, 64.0));
  EXPECT_EQ(cells.size(), 1u) << "a near-uniform region should fully collapse to one cell despite the rewrite's "
                                 "own floating-point residue";
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
