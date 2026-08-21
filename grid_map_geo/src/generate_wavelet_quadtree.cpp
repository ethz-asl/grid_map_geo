/**
 * @brief Offline CLI tool: converts a monolithic GeoTIFF DSM into a
 * persistent wavelet quadtree elevation store (elevation.wavelet_quadtree +
 * variance.wavelet_quadtree), consumable by GridMapGeo::LoadFromWaveletQuadtree.
 *
 * Reads the source raster in bounded tiles via GDAL windowed RasterIO,
 * rather than one whole-raster read (as GridMapGeo::Load()/
 * initializeFromGeotiff() does): peak memory stays O(tile size), not
 * O(raster size), which is what actually makes this usable on real
 * hundreds-of-km source DSMs instead of only small test rasters.
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "grid_map_geo/hashed_wavelet_quadtree.hpp"

#if __APPLE__
#include <gdal.h>
#include <gdal_priv.h>
#include <ogr_spatialref.h>
#else
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>
#endif

namespace {

struct Args {
  std::string input;
  std::string output;
  double prior_variance = 25.0;
  int tree_height = 6;
  float max_error = 0.0f;
};

void printUsage() {
  std::cerr << "Usage: generate_wavelet_quadtree --input <geotiff> --output <dir>\n"
               "                                 [--prior-variance <value>] [--tree-height <n>]\n"
               "                                 [--max-error <meters>]\n"
               "\n"
               "  --input           Path to the source elevation GeoTIFF.\n"
               "  --output          Output directory; will contain elevation.wavelet_quadtree\n"
               "                    and variance.wavelet_quadtree.\n"
               "  --prior-variance  Uniform variance assigned to the static prior (default: 25.0),\n"
               "                    i.e. how much to trust the source DSM before any onboard\n"
               "                    measurement has been fused in.\n"
               "  --tree-height     Wavelet quadtree height per hashed block (default: 6, i.e.\n"
               "                    64x64 leaf cells per block).\n"
               "  --max-error       Lossy compression tolerance, in meters (default: 0, i.e.\n"
               "                    disabled -- exact/lossless). When set, guarantees every\n"
               "                    stored elevation is within this many meters of the source\n"
               "                    value (see HashedWaveletQuadtree::boundedPrune) -- pick a\n"
               "                    real, physically-meaningful tolerance (e.g. your vehicle's\n"
               "                    own characteristic size), not a generic performance knob.\n";
}

bool parseArgs(int argc, char** argv, Args& args) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? std::string(argv[++i]) : std::string(); };
    if (arg == "--input") {
      args.input = next();
    } else if (arg == "--output") {
      args.output = next();
    } else if (arg == "--prior-variance") {
      args.prior_variance = std::stod(next());
    } else if (arg == "--tree-height") {
      args.tree_height = std::stoi(next());
    } else if (arg == "--max-error") {
      args.max_error = std::stof(next());
    } else if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      return false;
    }
  }
  if (args.input.empty() || args.output.empty()) {
    printUsage();
    return false;
  }
  return true;
}

// Ingesting a whole tile's worth of RasterIO output per iteration (rather
// than one GDAL read per pixel) is what keeps this fast at real-DSM sizes;
// the tile buffer itself is the only O(tile), not O(raster), allocation.
int tileSideFor(GDALRasterBand* band, int raster_size) {
  int block_x = 0, block_y = 0;
  band->GetBlockSize(&block_x, &block_y);
  constexpr int kMinUsefulBlock = 256;   // below this (e.g. 1-row strip tiling), prefer the fixed fallback
  constexpr int kFallbackTileSide = 1024;
  const int block_side = std::min(block_x, block_y);
  const int side = (block_side >= kMinUsefulBlock) ? block_side : kFallbackTileSide;
  return std::min(side, raster_size);
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parseArgs(argc, argv, args)) return 1;

  GDALAllRegister();
  // GDAL's own block cache defaults to a fraction of system RAM (e.g. 5%,
  // which is well over a gigabyte on a typical machine) and, left alone,
  // retains every decoded block for the life of the process -- for a
  // sequential single-pass scan like this one (each pixel read exactly
  // once via the windowed RasterIO calls below), that cache buys nothing
  // and would otherwise silently let peak RSS grow with the *raster's*
  // size, undermining the whole point of tiling. Bound it to a small,
  // fixed budget instead.
  GDALSetCacheMax64(64 * 1024 * 1024);
  const auto dataset = GDALDatasetUniquePtr(GDALDataset::FromHandle(GDALOpen(args.input.c_str(), GA_ReadOnly)));
  if (!dataset) {
    std::cerr << "Failed to open GeoTIFF: " << args.input << "\n";
    return 1;
  }

  std::array<double, 6> geo_transform{};
  if (dataset->GetGeoTransform(geo_transform.data()) != CE_None) {
    std::cerr << "Failed to read geotransform from " << args.input << "\n";
    return 1;
  }
  const double origin_x = geo_transform[0];
  const double origin_y = geo_transform[3];
  const double pixel_size_x = geo_transform[1];
  const double pixel_size_y = geo_transform[5];  // negative: image origin is north, +row goes south
  const double resolution = std::abs(pixel_size_x);

  const OGRSpatialReference* spatial_ref = dataset->GetSpatialRef();
  const int epsg = (spatial_ref && spatial_ref->GetAttrValue("AUTHORITY", 1)) ? std::stoi(spatial_ref->GetAttrValue("AUTHORITY", 1)) : -1;

  const int width = dataset->GetRasterXSize();
  const int height = dataset->GetRasterYSize();
  GDALRasterBand* band = dataset->GetRasterBand(1);

  HashedWaveletQuadtree elevation_tree(args.tree_height, resolution, 0.0f);
  // Deliberately never explicitly set: every unset cell already reads back
  // as `prior_variance` via the tree's own default value, at zero storage
  // cost, giving a uniform prior with no wasted allocation.
  HashedWaveletQuadtree variance_tree(args.tree_height, resolution, static_cast<float>(args.prior_variance));

  const int tile_w = tileSideFor(band, width);
  const int tile_h = tileSideFor(band, height);
  const int num_tiles_x = (width + tile_w - 1) / tile_w;
  const int num_tiles_y = (height + tile_h - 1) / tile_h;
  const size_t total_tiles = static_cast<size_t>(num_tiles_x) * static_cast<size_t>(num_tiles_y);
  // Every touched leaf allocates its full ancestor node chain at write
  // time, regardless of whether it will end up collapsing away -- prune()
  // is what actually reclaims a locally-uniform region's nodes. Pruning
  // only every many tiles lets that transient, not-yet-collapsed state
  // accumulate across all those tiles' worth of writes before ever being
  // reclaimed, which -- for a large flat expanse -- reintroduces an
  // ingestion-time memory footprint that scales with pixels scanned, not
  // with the (small) compressed result. prune() itself only costs O(the
  // tree's current, already-compressed size), not O(pixels processed), so
  // pruning every tile is cheap and keeps peak memory genuinely O(tile).
  constexpr size_t kPruneEveryNTiles = 1;
  constexpr size_t kProgressEveryNTiles = 64;

  std::cout << "Ingesting " << args.input << " (" << width << "x" << height << " px, " << resolution
            << "m resolution) in " << num_tiles_x << "x" << num_tiles_y << " tiles of " << tile_w << "x" << tile_h
            << " px\n";

  std::vector<float> tile_buffer;
  size_t num_cells = 0;
  size_t num_skipped = 0;
  size_t tiles_done = 0;
  for (int ty = 0; ty < num_tiles_y; ++ty) {
    for (int tx = 0; tx < num_tiles_x; ++tx) {
      const int xoff = tx * tile_w;
      const int yoff = ty * tile_h;
      const int xsize = std::min(tile_w, width - xoff);
      const int ysize = std::min(tile_h, height - yoff);
      tile_buffer.assign(static_cast<size_t>(xsize) * static_cast<size_t>(ysize), 0.0f);
      const CPLErr err =
          band->RasterIO(GF_Read, xoff, yoff, xsize, ysize, tile_buffer.data(), xsize, ysize, GDT_Float32, 0, 0);
      if (err != CE_None) {
        std::cerr << "\nFailed to read tile at (" << xoff << ", " << yoff << ")\n";
        return 1;
      }

      for (int row = 0; row < ysize; ++row) {
        for (int col = 0; col < xsize; ++col) {
          const float elevation = tile_buffer[static_cast<size_t>(row) * xsize + col];
          if (!std::isfinite(elevation)) {
            // Note: this only guards against NaN/Inf, which would corrupt
            // the wavelet coefficients' arithmetic. It does not filter
            // arbitrary nodata sentinel values (e.g. -9999) -- consistent
            // with GridMapGeo::initializeFromGeotiff(), which does not
            // handle nodata either.
            ++num_skipped;
            continue;
          }
          const double world_x = origin_x + (xoff + col + 0.5) * pixel_size_x;
          const double world_y = origin_y + (yoff + row + 0.5) * pixel_size_y;
          elevation_tree.setCellValue(Eigen::Vector2d(world_x, world_y), elevation);
          ++num_cells;
        }
      }

      ++tiles_done;
      if (tiles_done % kPruneEveryNTiles == 0) {
        elevation_tree.prune();
      }
      if (tiles_done % kProgressEveryNTiles == 0 || tiles_done == total_tiles) {
        std::cout << "\r  tiles " << tiles_done << "/" << total_tiles << ", " << num_cells << " cells, "
                  << elevation_tree.getMemoryUsage() << " bytes" << std::flush;
      }
    }
  }
  std::cout << "\n";

  if (args.max_error > 0.0f) {
    std::cout << "Applying bounded lossy compression (max error " << args.max_error << " m)...\n";
    elevation_tree.boundedPrune(args.max_error);
  } else {
    elevation_tree.prune();
  }
  variance_tree.prune();

  std::filesystem::create_directories(args.output);
  const bool elevation_ok = elevation_tree.saveToFile(args.output + "/elevation.wavelet_quadtree");
  const bool variance_ok = variance_tree.saveToFile(args.output + "/variance.wavelet_quadtree");

  // Records the source raster's own exact extent, so a consumer (see
  // GridMapGeo::LoadFromWaveletQuadtree's no-extent overload) can load the
  // whole store without the caller having to separately know/guess its
  // center and size -- guessing wrong (e.g. rounding up to be safe) leaves
  // a boundary strip of never-written (default-value) cells around the
  // true data, which is exactly the mismatch this file exists to avoid.
  const double world_center_x = origin_x + pixel_size_x * width * 0.5;
  const double world_center_y = origin_y + pixel_size_y * height * 0.5;
  const double extent_x_m = std::abs(pixel_size_x) * width;
  const double extent_y_m = std::abs(pixel_size_y) * height;
  std::ofstream extent_file(args.output + "/extent.txt");
  extent_file << std::setprecision(17) << world_center_x << " " << world_center_y << " " << extent_x_m << " "
              << extent_y_m << "\n";
  const bool extent_ok = static_cast<bool>(extent_file);

  if (!elevation_ok || !variance_ok || !extent_ok) {
    std::cerr << "Failed to write wavelet quadtree store to " << args.output << "\n";
    return 1;
  }

  const size_t dense_bytes = num_cells * sizeof(float);
  std::cout << "Wrote wavelet quadtree store to " << args.output << "\n"
            << "  cells inserted:     " << num_cells << " (" << num_skipped << " skipped, non-finite)\n"
            << "  resolution:         " << resolution << " m\n"
            << "  EPSG:               " << epsg << "\n"
            << "  max error:          " << (args.max_error > 0.0f ? std::to_string(args.max_error) + " m (lossy)"
                                                                   : std::string("0 (lossless)"))
            << "\n"
            << "  elevation memory:   " << elevation_tree.getMemoryUsage() << " bytes (dense equivalent: "
            << dense_bytes << " bytes)\n"
            << "  variance memory:    " << variance_tree.getMemoryUsage()
            << " bytes (uniform prior, no explicit cells set)\n";
  return 0;
}
