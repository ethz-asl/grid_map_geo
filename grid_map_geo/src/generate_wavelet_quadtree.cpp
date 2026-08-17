/**
 * @brief Offline CLI tool: converts a monolithic GeoTIFF DSM into a
 * persistent wavelet quadtree elevation store (elevation.wavelet_quadtree +
 * variance.wavelet_quadtree), consumable by GridMapGeo::LoadFromWaveletQuadtree.
 *
 * Reuses GridMapGeo::Load() (the existing, already-tested GDAL loading path)
 * to parse the source GeoTIFF, then walks the resulting dense grid_map into
 * a fresh HashedWaveletQuadtree -- rather than a standalone Python/GDAL
 * script -- since the wavelet quadtree's on-disk format is a bespoke binary
 * serialization tied directly to the HashedWaveletQuadtree C++ class; reusing
 * that same class here guarantees format compatibility with the runtime
 * loader with no risk of the two implementations drifting apart.
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

#include <grid_map_core/iterators/GridMapIterator.hpp>

#include "grid_map_geo/grid_map_geo.hpp"
#include "grid_map_geo/hashed_wavelet_quadtree.hpp"

namespace {

struct Args {
  std::string input;
  std::string output;
  double prior_variance = 25.0;
  int tree_height = 6;
};

void printUsage() {
  std::cerr << "Usage: generate_wavelet_quadtree --input <geotiff> --output <dir>\n"
               "                                 [--prior-variance <value>] [--tree-height <n>]\n"
               "\n"
               "  --input           Path to the source elevation GeoTIFF.\n"
               "  --output          Output directory; will contain elevation.wavelet_quadtree\n"
               "                    and variance.wavelet_quadtree.\n"
               "  --prior-variance  Uniform variance assigned to the static prior (default: 25.0),\n"
               "                    i.e. how much to trust the source DSM before any onboard\n"
               "                    measurement has been fused in.\n"
               "  --tree-height     Wavelet quadtree height per hashed block (default: 6, i.e.\n"
               "                    64x64 leaf cells per block).\n";
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

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parseArgs(argc, argv, args)) return 1;

  GridMapGeo source_map;
  if (!source_map.Load(args.input)) {
    std::cerr << "Failed to load GeoTIFF: " << args.input << "\n";
    return 1;
  }

  EPSG epsg;
  Eigen::Vector3d origin;
  source_map.getGlobalOrigin(epsg, origin);
  const Eigen::Vector2d world_origin = origin.head<2>();

  const double resolution = source_map.getGridMap().getResolution();
  HashedWaveletQuadtree elevation_tree(args.tree_height, resolution, 0.0f);
  // Deliberately never explicitly set: every unset cell already reads back
  // as `prior_variance` via the tree's own default value, at zero storage
  // cost, giving a uniform prior with no wasted allocation.
  HashedWaveletQuadtree variance_tree(args.tree_height, resolution, static_cast<float>(args.prior_variance));

  grid_map::GridMap& grid_map = source_map.getGridMap();
  const grid_map::Matrix& elevation_layer = grid_map["elevation"];
  size_t num_cells = 0;
  size_t num_skipped = 0;
  for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    const float elevation = elevation_layer(index(0), index(1));
    if (!std::isfinite(elevation)) {
      // Note: this only guards against NaN/Inf, which would corrupt the
      // wavelet coefficients' arithmetic. It does not filter arbitrary
      // nodata sentinel values (e.g. -9999) -- consistent with
      // GridMapGeo::initializeFromGeotiff(), which does not handle nodata
      // either.
      ++num_skipped;
      continue;
    }
    Eigen::Vector2d local_position;
    grid_map.getPosition(*it, local_position);
    const Eigen::Vector2d world_position = local_position + world_origin;
    elevation_tree.setCellValue(world_position, elevation);
    ++num_cells;
  }

  elevation_tree.prune();
  variance_tree.prune();

  std::filesystem::create_directories(args.output);
  const bool elevation_ok = elevation_tree.saveToFile(args.output + "/elevation.wavelet_quadtree");
  const bool variance_ok = variance_tree.saveToFile(args.output + "/variance.wavelet_quadtree");
  if (!elevation_ok || !variance_ok) {
    std::cerr << "Failed to write wavelet quadtree store to " << args.output << "\n";
    return 1;
  }

  const size_t dense_bytes = num_cells * sizeof(float);
  std::cout << "Wrote wavelet quadtree store to " << args.output << "\n"
            << "  cells inserted:     " << num_cells << " (" << num_skipped << " skipped, non-finite)\n"
            << "  resolution:         " << resolution << " m\n"
            << "  world origin (EPSG " << static_cast<int>(epsg) << "): " << world_origin.x() << ", "
            << world_origin.y() << "\n"
            << "  elevation memory:   " << elevation_tree.getMemoryUsage() << " bytes (dense equivalent: "
            << dense_bytes << " bytes)\n"
            << "  variance memory:    " << variance_tree.getMemoryUsage()
            << " bytes (uniform prior, no explicit cells set)\n";
  return 0;
}
