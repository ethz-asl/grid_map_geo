/**
 * @brief Offline CLI tool: benchmarks per-query elevation access time of the
 * two map representations this repo supports --
 *   (1) a dense grid_map::GridMap loaded straight from a GeoTIFF
 *       (GridMapGeo::Load + grid_map::GridMap::atPosition), and
 *   (2) a HashedWaveletQuadtree loaded directly from a wavelet quadtree
 *       store (HashedWaveletQuadtree::getCellValue), without materializing
 *       it into a dense grid_map_ first.
 *
 * Not a ROS node -- no rclcpp dependency, same style as
 * generate_wavelet_quadtree.cpp.
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "grid_map_geo/grid_map_geo.hpp"
#include "grid_map_geo_wavelet_quadtree/hashed_wavelet_quadtree.hpp"

namespace {

struct Args {
  std::string tif_path;
  std::string tile_store_dir;
  size_t num_queries = 100000;
  int query_height = 0;
  unsigned seed = 42;
};

void printUsage() {
  std::cerr << "Usage: evaluate_access_time [--tif <geotiff>] [--tile-store <dir>]\n"
               "                             [--num-queries <n>] [--query-height <n>] [--seed <n>]\n"
               "\n"
               "At least one of --tif / --tile-store must be given.\n"
               "\n"
               "  --tif            Path to a source elevation GeoTIFF. Benchmarks dense\n"
               "                   grid_map::GridMap::atPosition() access.\n"
               "  --tile-store     Directory containing elevation.wavelet_quadtree and\n"
               "                   extent.txt (as written by generate_wavelet_quadtree).\n"
               "                   Benchmarks HashedWaveletQuadtree::getCellValue() access\n"
               "                   directly against the tree, without materializing a\n"
               "                   dense grid_map_ first.\n"
               "  --num-queries    Number of random-position queries to time (default: 100000).\n"
               "  --query-height   Resolution level to query the tree at, 0 = finest (default: 0).\n"
               "  --seed           RNG seed for reproducible query positions (default: 42).\n";
}

bool parseArgs(int argc, char** argv, Args& args) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? std::string(argv[++i]) : std::string(); };
    if (arg == "--tif") {
      args.tif_path = next();
    } else if (arg == "--tile-store") {
      args.tile_store_dir = next();
    } else if (arg == "--num-queries") {
      args.num_queries = std::stoul(next());
    } else if (arg == "--query-height") {
      args.query_height = std::stoi(next());
    } else if (arg == "--seed") {
      args.seed = static_cast<unsigned>(std::stoul(next()));
    } else if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      return false;
    }
  }
  if (args.tif_path.empty() && args.tile_store_dir.empty()) {
    printUsage();
    return false;
  }
  return true;
}

// Read the same "center_x center_y extent_x extent_y" format generate_wavelet_quadtree
// writes to extent.txt (see GridMapGeo::LoadFromWaveletQuadtree's no-extent overload).
bool readExtent(const std::string& tile_store_dir, Eigen::Vector2d& center, Eigen::Vector2d& extent) {
  std::ifstream extent_file(tile_store_dir + "/extent.txt");
  if (!extent_file) return false;
  extent_file >> center.x() >> center.y() >> extent.x() >> extent.y();
  return static_cast<bool>(extent_file);
}

struct Stats {
  size_t count = 0;
  double total_ms = 0.0;
  double min_ns = 0.0;
  double p50_ns = 0.0;
  double p99_ns = 0.0;
  double max_ns = 0.0;
};

void printStats(const std::string& label, const Stats& s) {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  " << label << ":\n"
            << "    queries:       " << s.count << "\n"
            << "    total time:    " << s.total_ms << " ms\n"
            << "    mean:          " << (s.total_ms * 1e6 / static_cast<double>(s.count)) << " ns/query\n"
            << "    min / p50 / p99 / max: " << s.min_ns << " / " << s.p50_ns << " / " << s.p99_ns << " / "
            << s.max_ns << " ns\n"
            << "    throughput:    " << (static_cast<double>(s.count) / (s.total_ms / 1000.0)) << " queries/s\n";
}

Stats summarize(std::vector<double>& durations_ns) {
  Stats s;
  s.count = durations_ns.size();
  double sum_ns = 0.0;
  for (double d : durations_ns) sum_ns += d;
  s.total_ms = sum_ns / 1e6;
  std::sort(durations_ns.begin(), durations_ns.end());
  s.min_ns = durations_ns.front();
  s.max_ns = durations_ns.back();
  s.p50_ns = durations_ns[durations_ns.size() / 2];
  s.p99_ns = durations_ns[static_cast<size_t>(durations_ns.size() * 0.99)];
  return s;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parseArgs(argc, argv, args)) return 1;

  std::mt19937 rng(args.seed);

  if (!args.tif_path.empty()) {
    std::cout << "Loading GeoTIFF " << args.tif_path << " ...\n";
    const auto load_start = std::chrono::steady_clock::now();
    GridMapGeo map;
    if (!map.Load(args.tif_path)) {
      std::cerr << "Failed to load " << args.tif_path << "\n";
      return 1;
    }
    const double load_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - load_start).count();
    std::cout << "  load time:     " << load_ms << " ms\n";

    grid_map::GridMap& grid_map = map.getGridMap();
    const grid_map::Length length = grid_map.getLength();
    const grid_map::Position center = grid_map.getPosition();
    // Shrink slightly so sampled positions never fall exactly on/outside the boundary.
    std::uniform_real_distribution<double> dist_x(center.x() - 0.499 * length.x(), center.x() + 0.499 * length.x());
    std::uniform_real_distribution<double> dist_y(center.y() - 0.499 * length.y(), center.y() + 0.499 * length.y());

    std::vector<double> durations_ns;
    durations_ns.reserve(args.num_queries);
    volatile float sink = 0.0f;  // prevent the compiler from optimizing the lookup away
    for (size_t i = 0; i < args.num_queries; ++i) {
      const grid_map::Position query(dist_x(rng), dist_y(rng));
      const auto t0 = std::chrono::steady_clock::now();
      sink = grid_map.atPosition("elevation", query);
      const auto t1 = std::chrono::steady_clock::now();
      durations_ns.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count());
    }
    (void)sink;
    Stats stats = summarize(durations_ns);
    std::cout << "Dense grid_map::GridMap::atPosition() (" << grid_map.getSize().x() << "x" << grid_map.getSize().y()
              << " cells):\n";
    printStats("atPosition", stats);
    std::cout << "\n";
  }

  if (!args.tile_store_dir.empty()) {
    Eigen::Vector2d center, extent;
    if (!readExtent(args.tile_store_dir, center, extent)) {
      std::cerr << "Failed to read " << args.tile_store_dir << "/extent.txt\n";
      return 1;
    }

    std::cout << "Loading wavelet quadtree store " << args.tile_store_dir << " ...\n";
    const auto load_start = std::chrono::steady_clock::now();
    HashedWaveletQuadtree tree;
    if (!tree.loadFromFile(args.tile_store_dir + "/elevation.wavelet_quadtree")) {
      std::cerr << "Failed to load " << args.tile_store_dir << "/elevation.wavelet_quadtree\n";
      return 1;
    }
    const double load_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - load_start).count();
    std::cout << "  load time:     " << load_ms << " ms\n"
              << "  memory usage:  " << (tree.getMemoryUsage() / (1024.0 * 1024.0)) << " MiB\n"
              << "  min cell width:" << tree.getMinCellWidth() << " m\n";

    std::uniform_real_distribution<double> dist_x(center.x() - 0.499 * extent.x(), center.x() + 0.499 * extent.x());
    std::uniform_real_distribution<double> dist_y(center.y() - 0.499 * extent.y(), center.y() + 0.499 * extent.y());

    std::vector<double> durations_ns;
    durations_ns.reserve(args.num_queries);
    volatile float sink = 0.0f;
    for (size_t i = 0; i < args.num_queries; ++i) {
      const Eigen::Vector2d query(dist_x(rng), dist_y(rng));
      const auto t0 = std::chrono::steady_clock::now();
      sink = tree.getCellValue(query, args.query_height);
      const auto t1 = std::chrono::steady_clock::now();
      durations_ns.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count());
    }
    (void)sink;
    Stats stats = summarize(durations_ns);
    std::cout << "HashedWaveletQuadtree::getCellValue() (query height " << args.query_height << "):\n";
    printStats("getCellValue", stats);
  }

  return 0;
}
