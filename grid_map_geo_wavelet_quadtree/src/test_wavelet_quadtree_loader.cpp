/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Demo/manual-verification node: republishes a grid_map materialized
 * from a persistent wavelet quadtree store, for side-by-side comparison in
 * RViz against test_tif_loader's monolithic-GeoTIFF path over the same
 * source dataset.
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <grid_map_msgs/msg/grid_map.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grid_map_geo_msgs/msg/quadtree_structure.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

#include "grid_map_geo_wavelet_quadtree/wavelet_terrain_map.hpp"

#if __APPLE__
#include <gdal.h>
#include <gdal_priv.h>
#else
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#endif

using namespace std::chrono_literals;

namespace {

// Averages the RGB pixels of `dataset` (assumed >=3 bands) under the
// world-frame footprint [world_min, world_min + size), converting via
// `dataset`'s own geotransform (computed independently of the elevation
// resolution, so this works even when the color raster isn't pixel-aligned
// with its DEM -- it just happens to be, for davosdorf). Returns false if
// the footprint doesn't overlap the raster at all.
bool sampleAverageColor(GDALDataset* dataset, const std::array<double, 6>& geo_transform,
                        const Eigen::Vector2d& world_min, double size, std_msgs::msg::ColorRGBA& out_color) {
  const double origin_x = geo_transform[0];
  const double origin_y = geo_transform[3];
  const double pixel_size_x = geo_transform[1];
  const double pixel_size_y = geo_transform[5];  // negative: image origin is north

  const double px0 = (world_min.x() - origin_x) / pixel_size_x;
  const double px1 = (world_min.x() + size - origin_x) / pixel_size_x;
  const double py0 = (world_min.y() - origin_y) / pixel_size_y;
  const double py1 = (world_min.y() + size - origin_y) / pixel_size_y;

  const int width = dataset->GetRasterXSize();
  const int height = dataset->GetRasterYSize();
  int xoff = static_cast<int>(std::floor(std::min(px0, px1)));
  int yoff = static_cast<int>(std::floor(std::min(py0, py1)));
  int xsize = static_cast<int>(std::ceil(std::max(px0, px1))) - xoff;
  int ysize = static_cast<int>(std::ceil(std::max(py0, py1))) - yoff;
  xsize = std::max(xsize, 1);
  ysize = std::max(ysize, 1);
  if (xoff >= width || yoff >= height || xoff + xsize <= 0 || yoff + ysize <= 0) {
    return false;  // footprint entirely outside the color raster
  }
  // Clamp the window to the raster bounds (a cell at the DEM's edge can
  // extend slightly past the color raster if the two aren't identically
  // sized).
  const int clipped_xoff = std::max(xoff, 0);
  const int clipped_yoff = std::max(yoff, 0);
  xsize = std::min(xoff + xsize, width) - clipped_xoff;
  ysize = std::min(yoff + ysize, height) - clipped_yoff;
  xoff = clipped_xoff;
  yoff = clipped_yoff;
  if (xsize <= 0 || ysize <= 0) return false;

  std::vector<uint8_t> buffer(static_cast<size_t>(xsize) * ysize);
  std::array<double, 3> channel_mean{0.0, 0.0, 0.0};
  for (int band = 0; band < 3; ++band) {
    if (dataset->GetRasterBand(band + 1)->RasterIO(GF_Read, xoff, yoff, xsize, ysize, buffer.data(), xsize, ysize,
                                                    GDT_Byte, 0, 0) != CE_None) {
      return false;
    }
    double sum = 0.0;
    for (uint8_t value : buffer) sum += value;
    channel_mean[band] = sum / static_cast<double>(buffer.size());
  }
  out_color.r = static_cast<float>(channel_mean[0] / 255.0);
  out_color.g = static_cast<float>(channel_mean[1] / 255.0);
  out_color.b = static_cast<float>(channel_mean[2] / 255.0);
  out_color.a = 1.0f;
  return true;
}

}  // namespace

class WaveletQuadtreeMapPublisher : public rclcpp::Node {
 public:
  WaveletQuadtreeMapPublisher() : Node("wavelet_quadtree_map_publisher") {
    std::string tile_store_dir = this->declare_parameter("tile_store_dir", ".");
    double center_x = this->declare_parameter("center_x", 0.0);
    double center_y = this->declare_parameter("center_y", 0.0);
    // 0 (either) is a sentinel for "not given" -- load the store's own full
    // extent (see extent.txt, written by generate_wavelet_quadtree) instead
    // of a caller-guessed bounded region, so there's no boundary strip of
    // never-written cells from an over-estimate.
    double extent_x = this->declare_parameter("extent_x", 0.0);
    double extent_y = this->declare_parameter("extent_y", 0.0);
    int query_height = this->declare_parameter("query_height", 0);
    std::string frame_id = this->declare_parameter("frame_id", "map");
    std::string color_path = this->declare_parameter("color_path", "");

    map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map", 1);
    structure_pub_ = this->create_publisher<grid_map_geo_msgs::msg::QuadtreeStructure>("quadtree_structure", 1);

    RCLCPP_INFO_STREAM(get_logger(), "tile_store_dir " << tile_store_dir);
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    map_ = std::make_shared<WaveletTerrainMap>(frame_id);
    const bool use_full_extent = extent_x <= 0.0 || extent_y <= 0.0;
    const bool loaded = use_full_extent
                            ? map_->LoadFromWaveletQuadtree(tile_store_dir, query_height)
                            : map_->LoadFromWaveletQuadtree(tile_store_dir, Eigen::Vector2d(center_x, center_y),
                                                            grid_map::Length(extent_x, extent_y), query_height);
    if (!loaded) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to load wavelet quadtree store from "
                                            << tile_store_dir
                                            << (use_full_extent ? " (no extent_x/extent_y given and "
                                                                  "extent.txt missing/malformed -- regenerate the "
                                                                  "store, or pass extent_x/extent_y explicitly)"
                                                                : ""));
      return;
    }

    // The store doesn't change once loaded (this is a static demo loader,
    // not an online updateElevation()/checkpoint() consumer), so the
    // QuadtreeStructure message -- including any color sampling below,
    // which would otherwise mean re-reading the color GeoTIFF for
    // potentially tens of thousands of cells on every 5-second tick for no
    // reason -- only needs to be built once, here, and just republished
    // with a fresh timestamp from the timer callback.
    structure_msg_.header.frame_id = map_->getGridMap().getFrameId();

    EPSG epsg;
    Eigen::Vector3d map_origin;
    map_->getGlobalOrigin(epsg, map_origin);
    const Eigen::Vector2d world_center = map_origin.head<2>();

    GDALDatasetUniquePtr color_dataset;
    std::array<double, 6> color_geo_transform{};
    bool have_color = false;
    if (!color_path.empty()) {
      GDALAllRegister();
      color_dataset = GDALDatasetUniquePtr(GDALDataset::FromHandle(GDALOpen(color_path.c_str(), GA_ReadOnly)));
      if (color_dataset && color_dataset->GetGeoTransform(color_geo_transform.data()) == CE_None &&
          color_dataset->GetRasterCount() >= 3) {
        have_color = true;
      } else {
        RCLCPP_WARN_STREAM(get_logger(), "Failed to open color_path " << color_path << "; publishing without color");
      }
    }

    for (const auto& cell : map_->getElevationCells()) {
      grid_map_geo_msgs::msg::QuadtreeCell cell_msg;
      cell_msg.min_corner.x = static_cast<float>(cell.min_corner.x());
      cell_msg.min_corner.y = static_cast<float>(cell.min_corner.y());
      cell_msg.min_corner.z = 0.0f;
      cell_msg.size = static_cast<float>(cell.size);
      cell_msg.elevation = cell.value;
      if (have_color) {
        const Eigen::Vector2d world_min = cell.min_corner + world_center;
        sampleAverageColor(color_dataset.get(), color_geo_transform, world_min, cell.size, cell_msg.color);
      }
      structure_msg_.cells.push_back(cell_msg);
    }

    auto timer_callback = [this]() -> void {
      auto msg = grid_map::GridMapRosConverter::toMessage(map_->getGridMap());
      if (msg) {
        msg->header.stamp = now();
        map_pub_->publish(std::move(msg));
      }

      structure_msg_.header.stamp = now();
      structure_pub_->publish(structure_msg_);
    };
    timer_ = this->create_wall_timer(5s, timer_callback);

    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.frame_id = map_->getCoordinateName();
    static_transform_stamped.child_frame_id = map_->getGridMap().getFrameId();
    static_transform_stamped.transform.translation.x = map_origin.x();
    static_transform_stamped.transform.translation.y = map_origin.y();
    static_transform_stamped.transform.translation.z = 0.0;
    static_transform_stamped.transform.rotation.x = 0.0;
    static_transform_stamped.transform.rotation.y = 0.0;
    static_transform_stamped.transform.rotation.z = 0.0;
    static_transform_stamped.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(static_transform_stamped);
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
  rclcpp::Publisher<grid_map_geo_msgs::msg::QuadtreeStructure>::SharedPtr structure_pub_;
  std::shared_ptr<WaveletTerrainMap> map_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  grid_map_geo_msgs::msg::QuadtreeStructure structure_msg_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaveletQuadtreeMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
