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

#include <grid_map_msgs/msg/grid_map.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grid_map_geo_msgs/msg/quadtree_structure.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

#include "grid_map_geo/grid_map_geo.hpp"

using namespace std::chrono_literals;

class WaveletQuadtreeMapPublisher : public rclcpp::Node {
 public:
  WaveletQuadtreeMapPublisher() : Node("wavelet_quadtree_map_publisher") {
    std::string tile_store_dir = this->declare_parameter("tile_store_dir", ".");
    double center_x = this->declare_parameter("center_x", 0.0);
    double center_y = this->declare_parameter("center_y", 0.0);
    double extent_x = this->declare_parameter("extent_x", 500.0);
    double extent_y = this->declare_parameter("extent_y", 500.0);
    int query_height = this->declare_parameter("query_height", 0);
    std::string frame_id = this->declare_parameter("frame_id", "map");

    map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map", 1);
    structure_pub_ = this->create_publisher<grid_map_geo_msgs::msg::QuadtreeStructure>("quadtree_structure", 1);

    RCLCPP_INFO_STREAM(get_logger(), "tile_store_dir " << tile_store_dir);
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    map_ = std::make_shared<GridMapGeo>(frame_id);
    if (!map_->LoadFromWaveletQuadtree(tile_store_dir, Eigen::Vector2d(center_x, center_y),
                                       grid_map::Length(extent_x, extent_y), query_height)) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to load wavelet quadtree store from " << tile_store_dir);
      return;
    }

    auto timer_callback = [this]() -> void {
      auto msg = grid_map::GridMapRosConverter::toMessage(map_->getGridMap());
      if (msg) {
        msg->header.stamp = now();
        map_pub_->publish(std::move(msg));
      }

      grid_map_geo_msgs::msg::QuadtreeStructure structure_msg;
      structure_msg.header.stamp = now();
      structure_msg.header.frame_id = map_->getGridMap().getFrameId();
      for (const auto &cell : map_->getElevationCells()) {
        grid_map_geo_msgs::msg::QuadtreeCell cell_msg;
        cell_msg.min_corner.x = static_cast<float>(cell.min_corner.x());
        cell_msg.min_corner.y = static_cast<float>(cell.min_corner.y());
        cell_msg.min_corner.z = 0.0f;
        cell_msg.size = static_cast<float>(cell.size);
        cell_msg.elevation = cell.value;
        structure_msg.cells.push_back(cell_msg);
      }
      structure_pub_->publish(structure_msg);
    };
    timer_ = this->create_wall_timer(5s, timer_callback);

    EPSG epsg;
    Eigen::Vector3d map_origin;
    map_->getGlobalOrigin(epsg, map_origin);

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
  std::shared_ptr<GridMapGeo> map_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaveletQuadtreeMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
