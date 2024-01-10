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
 * @brief Node to test planner in the view utility map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include <grid_map_msgs/msg/grid_map.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "grid_map_geo/grid_map_geo.hpp"

using namespace std::chrono_literals;

class MapPublisher : public rclcpp::Node {
 public:
  MapPublisher() : Node("map_publisher") {
    original_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map", 1);
    const std::string frame_id = this->declare_parameter("frame_id", "map");
    const std::string gdal_dataset_path = this->declare_parameter("gdal_dataset_path", ".");
    const std::string color_path = this->declare_parameter("gdal_dataset_color_path", COLOR_MAP_DEFAULT_PATH);
    rcl_interfaces::msg::ParameterDescriptor max_map_descriptor;
    max_map_descriptor.read_only = true;
    max_map_descriptor.description =
        "Maximum number of raster pixels able to be loaded. \
        Useful when working with large raster datasets to limit memory usage. \
        Set to 0 to disable limits (may cause runtime crash).";
    rcl_interfaces::msg::IntegerRange max_map_descriptor_int_range;

    max_map_descriptor_int_range.from_value = 0;
    max_map_descriptor_int_range.to_value = std::numeric_limits<int>::max();
    max_map_descriptor.integer_range.push_back(max_map_descriptor_int_range);

    const uint16_t max_map_width =
        std::clamp(this->declare_parameter("max_map_width", std::numeric_limits<int>::max(), max_map_descriptor),
                   max_map_descriptor_int_range.from_value, max_map_descriptor_int_range.to_value);
    const uint16_t max_map_height =
        std::clamp(this->declare_parameter("max_map_height", std::numeric_limits<int>::max(), max_map_descriptor),
                   max_map_descriptor_int_range.from_value, max_map_descriptor_int_range.to_value);

    RCLCPP_INFO_STREAM(get_logger(), "gdal_dataset_path: '" << gdal_dataset_path << "'");
    RCLCPP_INFO_STREAM(get_logger(), "gdal_dataset_color_path: '" << color_path << "'");
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    map_ = std::make_shared<GridMapGeo>(frame_id);
    map_->setMaxMapSizePixels(max_map_width, max_map_height);

    rcl_interfaces::msg::ParameterDescriptor origin_descriptor;
    origin_descriptor.read_only = true;
    origin_descriptor.description = "Map origin latitude (WGS-84) in degrees.";
    rcl_interfaces::msg::FloatingPointRange origin_range;

    origin_range.from_value = -90.0;
    origin_range.to_value = 90.0;
    origin_descriptor.floating_point_range.push_back(origin_range);

    static_assert(std::numeric_limits<double>::has_quiet_NaN == true, "Need quiet NaN for default value");
    const auto map_origin_latitude = std::clamp(
        this->declare_parameter("map_origin_latitude", std::numeric_limits<double>::quiet_NaN(), origin_descriptor),
        origin_range.from_value, origin_range.to_value);

    origin_range.from_value = -180.0;
    origin_range.to_value = 180.0;
    origin_descriptor.floating_point_range.at(0) = origin_range;

    origin_descriptor.description = "Map origin longitude (WGS-84) in degrees.";
    const auto map_origin_longitude = std::clamp(
        this->declare_parameter("map_origin_longitude", std::numeric_limits<double>::quiet_NaN(), origin_descriptor),
        origin_range.from_value, origin_range.to_value);

    map_ = std::make_shared<GridMapGeo>();
    map_->setMaxMapSizePixels(max_map_width, max_map_height);
    map_->setGlobalOrigin(ESPG::WGS84, Eigen::Vector3d(map_origin_longitude, map_origin_latitude, 0.0));
    map_->Load(gdal_dataset_path, color_path);

    auto timer_callback = [this]() -> void {
      auto msg = grid_map::GridMapRosConverter::toMessage(map_->getGridMap());
      if (msg) {
        msg->header.stamp = now();
        original_map_pub_->publish(std::move(msg));
      }
    };
    timer_ = this->create_wall_timer(5s, timer_callback);
    ESPG epsg;
    Eigen::Vector3d map_origin;
    map_->getGlobalOrigin(epsg, map_origin);

    geometry_msgs::msg::TransformStamped static_transformStamped_;
    static_transformStamped_.header.frame_id = map_->getCoordinateName();
    static_transformStamped_.child_frame_id = map_->getGridMap().getFrameId();
    static_transformStamped_.transform.translation.x = map_origin.x();
    static_transformStamped_.transform.translation.y = map_origin.y();
    static_transformStamped_.transform.translation.z = 0.0;
    static_transformStamped_.transform.rotation.x = 0.0;
    static_transformStamped_.transform.rotation.y = 0.0;
    static_transformStamped_.transform.rotation.z = 0.0;
    static_transformStamped_.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(static_transformStamped_);
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr original_map_pub_;
  std::shared_ptr<GridMapGeo> map_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisher>());
  rclcpp::shutdown();
  return 0;
}
