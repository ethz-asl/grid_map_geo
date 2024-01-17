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
 * @brief Node to test planner in the view utiltiy map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "grid_map_geo/grid_map_geo.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "grid_map_msgs/msg/grid_map.h"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class MapPublisher : public rclcpp::Node {
 public:
  MapPublisher() : Node("map_publisher") {

    std::string file_path = this->declare_parameter("tif_path", ".");
    std::string color_path = this->declare_parameter("tif_color_path", ".");
    std::string frame_id = this->declare_parameter("frame_id", "map");
    std::string topic_name = this->declare_parameter("topic_name", "elevation_map");

    original_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(topic_name, 1);

    RCLCPP_INFO_STREAM(get_logger(), "file_path " << file_path);
    RCLCPP_INFO_STREAM(get_logger(), "color_path " << color_path);
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    map_ = std::make_shared<GridMapGeo>();
    map_->Load(file_path, color_path);
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

    geometry_msgs::msg::TransformStamped static_transformStamped;
    // static_transformStamped.header.frame_id = map_->getCoordinateName();
    static_transformStamped.child_frame_id = map_->getGridMap().getFrameId();
    static_transformStamped.transform.translation.x = map_origin(0);
    static_transformStamped.transform.translation.y = map_origin(1);
    static_transformStamped.transform.translation.z = 0.0;
    static_transformStamped.transform.rotation.x = 0.0;
    static_transformStamped.transform.rotation.y = 0.0;
    static_transformStamped.transform.rotation.z = 0.0;
    static_transformStamped.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(static_transformStamped);
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
