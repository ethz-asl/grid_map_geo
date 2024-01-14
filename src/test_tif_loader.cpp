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

using namespace std::chrono_literals;

class MapPublisher : public rclcpp::Node {
 public:
  MapPublisher() : Node("map_publisher") {
    original_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map", 1);

    std::string file_path = this->declare_parameter("tif_path", ".");
    std::string color_path = this->declare_parameter("tif_color_path", "");

    RCLCPP_INFO_STREAM(get_logger(), "file_path " << file_path);
    RCLCPP_INFO_STREAM(get_logger(), "color_path " << color_path);

    map_ = std::make_shared<GridMapGeo>();
    map_->Load(file_path, false, color_path);
    timer_ = this->create_wall_timer(5s, std::bind(&MapPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto msg = grid_map::GridMapRosConverter::toMessage(map_->getGridMap());
    original_map_pub_->publish(std::move(msg));
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr original_map_pub_;
  std::shared_ptr<GridMapGeo> map_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPublisher>());
  rclcpp::shutdown();
  return 0;
}
