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

#include "grid_map_geo/grid_map_geo.h"

#include <ros/ros.h>
#include "grid_map_ros/GridMapRosConverter.hpp"

void MapPublishOnce(ros::Publisher &pub, grid_map::GridMap &map) {
  map.setTimestamp(ros::Time::now().toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);
  pub.publish(message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gridmap_geo_tif_loader");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::string frame_id, file_path, color_path, topic_name;
  nh_private.param<std::string>("frame_id", frame_id, "map");
  nh_private.param<std::string>("tif_path", file_path, "");
  nh_private.param<std::string>("color_path", color_path, "");
  nh_private.param<std::string>("topic_name", topic_name, "elevation_map");

  ros::Publisher original_map_pub = nh.advertise<grid_map_msgs::GridMap>(topic_name, 1, true);

  std::shared_ptr<GridMapGeo> map = std::make_shared<GridMapGeo>(frame_id);
  map->Load(file_path, color_path);

  while (true) {
    /// TODO: Publish gridmap
    MapPublishOnce(original_map_pub, map->getGridMap());
    ros::Duration(1.0).sleep();
  }

  ros::spin();
  return 0;
}
