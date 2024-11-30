/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim. All rights reserved.
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

#include <gdal/ogr_geometry.h>
#include <gdal/ogr_spatialref.h>

#include <grid_map_geo/grid_map_geo.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

constexpr int ESPG_WGS84 = 4326;
constexpr int ESPG_CH1903_LV03 = 21781;

void transformCoordinates(int src_coord, const double &x, const double &y, const double &z, int tgt_coord, double &x_t,
                          double &y_t, double &z_t) {
  OGRSpatialReference source, target;
  source.importFromEPSG(src_coord);
  target.importFromEPSG(tgt_coord);

  OGRPoint p;
  p.setX(x);
  p.setY(y);
  p.setZ(z);
  p.assignSpatialReference(&source);

  p.transformTo(&target);
  x_t = p.getX();
  y_t = p.getY();
  z_t = p.getZ();
  return;
}

void LoadTerrainFromVrt(std::string path, const Eigen::Vector3d &query_position, const Eigen::Vector2d &offset,
                        grid_map::GridMap &map, grid_map::GridMap &vrt_map_object, rclcpp::Node::SharedPtr node_ptr) {
  std::shared_ptr<GridMapGeo> vrt_map = std::make_shared<GridMapGeo>();

  Eigen::Vector3d query_position_lv03 = query_position;
  /// Convert LV03 to WGS84
  Eigen::Vector3d map_center_wgs84;  // Map center in WGS84
  transformCoordinates(ESPG_CH1903_LV03, query_position_lv03(0), query_position_lv03(1), query_position_lv03(2),
                       ESPG_WGS84, map_center_wgs84.x(), map_center_wgs84.y(), map_center_wgs84.z());

  std::cout << "Loading VRT Map:" << std::endl;
  std::cout << "  - map_center_wgs84:" << map_center_wgs84.transpose() << std::endl;
  /// TODO: Discover extent from corners
  Eigen::Vector2d extent_wgs84(0.5, 0.5);
  vrt_map->initializeFromVrt(path, map_center_wgs84.head(2), extent_wgs84, node_ptr);
  std::cout << "  - Success!" << std::endl;

  /// TODO: Loaded VRT map
  std::cout << "Reprojecting map" << std::endl;
  Eigen::Vector2d extent_lv03(12000.0, 12000.0);
  double resolution{100.0};
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  map.setGeometry(extent_lv03, resolution, position);
  map.add("elevation");
  map.setFrameId("map");
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index = *iterator;
    Eigen::Vector2d cell_position;  // Position of cell relative to map coordinates
    map.getPosition(index, cell_position);
    auto cell_position_lv03 = cell_position + query_position_lv03.head(2);  // Position of cell in CH1903/LV03
    double dummy;
    Eigen::Vector2d cell_position_wgs84;
    transformCoordinates(ESPG_CH1903_LV03, cell_position_lv03(0), cell_position_lv03(1), cell_position_lv03(2),
                         ESPG_WGS84, cell_position_wgs84.x(), cell_position_wgs84.y(), dummy);
    // std::cout << "    - cell_position_wgs84:" << cell_position_wgs84.transpose() << std::endl;

    Eigen::Vector2d local_wgs84 = cell_position_wgs84 - map_center_wgs84.head(2);
    double tmp = local_wgs84(0);
    local_wgs84(0) = local_wgs84(1);
    local_wgs84(1) = tmp;
    // std::cout << "      - local_wgs84:" << local_wgs84.transpose() << std::endl;
    auto elevation = vrt_map->getGridMap().atPosition("elevation", local_wgs84);
    map.atPosition("elevation", cell_position) = elevation;
  }
  map.setPosition(offset);
  vrt_map_object = vrt_map->getGridMap();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("terrain_loader");

  auto map_pub = node->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", rclcpp::QoS(1).transient_local());
  auto map_pub2 = node->create_publisher<grid_map_msgs::msg::GridMap>("grid_map2", rclcpp::QoS(1).transient_local());

  auto const path = node->declare_parameter("terrain_path", "resources/cadastre.tif");

  Eigen::Vector3d test_position = Eigen::Vector3d(783199.15, 187585.10, 0.0);

  for (int i = 0; i < 4; i++) {
    Eigen::Vector3d offset = static_cast<double>(i) * Eigen::Vector3d(2500.0, 2500.0, 0.0);
    Eigen::Vector3d query_position = test_position + offset;

    grid_map::GridMap map;
    grid_map::GridMap vrt_map;
    LoadTerrainFromVrt(path, query_position, offset.head(2), map, vrt_map, node);
    std::cout << "i: " << i << " offset: " << offset.transpose() << std::endl;

    // grid_map_msgs::msg::GridMap msg;
    auto msg = grid_map::GridMapRosConverter::toMessage(map);
    if (msg) {
      map_pub->publish(std::move(msg));
    }

    auto msg2 = grid_map::GridMapRosConverter::toMessage(vrt_map);
    if (msg2) {
      map_pub2->publish(std::move(msg2));
    }
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
