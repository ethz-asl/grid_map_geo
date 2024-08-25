/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim, ASL, ETH Zurich, Switzerland
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
 * 3. Neither the name terrain-navigation nor the names of its contributors may be
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

#ifndef GRID_MAP_GEO_H
#define GRID_MAP_GEO_H

#include <tf2_ros/transform_broadcaster.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <iostream>

#include "transform.hpp"
struct Location {
  ESPG espg{ESPG::WGS84};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

class GridMapGeo {
 public:
  GridMapGeo(const std::string& frame_id = "map");
  virtual ~GridMapGeo();

  /**
   * @brief Get the Grid Map object
   *
   * @return grid_map::GridMap&
   */
  grid_map::GridMap& getGridMap() { return grid_map_; }

  /**
   * @brief Set the Grid Map object from an external process
   *
   * @param map
   */
  void setGridMap(grid_map::GridMap& map) { grid_map_ = map; }

  /**
   * @brief Set the Global Coordinate of the Origin of the map
   *
   * @param src_coord
   * @param origin
   */
  void setGlobalOrigin(ESPG src_coord, const Eigen::Vector3d origin);

  /**
   * @brief Get the Global Origin object
   *
   * @param src_coord
   * @param origin
   */
  void getGlobalOrigin(ESPG& src_coord, Eigen::Vector3d& origin) {
    src_coord = maporigin_.espg;
    origin = maporigin_.position;
  };

  /**
   * @brief Get the name of the coordinate frame of the dataset
   *
   * @return std::string
   */
  std::string getCoordinateName() { return coordinate_name_; };

  /**
   * @brief Overloading terrain loading with only elevation
   *
   * @param map_path Path to dsm path (Supported formats are *.tif)
   */
  bool Load(const std::string& map_path) { return Load(map_path, ""); }

  /**
   * @brief Helper function for loading terrain from path
   *
   * @param map_path Path to dsm path (Supported formats are *.tif)
   * @param color_map_path  Path to color raster files to visualize terrain texture (Supported formats are *.tif)
   * @return true Successfully loaded terrain
   * @return false Failed to load terrain
   */
  bool Load(const std::string& map_path, const std::string& color_map_path);

  /**
   * @brief Initialize grid map from a geotiff file
   *
   * @param path Path to dsm path (Supported formats are *.tif)
   * @return true Successfully loaded terrain
   * @return false Failed to load terrain
   */
  bool initializeFromGeotiff(const std::string& path);

  bool initializeFromVrt(const std::string& path, const Eigen::Vector2d& map_center, Eigen::Vector2d& extent);

  /**
   * @brief Load a color layer from a geotiff file (orthomosaic)
   *
   * @param path Path to orthomosaic  path (Supported formats are *.tif)
   * @return true Successfully loaded terrain
   * @return false Failed to load terrain
   */
  bool addColorFromGeotiff(const std::string& path);

  /**
   * @brief Load ROI layer from geotiff
   *
   * @param path
   * @return true
   * @return false
   */
  bool addLayerFromGeotiff(const std::string& layer_name, const std::string& path);

  /**
   * @brief Add layer using distance transform
   *
   * @param surface_distance surface distance to compute. If smaller than zero, will compute the distance transform
   * beneath the reference layer
   * @param layer_name
   * @return true
   * @return false
   */
  bool AddLayerDistanceTransform(const double surface_distance, const std::string& layer_name,
                                 std::string reference_layer = "elevation");

  /**
   * @brief Add layer using distance transform
   *
   * @param surface_distance surface distance to compute. If smaller than zero, will compute the distance transform
   * beneath the reference layer
   * @param layer_name
   * @return true
   * @return false
   */
  bool AddLayerHorizontalDistanceTransform(const double surface_distance, const std::string& layer_name,
                                           std::string reference_layer = "elevation");

  /**
   * @brief Add layer with an offset
   *
   * @param offset_distance
   * @param layer_name
   * @return true
   * @return false
   */
  bool AddLayerOffset(const double offset_distance, const std::string& layer_name);

  /**
   * @brief Compute normal vectors of the surface
   *
   * @param layer_name
   * @param reference_layer
   */
  void AddLayerNormals(std::string reference_layer);

  geometry_msgs::msg::TransformStamped static_transformStamped_;

 protected:
  grid_map::GridMap grid_map_;
  Location maporigin_;
  std::string frame_id_{""};
  std::string coordinate_name_{""};
};
#endif  // GRID_MAP_GEO_H
