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

#ifndef GRID_MAP_GEO_TRANSFORM_H
#define GRID_MAP_GEO_TRANSFORM_H

#include <Eigen/Dense>
#include <array>
#include <cassert>

enum class ESPG { ECEF = 4978, WGS84 = 4326, WGS84_32N = 32632, CH1903_LV03 = 21781 };

struct Location {
  ESPG espg{ESPG::WGS84};
  // <east (lat), north (lng),  up (alt)>
  //! @todo Switch to geographic_msgs/GeoPoint to make x-y not confusing?
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

/**
 * @brief Helper function for transforming using gdal
 *
 * @param src_coord
 * @param tgt_coord
 * @param source_coordinates
 * @return Eigen::Vector3d
 */
Eigen::Vector3d transformCoordinates(ESPG src_coord, ESPG tgt_coord, const Eigen::Vector3d source_coordinates);

/**
 * @brief Helper function for transforming using gdal
 *
 * @param src_coord
 * @param wkt
 * @param source_coordinates
 * @return Eigen::Vector3d
 */
Eigen::Vector3d transformCoordinates(ESPG src_coord, const std::string wkt, const Eigen::Vector3d source_coordinates);

struct Corners {
  ESPG espg{ESPG::WGS84};
  Eigen::Vector2d top_left{Eigen::Vector2d::Zero()};
  Eigen::Vector2d top_right{Eigen::Vector2d::Zero()};
  Eigen::Vector2d bottom_left{Eigen::Vector2d::Zero()};
  Eigen::Vector2d bottom_right{Eigen::Vector2d::Zero()};
};

/**
 * @brief Helper function converting from image to geo coordinates
 *
 * @ref
 https://gdal.org/tutorials/geotransforms_tut.html#transformation-from-image-coordinate-space-to-georeferenced-coordinate-space
 * @see GDALApplyGeoTransform
 *
 * @param geoTransform The 6-element Geo transform
 * @param imageCoords The image-coordinates <row, column>, also called <pixel, line>

 * @return The geo-coordinates in <x, y>
 */
inline Eigen::Vector2d imageToGeo(const std::array<double, 6> geoTransform, const Eigen::Vector2i imageCoords) {
  const auto x_pixel = imageCoords.x();
  const auto y_line = imageCoords.y();

  return {geoTransform.at(0) + x_pixel * geoTransform.at(1) + y_line * geoTransform.at(2),
          geoTransform.at(3) + x_pixel * geoTransform.at(4) + y_line * geoTransform.at(5)};
}

/**
 * @brief Helper function converting from geo to image coordinates. Assumes no rotation.
 *        Uses the assumption that GT2 and GT4 are zero
 *
 * @ref
 * https://gis.stackexchange.com/questions/384221/calculating-inverse-polynomial-transforms-for-pixel-sampling-when-map-georeferen
 * @see GDALApplyGeoTransform
 *
 * @param geoTransform The 6-element forward Geo transform
 * @param geoCoords The geo-coordinates in <x, y>
 *
 * @return The image-coordinates in <row, column>, also called <pixel, line>
 */
inline Eigen::Vector2d geoToImageNoRot(const std::array<double, 6>& geoTransform, const Eigen::Vector2d geoCoords) {
  assert(geoTransform.at(2) == 0);  // assume no rotation
  assert(geoTransform.at(4) == 0);  // assume no rotation

  return {(geoCoords.x() - geoTransform.at(0)) / geoTransform.at(1),
          (geoCoords.y() - geoTransform.at(3)) / geoTransform.at(5)};
}

#endif
