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

#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_p.h>
#include <gdal/ogr_spatialref.h>

#include <Eigen/Dense>

enum class ESPG { ECEF = 4978, WGS84 = 4326, WGS84_32N = 32632, CH1903_LV03 = 21781 };

/**
 * @brief Helper function for transforming using gdal
 *
 * @param src_coord
 * @param tgt_coord
 * @param source_coordinates
 * @return Eigen::Vector3d
 */
inline Eigen::Vector3d transformCoordinates(ESPG src_coord, ESPG tgt_coord, const Eigen::Vector3d source_coordinates) {
  OGRSpatialReference source, target;
  source.importFromEPSG(static_cast<int>(src_coord));
  target.importFromEPSG(static_cast<int>(tgt_coord));

  OGRPoint p;
  p.setX(source_coordinates(0));
  p.setY(source_coordinates(1));
  p.setZ(source_coordinates(2));
  p.assignSpatialReference(&source);

  p.transformTo(&target);
  Eigen::Vector3d target_coordinates(p.getX(), p.getY(), p.getZ());
  return target_coordinates;
}

/**
 * @brief Helper function for transforming using gdal
 *
 * @param src_coord
 * @param wkt
 * @param source_coordinates
 * @return Eigen::Vector3d
 */
inline Eigen::Vector3d transformCoordinates(ESPG src_coord, const std::string wkt, const Eigen::Vector3d source_coordinates) {
  OGRSpatialReference source, target;
  char* wkt_string = const_cast<char*>(wkt.c_str());
  source.importFromEPSG(static_cast<int>(src_coord));
  target.importFromWkt(&wkt_string);

  OGRPoint p;
  p.setX(source_coordinates(0));
  p.setY(source_coordinates(1));
  p.setZ(source_coordinates(2));
  p.assignSpatialReference(&source);

  p.transformTo(&target);
  Eigen::Vector3d target_coordinates(p.getX(), p.getY(), p.getZ());
  return target_coordinates;
}

#endif
