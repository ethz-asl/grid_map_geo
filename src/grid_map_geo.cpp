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

/**
 * @brief Terain map representation
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "grid_map_geo/grid_map_geo.hpp"

#include <array>
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#if __APPLE__
#include <cpl_string.h>
#include <gdal.h>
#include <gdal_priv.h>
#include <ogr_p.h>
#include <ogr_spatialref.h>
#else
#include <gdal/cpl_string.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_p.h>
#include <gdal/ogr_spatialref.h>
#endif

GridMapGeo::GridMapGeo() {}

GridMapGeo::GridMapGeo(const std::string frame_id) { frame_id_ = frame_id; }

GridMapGeo::~GridMapGeo() {}

bool GridMapGeo::Load(const std::string &map_path, const std::string color_map_path) {
  bool loaded = initializeFromGeotiff(map_path);
  if (!color_map_path.empty()) {  // Load color layer if the color path is nonempty
    bool color_loaded = addColorFromGeotiff(color_map_path);
  }
  if (!loaded) return false;
  return true;
}

bool GridMapGeo::initializeFromGeotiff(const std::string &path) {
  GDALAllRegister();
  const auto dataset = GDALDatasetUniquePtr(GDALDataset::FromHandle(GDALOpen(path.c_str(), GA_ReadOnly)));
  if (!dataset) {
    std::cout << "Failed to open" << std::endl;
    return false;
  }
  std::cout << std::endl << "Loading GeoTIFF file for gridmap" << std::endl;

  double originX, originY, pixelSizeX, pixelSizeY;
  std::array<double, 6> geoTransform;
  if (dataset->GetGeoTransform(geoTransform.data()) == CE_None) {
    originX = geoTransform[0];
    originY = geoTransform[3];
    pixelSizeX = geoTransform[1];
    pixelSizeY = geoTransform[5];
  } else {
    std::cout << "Failed read geotransform" << std::endl;
    return false;
  }

  const char *pszProjection = dataset->GetProjectionRef();

  if (strlen(pszProjection) == 0) {
    // https://gdal.org/user/raster_data_model.html#coordinate-system
    std::cerr << std::endl << "Wkt ProjectionRef is unknown!" << std::endl;
    return false;
  }

  std::cout << std::endl << "Wkt ProjectionRef: " << pszProjection << std::endl;

  const OGRSpatialReference *spatial_ref = dataset->GetSpatialRef();
  std::string name_coordinate = spatial_ref->GetAttrValue("geogcs");
  // Get image metadata
  unsigned width = dataset->GetRasterXSize();
  unsigned height = dataset->GetRasterYSize();
  double resolution = pixelSizeX;
  std::cout << "Width: " << width << " Height: " << height << " Resolution: " << resolution << std::endl;

  // pixelSizeY is negative because the origin of the image is the north-east corner and positive
  // Y pixel coordinates go towards the south
  const double lengthX = resolution * width;
  const double lengthY = resolution * height;
  grid_map::Length length(lengthX, lengthY);

  double mapcenter_e = originX + pixelSizeX * width * 0.5;
  double mapcenter_n = originY + pixelSizeY * height * 0.5;
  maporigin_.espg = ESPG::CH1903_LV03;
  maporigin_.position = Eigen::Vector3d(mapcenter_e, mapcenter_n, 0.0);

  static_transformStamped.header.frame_id = name_coordinate;
  static_transformStamped.child_frame_id = frame_id_;
  static_transformStamped.transform.translation.x = mapcenter_e;
  static_transformStamped.transform.translation.y = mapcenter_n;
  static_transformStamped.transform.translation.z = 0.0;
  static_transformStamped.transform.rotation.x = 0.0;
  static_transformStamped.transform.rotation.y = 0.0;
  static_transformStamped.transform.rotation.z = 0.0;
  static_transformStamped.transform.rotation.w = 1.0;

  Eigen::Vector2d position{Eigen::Vector2d::Zero()};

  grid_map_.setGeometry(length, resolution, position);
  /// TODO: Use TF for geocoordinates
  grid_map_.setFrameId(frame_id_);
  grid_map_.add("elevation");
  GDALRasterBand *elevationBand = dataset->GetRasterBand(1);

  std::vector<float> data(width * height, 0.0f);
  elevationBand->RasterIO(GF_Read, 0, 0, width, height, &data[0], width, height, GDT_Float32, 0, 0);

  grid_map::Matrix &layer_elevation = grid_map_["elevation"];
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    // TODO: This may be wrong if the pixelSizeY > 0
    int x = width - 1 - gridMapIndex(0);
    int y = gridMapIndex(1);

    layer_elevation(x, y) = data[gridMapIndex(0) + width * gridMapIndex(1)];
  }

  return true;
}

bool GridMapGeo::addColorFromGeotiff(const std::string &path) {
  GDALAllRegister();
  const auto dataset = GDALDatasetUniquePtr(GDALDataset::FromHandle(GDALOpen(path.c_str(), GA_ReadOnly)));
  if (!dataset) {
    std::cout << "Failed to open" << std::endl;
    return false;
  }
  std::cout << std::endl << "Loading color layer from GeoTIFF file for gridmap" << std::endl;

  double originX, originY, pixelSizeX, pixelSizeY;
  std::array<double, 6> geoTransform;
  if (dataset->GetGeoTransform(geoTransform.data()) == CE_None) {
    originX = geoTransform[0];
    originY = geoTransform[3];
    pixelSizeX = geoTransform[1];
    pixelSizeY = geoTransform[5];
  } else {
    std::cout << "Failed read geotransform" << std::endl;
    return false;
  }

  // Get image metadata
  unsigned width = dataset->GetRasterXSize();
  unsigned height = dataset->GetRasterYSize();
  double resolution = pixelSizeX;
  std::cout << "Width: " << width << " Height: " << height << " Resolution: " << resolution << std::endl;

  // pixelSizeY is negative because the origin of the image is the north-east corner and positive
  // Y pixel coordinates go towards the south
  const double lengthX = resolution * width;
  const double lengthY = resolution * height;
  grid_map::Length length(lengthX, lengthY);

  grid_map_.add("color");
  GDALRasterBand *raster_red = dataset->GetRasterBand(1);
  GDALRasterBand *raster_green = dataset->GetRasterBand(2);
  GDALRasterBand *raster_blue = dataset->GetRasterBand(3);

  std::vector<uint16_t> data_red(width * height, 0.0f);
  std::vector<uint16_t> data_green(width * height, 0.0f);
  std::vector<uint16_t> data_blue(width * height, 0.0f);

  raster_red->RasterIO(GF_Read, 0, 0, width, height, &data_red[0], width, height, GDT_UInt16, 0, 0);
  raster_green->RasterIO(GF_Read, 0, 0, width, height, &data_green[0], width, height, GDT_UInt16, 0, 0);
  raster_blue->RasterIO(GF_Read, 0, 0, width, height, &data_blue[0], width, height, GDT_UInt16, 0, 0);

  grid_map::Matrix &layer_color = grid_map_["color"];
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    /// TODO: This may be wrong if the pixelSizeY > 0
    int x = width - 1 - gridMapIndex(0);
    int y = gridMapIndex(1);
    Eigen::Vector3i colorVector;
    colorVector(0) = data_red[gridMapIndex(0) + width * gridMapIndex(1)];
    colorVector(1) = data_green[gridMapIndex(0) + width * gridMapIndex(1)];
    colorVector(2) = data_blue[gridMapIndex(0) + width * gridMapIndex(1)];
    grid_map::colorVectorToValue(colorVector, layer_color(x, y));
  }
  return true;
}

bool GridMapGeo::addLayerFromGeotiff(const std::string &layer_name, const std::string &path) {
  GDALAllRegister();
  const auto dataset = GDALDatasetUniquePtr(GDALDataset::FromHandle(GDALOpen(path.c_str(), GA_ReadOnly)));
  if (!dataset) {
    std::cout << "Failed to open" << std::endl;
    return false;
  }
  std::cout << std::endl << "Loading color layer from GeoTIFF file for gridmap" << std::endl;

  double originX, originY, pixelSizeX, pixelSizeY;
  std::array<double, 6> geoTransform;
  if (dataset->GetGeoTransform(geoTransform.data()) == CE_None) {
    originX = geoTransform[0];
    originY = geoTransform[3];
    pixelSizeX = geoTransform[1];
    pixelSizeY = geoTransform[5];
  } else {
    std::cout << "Failed read geotransform" << std::endl;
    return false;
  }

  // Get image metadata
  unsigned width = dataset->GetRasterXSize();
  unsigned height = dataset->GetRasterYSize();
  double resolution = pixelSizeX;
  std::cout << "Width: " << width << " Height: " << height << " Resolution: " << resolution << std::endl;

  // pixelSizeY is negative because the origin of the image is the north-east corner and positive
  // Y pixel coordinates go towards the south
  const double lengthX = resolution * width;
  const double lengthY = resolution * height;
  grid_map::Length length(lengthX, lengthY);

  grid_map_.add(layer_name);
  GDALRasterBand *raster_blue = dataset->GetRasterBand(3);

  std::vector<float> data(width * height, 0.0f);
  raster_blue->RasterIO(GF_Read, 0, 0, width, height, &data[0], width, height, GDT_Float32, 0, 0);

  grid_map::Matrix &layer_roi = grid_map_[layer_name];
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;
    /// TODO: This may be wrong if the pixelSizeY > 0
    int x = width - 1 - gridMapIndex(0);
    int y = gridMapIndex(1);
    layer_roi(x, y) = data[gridMapIndex(0) + width * gridMapIndex(1)];
  }
  return true;
}

bool GridMapGeo::AddLayerDistanceTransform(const double surface_distance, const std::string &layer_name,
                                           std::string reference_layer) {
  grid_map_.add(layer_name);

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index MapIndex = *iterator;
    Eigen::Vector3d center_pos;
    grid_map_.getPosition3(reference_layer, MapIndex, center_pos);
    for (grid_map::CircleIterator submapIterator(grid_map_, center_pos.head(2), std::abs(surface_distance));
         !submapIterator.isPastEnd(); ++submapIterator) {
      const grid_map::Index SubmapIndex = *submapIterator;
      Eigen::Vector3d cell_position;
      grid_map_.getPosition3(reference_layer, SubmapIndex, cell_position);
      double distance_2d = (cell_position.head(2) - center_pos.head(2)).norm();
      double elevation_difference = std::sqrt(std::pow(surface_distance, 2) - std::pow(distance_2d, 2));
      if (surface_distance > 0.0) {
        if (center_pos(2) < cell_position(2) + elevation_difference) {
          center_pos(2) = cell_position(2) + elevation_difference;
        }
      } else {
        if (center_pos(2) > cell_position(2) - elevation_difference) {
          center_pos(2) = cell_position(2) - elevation_difference;
        }
      }
    }
    grid_map_.at(layer_name, MapIndex) = center_pos(2);
  }
  return true;
}

bool GridMapGeo::AddLayerHorizontalDistanceTransform(const double surface_distance, const std::string &layer_name,
                                                     std::string reference_layer) {
  grid_map_.add(layer_name);

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index MapIndex = *iterator;
    Eigen::Vector3d center_pos;
    grid_map_.getPosition3(reference_layer, MapIndex, center_pos);
    const Eigen::Vector2d center_pos_2d(center_pos(0), center_pos(1));
    grid_map_.at(layer_name, MapIndex) = center_pos(2);  // elevation of reference layer
    for (grid_map::CircleIterator submapIterator(grid_map_, center_pos_2d, std::abs(surface_distance));
         !submapIterator.isPastEnd(); ++submapIterator) {
      const grid_map::Index SubmapIndex = *submapIterator;
      Eigen::Vector3d cell_position;
      grid_map_.getPosition3(reference_layer, SubmapIndex, cell_position);
      double distance = cell_position(2) - grid_map_.at(layer_name, MapIndex);
      if ((surface_distance > 0.0 && distance > 0.0) || (surface_distance < 0.0 && distance < 0.0)) {
        grid_map_.at(layer_name, MapIndex) = grid_map_.at(layer_name, MapIndex) + distance;
      }
    }
  }
  return true;
}

bool GridMapGeo::AddLayerOffset(const double offset_distance, const std::string &layer_name) {
  grid_map_.add(layer_name);

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index MapIndex = *iterator;
    Eigen::Vector3d center_pos;
    grid_map_.getPosition3("elevation", MapIndex, center_pos);
    grid_map_.at(layer_name, MapIndex) = center_pos(2) + offset_distance;  // elevation
  }
  return true;
}

void GridMapGeo::setGlobalOrigin(ESPG src_coord, const Eigen::Vector3d origin) {
  // Transform global origin into CH1903 / LV03 coordinates
  maporigin_.espg = src_coord;
  maporigin_.position = origin;
}

void GridMapGeo::AddLayerNormals(const std::string reference_layer) {
  grid_map_.add(reference_layer + "_normal_x");
  grid_map_.add(reference_layer + "_normal_y");
  grid_map_.add(reference_layer + "_normal_z");

  grid_map::Matrix &layer_elevation = grid_map_[reference_layer];
  grid_map::Matrix &layer_normal_x = grid_map_[reference_layer + "_normal_x"];
  grid_map::Matrix &layer_normal_y = grid_map_[reference_layer + "_normal_y"];
  grid_map::Matrix &layer_normal_z = grid_map_[reference_layer + "_normal_z"];

  unsigned width = grid_map_.getSize()(0);
  unsigned height = grid_map_.getSize()(1);
  double resolution = grid_map_.getResolution();
  // Compute normals from elevation map
  // Surface normal calculation from: https://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;

    /// TODO: Verify normal by visualization
    int x = gridMapIndex(0);
    int y = height - 1 - gridMapIndex(1);

    float sx = layer_elevation(x < width - 1 ? x + 1 : x, y) - layer_elevation(x > 0 ? x - 1 : x, y);
    if (x == 0 || x == width - 1) sx *= 2;

    float sy = layer_elevation(x, y < height - 1 ? y + 1 : y) - layer_elevation(x, y > 0 ? y - 1 : y);
    if (y == 0 || y == height - 1) sy *= 2;

    Eigen::Vector3d normal(Eigen::Vector3d(sx, sy, 2 * resolution));
    normal.normalize();

    layer_normal_x(x, y) = normal(0);
    layer_normal_y(x, y) = normal(1);
    layer_normal_z(x, y) = normal(2);
  }
}
