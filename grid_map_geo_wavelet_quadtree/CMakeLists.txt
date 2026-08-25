cmake_minimum_required(VERSION 3.14.4)
project(grid_map_geo_wavelet_quadtree)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set policy for 3.16 for CMP0076 defaulting to ON
cmake_policy(VERSION 3.16)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_geo REQUIRED)
find_package(grid_map_geo_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(GDAL 3 REQUIRED)

# Reverse compatability for GDAL<3.5
# https://gdal.org/development/cmake.html
if(NOT TARGET GDAL::GDAL)
  add_library(GDAL::GDAL ALIAS ${GDAL_LIBRARY})
endif()

# Build
add_library(${PROJECT_NAME} SHARED
  src/hashed_wavelet_quadtree.cpp
  src/wavelet_terrain_map.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  grid_map_core
  grid_map_geo
)

# Offline preprocessing CLI (GeoTIFF -> wavelet quadtree store); not a ROS
# node, no rclcpp dependency. Reads the GDAL dataset directly (windowed
# tiles), so it needs GDAL itself.
add_executable(generate_wavelet_quadtree
  src/generate_wavelet_quadtree.cpp
)

target_link_libraries(generate_wavelet_quadtree PUBLIC
  ${PROJECT_NAME}
PRIVATE
  GDAL::GDAL
)

# Demo/visualization ROS node: republishes a grid_map materialized from a
# wavelet quadtree store, plus its multi-resolution cell structure. Also
# samples an orthomosaic color GeoTIFF directly, so it needs GDAL itself.
add_executable(test_wavelet_quadtree_loader
  src/test_wavelet_quadtree_loader.cpp
)

target_link_libraries(test_wavelet_quadtree_loader PUBLIC
  ${PROJECT_NAME}
PRIVATE
  GDAL::GDAL
)

ament_target_dependencies(test_wavelet_quadtree_loader PUBLIC
  grid_map_geo_msgs
  grid_map_msgs
  grid_map_ros
  tf2_ros
  rclcpp
)

# Offline benchmarking CLI (per-query elevation access time of the dense
# GeoTIFF-backed grid_map_geo vs. this package's wavelet quadtree store); not
# a ROS node, no rclcpp dependency.
add_executable(evaluate_access_time
  src/evaluate_access_time.cpp
)

target_link_libraries(evaluate_access_time PUBLIC
  ${PROJECT_NAME}
)

# Install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Everything in find_package goes here too, ordered alphabetically.
ament_export_dependencies(Eigen3 GDAL grid_map_core grid_map_geo grid_map_geo_msgs grid_map_msgs grid_map_ros tf2_ros rclcpp)

install(
  TARGETS
  generate_wavelet_quadtree
  test_wavelet_quadtree_loader
  evaluate_access_time
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Test
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test/unit)
endif()

ament_package()
