cmake_minimum_required(VERSION 3.14.4)
project(grid_map_geo)

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

# Vendor wavemap's core library (dimension/value-generic Haar-wavelet tree +
# hashed sparse block storage) for the wavelet quadtree elevation map, before
# any find_package() calls below. grid_map_core's exported CMake config adds
# a directory-scoped Eigen plugin compile definition
# (-DEIGEN_MATRIXBASE_PLUGIN=grid_map_core/eigen_plugins/FunctorsPlugin.hpp);
# since that's a directory (not target) property, it would otherwise leak
# into wavemap_core's unrelated translation units if fetched afterward.
# We only need library/cpp (ROS-independent), not wavemap's ROS1
# interfaces/examples. Built as a private, in-tree static dependency: not
# installed or exported, since only grid_map_geo's implementation uses it
# (matches the GDAL-private link convention below).
include(FetchContent)
set(ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(ENABLE_BENCHMARKING OFF CACHE BOOL "" FORCE)
set(GENERATE_WAVEMAP_INSTALL_RULES OFF CACHE BOOL "" FORCE)
# Fetch Boost::preprocessor and glog directly instead of relying on
# find_package(Boost ... COMPONENTS headers): in an ament/colcon-sourced
# environment this can fail to promote Boost::headers to global scope
# (target already imported in a different directory scope).
set(USE_SYSTEM_BOOST OFF CACHE BOOL "" FORCE)
# Build wavemap_core as a static lib so it's simply absorbed into
# libgrid_map_geo.so as a private implementation detail, with no separate
# runtime shared-library or install/export story of its own.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(wavemap
  GIT_REPOSITORY https://github.com/ethz-asl/wavemap.git
  GIT_TAG v2.2.1
  SOURCE_SUBDIR library/cpp
)
FetchContent_MakeAvailable(wavemap)
# grid_map_geo itself must stay shared regardless of the override above (its
# own add_library() call below is explicit SHARED, so this restore isn't
# strictly required, but avoids leaving BUILD_SHARED_LIBS=OFF as a surprising
# side effect for anything configured later in this same build).
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)

# wavemap_core sets `-march=native` and `EIGEN_INITIALIZE_MATRICES_BY_NAN` as
# PUBLIC (i.e. also INTERFACE) compile options/definitions on itself
# (cmake/wavemap-extras.cmake). PRIVATE-linking wavemap_core into
# grid_map_geo does not stop those from propagating into grid_map_geo's own
# compilation (PRIVATE only controls re-export to *further* downstream
# consumers) -- and applying `-march=native` here, while the system's
# precompiled grid_map_core/grid_map_ros .so were not built with it, is a
# real Eigen ABI hazard: it caused a heap-buffer-overflow (confirmed via
# ASan) inside Eigen's aligned allocator when grid_map::GridMap objects
# crossed the .so boundary. Strip wavemap_core's INTERFACE compile
# options/definitions so only wavemap_core's own translation units (already
# compiled by the time this runs) are affected, not ours.
set_target_properties(wavemap_core PROPERTIES INTERFACE_COMPILE_OPTIONS "")
set_target_properties(wavemap_core PROPERTIES INTERFACE_COMPILE_DEFINITIONS "")

find_package(ament_cmake REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
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
# Explicitly SHARED (rather than following the ambient BUILD_SHARED_LIBS,
# which this workspace otherwise leaves unset/STATIC): the wavelet quadtree
# implementation privately links a static wavemap_core, and only a SHARED
# grid_map_geo keeps that dependency fully opaque to consumers/install
# exports (a STATIC grid_map_geo would require wavemap_core - and its own
# transitive static deps - to be installed/exported too, which defeats the
# point of vendoring it as a private implementation detail).
add_library(${PROJECT_NAME} SHARED
  src/grid_map_geo.cpp
  src/transform.cpp
  src/hashed_wavelet_quadtree.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen PRIVATE GDAL::GDAL wavemap_core)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  grid_map_core
  grid_map_ros
  tf2_ros
)

add_executable(test_tif_loader
  src/test_tif_loader.cpp
)

target_link_libraries(test_tif_loader PUBLIC
  ${PROJECT_NAME}
)

add_executable(test_wavelet_quadtree_loader
  src/test_wavelet_quadtree_loader.cpp
)

target_link_libraries(test_wavelet_quadtree_loader PUBLIC
  ${PROJECT_NAME}
)

# Offline preprocessing CLI (GeoTIFF -> wavelet quadtree store); not a ROS
# node, no rclcpp dependency.
add_executable(generate_wavelet_quadtree
  src/generate_wavelet_quadtree.cpp
)

target_link_libraries(generate_wavelet_quadtree PUBLIC
  ${PROJECT_NAME}
)

# Fix for yaml_cpp not resolving correctly on macOS
# https://github.com/ethz-asl/grid_map_geo/pull/59#discussion_r1474669370
if (APPLE)
  find_package(yaml_cpp_vendor REQUIRED)
  link_directories(${yaml_cpp_vendor_LIBRARY_DIRS})
endif()

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
ament_export_dependencies(Eigen3 GDAL grid_map_core grid_map_msgs grid_map_ros tf2_ros rclcpp)

install(
  TARGETS
  test_tif_loader
  test_wavelet_quadtree_loader
  generate_wavelet_quadtree
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

install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME}/
)

# Test
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test/unit)
endif()

ament_package()
