cmake_minimum_required(VERSION 3.8)
project(grid_map_geo)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(GDAL REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)


include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/grid_map_geo.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs geometry_msgs grid_map_core grid_map_ros grid_map_msgs)
target_include_directories(${PROJECT_NAME} PUBLIC "${Eigen3_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} ${GDAL_LIBRARY})

add_executable(test_tif_loader
  src/test_tif_loader.cpp
)
ament_target_dependencies(test_tif_loader rclcpp std_msgs)
target_include_directories(test_tif_loader PUBLIC "${Eigen3_INCLUDE_DIRS}")
target_link_libraries(test_tif_loader ${PROJECT_NAME} ${GDAL_LIBRARY})

install(TARGETS
  test_tif_loader
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
