cmake_minimum_required(VERSION 3.14.4)
project(grid_map_geo_rviz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

cmake_policy(VERSION 3.16)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(grid_map_geo_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)

# Explicitly listed (rather than relying purely on AUTOMOC's implicit
# header-following from the .cpp files' includes, which does not reliably
# pick up Q_OBJECT headers included via a subdirectory-prefixed path here)
# and added directly to the target's sources, mirroring the working pattern
# in the in-repo mav_planning_rviz plugin.
set(${PROJECT_NAME}_headers_to_moc
  include/grid_map_geo_rviz_plugin/quadtree_structure_display.hpp
)

add_library(${PROJECT_NAME} SHARED
  src/quadtree_structure_display.cpp
  src/quadtree_structure_visual.cpp
  ${${PROJECT_NAME}_headers_to_moc}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  grid_map_geo_msgs
  pluginlib
  rclcpp
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
)

target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Core Qt5::Gui Qt5::Widgets)

target_compile_definitions(${PROJECT_NAME} PRIVATE "QUADTREE_STRUCTURE_DISPLAY_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(grid_map_geo_msgs pluginlib rviz_common rviz_ogre_vendor rviz_rendering)

ament_package()
