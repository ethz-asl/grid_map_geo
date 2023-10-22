target_sources(${PROJECT_NAME} PRIVATE grid_map_geo.cpp)


find_package(rclcpp REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
add_executable(test_tif_loader)
target_sources(test_tif_loader PRIVATE test_tif_loader.cpp)
target_link_libraries(test_tif_loader PRIVATE ${PROJECT_NAME} rclcpp::rclcpp ${grid_map_msgs_TARGETS})
ament_target_dependencies(${PROJECT_NAME} grid_map_ros)