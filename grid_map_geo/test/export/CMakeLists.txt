# Add gtest based cpp test target and link libraries
cmake_minimum_required(VERSION 3.14)
project(grid_map_export_test)
find_package(grid_map_geo REQUIRED)
add_executable(main main.cpp)
target_link_libraries(main PRIVATE grid_map_geo::grid_map_geo)

