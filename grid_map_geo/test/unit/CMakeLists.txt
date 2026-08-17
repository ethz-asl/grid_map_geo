# Add gtest based cpp test target and link libraries
find_package(ament_cmake_gtest)
ament_add_gtest(${PROJECT_NAME}-test
  main.cpp
  test_grid_map_geo.cpp
  test_hashed_wavelet_quadtree.cpp
  test_grid_map_geo_wavelet_quadtree.cpp
)

target_link_libraries(${PROJECT_NAME}-test
  ${PROJECT_NAME}
)
