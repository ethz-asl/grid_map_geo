cmake_minimum_required(VERSION 2.8.12)
project(grid_map_geo)
add_definitions(-std=c++11)

find_package(GDAL)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    eigen_catkin
    grid_map_core
    grid_map_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_map_geo
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/grid_map_geo.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GeographicLib_LIBRARIES} ${GDAL_LIBRARY})

add_executable(test_tif_loader
  src/test_tif_loader.cpp
)
add_dependencies(test_tif_loader ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(test_tif_loader ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})

add_executable(terrain_loader
  src/terrain_loader.cpp
)
add_dependencies(terrain_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GDAL_LIBRARY})
target_link_libraries(terrain_loader ${PROJECT_NAME} ${catkin_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
    # Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
                                          test/test_grid_map_geo.cpp)

    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
                                                 ${catkin_LIBRARIES}
                                                 ${YAML_CPP_LIBRARIES} ${GDAL_LIBRARY} ${OpenCV_LIBRARIES})
    endif()

endif()
