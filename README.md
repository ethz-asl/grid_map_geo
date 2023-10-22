# grid_map_geo
[![Build Test](https://github.com/ethz-asl/grid_map_geo/actions/workflows/build_test.yml/badge.svg)](https://github.com/ethz-asl/grid_map_geo/actions/workflows/build_test.yml)

This package provides a georeferenced extension to the elevation map [grid_map](https://github.com/ANYbotics/grid_map) using [GDAL](https://gdal.org/), library for raster and vector geospatial data formats

![rviz_screenshot_2022_12_20-22_10_17](https://user-images.githubusercontent.com/5248102/208767846-6511a150-9924-44ea-8b6e-41b57407e26e.png)


**Authors: Jaeyoung Lim<br />
Affiliation: [ETH Zurich, Autonomous Systems Lab](https://asl.ethz.ch/)<br />**
## Setup
Install the dependencies. This package depends on gdal, to read georeferenced images and GeoTIFF files.

Pull in dependencies using rosdep
```
source /opt/ros/humble/setup.bash
rosdep update
# Assuming the package is cloned in the src folder of a ROS workspace...
rosdep install --from-paths src --ignore-src -y
```

Build the package
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False --packages-up-to grid_map_geo
```
## Running the package
The default launch file can be run as the following command. 
```
source install/setup.bash
ros2 launch grid_map_geo load_tif_launch.xml
```
