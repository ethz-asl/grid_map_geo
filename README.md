# grid_map_geo
This package provides a georeferenced extension to the elevation map [grid_map](https://github.com/ANYbotics/grid_map)

## Setup
Install the dependencies. This package depends on gdal, to read georeferenced images and GeoTIFF files.
```
apt install libgdal-dev
```
Configure the catkin workspace
```
catkin config --extend "/opt/ros/noetic"
catkin config --merge-devel
```

Pull in dependencies using rosinstall / rosdep
```
wstool init src src/grid_map_geo/dependencies.rosinstall
wstool update -t src -j4
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro noetic
```

Build the package
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
catkin build -j$(nproc) -l$(nproc) grid_map_geo
```
## Running the package
The default launch file can be run as the following command. 
```
roslaunch grid_map_geo test_grid_map_geo.launch
```
