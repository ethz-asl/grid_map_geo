# grid_map_geo

[![Build Test](https://github.com/ethz-asl/grid_map_geo/actions/workflows/build_test.yml/badge.svg?branch=ros2)](https://github.com/ethz-asl/grid_map_geo/actions/workflows/build_test.yml)
[![Doxygen Build](https://github.com/ethz-asl/grid_map_geo/actions/workflows/doxygen_build.yml/badge.svg?branch=ros2)](https://github.com/ethz-asl/grid_map_geo/actions/workflows/doxygen_build.yml)
[![Style Checks](https://github.com/ethz-asl/grid_map_geo/actions/workflows/check_style.yml/badge.svg?branch=ros2)](https://github.com/ethz-asl/grid_map_geo/actions/workflows/check_style.yml)

This package provides a georeferenced extension to the elevation map [grid_map](https://github.com/ANYbotics/grid_map) using [GDAL](https://gdal.org/), library for raster and vector geospatial data formats

![rviz_screenshot_2022_12_20-22_10_17](https://user-images.githubusercontent.com/5248102/208767846-6511a150-9924-44ea-8b6e-41b57407e26e.png)


**Authors: Jaeyoung Lim<br />
Affiliation: [ETH Zurich, Autonomous Systems Lab](https://asl.ethz.ch/)<br />**

## Setup

Install the dependencies. This package depends on GDAL, to read georeferenced images and DEM files.

Pull in dependencies using rosdep
```bash
source /opt/ros/humble/setup.bash
rosdep update
# Assuming the package is cloned in the src folder of a ROS workspace...
rosdep install --from-paths src --ignore-src -y
```

Build the package

```bash
colcon build --mixin release --packages-up-to grid_map_geo
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to grid_map_geo
```
## Docker Setup

If your operating system doesn't support ROS 2 humble, docker is a great alternative.

First of all, you have to build the project and create an  image like so:

```bash
## Assuimg you are in the correct project directory
docker build -t gmg .
```
To use a shortcut, you may use the following command:

```bash
## Assuimg you are in the correct project directory
make docker_build
```

After the image is created, copy and paste the following command to the terminal to run the image:

```bash
## Assuimg you are in the correct project directory
docker run -it --net=host --ipc=host --privileged --env="DISPLAY"  --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="${XAUTHORITY}:/root/.Xauthority"  --entrypoint /bin/bash gmg
```
To use a shortcut, you may use following command:

```bash
make docker_run
```
## Running the package at docker image

If you are in the docker image , this project is already sourced and the default launch file can be run as the following command;

```bash
ros2 launch grid_map_geo load_tif_launch.xml
```

## Running the package

The default launch file can be run as the following command. 
```bash
source install/setup.bash
ros2 launch grid_map_geo load_tif_launch.xml
```

To debug the map publisher in GDB:

```bash
colcon build --mixin debug --packages-up-to grid_map_geo --symlink-install
source install/setup.bash

# To debug the node under GDB
ros2 run --prefix 'gdb -ex run --args' \
grid_map_geo map_publisher --ros-args \
-p gdal_dataset_path:=install/grid_map_geo/share/grid_map_geo/resources/ap_srtm1.vrt

# To debug from the launch file
ros2 launch grid_map_geo load_vrt_launch.xml
```

**Note:** `grid_map_geo` uses asserts to catch coding errors; they are enabled by default when 
building in debug mode, and removed from release mode.

## Mixing data from different datums, resolutions, and raster sizes

`grid_map_geo` does not yet support automatically overlaying color data over elevation data from 
different sources. Currently, color data must be the same datum, resolution and size for both the 
DEM raster and the color raster.

As an example, `sertig_color.tif` color data can be requested loaded on the SRTM1 DEM at sertig:

```bash
ros2 launch grid_map_geo load_tif.launch.py params_file:=config/sargans_color_over_srtm1.yaml
# OR 
ros2 run --prefix 'gdb -ex run --args' grid_map_geo map_publisher --ros-args --params-file config/sargans_color_over_srtm1.yaml
```

These datasets are different sizes, different resultions, and use different datums.

Either of these ways of launching cause a floating point crash in `grid_map::getIndexFromLinearIndex`. 
This limitation may be addressed in a future version.
