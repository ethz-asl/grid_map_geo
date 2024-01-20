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
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to grid_map_geo
```
## Docker Setup

If your operating system doesn't support ROS 2 humble, docker is a great alternative.

First of all, you have to build the project and create an  image like so:

```bash
## Assuimg you are in the correct project directory
docker build -t gmg .
```
To use shortcut , you may use following command;

```bash
## Assuimg you are in the correct project directory
make docker_build
```

After create image, copy and paste the following command to terminal to run image;

```bash
## Assuimg you are in the correct project directory
docker run -it --net=host --ipc=host --privileged --env="DISPLAY"  --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="${XAUTHORITY}:/root/.Xauthority"  --entrypoint /bin/bash gmg
```
To use shortcut, you may use following commad;

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
```
source install/setup.bash
ros2 launch grid_map_geo load_tif_launch.xml
```

