# grid_map_geo

[![Build Test](https://github.com/ethz-asl/grid_map_geo/actions/workflows/build_test.yml/badge.svg?branch=ros2)](https://github.com/ethz-asl/grid_map_geo/actions/workflows/build_test.yml)
[![Doxygen Build](https://github.com/ethz-asl/grid_map_geo/actions/workflows/doxygen_build.yml/badge.svg?branch=ros2)](https://github.com/ethz-asl/grid_map_geo/actions/workflows/doxygen_build.yml)
[![Style Checks](https://github.com/ethz-asl/grid_map_geo/actions/workflows/check_style.yml/badge.svg?branch=ros2)](https://github.com/ethz-asl/grid_map_geo/actions/workflows/check_style.yml)

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
```
source install/setup.bash
ros2 launch grid_map_geo load_tif_launch.xml
```

## Earth-scale elevation maps (wavelet quadtree)

For terrain too large to load as a single dense GeoTIFF (e.g. several hundred km
across), `grid_map_geo` can compress a DSM into a multi-resolution wavelet quadtree
store on disk, and load a bounded working-set region from it instead of the whole
raster at once. See `HashedWaveletQuadtree` for how the compression works.

### 1. Generate a tile store

```bash
source install/setup.bash
ros2 run grid_map_geo generate_wavelet_quadtree \
  --input <path-to-dem.tif> \
  --output <output-directory>
```

This writes `elevation.wavelet_quadtree`, `variance.wavelet_quadtree`, and
`extent.txt` (the source raster's exact center and extent) into
`<output-directory>`. Run with `--help` to see all options (`--tree-height`,
`--prior-variance`).

By default this is lossless. To also merge locally-uniform regions into larger
cells, pass a real, physically-meaningful tolerance in meters via `--max-error` —
e.g. your vehicle's own characteristic size — which guarantees every stored
elevation stays within that many meters of the source value:

```bash
ros2 run grid_map_geo generate_wavelet_quadtree \
  --input <path-to-dem.tif> \
  --output <output-directory> \
  --max-error 3.0
```

### 2. Visualize it

```bash
source install/setup.bash
ros2 launch grid_map_geo load_wavelet_quadtree.launch.py \
  tile_store_dir:=<output-directory>
```

This loads the store's full extent (from `extent.txt`, no need to know it up
front) and opens RViz with the elevation surface
(`grid_map_rviz_plugin/GridMap`) and, overlaid on top, the quadtree's actual
multi-resolution cell structure (`grid_map_geo_rviz_plugin/QuadtreeStructure`)
— large cells over locally-uniform terrain, small cells wherever full detail is
kept.

To color the overlay from an orthomosaic instead of by cell size, pass
`color_path`:

```bash
ros2 launch grid_map_geo load_wavelet_quadtree.launch.py \
  tile_store_dir:=<output-directory> \
  color_path:=<path-to-orthomosaic.tif>
```

To materialize only a bounded region instead of the whole store (e.g. to preview
a vehicle's local working set), pass `center_x`/`center_y`/`extent_x`/`extent_y`
explicitly instead of leaving them at their defaults.

