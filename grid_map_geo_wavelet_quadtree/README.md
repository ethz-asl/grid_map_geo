# grid_map_geo_wavelet_quadtree

Earth-scale elevation map representation for [grid_map_geo](../grid_map_geo):
a self-contained, hashed, Haar-wavelet-compressed quadtree (`HashedWaveletQuadtree`),
plus `WaveletTerrainMap` (a `GridMapGeo` that materializes a bounded working-set
grid map from a wavelet quadtree store instead of a single monolithic GeoTIFF).

For terrain too large to load as a single dense GeoTIFF (e.g. several hundred km
across), this package compresses a DSM into a multi-resolution wavelet quadtree
store on disk, and loads a bounded working-set region from it instead of the
whole raster at once. See `HashedWaveletQuadtree` for how the compression works.

## 1. Generate a tile store

```bash
source install/setup.bash
ros2 run grid_map_geo_wavelet_quadtree generate_wavelet_quadtree \
  --input <path-to-dem.tif> \
  --output <output-directory>
```

This writes `elevation.wavelet_quadtree`, `variance.wavelet_quadtree`, and
`extent.txt` (the source raster's exact center, extent, and compression error
bound) into `<output-directory>`. Run with `--help` to see all options
(`--tree-height`, `--prior-variance`).

By default this is lossless. To also merge locally-uniform regions into larger
cells, pass a real, physically-meaningful tolerance in meters via `--max-error` —
e.g. your vehicle's own characteristic size — which guarantees every stored
elevation stays within that many meters of the source value:

```bash
ros2 run grid_map_geo_wavelet_quadtree generate_wavelet_quadtree \
  --input <path-to-dem.tif> \
  --output <output-directory> \
  --max-error 3.0
```

## 2. Visualize it

```bash
source install/setup.bash
ros2 launch grid_map_geo_wavelet_quadtree load_wavelet_quadtree.launch.py \
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
ros2 launch grid_map_geo_wavelet_quadtree load_wavelet_quadtree.launch.py \
  tile_store_dir:=<output-directory> \
  color_path:=<path-to-orthomosaic.tif>
```

To materialize only a bounded region instead of the whole store (e.g. to preview
a vehicle's local working set), pass `center_x`/`center_y`/`extent_x`/`extent_y`
explicitly instead of leaving them at their defaults.

## 3. Benchmark access time

```bash
source install/setup.bash
ros2 run grid_map_geo_wavelet_quadtree evaluate_access_time \
  --tif <path-to-dem.tif> \
  --tile-store <output-directory>
```

Reports per-query elevation lookup latency for the dense GeoTIFF-backed
`grid_map::GridMap` and for `HashedWaveletQuadtree::getCellValue()` directly
against the store, plus load time and memory usage for each. Run with `--help`
to see all options.
