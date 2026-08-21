# One multi-resolution leaf cell of a grid_map_geo HashedWaveletQuadtree,
# for visualizing the tree's actual compression structure (grid_map_geo_rviz_plugin
# QuadtreeStructure display) directly on top of the elevation surface. All
# fields are already in the *local* frame of the accompanying
# QuadtreeStructure message's header (see that message), matching the
# convention grid_map_geo's elevation_map topic already uses: world position
# minus the map's real-world center.

# Local-frame min (south-west) corner of this cell's square footprint.
geometry_msgs/Point32 min_corner

# Side length, in meters, of this (square) cell -- large for locally-uniform
# terrain, down to the tree's finest resolution where full detail is kept.
float32 size

# This cell's (already-reconstructed) elevation.
float32 elevation

# Orthomosaic color sampled over this cell's footprint, if available.
# color.a == 0 (the default) means no color data was provided -- the
# display falls back to coloring by cell size instead. A populated color
# always has a == 1.0, so the two cases are unambiguous.
std_msgs/ColorRGBA color
