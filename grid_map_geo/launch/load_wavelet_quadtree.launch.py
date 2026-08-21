from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description.

    Companion to load_tif.launch.py, for manually comparing (e.g. side by
    side in RViz, or by diffing the published grid_map_msgs/GridMap
    messages) a wavelet-quadtree-backed load against the monolithic-GeoTIFF
    path over the same source dataset -- run
    `generate_wavelet_quadtree --input <tif> --output <dir>` first to
    produce the store this launch file reads.
    """

    pkg_grid_map_geo = get_package_share_directory("grid_map_geo")

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_map",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "map",
        ],
    )

    wavelet_quadtree_loader = Node(
        package="grid_map_geo",
        namespace="grid_map_geo",
        executable="test_wavelet_quadtree_loader",
        name="wavelet_quadtree_loader",
        parameters=[
            {"tile_store_dir": LaunchConfiguration("tile_store_dir")},
            {"center_x": LaunchConfiguration("center_x")},
            {"center_y": LaunchConfiguration("center_y")},
            {"extent_x": LaunchConfiguration("extent_x")},
            {"extent_y": LaunchConfiguration("extent_y")},
            {"query_height": LaunchConfiguration("query_height")},
            {"color_path": LaunchConfiguration("color_path")},
        ],
        output="screen",
        emulate_tty=True,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f'{Path(pkg_grid_map_geo) / "rviz" / "config.rviz"}'],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    default_store_dir = "sargans_wavelet_quadtree"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            DeclareLaunchArgument(
                "tile_store_dir",
                default_value=f'{Path(pkg_grid_map_geo) / "resources" / default_store_dir}',
                description="Directory containing elevation.wavelet_quadtree and variance.wavelet_quadtree.",
            ),
            DeclareLaunchArgument(
                "center_x",
                default_value="0.0",
                description="World-frame (projected meters) X center of the region to materialize. "
                "Ignored unless extent_x/extent_y are both also given (see below).",
            ),
            DeclareLaunchArgument(
                "center_y",
                default_value="0.0",
                description="World-frame (projected meters) Y center of the region to materialize. "
                "Ignored unless extent_x/extent_y are both also given (see below).",
            ),
            DeclareLaunchArgument(
                "extent_x",
                default_value="0.0",
                description="Extent in X, in meters, of the region to materialize. Leave at 0 (with "
                "extent_y) to load the store's own full extent (extent.txt, written by "
                "generate_wavelet_quadtree) instead of a bounded region -- no boundary strip, "
                "no need to know the store's size up front.",
            ),
            DeclareLaunchArgument(
                "extent_y",
                default_value="0.0",
                description="Extent in Y, in meters, of the region to materialize. See extent_x.",
            ),
            DeclareLaunchArgument(
                "query_height",
                default_value="0",
                description="Wavelet quadtree resolution level to reconstruct at (0 = finest).",
            ),
            DeclareLaunchArgument(
                "color_path",
                default_value="",
                description="Optional path to an orthomosaic GeoTIFF to color the quadtree_structure "
                "overlay's cells from. Leave empty to color by cell size instead.",
            ),
            static_transform_publisher,
            wavelet_quadtree_loader,
            rviz,
        ]
    )
