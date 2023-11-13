import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description."""

    pkg_grid_map_geo = get_package_share_directory("grid_map_geo")

    # static transform node
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

    # tif loader node
    tif_loader = Node(
        package="grid_map_geo",
        namespace="grid_map_geo",
        executable="test_tif_loader",
        name="tif_loader",
        parameters=[
            {"tif_path": LaunchConfiguration("tif_path")},
            {"tif_color_path": LaunchConfiguration("tif_color_path")},
        ],
        output="screen",
        emulate_tty=True,
    )

    # rviz node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_grid_map_geo, "rviz", "config.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    default_location = "sargans"
    default_tif_file = "sargans.tif"
    default_tif_color_file = "sargans_color.tif"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            DeclareLaunchArgument(
                "location",
                default_value=default_location,
                description="Location.",
            ),
            DeclareLaunchArgument(
                "tif_path",
                default_value=os.path.join(
                    pkg_grid_map_geo, "resources", default_tif_file
                ),
                description="Full path to the elevation map file.",
            ),
            DeclareLaunchArgument(
                "tif_color_path",
                default_value=os.path.join(
                    pkg_grid_map_geo, "resources", default_tif_color_file
                ),
                description="Full path to the elevation texture file.",
            ),
            static_transform_publisher,
            tif_loader,
            rviz,
        ]
    )
