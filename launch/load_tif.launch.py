from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
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

    # map publisher node
    map_publisher = Node(
        package="grid_map_geo",
        namespace="grid_map_geo",
        executable="map_publisher",
        name="map_publisher",
        parameters=[
            {"gdal_dataset_path": LaunchConfiguration("gdal_dataset_path")},
            {"gdal_dataset_color_path": LaunchConfiguration("gdal_dataset_color_path")},
        ],
        output="screen",
        emulate_tty=True,
        # condition=LaunchConfigurationEquals(LaunchConfiguration("params_file"), "")
    )
    
    # map publisher node with params file
    map_publisher_with_param_file = Node(
        package="grid_map_geo",
        namespace="grid_map_geo",
        executable="map_publisher",
        name="map_publisher",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
        emulate_tty=True,
        condition=LaunchConfigurationNotEquals(LaunchConfiguration("params_file"), "")
    )

    # rviz node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f'{Path(pkg_grid_map_geo) / "rviz" / "config.rviz"}'],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    default_location = "sargans"
    default_gdal_dataset = "sargans.tif"
    default_gdal_color_dataset = "sargans_color.tif"
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
                "gdal_dataset_path",
                default_value=f'{Path(pkg_grid_map_geo) / "resources" / default_gdal_dataset}',
                description="Full path to the elevation map file.",
            ),
            DeclareLaunchArgument(
                "gdal_dataset_color_path",
                default_value=f'{Path(pkg_grid_map_geo) / "resources" / default_gdal_color_dataset}',
                description="Full path to the elevation texture file.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="YAML parameter file path.",
            ),
            
            static_transform_publisher,
            map_publisher,
            # map_publisher_with_param_file,
            rviz,
        ]
    )
    

