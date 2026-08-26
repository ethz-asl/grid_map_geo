from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import ExecutableInPackage


def generate_launch_description():
    """Generate a launch description.

    Runs the offline generate_wavelet_quadtree CLI (see
    src/generate_wavelet_quadtree.cpp) against a source GeoTIFF from the
    thermal_navigation_ros package's resources directory, keyed by
    `location` -- e.g. `location:=wsmr` looks up
    thermal_navigation_ros/resources/wsmr.tif. Pass `input` directly to
    bypass that lookup entirely (e.g. for a GeoTIFF that isn't in that
    package). The resulting store defaults to the same
    share/grid_map_geo_wavelet_quadtree/resources/<location>_wavelet_quadtree
    path that load_wavelet_quadtree.launch.py's `tile_store_dir` defaults to
    for the same location, so the two launch files chain without extra args.
    """

    pkg_thermal_navigation_ros = get_package_share_directory("thermal_navigation_ros")
    pkg_grid_map_geo_wavelet_quadtree = get_package_share_directory("grid_map_geo_wavelet_quadtree")

    generate_wavelet_quadtree = ExecuteProcess(
        cmd=[
            ExecutableInPackage(package="grid_map_geo_wavelet_quadtree", executable="generate_wavelet_quadtree"),
            "--input",
            LaunchConfiguration("input"),
            "--output",
            LaunchConfiguration("output"),
            "--prior-variance",
            LaunchConfiguration("prior_variance"),
            "--tree-height",
            LaunchConfiguration("tree_height"),
            "--max-error",
            LaunchConfiguration("max_error"),
        ],
        output="screen",
        emulate_tty=True,
    )

    default_location = "wsmr"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "location",
                default_value=default_location,
                description="Name of a thermal_navigation_ros/resources/<location>.tif GeoTIFF "
                "to ingest. Ignored if `input` is also given.",
            ),
            DeclareLaunchArgument(
                "input",
                default_value=[
                    f'{Path(pkg_thermal_navigation_ros) / "resources"}/',
                    LaunchConfiguration("location"),
                    ".tif",
                ],
                description="Full path to the source elevation GeoTIFF. Overrides `location`.",
            ),
            DeclareLaunchArgument(
                "output",
                default_value=[
                    f'{Path(pkg_grid_map_geo_wavelet_quadtree) / "resources"}/',
                    LaunchConfiguration("location"),
                    "_wavelet_quadtree",
                ],
                description="Output directory; will contain elevation.wavelet_quadtree and "
                "variance.wavelet_quadtree. Defaults to the location-keyed store dir that "
                "load_wavelet_quadtree.launch.py's tile_store_dir also defaults to.",
            ),
            DeclareLaunchArgument(
                "prior_variance",
                default_value="25.0",
                description="Uniform variance assigned to the static prior, i.e. how much to "
                "trust the source DSM before any onboard measurement has been fused in.",
            ),
            DeclareLaunchArgument(
                "tree_height",
                default_value="6",
                description="Wavelet quadtree height per hashed block (6 = 64x64 leaf cells per block).",
            ),
            DeclareLaunchArgument(
                "max_error",
                default_value="0.0",
                description="Lossy compression tolerance, in meters (0 = disabled, i.e. "
                "exact/lossless). When set, guarantees every stored elevation is within this "
                "many meters of the source value -- pick a real, physically-meaningful "
                "tolerance (e.g. your vehicle's own characteristic size), not a generic "
                "performance knob.",
            ),
            generate_wavelet_quadtree,
        ]
    )
