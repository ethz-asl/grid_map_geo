from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('grid_map_geo')
    tif_loader = Node(
        package='grid_map_geo',
        namespace='grid_map_geo',
        executable='test_tif_loader',
        name='tif_loader',
        output="screen",
        emulate_tty=True,
        parameters=[
                {"tif_path": "/home/jaeyoung/dev/terrain-models/models/hinwil.tif"},
                {"tif_color_path": "/home/jaeyoung/dev/terrain-models/models/hinwil_color.tif"},
        ]        
    )
    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(package_dir, 'launch/config.rviz')]]
    )
    return LaunchDescription([
        tif_loader,
        rviz
])
