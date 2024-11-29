<launch>
    <arg name="rviz" default="true"/>
    <arg name="location" default="sargans"/>
    <node pkg="tf2_ros" exec="static_transform_publisher" name="world_map" args="--frame-id world --child-frame-id map"/>

    <node pkg="grid_map_geo" exec="map_publisher" name="map_publisher" namespace="grid_map_geo" output="screen">
        <param name="gdal_dataset_path" value="$(find-pkg-share grid_map_geo)/resources/sargans.tif"/>
        <param name="gdal_dataset_color_path" value="$(find-pkg-share grid_map_geo)/resources/sargans_color.tif"/>

        <!-- These values match sargans.tif, but could be increased with no effect on sargans -->
        <param name="max_map_width" value="748"/>
        <param name="max_map_height" value="1220"/>
    </node>

    <group if="$(var rviz)">
        <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share grid_map_geo)/rviz/config.rviz"/>
    </group>
</launch>
