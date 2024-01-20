# Use the official ROS 2 humble base image
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} as deps

# Set the working directory
WORKDIR /root/ros2_ws

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN --mount=type=bind,source=package.xml,target=src/grid_map_geo/package.xml \ 
    apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Run a default command, e.g., starting a bash shell
CMD ["bash"]

FROM deps as builder
ARG CMAKE_BUILD_TYPE=Release

RUN source /opt/ros/${ROS_DISTRO}/setup.bash
COPY . .
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} --packages-up-to grid_map_geo

# Source the ROS2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["bash"]

