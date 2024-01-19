# Use the official ROS 2 humble base image
FROM ros:humble

# Set the working directory
WORKDIR /root/ros2_ws/src

COPY . .
# Install additional dependencies
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gcc \
    git \
    libxml2-dev \
    libxslt-dev \
    python3 \
    ros-humble-ament-cmake-clang-format\
    ros-humble-rviz2

# Set environment variables
ENV ROS_VERSION 2
WORKDIR /ros2_workspace
RUN . /opt/ros/humble/setup.sh
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y

RUN . /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to grid_map_geo

# Source the ROS2 setup file
RUN echo "source /ros2_workspace/install/setup.bash" >> ~/.bashrc
# Expose the default ROS 2 communication ports
EXPOSE 11311

# Run a default command, e.g., starting a bash shell
CMD ["bash"]