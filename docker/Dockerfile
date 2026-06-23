FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Build tooling + mcap storage plugin (bag was recorded on Jazzy/mcap).
# Most localization deps (pcl, tf2, diagnostic_msgs, rclcpp_lifecycle) ship with
# the desktop image; remaining ones are resolved by rosdep at build time.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        git \
        wget \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-pcl-conversions \
        ros-jazzy-pcl-ros \
        ros-jazzy-pybind11-vendor \
    && rm -rf /var/lib/apt/lists/*

# rosdep is pre-initialized in osrf images; just refresh the cache.
RUN rosdep update --rosdistro jazzy || true

# Auto-source ROS + the workspace overlay for every interactive shell.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ws
CMD ["bash"]
