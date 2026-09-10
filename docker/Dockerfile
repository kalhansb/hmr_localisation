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
        ros-jazzy-robot-localization \
        libeigen3-dev \
        libomp-dev \
    && rm -rf /var/lib/apt/lists/*

# small_gicp: GICP/VGICP registration backend for lidar_localization_ros2
# (registration_method: SMALL_VGICP / SMALL_GICP). Not in the Jazzy binary index, so
# it is built from source at a pinned tag and installed system-wide -- the fork's
# find_package(small_gicp CONFIG) then resolves regardless of colcon build order.
# Header-only core + small helper lib; no PCL/TBB/-march=native so the image stays
# portable across x86 and Jetson. ldconfig registers libsmall_gicp.so in /usr/local/lib.
ARG SMALL_GICP_REF=57c1106daf83c2c79ee0c58a9c7ed0032298ff4e   # v1.0.1
RUN git clone https://github.com/koide3/small_gicp.git /tmp/small_gicp \
    && git -C /tmp/small_gicp checkout ${SMALL_GICP_REF} \
    && cmake -S /tmp/small_gicp -B /tmp/small_gicp/build \
         -DCMAKE_BUILD_TYPE=Release -DBUILD_HELPER=ON \
         -DBUILD_WITH_PCL=OFF -DBUILD_WITH_TBB=OFF -DBUILD_WITH_MARCH_NATIVE=OFF \
         -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
    && cmake --build /tmp/small_gicp/build --parallel "$(nproc)" \
    && cmake --install /tmp/small_gicp/build \
    && ldconfig \
    && rm -rf /tmp/small_gicp

# rosdep is pre-initialized in osrf images; just refresh the cache.
RUN rosdep update --rosdistro jazzy || true

# Auto-source ROS + the workspace overlay for every interactive shell.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ws
CMD ["bash"]
