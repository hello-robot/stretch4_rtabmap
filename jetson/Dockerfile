# Image for the Jetson half of navigation_jetson.launch.py (JetPack 6 / L4T r36.4, Orin).
#
# Base: Ubuntu 24.04 and CUDA 12.8 for Jetson. GTSAM,
# gtsam_points, GLIM and RTAB-Map are built into the mounted workspace by build.sh.
#
#   docker build -t stretch4_rtabmap_gpu:jazzy ~/stretch_gpu/jetson
ARG BASE_IMAGE=dustynv/ros:jazzy-desktop-r36.4.0-cu128-24.04
FROM ${BASE_IMAGE}

# The current ROS apt key (the base's has expired).
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list

# Removes the base's OpenCV 4.11 (it conflicts with Ubuntu's libopencv-dev, which cv_bridge needs)
# and upgrades the base's ROS packages so every ROS package is from the same release.
RUN apt-get update \
 && apt-get purge -y $(dpkg-query -W -f '${Package}\n' 'opencv-*' 2>/dev/null || true) \
 && old=$(dpkg-query -W -f '${Package}\n' 'ros-jazzy-*' 2>/dev/null || true) \
 && if [ -n "$old" ]; then apt-get install -y --only-upgrade $old; fi \
 && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-cv-bridge ros-jazzy-image-geometry ros-jazzy-image-transport \
    ros-jazzy-laser-geometry ros-jazzy-message-filters ros-jazzy-diagnostic-updater \
    ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
    ros-jazzy-tf2-ros ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-nav-msgs ros-jazzy-nav2-msgs ros-jazzy-stereo-msgs ros-jazzy-visualization-msgs \
    ros-jazzy-std-srvs ros-jazzy-rclcpp-components ros-jazzy-octomap-msgs \
    ros-jazzy-rosbag2-cpp ros-jazzy-rosbag2-storage ros-jazzy-rosbag2-compression \
    ros-jazzy-libpointmatcher ros-jazzy-libg2o \
    ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python ros-jazzy-rosidl-default-generators \
    ros-jazzy-ros-environment ros-jazzy-launch-ros \
    liboctomap-dev libproj-dev libsqlite3-dev libtbb-dev libmetis-dev \
    libspdlog-dev libfmt-dev libomp-dev libeigen3-dev libpcl-dev libboost-all-dev libopencv-dev \
    zlib1g-dev python3-yaml python3-numpy python3-opencv python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# Same middleware as the robot computer.
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

ENTRYPOINT []
CMD ["bash"]
