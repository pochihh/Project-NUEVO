FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Core development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    nano \
    curl \
    lsb-release \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Gazebo (official Jazzy pairing)
RUN apt-get update && apt-get install -y \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

WORKDIR /ros2_ws
RUN mkdir -p src

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]