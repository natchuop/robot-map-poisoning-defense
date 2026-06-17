# syntax=docker/dockerfile:1.7
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pip \
    python3-scipy \
    git \
    less \
    nano \
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-dwb-controller \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-navfn-planner \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-robot-localization \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-webots-ros2 \
    ros-jazzy-webots-ros2-driver \
    ros-jazzy-webots-ros2-turtlebot \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Source ROS 2 (includes webots_ros2) and workspace in interactive shells
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo '[[ -f /workspace/install/setup.bash ]] && source /workspace/install/setup.bash' >> /etc/bash.bashrc

CMD ["/bin/bash"]
