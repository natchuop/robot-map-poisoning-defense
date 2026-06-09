FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    nano \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    ros-jazzy-rviz2 \
    ros-jazzy-webots-ros2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Source ROS 2 (includes webots_ros2) and workspace in interactive shells
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo '[[ -f /workspace/install/setup.bash ]] && source /workspace/install/setup.bash' >> /etc/bash.bashrc

CMD ["/bin/bash"]
