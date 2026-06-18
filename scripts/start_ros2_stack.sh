#!/bin/bash
set -eo pipefail

CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-/workspace}"
cd "$CONTAINER_WORKSPACE"
set +u
source /opt/ros/jazzy/setup.bash
set -u
rm -rf build/robot_patrol_node install/robot_patrol_node log/latest
echo "Building robot_patrol_node..."
colcon build --packages-select robot_patrol_node --symlink-install
set +u
source install/setup.bash
set -u
echo "ROS 2 workspace ready"

if [ "${RMPD_TEST_MODE:-mapping}" = "amcl" ]; then
    if [ -z "${RMPD_AMCL_MAP_YAML:-}" ]; then
        echo "RMPD_AMCL_MAP_YAML is required when RMPD_TEST_MODE=amcl" >&2
        exit 1
    fi
    START_CHECKPOINT_PATROL="${RMPD_START_CHECKPOINT_PATROL:-true}"
    echo "Launching AMCL + Nav2 stack with Webots GPS/IMU odom + LiDAR + static map: $RMPD_AMCL_MAP_YAML"
    exec ros2 launch robot_patrol_node nav2_with_amcl.launch.py \
        map_yaml:="$RMPD_AMCL_MAP_YAML" \
        start_checkpoint_patrol:="$START_CHECKPOINT_PATROL"
fi

echo "Launching live mapping stack (Webots GPS/IMU pose + LiDAR, no AMCL)"
exec ros2 launch robot_patrol_node mapping_stack.launch.py
