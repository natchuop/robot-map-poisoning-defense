#!/bin/bash
set -eo pipefail

CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-/workspace}"
cd "$CONTAINER_WORKSPACE"
set +u
source /opt/ros/jazzy/setup.bash
set -u
rm -rf build install log/latest
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
    INITIAL_POSE_X="${RMPD_AMCL_INITIAL_POSE_X:-0.0}"
    INITIAL_POSE_Y="${RMPD_AMCL_INITIAL_POSE_Y:-0.0}"
    INITIAL_POSE_YAW="${RMPD_AMCL_INITIAL_POSE_YAW:-0.0}"
    START_CHECKPOINT_PATROL="${RMPD_START_CHECKPOINT_PATROL:-true}"
    START_NAVIGATION_DIAGNOSTICS="${RMPD_START_NAVIGATION_DIAGNOSTICS:-true}"
    START_LIVE_MAPPING="${RMPD_START_LIVE_MAPPING:-true}"
    echo "Launching AMCL + Nav2 stack with Webots GPS/IMU odom + LiDAR + static map: $RMPD_AMCL_MAP_YAML"
    exec ros2 launch robot_patrol_node nav2_with_amcl.launch.py \
        map_yaml:="$RMPD_AMCL_MAP_YAML" \
        initial_pose_x:="$INITIAL_POSE_X" \
        initial_pose_y:="$INITIAL_POSE_Y" \
        initial_pose_yaw:="$INITIAL_POSE_YAW" \
        start_checkpoint_patrol:="$START_CHECKPOINT_PATROL" \
        start_navigation_diagnostics:="$START_NAVIGATION_DIAGNOSTICS" \
        start_live_mapping:="$START_LIVE_MAPPING"
fi

echo "Launching live mapping stack (Webots GPS/IMU pose + LiDAR, no AMCL)"
exec ros2 launch robot_patrol_node mapping_stack.launch.py
