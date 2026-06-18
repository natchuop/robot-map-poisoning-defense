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
    INITIAL_POSE_X="${RMPD_AMCL_INITIAL_POSE_X:-0.0}"
    INITIAL_POSE_Y="${RMPD_AMCL_INITIAL_POSE_Y:-0.0}"
    INITIAL_POSE_YAW="${RMPD_AMCL_INITIAL_POSE_YAW:-0.0}"
    echo "Launching AMCL localization stack with map: $RMPD_AMCL_MAP_YAML"
    exec ros2 launch robot_patrol_node amcl_stack.launch.py \
        map_yaml:="$RMPD_AMCL_MAP_YAML" \
        initial_pose_x:="$INITIAL_POSE_X" \
        initial_pose_y:="$INITIAL_POSE_Y" \
        initial_pose_yaw:="$INITIAL_POSE_YAW"
fi

echo "Launching live mapping stack"
exec ros2 launch robot_patrol_node mapping_stack.launch.py
