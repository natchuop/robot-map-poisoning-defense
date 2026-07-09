#!/bin/bash
set -eo pipefail

CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-/workspace}"
cd "$CONTAINER_WORKSPACE"
set +u
source /opt/ros/jazzy/setup.bash
set -u
COLCON_BUILD_BASE="${RMPD_COLCON_BUILD_BASE:-/tmp/rmpd_colcon_build}"
COLCON_INSTALL_BASE="${RMPD_COLCON_INSTALL_BASE:-/tmp/rmpd_colcon_install}"
COLCON_LOG_BASE="${RMPD_COLCON_LOG_BASE:-/tmp/rmpd_colcon_log}"
rm -rf "$COLCON_BUILD_BASE" "$COLCON_INSTALL_BASE" "$COLCON_LOG_BASE"
echo "Building robot_patrol_node..."
colcon --log-base "$COLCON_LOG_BASE" build \
    --base-paths src \
    --packages-up-to robot_patrol_node \
    --symlink-install \
    --build-base "$COLCON_BUILD_BASE" \
    --install-base "$COLCON_INSTALL_BASE"
set +u
source "$COLCON_INSTALL_BASE/setup.bash"
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
    INITIAL_POSE_USE_ODOM="${RMPD_AMCL_INITIAL_POSE_USE_ODOM:-true}"
    START_CHECKPOINT_PATROL="${RMPD_START_CHECKPOINT_PATROL:-true}"
    START_CHECKPOINT_METRICS="${RMPD_START_CHECKPOINT_METRICS:-true}"
    START_NAVIGATION_DIAGNOSTICS="${RMPD_START_NAVIGATION_DIAGNOSTICS:-true}"
    START_LIVE_MAPPING="${RMPD_START_LIVE_MAPPING:-true}"
    LIVE_MAP_WIDTH_M="${RMPD_LIVE_MAP_WIDTH_M:-8.0}"
    LIVE_MAP_HEIGHT_M="${RMPD_LIVE_MAP_HEIGHT_M:-8.0}"
    LIVE_MAP_ORIGIN_X="${RMPD_LIVE_MAP_ORIGIN_X:-nan}"
    LIVE_MAP_ORIGIN_Y="${RMPD_LIVE_MAP_ORIGIN_Y:-nan}"
    MAP_ID="${RMPD_MAP_ID:-$(basename "${RMPD_AMCL_MAP_YAML%.yaml}")}"
    CHECKPOINT_METRICS_DIR="${RMPD_CHECKPOINT_METRICS_DIR:-${CONTAINER_WORKSPACE%/}/logs/checkpoint_metrics}"
    CHECKPOINT_METRICS_RUN_NAME="${RMPD_CHECKPOINT_METRICS_RUN_NAME:-${MAP_ID:-map}}"
    CHECKPOINT_METRICS_OUTPUT_CSV="${RMPD_CHECKPOINT_METRICS_OUTPUT_CSV:-${CHECKPOINT_METRICS_DIR%/}/${CHECKPOINT_METRICS_RUN_NAME}/robot_1_metrics.csv}"
    CHECKPOINT_METRICS_RADIUS_M="${RMPD_CHECKPOINT_METRICS_RADIUS_M:-0.40}"
    CHECKPOINT_METRICS_RESET="${RMPD_CHECKPOINT_METRICS_RESET:-true}"
    echo "Launching AMCL + Nav2 stack with Webots GPS/IMU odom + LiDAR + static map: $RMPD_AMCL_MAP_YAML"
    exec ros2 launch robot_patrol_node nav2_with_amcl.launch.py \
        map_yaml:="$RMPD_AMCL_MAP_YAML" \
        initial_pose_x:="$INITIAL_POSE_X" \
        initial_pose_y:="$INITIAL_POSE_Y" \
        initial_pose_yaw:="$INITIAL_POSE_YAW" \
        initial_pose_use_odom:="$INITIAL_POSE_USE_ODOM" \
        start_checkpoint_patrol:="$START_CHECKPOINT_PATROL" \
        start_checkpoint_metrics:="$START_CHECKPOINT_METRICS" \
        start_navigation_diagnostics:="$START_NAVIGATION_DIAGNOSTICS" \
        start_live_mapping:="$START_LIVE_MAPPING" \
        live_map_width_m:="$LIVE_MAP_WIDTH_M" \
        live_map_height_m:="$LIVE_MAP_HEIGHT_M" \
        live_map_origin_x:="$LIVE_MAP_ORIGIN_X" \
        live_map_origin_y:="$LIVE_MAP_ORIGIN_Y" \
        map_id:="$MAP_ID" \
        checkpoint_metrics_output_csv:="$CHECKPOINT_METRICS_OUTPUT_CSV" \
        checkpoint_metrics_radius_m:="$CHECKPOINT_METRICS_RADIUS_M" \
        checkpoint_metrics_reset:="$CHECKPOINT_METRICS_RESET"
fi

if [ "${RMPD_TEST_MODE:-mapping}" = "multi_mapping" ]; then
    LIVE_MAP_WIDTH_M="${RMPD_LIVE_MAP_WIDTH_M:-10.0}"
    LIVE_MAP_HEIGHT_M="${RMPD_LIVE_MAP_HEIGHT_M:-10.0}"
    LIVE_MAP_ORIGIN_X="${RMPD_LIVE_MAP_ORIGIN_X:--4.0}"
    LIVE_MAP_ORIGIN_Y="${RMPD_LIVE_MAP_ORIGIN_Y:--4.0}"
    echo "Launching multi-robot shared mapping stack"
    exec ros2 launch robot_patrol_node multi_robot_mapping.launch.py \
        live_map_width_m:="$LIVE_MAP_WIDTH_M" \
        live_map_height_m:="$LIVE_MAP_HEIGHT_M" \
        live_map_origin_x:="$LIVE_MAP_ORIGIN_X" \
        live_map_origin_y:="$LIVE_MAP_ORIGIN_Y"
fi

echo "Launching live mapping stack (Webots GPS/IMU pose + LiDAR, no AMCL)"
exec ros2 launch robot_patrol_node mapping_stack.launch.py
