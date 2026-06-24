#!/usr/bin/env bash
# Verification script for robot-map-poisoning-defense.
#
# Run from the repo root on macOS Terminal or Windows WSL Ubuntu:
#   bash scripts/verify.sh
#
# This validates the Docker ROS 2 workspace and the Webots bridge paths used by
# the project. It stays headless and checks both the mapping stack and the
# AMCL/Nav2 stack wiring.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_NAME="rmpd_verify"
MAPPING_CONTAINER="rmpd_verify_mapping_$RANDOM"
AMCL_CONTAINER="rmpd_verify_amcl_$RANDOM"
HOST_PORT="${RMPD_VERIFY_PORT:-15005}"
CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-/workspace}"
AMCL_MAP_HOST="$REPO_DIR/webots/worlds/testRvizMap/amcl_map/arena.yaml"
AMCL_MAP_CONTAINER="${CONTAINER_WORKSPACE%/}/webots/worlds/testRvizMap/amcl_map/arena.yaml"
export RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE"
export RMPD_INSTALL_FULL_STACK=true
PASS=0
FAIL=0

green() { printf '\033[32m[PASS]\033[0m %s\n' "$1"; PASS=$((PASS + 1)); }
red()   { printf '\033[31m[FAIL]\033[0m %s\n' "$1"; FAIL=$((FAIL + 1)); }
info()  { printf '\033[34m[....]\033[0m %s\n' "$1"; }

cleanup() {
    docker rm -f "$MAPPING_CONTAINER" "$AMCL_CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup EXIT

run_check() {
    local label="$1"
    shift
    info "$label"
    if "$@"; then
        green "$label"
    else
        red "$label"
    fi
}

check_docker() {
    docker info >/dev/null 2>&1
}

check_python3() {
    command -v python3 >/dev/null 2>&1
}

check_compose_config() {
    docker_compose config --quiet >/dev/null
    docker_compose config --services | grep -qx ros2
}

docker_compose() {
    docker compose -p "$PROJECT_NAME" -f "$REPO_DIR/docker/compose.yml" "$@"
}

container_exec() {
    local container="$1"
    shift
    docker exec "$container" bash -lc "$1"
}

check_ros2_pkg() {
    local pkg="$1"
    docker_compose run --rm ros2 bash -lc \
        "source /opt/ros/jazzy/setup.bash && ros2 pkg prefix $pkg >/dev/null"
}

check_amcl_executable() {
    docker_compose run --rm ros2 bash -lc \
        "source /opt/ros/jazzy/setup.bash && ros2 pkg executables nav2_amcl | grep -q 'nav2_amcl amcl'"
}

wait_for_log() {
    local container="$1"
    local pattern="$2"
    local timeout_seconds="${3:-60}"
    local elapsed=0

    while [ "$elapsed" -lt "$timeout_seconds" ]; do
        if docker logs "$container" 2>&1 | grep -q "$pattern"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    docker logs "$container" 2>&1 | tail -80
    return 1
}

topic_has_message() {
    local outfile="$1"
    [ -s "$outfile" ] && ! grep -qiE 'error|failed|timeout|Traceback' "$outfile"
}

check_node_info_contains() {
    local container="$1"
    local node="$2"
    shift 2

    local info
    info="$(
        container_exec "$container" \
            "source /opt/ros/jazzy/setup.bash && source \"\${RMPD_CONTAINER_WORKSPACE:-/workspace}/install/setup.bash\" && ros2 node info $node"
    )"

    for pattern in "$@"; do
        if ! grep -Fq "$pattern" <<<"$info"; then
            printf '%s\n' "$info"
            return 1
        fi
    done
}

check_topic_wiring() {
    local container="$1"
    local node="$2"
    shift 2
    check_node_info_contains "$container" "$node" "$@"
}

echo ""
echo "========================================"
echo " Robot Map Poisoning Defense - Verify"
echo "========================================"
echo ""
echo "Repo: $REPO_DIR"
echo "Temporary mapping container: $MAPPING_CONTAINER"
echo "Temporary AMCL container: $AMCL_CONTAINER"
echo "Temporary host TCP port: $HOST_PORT"
echo "Container workspace: $CONTAINER_WORKSPACE"
echo ""

run_check "Docker daemon is running" check_docker
run_check "Docker Compose config is valid" check_compose_config
run_check "Host python3 is available" check_python3

info "Building Docker image"
if docker_compose build; then
    green "Docker image builds"
else
    red "Docker image builds"
    exit 1
fi

info "Checking Python syntax"
if python3 -m py_compile \
    "$REPO_DIR/scripts/send_test_bridge_packet.py" \
    "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/checkpoint_patrol_node.py" \
    "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/initial_pose_publisher_node.py" \
    "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/map_builder_node.py" \
    "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/navigation_diagnostics_node.py" \
    "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/pose_to_odom_node.py" \
    "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/udp_bridge_node.py" \
    "$REPO_DIR/webots/robot_controllers/patrol_robot/patrol_robot.py" \
    "$REPO_DIR/webots/worlds/controllers/patrol_robot/patrol_robot.py" \
    "$REPO_DIR/webots/robot_controllers/user_controlled_robot/user_controlled_robot.py" \
    "$REPO_DIR/webots/worlds/controllers/user_controlled_robot/user_controlled_robot.py"; then
    green "Python files compile"
else
    red "Python files compile"
fi

info "Building ROS 2 workspace in Docker"
if docker_compose run --rm ros2 bash -lc \
    'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'; then
    green "colcon build succeeds"
else
    red "colcon build succeeds"
    exit 1
fi

cleanup
info "Starting temporary mapping stack"
if docker_compose run --rm -d \
    --name "$MAPPING_CONTAINER" \
    -p "127.0.0.1:${HOST_PORT}:5005/tcp" \
    ros2 bash -lc 'bash scripts/start_ros2_stack.sh'; then
    green "Temporary mapping stack container starts"
else
    red "Temporary mapping stack container starts"
    exit 1
fi

run_check "bridge reports listening on TCP/UDP" wait_for_log "$MAPPING_CONTAINER" 'Listening for Webots packets' 90
run_check "pose-to-odom bridge is ready" wait_for_log "$MAPPING_CONTAINER" 'Pose-to-odom ready' 90
run_check "map builder reports ready" wait_for_log "$MAPPING_CONTAINER" 'Map builder ready' 90

info "Allowing the ROS graph to settle"
sleep 5

info "Starting topic watchers for the mapping stack"
container_exec "$MAPPING_CONTAINER" 'bash "${RMPD_CONTAINER_WORKSPACE:-/workspace}/scripts/watch_topic.sh" /robot_pose geometry_msgs/msg/Pose2D /tmp/verify_robot_pose.out' &
POSE_WATCH_PID=$!
container_exec "$MAPPING_CONTAINER" 'bash "${RMPD_CONTAINER_WORKSPACE:-/workspace}/scripts/watch_topic.sh" /scan sensor_msgs/msg/LaserScan /tmp/verify_scan.out' &
SCAN_WATCH_PID=$!
container_exec "$MAPPING_CONTAINER" 'bash "${RMPD_CONTAINER_WORKSPACE:-/workspace}/scripts/watch_topic.sh" /odom nav_msgs/msg/Odometry /tmp/verify_odom.out' &
ODOM_WATCH_PID=$!
container_exec "$MAPPING_CONTAINER" 'bash "${RMPD_CONTAINER_WORKSPACE:-/workspace}/scripts/watch_topic.sh" /map nav_msgs/msg/OccupancyGrid /tmp/verify_map.out 60' &
MAP_WATCH_PID=$!

sleep 6

info "Sending fake Webots bridge packets from host to Docker"
if python3 "$REPO_DIR/scripts/send_test_bridge_packet.py" \
    --host 127.0.0.1 \
    --port "$HOST_PORT" \
    --count 30 \
    --delay 0.05; then
    green "host can send bridge packets to Docker"
else
    red "host can send bridge packets to Docker"
fi

info "Allowing packets to propagate through ROS"
sleep 8

wait "$POSE_WATCH_PID" >/dev/null 2>&1 || true
wait "$SCAN_WATCH_PID" >/dev/null 2>&1 || true
wait "$ODOM_WATCH_PID" >/dev/null 2>&1 || true
wait "$MAP_WATCH_PID" >/dev/null 2>&1 || true

container_exec "$MAPPING_CONTAINER" 'cat /tmp/verify_robot_pose.out 2>/dev/null || true' > /tmp/rmpd_verify_robot_pose.out
container_exec "$MAPPING_CONTAINER" 'cat /tmp/verify_scan.out 2>/dev/null || true' > /tmp/rmpd_verify_scan.out
container_exec "$MAPPING_CONTAINER" 'cat /tmp/verify_odom.out 2>/dev/null || true' > /tmp/rmpd_verify_odom.out
container_exec "$MAPPING_CONTAINER" 'cat /tmp/verify_map.out 2>/dev/null || true' > /tmp/rmpd_verify_map.out

if topic_has_message /tmp/rmpd_verify_robot_pose.out && grep -q 'theta:' /tmp/rmpd_verify_robot_pose.out; then
    green "/robot_pose receives bridge data"
else
    red "/robot_pose receives bridge data"
    cat /tmp/rmpd_verify_robot_pose.out
fi

if topic_has_message /tmp/rmpd_verify_scan.out && grep -q 'ranges:' /tmp/rmpd_verify_scan.out; then
    green "/scan receives bridge data"
else
    red "/scan receives bridge data"
    cat /tmp/rmpd_verify_scan.out
fi

if topic_has_message /tmp/rmpd_verify_odom.out && grep -q 'pose:' /tmp/rmpd_verify_odom.out; then
    green "/odom receives bridge data"
else
    red "/odom receives bridge data"
    cat /tmp/rmpd_verify_odom.out
fi

if topic_has_message /tmp/rmpd_verify_map.out && grep -q 'OccupancyGrid\|info:' /tmp/rmpd_verify_map.out; then
    green "/map publishes an occupancy grid"
else
    red "/map publishes an occupancy grid"
    cat /tmp/rmpd_verify_map.out
fi

run_check "mapping bridge wiring is present" check_topic_wiring "$MAPPING_CONTAINER" /udp_bridge \
    '/robot_pose' \
    '/scan' \
    '/odom' \
    '/cmd_vel' \
    '/active_checkpoint' \
    '/webots_checkpoint_event' \
    '/webots_checkpoint_contact'

run_check "mapping map_builder wiring is present" check_topic_wiring "$MAPPING_CONTAINER" /map_builder \
    '/robot_pose' \
    '/scan' \
    '/map'

run_check "RViz binary is installed in Docker" docker_compose run --rm ros2 bash -lc \
    'source /opt/ros/jazzy/setup.bash && command -v rviz2 >/dev/null'

cleanup
info "Starting temporary AMCL/Nav2 stack"
if [ ! -f "$AMCL_MAP_HOST" ]; then
    echo "AMCL map file not found: $AMCL_MAP_HOST" >&2
    exit 1
fi

if docker_compose run --rm -d \
    --name "$AMCL_CONTAINER" \
    -p "127.0.0.1:${HOST_PORT}:5005/tcp" \
    -e RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE" \
    -e RMPD_TEST_MODE=amcl \
    -e RMPD_AMCL_MAP_YAML="$AMCL_MAP_CONTAINER" \
    -e RMPD_AMCL_INITIAL_POSE_USE_ODOM=true \
    -e RMPD_START_CHECKPOINT_PATROL=true \
    -e RMPD_START_NAVIGATION_DIAGNOSTICS=true \
    -e RMPD_START_LIVE_MAPPING=true \
    ros2 bash -lc 'bash scripts/start_ros2_stack.sh'; then
    green "Temporary AMCL stack container starts"
else
    red "Temporary AMCL stack container starts"
    exit 1
fi

run_check "AMCL stack bridge is listening" wait_for_log "$AMCL_CONTAINER" 'Listening for Webots packets' 120
run_check "AMCL pose-to-odom bridge is ready" wait_for_log "$AMCL_CONTAINER" 'Pose-to-odom ready' 120
run_check "initial pose publisher is ready" wait_for_log "$AMCL_CONTAINER" 'Initial pose publisher ready' 120
run_check "AMCL lifecycle manager reaches active state" wait_for_log "$AMCL_CONTAINER" 'Managed nodes are active' 180

info "Starting topic watchers for the AMCL stack"
container_exec "$AMCL_CONTAINER" 'bash "${RMPD_CONTAINER_WORKSPACE:-/workspace}/scripts/watch_topic.sh" /robot_pose geometry_msgs/msg/Pose2D /tmp/verify_amcl_robot_pose.out' &
AMCL_POSE_WATCH_PID=$!
container_exec "$AMCL_CONTAINER" 'bash "${RMPD_CONTAINER_WORKSPACE:-/workspace}/scripts/watch_topic.sh" /scan sensor_msgs/msg/LaserScan /tmp/verify_amcl_scan.out' &
AMCL_SCAN_WATCH_PID=$!
container_exec "$AMCL_CONTAINER" 'bash "${RMPD_CONTAINER_WORKSPACE:-/workspace}/scripts/watch_topic.sh" /odom nav_msgs/msg/Odometry /tmp/verify_amcl_odom.out' &
AMCL_ODOM_WATCH_PID=$!
container_exec "$AMCL_CONTAINER" 'bash "${RMPD_CONTAINER_WORKSPACE:-/workspace}/scripts/watch_topic.sh" /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped /tmp/verify_amcl_pose.out 60' &
AMCL_POSE_TOPIC_WATCH_PID=$!
container_exec "$AMCL_CONTAINER" 'bash "${RMPD_CONTAINER_WORKSPACE:-/workspace}/scripts/watch_topic.sh" /live_map nav_msgs/msg/OccupancyGrid /tmp/verify_live_map.out 60' &
LIVE_MAP_WATCH_PID=$!

sleep 6

info "Sending fake Webots bridge packets into the AMCL stack"
if python3 "$REPO_DIR/scripts/send_test_bridge_packet.py" \
    --host 127.0.0.1 \
    --port "$HOST_PORT" \
    --count 40 \
    --delay 0.05; then
    green "host can send bridge packets to the AMCL stack"
else
    red "host can send bridge packets to the AMCL stack"
fi

info "Allowing AMCL data to propagate"
sleep 10

wait "$AMCL_POSE_WATCH_PID" >/dev/null 2>&1 || true
wait "$AMCL_SCAN_WATCH_PID" >/dev/null 2>&1 || true
wait "$AMCL_ODOM_WATCH_PID" >/dev/null 2>&1 || true
wait "$AMCL_POSE_TOPIC_WATCH_PID" >/dev/null 2>&1 || true
wait "$LIVE_MAP_WATCH_PID" >/dev/null 2>&1 || true

container_exec "$AMCL_CONTAINER" 'cat /tmp/verify_amcl_robot_pose.out 2>/dev/null || true' > /tmp/rmpd_verify_amcl_robot_pose.out
container_exec "$AMCL_CONTAINER" 'cat /tmp/verify_amcl_scan.out 2>/dev/null || true' > /tmp/rmpd_verify_amcl_scan.out
container_exec "$AMCL_CONTAINER" 'cat /tmp/verify_amcl_odom.out 2>/dev/null || true' > /tmp/rmpd_verify_amcl_odom.out
container_exec "$AMCL_CONTAINER" 'cat /tmp/verify_amcl_pose.out 2>/dev/null || true' > /tmp/rmpd_verify_amcl_pose.out
container_exec "$AMCL_CONTAINER" 'cat /tmp/verify_live_map.out 2>/dev/null || true' > /tmp/rmpd_verify_live_map.out

if topic_has_message /tmp/rmpd_verify_amcl_robot_pose.out && grep -q 'theta:' /tmp/rmpd_verify_amcl_robot_pose.out; then
    green "AMCL stack receives /robot_pose bridge data"
else
    red "AMCL stack receives /robot_pose bridge data"
    cat /tmp/rmpd_verify_amcl_robot_pose.out
fi

if topic_has_message /tmp/rmpd_verify_amcl_scan.out && grep -q 'ranges:' /tmp/rmpd_verify_amcl_scan.out; then
    green "AMCL stack receives /scan bridge data"
else
    red "AMCL stack receives /scan bridge data"
    cat /tmp/rmpd_verify_amcl_scan.out
fi

if topic_has_message /tmp/rmpd_verify_amcl_odom.out && grep -q 'pose:' /tmp/rmpd_verify_amcl_odom.out; then
    green "AMCL stack receives /odom bridge data"
else
    red "AMCL stack receives /odom bridge data"
    cat /tmp/rmpd_verify_amcl_odom.out
fi

if topic_has_message /tmp/rmpd_verify_amcl_pose.out && grep -q 'position:' /tmp/rmpd_verify_amcl_pose.out; then
    green "/amcl_pose is published"
else
    red "/amcl_pose is published"
    cat /tmp/rmpd_verify_amcl_pose.out
fi

if topic_has_message /tmp/rmpd_verify_live_map.out && grep -q 'OccupancyGrid\|info:' /tmp/rmpd_verify_live_map.out; then
    green "/live_map publishes an occupancy grid"
else
    red "/live_map publishes an occupancy grid"
    cat /tmp/rmpd_verify_live_map.out
fi

run_check "AMCL bridge wiring is present" check_topic_wiring "$AMCL_CONTAINER" /udp_bridge \
    '/robot_pose' \
    '/scan' \
    '/odom' \
    '/cmd_vel' \
    '/active_checkpoint' \
    '/webots_checkpoint_event' \
    '/webots_checkpoint_contact'

run_check "checkpoint patrol wiring is present" check_topic_wiring "$AMCL_CONTAINER" /checkpoint_patrol_node \
    '/robot_pose' \
    '/amcl_pose' \
    '/scan' \
    '/cmd_vel' \
    '/active_checkpoint' \
    '/webots_checkpoint_contact' \
    '/webots_checkpoint_event'

run_check "initial pose publisher wiring is present" check_topic_wiring "$AMCL_CONTAINER" /initial_pose_publisher \
    '/odom' \
    '/initialpose'

run_check "AMCL node wiring is present" check_topic_wiring "$AMCL_CONTAINER" /amcl \
    '/scan' \
    '/amcl_pose'

run_check "Nav2 controller is installed in Docker" check_ros2_pkg nav2_controller
run_check "Nav2 planner is installed in Docker" check_ros2_pkg nav2_planner
run_check "Nav2 map server is installed in Docker" check_ros2_pkg nav2_map_server
run_check "Nav2 waypoint follower is installed in Docker" check_ros2_pkg nav2_waypoint_follower
run_check "AMCL is installed in Docker" check_ros2_pkg nav2_amcl
run_check "AMCL executable is available in Docker" check_amcl_executable

echo ""
echo "========================================"
echo " Results: $PASS passed, $FAIL failed"
echo "========================================"
echo ""

if [ "$FAIL" -eq 0 ]; then
    echo "All checks passed. Teammates can build the Docker image, verify the bridge, and run the AMCL/Nav2 stack."
    exit 0
fi

echo "Some checks failed. Review the output above, then see docs/VERIFICATION.md."
exit 1
