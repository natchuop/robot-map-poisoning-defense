#!/usr/bin/env bash
# Verification script for robot-map-poisoning-defense.
#
# Run from the repo root on macOS Terminal or Windows WSL Ubuntu:
#   bash scripts/verify.sh
#
# This validates the Docker ROS 2 workspace and the Webots bridge path used by
# the project: host TCP -> Docker bridge -> /robot_pose + /scan -> /map.

set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_NAME="rmpd_verify"
CONTAINER="rmpd_verify_$RANDOM"
HOST_PORT="${RMPD_VERIFY_PORT:-15005}"
PASS=0
FAIL=0

green() { printf '\033[32m[PASS]\033[0m %s\n' "$1"; PASS=$((PASS + 1)); }
red()   { printf '\033[31m[FAIL]\033[0m %s\n' "$1"; FAIL=$((FAIL + 1)); }
info()  { printf '\033[34m[....]\033[0m %s\n' "$1"; }

cleanup() {
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
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

docker_compose() {
    docker compose -p "$PROJECT_NAME" -f "$REPO_DIR/docker-compose.yml" "$@"
}

container_exec() {
    docker exec "$CONTAINER" bash -lc "$1"
}

wait_for_log() {
    local pattern="$1"
    local timeout_seconds="${2:-60}"
    local elapsed=0

    while [ "$elapsed" -lt "$timeout_seconds" ]; do
        if docker logs "$CONTAINER" 2>&1 | grep -q "$pattern"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    docker logs "$CONTAINER" 2>&1 | tail -80
    return 1
}

topic_has_message() {
    local outfile="$1"
    [ -s "$outfile" ] && ! grep -qiE 'error|failed|timeout|Traceback' "$outfile"
}

echo ""
echo "========================================"
echo " Robot Map Poisoning Defense - Verify"
echo "========================================"
echo ""
echo "Repo: $REPO_DIR"
echo "Temporary container: $CONTAINER"
echo "Temporary host TCP port: $HOST_PORT"
echo ""

run_check "Docker daemon is running" check_docker
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
    "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/map_builder_node.py" \
    "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/udp_bridge_node.py" \
    "$REPO_DIR/webots/controllers/testRvizMap/testRvizMap.py"; then
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
    --name "$CONTAINER" \
    -p "127.0.0.1:${HOST_PORT}:5005/tcp" \
    ros2 bash -lc 'bash scripts/start_ros2_stack.sh'; then
    green "temporary mapping stack container starts"
else
    red "temporary mapping stack container starts"
    exit 1
fi

run_check "bridge reports listening on TCP/UDP" wait_for_log 'Listening for Webots packets' 90
run_check "map builder reports ready" wait_for_log 'Map builder ready' 90

info "Allowing the ROS graph to settle"
sleep 5

info "Starting topic watchers"
container_exec 'bash /workspace/scripts/watch_topic.sh /robot_pose /tmp/verify_robot_pose.out' &
POSE_WATCH_PID=$!
container_exec 'bash /workspace/scripts/watch_topic.sh /scan /tmp/verify_scan.out' &
SCAN_WATCH_PID=$!
container_exec 'bash /workspace/scripts/watch_topic.sh /map /tmp/verify_map.out' &
MAP_WATCH_PID=$!

sleep 2

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
sleep 5

wait "$POSE_WATCH_PID" >/dev/null 2>&1 || true
wait "$SCAN_WATCH_PID" >/dev/null 2>&1 || true
wait "$MAP_WATCH_PID" >/dev/null 2>&1 || true

container_exec 'cat /tmp/verify_robot_pose.out 2>/dev/null || true' > /tmp/rmpd_verify_robot_pose.out
container_exec 'cat /tmp/verify_scan.out 2>/dev/null || true' > /tmp/rmpd_verify_scan.out
container_exec 'cat /tmp/verify_map.out 2>/dev/null || true' > /tmp/rmpd_verify_map.out

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

if topic_has_message /tmp/rmpd_verify_map.out && grep -q 'OccupancyGrid\|info:' /tmp/rmpd_verify_map.out; then
    green "/map publishes an occupancy grid"
else
    red "/map publishes an occupancy grid"
    cat /tmp/rmpd_verify_map.out
fi

info "Checking RViz binary in Docker"
RVIZ_OUT="$(docker_compose run --rm ros2 bash -lc 'source /opt/ros/jazzy/setup.bash && command -v rviz2' 2>&1 || true)"
echo "$RVIZ_OUT"
if echo "$RVIZ_OUT" | grep -q 'rviz2'; then
    green "rviz2 is installed"
else
    red "rviz2 is installed"
fi

echo ""
echo "========================================"
echo " Results: $PASS passed, $FAIL failed"
echo "========================================"
echo ""

if [ "$FAIL" -eq 0 ]; then
    echo "All checks passed. Teammates can build, run the bridge, and visualize /map in RViz."
    exit 0
fi

echo "Some checks failed. Review the output above, then see docs/VERIFICATION.md."
exit 1
