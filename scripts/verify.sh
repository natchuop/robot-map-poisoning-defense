#!/usr/bin/env bash
# Basic Docker verification for robot-map-poisoning-defense.
#
# Run from the repo root:
#   bash scripts/verify.sh
#
# This stays intentionally small. It checks that Docker/Compose are available,
# the image builds, and the core ROS 2 + Nav2 + Webots downloads are present
# and connected in headless mode.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_NAME="rmpd_verify"
COMPOSE_FILE="$REPO_DIR/docker/compose.yml"
CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-/workspace}"

PASS=0
FAIL=0

green() { printf '\033[32m[PASS]\033[0m %s\n' "$1"; PASS=$((PASS + 1)); }
red() { printf '\033[31m[FAIL]\033[0m %s\n' "$1"; FAIL=$((FAIL + 1)); }
info() { printf '\033[34m[....]\033[0m %s\n' "$1"; }

docker_compose() {
    docker compose -p "$PROJECT_NAME" -f "$COMPOSE_FILE" "$@"
}

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
    command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1
}

check_python3() {
    command -v python3 >/dev/null 2>&1
}

check_compose_config() {
    docker_compose config --quiet >/dev/null
}

check_service_exists() {
    docker_compose config --services | grep -qx ros2
}

check_image_builds() {
    docker_compose build ros2
}

container_ros_exec() {
    local container="$1"
    shift
    local command="$*"

    docker exec "$container" bash -lc '
        set -euo pipefail
        set +u
        source /opt/ros/jazzy/setup.bash
        if [ -f "${RMPD_COLCON_INSTALL_BASE:-/tmp/rmpd_colcon_install}/setup.bash" ]; then
            source "${RMPD_COLCON_INSTALL_BASE:-/tmp/rmpd_colcon_install}/setup.bash"
        fi
        set -u
        eval "$1"
    ' _ "$command"
}

check_package_downloads() {
    docker_compose run --rm ros2 bash -lc '
        set -euo pipefail
        set +u
        source /opt/ros/jazzy/setup.bash
        set -u

        test -d "${RMPD_CONTAINER_WORKSPACE:-/workspace}"
        test -f "${RMPD_CONTAINER_WORKSPACE:-/workspace}/README.md"

        command -v ros2 >/dev/null
        command -v colcon >/dev/null
        command -v rviz2 >/dev/null

        ros2 pkg prefix nav2_amcl >/dev/null
        ros2 pkg prefix nav2_controller >/dev/null
        ros2 pkg prefix nav2_lifecycle_manager >/dev/null
        ros2 pkg prefix nav2_map_server >/dev/null
        ros2 pkg prefix nav2_planner >/dev/null
        ros2 pkg prefix nav2_rviz_plugins >/dev/null
        ros2 pkg prefix nav2_waypoint_follower >/dev/null
        ros2 pkg prefix robot_localization >/dev/null
        ros2 pkg prefix tf2_ros >/dev/null
        ros2 pkg prefix webots_ros2 >/dev/null
        ros2 pkg prefix webots_ros2_driver >/dev/null
        ros2 pkg prefix webots_ros2_turtlebot >/dev/null
        ros2 pkg prefix foxglove_bridge >/dev/null

        ros2 topic list >/dev/null
        ros2 node list >/dev/null
    '
}

check_ros_executables() {
    docker_compose run --rm ros2 bash -lc '
        set -euo pipefail
        set +u
        source /opt/ros/jazzy/setup.bash
        set -u

        command -v rviz2 >/dev/null
        ros2 pkg executables nav2_amcl | grep -q "nav2_amcl amcl"
    '
}

check_python_syntax() {
    python3 -m py_compile \
        "$REPO_DIR/scripts/send_test_bridge_packet.py" \
        "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/checkpoint_patrol_node.py" \
        "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/confidence_marker_node.py" \
        "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/fake_obstacle_injector_node.py" \
        "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/initial_pose_publisher_node.py" \
        "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/localized_pose2d_node.py" \
        "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/map_builder_node.py" \
        "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/map_merge_node.py" \
        "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/navigation_diagnostics_node.py" \
        "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/pose_to_odom_node.py" \
        "$REPO_DIR/src/robot_patrol_node/robot_patrol_node/udp_bridge_node.py"
}

check_portability_warnings() {
    local patterns=(
        '/home/nathan'
        '/Users/nathan'
        'C:\\Users\\Nathan'
    )
    local search_roots=("$REPO_DIR/docs" "$REPO_DIR/scripts" "$REPO_DIR/src" "$REPO_DIR/webots")
    local found=0

    info "Scanning for obvious hardcoded personal paths"
    for pattern in "${patterns[@]}"; do
        if grep -RInF --exclude-dir=build --exclude-dir=install --exclude-dir=log --exclude-dir=.git "$pattern" "${search_roots[@]}" 2>/dev/null; then
            found=1
        fi
    done

    if [ "$found" -eq 0 ]; then
        green "No obvious hardcoded personal paths found"
    else
        printf '\033[33m[WARN]\033[0m Hardcoded personal path patterns were found\n'
        info "Review the warnings above and prefer repo-relative paths or config/env overrides."
    fi
}

wait_for_container_log_pattern() {
    local container_name="$1"
    local pattern="$2"
    local timeout_seconds="${3:-180}"
    local label="${4:-$pattern}"
    local elapsed=0
    local logs=""

    info "Waiting for $label"
    while [ "$elapsed" -lt "$timeout_seconds" ]; do
        if ! docker inspect "$container_name" >/dev/null 2>&1; then
            echo "Container $container_name is not available while waiting for '$label'." >&2
            return 1
        fi

        logs="$(docker logs "$container_name" 2>&1 || true)"
        if grep -Fq "$pattern" <<<"$logs"; then
            return 0
        fi

        sleep 2
        elapsed=$((elapsed + 2))
    done

    echo "Timed out waiting for '$label'." >&2
    printf '%s\n' "$logs" | tail -120 >&2 || true
    return 1
}

wait_for_node_list_contains() {
    local container="$1"
    local timeout_seconds="${2:-90}"
    shift 2

    local elapsed=0
    local nodes=""

    info "Waiting for node graph"
    while [ "$elapsed" -lt "$timeout_seconds" ]; do
        if ! docker inspect "$container" >/dev/null 2>&1; then
            echo "Container $container is not available while waiting for nodes." >&2
            return 1
        fi

        if nodes="$(container_ros_exec "$container" "ros2 node list" 2>/dev/null)"; then
            local node
            local missing=0
            for node in "$@"; do
                if ! grep -Fxq "$node" <<<"$nodes"; then
                    missing=1
                    break
                fi
            done

            if [ "$missing" -eq 0 ]; then
                return 0
            fi
        fi

        sleep 2
        elapsed=$((elapsed + 2))
    done

    printf '%s\n' "$nodes"
    return 1
}

wait_for_topic_info_contains() {
    local container="$1"
    local topic="$2"
    local timeout_seconds="${3:-90}"
    shift 3

    local elapsed=0
    local info=""

    info "Waiting for topic wiring on $topic"
    while [ "$elapsed" -lt "$timeout_seconds" ]; do
        if ! docker inspect "$container" >/dev/null 2>&1; then
            echo "Container $container is not available while waiting for topic '$topic'." >&2
            return 1
        fi

        if info="$(container_ros_exec "$container" "ros2 topic info -v $topic" 2>/dev/null)"; then
            local pattern
            local missing=0
            for pattern in "$@"; do
                if ! grep -Fq "$pattern" <<<"$info"; then
                    missing=1
                    break
                fi
            done

            if [ "$missing" -eq 0 ]; then
                return 0
            fi
        fi

        sleep 2
        elapsed=$((elapsed + 2))
    done

    printf '%s\n' "$info"
    return 1
}

wait_for_container_log_patterns() {
    local container="$1"
    shift

    local pattern
    for pattern in "$@"; do
        if ! wait_for_container_log_pattern "$container" "$pattern" 180 "$pattern"; then
            return 1
        fi
    done
}

check_amcl_topic_wiring() {
    local container_name="$1"

    wait_for_topic_info_contains "$container_name" /robot_pose 90 'udp_bridge' 'checkpoint_patrol_node' 'navigation_diagnostics' || return 1
    wait_for_topic_info_contains "$container_name" /scan 90 'udp_bridge' 'amcl' || return 1
    wait_for_topic_info_contains "$container_name" /odom 90 'udp_bridge' 'controller_server' 'bt_navigator' || return 1
    wait_for_topic_info_contains "$container_name" /map 90 'map_server' 'amcl' || return 1
    wait_for_topic_info_contains "$container_name" /amcl_pose 90 'amcl' 'localized_pose2d' || return 1
    wait_for_topic_info_contains "$container_name" /initialpose 90 'initial_pose_publisher' 'amcl' || return 1
}

check_multi_topic_wiring() {
    local container_name="$1"

    wait_for_topic_info_contains "$container_name" /map_updates 120 'robot_1_fake_obstacle_injector' 'robot_2_fake_obstacle_injector' 'robot_1_view_belief' 'robot_2_view_belief' || return 1
    wait_for_topic_info_contains "$container_name" /robot_1/live_map 90 'robot_1_map_builder' 'robot_1_view_belief' || return 1
    wait_for_topic_info_contains "$container_name" /robot_2/live_map 90 'robot_2_map_builder' 'robot_2_view_belief' || return 1
}

check_headless_amcl_stack() {
    local container_name="rmpd_verify_amcl_$RANDOM"
    local bridge_port="$((15005 + (RANDOM % 1000)))"
    local map_yaml_in_container="$CONTAINER_WORKSPACE/webots/worlds/testRvizMap/amcl_map/arena.yaml"

    docker rm -f "$container_name" >/dev/null 2>&1 || true
    info "Starting headless AMCL stack"
    container_name="$(
        docker compose -p "$PROJECT_NAME" -f "$COMPOSE_FILE" run -d \
            --name "$container_name" \
            -p "127.0.0.1:${bridge_port}:5005/tcp" \
            -p "127.0.0.1:${bridge_port}:5005/udp" \
            -e RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE" \
            -e RMPD_COLCON_INSTALL_BASE=/tmp/rmpd_colcon_install \
            -e RMPD_TEST_MODE=amcl \
            -e RMPD_AMCL_MAP_YAML="$map_yaml_in_container" \
            -e RMPD_AMCL_INITIAL_POSE_USE_ODOM=true \
            -e RMPD_START_CHECKPOINT_PATROL=true \
            -e RMPD_START_NAVIGATION_DIAGNOSTICS=true \
            -e RMPD_START_LIVE_MAPPING=true \
            -e RMPD_BRIDGE_PORT="$bridge_port" \
            ros2 bash scripts/start_ros2_stack.sh
    )"

    if ! wait_for_container_log_patterns "$container_name" \
        'ROS 2 workspace ready' \
        'Listening for Webots packets' \
        'Pose-to-odom ready'; then
        docker rm -f "$container_name" >/dev/null 2>&1 || true
        return 1
    fi

    run_check "AMCL container is running" docker inspect "$container_name" >/dev/null 2>&1
    run_check "robot_patrol_node is built in the AMCL container" container_ros_exec "$container_name" "ros2 pkg prefix robot_patrol_node >/dev/null"

    info "Sending bridge packets into the AMCL stack"
    run_check "AMCL bridge packets are accepted" python3 "$REPO_DIR/scripts/send_test_bridge_packet.py" --host 127.0.0.1 --port "$bridge_port" --count 20 --delay 0.05
    if ! wait_for_container_log_pattern "$container_name" 'Published Webots packet #' 60 'AMCL bridge packet processing'; then
        docker rm -f "$container_name" >/dev/null 2>&1 || true
        return 1
    fi

    run_check "AMCL node graph is present" wait_for_node_list_contains "$container_name" 120 \
        /udp_bridge \
        /map_server \
        /amcl \
        /lifecycle_manager_localization \
        /initial_pose_publisher \
        /live_map_builder \
        /checkpoint_patrol_node \
        /navigation_diagnostics
    run_check "AMCL topic wiring is connected" check_amcl_topic_wiring "$container_name"

    docker rm -f "$container_name" >/dev/null 2>&1 || true
}

check_headless_multi_stack() {
    local container_name="rmpd_verify_multi_$RANDOM"
    local bridge_port="$((16005 + (RANDOM % 1000)))"
    local bridge_port_secondary="$((bridge_port + 1))"

    docker rm -f "$container_name" >/dev/null 2>&1 || true
    info "Starting headless multi-robot stack"
    container_name="$(
        docker compose -p "$PROJECT_NAME" -f "$COMPOSE_FILE" run -d \
            --name "$container_name" \
            -p "127.0.0.1:${bridge_port}:5005/tcp" \
            -p "127.0.0.1:${bridge_port}:5005/udp" \
            -p "127.0.0.1:${bridge_port_secondary}:5006/tcp" \
            -p "127.0.0.1:${bridge_port_secondary}:5006/udp" \
            -e RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE" \
            -e RMPD_COLCON_INSTALL_BASE=/tmp/rmpd_colcon_install \
            -e RMPD_TEST_MODE=multi_mapping \
            -e RMPD_FAKE_OBSTACLE_INJECTOR_MODE=manual \
            -e RMPD_ROBOT_1_COMPROMISED=true \
            -e RMPD_ROBOT_2_COMPROMISED=true \
            -e RMPD_BRIDGE_PORT="$bridge_port" \
            -e RMPD_BRIDGE_PORT_SECONDARY="$bridge_port_secondary" \
            ros2 bash scripts/start_ros2_stack.sh
    )"

    if ! wait_for_container_log_patterns "$container_name" \
        'ROS 2 workspace ready' \
        'Fake obstacle injector ready' \
        'published fake obstacle marker'; then
        docker rm -f "$container_name" >/dev/null 2>&1 || true
        return 1
    fi

    run_check "multi-robot container is running" docker inspect "$container_name" >/dev/null 2>&1
    run_check "robot_patrol_node is built in the multi-robot container" container_ros_exec "$container_name" "ros2 pkg prefix robot_patrol_node >/dev/null"
    run_check "multi-robot node graph is present" container_ros_exec "$container_name" "ros2 node list | grep -Eq '(^|[[:space:]])(/robot_1_bridge|/robot_2_bridge|/robot_1_map_builder|/robot_2_map_builder|/robot_1_view_belief|/robot_2_view_belief|/robot_1_fake_obstacle_injector|/robot_2_fake_obstacle_injector)([[:space:]]|$)'"
    run_check "multi-robot topic wiring is connected" check_multi_topic_wiring "$container_name"

    docker rm -f "$container_name" >/dev/null 2>&1 || true
}

echo
echo "========================================"
echo " Robot Map Poisoning Defense - Verify"
echo "========================================"
echo
echo "Repo: $REPO_DIR"
echo "Compose file: $COMPOSE_FILE"
echo "Container workspace: $CONTAINER_WORKSPACE"
echo

run_check "Docker daemon is running" check_docker
run_check "Host python3 is available" check_python3
run_check "Docker Compose config is valid" check_compose_config
run_check "Compose service exists" check_service_exists
run_check "Docker image builds" check_image_builds
run_check "Python sources compile" check_python_syntax
check_portability_warnings
run_check "Core ROS/Nav2/Webots packages are installed" check_package_downloads
run_check "Core ROS executables are available" check_ros_executables
check_headless_amcl_stack
check_headless_multi_stack

echo
echo "========================================"
echo " Results: $PASS passed, $FAIL failed"
echo "========================================"
echo

if [ "$FAIL" -eq 0 ]; then
    echo "All checks passed. Docker, Compose, Python syntax, the image build, core ROS package downloads, and headless stack connections look healthy."
    exit 0
fi

echo "Some checks failed. Review the output above, then run the smoke tests separately:"
echo "  bash scripts/quick_test.sh"
echo "  bash scripts/runTestFakeObstacle.sh"
exit 1
