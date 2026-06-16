#!/bin/bash
# Verification script for robot-map-poisoning-defense
# Run this from the repo root inside WSL (not PowerShell):
#   bash scripts/verify.sh
#
# Checks: Docker image, colcon build, ROS 2 pub/sub, project packages,
# Webots packages, Nav2/TurtleBot3 stack, Python deps, and rviz2.

set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="robot-map-poisoning-defense-ros2"
PASS=0
FAIL=0

green() { echo -e "\e[32m[PASS]\e[0m $1"; PASS=$((PASS + 1)); }
red()   { echo -e "\e[31m[FAIL]\e[0m $1"; FAIL=$((FAIL + 1)); }
info()  { echo -e "\e[34m[....]\e[0m $1"; }

echo ""
echo "========================================"
echo " Robot Map Poisoning Defense - Verify"
echo "========================================"
echo ""

info "Checking Docker..."
if docker info > /dev/null 2>&1; then
    green "Docker is running"
else
    red "Docker is not running - start Docker Desktop and retry"
    exit 1
fi

info "Checking image '$IMAGE'..."
if docker image inspect "$IMAGE" > /dev/null 2>&1; then
    SIZE=$(docker image inspect "$IMAGE" --format='{{.Size}}' | awk '{printf "%.1f GB", $1/1073741824}')
    green "Image exists ($SIZE)"
else
    info "Image not found - building now (this takes 5-15 min first time)..."
    docker compose -f "$REPO_DIR/docker-compose.yml" build
    green "Image built"
fi

info "Running colcon build..."
BUILD_OUT=$(docker run --rm \
    -v "$REPO_DIR:/workspace" \
    -w /workspace \
    "$IMAGE" \
    bash -lc "source /opt/ros/jazzy/setup.bash && colcon build 2>&1 | tail -3" 2>&1) || true

echo "$BUILD_OUT"
if echo "$BUILD_OUT" | grep -q "Summary: 5 packages finished"; then
    green "colcon build - 5 packages finished"
else
    red "colcon build failed or wrong package count"
fi

info "Testing ROS 2 pub/sub (talker + listener)..."
PUBSUB_OUT=$(docker run --rm "$IMAGE" bash -lc \
    "source /opt/ros/jazzy/setup.bash && \
     ros2 run demo_nodes_cpp talker & sleep 3 && \
     ros2 run demo_nodes_py listener & sleep 4 && \
     kill %1 %2 2>/dev/null; true" 2>&1) || true

echo "$PUBSUB_OUT" | grep -E "Hello World|Publishing" | head -5 || true
if echo "$PUBSUB_OUT" | grep -q "Hello World"; then
    green "ROS 2 pub/sub - Hello World messages confirmed"
else
    red "ROS 2 pub/sub - no Hello World output detected"
fi

info "Checking project packages..."
PKG_OUT=$(docker run --rm \
    -v "$REPO_DIR:/workspace" \
    -w /workspace \
    "$IMAGE" \
    bash -lc "source /opt/ros/jazzy/setup.bash && source install/setup.bash 2>/dev/null && \
              ros2 pkg list | grep -E 'attack_node|defense_node|llm_security_agent|map_sharing_msgs|robot_patrol_node'" 2>&1) || true

FOUND=$(echo "$PKG_OUT" | grep -c '.' || true)
echo "$PKG_OUT"
if [ "$FOUND" -eq 5 ]; then
    green "Project packages - all 5 found"
else
    red "Project packages - expected 5, found $FOUND"
fi

info "Checking webots_ros2 packages..."
WEBOTS_OUT=$(docker run --rm "$IMAGE" bash -lc \
    "source /opt/ros/jazzy/setup.bash && ros2 pkg list | grep webots_ros2" 2>&1) || true

WEBOTS_COUNT=$(echo "$WEBOTS_OUT" | grep -c "webots" || true)
echo "$WEBOTS_OUT"
if [ "$WEBOTS_COUNT" -ge 5 ]; then
    green "webots_ros2 - $WEBOTS_COUNT packages found"
else
    red "webots_ros2 - expected at least 5 packages, found $WEBOTS_COUNT"
fi

info "Checking Nav2 and localization packages..."
NAV_OUT=$(docker run --rm "$IMAGE" bash -lc \
    "source /opt/ros/jazzy/setup.bash && \
     ros2 pkg list | grep -E '^(navigation2|nav2_bringup|nav2_simple_commander|robot_localization|tf2_ros|tf2_tools)$'" 2>&1) || true

NAV_COUNT=$(echo "$NAV_OUT" | grep -c '.' || true)
echo "$NAV_OUT"
if [ "$NAV_COUNT" -eq 5 ]; then
    green "Nav2/localization stack - all 5 packages found"
elif [ "$NAV_COUNT" -eq 6 ]; then
    green "Nav2/localization stack - all 6 packages found"
else
    red "Nav2/localization stack - expected 5 or 6, found $NAV_COUNT"
fi

info "Checking TurtleBot3 packages..."
TB3_OUT=$(docker run --rm "$IMAGE" bash -lc \
    "source /opt/ros/jazzy/setup.bash && \
     ros2 pkg list | grep -E '^(turtlebot3|turtlebot3_msgs|turtlebot3_navigation2)$'" 2>&1) || true

TB3_COUNT=$(echo "$TB3_OUT" | grep -c '.' || true)
echo "$TB3_OUT"
if [ "$TB3_COUNT" -eq 3 ]; then
    green "TurtleBot3 stack - all 3 packages found"
else
    red "TurtleBot3 stack - expected 3, found $TB3_COUNT"
fi

info "Checking Python dependencies..."
PY_OUT=$(docker run --rm "$IMAGE" bash -lc \
    "python3 - <<'PY'
import numpy
import scipy
print('numpy', numpy.__version__)
print('scipy', scipy.__version__)
PY" 2>&1) || true

echo "$PY_OUT"
if echo "$PY_OUT" | grep -q "^numpy " && echo "$PY_OUT" | grep -q "^scipy "; then
    green "Python dependencies - numpy and scipy import successfully"
else
    red "Python dependencies - numpy/scipy import failed"
fi

info "Checking rviz2..."
RVIZ_OUT=$(docker run --rm "$IMAGE" bash -lc \
    "source /opt/ros/jazzy/setup.bash && which rviz2" 2>&1) || true

echo "$RVIZ_OUT"
if echo "$RVIZ_OUT" | grep -q "rviz2"; then
    green "rviz2 - found at $RVIZ_OUT"
else
    red "rviz2 - not found"
fi

echo ""
echo "========================================"
echo " Results: $PASS passed, $FAIL failed"
echo "========================================"
echo ""

if [ "$FAIL" -eq 0 ]; then
    echo "All checks passed. Environment is ready."
else
    echo "Some checks failed. See output above and check docs/VERIFICATION.md."
fi
