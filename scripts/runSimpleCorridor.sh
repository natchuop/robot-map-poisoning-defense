#!/usr/bin/env bash
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_DIR="$REPO_DIR/webots/worlds/simpleCorridor"
SOURCE_WORLD="$WORLD_DIR/simple_corridor.wbt"
SANDBOX_WORLD="$WORLD_DIR/sandbox.wbt"

cleanup() {
  rm -f "$SANDBOX_WORLD"
}

trap cleanup EXIT

cp "$SOURCE_WORLD" "$SANDBOX_WORLD"
export RMPD_WEBOTS_WORLD="$SANDBOX_WORLD"
export RMPD_TEST_MODE="multi_mapping"
export RMPD_MAP_ID="simple_corridor"
export RMPD_MULTI_ROBOT_CONFIG="$REPO_DIR/webots/worlds/simpleCorridor/multi_robot_config.json"
export RMPD_RVIZ_CONFIG_FILES="${RMPD_RVIZ_CONFIG_FILES:-multi_robot_robot_1_view.rviz,multi_robot_robot_2_view.rviz}"
export RMPD_RVIZ_WINDOW_COUNT="${RMPD_RVIZ_WINDOW_COUNT:-2}"
export RMPD_QUICK_TEST_RVIZ="${RMPD_QUICK_TEST_RVIZ:-true}"
export RMPD_QUICK_TEST_FORCE_CLEAN="${RMPD_QUICK_TEST_FORCE_CLEAN:-true}"
export RMPD_ROBOT_1_COMPROMISED="${RMPD_ROBOT_1_COMPROMISED:-true}"
export RMPD_ROBOT_2_COMPROMISED="${RMPD_ROBOT_2_COMPROMISED:-true}"
export RMPD_ROBOT_1_TRUST_WEIGHT="${RMPD_ROBOT_1_TRUST_WEIGHT:-1.0}"
export RMPD_ROBOT_2_TRUST_WEIGHT="${RMPD_ROBOT_2_TRUST_WEIGHT:-0.35}"
export RMPD_FAKE_REPORT_RADIUS_CELLS="${RMPD_FAKE_REPORT_RADIUS_CELLS:-2}"
export RMPD_FAKE_OBSTACLE_INJECTOR_MODE="${RMPD_FAKE_OBSTACLE_INJECTOR_MODE:-clicked_point}"
export RMPD_LIVE_MAP_WIDTH_M="${RMPD_LIVE_MAP_WIDTH_M:-14.0}"
export RMPD_LIVE_MAP_HEIGHT_M="${RMPD_LIVE_MAP_HEIGHT_M:-4.0}"
export RMPD_LIVE_MAP_ORIGIN_X="${RMPD_LIVE_MAP_ORIGIN_X:--6.5}"
export RMPD_LIVE_MAP_ORIGIN_Y="${RMPD_LIVE_MAP_ORIGIN_Y:--2.0}"
bash "$REPO_DIR/scripts/quick_test.sh"
