#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export RMPD_TEST_MODE="multi_mapping"
export RMPD_WEBOTS_WORLD="$REPO_DIR/webots/worlds/TestCombineRvizMap/TestCombineRvizMap.wbt"
export RMPD_RVIZ_CONFIG_FILES="${RMPD_RVIZ_CONFIG_FILES:-multi_robot_robot_1_view.rviz,multi_robot_robot_2_view.rviz}"
export RMPD_RVIZ_WINDOW_COUNT="${RMPD_RVIZ_WINDOW_COUNT:-2}"
export RMPD_START_CHECKPOINT_PATROL="false"
export RMPD_START_NAVIGATION_DIAGNOSTICS="false"
export RMPD_START_LIVE_MAPPING="true"
export RMPD_LIVE_MAP_WIDTH_M="${RMPD_LIVE_MAP_WIDTH_M:-10.0}"
export RMPD_LIVE_MAP_HEIGHT_M="${RMPD_LIVE_MAP_HEIGHT_M:-10.0}"
export RMPD_LIVE_MAP_ORIGIN_X="${RMPD_LIVE_MAP_ORIGIN_X:--4.0}"
export RMPD_LIVE_MAP_ORIGIN_Y="${RMPD_LIVE_MAP_ORIGIN_Y:--4.0}"
export RMPD_BRIDGE_PORT="${RMPD_BRIDGE_PORT:-5005}"
export RMPD_BRIDGE_PORT_SECONDARY="${RMPD_BRIDGE_PORT_SECONDARY:-5006}"

exec bash "$REPO_DIR/scripts/quick_test.sh"
