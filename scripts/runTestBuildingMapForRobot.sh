#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_PATH="$REPO_DIR/webots/worlds/testBuildingMapForRobot/turtlebot3_burger.wbt"

if [ ! -f "$WORLD_PATH" ]; then
  echo "World file not found: $WORLD_PATH" >&2
  exit 1
fi

export RMPD_TEST_MODE="${RMPD_TEST_MODE:-amcl}"
export RMPD_WEBOTS_WORLD="$WORLD_PATH"
export RMPD_START_CHECKPOINT_PATROL="${RMPD_START_CHECKPOINT_PATROL:-false}"
export RMPD_START_LIVE_MAPPING="${RMPD_START_LIVE_MAPPING:-true}"
export RMPD_LIVE_MAP_WIDTH_M="${RMPD_LIVE_MAP_WIDTH_M:-16.0}"
export RMPD_LIVE_MAP_HEIGHT_M="${RMPD_LIVE_MAP_HEIGHT_M:-16.0}"
export RMPD_LIVE_MAP_ORIGIN_X="${RMPD_LIVE_MAP_ORIGIN_X:--4.0}"
export RMPD_LIVE_MAP_ORIGIN_Y="${RMPD_LIVE_MAP_ORIGIN_Y:--4.0}"
export RMPD_QUICK_TEST_RVIZ_CONFIG="${RMPD_QUICK_TEST_RVIZ_CONFIG:-amcl.rviz}"

exec bash "$REPO_DIR/scripts/quick_test.sh"
