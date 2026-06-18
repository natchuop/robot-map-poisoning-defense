#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_PATH="$REPO_DIR/webots/worlds/testBuildingMapForRobot/turtlebot3_burger.wbt"

if [ ! -f "$WORLD_PATH" ]; then
  echo "World file not found: $WORLD_PATH" >&2
  exit 1
fi

export RMPD_TEST_MODE="${RMPD_TEST_MODE:-mapping}"
export RMPD_WEBOTS_WORLD="$WORLD_PATH"

exec bash "$REPO_DIR/scripts/quick_test.sh"
