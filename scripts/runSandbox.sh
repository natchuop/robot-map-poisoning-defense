#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export RMPD_WEBOTS_WORLD="$REPO_DIR/webots/worlds/sandbox/sandbox.wbt"
export RMPD_AMCL_MAP_NAME="sandbox"
export RMPD_AMCL_INITIAL_POSE_X="2.0"
export RMPD_AMCL_INITIAL_POSE_Y="2.0"
export RMPD_AMCL_INITIAL_POSE_YAW="0.0"
export RMPD_AMCL_INITIAL_POSE_USE_ODOM="${RMPD_AMCL_INITIAL_POSE_USE_ODOM:-false}"
export RMPD_QUICK_TEST_RVIZ_CONFIG="amcl.rviz"
export RMPD_START_CHECKPOINT_PATROL="${RMPD_START_CHECKPOINT_PATROL:-false}"
export RMPD_START_LIVE_MAPPING="${RMPD_START_LIVE_MAPPING:-true}"
export RMPD_LIVE_MAP_WIDTH_M="${RMPD_LIVE_MAP_WIDTH_M:-10.0}"
export RMPD_LIVE_MAP_HEIGHT_M="${RMPD_LIVE_MAP_HEIGHT_M:-10.0}"
export RMPD_LIVE_MAP_ORIGIN_X="${RMPD_LIVE_MAP_ORIGIN_X:--4.0}"
export RMPD_LIVE_MAP_ORIGIN_Y="${RMPD_LIVE_MAP_ORIGIN_Y:--4.0}"

exec bash "$REPO_DIR/scripts/quick_test.sh"
