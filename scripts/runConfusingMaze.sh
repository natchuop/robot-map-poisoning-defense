#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export RMPD_WEBOTS_WORLD="$REPO_DIR/webots/worlds/confusingMaze/confusing_maze.wbt"
export RMPD_AMCL_MAP_NAME="confusing_maze"
export RMPD_AMCL_INITIAL_POSE_X="-3.5"
export RMPD_AMCL_INITIAL_POSE_Y="-3.5"
export RMPD_AMCL_INITIAL_POSE_YAW="0.0"
export RMPD_AMCL_INITIAL_POSE_USE_ODOM="${RMPD_AMCL_INITIAL_POSE_USE_ODOM:-false}"
export RMPD_QUICK_TEST_RVIZ_CONFIG="amcl.rviz"
export RMPD_START_CHECKPOINT_PATROL="${RMPD_START_CHECKPOINT_PATROL:-false}"
export RMPD_START_LIVE_MAPPING="${RMPD_START_LIVE_MAPPING:-true}"
export WEBOTS_DRIVE_SPEED="${WEBOTS_DRIVE_SPEED:-2.4}"
export WEBOTS_TURN_SPEED="${WEBOTS_TURN_SPEED:-1.6}"

exec bash "$REPO_DIR/scripts/quick_test.sh"
