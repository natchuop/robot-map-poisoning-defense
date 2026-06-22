#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export RMPD_WEBOTS_WORLD="$REPO_DIR/webots/worlds/office/office.wbt"
export RMPD_AMCL_MAP_NAME="office"
export RMPD_AMCL_INITIAL_POSE_X="-4.35"
export RMPD_AMCL_INITIAL_POSE_Y="-5.35"
export RMPD_AMCL_INITIAL_POSE_YAW="0.004641574238792719"
export WEBOTS_DRIVE_SPEED="${WEBOTS_DRIVE_SPEED:-1.2}"
export WEBOTS_TURN_SPEED="${WEBOTS_TURN_SPEED:-0.8}"

exec bash "$REPO_DIR/scripts/quick_test.sh"
