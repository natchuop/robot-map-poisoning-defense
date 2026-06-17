#!/bin/bash
set -eo pipefail

TOPIC="${1:-/robot_pose}"
TYPE_NAME="${2:-}"
OUTFILE="${3:-/tmp/topic_watch.log}"
TIMEOUT_SECONDS="${4:-30}"
CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-/workspace}"

set +u
source /opt/ros/jazzy/setup.bash
if [ -f "$CONTAINER_WORKSPACE/install/setup.bash" ]; then
  source "$CONTAINER_WORKSPACE/install/setup.bash"
fi
set -u

if [ -n "$TYPE_NAME" ]; then
  timeout "$TIMEOUT_SECONDS" ros2 topic echo "$TOPIC" "$TYPE_NAME" --once > "$OUTFILE" 2>&1
else
  timeout "$TIMEOUT_SECONDS" ros2 topic echo "$TOPIC" --once > "$OUTFILE" 2>&1
fi
