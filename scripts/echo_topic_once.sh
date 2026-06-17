#!/bin/bash
set -eo pipefail

TOPIC="${1:-/robot_pose}"
CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-/workspace}"

set +u
source /opt/ros/jazzy/setup.bash
if [ -f "$CONTAINER_WORKSPACE/install/setup.bash" ]; then
  source "$CONTAINER_WORKSPACE/install/setup.bash"
fi
set -u

exec ros2 topic echo "$TOPIC" --once
