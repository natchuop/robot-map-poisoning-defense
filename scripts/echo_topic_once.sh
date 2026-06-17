#!/bin/bash
set -eo pipefail

TOPIC="${1:-/robot_pose}"

set +u
source /opt/ros/jazzy/setup.bash
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi
set -u

exec ros2 topic echo "$TOPIC" --once
