#!/bin/bash
set -eo pipefail

TOPIC="${1:-/robot_pose}"
OUTFILE="${2:-/tmp/topic_watch.log}"

set +u
source /opt/ros/jazzy/setup.bash
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi
set -u

timeout 30 ros2 topic echo "$TOPIC" --once > "$OUTFILE" 2>&1
