#!/bin/bash
set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-/workspace}"
if [ -f "$CONTAINER_WORKSPACE/install/setup.bash" ]; then
  set +u
  source "$CONTAINER_WORKSPACE/install/setup.bash"
  set -u
fi

echo "ROS binary: $(command -v ros2)"
echo "Nodes:"
ros2 node list
echo "Topics:"
ros2 topic list
