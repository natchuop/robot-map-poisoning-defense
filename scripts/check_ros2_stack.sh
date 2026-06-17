#!/bin/bash
set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

if [ -f /workspace/install/setup.bash ]; then
  set +u
  source /workspace/install/setup.bash
  set -u
fi

echo "ROS binary: $(command -v ros2)"
echo "Nodes:"
ros2 node list
echo "Topics:"
ros2 topic list
