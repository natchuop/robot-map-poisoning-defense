#!/bin/bash
# Source ROS 2 and this workspace (run inside Docker).
source /opt/ros/jazzy/setup.bash
if [[ -f "$(dirname "${BASH_SOURCE[0]}")/../install/setup.bash" ]]; then
  source "$(dirname "${BASH_SOURCE[0]}")/../install/setup.bash"
fi
