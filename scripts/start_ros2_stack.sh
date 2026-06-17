#!/bin/bash
set -eo pipefail

cd /workspace
set +u
source /opt/ros/jazzy/setup.bash
set -u
rm -rf build/robot_patrol_node install/robot_patrol_node log/latest
colcon build --packages-select robot_patrol_node --symlink-install
set +u
source install/setup.bash
set -u
exec ros2 launch robot_patrol_node mapping_stack.launch.py
