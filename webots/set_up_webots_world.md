# Set Up A Webots World For ROS 2, Docker, And RViz

This document gives an AI enough context to add or repair a Webots world in this repository.

## How The Pieces Connect

Webots runs on the laptop and simulates the robot, sensors, and world geometry. A Webots Python controller reads GPS, IMU, and LiDAR data, converts that data into a small JSON packet, and sends it over TCP to Docker on port `5005`.

Docker runs ROS 2 Jazzy. Inside Docker, `robot_patrol_node` starts a bridge node that listens on port `5005`, publishes `/robot_pose` and `/scan`, and starts a map builder that publishes `/map` and `/tf`. RViz also runs from Docker and loads `default.rviz`, which already displays `/map`, `/scan`, and `TF` using fixed frame `map`.

Data flow:

```text
Webots world -> Webots Python controller -> tcp://*:5005 -> Docker ROS 2 bridge -> /robot_pose + /scan -> map builder -> /map + /tf -> RViz
```

## Existing Working Example

World:

```text
webots/worlds/testRvizMap/turtlebot3_burger.wbt
```

Real controller:

```text
webots/controllers/testRvizMap/testRvizMap.py
```

World-local Webots controller wrapper:

```text
webots/worlds/controllers/testRvizMap/testRvizMap.py
```

ROS bridge:

```text
src/robot_patrol_node/robot_patrol_node/udp_bridge_node.py
```

Map builder:

```text
src/robot_patrol_node/robot_patrol_node/map_builder_node.py
```

Launch files:

```text
src/robot_patrol_node/launch/mapping_stack.launch.py
src/robot_patrol_node/launch/rviz.launch.py
```

RViz config:

```text
src/robot_patrol_node/config/default.rviz
```

## Webots World Requirements

The `.wbt` file must include a robot with a controller name that matches a controller folder.

Example:

```text
TurtleBot3Burger {
  translation 0 0 0
  rotation 0 1 0 0
  controller "testRvizMap"
  extensionSlot [
    GPS {
    }
    InertialUnit {
    }
    RobotisLds01 {
    }
  ]
}
```

The current controller expects these Webots device names:

```text
gps
inertial unit
LDS-01
LDS-01_main_motor
LDS-01_secondary_motor
left wheel motor
right wheel motor
```

If the robot or sensor PROTO uses different device names, update `webots/controllers/<controller_name>/<controller_name>.py` to match the new names.

## Controller Folder Rules

Webots searches for controller code near the world. For this repo, keep a small wrapper under:

```text
webots/worlds/controllers/<controller_name>/<controller_name>.py
```

That wrapper should forward to the real controller under:

```text
webots/controllers/<controller_name>/<controller_name>.py
```

Example wrapper:

```python
from pathlib import Path
import runpy


REPO_ROOT = Path(__file__).resolve().parents[4]
REAL_CONTROLLER = REPO_ROOT / 'webots' / 'controllers' / 'testRvizMap' / 'testRvizMap.py'

runpy.run_path(str(REAL_CONTROLLER), run_name='__main__')
```

If creating a new controller, change both folder names and `REAL_CONTROLLER` to the new controller name.

## Bridge Packet Contract

The Webots controller sends newline-delimited JSON over TCP by default.

Default controller settings:

```text
WEBOTS_BRIDGE_PROTOCOL=tcp
WEBOTS_BRIDGE_TARGETS=172.28.64.1,127.0.0.1
WEBOTS_BRIDGE_PORT=5005
```

The bridge packet must include:

```json
{
  "pose": {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0
  },
  "scan": {
    "angle_min": -3.14159,
    "angle_max": 3.14159,
    "angle_increment": 0.01745,
    "range_min": 0.05,
    "range_max": 4.0,
    "scan_time": 0.064,
    "ranges": [1.0, 1.1, 1.2]
  }
}
```

ROS publishes this as:

```text
/robot_pose  geometry_msgs/Pose2D
/scan        sensor_msgs/LaserScan
/map         nav_msgs/OccupancyGrid
/tf          map -> base_link
/tf_static   base_link -> laser
```

## Docker Requirements

The main compose file must expose port `5005` for TCP and UDP:

```yaml
ports:
  - "5005:5005/udp"
  - "5005:5005/tcp"
```

Windows RViz GUI uses:

```text
docker-compose.wslg.yml
```

macOS should not use `docker-compose.wslg.yml`. macOS RViz GUI from Docker may need XQuartz or another X11 setup.

## ROS 2 Package Requirements

If adding new launch or config files, include them in:

```text
src/robot_patrol_node/setup.py
```

The current package installs:

```python
('share/' + package_name + '/launch', ['launch/mapping_stack.launch.py']),
('share/' + package_name + '/launch', ['launch/rviz.launch.py']),
('share/' + package_name + '/config', ['config/default.rviz']),
```

The launch file used for the full stack is:

```bash
ros2 launch robot_patrol_node mapping_stack.launch.py
```

The helper script wraps build, source, and launch:

```bash
bash scripts/start_ros2_stack.sh
```

## RViz Requirements

`default.rviz` should keep:

```text
Fixed Frame: map
Map topic: /map
LaserScan topic: /scan
TF enabled
```

If RViz opens but the view is empty, first verify that `/robot_pose`, `/scan`, and `/map` are publishing before changing RViz settings.

## How To Add A New World

1. Create a new folder:

```text
webots/worlds/<world_name>/
```

2. Put the `.wbt` file in that folder.

3. Add or reuse a controller:

```text
webots/controllers/<controller_name>/<controller_name>.py
webots/worlds/controllers/<controller_name>/<controller_name>.py
```

4. In the `.wbt` robot node, set:

```text
controller "<controller_name>"
```

5. Make sure the robot includes GPS, IMU, and LiDAR devices, or update the controller to match the new sensors.

6. Start the ROS stack:

```bash
docker rm -f ros2_dev
docker compose -f docker-compose.yml -f docker-compose.wslg.yml run --rm --service-ports --name ros2_dev ros2
bash scripts/start_ros2_stack.sh
```

7. Open the world in Webots and press Play.

8. Launch RViz from another terminal:

```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --packages-select robot_patrol_node --symlink-install
source install/setup.bash
ros2 launch robot_patrol_node rviz.launch.py
```

9. Check topics if needed:

```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 topic echo /robot_pose
ros2 topic echo /scan
ros2 topic echo /map
```

## Verification

Before committing world or controller changes, run:

```bash
bash scripts/verify.sh
```

Expected result:

```text
Results: ... passed, 0 failed
All checks passed.
```

This proves Docker builds, ROS 2 builds, the bridge accepts packets, `/robot_pose` and `/scan` publish, `/map` publishes, and `rviz2` is installed.
