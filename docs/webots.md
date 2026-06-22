# Webots Guide

This repo currently uses a TurtleBot3 Burger world in Webots and a ROS 2 bridge running in Docker.

## Current Demo

- Main AMCL world: `webots/worlds/testRvizMap/turtlebot3_burger.wbt`
- Office world: `webots/worlds/office/office.wbt`
- Live map-building world: `webots/worlds/testBuildingMapForRobot/turtlebot3_burger.wbt`
- Known AMCL map: `webots/worlds/testRvizMap/amcl_map/arena.yaml` and `arena.pgm`
- Office AMCL map: `webots/worlds/office/amcl_map/office.yaml` and `office.pgm`
- Checkpoint patrol controller: `webots/controllers/patrol_robot/patrol_robot.py`
- User-controlled controller: `webots/controllers/user_controlled_robot/user_controlled_robot.py`
- Webots wrapper: `webots/worlds/controllers/patrol_robot/patrol_robot.py`
- User-controlled wrapper: `webots/worlds/controllers/user_controlled_robot/user_controlled_robot.py`

The office map is built from the world walls and floor bounds only. Furniture and other movable objects are left out so AMCL keys off the room structure.

The wrappers exist only because the world tree is nested. Keep behavior in `webots/controllers/`, and keep the wrapper files as thin forwarders.

## Data Flow

Webots world -> Python controller -> ROS bridge -> `/robot_pose` and `/scan` -> AMCL / Nav2 -> `/cmd_vel` -> ROS bridge -> Webots

The bridge also forwards `/active_checkpoint` to Webots and publishes `/webots_checkpoint_contact` when the robot footprint touches the active colored checkpoint block.

## Nav2 In 2D

Nav2 works from a 2D occupancy grid:

- `map_server` loads the static map, which usually captures walls and other fixed structure.
- AMCL estimates the robot's pose on that map using lidar scans.
- The global planner picks a route to the goal using the static map.
- The local costmap uses live sensor data to react to new obstacles like boxes, chairs, or people.

So the map gives Nav2 the room layout, and lidar gives it the chance to slow down, stop, or route around things that are not in the static map.

## Quick Test

Run:

```bash
bash scripts/quick_test.sh
```

By default this launches the AMCL smoke test. If you want the older mapping path instead, run:

```bash
RMPD_TEST_MODE=mapping bash scripts/quick_test.sh
```

For the new live robot-built map demo, run:

```bash
bash scripts/runTestBuildingMapForRobot.sh
```

To launch the office world, point `RMPD_WEBOTS_WORLD` at `webots/worlds/office/office.wbt` before running the script.

For convenience, `bash scripts/runOffice.sh` launches the office world with its office-specific AMCL map, startup pose, simplified office RViz config, and WASD user-controlled robot. The office robot starts at `x=-4.35`, `y=-5.35`, `yaw=0.00464`; the script publishes that configured AMCL initial pose and disables checkpoint patrol and the live-map RViz overlay.

Mapping mode builds `/map` from Webots pose and LiDAR. AMCL mode localizes against the known map.

In default AMCL mode, RViz uses `amcl.rviz` and displays both the static `/map` and the robot-built `/live_map`. The live map uses RViz's costmap color scheme and can appear pink or purple. The office script uses `office_amcl.rviz`, which hides `/live_map` so the static office map, scan, AMCL particles, and TF are easier to inspect. The test-building script uses mapping mode, so it only shows the live occupancy map as `/map`.

## Controller Contract

The controllers expect these Webots devices:

- `gps`
- `inertial unit`
- `LDS-01`
- `LDS-01_main_motor`
- `LDS-01_secondary_motor`
- `left wheel motor`
- `right wheel motor`

They send newline-delimited JSON over TCP by default. The default bridge target is `tcp://172.28.64.1:5005`, with `127.0.0.1` as fallback.

## Controller Choices

- `patrol_robot` is the Nav2-capable checkpoint patrol controller and is used by `testRvizMap`.
- `user_controlled_robot` is the WASD controller used by the office and live map-building demos.

## Adding A World

1. Create `webots/worlds/<world_name>/`.
2. Add the `.wbt` world file.
3. Set the world `controller` field to `patrol_robot` or `user_controlled_robot`.
4. Put world-specific map or route data alongside that world.
5. Run `bash scripts/quick_test.sh` and `bash scripts/verify.sh`.

If you want the office world specifically, use `bash scripts/runOffice.sh`.
