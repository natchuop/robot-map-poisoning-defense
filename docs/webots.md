# Webots Guide

This repo currently uses a TurtleBot3 Burger world in Webots and a ROS 2 bridge running in Docker.

## Current Demo

- World: `webots/worlds/testRvizMap/turtlebot3_burger.wbt`
- Known AMCL map: `webots/worlds/testRvizMap/amcl_map/arena.yaml` and `arena.pgm`
- Shared controller: `webots/controllers/patrol_robot/patrol_robot.py`
- Webots wrapper: `webots/worlds/controllers/patrol_robot/patrol_robot.py`

The wrapper exists only because the world tree is nested. Keep behavior in the shared controller.

## Data Flow

Webots world -> Python controller -> ROS bridge -> `/robot_pose` and `/scan` -> AMCL / map server -> `/map`, `/odom`, `/tf` -> RViz

## Quick Test

Run:

```bash
bash scripts/quick_test.sh
```

By default this launches the AMCL smoke test. If you want the older mapping path instead, run:

```bash
RMPD_TEST_MODE=mapping bash scripts/quick_test.sh
```

Mapping mode builds `/map` from Webots pose and LiDAR. AMCL mode localizes against the known map.

## Controller Contract

The controller expects these Webots devices:

- `gps`
- `inertial unit`
- `LDS-01`
- `LDS-01_main_motor`
- `LDS-01_secondary_motor`
- `left wheel motor`
- `right wheel motor`

It sends newline-delimited JSON over TCP by default. The default bridge target is `tcp://172.28.64.1:5005`, with `127.0.0.1` as fallback.

## Adding A World

1. Create `webots/worlds/<world_name>/`.
2. Add the `.wbt` world file.
3. Reuse the shared controller name `patrol_robot`.
4. Put world-specific map or route data alongside that world.
5. Run `bash scripts/quick_test.sh` and `bash scripts/verify.sh`.

