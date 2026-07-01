# Bright brown map-poisoning test worlds

This bundle contains only two test worlds:

1. `simpleCorridor/simple_corridor.wbt`
   - One straight corridor.
   - Fake-obstacle report target: `(0.0, 0.0)`.
   - Run it with `bash scripts/runSimpleCorridor.sh`.
   - Uses the `patrol_robot` controller and auto-starts patrol by default.

2. `twoRoute/two_route.wbt`
   - Exactly two paths between start and goal:
     - upper, shorter route
     - lower, longer backup route
   - Fake-obstacle report target on the upper path: `(0.0, 1.05)`.
   - Run it with `bash scripts/runTwoRoute.sh`.
   - Uses the `patrol_robot` controller and auto-starts patrol by default.

Both floors use the included warm brown checkerboard:
`webots/worlds/textures/bright_brown_checkerboard.png`

The green, red, and yellow squares are non-physical visual markers. Robots can drive through them.

## Webots Reference

The actual controller code lives in `webots/robot_controllers/`, and the world-tree wrappers in `webots/worlds/controllers/` just forward to those controllers.

Controller choices in the current demo worlds:

- `testRvizMap`, `simpleCorridor`, and `twoRoute` use `patrol_robot`
- `office`, `confusingMaze`, and `sandbox` use `user_controlled_robot`
- `TestCombineRvizMap` uses `user_controlled_robot` for both robots

Each world keeps its own generated AMCL map inside `amcl_map/`, and the matching `run*.sh` script launches that world with the right map and startup pose.

The controllers expect these Webots devices:

- `gps`
- `inertial unit`
- `LDS-01`
- `LDS-01_main_motor`
- `LDS-01_secondary_motor`
- `left wheel motor`
- `right wheel motor`

The bridge sends newline-delimited JSON from Webots into ROS 2 and publishes pose, scan, odom, and checkpoint state back out.
