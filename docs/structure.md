# Repo Structure

This repo is built around one active ROS 2 + Webots demo and a few supporting docs.

## Where To Work

- `webots/robot_controllers/patrol_robot/patrol_robot.py` is the Nav2-capable checkpoint patrol controller used by `testRvizMap`.
- `webots/robot_controllers/user_controlled_robot/user_controlled_robot.py` is the WASD controller used by the office and live map-building worlds.
- `webots/worlds/<world_name>/` holds each Webots world and its world-specific assets.
- `webots/worlds/controllers/<controller_name>/<controller_name>.py` files are only Webots lookup wrappers.
- `src/robot_patrol_node/` is the active ROS 2 package for the bridge, map builder, AMCL helpers, launches, and RViz configs.
- `scripts/quick_test.sh` runs the end-to-end demo.
- `scripts/runOffice.sh` runs the office world with its own AMCL map, RViz config, and configured initial pose.
- `scripts/runConfusingMaze.sh` runs the confusing maze world with its own AMCL map and configured initial pose.
- `scripts/runSandbox.sh` runs the sandbox world with its own AMCL map and configured initial pose.
- `scripts/runTestBuildingMapForRobot.sh` runs the test-building world with the same AMCL default as the main quick test.
- `scripts/verify.sh` runs the headless environment check.
- `docker/` holds the Dockerfile and compose files.
- `docs/` holds the project notes, structure guide, Webots setup guide, and verification guide.

## How Things Connect

1. Webots runs the world and controller.
2. The controller sends pose and scan packets to the ROS 2 bridge in Docker.
3. The bridge republishes those packets as ROS topics and also publishes `/odom`.
4. The AMCL stack localizes the robot against the known map.
5. Nav2 sends `/cmd_vel` through the bridge to `patrol_robot` for checkpoint patrols.
6. The bridge also forwards `/active_checkpoint` and reports Webots checkpoint contact events back to ROS.
7. RViz shows the static map, live map, scan, path, and navigation state.

## Generated Files

These are disposable and should not be edited by hand:

- `build/`
- `install/`
- `log/`
- `logs/`
- `__pycache__/`

## Adding A New World

Add a new world under `webots/worlds/<world_name>/`, choose the controller that matches the test, and keep world-specific paths, trust values, and map outputs inside that world's config or launch flow.

Patrol worlds should use `patrol_robot`. Exploration or operator-driven worlds should use `user_controlled_robot`.
