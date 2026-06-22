# Repo Structure

This repo is built around one active ROS 2 + Webots demo and a few supporting docs.

## Where To Work

- `webots/controllers/anna_bot/anna_bot.py` is the older autonomous controller used by `testRvizMap`.
- `webots/controllers/patrol_robot/patrol_robot.py` is the shared autonomous patrol controller.
- `webots/controllers/patrol_robot/user_controlled_robot.py` is the WASD controller used by the office and live map-building worlds.
- `webots/worlds/<world_name>/` holds each Webots world and its world-specific assets.
- `webots/worlds/controllers/<controller_name>/<controller_name>.py` files are only Webots lookup wrappers.
- `src/robot_patrol_node/` is the active ROS 2 package for the bridge, map builder, AMCL helpers, launches, and RViz configs.
- `scripts/quick_test.sh` runs the end-to-end demo.
- `scripts/verify.sh` runs the headless environment check.
- `docker/` holds the Dockerfile and compose files.
- `docs/` holds the project notes, structure guide, Webots setup guide, and verification guide.

## How Things Connect

1. Webots runs the world and controller.
2. The controller sends pose and scan packets to the ROS 2 bridge in Docker.
3. The ROS package republishes those packets as ROS topics.
4. The AMCL stack localizes the robot against the known map.
5. Nav2 uses the static map for planning and the live local costmap for obstacle avoidance.
6. RViz shows the result.

## Generated Files

These are disposable and should not be edited by hand:

- `build/`
- `install/`
- `log/`
- `logs/`
- `__pycache__/`

## Adding A New World

Add a new world under `webots/worlds/<world_name>/`, choose the controller that matches the test, and keep world-specific paths, trust values, and map outputs inside that world's config or launch flow.
