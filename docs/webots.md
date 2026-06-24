# Webots Guide

This repo currently uses TurtleBot3 Burger worlds in Webots and a ROS 2 bridge running in Docker.
The longer-term Webots direction is to support trust-based shared mapping, verification, and quarantine across multiple robots.

## Current Demo

- Main AMCL world: `webots/worlds/testRvizMap/turtlebot3_burger.wbt`
- Office world: `webots/worlds/office/office.wbt`
- Live map-building world: `webots/worlds/testBuildingMapForRobot/turtlebot3_burger.wbt`
- Sandbox world: `webots/worlds/sandbox/sandbox.wbt`
- Shared-map sandbox copy: `webots/worlds/TestCombineRvizMap/TestCombineRvizMap.wbt`
- Known AMCL map: `webots/worlds/testRvizMap/amcl_map/arena.yaml` and `arena.pgm`
- Office AMCL map: `webots/worlds/office/amcl_map/office.yaml` and `office.pgm`
- Confusing maze AMCL map: `webots/worlds/confusingMaze/amcl_map/confusing_maze.yaml` and `confusing_maze.pgm`
- Sandbox AMCL map: `webots/worlds/sandbox/amcl_map/sandbox.yaml` and `sandbox.pgm`
- Checkpoint patrol controller: `webots/robot_controllers/patrol_robot/patrol_robot.py`
- User-controlled controller: `webots/robot_controllers/user_controlled_robot/user_controlled_robot.py`
- Webots wrapper: `webots/worlds/controllers/patrol_robot/patrol_robot.py`
- User-controlled wrapper: `webots/worlds/controllers/user_controlled_robot/user_controlled_robot.py`

The office map is built from the world walls and floor bounds only. Furniture and other movable objects are left out so AMCL keys off the room structure.

The wrappers exist only because the world tree is nested. Keep behavior in `webots/robot_controllers/`, and keep the wrapper files in `webots/worlds/controllers/` as thin forwarders.

## Data Flow

Webots world -> Python controller -> ROS bridge -> `/robot_pose`, `/scan`, and `/odom` -> AMCL / Nav2 -> `/cmd_vel` -> ROS bridge -> Webots

The bridge also forwards `/active_checkpoint` to Webots and publishes `/webots_checkpoint_contact` when the robot center reaches the active colored checkpoint block.

For the trust-based project plan, the next layer on top of this flow is:

Webots world -> controller -> ROS bridge -> shared map update -> trust weighting -> map cell confidence -> navigation decision

That future flow uses:

- robot trust to measure how reliable a reporting robot seems
- trust confidence to measure how much evidence supports that trust score
- map cell confidence to decide whether a cell is occupied, clear, suspicious, or disputed
- verification scans to compare a report with real LiDAR data
- quarantine rules to ignore robots that are both untrusted and well supported by evidence

Example planned update shape:

```json
{
  "cell_x": 10,
  "cell_y": 12,
  "occupied": true,
  "reporting_robot": "robot_3"
}
```

The idea is not to accept that report automatically. Instead, the system would weight it using the reporting robot's trust and trust confidence before deciding how much it should change the shared map.

## Nav2 In 2D

Nav2 works from a 2D occupancy grid:

- `map_server` loads the static map, which usually captures walls and other fixed structure.
- AMCL estimates the robot's pose on that map using lidar scans.
- The global planner picks a route to the goal using the static map.
- The local costmap uses live sensor data to react to new obstacles like boxes, chairs, or people.

So the map gives Nav2 the room layout, and lidar gives it the chance to slow down, stop, or route around things that are not in the static map. The current Nav2 tuning uses Regulated Pure Pursuit and tighter obstacle-aware costmaps to keep the patrol behavior conservative near walls and furniture.

That same structure is a good fit for the project plan because the shared-map defense also works on occupancy-grid cells. The plan is to update confidence per cell instead of treating every robot report as equally trustworthy.

## Quick Test

Run:

```bash
bash scripts/quick_test.sh
```

By default this launches the AMCL smoke test. If you want the older mapping path instead, run:

```bash
RMPD_TEST_MODE=mapping bash scripts/quick_test.sh
```

For the test-building world with the same remembered-map overlay as the quick test, run:

```bash
bash scripts/runTestBuildingMapForRobot.sh
```

If you specifically want the older live-mapping-only flow, run:

```bash
RMPD_TEST_MODE=mapping bash scripts/runTestBuildingMapForRobot.sh
```

To launch the office world, point `RMPD_WEBOTS_WORLD` at `webots/worlds/office/office.wbt` before running the script.

For convenience, `bash scripts/runOffice.sh` launches the office world with its office-specific AMCL map, startup pose, the office RViz view, and WASD user-controlled robot. The office robot starts at `x=-4.35`, `y=-5.35`, `yaw=0.00464`; the script publishes that configured AMCL initial pose and keeps the live `/live_map` overlay enabled so previously explored areas remain visible.

For convenience, `bash scripts/runConfusingMaze.sh` launches `webots/worlds/confusingMaze/confusing_maze.wbt` with its generated AMCL map and the `user_controlled_robot` controller. The maze robot starts at `x=-3.5`, `y=-3.5`, `yaw=0.0`.

For convenience, `bash scripts/runSandbox.sh` launches `webots/worlds/sandbox/sandbox.wbt` with its generated AMCL map and the `user_controlled_robot` controller. The sandbox robot starts at `x=2.0`, `y=2.0`, `yaw=0.0`.

For convenience, `bash scripts/runTestCombineRvizMap.sh` launches `webots/worlds/TestCombineRvizMap/TestCombineRvizMap.wbt` with two `user_controlled_robot` TurtleBots, two identical RViz windows, a merged `/shared_live_map`, and a `/shared_confidence_map` overlay. `robot_1` starts at `x=2.0`, `y=2.0`, `yaw=0.0` and uses WASD. `robot_2` starts at `x=-2.0`, `y=-2.0`, `yaw=1.5708` and uses the arrow keys.

Mapping mode builds `/map` from Webots pose and LiDAR. AMCL mode localizes against the known map.

The shared two-robot mapping flow uses separate bridge ports and topics per robot, then merges the per-robot maps into `/shared_live_map`. The confidence overlay currently marks every observed cell with full confidence so both RViz windows show a single-color heat-map layer until trust weighting is added later.

In default AMCL mode, RViz uses `amcl.rviz` and displays both the static `/map` and the robot-built `/live_map`. The live map uses RViz's costmap color scheme and can appear pink or purple. The office script uses `office_amcl.rviz` plus office-specific startup pose settings so the larger office map and remembered overlay remain visible. The test-building script now also uses AMCL mode by default so it matches the quick-test remembered-map behavior. The confusing maze and sandbox scripts reuse the same AMCL flow with world-specific map sizes and initial poses.

## Project Plan Fit

The project plan in `docs/project_plan.md` adds trust behavior on top of the current Webots setup:

1. Multiple robots share map updates.
2. One robot can be compromised and inject fake occupied cells.
3. Honest robots verify reports with LiDAR.
4. Trust scores and trust confidence decide how much to believe each reporter.
5. Map cell confidence determines whether the cell should be treated as occupied, clear, suspicious, or disputed.
6. Low-trust, high-confidence attackers can be quarantined so their reports stop influencing navigation.

## Controller Contract

The controllers expect these Webots devices:

- `gps`
- `inertial unit`
- `LDS-01`
- `LDS-01_main_motor`
- `LDS-01_secondary_motor`
- `left wheel motor`
- `right wheel motor`

They send newline-delimited JSON over TCP by default. The default bridge target is `tcp://172.28.64.1:5005`, with `127.0.0.1` as fallback. The ROS bridge listens on both TCP and UDP port `5005`, and in AMCL mode it also publishes `/odom` for the localization stack and the initial-pose helper.

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
