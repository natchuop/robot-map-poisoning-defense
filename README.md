# Robot Map Poisoning Defense

ROS 2 + Webots simulation for studying decentralized defenses against robot-to-robot map poisoning.

The latest project direction is documented in:

- [docs/project_plan.md](docs/project_plan.md)
- [docs/file_structure.md](docs/file_structure.md)
- [docs/file_verification.md](docs/file_verification.md)
- [docs/equations.md](docs/equations.md)

## Current Project Focus

The current experiment plan compares three trust and confidence systems/methods:

```text
1. log_odds
2. mate_log_odds
3. mate_claim_verification
```

`log_odds` treats all robot reports as fully trusted. `mate_log_odds` uses MATE-style Bayesian robot trust with optional trust propagation, but still performs simple trust-weighted log-odds fusion. `mate_claim_verification` extends MATE with claim-level weights, occupied/free evidence layers, suspicious/disputed states, and quarantine.

The Method 1 baseline is now implemented as the real full-trust log-odds path. It uses claim-based `MapUpdate` messages, stores active claims by `claim_id`, and rebuilds each shared map from local evidence plus active claims on every publish.

The main research question is whether decentralized MATE-based trust-weighted map fusion can reduce the effects of map-poisoning attacks on navigation and final map accuracy.

The current shared-mapping baseline uses:

- per-robot live maps and confidence maps
- a log-odds shared map per robot with claim-based fake-obstacle fusion
- per-robot RViz windows for shared live maps and confidence overlays
- temporary fake obstacle injections published as `MapUpdate` claims that can later be cleared by real LiDAR evidence
- a LiDAR confidence falloff that reduces both score updates and current-observation certainty as range increases

The RViz semantic overlay uses `MarkerArray` colors so overlapping robot contributions can be blended without replacing the normal shared occupancy grid used for navigation. Higher confidence now renders as darker and bolder overlay colors.

The docs also describe the longer-term defense flow:

```text
robot report -> MATE robot trust -> trust confidence -> claim verification -> map-cell confidence -> navigation decision
```

## What You Need

Windows:

- Windows 11
- WSL2 with Ubuntu 24.04
- Docker Desktop with WSL integration enabled
- Webots R2025a installed on Windows
- Git inside WSL

macOS:

- macOS
- Docker Desktop
- Webots R2025a installed on macOS
- Git
- Python 3

ROS 2 runs in Docker, not on the host. The verification and quick-test scripts also expect `python3` on the host or inside WSL on Windows.

## First Time Setup

Before cloning:

- Install Docker Desktop.
- Install Webots R2025a.
- Install Git.
- On Windows, also install WSL2 with Ubuntu 24.04.
  - Then, inside Docker Desktop go to settings and enable WSL Integration

<p align="center"><strong>Windows Docker Settings</strong></p>

![Windows Docker Settings](media/WindowsDockerSettings.png "Windows Docker Settings")

Then clone and run the following commands in the Linux terminal:

```bash
git clone git@github.com:natchuop/robot-map-poisoning-defense.git
cd robot-map-poisoning-defense
docker system prune -a --volumes -f
docker builder prune -a -f
docker compose -f docker/compose.yml build --no-cache
bash scripts/verify.sh
```

Make sure all checks from `verify.sh` pass.

If you prefer HTTPS, clone with your GitHub HTTPS URL instead.

## Final Testing After Setup

Once verification passes, run the main smoke test:

```bash
bash scripts/quick_test.sh
```

That launches Webots, ROS 2, RViz, and the AMCL/Nav2 checkpoint patrol demo together. You should have Webots open automatically with a robot driving by itself along a predetermined route. A live RViz window should also pop up and start recording what the robot's lidar sensor is sensing. Press `Ctrl-C` in the terminal, not in Webots, to close everything.

## Map Accuracy Evaluator

The map accuracy evaluator compares each robot's final shared occupancy map against the clean ground-truth map. The shared live map is the primary evaluation target. The confidence map is only used as a secondary diagnostic to help explain behavior around suspicious or false cells.

### Fake-obstacle demo

`runTestFakeObstacle.sh` enables the evaluator by default and writes CSVs under `results/map_accuracy/`.

```bash
# Fake-obstacle demo with evaluator enabled by default
./scripts/runTestFakeObstacle.sh

# Fake-obstacle demo with evaluator disabled
RMPD_ENABLE_MAP_ACCURACY_EVALUATOR=false ./scripts/runTestFakeObstacle.sh
```

Expected outputs:

- `results/map_accuracy/raw/map_accuracy_timeseries.csv`: periodic time-series rows for whole-map metrics and attack-region metrics.
- `results/map_accuracy/processed/summary_by_trial.csv`: final summary rows written at the end of each trial.

### Quick test

`quick_test.sh` keeps the evaluator opt-in so the broader smoke test stays lighter. Enable it with `RMPD_ENABLE_MAP_ACCURACY_EVALUATOR=true`.

```bash
# Quick test with evaluator enabled
RMPD_ENABLE_MAP_ACCURACY_EVALUATOR=true ./scripts/quick_test.sh
```

If you want a faster smoke test, leave the variable unset or set it to `false`.

### Output location

The evaluator always writes to `results/map_accuracy/` unless you override `RMPD_MAP_ACCURACY_RESULTS_DIR`. The time-series CSV tracks metrics over time, while the summary CSV captures the final per-trial values.

<p align="center"><strong>runTestFakeObstacle.sh Tutorial Video</strong></p>

<video controls src="media/FakeObstacleTutorial.mp4" width="800"></video>

Link to Video if it does not show up: [FakeObstacleTutorial.mp4](media/FakeObstacleTutorial.mp4)

This script launches the interactive fake-obstacle demo again, with 2 RViz windows by default. The shared mapper now uses a smooth LiDAR range falloff, so very distant hits and clears contribute less confidence than nearby ones. If you want to switch to a headless smoke-test run, override `RMPD_FAKE_OBSTACLE_INJECTOR_MODE=manual`, `RMPD_QUICK_TEST_RVIZ=false`, and `RMPD_QUICK_TEST_HOLD_OPEN=false` before running it.

Once you try out all of these commands, your setup should be complete.


## Other Scripts Found in the Repository

- Run verification: `bash scripts/verify.sh`
- Run the main smoke test: `bash scripts/quick_test.sh`
- Run the fake-obstacle shared-mapping demo: `bash scripts/runTestFakeObstacle.sh`
- Run the office demo: `bash scripts/runOffice.sh`
- Run the confusing maze demo: `bash scripts/runConfusingMaze.sh`
- Run the sandbox demo: `bash scripts/runSandbox.sh`
- Run the test-building demo: `bash scripts/runTestBuildingMapForRobot.sh`
- Run the RViz combination demo: `bash scripts/runTestCombineRvizMap.sh`
- Use the older mapping-only path only if you still want it: `RMPD_TEST_MODE=mapping bash scripts/quick_test.sh`

## Repository Map

- `docs/` holds the latest project plan, file structure guide, and verification guide.
- `scripts/` holds launch scripts, verification helpers, and demo runners.
- `src/` contains the ROS 2 packages, including `robot_patrol_msgs` and `robot_patrol_node`.
- `webots/` contains the Webots worlds, controllers, and map assets.
- `docker/` contains the Docker build and compose setup.

Generated folders such as `build/`, `install/`, and `log/` are safe to delete.

## Current ROS Pieces

The main package surface currently includes:

- `src/robot_patrol_msgs/msg/MapUpdate.msg`
- `src/robot_patrol_node/launch/multi_robot_mapping.launch.py`
- `src/robot_patrol_node/launch/fake_obstacle_injector.launch.py`
- `src/robot_patrol_node/launch/amcl_stack.launch.py`
- `src/robot_patrol_node/launch/nav2_stack.launch.py`
- `src/robot_patrol_node/launch/rviz.launch.py`

The active node set includes the Webots bridge, map builder, map merge, fake obstacle injector, confidence overlay, checkpoint patrol, pose helpers, and diagnostics nodes.

## Current Worlds

The repo currently includes these Webots worlds:

- `webots/worlds/office`
- `webots/worlds/confusingMaze`
- `webots/worlds/sandbox`
- `webots/worlds/testRvizMap`
- `webots/worlds/testBuildingMapForRobot`
- `webots/worlds/TestFakeObstacle`
- `webots/worlds/TestCombineRvizMap`

The docs define the main experiment maps as:

```text
office
single_hallway
two_path
```

with `small_maze` and `random_sandbox` as later optional stress tests.

## Notes

- `build/`, `install/`, and `log/` are generated and can be deleted safely.
- `testRvizMap` uses `webots/robot_controllers/patrol_robot/patrol_robot.py`, the Nav2-capable checkpoint patrol controller.
- `simpleCorridor` and `twoRoute` use a mixed setup: `robot_1` runs `patrol_robot` and `robot_2` runs `user_controlled_robot`, and both robots accept `F` to inject a fake obstacle in front of the robot.
- `patrol_robot` has a built-in checkpoint autopilot fallback so the patrol bot still moves if the ROS route stack is late or absent.
- `simpleCorridor` and `twoRoute` now use shared looping patrol logic, so the route repeats back and forth between checkpoints instead of ending after one pass.
- Robot behavior should stay in shared controller code whenever possible so the same controller can be reused across worlds; world files should only change map-specific details like spawn locations, route geometry, robot ids, and per-world config values for RViz, Nav2, or fake-obstacle parameters.
- The saved Webots project files (`.wbproj`) are part of the world setup too; they control UI layout such as whether the console is docked or floating.
- `office`, `testBuildingMapForRobot`, `confusingMaze`, and `sandbox` use `webots/robot_controllers/user_controlled_robot/user_controlled_robot.py`.
- `runSimpleCorridor.sh` launches the simple corridor shared-mapping experiment with `robot_1` patrol and `robot_2` user-controlled.
- `runTwoRoute.sh` launches the two-route shared-mapping experiment with `robot_1` patrol and `robot_2` user-controlled.
- `runOffice.sh` starts the office world at `(-4.35, -5.35, 0.00464)` and publishes that configured AMCL initial pose instead of assuming the robot starts at the origin.
- `runConfusingMaze.sh` starts the maze world at `(-3.5, -3.5, 0.0)` and generates `webots/worlds/confusingMaze/amcl_map/confusing_maze.yaml`.
- `runSandbox.sh` starts the sandbox world at `(2.0, 2.0, 0.0)` and generates `webots/worlds/sandbox/amcl_map/sandbox.yaml`.
- `runTestFakeObstacle.sh` starts `webots/worlds/TestFakeObstacle/TestFakeObstacle.wbt`, generates a static map from that world, and launches the two-robot shared-mapping stack with the claim-based fake-obstacle injector enabled.
- The Docker bridge listens on TCP and UDP port `5005`, publishes `/robot_pose`, `/scan`, and `/odom`, and forwards `/cmd_vel` plus checkpoint feedback topics between ROS and Webots.
- The docs' longer-term goal is to extend that bridge and map flow with MATE-style trust distributions, optional trust propagation, claim verification, map-cell confidence, and quarantine logic.


