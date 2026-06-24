# Robot Map Poisoning Defense

This repo is a ROS 2 + Webots demo for studying robot-to-robot map poisoning and defenses.

## Start Here

- Run verification: `bash scripts/verify.sh`
- After verification passes, run the main demo: `bash scripts/quick_test.sh`
- Run the office demo: `bash scripts/runOffice.sh`
- Run the confusing maze demo: `bash scripts/runConfusingMaze.sh`
- Run the sandbox demo: `bash scripts/runSandbox.sh`
- Run the test-building demo: `bash scripts/runTestBuildingMapForRobot.sh`
- Use the optional mapping path only if you still want the older map-building demo: `RMPD_TEST_MODE=mapping bash scripts/quick_test.sh`

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

ROS 2 runs in Docker, not on the host.
The verification and quick-test scripts also expect `python3` on the host or inside WSL on Windows.

## First Time Setup

Before cloning:

- Install Docker Desktop.
- Install Webots R2025a.
- Install Git.
- On Windows, also install WSL2 with Ubuntu 24.04.

Then clone and prepare the repo:

```bash
git clone git@github.com:natchuop/robot-map-poisoning-defense.git
cd robot-map-poisoning-defense
docker system prune -a --volumes -f
docker builder prune -a -f
docker compose -f docker/compose.yml build --no-cache
bash scripts/verify.sh
```

If you prefer HTTPS, clone with your GitHub HTTPS URL instead.

## After Setup

Once verification passes, run the quick test:

```bash
bash scripts/quick_test.sh
```

That launches Webots, ROS 2, RViz, and the AMCL/Nav2 checkpoint patrol demo together. Press `Ctrl-C` in the terminal to close everything.

`quick_test.sh` now launches the default AMCL + Nav2 smoke test, so RViz shows the static `/map` plus the live `/live_map` overlay. The overlay uses RViz's costmap colors, which can appear pink or purple. `runOffice.sh` uses the office-specific RViz view and startup pose settings so the larger office map and remembered LiDAR overlay stay visible. `runConfusingMaze.sh` and `runSandbox.sh` both reuse the same AMCL pipeline with world-specific maps and initial poses. `runTestBuildingMapForRobot.sh` follows the same AMCL + live `/live_map` pattern by default; set `RMPD_TEST_MODE=mapping` if you want the older live-mapping-only path.

## Project Files

- [docs/structure.md](docs/structure.md)
- [docs/webots.md](docs/webots.md)
- [docs/VERIFICATION.md](docs/VERIFICATION.md)
- [docs/project_plan.md](docs/project_plan.md)
- [scripts/](scripts)

## Notes

- `build/`, `install/`, and `log/` are generated and can be deleted safely.
- `testRvizMap` uses `webots/robot_controllers/patrol_robot/patrol_robot.py`, the Nav2-capable checkpoint patrol controller.
- `office`, `testBuildingMapForRobot`, `confusingMaze`, and `sandbox` use `webots/robot_controllers/user_controlled_robot/user_controlled_robot.py`.
- `runOffice.sh` starts the office world at `(-4.35, -5.35, 0.00464)` and publishes that configured AMCL initial pose instead of assuming the robot starts at the origin.
- `runConfusingMaze.sh` starts the maze world at `(-3.5, -3.5, 0.0)` and generates `webots/worlds/confusingMaze/amcl_map/confusing_maze.yaml`.
- `runSandbox.sh` starts the sandbox world at `(2.0, 2.0, 0.0)` and generates `webots/worlds/sandbox/amcl_map/sandbox.yaml`.
- The Docker bridge listens on TCP and UDP port `5005`, publishes `/robot_pose`, `/scan`, and `/odom`, and forwards `/cmd_vel` plus checkpoint feedback topics between ROS and Webots.
