# Robot Map Poisoning Defense

This repo is a ROS 2 + Webots demo for studying robot-to-robot map poisoning and defenses.

## Start Here

- Run verification: `bash scripts/verify.sh`
- After verification passes, run the main demo: `bash scripts/quick_test.sh`
- Run the office demo: `bash scripts/runOffice.sh`
- Run the confusing maze demo: `bash scripts/runConfusingMaze.sh`
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

That launches Webots, ROS 2, RViz, and the AMCL demo together. Press `Ctrl-C` in the terminal to close everything. 

`quick_test.sh` launches the full AMCL/Nav2 demo, so RViz shows the static `/map` plus the live `/live_map` overlay. The overlay uses RViz's costmap colors, which can appear pink or purple. `runOffice.sh` uses the same RViz view with office-specific startup pose settings, so it keeps the remembered LiDAR map visible while localizing in the office world. `runTestBuildingMapForRobot.sh` uses live mapping mode, so it shows the robot-built map without the AMCL/Nav2 overlay.

## Project Files

- [docs/structure.md](docs/structure.md)
- [docs/webots.md](docs/webots.md)
- [docs/VERIFICATION.md](docs/VERIFICATION.md)
- [docs/project_plan.md](docs/project_plan.md)

## Notes

- `build/`, `install/`, and `log/` are generated and can be deleted safely.
- `testRvizMap` uses `webots/robot_controllers/patrol_robot/patrol_robot.py`, the Nav2-capable checkpoint patrol controller.
- `office`, `testBuildingMapForRobot`, and `confusingMaze` use `webots/robot_controllers/user_controlled_robot/user_controlled_robot.py`.
- `runOffice.sh` starts the office world at `(-4.35, -5.35, 0.00464)` and publishes that configured AMCL initial pose instead of assuming the robot starts at the origin.
- `runConfusingMaze.sh` starts the maze world at `(-3.5, -3.5, 0.0)` and generates `webots/worlds/confusingMaze/amcl_map/confusing_maze.yaml`.
