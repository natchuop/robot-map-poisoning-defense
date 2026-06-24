# Verification

Last updated: 2026-06-24.

Run verification from the repo root:

```bash
bash scripts/verify.sh
```

Use macOS Terminal on macOS. Use WSL Ubuntu on Windows, not PowerShell.

## Where Things Run

On the verified Windows setup:

- WSL Ubuntu is the terminal environment where you run project commands.
- Docker Desktop builds and runs the Linux ROS 2 container.
- ROS 2 runs inside Docker, not directly in WSL.
- RViz also runs inside Docker.
- RViz is displayed on the Windows desktop through WSLg, WSL's built-in Linux GUI support.
- Webots runs as the normal Windows Webots app, but the scripts can launch it from WSL.

On macOS, commands run in macOS Terminal, ROS 2 runs in Docker, and Webots runs as the macOS Webots app. RViz inside Docker usually needs XQuartz; otherwise `quick_test.sh` can skip RViz GUI startup while still checking Docker, ROS 2, and Webots bridge data.

## What It Checks

`scripts/verify.sh` validates:

- Docker daemon is running
- Docker Compose config is valid
- host `python3` is available
- Docker image builds
- Python source files compile
- ROS 2 workspace builds with `colcon`
- temporary mapping stack starts
- mapping bridge listens on TCP/UDP inside Docker
- mapping ROS graph contains:
  - `/robot_pose`
  - `/scan`
  - `/map`
- mapping bridge wiring includes:
  - `/odom`
  - `/cmd_vel`
  - `/active_checkpoint`
  - `/webots_checkpoint_event`
  - `/webots_checkpoint_contact`
- fake Webots packets can travel from host TCP into Docker
- `/robot_pose` receives bridge data
- `/scan` receives bridge data
- `/odom` receives bridge data
- `/map` publishes an occupancy grid
- temporary AMCL/Nav2 stack starts
- AMCL bridge listens on TCP/UDP inside Docker
- AMCL stack receives:
  - `/robot_pose`
  - `/scan`
  - `/odom`
- AMCL publishes `/amcl_pose` and `/live_map`
- AMCL/Nav2 topic wiring includes:
  - `/cmd_vel`
  - `/active_checkpoint`
  - `/webots_checkpoint_event`
  - `/webots_checkpoint_contact`
- `checkpoint_patrol_node`, `initial_pose_publisher`, `udp_bridge`, and `amcl` expose the expected ROS connections
- Nav2 controller, planner, map server, and waypoint follower packages are installed in Docker
- AMCL is installed in Docker via `nav2_amcl`
- AMCL's `amcl` executable is discoverable in Docker
- `rviz2` is installed in Docker

The verifier uses a temporary container and maps host port `15005` to container port `5005`, so it will not conflict with the normal Webots port unless `15005` is already busy.

`scripts/verify.sh` is a headless setup check. It proves Docker, ROS 2, the bridge paths, mapping topics, AMCL/Nav2 topic wiring, Nav2 packages, AMCL packages, and RViz installation. It does not open Webots or RViz.

For the full GUI smoke test, run:

```bash
bash scripts/quick_test.sh
```

The quick test starts the AMCL localization stack, opens Webots, waits for bridge packets and TF, and launches RViz with `amcl.rviz`. In the current setup, `udp_bridge` publishes `/odom` directly for AMCL and the initial-pose helper, so `verify.sh` checks that path instead of a separate `pose_to_odom` node.

In that default RViz config, the static AMCL map is `/map` and the robot-built LiDAR map is `/live_map`. `/live_map` uses costmap-style colors, so pink or purple cells are expected. `bash scripts/runOffice.sh` uses the same RViz view with the office-world startup pose and keeps `/live_map` enabled so previously explored areas remain visible in RViz. `bash scripts/runTestBuildingMapForRobot.sh` now uses the same AMCL default by default; set `RMPD_TEST_MODE=mapping` if you want the older live-mapping-only flow where the robot-built map is published directly as `/map`.

To override the verification host port:

```bash
RMPD_VERIFY_PORT=16005 bash scripts/verify.sh
```

## Expected Result

Healthy setup:

```text
Results: ... passed, 0 failed
All checks passed.
```

## If Verification Fails

Docker is not running:

```bash
docker info
```

Start Docker Desktop, then rerun:

```bash
bash scripts/verify.sh
```

Port is busy:

```bash
RMPD_VERIFY_PORT=16005 bash scripts/verify.sh
```

Build cache is stale:

```bash
docker compose -f docker/compose.yml build --no-cache
bash scripts/verify.sh
```

Old containers are holding port `5005`:

```bash
docker ps
docker rm -f ros2_dev
```

## Bridge Test

The preferred end-to-end command is:

```bash
bash scripts/quick_test.sh
```

If you only want to test the Docker bridge path without opening Webots or RViz, use fake packets:

```bash
python3 scripts/send_test_bridge_packet.py --count 30 --delay 0.05
```

The normal quick test container is named `ros2_dev`, so you can inspect topics from another terminal while it runs:

```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
source "${RMPD_CONTAINER_WORKSPACE:-/workspace}/install/setup.bash"
ros2 topic echo /robot_pose
```

If `/robot_pose`, `/scan`, `/map`, `/odom`, `/amcl_pose`, and `/live_map` echo data, the Docker/ROS side is working.

The AMCL quick test uses this world-local known map:

```text
webots/worlds/testRvizMap/amcl_map/arena.yaml
webots/worlds/testRvizMap/amcl_map/arena.pgm
```

The map is generated from known `testRvizMap` geometry by `scripts/quick_test.sh`; it is not automatically parsed from the `.wbt` file.

## RViz Note

The verifier checks that `rviz2` is installed, but it does not launch the GUI. The GUI quick test is `bash scripts/quick_test.sh`. GUI forwarding is platform-specific:

- Windows WSL: use `docker/compose.wslg.yml`.
- macOS: use XQuartz or another X11 setup if you want RViz inside Docker.
- Without RViz, topic echo and TF checks still prove the ROS side is running.

Useful RViz/TF checks while `quick_test.sh` is running:

```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
source "${RMPD_CONTAINER_WORKSPACE:-/workspace}/install/setup.bash"
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once
ros2 run tf2_ros tf2_echo map base_link
```
