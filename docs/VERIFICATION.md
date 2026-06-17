# Verification

Last updated: 2026-06-17.

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
- host `python3` is available
- Docker image builds
- Python source files compile
- ROS 2 workspace builds with `colcon`
- temporary mapping stack starts
- bridge listens on TCP/UDP inside Docker
- ROS graph contains:
  - `/robot_pose`
  - `/scan`
  - `/map`
- fake Webots packets can travel from host TCP into Docker
- `/robot_pose` receives bridge data
- `/scan` receives bridge data
- `/map` publishes an occupancy grid
- Nav2 controller, planner, map server, and waypoint follower packages are installed in Docker
- AMCL is installed in Docker via `nav2_amcl`
- AMCL's `amcl` executable is discoverable in Docker
- `rviz2` is installed in Docker

The verifier uses a temporary container and maps host port `15005` to container port `5005`, so it will not conflict with the normal Webots port unless `15005` is already busy.

`scripts/verify.sh` is a headless setup check. It proves Docker, ROS 2, the bridge path, mapping topics, Nav2 packages, AMCL packages, and RViz installation. It does not open Webots or RViz.

For the full GUI smoke test, run:

```bash
bash scripts/quick_test.sh
```

The quick test starts the AMCL localization stack, opens Webots, waits for bridge packets and TF, and launches RViz with `amcl.rviz`.

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
docker compose build --no-cache
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

If `/robot_pose`, `/scan`, `/map`, and `/odom` echo data, the Docker/ROS side is working.

The AMCL quick test uses this world-local known map:

```text
webots/worlds/testRvizMap/amcl_map/arena.yaml
webots/worlds/testRvizMap/amcl_map/arena.pgm
```

The map is generated from known `testRvizMap` geometry by `scripts/quick_test.sh`; it is not automatically parsed from the `.wbt` file.

## RViz Note

The verifier checks that `rviz2` is installed, but it does not launch the GUI. The GUI quick test is `bash scripts/quick_test.sh`. GUI forwarding is platform-specific:

- Windows WSL: use `docker-compose.wslg.yml`.
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
