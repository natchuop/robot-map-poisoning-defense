# Verification

Last updated: 2026-06-16.

Run verification from the repo root:

```bash
bash scripts/verify.sh
```

Use macOS Terminal on macOS. Use WSL Ubuntu on Windows, not PowerShell.

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
- `rviz2` is installed in Docker

The verifier uses a temporary container and maps host port `15005` to container port `5005`, so it will not conflict with the normal Webots port unless `15005` is already busy.

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

## Manual Bridge Test

Start the mapping stack:

```bash
docker compose run --rm --service-ports --name ros2_dev ros2
```

Inside Docker:

```bash
bash scripts/start_ros2_stack.sh
```

From another terminal:

```bash
python3 scripts/send_test_bridge_packet.py --count 30 --delay 0.05
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 topic echo /robot_pose
```

If `/robot_pose`, `/scan`, and `/map` echo data, the Docker/ROS side is working.

## RViz Note

The verifier checks that `rviz2` is installed, but it does not launch the GUI. GUI forwarding is platform-specific:

- Windows WSL: use `docker-compose.wslg.yml`.
- macOS: use XQuartz or another X11 setup if you want RViz inside Docker.
- Without RViz, topic echo checks still prove the mapper is running.

If you are doing the manual RViz test, keep the terminal roles separate:

- `Terminal 1`: host shell that starts `docker compose ...`
- `Terminal 2`: shell inside the container that runs `bash scripts/start_ros2_stack.sh`
- `Terminal 3`: extra shell inside the container for `ros2 topic echo`
