# Verification

Last updated: 2026-06-26.

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
- fake-obstacle shared mapping brings up `/robot_1/shared_live_map` and `/robot_2/shared_live_map`
- fake-obstacle shared mapping activates the static `/map` server so RViz can draw the gray floor behind the shared overlays
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

## Fake Obstacle Demo

To verify the fake-obstacle path, run:

```bash
bash scripts/runTestFakeObstacle.sh
```

That demo launches the two-robot shared-mapping stack, two RViz windows, and the per-robot fake-obstacle injectors. The default setup is:

- `robot_1` compromised
- `robot_2` compromised
- `robot_1` RViz publish-point tool sends to `/robot_1/clicked_point`
- `robot_2` RViz publish-point tool sends to `/robot_2/clicked_point`
- `robot_1` displays `/robot_1/shared_live_map`, `/robot_1/shared_confidence_map`, and `/robot_1/confidence_markers`
- `robot_2` displays `/robot_2/shared_live_map`, `/robot_2/shared_confidence_map`, `/robot_2/confidence_markers`, and `/robot_2/fake_obstacle_markers`
- the confidence heatmap is trust-weighted so each robot can inject and receive fake data while still keeping its own trust bias
- the shared-mapping launch activates the static `/map` server, which is what makes the gray floor appear in RViz

`bash scripts/runTestFakeObstacle.sh` now enables force-clean by default, so it will remove a stale `ros2_dev` demo before starting a new one. If you want to keep an existing stack alive, set `RMPD_QUICK_TEST_FORCE_CLEAN=false`.

Useful checks while the demo is running:

```bash
ros2 topic echo /map_updates --once
ros2 topic info /map_updates -v
ros2 topic info /robot_1/shared_live_map -v
ros2 topic info /robot_2/shared_live_map -v
```

Expected behavior:

- a click in the compromised robot's RViz produces one or more `/map_updates` messages
- the compromised robot ignores its own fake report
- the victim robot accepts the report only when `target_robot` matches its `view_robot_id`
- accepted fake reports stay visible until real LiDAR evidence clears them

## Common Failure Modes

These are the regressions that have shown up most often:

- RViz or Webots looks broken right after a code change because the workspace has not rebuilt or an old container is still holding ports `5005` and `5006`.
- The confidence overlay nodes fail on startup because of ROS parameter typing or missing helper methods.
- The shared map appears to lose a robot's own cleared areas when the merge logic reweights or replays fake reports incorrectly.
- Fake obstacle injections stay stuck on the map because the merge path keeps repainting them after the cell has been verified clear.

If one of those happens, check the recent launch logs first, then rebuild and rerun:

```bash
bash scripts/quick_test.sh
```

For the combined two-robot demo, `bash scripts/runTestCombineRvizMap.sh` is the quickest way to confirm that the Webots, RViz, merge, and fake-obstacle paths still agree.

If you rerun the demo without force-clean, stop the previous `runTestFakeObstacle.sh` session first. The bridge ports are fixed at `5005` and `5006`, so a still-running stack will block the next launch until you terminate the old run or remove its container/process tree.

`scripts/quick_test.sh` now checks those bridge ports before starting Docker and will print a direct warning if either one is already occupied. With force-clean enabled, it removes the stale container first and waits for the bridge ports to clear before launching again.

In `multi_mapping` mode, the quick test now launches Webots and RViz before it performs the merge-node confirmation, so the GUI cannot be blocked by a slow node graph check.

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
If you are checking the fake-obstacle demo, also confirm that `/robot_1/shared_live_map`, `/robot_2/shared_live_map`, `/robot_1/shared_confidence_map`, `/robot_2/shared_confidence_map`, `/robot_1/confidence_markers`, and `/robot_2/confidence_markers` are publishing. The older occupied/clear confidence topic names are left in the launch params for compatibility, but the current overlays use the shared confidence map plus the marker overlay.

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
