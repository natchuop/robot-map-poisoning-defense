# Docker Verification

Last verified: 2026-06-09. All checks passed (confirmed against actual terminal output).

---

## Verified results

`bash scripts/verify.sh` → **Results: 7 passed, 0 failed**

| Check | Result | Actual output |
|-------|--------|---------------|
| Docker running | ✅ | `Docker is running` |
| Image exists | ✅ | `robot-map-poisoning-defense-ros2` present |
| `colcon build` | ✅ | `Summary: 5 packages finished [1.59s]` |
| ROS 2 pub/sub | ✅ | `Publishing: 'Hello World: 1/2/3'` |
| Project packages | ✅ | All 5 listed |
| `webots_ros2` packages | ✅ | 13 packages listed |
| `which rviz2` | ✅ | `/opt/ros/jazzy/bin/rviz2` |

---

## How to re-verify

### Quickest way — run the script

From the repo root in **WSL Ubuntu**:
```bash
bash scripts/verify.sh
```

Runs all checks automatically and prints a pass/fail summary.

### Manual commands

Run all commands from inside **WSL Ubuntu** — not PowerShell. See "Shell gotcha" below.

### 1. Build the image
```bash
cd ~/projects/robot-map-poisoning-defense
docker compose build
```
Expected last line: `Image robot-map-poisoning-defense-ros2 Built`

### 2. colcon build
```bash
docker run --rm \
  -v ~/projects/robot-map-poisoning-defense:/workspace \
  -w /workspace \
  robot-map-poisoning-defense-ros2 \
  bash -c "source /opt/ros/jazzy/setup.bash && colcon build 2>&1 | tail -5"
```
Expected: `Summary: 5 packages finished`

### 3. ROS 2 pub/sub
```bash
docker run --rm robot-map-poisoning-defense-ros2 bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   ros2 run demo_nodes_cpp talker & sleep 3 && \
   ros2 run demo_nodes_py listener & sleep 4 && \
   kill %1 %2 2>/dev/null; true"
```
Expected: `[talker]: Publishing: 'Hello World: 1'` etc.

Note: listener output may not appear (it starts as a background process and can exit before flushing). Talker output alone confirms the ROS 2 DDS layer is working.

### 4. Package and binary checks
```bash
docker run --rm \
  -v ~/projects/robot-map-poisoning-defense:/workspace \
  -w /workspace \
  robot-map-poisoning-defense-ros2 \
  bash -c "
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash 2>/dev/null
    echo '=== project ===' && ros2 pkg list | grep -E 'attack_node|defense_node|llm_security_agent|map_sharing_msgs|robot_patrol_node'
    echo '=== webots_ros2 ===' && ros2 pkg list | grep webots_ros2
    echo '=== rviz2 ===' && which rviz2
  "
```
Expected output:
```
=== project ===
attack_node
defense_node
llm_security_agent
map_sharing_msgs
robot_patrol_node
=== webots_ros2 ===
webots_ros2
webots_ros2_control
webots_ros2_crazyflie
webots_ros2_driver
webots_ros2_epuck
webots_ros2_husarion
webots_ros2_importer
webots_ros2_mavic
webots_ros2_msgs
webots_ros2_tesla
webots_ros2_tiago
webots_ros2_turtlebot
webots_ros2_universal_robot
=== rviz2 ===
/opt/ros/jazzy/bin/rviz2
```

---

## Shell gotcha — important

**`docker compose run` blocks when called from PowerShell via WSL passthrough.**

The Cursor IDE terminal runs PowerShell, which calls WSL to run commands. When `docker compose run` is invoked this way, it hangs indefinitely with no output or exit code. This caused repeated failures during setup.

**What works reliably for scripted checks:**
- `docker run --rm <image-name> bash -c "..."` — one-shot container, exits cleanly
- Running commands directly from inside a WSL terminal (not via PowerShell passthrough)

**`docker compose run` is still the right command for interactive dev sessions.** The hang only occurs with automated/scripted calls from PowerShell. For daily use (`docker compose run --rm ros2` typed in a WSL terminal) it works fine.

**Image name:** `robot-map-poisoning-defense-ros2`  
Docker Compose derives this from the project folder name (`robot-map-poisoning-defense`) + service name (`ros2`).

---

## Notes

- `rviz2 --help` fails in a headless container (Qt display error). Use `which rviz2` instead to confirm the binary is installed.
- `colcon build` must be run with `-v ~/projects/..:/workspace` so it writes `build/` and `install/` to your local repo, not inside the container.
- ROS 2 is auto-sourced in interactive shells (via `/etc/bash.bashrc`). In one-shot `bash -c "..."` calls, you must `source /opt/ros/jazzy/setup.bash` manually.
- Stray files (`*.log`, `*.exit`, `*.out`) are blocked by `.gitignore` but may accumulate locally. Delete them with `rm ~/projects/robot-map-poisoning-defense/*.log` etc.
