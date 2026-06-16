# Robot Map Poisoning Defense

A ROS 2 cooperative multi-robot cybersecurity simulation. Robots share map data; a compromised robot injects false updates. Defense nodes detect and contain poisoning using trust scores and multi-robot verification. An LLM acts as a security analyst - classifying attacks and recommending quarantine.

**Repo:** [github.com/natchuop/robot-map-poisoning-defense](https://github.com/natchuop/robot-map-poisoning-defense)

---

## Project Overview

**Attacks being simulated:** fake obstacles, false blocked/cleared routes, robot-to-robot misinformation, and future: MITM, unauthorized publishers, DoS.

**Defenses:** per-robot trust scores, per-object confidence scores, multi-robot verification, quarantine.

**LLM role (analyst, not controller):**
```
Robot_2 repeatedly reports obstacles that no other robot confirms.
-> classify map poisoning -> recommend quarantine -> generate explanation
```

---

## Current Status

Infrastructure is fully verified. A fresh clone builds and runs cleanly inside Docker.

| Package | Purpose |
|---------|---------|
| `attack_node` | Inject false map/route updates |
| `defense_node` | Trust scoring, verification, quarantine |
| `llm_security_agent` | Analyze events, recommend actions |
| `map_sharing_msgs` | Custom message definitions |
| `robot_patrol_node` | Normal patrol and map sharing |

- All packages build successfully
- No custom messages or executables implemented yet - added incrementally
- Docker image includes ROS 2, RViz, and `webots_ros2`
- Webots app installed locally; ROS bridge not wired up yet

## Recommended setup

Do **not** install the full ROS 2 package list on WSL unless you specifically want a separate native ROS install for debugging.

This repo is designed to run ROS 2 inside Docker, so on WSL you only need:

- Git
- Docker Desktop with WSL integration
- Webots on the host machine

For this project, **skip SLAM for now**. Start with:

- GPS + IMU + LiDAR
- waypoint patrol
- map sharing
- poisoning and trust logic

Add SLAM later only if you want autonomous map building without relying on GPS/IMU. For the current research goal, SLAM is optional and adds complexity you do not need on day one.

---

## Quick Start

Full install steps (Git, Docker, Webots, SSH) are in **[SETUP.md](SETUP.md)**.

**1. Clone**
```bash
git clone git@github.com:natchuop/robot-map-poisoning-defense.git
cd robot-map-poisoning-defense
```

**2. Build image and enter container**
```bash
docker compose build
docker compose run --rm ros2
```
Windows: enable WSL integration in Docker Desktop first.

Only use `docker compose build --no-cache` when recovering from a broken build or after aggressive Docker cleanup. For normal work, plain `docker compose build` reuses cached layers and is much faster.

**3. Build workspace** (inside container - ROS 2 is auto-sourced)
```bash
colcon build && source install/setup.bash
```

**4. Verify pub/sub** (see [SETUP.md](SETUP.md) §7 for full steps)

From the repo root - `cd` into the project first:
```bash
cd ~/projects/robot-map-poisoning-defense
docker compose run --name ros2_dev ros2
# inside container:
ros2 run demo_nodes_cpp talker
```

Second terminal:
```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

**5. Run the verify script**
```bash
bash scripts/verify.sh
```
Expected: `Results: 10 passed, 0 failed`

## If Docker gets bloated

To clear old Docker images, cache, and unused volumes before a clean rebuild:

```bash
docker system prune -a --volumes -f
docker builder prune -a -f
```

Then rebuild:

```bash
docker compose build
```

---

## Repo Structure

```
src/        ROS 2 packages
worlds/     Webots simulation worlds (future)
launch/     Launch files (future)
config/     Node parameters (future)
docs/       Design notes (future)
scripts/    Helper scripts
```

`build/`, `install/`, `log/`, `logs/` are generated locally and not tracked in Git.

---

## Daily Workflow

Each time you work on the project:

1. Open **Ubuntu** (Windows) or **Terminal** (Mac) - not PowerShell
2. Start **Docker Desktop**
3. `cd ~/projects/robot-map-poisoning-defense`
4. `git pull` (if teammates may have pushed)
5. `docker compose run --rm ros2`
6. `colcon build && source install/setup.bash` (if code changed)
7. Work inside the container; type `exit` when done

Full step-by-step with troubleshooting: **[SETUP.md § Daily workflow](SETUP.md#daily-workflow)**

---

## Next Steps

1. Define messages in `map_sharing_msgs`
2. Implement nodes: `robot_patrol_node` -> `attack_node` -> `defense_node` -> `llm_security_agent`
3. Add Webots world and launch files

**Help:** [SETUP.md](SETUP.md) · [ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/)
