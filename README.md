# Robot Map Poisoning Defense

This project is a ROS 2 + Webots simulation for studying robot-to-robot map poisoning attacks and defenses. The current demo runs a TurtleBot3 Burger Robot in Webots, reads GPS + IMU + LiDAR, sends pose and scan data into ROS 2 running in Docker, builds an accumulating occupancy grid, and visualizes it in RViz.

The research goal is to understand how compromised robots can spread false map information through shared updates, and how trust strategies like decay, verification, and quarantine can reduce the damage. The long-term system is intentionally simple: multi-robot mapping, shared map messages, and trust-based acceptance or rejection of updates.

Repo: `github.com/natchuop/robot-map-poisoning-defense`

## First-Time Setup

### What You Need

Windows:

- Windows 11 recommended
- WSL2 with Ubuntu 24.04
- Docker Desktop with WSL integration enabled and running before you start the stack
- Webots R2025a installed on Windows
- Git inside WSL

macOS:

- macOS
- Docker Desktop
- Webots R2025a installed on macOS
- Git
- Python 3

Notes:

- ROS 2 runs inside Docker, so you do not need to install ROS 2 on your laptop unless you want a separate native debugging setup.
- On Windows, use WSL Ubuntu for the commands below, not PowerShell.
- If Docker Desktop is not running, `docker compose` and `bash scripts/verify.sh` will fail.

### Clone The Repo

```bash
git clone git@github.com:natchuop/robot-map-poisoning-defense.git
cd robot-map-poisoning-defense
```

If you use HTTPS instead of SSH, clone with your preferred GitHub URL.

### Build The Docker Image

If you want to clear out old unused Docker build data before rebuilding, run:

```bash
docker system prune -a --volumes -f
docker builder prune -a -f
```

This removes unused containers, images, build cache, and volumes, so only use it when you are okay with Docker needing to rebuild cached layers later.

Then build the image:

```bash
docker compose build
```

If you make dependency changes later or need to recover from a broken Docker cache, rebuild with:

```bash
docker compose build --no-cache
```

## Verify The Setup

Run verification from the repo root:

```bash
bash scripts/verify.sh
```

Expected result:

```text
Results: ... passed, 0 failed
All checks passed.
```

If verification fails, the most common causes are:

- Docker is not running (you have to open it up manually)
- an old container is still holding port `5005`
- the Docker image needs a clean rebuild

## Quick ROS 2 + Webots + RViz Test

This is the smallest useful end-to-end test for the current project.

### Terminal 1

Use `Terminal 1` to start Docker and run the ROS stack.

If `Terminal 1` is already printing logs like `[udp_bridge-1] ... Received TCP packet`, leave it running and skip to the Webots step.

Windows:

```bash
docker rm -f ros2_dev
docker compose -f docker-compose.yml -f docker-compose.wslg.yml run --rm --service-ports --name ros2_dev ros2
```

macOS:

```bash
docker rm -f ros2_dev
docker compose run --rm --service-ports --name ros2_dev ros2
```

Both: When `Terminal 1` shows a prompt like `root@...:/workspace#`, run:

```bash
bash scripts/start_ros2_stack.sh
```

Leave `Terminal 1` running.

### Webots

Open this world in Webots by clicking on this file, found in this repository:

`webots/worlds/testRvizMap/turtlebot3_burger.wbt`

Press Play.

`Terminal 1` should now print logs like:

```text
[udp_bridge-1] [INFO] ... Received TCP packet #20 with keys: ['pose', 'scan']
```

### Terminal 2

Use `Terminal 2` to launch RViz.

If `Terminal 2` is currently running `ros2 topic echo`, press `Ctrl-C` first.

```bash
docker exec -it ros2_dev bash
```

When `Terminal 2` shows a prompt like `root@...:/workspace#`, run:

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --packages-select robot_patrol_node --symlink-install
source install/setup.bash
ros2 launch robot_patrol_node rviz.launch.py
```

RViz should open with these displays already loaded:

- Fixed Frame: `map`
- `Map` on `/map`
- `LaserScan` on `/scan`
- `TF`

If RViz opens but shows an empty grid, the GUI is working but no data is arriving yet.

### Terminal 3 (Can Skip)

Use `Terminal 3` only if you want to check live ROS topic data while RViz stays open.

```bash
docker exec -it ros2_dev bash
```

When `Terminal 3` shows a prompt like `root@...:/workspace#`, run:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 topic echo /robot_pose
```

It should print something like:

```text
x: -1.2346312975669118

y: 0.15221876841530446

theta: 2.1887770592374265

---

x: -1.2365101282927797

y: 0.15221876954357397

theta: 2.1870170408605967
```

Other useful checks from `Terminal 3`:

```bash
ros2 topic echo /scan
ros2 topic echo /map
ros2 topic list
ros2 node list
```

If `/robot_pose`, `/scan`, and `/map` all show data, RViz should eventually draw the map as the robot explores.

Notes:

- `Terminal 1` runs the stack and must stay open.
- `Terminal 2` runs RViz and must stay open while using RViz.
- `Terminal 3` is optional for live topic checks.
- Do not run `docker rm -f ros2_dev` while `Terminal 1` is running unless you want to stop the stack.
- If `rviz.launch.py` is missing, rebuild the package in `/workspace` and source `install/setup.bash` again.
- On macOS, GUI forwarding may require XQuartz or another X11 setup.

## Platform Notes

Windows:

- Use WSL Ubuntu for all terminal commands.
- Start Docker Desktop first, then run the project commands.
- Keep WSL integration enabled in Docker Desktop.
- Use `docker-compose.wslg.yml` if you want RViz GUI support through WSLg.

macOS:

- Start Docker Desktop first, then run the project commands.
- Docker handles the ROS 2 stack, but Webots still runs on the host machine.
- If RViz does not open cleanly from Docker, use XQuartz or verify GUI forwarding.

In both cases:

- Webots must be installed on the host machine.
- ROS 2 itself lives in Docker, not on the host.
- The core check is `bash scripts/verify.sh`.

## Start Of Day Workflow

Use this when you are returning to the project and want to get moving quickly.

1. Open Docker Desktop.

2. In `Terminal 1`, update and verify the repo:

```bash
cd ~/projects/robot-map-poisoning-defense
git pull
bash scripts/verify.sh
```

3. If Docker is taking too much space, optionally clean unused Docker data:

```bash
docker rm -f ros2_dev
docker compose down --remove-orphans
docker builder prune -f
docker image prune -f
docker container prune -f
```

If the local ROS build folders are stuck because Docker created them as root, remove them from the repo root with:

```bash
sudo rm -rf build install log
```

4. If the repo changed in a way that affects dependencies or the container image, rebuild:

```bash
docker compose build
```

5. Start the working stack in `Terminal 1`.

Windows:

```bash
docker rm -f ros2_dev
docker compose -f docker-compose.yml -f docker-compose.wslg.yml run --rm --service-ports --name ros2_dev ros2
```

macOS:

```bash
docker rm -f ros2_dev
docker compose run --rm --service-ports --name ros2_dev ros2
```

6. When `Terminal 1` shows `root@...:/workspace#`, run:

```bash
bash scripts/start_ros2_stack.sh
```

7. Open Webots, load `webots/worlds/testRvizMap/turtlebot3_burger.wbt`, and press Play.

8. Use `Terminal 2` for RViz:

```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --packages-select robot_patrol_node --symlink-install
source install/setup.bash
ros2 launch robot_patrol_node rviz.launch.py
```

## Current Project Notes

- The current controller is C and uses obstacle avoidance.
- GPS, IMU, and LiDAR are already working in Webots.
- The main mapping direction is occupancy grids, not object classification.
- The project should eventually move toward Python controllers when deeper ROS 2 integration is needed.
- The research focus is trust management for poisoned shared maps, not SLAM-first mapping.

## Repo Layout

```text
src/                         ROS 2 packages
scripts/                     helper and verification scripts
webots/worlds/               Webots worlds, one folder per world
webots/controllers/          Webots Python controllers, one folder per controller
docs/                        design and setup notes
docker-compose.yml           portable Docker setup
docker-compose.wslg.yml      optional Windows WSLg GUI overlay
```

Generated local folders are ignored by Git:

- `build/`
- `install/`
- `log/`
- `logs/`
- `__pycache__/`

## Troubleshooting

If `ros2_dev` already exists:

```bash
docker rm -f ros2_dev
```

If Webots cannot connect to the ROS bridge:

- make sure `bash scripts/start_ros2_stack.sh` is running in Docker
- make sure Docker was started with `--service-ports`
- restart Webots so it reloads the controller

If the mapping stack looks stale after dependency changes:

```bash
docker compose build --no-cache
bash scripts/verify.sh
```

## Helpful Reference Files

- `[docs/project_plan.md](/home/natch/projects/robot-map-poisoning-defense/docs/project_plan.md)`
- `[docs/VERIFICATION.md](/home/natch/projects/robot-map-poisoning-defense/docs/VERIFICATION.md)`
- `[webots/set_up_webots_world.md](/home/natch/projects/robot-map-poisoning-defense/webots/set_up_webots_world.md)`
