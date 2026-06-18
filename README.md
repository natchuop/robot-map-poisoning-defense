# Robot Map Poisoning Defense

This project is a ROS 2 + Webots simulation for studying robot-to-robot map poisoning attacks and defenses. The current demo runs a TurtleBot3 Burger Robot in Webots, reads GPS + IMU + LiDAR, sends pose and scan data into ROS 2 running in Docker, localizes against a known AMCL map, and visualizes the result in RViz.

The planned navigation stack is **known map + AMCL + Nav2**. The robots will navigate between predefined checkpoints on a known Webots map. AMCL handles localization on that known map, Nav2 handles path planning and motion to checkpoints, and RViz2 is used for visualization and debugging. SLAM is intentionally not part of the main implementation path because the project is focused on poisoned map updates and trust strategies, not online map construction.

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

### Docker Portability Rules

Use these rules when changing Docker files or scripts that run inside Docker:

- Prefer official multi-architecture base images, like `ros`, `ubuntu`, `python`, or `node`.
- Keep local development builds separate from published multi-architecture builds: `docker compose build` for your machine, `docker buildx build --platform linux/amd64,linux/arm64 --push` for shared images.
- Do not commit machine-specific host paths. Use repo-relative mounts like `.:/workspace`, Docker volumes, or variables in `.env`.
- Do not hardcode published host ports. This repo defaults the Webots bridge to `127.0.0.1:5005`, but you can override it with `RMPD_BRIDGE_BIND` and `RMPD_BRIDGE_PORT`.
- Avoid `container_name` in compose files unless there is a strong reason. Generated names prevent conflicts when multiple clones or users run the project.
- Do not copy binaries, virtual environments, build folders, or host-generated artifacts into the image. Install dependencies inside the Docker build so they match the target architecture.
- Keep architecture-specific dependencies behind clear build args or install logic. If a package is only available on `amd64`, document it and fail clearly on other platforms.
- Use Linux container paths inside Docker, and discover the repo root in scripts with paths relative to the script location.
- Use `.dockerignore` aggressively so local `build/`, `install/`, `log/`, caches, and Git metadata do not affect image builds.
- Avoid assuming GUI support is available. Keep WSLg/XQuartz/X11 mounts in optional compose overlays.
- Keep secrets, tokens, and personal registry credentials out of Dockerfiles, compose files, and `.env.example`.

Optional local overrides can go in a private `.env` file:

```bash
cp .env.example .env
```

For example, if port `5005` is busy:

```env
RMPD_BRIDGE_PORT=15005
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

Run the whole test with one command from the repo root:

```bash
bash scripts/quick_test.sh
```

By default, this now runs an AMCL + Nav2 checkpoint patrol smoke test:

- launches the ROS 2 Docker stack
- generates or refreshes the known map for the `testRvizMap` Webots arena
- mirrors the Webots pose into a temporary `odom` frame for testing
- starts `map_server`, `amcl`, Nav2, and the checkpoint patrol node
- publishes an initial pose automatically
- opens RViz when Docker can see a GUI display, or verifies `rviz2` is installed and skips GUI startup when no display is available
- launches `webots/worlds/testRvizMap/turtlebot3_burger.wbt`

The AMCL map lives beside the Webots world:

```text
webots/worlds/testRvizMap/amcl_map/arena.yaml
webots/worlds/testRvizMap/amcl_map/arena.pgm
```

That map is generated from the known geometry in `webots/worlds/testRvizMap/turtlebot3_burger.wbt`: the circular arena size, box positions, map origin, and resolution. It is not parsed automatically from the `.wbt` file yet, so if the Webots world changes, update the generator in `scripts/quick_test.sh` or replace the AMCL map files to match.

This smoke test now brings up Nav2 checkpoint patrol as well. The odom source is still a test harness that mirrors the Webots ground-truth pose, and the AMCL launch publishes test-harness `map -> odom`, `odom -> base_link`, and `base_link -> laser` transforms so RViz has a stable `map` fixed frame.

If you want the older live mapping demo instead, run `RMPD_TEST_MODE=mapping bash scripts/quick_test.sh`.

If Webots is not on your `PATH`, set `WEBOTS_CMD` to its executable path before running the script.

When RViz launches, it should open with `amcl.rviz`, fixed frame `map`, the generated arena map, `/scan`, AMCL particles, and TF. On machines without Docker GUI forwarding, including many macOS setups, the smoke test skips RViz GUI startup by default so Docker/Webots bridge verification can still pass. To require RViz GUI startup, run `RMPD_QUICK_TEST_RVIZ=true bash scripts/quick_test.sh`. To keep Webots and RViz open for manual inspection, run `RMPD_QUICK_TEST_HOLD_OPEN=true bash scripts/quick_test.sh`.

## Platform Notes

Windows:

- Use WSL Ubuntu for all terminal commands.
- Start Docker Desktop first, then run the project commands.
- Keep WSL integration enabled in Docker Desktop.
- Use `docker-compose.wslg.yml` if you want RViz GUI support through WSLg.
- Current verified Windows setup:
  - You type commands in WSL Ubuntu.
  - Docker Desktop runs the Linux container.
  - ROS 2 and RViz run inside that Docker container.
  - RViz appears on your Windows desktop through WSLg, which is WSL's built-in Linux GUI support.
  - Webots runs as the normal Windows Webots app, but `scripts/quick_test.sh` launches it from WSL.
- If RViz needs direct rendering and WSL exposes `/dev/dri`, add the optional DRI overlay:

```bash
docker compose -f docker-compose.yml -f docker-compose.wslg.yml -f docker-compose.dri.yml run --rm --service-ports ros2
```

macOS:

- Start Docker Desktop first, then run the project commands.
- Docker handles the ROS 2 stack, but Webots still runs on the host machine.
- On Apple Silicon, including M1/M2/M3, Docker builds the ARM64 image locally. The ROS Jazzy packages used by this project are available for `linux/arm64`.
- If RViz does not open cleanly from Docker, let `quick_test.sh` skip Docker GUI startup, run RViz from a native ROS install, or configure XQuartz.
- To try RViz in Docker through XQuartz, install and start XQuartz, enable network clients in XQuartz settings, restart XQuartz, then run:

```bash
xhost +localhost
RMPD_QUICK_TEST_RVIZ=true DISPLAY=host.docker.internal:0 bash scripts/quick_test.sh
```

In both cases:

- Webots must be installed on the host machine.
- ROS 2 itself lives in Docker, not on the host.
- The core check is `bash scripts/verify.sh`.

## Start Of Day Workflow

Use this when you are returning to the project and want to get moving quickly.

1. Open Docker Desktop.
2. Update and verify the repo:

```bash
cd ~/projects/robot-map-poisoning-defense
git pull
bash scripts/verify.sh
```

1. If Docker is taking too much space, optionally clean unused Docker data:

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

1. If the repo changed in a way that affects dependencies or the container image, rebuild:

```bash
docker compose build
```

1. Start the AMCL quick test:

```bash
bash scripts/quick_test.sh
```

The script starts Docker, launches Webots, waits for bridge data and TF, checks RViz availability or GUI startup depending on display support, and exits with a pass/fail status.

## Current Project Notes

- The current controller is Python and uses obstacle avoidance.
- GPS, IMU, and LiDAR are already working in Webots.
- The current quick test uses a known AMCL map in `webots/worlds/testRvizMap/amcl_map/`.
- The main mapping direction is occupancy grids, not object classification.
- The main navigation direction is known map + AMCL + Nav2, not SLAM.
- The project should eventually move toward Python controllers when deeper ROS 2 integration is needed.
- The research focus is trust management for poisoned shared maps, not SLAM-first mapping.

## Repo Layout

```text
src/                         ROS 2 packages
scripts/                     helper and verification scripts
webots/worlds/               Webots worlds, one folder per world
webots/worlds/testRvizMap/amcl_map/
                              Generated known map used by AMCL for this world
webots/controllers/          Webots Python controllers, one folder per shared controller
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

- run `bash scripts/quick_test.sh` from the repo root so Docker starts with the expected ports
- restart Webots so it reloads the controller

If RViz says `Frame [map] does not exist`:

- close old RViz/Webots windows and rerun `bash scripts/quick_test.sh`
- give RViz a few seconds after Webots starts; the map and TF tree can settle shortly after the window opens
- check that RViz is using `amcl.rviz`, not `default.rviz`

If RViz fails on Windows with Qt, WSLg, or OpenGL errors:

- restart WSL and Docker Desktop:

```powershell
wsl --shutdown
```

- recreate the container with the WSLg overlay:

```bash
docker compose -f docker-compose.yml -f docker-compose.wslg.yml run --rm --service-ports ros2
```

- inside the container, confirm the WSLg runtime and GL diagnostics:

```bash
echo "$XDG_RUNTIME_DIR"
ls -la "$XDG_RUNTIME_DIR"
glxinfo -B
```

- if Wayland fails, try XCB:

```bash
RMPD_QT_QPA_PLATFORM=xcb docker compose -f docker-compose.yml -f docker-compose.wslg.yml run --rm --service-ports ros2
```

The Docker image includes common Qt/X11/Wayland and Mesa diagnostic packages for RViz. If GUI issues continue, running RViz directly in WSL Ubuntu is usually more reliable than running RViz inside Docker.

If RViz reports message-filter drops with `timestamp earlier than all the data in the transform cache`, check that scans and TF are being published with the same clock:

```bash
ros2 topic echo -n1 /scan header.stamp
ros2 topic echo -n1 /tf
ros2 param list | grep use_sim_time
```

The project launch files use wall time by default. Avoid mixing `use_sim_time=true` nodes with wall-time nodes unless `/clock` is also being published.

If the mapping stack looks stale after dependency changes:

```bash
docker compose build --no-cache
bash scripts/verify.sh
```

## Helpful Reference Files

- [docs/project_plan.md](docs/project_plan.md)
- [docs/VERIFICATION.md](docs/VERIFICATION.md)
- [webots/set_up_webots_world.md](webots/set_up_webots_world.md)

