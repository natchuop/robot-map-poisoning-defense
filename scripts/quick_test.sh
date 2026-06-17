#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${RMPD_CONTAINER_NAME:-ros2_dev}"
WORLD_PATH="${RMPD_WEBOTS_WORLD:-$REPO_DIR/webots/worlds/testRvizMap/turtlebot3_burger.wbt}"
WEBOTS_CMD="${WEBOTS_CMD:-}"
TEST_MODE="${RMPD_TEST_MODE:-amcl}"
AMCL_MAP_DIR="$REPO_DIR/.generated/amcl_test_map"
AMCL_MAP_YAML="$AMCL_MAP_DIR/arena.yaml"
AMCL_MAP_YAML_IN_CONTAINER="/workspace/.generated/amcl_test_map/arena.yaml"

COMPOSE_FILES=("-f" "$REPO_DIR/docker-compose.yml")
if [[ -n "${WSL_DISTRO_NAME:-}" && -f "$REPO_DIR/docker-compose.wslg.yml" ]]; then
  COMPOSE_FILES+=("-f" "$REPO_DIR/docker-compose.wslg.yml")
fi

docker_compose() {
  docker compose "${COMPOSE_FILES[@]}" "$@"
}

log_step() {
  printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$1"
}

cleanup() {
  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
  if [ -n "${WEBOTS_PID:-}" ] && kill -0 "$WEBOTS_PID" >/dev/null 2>&1; then
    kill "$WEBOTS_PID" >/dev/null 2>&1 || true
  fi
}

wait_for_log() {
  local pattern="$1"
  local timeout_seconds="${2:-120}"
  local elapsed=0

  while [ "$elapsed" -lt "$timeout_seconds" ]; do
    if ! docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
      echo "Container $CONTAINER_NAME stopped before it was ready." >&2
      docker logs "$CONTAINER_NAME" >&2 || true
      return 1
    fi

    if docker logs "$CONTAINER_NAME" 2>&1 | grep -Fq "$pattern"; then
      return 0
    fi

    if [ $((elapsed % 10)) -eq 0 ]; then
      echo "  still waiting for: $pattern (${elapsed}s/${timeout_seconds}s)"
      docker logs "$CONTAINER_NAME" 2>&1 | tail -8 || true
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  echo "Timed out waiting for '$pattern' in $CONTAINER_NAME logs." >&2
  docker logs "$CONTAINER_NAME" 2>&1 | tail -80 >&2 || true
  return 1
}

generate_amcl_map() {
  if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 is required to generate the AMCL test map." >&2
    exit 1
  fi

  mkdir -p "$AMCL_MAP_DIR"
  python3 - "$AMCL_MAP_DIR" <<'PY'
from pathlib import Path
import math
import sys

outdir = Path(sys.argv[1])
width = 160
height = 160
resolution = 0.05
origin_x = -4.0
origin_y = -4.0
center_x = width // 2
center_y = height // 2
radius = 60
wall_thickness = 3

grid = [[254 for _ in range(width)] for _ in range(height)]

def mark_cell(x, y, value=0):
    if 0 <= x < width and 0 <= y < height:
        grid[y][x] = value

def mark_box(world_x, world_y, size=0.3):
    cell_x = int((world_x - origin_x) / resolution)
    cell_y = int((world_y - origin_y) / resolution)
    half = max(1, int((size / resolution) / 2.0))
    for y in range(cell_y - half, cell_y + half + 1):
        for x in range(cell_x - half, cell_x + half + 1):
            mark_cell(x, y, 0)

for y in range(height):
    for x in range(width):
        distance = math.hypot(x - center_x, y - center_y)
        if abs(distance - radius) <= wall_thickness:
            mark_cell(x, y, 0)

for world_x, world_y in [
    (0.467367, -0.545426),
    (1.26618, 1.07342),
    (-0.15697, 0.782967),
    (-1.62271, 1.08968),
    (-1.09887, -0.301011),
    (-2.71307, -0.22263),
    (-0.744103, 2.69736),
    (2.30103, 0.203241),
    (0.005323, -1.57959),
    (-1.0974, -1.8598),
]:
    mark_box(world_x, world_y)

pgm_path = outdir / 'arena.pgm'
with pgm_path.open('w', encoding='ascii') as handle:
    handle.write('P2\n')
    handle.write(f'{width} {height}\n')
    handle.write('255\n')
    for row in reversed(grid):
        handle.write(' '.join(str(value) for value in row))
        handle.write('\n')

yaml_path = outdir / 'arena.yaml'
yaml_path.write_text(
    '\n'.join(
        [
            f'image: {Path("/workspace/.generated/amcl_test_map/arena.pgm").as_posix()}',
            f'resolution: {resolution}',
            f'origin: [{origin_x}, {origin_y}, 0.0]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.2',
        ]
    )
    + '\n',
    encoding='ascii',
)
PY
}

find_webots_cmd() {
  if [ -n "$WEBOTS_CMD" ]; then
    printf '%s\n' "$WEBOTS_CMD"
    return 0
  fi

  local candidate
  local install_dir

  install_dir="$(
    powershell.exe -NoProfile -Command "
      \$keys = @(
        'HKCU:\\Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\*',
        'HKLM:\\Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\*',
        'HKLM:\\Software\\WOW6432Node\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\*'
      )
      Get-ItemProperty \$keys -ErrorAction SilentlyContinue |
        Where-Object { \$_.DisplayName -match 'Webots' } |
        Select-Object -First 1 -ExpandProperty InstallLocation
    " 2>/dev/null | tr -d '\r'
  )"

  if [ -n "$install_dir" ]; then
    install_dir="${install_dir%\\}"
    install_dir="${install_dir%/}"
    if command -v wslpath >/dev/null 2>&1; then
      install_dir="$(wslpath -u "$install_dir" 2>/dev/null || printf '%s' "$install_dir")"
    fi
    for candidate in \
      "$install_dir/webots.exe" \
      "$install_dir/webots" \
      "$install_dir/webots-bin.exe" \
      "$install_dir/msys64/mingw64/bin/webots.exe" \
      "$install_dir/msys64/mingw64/bin/webots-bin.exe"
    do
      if [ -n "$candidate" ] && [ -x "$candidate" ]; then
        printf '%s\n' "$candidate"
        return 0
      fi
    done
  fi

  for candidate in \
    "$(command -v webots 2>/dev/null || true)" \
    "$(command -v webots.exe 2>/dev/null || true)" \
    "/usr/local/webots/webots" \
    "/usr/local/webots/webots.sh" \
    "/opt/webots/webots" \
    "/opt/webots/webots.sh" \
    "/Applications/Webots.app/Contents/MacOS/webots" \
    "/Applications/Webots.app/Contents/MacOS/Webots" \
    "/mnt/c/Program Files/Webots/webots.exe" \
    "/mnt/c/Program Files/Webots/webots" \
    "/mnt/c/Users/natch/AppData/Local/Programs/Webots/webots.exe" \
    "/mnt/c/Users/natch/AppData/Local/Programs/Webots/webots-bin.exe" \
    "/mnt/c/Program Files/Webots/msys64/mingw64/bin/webots.exe" \
    "/mnt/c/Program Files (x86)/Webots/webots.exe"
  do
    if [ -n "$candidate" ] && [ -x "$candidate" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

trap cleanup EXIT INT TERM

log_step "Starting the ROS 2 + Webots + RViz quick test"
echo "Repo: $REPO_DIR"
echo "World: $WORLD_PATH"
echo "Container: $CONTAINER_NAME"
echo "Mode: $TEST_MODE"

if [ ! -f "$WORLD_PATH" ]; then
  echo "World file not found: $WORLD_PATH" >&2
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is not available on PATH." >&2
  exit 1
fi

if [ "$TEST_MODE" = "amcl" ]; then
  log_step "Generating the AMCL test map"
  generate_amcl_map
  if [ ! -f "$AMCL_MAP_YAML" ]; then
    echo "Failed to generate AMCL map at $AMCL_MAP_YAML" >&2
    exit 1
  fi
  echo "AMCL map YAML: $AMCL_MAP_YAML"
fi

WEBOTS_BIN="$(find_webots_cmd || true)"
if [ -z "$WEBOTS_BIN" ]; then
  echo "Webots was not found." >&2
  echo "Set WEBOTS_CMD to the Webots executable path, then rerun this script." >&2
  exit 1
fi

WEBOTS_WORLD_ARG="$WORLD_PATH"
case "$WEBOTS_BIN" in
  *.exe|*/webots-bin.exe|*/webots.exe)
    if command -v wslpath >/dev/null 2>&1; then
      WEBOTS_WORLD_ARG="$(wslpath -w "$WORLD_PATH" 2>/dev/null || printf '%s' "$WORLD_PATH")"
    fi
    ;;
esac

docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

log_step "Starting Docker stack"
if [ "$TEST_MODE" = "amcl" ]; then
  docker_compose run -d \
    --service-ports \
    --name "$CONTAINER_NAME" \
    -e RMPD_TEST_MODE="$TEST_MODE" \
    -e RMPD_AMCL_MAP_YAML="$AMCL_MAP_YAML_IN_CONTAINER" \
    ros2 bash -lc 'bash scripts/start_ros2_stack.sh' >/dev/null
else
  docker_compose run -d --service-ports --name "$CONTAINER_NAME" ros2 bash -lc 'bash scripts/start_ros2_stack.sh' >/dev/null
fi

log_step "Waiting for ROS 2 build/setup"
echo "Step 1/5: waiting for the workspace to finish building"
wait_for_log 'ROS 2 workspace ready' 240

log_step "Waiting for the ROS bridge and localization stack to come up"
echo "Step 2/5: waiting for the bridge to start"
wait_for_log 'Listening for Webots packets' 180
if [ "$TEST_MODE" = "amcl" ]; then
  echo "Step 3/5: waiting for pose-to-odom"
  wait_for_log 'Pose-to-odom ready' 120
  echo "Step 4/5: waiting for initial pose publisher"
  wait_for_log 'Initial pose publisher ready' 120
  echo "Step 5/5: waiting for the first initial pose"
  wait_for_log 'Published initial pose #1' 120
  echo "Waiting for /map to become available before RViz settles"
  docker exec "$CONTAINER_NAME" bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && timeout 30 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once >/tmp/rmpd_map_ready.log 2>&1' || {
    echo "Warning: /map was not observed within 30 seconds. RViz may show an empty map initially." >&2
    docker exec "$CONTAINER_NAME" bash -lc 'cat /tmp/rmpd_map_ready.log 2>/dev/null || true' >&2 || true
  }
  sleep 5
else
  echo "Step 2/2: waiting for the map builder"
  wait_for_log 'Map builder ready' 180
fi

log_step "Launching Webots"
echo "Webots executable: $WEBOTS_BIN"
echo "Webots world arg: $WEBOTS_WORLD_ARG"
"$WEBOTS_BIN" "$WEBOTS_WORLD_ARG" &
WEBOTS_PID=$!
echo "Webots PID: $WEBOTS_PID"

log_step "Waiting for Webots data"
echo "Waiting for the bridge to receive at least one Webots packet"
wait_for_log 'Received TCP packet' 120

if [ "$TEST_MODE" = "amcl" ]; then
  echo "Waiting for AMCL/TF to create the map frame"
  docker exec "$CONTAINER_NAME" bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /workspace/install/setup.bash
    for _ in $(seq 1 30); do
      timeout 2 ros2 run tf2_ros tf2_echo map base_link >/tmp/rmpd_tf_ready.log 2>&1 || true
      if grep -q "At time" /tmp/rmpd_tf_ready.log; then
        exit 0
      fi
      sleep 1
    done
    cat /tmp/rmpd_tf_ready.log 2>/dev/null || true
    exit 1
  ' || {
    echo "Warning: map -> base_link TF was not observed within 30 seconds. RViz may need a few more seconds after opening." >&2
  }
fi

log_step "Launching RViz in Docker"
RVIZ_CONFIG_ARG=''
if [ "$TEST_MODE" = "amcl" ]; then
  RVIZ_CONFIG_PATH='/workspace/install/robot_patrol_node/share/robot_patrol_node/config/amcl.rviz'
  if ! docker exec "$CONTAINER_NAME" test -f "$RVIZ_CONFIG_PATH"; then
    echo "Warning: AMCL RViz config was not found at $RVIZ_CONFIG_PATH; falling back to default.rviz" >&2
  else
    echo "Using AMCL RViz config: $RVIZ_CONFIG_PATH"
    RVIZ_CONFIG_ARG="rviz_config:=$RVIZ_CONFIG_PATH"
  fi
fi
if docker exec -d "$CONTAINER_NAME" bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 launch robot_patrol_node rviz.launch.py $RVIZ_CONFIG_ARG"; then
  echo "RViz launch command submitted to $CONTAINER_NAME"
else
  echo "Failed to submit RViz launch command." >&2
fi

log_step "Quick test is running"
echo "If you want to stop everything, close Webots or press Ctrl-C here."
wait "$WEBOTS_PID"
