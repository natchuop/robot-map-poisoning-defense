#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

env_file_value() {
  local key="$1"
  local default_value="$2"
  local env_file="$REPO_DIR/.env"
  local value

  if [ -f "$env_file" ]; then
    value="$(
      awk -F= -v key="$key" '
        $0 !~ /^[[:space:]]*(#|$)/ && $1 == key {
          sub(/^[^=]*=/, "")
          gsub(/^[[:space:]]+|[[:space:]]+$/, "")
          gsub(/^["'\''"]|["'\''"]$/, "")
          print
          exit
        }
      ' "$env_file"
    )"
    if [ -n "$value" ]; then
      printf '%s\n' "$value"
      return 0
    fi
  fi

  printf '%s\n' "$default_value"
}

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required to run quick_test.sh." >&2
  exit 1
fi

DEFAULT_CONTAINER_WORKSPACE="$(env_file_value RMPD_CONTAINER_WORKSPACE /workspace)"
DEFAULT_BRIDGE_PORT="$(env_file_value RMPD_BRIDGE_PORT 5005)"
DEFAULT_HOLD_OPEN="$(env_file_value RMPD_QUICK_TEST_HOLD_OPEN true)"
DEFAULT_RVIZ_MODE="$(env_file_value RMPD_QUICK_TEST_RVIZ auto)"

CONTAINER_NAME="${RMPD_CONTAINER_NAME:-ros2_dev}"
CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-$DEFAULT_CONTAINER_WORKSPACE}"
export RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE"
WEBOTS_CMD="${WEBOTS_CMD:-}"
TEST_MODE="${RMPD_TEST_MODE:-amcl}"
HOLD_OPEN="${RMPD_QUICK_TEST_HOLD_OPEN:-$DEFAULT_HOLD_OPEN}"
RVIZ_CHECK_SECONDS="${RMPD_RVIZ_CHECK_SECONDS:-10}"
RVIZ_MODE="${RMPD_QUICK_TEST_RVIZ:-$DEFAULT_RVIZ_MODE}"
HOST_BRIDGE_PORT="${RMPD_BRIDGE_PORT:-$DEFAULT_BRIDGE_PORT}"
HOST_BRIDGE_TARGETS="${WEBOTS_BRIDGE_TARGETS:-}"
RVIZ_HOST_LOG="$REPO_DIR/.generated/rmpd_rviz.log"
export RMPD_INSTALL_FULL_STACK=true

find_default_world() {
  local worlds_dir="$REPO_DIR/webots/worlds"
  local world_path

  world_path="$(find "$worlds_dir" -type f -name 'turtlebot3_burger.wbt' -print -quit 2>/dev/null || true)"
  if [ -z "$world_path" ]; then
    world_path="$(find "$worlds_dir" -type f -name '*.wbt' -print -quit 2>/dev/null || true)"
  fi

  printf '%s\n' "$world_path"
}

abs_path() {
  python3 - "$1" <<'PY'
from pathlib import Path
import sys

print(Path(sys.argv[1]).expanduser().resolve(strict=False))
PY
}

repo_relative_path() {
  python3 - "$REPO_DIR" "$1" <<'PY'
import os
import sys

print(os.path.relpath(sys.argv[2], sys.argv[1]))
PY
}

container_path_for_host_path() {
  local relative_path
  relative_path="$(repo_relative_path "$1")"
  printf '%s/%s\n' "${CONTAINER_WORKSPACE%/}" "$relative_path"
}

WORLD_PATH="${RMPD_WEBOTS_WORLD:-$(find_default_world)}"
if [ -n "$WORLD_PATH" ]; then
  WORLD_PATH="$(abs_path "$WORLD_PATH")"
fi
AMCL_MAP_DIR="${RMPD_AMCL_MAP_DIR:-$(dirname "$WORLD_PATH")/amcl_map}"
AMCL_MAP_DIR="$(abs_path "$AMCL_MAP_DIR")"
AMCL_MAP_YAML="$AMCL_MAP_DIR/arena.yaml"
AMCL_MAP_YAML_IN_CONTAINER="$(container_path_for_host_path "$AMCL_MAP_YAML")"

COMPOSE_FILES=("-f" "$REPO_DIR/docker-compose.yml")
if [[ -n "${WSL_DISTRO_NAME:-}" && -f "$REPO_DIR/docker-compose.wslg.yml" ]]; then
  COMPOSE_FILES+=("-f" "$REPO_DIR/docker-compose.wslg.yml")
fi
if [[ "$(uname -s)" = "Darwin" && -f "$REPO_DIR/docker-compose.xquartz.yml" ]]; then
  if [ -n "${DISPLAY:-}" ] || [[ "$RVIZ_MODE" =~ ^(1|true|yes|on)$ ]]; then
    COMPOSE_FILES+=("-f" "$REPO_DIR/docker-compose.xquartz.yml")
  fi
fi

docker_compose() {
  docker compose "${COMPOSE_FILES[@]}" "$@"
}

should_launch_rviz() {
  case "$RVIZ_MODE" in
    1|true|yes|on)
      return 0
      ;;
    0|false|no|off|skip)
      return 1
      ;;
    auto)
      if [ -n "${WSL_DISTRO_NAME:-}" ]; then
        return 0
      fi
      if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
        return 0
      fi
      return 1
      ;;
    *)
      echo "Invalid RMPD_QUICK_TEST_RVIZ value: $RVIZ_MODE" >&2
      echo "Use auto, true, or false." >&2
      exit 1
      ;;
  esac
}

log_step() {
  printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$1"
}

cleanup() {
  if [ -n "${RVIZ_EXEC_PID:-}" ] && kill -0 "$RVIZ_EXEC_PID" >/dev/null 2>&1; then
    kill "$RVIZ_EXEC_PID" >/dev/null 2>&1 || true
  fi

  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

  if [ -n "${WEBOTS_PID:-}" ] && kill -0 "$WEBOTS_PID" >/dev/null 2>&1; then
    kill "$WEBOTS_PID" >/dev/null 2>&1 || true
  fi
}

wait_for_log() {
  local pattern="$1"
  local timeout_seconds="${2:-120}"
  local label="${3:-$pattern}"
  local elapsed=0

  echo "Waiting: $label"
  while [ "$elapsed" -lt "$timeout_seconds" ]; do
    if ! docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
      echo "Container $CONTAINER_NAME stopped before it was ready." >&2
      docker logs "$CONTAINER_NAME" >&2 || true
      return 1
    fi

    if docker logs "$CONTAINER_NAME" 2>&1 | grep -Fq "$pattern"; then
      echo "Ready: $label"
      return 0
    fi

    if [ "$elapsed" -gt 0 ] && [ $((elapsed % 15)) -eq 0 ]; then
      echo "  still waiting (${elapsed}s/${timeout_seconds}s): $label"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  echo "Timed out waiting for '$label'." >&2
  echo "Last container logs:" >&2
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
            'image: arena.pgm',
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
  local windows_home

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

  if command -v cmd.exe >/dev/null 2>&1 && command -v wslpath >/dev/null 2>&1; then
    windows_home="$(cmd.exe /C echo %USERPROFILE% 2>/dev/null | tr -d '\r')"
    windows_home="$(wslpath -u "$windows_home" 2>/dev/null || true)"
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
    "/Applications/Webots.app/Contents/MacOS/webots-bin" \
    "/Applications/Webots.app/Contents/MacOS/Webots-bin" \
    "/mnt/c/Program Files/Webots/webots.exe" \
    "/mnt/c/Program Files/Webots/webots" \
    "$windows_home/AppData/Local/Programs/Webots/webots.exe" \
    "$windows_home/AppData/Local/Programs/Webots/webots-bin.exe" \
    "$windows_home/AppData/Local/Programs/Webots/msys64/mingw64/bin/webots.exe" \
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
echo "Container workspace: $CONTAINER_WORKSPACE"
echo "Mode: $TEST_MODE"
echo "Hold open: $HOLD_OPEN"
echo "RViz mode: $RVIZ_MODE"
echo "Host bridge port: $HOST_BRIDGE_PORT"

if [ -z "$WORLD_PATH" ] || [ ! -f "$WORLD_PATH" ]; then
  echo "World file not found: $WORLD_PATH" >&2
  echo "Set RMPD_WEBOTS_WORLD to the Webots world file you want to use." >&2
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
    -e RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE" \
    -e RMPD_TEST_MODE="$TEST_MODE" \
    -e RMPD_AMCL_MAP_YAML="$AMCL_MAP_YAML_IN_CONTAINER" \
    ros2 bash -lc 'bash scripts/start_ros2_stack.sh' >/dev/null
else
  docker_compose run -d \
    --service-ports \
    --name "$CONTAINER_NAME" \
    -e RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE" \
    ros2 bash -lc 'bash scripts/start_ros2_stack.sh' >/dev/null
fi

log_step "Waiting for ROS 2 build/setup"
wait_for_log 'ROS 2 workspace ready' 240 'ROS 2 workspace build'

log_step "Waiting for the ROS bridge and localization stack to come up"
wait_for_log 'Listening for Webots packets' 180 'ROS bridge listener'
if [ "$TEST_MODE" = "amcl" ]; then
  wait_for_log 'Pose-to-odom ready' 120 'pose-to-odom node'
  wait_for_log 'Initial pose publisher ready' 120 'initial pose publisher'
  wait_for_log 'map @ 0.050 m/pix' 45 '/map loaded by map_server'
  wait_for_log 'Managed nodes are active' 45 'AMCL lifecycle active'
else
  wait_for_log 'Map builder ready' 180 'map builder'
fi

log_step "Launching Webots"
echo "Webots executable: $WEBOTS_BIN"
echo "Webots world arg: $WEBOTS_WORLD_ARG"

if [ -z "$HOST_BRIDGE_TARGETS" ]; then
  if [ "$(uname -s)" = "Darwin" ]; then
    HOST_BRIDGE_TARGETS="127.0.0.1"
  else
    HOST_BRIDGE_TARGETS="172.28.64.1,127.0.0.1"
  fi
fi

WEBOTS_BRIDGE_PORT="$HOST_BRIDGE_PORT" WEBOTS_BRIDGE_TARGETS="$HOST_BRIDGE_TARGETS" "$WEBOTS_BIN" "$WEBOTS_WORLD_ARG" &
WEBOTS_PID=$!

echo "Webots PID: $WEBOTS_PID"
echo "Webots bridge targets: $HOST_BRIDGE_TARGETS"
echo "Webots bridge port: $HOST_BRIDGE_PORT"

log_step "Waiting for Webots data"
wait_for_log 'Received TCP packet' 120 'first Webots packet'

if should_launch_rviz; then
  if [ "$TEST_MODE" = "amcl" ]; then
    echo "RViz will open now; the map and TF tree may take a few seconds to settle."
  fi

  log_step "Launching RViz in Docker"
  RVIZ_CONFIG_ARG=''

  if [ "$TEST_MODE" = "amcl" ]; then
    RVIZ_CONFIG_PATH="$(
      docker exec "$CONTAINER_NAME" bash -lc '
        source /opt/ros/jazzy/setup.bash
        source "${RMPD_CONTAINER_WORKSPACE:-/workspace}/install/setup.bash"
        pkg_prefix="$(ros2 pkg prefix robot_patrol_node)"
        printf "%s/share/robot_patrol_node/config/amcl.rviz\n" "$pkg_prefix"
      ' | tr -d '\r'
    )"

    if ! docker exec "$CONTAINER_NAME" test -f "$RVIZ_CONFIG_PATH"; then
      echo "Warning: AMCL RViz config was not found at $RVIZ_CONFIG_PATH; falling back to default.rviz" >&2
    else
      echo "Using AMCL RViz config: $RVIZ_CONFIG_PATH"
      RVIZ_CONFIG_ARG="rviz_config:=$RVIZ_CONFIG_PATH"
    fi
  fi

  mkdir -p "$(dirname "$RVIZ_HOST_LOG")"
  rm -f "$RVIZ_HOST_LOG"

  docker exec "$CONTAINER_NAME" bash -lc "source /opt/ros/jazzy/setup.bash && source \"\${RMPD_CONTAINER_WORKSPACE:-/workspace}/install/setup.bash\" && export LIBGL_ALWAYS_SOFTWARE=1 && export QT_X11_NO_MITSHM=1 && export QT_QPA_PLATFORM=xcb && exec ros2 launch robot_patrol_node rviz.launch.py $RVIZ_CONFIG_ARG" >"$RVIZ_HOST_LOG" 2>&1 &
  RVIZ_EXEC_PID=$!

  echo "RViz launch command submitted to $CONTAINER_NAME"
  echo "RViz host log: $RVIZ_HOST_LOG"

  log_step "Checking RViz startup"
  sleep "$RVIZ_CHECK_SECONDS"

  if ! kill -0 "$RVIZ_EXEC_PID" >/dev/null 2>&1; then
    echo "RViz appears to have exited during startup. RViz log:" >&2
    tail -120 "$RVIZ_HOST_LOG" >&2 || true
    exit 1
  fi

  if grep -Eqi 'rviz2-[0-9]+.*process has died|GLSL link result|Invalid parentWindowHandle|Unable to create the rendering window|Qt.*could not connect to display|Could not load the Qt platform plugin|Aborted|segmentation fault|core dumped' "$RVIZ_HOST_LOG" 2>/dev/null; then
    echo "RViz appears to have failed during startup. RViz log:" >&2
    tail -120 "$RVIZ_HOST_LOG" >&2 || true
    exit 1
  fi
else
  log_step "Skipping RViz GUI startup"
  echo "No Docker-visible display was detected. Set RMPD_QUICK_TEST_RVIZ=true after configuring XQuartz/WSLg to require RViz GUI startup."
  docker exec "$CONTAINER_NAME" bash -lc 'source /opt/ros/jazzy/setup.bash && command -v rviz2 >/dev/null'
fi

log_step "Quick test checks passed"
echo "Docker stack started, Webots bridge data arrived, and RViz availability/startup checks passed."

log_step "Quick test is running"
echo "If you want to stop everything, close Webots or press Ctrl-C here."
echo "RViz log will remain available at: $RVIZ_HOST_LOG"

wait "$WEBOTS_PID"