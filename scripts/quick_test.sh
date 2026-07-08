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
DEFAULT_RVIZ_CONFIG="$(env_file_value RMPD_QUICK_TEST_RVIZ_CONFIG '')"
DEFAULT_RVIZ_CONFIG_FILES="$(env_file_value RMPD_RVIZ_CONFIG_FILES '')"
DEFAULT_RVIZ_WINDOW_COUNT="$(env_file_value RMPD_RVIZ_WINDOW_COUNT 1)"
DEFAULT_STREAM_ROS_LOGS="$(env_file_value RMPD_STREAM_ROS_LOGS true)"
DEFAULT_FORCE_CLEAN="$(env_file_value RMPD_QUICK_TEST_FORCE_CLEAN false)"

CONTAINER_NAME="${RMPD_CONTAINER_NAME:-ros2_dev}"
CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-$DEFAULT_CONTAINER_WORKSPACE}"
CONTAINER_INSTALL_BASE="${RMPD_COLCON_INSTALL_BASE:-/tmp/rmpd_colcon_install}"
export RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE"
WEBOTS_CMD="${WEBOTS_CMD:-}"
TEST_MODE="${RMPD_TEST_MODE:-amcl}"
HOLD_OPEN="${RMPD_QUICK_TEST_HOLD_OPEN:-$DEFAULT_HOLD_OPEN}"
RVIZ_CHECK_SECONDS="${RMPD_RVIZ_CHECK_SECONDS:-10}"
RVIZ_MODE="${RMPD_QUICK_TEST_RVIZ:-$DEFAULT_RVIZ_MODE}"
RVIZ_CONFIG_FILE="${RMPD_QUICK_TEST_RVIZ_CONFIG:-$DEFAULT_RVIZ_CONFIG}"
RVIZ_CONFIG_FILES="${RMPD_RVIZ_CONFIG_FILES:-$DEFAULT_RVIZ_CONFIG_FILES}"
RVIZ_WINDOW_COUNT="${RMPD_RVIZ_WINDOW_COUNT:-$DEFAULT_RVIZ_WINDOW_COUNT}"
STREAM_ROS_LOGS="${RMPD_STREAM_ROS_LOGS:-$DEFAULT_STREAM_ROS_LOGS}"
FORCE_CLEAN="${RMPD_QUICK_TEST_FORCE_CLEAN:-$DEFAULT_FORCE_CLEAN}"
HOST_BRIDGE_PORT="${RMPD_BRIDGE_PORT:-$DEFAULT_BRIDGE_PORT}"
HOST_BRIDGE_TARGETS="${WEBOTS_BRIDGE_TARGETS:-}"
RVIZ_HOST_LOG_DIR="${TMPDIR:-/tmp}/rmpd/quick_test"
export RMPD_INSTALL_FULL_STACK=true
ROS_LOG_FOLLOW_PID=""
RVIZ_EXEC_PIDS=()
RVIZ_HOST_LOGS=()
RVIZ_CONFIG_PATHS=()

if [ "$TEST_MODE" = "multi_mapping" ] && [ -z "${RMPD_QUICK_TEST_RVIZ_CONFIG:-}" ] && [ -z "${RMPD_RVIZ_CONFIG_FILES:-}" ]; then
  RVIZ_CONFIG_FILES="multi_robot_robot_1_view.rviz,multi_robot_robot_2_view.rviz"
  if [ -z "${RMPD_RVIZ_WINDOW_COUNT:-}" ]; then
    RVIZ_WINDOW_COUNT=2
  fi
fi

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
WORLD_BASENAME="$(basename "$WORLD_PATH")"
if [ -z "$RVIZ_CONFIG_FILE" ]; then
  case "$WORLD_BASENAME" in
    office.wbt)
      RVIZ_CONFIG_FILE="office_amcl.rviz"
      ;;
    *)
      RVIZ_CONFIG_FILE="amcl.rviz"
      ;;
  esac
fi
world_amcl_defaults() {
  case "$WORLD_BASENAME" in
    office.wbt)
      printf '%s %s %s %s\n' 'office' '-4.35' '-5.35' '0.004641574238792719'
      ;;
    confusing_maze.wbt)
      printf '%s %s %s %s\n' 'confusing_maze' '-3.5' '-3.5' '0.0'
      ;;
    sandbox.wbt)
      printf '%s %s %s %s\n' 'sandbox' '2.0' '2.0' '0.0'
      ;;
    *)
      printf '%s %s %s %s\n' 'arena' '0.0' '0.0' '0.0'
      ;;
  esac
}

read -r DEFAULT_AMCL_MAP_BASENAME DEFAULT_AMCL_INITIAL_POSE_X DEFAULT_AMCL_INITIAL_POSE_Y DEFAULT_AMCL_INITIAL_POSE_YAW < <(world_amcl_defaults)
AMCL_MAP_BASENAME="${RMPD_AMCL_MAP_NAME:-$DEFAULT_AMCL_MAP_BASENAME}"
AMCL_MAP_DIR="${RMPD_AMCL_MAP_DIR:-$(dirname "$WORLD_PATH")/amcl_map}"
AMCL_MAP_DIR="$(abs_path "$AMCL_MAP_DIR")"
AMCL_MAP_YAML="$AMCL_MAP_DIR/$AMCL_MAP_BASENAME.yaml"
AMCL_MAP_YAML_IN_CONTAINER="$(container_path_for_host_path "$AMCL_MAP_YAML")"
AMCL_INITIAL_POSE_X="${RMPD_AMCL_INITIAL_POSE_X:-$DEFAULT_AMCL_INITIAL_POSE_X}"
AMCL_INITIAL_POSE_Y="${RMPD_AMCL_INITIAL_POSE_Y:-$DEFAULT_AMCL_INITIAL_POSE_Y}"
AMCL_INITIAL_POSE_YAW="${RMPD_AMCL_INITIAL_POSE_YAW:-$DEFAULT_AMCL_INITIAL_POSE_YAW}"

COMPOSE_FILES=("-f" "$REPO_DIR/docker/compose.yml")
if [[ -n "${WSL_DISTRO_NAME:-}" && -f "$REPO_DIR/docker/compose.wslg.yml" ]]; then
  COMPOSE_FILES+=("-f" "$REPO_DIR/docker/compose.wslg.yml")
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

  for rviz_pid in "${RVIZ_EXEC_PIDS[@]:-}"; do
    if [ -n "$rviz_pid" ] && kill -0 "$rviz_pid" >/dev/null 2>&1; then
      kill "$rviz_pid" >/dev/null 2>&1 || true
    fi
  done

  if [ -n "${ROS_LOG_FOLLOW_PID:-}" ] && kill -0 "$ROS_LOG_FOLLOW_PID" >/dev/null 2>&1; then
    kill "$ROS_LOG_FOLLOW_PID" >/dev/null 2>&1 || true
  fi

  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

  if [ -n "${WEBOTS_PID:-}" ] && kill -0 "$WEBOTS_PID" >/dev/null 2>&1; then
    kill "$WEBOTS_PID" >/dev/null 2>&1 || true
  fi
}

stream_ros_logs() {
  if ! [[ "$STREAM_ROS_LOGS" =~ ^(1|true|yes|on)$ ]]; then
    return 0
  fi

  echo "Streaming ROS container logs below."
  docker logs -f --tail 200 "$CONTAINER_NAME" &
  ROS_LOG_FOLLOW_PID=$!
}

wait_for_log() {
  local pattern="$1"
  local timeout_seconds="${2:-120}"
  local label="${3:-$pattern}"
  local elapsed=0
  local logs

  echo "Waiting: $label"
  while [ "$elapsed" -lt "$timeout_seconds" ]; do
    if ! docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
      echo "Container $CONTAINER_NAME stopped before it was ready." >&2
      docker logs "$CONTAINER_NAME" >&2 || true
      return 1
    fi

    logs="$(docker logs "$CONTAINER_NAME" 2>&1 || true)"
    if grep -Fq "$pattern" <<<"$logs"; then
      echo "Ready: $label"
      return 0
    fi

    if [ "$elapsed" -gt 0 ] && [ $((elapsed % 10)) -eq 0 ]; then
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

wait_for_file_log() {
  local log_file="$1"
  local pattern="$2"
  local timeout_seconds="${3:-120}"
  local label="${4:-$pattern}"
  local elapsed=0

  echo "Waiting: $label"
  while [ "$elapsed" -lt "$timeout_seconds" ]; do
    if [ -f "$log_file" ] && grep -Fq "$pattern" "$log_file" 2>/dev/null; then
      echo "Ready: $label"
      return 0
    fi

    if [ "$elapsed" -gt 0 ] && [ $((elapsed % 10)) -eq 0 ]; then
      echo "  still waiting (${elapsed}s/${timeout_seconds}s): $label"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  echo "Timed out waiting for '$label'." >&2
  if [ -f "$log_file" ]; then
    echo "Last log lines:" >&2
    tail -80 "$log_file" >&2 || true
  else
    echo "Log file did not appear: $log_file" >&2
  fi
  return 1
}

wait_for_ros2_nodes() {
  local node_patterns="$1"
  local timeout_seconds="${2:-120}"
  local label="${3:-$node_patterns}"
  local elapsed=0
  local nodes_output

  echo "Waiting: $label"
  while [ "$elapsed" -lt "$timeout_seconds" ]; do
    if ! docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
      echo "Container $CONTAINER_NAME stopped before it was ready." >&2
      docker logs "$CONTAINER_NAME" >&2 || true
      return 1
    fi

    nodes_output="$(
      docker exec "$CONTAINER_NAME" bash -lc '
        source /opt/ros/jazzy/setup.bash
        source "${RMPD_CONTAINER_WORKSPACE:-/workspace}/install/setup.bash"
        ros2 node list
      ' 2>/dev/null || true
    )"

    local all_found=true
    for node_pattern in $node_patterns; do
      if ! grep -Fq "$node_pattern" <<<"$nodes_output"; then
        all_found=false
        break
      fi
    done

    if [ "$all_found" = true ]; then
      echo "Ready: $label"
      return 0
    fi

    if [ "$elapsed" -gt 0 ] && [ $((elapsed % 10)) -eq 0 ]; then
      echo "  still waiting (${elapsed}s/${timeout_seconds}s): $label"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  echo "Timed out waiting for '$label'." >&2
  if [ -n "$nodes_output" ]; then
    echo "Last visible ros2 node list:" >&2
    printf '%s\n' "$nodes_output" >&2
  fi
  return 1
}

check_bridge_ports_available() {
  local ports=("5005" "5006")
  local in_use=()
  local port
  local listen_output

  for port in "${ports[@]}"; do
    listen_output="$(ss -H -ltnup "sport = :$port" 2>/dev/null || true)"
    if [ -n "$listen_output" ]; then
      in_use+=("$port")
      echo "Bridge port $port is already in use:" >&2
      printf '%s\n' "$listen_output" >&2
    fi
  done

  if [ "${#in_use[@]}" -eq 0 ]; then
    return 0
  fi

  echo "Stop the existing fake-obstacle/demo session, then rerun." >&2
  echo "If the old stack is in this WSL instance, use:" >&2
  echo "  ps -ef | egrep 'ros2 launch robot_patrol_node multi_robot_mapping|docker-proxy' | egrep -v 'egrep|grep'" >&2
  echo "  kill <ros2-pid>" >&2
  return 1
}

cleanup_existing_demo() {
  echo "Force clean requested; removing any stale demo container and waiting for bridge ports."
  local stale_container_ids=""
  stale_container_ids="$(
    {
      docker ps -aq --filter "publish=5005"
      docker ps -aq --filter "publish=5006"
      docker ps -aq --filter "name=${CONTAINER_NAME}"
    } | awk 'NF { print }' | sort -u
  )"

  if [ -n "$stale_container_ids" ]; then
    echo "Removing stale demo containers:"
    printf '%s\n' "$stale_container_ids"
    while IFS= read -r container_id; do
      [ -n "$container_id" ] || continue
      docker rm -f "$container_id" >/dev/null 2>&1 || true
    done <<< "$stale_container_ids"
  else
    docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
  fi

  local elapsed=0
  local timeout_seconds=30
  local listen_output=""
  while [ "$elapsed" -lt "$timeout_seconds" ]; do
    listen_output="$(
      {
        ss -H -ltnup 'sport = :5005' 2>/dev/null || true
        ss -H -ltnup 'sport = :5006' 2>/dev/null || true
      }
    )"
    if [ -z "$listen_output" ]; then
      echo "Stale bridge listeners cleared."
      return 0
    fi

    if [ "$elapsed" -eq 0 ]; then
      echo "Waiting for stale bridge listeners to exit:"
      printf '%s\n' "$listen_output"
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  echo "Bridge ports are still busy after force clean:" >&2
  printf '%s\n' "$listen_output" >&2
  return 1
}

generate_amcl_map() {
  if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 is required to generate the AMCL test map." >&2
    exit 1
  fi

  mkdir -p "$AMCL_MAP_DIR"
  python3 - "$AMCL_MAP_DIR" "$AMCL_MAP_BASENAME" <<'PY'
from pathlib import Path
import math
import sys

outdir = Path(sys.argv[1])
map_name = sys.argv[2]
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

pgm_path = outdir / f'{map_name}.pgm'
with pgm_path.open('w', encoding='ascii') as handle:
    handle.write('P2\n')
    handle.write(f'{width} {height}\n')
    handle.write('255\n')
    for row in reversed(grid):
        handle.write(' '.join(str(value) for value in row))
        handle.write('\n')

yaml_path = outdir / f'{map_name}.yaml'
yaml_path.write_text(
    '\n'.join(
        [
            f'image: {map_name}.pgm',
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

generate_office_amcl_map() {
  if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 is required to generate the office AMCL map." >&2
    exit 1
  fi

  mkdir -p "$AMCL_MAP_DIR"
  python3 - "$WORLD_PATH" "$AMCL_MAP_DIR" "$AMCL_MAP_BASENAME" <<'PY'
from pathlib import Path
import math
import re
import sys

world_path = Path(sys.argv[1])
outdir = Path(sys.argv[2])
map_name = sys.argv[3]
text = world_path.read_text(encoding='utf-8')

number = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'

def extract_blocks(kind):
    blocks = []
    needle = f'{kind} {{'
    cursor = 0
    while True:
        start = text.find(needle, cursor)
        if start < 0:
            return blocks

        depth = 0
        for index in range(start, len(text)):
            if text[index] == '{':
                depth += 1
            elif text[index] == '}':
                depth -= 1
                if depth == 0:
                    blocks.append(text[start:index + 1])
                    cursor = index + 1
                    break
        else:
            return blocks


def parse_vector(block, label, default=(0.0, 0.0, 0.0)):
    match = re.search(rf'^\s*{label}\s+({number})\s+({number})\s+({number})\s*$', block, re.M)
    if not match:
        return default
    return tuple(float(value) for value in match.groups())


def parse_rotation(block, default=(0.0, 0.0, 1.0, 0.0)):
    match = re.search(rf'^\s*rotation\s+({number})\s+({number})\s+({number})\s+({number})\s*$', block, re.M)
    if not match:
        return default
    return tuple(float(value) for value in match.groups())


floor_blocks = extract_blocks('Floor')
if not floor_blocks:
    raise SystemExit('No Floor blocks found in office world.')

preferred_floor = next((block for block in floor_blocks if 'name "floor(1)"' in block), floor_blocks[-1])
floor_translation = parse_vector(preferred_floor, 'translation')
floor_size_match = re.search(rf'^\s*size\s+({number})\s+({number})\s*$', preferred_floor, re.M)
if not floor_size_match:
    raise SystemExit('No floor size found in office world.')
floor_size = tuple(float(value) for value in floor_size_match.groups())

resolution = 0.05
origin_x = floor_translation[0] - floor_size[0] / 2.0
origin_y = floor_translation[1] - floor_size[1] / 2.0
width = max(1, int(math.ceil(floor_size[0] / resolution)))
height = max(1, int(math.ceil(floor_size[1] / resolution)))
grid = [[254 for _ in range(width)] for _ in range(height)]
inflate = resolution / 2.0
fixed_obstacle_defaults = {
    'Cabinet': (1.6, 0.45),
    'RoundTable': (1.0, 1.0),
    'Fridge': (0.8, 0.8),
    'Sink': (1.1, 0.7),
}

def mark_cell(ix, iy):
    if 0 <= ix < width and 0 <= iy < height:
        grid[iy][ix] = 0


def parse_size_2d(block, default=None):
    match_3d = re.search(rf'^\s*size\s+({number})\s+({number})\s+({number})\s*$', block, re.M)
    if match_3d:
        return tuple(float(value) for value in match_3d.groups()[:2])

    match_2d = re.search(rf'^\s*size\s+({number})\s+({number})\s*$', block, re.M)
    if match_2d:
        return tuple(float(value) for value in match_2d.groups())

    return default


def mark_rotated_rect(tx, ty, sx, sy, theta, extra_inflate=inflate):
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    half_x = abs(sx) / 2.0 + extra_inflate
    half_y = abs(sy) / 2.0 + extra_inflate
    span_x = abs(half_x * cos_t) + abs(half_y * sin_t)
    span_y = abs(half_x * sin_t) + abs(half_y * cos_t)
    min_x = max(0, int(math.floor((tx - span_x - origin_x) / resolution)))
    max_x = min(width - 1, int(math.ceil((tx + span_x - origin_x) / resolution)))
    min_y = max(0, int(math.floor((ty - span_y - origin_y) / resolution)))
    max_y = min(height - 1, int(math.ceil((ty + span_y - origin_y) / resolution)))

    for iy in range(min_y, max_y + 1):
        cy = origin_y + (iy + 0.5) * resolution
        dy = cy - ty
        for ix in range(min_x, max_x + 1):
            cx = origin_x + (ix + 0.5) * resolution
            dx = cx - tx
            local_x = dx * cos_t + dy * sin_t
            local_y = -dx * sin_t + dy * cos_t
            if abs(local_x) <= half_x and abs(local_y) <= half_y:
                mark_cell(ix, iy)


def yaw_from_block(block):
    _, _, axis_z, angle = parse_rotation(block)
    return angle if axis_z >= 0 else -angle


wall_count = 0
fixed_obstacle_count = 0

for block in extract_blocks('Wall'):
    tx, ty, _ = parse_vector(block, 'translation')
    size = parse_size_2d(block)
    if size is None:
        continue
    mark_rotated_rect(tx, ty, size[0], size[1], yaw_from_block(block))
    wall_count += 1

for kind in ['Window', 'Cabinet', 'Table', 'RoundTable', 'Fridge', 'Sink']:
    for block in extract_blocks(kind):
        tx, ty, _ = parse_vector(block, 'translation')
        size = parse_size_2d(block, fixed_obstacle_defaults.get(kind))
        if size is None:
            continue
        mark_rotated_rect(tx, ty, size[0], size[1], yaw_from_block(block), extra_inflate=0.0)
        fixed_obstacle_count += 1

pgm_path = outdir / f'{map_name}.pgm'
with pgm_path.open('w', encoding='ascii') as handle:
    handle.write('P2\n')
    handle.write(f'{width} {height}\n')
    handle.write('255\n')
    for row in reversed(grid):
        handle.write(' '.join(str(value) for value in row))
        handle.write('\n')

yaml_path = outdir / f'{map_name}.yaml'
yaml_path.write_text(
    '\n'.join(
        [
            f'image: {map_name}.pgm',
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

print(
    f'Generated {map_name}.yaml from {world_path.name} with '
    f'{wall_count} walls and {fixed_obstacle_count} fixed obstacles'
)
PY
}

generate_rectangular_arena_amcl_map() {
  if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 is required to generate the rectangular arena AMCL map." >&2
    exit 1
  fi

  mkdir -p "$AMCL_MAP_DIR"
  python3 - "$WORLD_PATH" "$AMCL_MAP_DIR" "$AMCL_MAP_BASENAME" <<'PY'
from pathlib import Path
import math
import re
import sys

world_path = Path(sys.argv[1])
outdir = Path(sys.argv[2])
map_name = sys.argv[3]
text = world_path.read_text(encoding='utf-8')

number = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
resolution = 0.05
wall_thickness = 0.1


def extract_blocks(kind):
    blocks = []
    needle = f'{kind} {{'
    cursor = 0
    while True:
        start = text.find(needle, cursor)
        if start < 0:
            return blocks

        depth = 0
        for index in range(start, len(text)):
            if text[index] == '{':
                depth += 1
            elif text[index] == '}':
                depth -= 1
                if depth == 0:
                    blocks.append(text[start:index + 1])
                    cursor = index + 1
                    break
        else:
            return blocks


def parse_vector(block, label, default=(0.0, 0.0, 0.0)):
    match = re.search(rf'^\s*{label}\s+({number})\s+({number})\s+({number})\s*$', block, re.M)
    if not match:
        return default
    return tuple(float(value) for value in match.groups())


def parse_rotation(block, default=(0.0, 0.0, 1.0, 0.0)):
    match = re.search(rf'^\s*rotation\s+({number})\s+({number})\s+({number})\s+({number})\s*$', block, re.M)
    if not match:
        return default
    return tuple(float(value) for value in match.groups())


def parse_box_size(block):
    match = re.search(
        rf'geometry\s+Box\s*\{{\s*size\s+({number})\s+({number})\s+({number})\s*',
        block,
        re.S,
    )
    if not match:
        match = re.search(
            rf'boundingObject\s+Box\s*\{{\s*size\s+({number})\s+({number})\s+({number})\s*',
            block,
            re.S,
        )
    if not match:
        return None
    return tuple(float(value) for value in match.groups())


arena_blocks = extract_blocks('RectangleArena')
if not arena_blocks:
    raise SystemExit(f'No RectangleArena block found in {world_path.name}.')

arena_block = arena_blocks[0]
floor_size_match = re.search(rf'^\s*floorSize\s+({number})\s+({number})\s*$', arena_block, re.M)
if not floor_size_match:
    raise SystemExit(f'No floorSize found in {world_path.name}.')

floor_width, floor_height = (float(value) for value in floor_size_match.groups())
origin_x = -floor_width / 2.0
origin_y = -floor_height / 2.0
width = max(1, int(math.ceil(floor_width / resolution)))
height = max(1, int(math.ceil(floor_height / resolution)))
grid = [[254 for _ in range(width)] for _ in range(height)]


def mark_cell(ix, iy):
    if 0 <= ix < width and 0 <= iy < height:
        grid[iy][ix] = 0


def yaw_from_rotation(rotation):
    axis_x, axis_y, axis_z, angle = rotation
    if abs(axis_z) >= max(abs(axis_x), abs(axis_y)):
        return angle if axis_z >= 0 else -angle
    return 0.0


def mark_rotated_rect(cx, cy, sx, sy, yaw, inflate=resolution / 2.0):
    half_x = abs(sx) / 2.0 + inflate
    half_y = abs(sy) / 2.0 + inflate
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    span_x = abs(half_x * cos_yaw) + abs(half_y * sin_yaw)
    span_y = abs(half_x * sin_yaw) + abs(half_y * cos_yaw)
    min_x = max(0, int(math.floor((cx - span_x - origin_x) / resolution)))
    max_x = min(width - 1, int(math.ceil((cx + span_x - origin_x) / resolution)))
    min_y = max(0, int(math.floor((cy - span_y - origin_y) / resolution)))
    max_y = min(height - 1, int(math.ceil((cy + span_y - origin_y) / resolution)))
    for iy in range(min_y, max_y + 1):
        world_y = origin_y + (iy + 0.5) * resolution
        dy = world_y - cy
        for ix in range(min_x, max_x + 1):
            world_x = origin_x + (ix + 0.5) * resolution
            dx = world_x - cx
            local_x = dx * cos_yaw + dy * sin_yaw
            local_y = -dx * sin_yaw + dy * cos_yaw
            if abs(local_x) <= half_x and abs(local_y) <= half_y:
                mark_cell(ix, iy)


outer_walls = [
    (0.0, floor_height / 2.0, floor_width, wall_thickness),
    (0.0, -floor_height / 2.0, floor_width, wall_thickness),
    (-floor_width / 2.0, 0.0, wall_thickness, floor_height),
    (floor_width / 2.0, 0.0, wall_thickness, floor_height),
]
for wall in outer_walls:
    mark_rotated_rect(*wall, 0.0)

wall_count = len(outer_walls)
for block in extract_blocks('Solid'):
    name_match = re.search(r'^\s*name\s+"([^"]+)"\s*$', block, re.M)
    name = name_match.group(1).lower() if name_match else ''
    if name in {'start', 'end'}:
        continue

    size = parse_box_size(block)
    if size is None or size[2] <= 0.2:
        continue

    tx, ty, _ = parse_vector(block, 'translation')
    yaw = yaw_from_rotation(parse_rotation(block))
    mark_rotated_rect(tx, ty, size[0], size[1], yaw)
    wall_count += 1

pgm_path = outdir / f'{map_name}.pgm'
with pgm_path.open('w', encoding='ascii') as handle:
    handle.write('P2\n')
    handle.write(f'{width} {height}\n')
    handle.write('255\n')
    for row in reversed(grid):
        handle.write(' '.join(str(value) for value in row))
        handle.write('\n')

yaml_path = outdir / f'{map_name}.yaml'
yaml_path.write_text(
    '\n'.join(
        [
            f'image: {map_name}.pgm',
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

print(f'Generated {map_name}.yaml from {world_path.name} with {wall_count} wall segments')
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
echo "RViz windows: $RVIZ_WINDOW_COUNT"
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
  if [ "$WORLD_BASENAME" = "office.wbt" ]; then
    log_step "Generating the office AMCL map"
    generate_office_amcl_map
  elif [ "$WORLD_BASENAME" = "confusing_maze.wbt" ] || [ "$WORLD_BASENAME" = "sandbox.wbt" ]; then
    log_step "Generating the rectangular arena AMCL map"
    generate_rectangular_arena_amcl_map
  else
    log_step "Generating the AMCL test map"
    generate_amcl_map
  fi
  if [ ! -f "$AMCL_MAP_YAML" ]; then
    echo "Failed to generate AMCL map at $AMCL_MAP_YAML" >&2
    exit 1
  fi
  echo "AMCL map YAML: $AMCL_MAP_YAML"
fi

STATIC_MAP_YAML=''
STATIC_MAP_YAML_IN_CONTAINER=''
if [ "$TEST_MODE" = "multi_mapping" ]; then
  log_step "Generating the static map for the multi-robot demo"
  generate_rectangular_arena_amcl_map
  STATIC_MAP_YAML="$AMCL_MAP_YAML"
  STATIC_MAP_YAML_IN_CONTAINER="$(container_path_for_host_path "$STATIC_MAP_YAML")"
  if [ ! -f "$STATIC_MAP_YAML" ]; then
    echo "Failed to generate static map at $STATIC_MAP_YAML" >&2
    exit 1
  fi
  echo "Static map YAML: $STATIC_MAP_YAML"
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
if [[ "$FORCE_CLEAN" =~ ^(1|true|yes|on)$ ]]; then
  cleanup_existing_demo
fi
check_bridge_ports_available
if [ "$TEST_MODE" = "amcl" ]; then
  docker_compose run -d \
    --service-ports \
    --name "$CONTAINER_NAME" \
    -e RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE" \
    -e RMPD_TEST_MODE="$TEST_MODE" \
    -e RMPD_MAP_ID="${RMPD_MAP_ID:-}" \
    -e RMPD_AMCL_MAP_NAME="${RMPD_AMCL_MAP_NAME:-$AMCL_MAP_BASENAME}" \
    -e RMPD_BRIDGE_PORT_SECONDARY="${RMPD_BRIDGE_PORT_SECONDARY:-}" \
    -e RMPD_START_SECONDARY_BRIDGE="${RMPD_START_SECONDARY_BRIDGE:-}" \
    -e RMPD_SECONDARY_BRIDGE_TOPIC_PREFIX="${RMPD_SECONDARY_BRIDGE_TOPIC_PREFIX:-}" \
    -e RMPD_AMCL_MAP_YAML="$AMCL_MAP_YAML_IN_CONTAINER" \
    -e RMPD_AMCL_INITIAL_POSE_X="$AMCL_INITIAL_POSE_X" \
    -e RMPD_AMCL_INITIAL_POSE_Y="$AMCL_INITIAL_POSE_Y" \
    -e RMPD_AMCL_INITIAL_POSE_YAW="$AMCL_INITIAL_POSE_YAW" \
    -e RMPD_AMCL_INITIAL_POSE_USE_ODOM="${RMPD_AMCL_INITIAL_POSE_USE_ODOM:-true}" \
    -e RMPD_START_CHECKPOINT_PATROL="${RMPD_START_CHECKPOINT_PATROL:-true}" \
    -e RMPD_START_CHECKPOINT_METRICS="${RMPD_START_CHECKPOINT_METRICS:-true}" \
    -e RMPD_CHECKPOINT_METRICS_RADIUS_M="${RMPD_CHECKPOINT_METRICS_RADIUS_M:-0.40}" \
    -e RMPD_CHECKPOINT_METRICS_RESET="${RMPD_CHECKPOINT_METRICS_RESET:-true}" \
    -e RMPD_CHECKPOINT_METRICS_DIR="${RMPD_CHECKPOINT_METRICS_DIR:-${CONTAINER_WORKSPACE%/}/logs/checkpoint_metrics}" \
    -e RMPD_CHECKPOINT_METRICS_OUTPUT_CSV="${RMPD_CHECKPOINT_METRICS_OUTPUT_CSV:-}" \
    -e RMPD_START_NAVIGATION_DIAGNOSTICS="${RMPD_START_NAVIGATION_DIAGNOSTICS:-true}" \
    -e RMPD_START_LIVE_MAPPING="${RMPD_START_LIVE_MAPPING:-true}" \
    ros2 bash scripts/start_ros2_stack.sh >/dev/null
else
  docker_compose run -d \
    --service-ports \
    --name "$CONTAINER_NAME" \
    -e RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE" \
    -e RMPD_TEST_MODE="$TEST_MODE" \
    -e RMPD_MAP_ID="${RMPD_MAP_ID:-}" \
    -e RMPD_BRIDGE_PORT_SECONDARY="${RMPD_BRIDGE_PORT_SECONDARY:-}" \
    -e RMPD_START_SECONDARY_BRIDGE="${RMPD_START_SECONDARY_BRIDGE:-}" \
    -e RMPD_SECONDARY_BRIDGE_TOPIC_PREFIX="${RMPD_SECONDARY_BRIDGE_TOPIC_PREFIX:-}" \
    -e RMPD_LIVE_MAP_WIDTH_M="${RMPD_LIVE_MAP_WIDTH_M:-8.0}" \
    -e RMPD_LIVE_MAP_HEIGHT_M="${RMPD_LIVE_MAP_HEIGHT_M:-8.0}" \
    -e RMPD_LIVE_MAP_ORIGIN_X="${RMPD_LIVE_MAP_ORIGIN_X:-nan}" \
    -e RMPD_LIVE_MAP_ORIGIN_Y="${RMPD_LIVE_MAP_ORIGIN_Y:-nan}" \
    -e RMPD_STATIC_MAP_YAML="$STATIC_MAP_YAML_IN_CONTAINER" \
    -e RMPD_AMCL_MAP_YAML="$STATIC_MAP_YAML_IN_CONTAINER" \
    -e RMPD_ROBOT_1_COMPROMISED="${RMPD_ROBOT_1_COMPROMISED:-false}" \
    -e RMPD_ROBOT_2_COMPROMISED="${RMPD_ROBOT_2_COMPROMISED:-false}" \
    -e RMPD_FAKE_REPORT_RADIUS_CELLS="${RMPD_FAKE_REPORT_RADIUS_CELLS:-2}" \
    -e RMPD_FAKE_OBSTACLE_INJECTOR_MODE="${RMPD_FAKE_OBSTACLE_INJECTOR_MODE:-clicked_point}" \
    -e RMPD_START_CHECKPOINT_PATROL="${RMPD_START_CHECKPOINT_PATROL:-false}" \
    -e RMPD_START_CHECKPOINT_METRICS="${RMPD_START_CHECKPOINT_METRICS:-true}" \
    -e RMPD_CHECKPOINT_METRICS_RADIUS_M="${RMPD_CHECKPOINT_METRICS_RADIUS_M:-0.40}" \
    -e RMPD_CHECKPOINT_METRICS_RESET="${RMPD_CHECKPOINT_METRICS_RESET:-true}" \
    -e RMPD_CHECKPOINT_METRICS_DIR="${RMPD_CHECKPOINT_METRICS_DIR:-${CONTAINER_WORKSPACE%/}/logs/checkpoint_metrics}" \
    ros2 bash scripts/start_ros2_stack.sh >/dev/null
fi

log_step "Waiting for ROS 2 build/setup"
wait_for_log 'ROS 2 workspace ready' 240 'ROS 2 workspace build'

log_step "Waiting for the ROS bridge and localization stack to come up"
if [ "$TEST_MODE" = "amcl" ]; then
  wait_for_log 'Listening for Webots packets' 180 'ROS bridge listener'
  wait_for_log 'Pose-to-odom ready' 120 'pose-to-odom node'
  wait_for_log 'Initial pose publisher ready' 120 'initial pose publisher'
  wait_for_log 'Read map' 45 '/map loaded by map_server' || echo 'Warning: map load log was not observed before Webots data wait.'
  stream_ros_logs
else
  wait_for_ros2_nodes 'robot_1_bridge robot_2_bridge robot_1_map_builder robot_2_map_builder robot_1_view_belief robot_2_view_belief' 180 'multi-robot mapping nodes' || \
    echo 'Warning: multi-robot mapping nodes were not confirmed before Webots data wait.'
  wait_for_log 'Map builder ready' 180 'map builder'
  stream_ros_logs
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
wait_for_log 'Published Webots packet #' 120 'Webots bridge packets' || echo 'Warning: bridge packet log was not observed before continuing.'
if [ "$TEST_MODE" = "amcl" ]; then
  log_step "Waiting for Nav2 lifecycle after Webots pose is available"
  wait_for_log 'Managed nodes are active' 90 'Nav2 lifecycle active' || echo 'Warning: lifecycle active log was not observed before continuing.'
fi

if should_launch_rviz; then
  if [ "$TEST_MODE" = "amcl" ]; then
    echo "RViz will open now; the map and TF tree may take a few seconds to settle."
  fi

  log_step "Launching RViz in Docker"
  RVIZ_CONFIG_PATH=''
  RVIZ_CONFIG_PATHS=()

  if [ -n "$RVIZ_CONFIG_FILES" ]; then
    IFS=',' read -r -a requested_rviz_configs <<< "$RVIZ_CONFIG_FILES"
    for rviz_config_file in "${requested_rviz_configs[@]}"; do
      rviz_config_file="${rviz_config_file#"${rviz_config_file%%[![:space:]]*}"}"
      rviz_config_file="${rviz_config_file%"${rviz_config_file##*[![:space:]]}"}"
      [ -n "$rviz_config_file" ] || continue
      rviz_config_path="$(
        docker exec -e RMPD_RVIZ_CONFIG_FILE="$rviz_config_file" -e RMPD_COLCON_INSTALL_BASE="$CONTAINER_INSTALL_BASE" "$CONTAINER_NAME" bash -lc '
          source /opt/ros/jazzy/setup.bash
          source "${RMPD_COLCON_INSTALL_BASE:-/tmp/rmpd_colcon_install}/setup.bash"
          rviz_config_file="${RMPD_RVIZ_CONFIG_FILE}"
          if [[ "$rviz_config_file" = /* ]]; then
            printf "%s\n" "$rviz_config_file"
          else
            pkg_prefix="$(ros2 pkg prefix robot_patrol_node)"
            printf "%s/share/robot_patrol_node/config/%s\n" "$pkg_prefix" "$rviz_config_file"
          fi
        ' | tr -d '\r'
      )"
      if docker exec "$CONTAINER_NAME" test -f "$rviz_config_path"; then
        RVIZ_CONFIG_PATHS+=("$rviz_config_path")
        echo "Using RViz config: $rviz_config_path"
      else
        echo "Warning: RViz config was not found at $rviz_config_path; skipping it." >&2
      fi
    done
  elif [ -n "$RVIZ_CONFIG_FILE" ]; then
    RVIZ_CONFIG_PATH="$(
      docker exec -e RMPD_RVIZ_CONFIG_FILE="$RVIZ_CONFIG_FILE" -e RMPD_COLCON_INSTALL_BASE="$CONTAINER_INSTALL_BASE" "$CONTAINER_NAME" bash -lc '
        source /opt/ros/jazzy/setup.bash
        source "${RMPD_COLCON_INSTALL_BASE:-/tmp/rmpd_colcon_install}/setup.bash"
        rviz_config_file="${RMPD_RVIZ_CONFIG_FILE}"
        if [[ "$rviz_config_file" = /* ]]; then
          printf "%s\n" "$rviz_config_file"
        else
          pkg_prefix="$(ros2 pkg prefix robot_patrol_node)"
          printf "%s/share/robot_patrol_node/config/%s\n" "$pkg_prefix" "$rviz_config_file"
        fi
      ' | tr -d '\r'
    )"

    if ! docker exec "$CONTAINER_NAME" test -f "$RVIZ_CONFIG_PATH"; then
      echo "Warning: RViz config was not found at $RVIZ_CONFIG_PATH; falling back to default.rviz" >&2
      RVIZ_CONFIG_PATH=''
    else
      echo "Using RViz config: $RVIZ_CONFIG_PATH"
    fi
  fi

  if [ -z "$RVIZ_CONFIG_PATH" ]; then
    if [ "${#RVIZ_CONFIG_PATHS[@]}" -eq 0 ]; then
      RVIZ_CONFIG_PATH="$(
      docker exec -e RMPD_COLCON_INSTALL_BASE="$CONTAINER_INSTALL_BASE" "$CONTAINER_NAME" bash -lc '
          source /opt/ros/jazzy/setup.bash
          source "${RMPD_COLCON_INSTALL_BASE:-/tmp/rmpd_colcon_install}/setup.bash"
          pkg_prefix="$(ros2 pkg prefix robot_patrol_node)"
          printf "%s/share/robot_patrol_node/config/default.rviz\n" "$pkg_prefix"
        ' | tr -d '\r'
      )"
      if ! docker exec "$CONTAINER_NAME" test -f "$RVIZ_CONFIG_PATH"; then
        echo "Warning: default RViz config was not found at $RVIZ_CONFIG_PATH; letting ros2 launch choose its own default." >&2
        RVIZ_CONFIG_PATH=''
      fi
    fi
  fi

  mkdir -p "$RVIZ_HOST_LOG_DIR"
  for ((rviz_index = 1; rviz_index <= RVIZ_WINDOW_COUNT; rviz_index++)); do
    rviz_host_log="$RVIZ_HOST_LOG_DIR/rmpd_rviz_${rviz_index}.log"
    RVIZ_HOST_LOGS+=("$rviz_host_log")
    rm -f "$rviz_host_log"
    rviz_config_path_for_window=''
    if [ "${#RVIZ_CONFIG_PATHS[@]}" -gt 0 ]; then
      rviz_config_array_index=$((rviz_index - 1))
      if [ "$rviz_config_array_index" -lt "${#RVIZ_CONFIG_PATHS[@]}" ]; then
        rviz_config_path_for_window="${RVIZ_CONFIG_PATHS[$rviz_config_array_index]}"
      else
        rviz_config_path_for_window="${RVIZ_CONFIG_PATHS[$((${#RVIZ_CONFIG_PATHS[@]} - 1))]}"
      fi
    else
      rviz_config_path_for_window="$RVIZ_CONFIG_PATH"
    fi

    if [ -n "$rviz_config_path_for_window" ]; then
      docker exec -e RMPD_RVIZ_CONFIG_PATH="$rviz_config_path_for_window" -e RMPD_COLCON_INSTALL_BASE="$CONTAINER_INSTALL_BASE" "$CONTAINER_NAME" bash -lc 'source /opt/ros/jazzy/setup.bash && source "${RMPD_COLCON_INSTALL_BASE:-/tmp/rmpd_colcon_install}/setup.bash" && export LIBGL_ALWAYS_SOFTWARE=1 && export QT_X11_NO_MITSHM=1 && export QT_QPA_PLATFORM=xcb && exec ros2 launch robot_patrol_node rviz.launch.py rviz_config:="$RMPD_RVIZ_CONFIG_PATH"' >"$rviz_host_log" 2>&1 &
    else
      docker exec -e RMPD_COLCON_INSTALL_BASE="$CONTAINER_INSTALL_BASE" "$CONTAINER_NAME" bash -lc 'source /opt/ros/jazzy/setup.bash && source "${RMPD_COLCON_INSTALL_BASE:-/tmp/rmpd_colcon_install}/setup.bash" && export LIBGL_ALWAYS_SOFTWARE=1 && export QT_X11_NO_MITSHM=1 && export QT_QPA_PLATFORM=xcb && exec ros2 launch robot_patrol_node rviz.launch.py' >"$rviz_host_log" 2>&1 &
    fi
    RVIZ_EXEC_PIDS+=("$!")
    echo "RViz window $rviz_index launch command submitted to $CONTAINER_NAME"
    echo "RViz window $rviz_index host log: $rviz_host_log"
  done

  log_step "Checking RViz startup"
  sleep "$RVIZ_CHECK_SECONDS"

  for rviz_index in "${!RVIZ_EXEC_PIDS[@]}"; do
    rviz_pid="${RVIZ_EXEC_PIDS[$rviz_index]}"
    rviz_host_log="${RVIZ_HOST_LOGS[$rviz_index]}"
    window_number=$((rviz_index + 1))

    if ! kill -0 "$rviz_pid" >/dev/null 2>&1; then
      echo "RViz window $window_number appears to have exited during startup. RViz log:" >&2
      tail -120 "$rviz_host_log" >&2 || true
      exit 1
    fi

    # RViz's Map display can emit a known GLSL shader warning under WSLg/Docker
    # even when the process stays alive and the map continues to render.
    if grep -Eqi 'rviz2-[0-9]+.*process has died|Invalid parentWindowHandle|Unable to create the rendering window|Qt.*could not connect to display|Could not load the Qt platform plugin|Aborted|segmentation fault|core dumped' "$rviz_host_log" 2>/dev/null; then
      echo "RViz window $window_number appears to have failed during startup. RViz log:" >&2
      tail -120 "$rviz_host_log" >&2 || true
      exit 1
    fi
  done
else
  log_step "Skipping RViz GUI startup"
  echo "No Docker-visible display was detected. Set RMPD_QUICK_TEST_RVIZ=true after configuring a Docker-visible display to require RViz GUI startup."
  docker exec "$CONTAINER_NAME" bash -lc 'source /opt/ros/jazzy/setup.bash && command -v rviz2 >/dev/null'
fi

log_step "Quick test checks passed"
echo "Docker stack started, Webots bridge data arrived, and RViz availability/startup checks passed."

log_step "Quick test is running"
echo "If you want to stop everything, close Webots or press Ctrl-C here."
  if [ "${#RVIZ_HOST_LOGS[@]}" -gt 0 ]; then
    echo "RViz logs will remain available at:"
    for rviz_host_log in "${RVIZ_HOST_LOGS[@]}"; do
      echo "  $rviz_host_log"
    done
  fi

  if [ "$TEST_MODE" = "multi_mapping" ]; then
    wait_for_ros2_nodes 'robot_1_view_belief robot_2_view_belief' 60 'shared map belief nodes' || \
      echo 'Warning: shared map belief nodes were not confirmed after RViz launch.'
  fi

  wait "$WEBOTS_PID"
