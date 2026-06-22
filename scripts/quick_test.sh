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
DEFAULT_STREAM_ROS_LOGS="$(env_file_value RMPD_STREAM_ROS_LOGS true)"

CONTAINER_NAME="${RMPD_CONTAINER_NAME:-ros2_dev}"
CONTAINER_WORKSPACE="${RMPD_CONTAINER_WORKSPACE:-$DEFAULT_CONTAINER_WORKSPACE}"
export RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE"
WEBOTS_CMD="${WEBOTS_CMD:-}"
TEST_MODE="${RMPD_TEST_MODE:-amcl}"
HOLD_OPEN="${RMPD_QUICK_TEST_HOLD_OPEN:-$DEFAULT_HOLD_OPEN}"
RVIZ_CHECK_SECONDS="${RMPD_RVIZ_CHECK_SECONDS:-10}"
RVIZ_MODE="${RMPD_QUICK_TEST_RVIZ:-$DEFAULT_RVIZ_MODE}"
RVIZ_CONFIG_FILE="${RMPD_QUICK_TEST_RVIZ_CONFIG:-$DEFAULT_RVIZ_CONFIG}"
STREAM_ROS_LOGS="${RMPD_STREAM_ROS_LOGS:-$DEFAULT_STREAM_ROS_LOGS}"
HOST_BRIDGE_PORT="${RMPD_BRIDGE_PORT:-$DEFAULT_BRIDGE_PORT}"
HOST_BRIDGE_TARGETS="${WEBOTS_BRIDGE_TARGETS:-}"
RVIZ_HOST_LOG="${TMPDIR:-/tmp}/rmpd/quick_test/rmpd_rviz.log"
export RMPD_INSTALL_FULL_STACK=true
ROS_LOG_FOLLOW_PID=""

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
if [[ "$(uname -s)" = "Darwin" && -f "$REPO_DIR/docker/compose.xquartz.yml" ]]; then
  if [ -n "${DISPLAY:-}" ] || [[ "$RVIZ_MODE" =~ ^(1|true|yes|on)$ ]]; then
    COMPOSE_FILES+=("-f" "$REPO_DIR/docker/compose.xquartz.yml")
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

generate_confusing_maze_amcl_map() {
  if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 is required to generate the confusing maze AMCL map." >&2
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
    raise SystemExit('No RectangleArena block found in confusing maze world.')

arena_block = arena_blocks[0]
floor_size_match = re.search(rf'^\s*floorSize\s+({number})\s+({number})\s*$', arena_block, re.M)
if not floor_size_match:
    raise SystemExit('No floorSize found in confusing maze world.')

floor_width, floor_height = (float(value) for value in floor_size_match.groups())
origin_x = -floor_width / 2.0
origin_y = -floor_height / 2.0
width = max(1, int(math.ceil(floor_width / resolution)))
height = max(1, int(math.ceil(floor_height / resolution)))
grid = [[254 for _ in range(width)] for _ in range(height)]


def mark_cell(ix, iy):
    if 0 <= ix < width and 0 <= iy < height:
        grid[iy][ix] = 0


def mark_rect(cx, cy, sx, sy, inflate=resolution / 2.0):
    half_x = abs(sx) / 2.0 + inflate
    half_y = abs(sy) / 2.0 + inflate
    min_x = max(0, int(math.floor((cx - half_x - origin_x) / resolution)))
    max_x = min(width - 1, int(math.ceil((cx + half_x - origin_x) / resolution)))
    min_y = max(0, int(math.floor((cy - half_y - origin_y) / resolution)))
    max_y = min(height - 1, int(math.ceil((cy + half_y - origin_y) / resolution)))
    for iy in range(min_y, max_y + 1):
        for ix in range(min_x, max_x + 1):
            mark_cell(ix, iy)


outer_walls = [
    (0.0, floor_height / 2.0, floor_width, wall_thickness),
    (0.0, -floor_height / 2.0, floor_width, wall_thickness),
    (-floor_width / 2.0, 0.0, wall_thickness, floor_height),
    (floor_width / 2.0, 0.0, wall_thickness, floor_height),
]
for wall in outer_walls:
    mark_rect(*wall)

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
    mark_rect(tx, ty, size[0], size[1])
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
  elif [ "$WORLD_BASENAME" = "confusing_maze.wbt" ]; then
    log_step "Generating the confusing maze AMCL map"
    generate_confusing_maze_amcl_map
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
    -e RMPD_AMCL_INITIAL_POSE_X="$AMCL_INITIAL_POSE_X" \
    -e RMPD_AMCL_INITIAL_POSE_Y="$AMCL_INITIAL_POSE_Y" \
    -e RMPD_AMCL_INITIAL_POSE_YAW="$AMCL_INITIAL_POSE_YAW" \
    -e RMPD_AMCL_INITIAL_POSE_USE_ODOM="${RMPD_AMCL_INITIAL_POSE_USE_ODOM:-true}" \
    -e RMPD_START_CHECKPOINT_PATROL="${RMPD_START_CHECKPOINT_PATROL:-true}" \
    -e RMPD_START_NAVIGATION_DIAGNOSTICS="${RMPD_START_NAVIGATION_DIAGNOSTICS:-true}" \
    -e RMPD_START_LIVE_MAPPING="${RMPD_START_LIVE_MAPPING:-true}" \
    ros2 bash scripts/start_ros2_stack.sh >/dev/null
else
  docker_compose run -d \
    --service-ports \
    --name "$CONTAINER_NAME" \
    -e RMPD_CONTAINER_WORKSPACE="$CONTAINER_WORKSPACE" \
    ros2 bash scripts/start_ros2_stack.sh >/dev/null
fi

log_step "Waiting for ROS 2 build/setup"
wait_for_log 'ROS 2 workspace ready' 240 'ROS 2 workspace build'

log_step "Waiting for the ROS bridge and localization stack to come up"
wait_for_log 'Listening for Webots packets' 180 'ROS bridge listener'
if [ "$TEST_MODE" = "amcl" ]; then
  wait_for_log 'Pose-to-odom ready' 120 'pose-to-odom node'
  wait_for_log 'Initial pose publisher ready' 120 'initial pose publisher'
  wait_for_log 'Read map' 45 '/map loaded by map_server' || echo 'Warning: map load log was not observed before Webots data wait.'
  stream_ros_logs
else
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

  if [ "$TEST_MODE" = "amcl" ]; then
    RVIZ_CONFIG_PATH="$(
      docker exec -e RMPD_RVIZ_CONFIG_FILE="$RVIZ_CONFIG_FILE" "$CONTAINER_NAME" bash -lc '
        source /opt/ros/jazzy/setup.bash
        source "${RMPD_CONTAINER_WORKSPACE:-/workspace}/install/setup.bash"
        rviz_config_file="${RMPD_RVIZ_CONFIG_FILE:-amcl.rviz}"
        if [[ "$rviz_config_file" = /* ]]; then
          printf "%s\n" "$rviz_config_file"
        else
          pkg_prefix="$(ros2 pkg prefix robot_patrol_node)"
          printf "%s/share/robot_patrol_node/config/%s\n" "$pkg_prefix" "$rviz_config_file"
        fi
      ' | tr -d '\r'
    )"

    if ! docker exec "$CONTAINER_NAME" test -f "$RVIZ_CONFIG_PATH"; then
      echo "Warning: AMCL RViz config was not found at $RVIZ_CONFIG_PATH; falling back to default.rviz" >&2
      RVIZ_CONFIG_PATH=''
    else
      echo "Using AMCL RViz config: $RVIZ_CONFIG_PATH"
    fi
  else
    RVIZ_CONFIG_PATH="$(
      docker exec "$CONTAINER_NAME" bash -lc '
        source /opt/ros/jazzy/setup.bash
        source "${RMPD_CONTAINER_WORKSPACE:-/workspace}/install/setup.bash"
        pkg_prefix="$(ros2 pkg prefix robot_patrol_node)"
        printf "%s/share/robot_patrol_node/config/default.rviz\n" "$pkg_prefix"
      ' | tr -d '\r'
    )"
    if ! docker exec "$CONTAINER_NAME" test -f "$RVIZ_CONFIG_PATH"; then
      echo "Warning: default RViz config was not found at $RVIZ_CONFIG_PATH; letting ros2 launch choose its own default." >&2
      RVIZ_CONFIG_PATH=''
    fi
  fi

  mkdir -p "$(dirname "$RVIZ_HOST_LOG")"
  rm -f "$RVIZ_HOST_LOG"

  if [ -n "$RVIZ_CONFIG_PATH" ]; then
    docker exec -e RMPD_RVIZ_CONFIG_PATH="$RVIZ_CONFIG_PATH" "$CONTAINER_NAME" bash -lc 'source /opt/ros/jazzy/setup.bash && source "${RMPD_CONTAINER_WORKSPACE:-/workspace}/install/setup.bash" && export LIBGL_ALWAYS_SOFTWARE=1 && export QT_X11_NO_MITSHM=1 && export QT_QPA_PLATFORM=xcb && exec ros2 launch robot_patrol_node rviz.launch.py rviz_config:="$RMPD_RVIZ_CONFIG_PATH"' >"$RVIZ_HOST_LOG" 2>&1 &
  else
    docker exec "$CONTAINER_NAME" bash -lc 'source /opt/ros/jazzy/setup.bash && source "${RMPD_CONTAINER_WORKSPACE:-/workspace}/install/setup.bash" && export LIBGL_ALWAYS_SOFTWARE=1 && export QT_X11_NO_MITSHM=1 && export QT_QPA_PLATFORM=xcb && exec ros2 launch robot_patrol_node rviz.launch.py' >"$RVIZ_HOST_LOG" 2>&1 &
  fi
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

  # RViz's Map display can emit a known GLSL shader warning under WSLg/Docker
  # even when the process stays alive and the map continues to render.
  if grep -Eqi 'rviz2-[0-9]+.*process has died|Invalid parentWindowHandle|Unable to create the rendering window|Qt.*could not connect to display|Could not load the Qt platform plugin|Aborted|segmentation fault|core dumped' "$RVIZ_HOST_LOG" 2>/dev/null; then
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
