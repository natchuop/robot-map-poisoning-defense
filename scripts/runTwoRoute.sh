#!/usr/bin/env bash
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_DIR="$REPO_DIR/webots/worlds/twoRoute"
SOURCE_WORLD="$WORLD_DIR/two_route.wbt"
SANDBOX_WORLD="$WORLD_DIR/sandbox.wbt"

cleanup() {
  rm -f "$SANDBOX_WORLD"
}

trap cleanup EXIT

next_trial_id() {
  python3 - "$REPO_DIR" <<'PY'
import csv
import re
import sys
from pathlib import Path

repo_dir = Path(sys.argv[1])
prefix = 'runTwoRoute_trial_'
max_index = 0

for csv_path in [
    repo_dir / 'results/map_accuracy/raw/map_accuracy_timeseries.csv',
    repo_dir / 'results/map_accuracy/processed/summary_by_trial.csv',
]:
    if not csv_path.is_file():
        continue
    try:
        with csv_path.open('r', newline='', encoding='utf-8') as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames or 'trial_id' not in reader.fieldnames:
                continue
            for row in reader:
                trial_id = str(row.get('trial_id', '')).strip()
                match = re.fullmatch(r'runTwoRoute_trial_(\d+)', trial_id)
                if match:
                    max_index = max(max_index, int(match.group(1)))
    except OSError:
        continue

print(f'{prefix}{max_index + 1:03d}')
PY
}

cp "$SOURCE_WORLD" "$SANDBOX_WORLD"
export RMPD_WEBOTS_WORLD="$SANDBOX_WORLD"
export RMPD_TEST_MODE="multi_mapping"
export RMPD_MULTI_ROBOT_CONFIG="$REPO_DIR/webots/worlds/twoRoute/multi_robot_config.json"
export RMPD_RVIZ_CONFIG_FILES="${RMPD_RVIZ_CONFIG_FILES:-multi_robot_robot_1_view.rviz,multi_robot_robot_2_view.rviz}"
export RMPD_RVIZ_WINDOW_COUNT="${RMPD_RVIZ_WINDOW_COUNT:-2}"
export RMPD_QUICK_TEST_RVIZ="${RMPD_QUICK_TEST_RVIZ:-true}"
export RMPD_QUICK_TEST_FORCE_CLEAN="${RMPD_QUICK_TEST_FORCE_CLEAN:-true}"
export RMPD_ROBOT_1_COMPROMISED="${RMPD_ROBOT_1_COMPROMISED:-true}"
export RMPD_ROBOT_2_COMPROMISED="${RMPD_ROBOT_2_COMPROMISED:-true}"
export RMPD_ROBOT_1_TRUST_WEIGHT="${RMPD_ROBOT_1_TRUST_WEIGHT:-1.0}"
export RMPD_ROBOT_2_TRUST_WEIGHT="${RMPD_ROBOT_2_TRUST_WEIGHT:-0.35}"
export RMPD_FAKE_REPORT_RADIUS_CELLS="${RMPD_FAKE_REPORT_RADIUS_CELLS:-2}"
export RMPD_FAKE_OBSTACLE_INJECTOR_MODE="${RMPD_FAKE_OBSTACLE_INJECTOR_MODE:-clicked_point}"
export RMPD_LIVE_MAP_WIDTH_M="${RMPD_LIVE_MAP_WIDTH_M:-14.0}"
export RMPD_LIVE_MAP_HEIGHT_M="${RMPD_LIVE_MAP_HEIGHT_M:-6.0}"
export RMPD_LIVE_MAP_ORIGIN_X="${RMPD_LIVE_MAP_ORIGIN_X:--6.5}"
export RMPD_LIVE_MAP_ORIGIN_Y="${RMPD_LIVE_MAP_ORIGIN_Y:--3.0}"
export RMPD_ENABLE_MAP_ACCURACY_EVALUATOR="${RMPD_ENABLE_MAP_ACCURACY_EVALUATOR:-true}"
export RMPD_MAP_ACCURACY_RESULTS_DIR="${RMPD_MAP_ACCURACY_RESULTS_DIR:-results/map_accuracy}"
export RMPD_MAP_ACCURACY_YAML_CONFIG_FILE="${RMPD_MAP_ACCURACY_YAML_CONFIG_FILE:-map_accuracy_evaluator_two_route.yaml}"
export RMPD_MAP_ACCURACY_TRIAL_ID="${RMPD_MAP_ACCURACY_TRIAL_ID:-$(next_trial_id)}"
export RMPD_MAP_ACCURACY_LOG_PERIOD_SEC="${RMPD_MAP_ACCURACY_LOG_PERIOD_SEC:-2.0}"
bash "$REPO_DIR/scripts/quick_test.sh"
