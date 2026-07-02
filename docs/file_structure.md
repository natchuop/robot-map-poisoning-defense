# Repo Structure and Simulation Setup for Final Trust-Based Map Poisoning Defense

This repository should support a ROS 2 + Webots simulation for decentralized multi-robot mapping, trust-based map confidence, fake-obstacle injection, claim-level verification, and navigation evaluation.

The final project should be organized so the same maps, routes, and attacks can be run under three fusion models:

```text
1. log_odds
2. mate_log_odds
3. mate_claim_verification
```

The project claim is:

```text
Can decentralized trust-weighted map fusion reduce the effects of map-poisoning attacks on multi-robot navigation and final map accuracy?
```

## Top Level

```text
README.md
docs/
scripts/
src/
webots/
docker/
results/
logs/
```

Recommended purpose of each folder:

- `README.md`: short setup instructions, experiment overview, and main launch commands.
- `docs/`: project plan, combined structure/simulation guide, verification guide, and metric definitions.
- `scripts/`: launch scripts, verification scripts, trial runners, and analysis helpers.
- `src/`: ROS 2 packages for messages, mapping, trust, verification, navigation adapters, and fake obstacle injection.
- `webots/`: Webots worlds, maps, robot controllers, and world-specific assets.
- `docker/`: Docker image and compose files for ROS 2, Nav2, RViz, and build tools.
- `results/`: generated CSV files, plots, and final experiment summaries.
- `logs/`: raw run logs and debugging output.

Generated build folders such as `build/`, `install/`, and `log/` should not be edited by hand.

## ROS Packages

### `src/robot_patrol_msgs`

This package should define custom messages used by the trust and map-poisoning defense system.

Recommended messages:

```text
MapUpdate.msg
VerificationReceipt.msg
TrustState.msg
TrialStatus.msg
```

Current code status:

- `MapUpdate.msg` exists now.
- The verification, trust-state, and trial-status messages are still planned.

### `MapUpdate.msg`

Purpose: represent a shared map claim from one robot.

Suggested fields:

```text
string claim_id
string reporting_robot_id
string target_robot_id
int32 cell_x
int32 cell_y
float64 world_x
float64 world_y
string reported_state        # OCCUPIED or FREE
bool occupied
string source
string attack_type
bool is_attack_report        # only for simulation/evaluation logs
builtin_interfaces/Time stamp
```

### `VerificationReceipt.msg`

Purpose: record whether a later observation confirmed or contradicted a claim.

Suggested fields:

```text
string claim_id
string reporting_robot_id
string verifying_robot_id
int32 cell_x
int32 cell_y
string original_claim_type
string verification_result   # CONFIRMED, CONTRADICTED, or UNCERTAIN
builtin_interfaces/Time verification_time
geometry_msgs/Pose verifier_pose
string scan_id
```

### `TrustState.msg`

Purpose: expose robot trust and trust confidence for logging or visualization.

Suggested fields:

```text
string observer_robot_id
string reporting_robot_id
float64 alpha_lifetime
float64 beta_lifetime
float64 alpha_recent
float64 beta_recent
float64 trust_lifetime
float64 trust_recent
float64 trust_combined
float64 trust_precision
float64 trust_confidence
float64 caution_lambda
float64 update_ramp
bool quarantined
```

## Main ROS Package

### `src/robot_patrol_node`

This package should contain the executable nodes, launch files, configs, and experiment logic.

Recommended subfolders:

```text
launch/
config/
robot_patrol_node/
resource/
test/
```

Current code status:

- `launch/`, `config/`, `robot_patrol_node/`, `resource/`, and `test/` all exist.
- The package currently contains executable nodes, launch files, config files, and tests, but not yet every node described below.

## Launch Files

Recommended final launch files:

```text
multi_robot_mapping.launch.py
fake_obstacle_experiment.launch.py
nav2_stack.launch.py
amcl_stack.launch.py
rviz.launch.py
```

Current code status:

- `multi_robot_mapping.launch.py` exists now.
- `amcl_stack.launch.py`, `nav2_stack.launch.py`, and `rviz.launch.py` exist now.
- `fake_obstacle_experiment.launch.py` is still a planned umbrella launch file.
- The code also currently includes `nav2_with_amcl.launch.py`, `map_builder.launch.py`, `mapping_stack.launch.py`, and `fake_obstacle_injector.launch.py`.

### `multi_robot_mapping.launch.py`

Starts the shared mapping system:

- ROS bridge nodes for each robot.
- Per-robot map builders.
- Per-robot shared-map merge nodes.
- Static map server if using AMCL/Nav2.
- Confidence overlays.
- Claim-based fake obstacle injectors.
- Method 1 log-odds fusion parameters.
- Per-robot RViz views when enabled.

### `fake_obstacle_experiment.launch.py`

Starts the full experiment path:

- Selected Webots world.
- Selected checkpoint route.
- Selected fusion mode.
- Fake obstacle injector.
- Trial logger.
- RViz views if GUI mode is enabled.

The launch should accept parameters such as:

```text
fusion_mode
map_name
trial_id
random_seed
attack_enabled
attack_location
attacker_robot_id
route_name
```

## Runtime Nodes

### `udp_bridge_node.py`

Bridges Webots robot data into ROS 2 topics and sends `/cmd_vel` commands back to Webots.

Typical topics:

```text
/<robot>/robot_pose
/<robot>/scan
/<robot>/odom
/<robot>/cmd_vel
```

### `map_builder_node.py`

Builds each robot's local occupancy grid from its pose and LiDAR data.

Outputs:

```text
/<robot>/local_map
/<robot>/local_confidence_map
/<robot>/current_observation_map
```

### `map_merge_node.py`

Combines local maps and shared map updates into a robot-specific shared view.

The robot's own current observation is applied last for cells that are known in `/<robot>/current_observation_map`, and only claims contradicted by that observation should be removed.

It should support:

```text
fusion_mode = log_odds
fusion_mode = mate_log_odds
fusion_mode = mate_claim_verification
```

Outputs:

```text
/<robot>/shared_live_map
/<robot>/shared_confidence_map
/<robot>/cell_state_map
```

The shared occupancy grid remains the navigation map, while the RViz `MarkerArray` overlay carries the blended per-robot colors and dispute shading.

### `trust_manager_node.py`

Maintains each robot's MATE-style trust table for other robots.

Responsibilities:

- Store lifetime and recent MATE trust parameters `alpha` and `beta`.
- Apply optional MATE-style trust propagation toward the neutral prior.
- Convert verification receipts into trust pseudomeasurements.
- Apply positive and negative pseudomeasurement trust updates.
- Compute lifetime trust, recent trust, and combined trust.
- Compute trust precision, effective evidence count, and trust confidence.
- Compute caution ramp for Method 3.
- Determine quarantine status for Method 3.
- Publish trust state for logging and visualization.

### `claim_verifier_node.py`

Checks whether map claims are confirmed or contradicted by later LiDAR observations.

Responsibilities:

- Track unverified claims.
- Compare claimed cells against physical LiDAR observations.
- Publish verification receipts with `CONFIRMED`, `CONTRADICTED`, or `UNCERTAIN`.
- Compute pseudomeasurement value and confidence for trust updates.
- Send verification results and pseudomeasurements to the trust manager.
- For Method 3, trigger claim-level evidence removal or downgrading when a claim is contradicted.

### `fake_obstacle_injector_node.py`

Publishes fake occupied-cell claims for the attack scenario as `MapUpdate` messages.

Modes:

```text
manual
clicked_point
```

The first experiment should use fake obstacle insertion as the main attack. Fake clearing can be added later as a secondary safety test.

Current code status:

- A fake obstacle injector node exists now.
- It is launched by `fake_obstacle_injector.launch.py` and also included in the multi-robot launch path.
- It publishes `claim_id`, `reporting_robot_id`, `target_robot_id`, and other claim metadata on `/map_updates`.

### `navigation_adapter_node.py`

Converts final map state into a Nav2-compatible costmap or occupancy grid.

Suggested behavior:

| Cell state | Navigation behavior |
|---|---|
| Occupied | High or lethal cost |
| Clear | Normal traversal |
| Unknown | Use normal Nav2 unknown behavior |
| Suspicious | Medium cost or verification target |
| Disputed | Avoid if possible and request verification |

### `trial_logger_node.py`

Logs per-update and per-trial metrics to CSV or JSON.

Logs should include final map accuracy, false occupied rate, route success, checkpoint delay, path length increase, poisoned data removal time, trust changes, quarantine status, reroute behavior, and runtime per update.

## Config Files

Recommended configs:

```text
config/multi_robot_config.json
config/fusion_modes.yaml
config/trust_params.yaml
config/experiment_maps.yaml
config/routes.yaml
config/attack_scenarios.yaml
config/nav2_params.yaml
config/rviz/
```

Current code status:

- `config/multi_robot_config.json`, `config/nav2_params.yaml`, and multiple RViz configs exist now.
- World-specific multi-robot configs exist for `webots/worlds/simpleCorridor/` and `webots/worlds/twoRoute/`.
- `patrol_robot` includes a checkpoint autopilot fallback for patrol worlds when route commands are not available yet.
- Shared robot behavior should stay in the controller code so new worlds can reuse the same RViz, Nav2, and fake-obstacle logic without copying behavior into each `.wbt`; per-world files should only override spawn positions, route layout, robot ids, and world-specific config values.
- The saved Webots project files (`.wbproj`) are also world-specific layout data and may be reused to keep the console and other Webots panes docked the same way across worlds.
- Project code should avoid hardcoded host-specific paths, usernames, machine names, and fixed ports whenever a repo-relative path, launch argument, environment variable, or config file can provide the same value. Default values are fine, but machine-dependent settings should remain overridable.
- The YAML config set described below is still the target structure rather than the complete current set.

### `fusion_modes.yaml`

Defines the three comparison methods:

```yaml
fusion_modes:
  - log_odds
  - mate_log_odds
  - mate_claim_verification
```

### `trust_params.yaml`

Recommended parameters:

```yaml
mate_alpha0: 1.0
mate_beta0: 1.0
mate_prior_pull_omega: 0.005
mate_negative_bias: 3.0
mate_negative_threshold: 0.5
mate_min_psm_confidence: 0.2
trust_confidence_k: 10
lambda_initial_caution: 0.9
lambda_decay_gamma: 0.1
recent_window_size: 20
sensor_distance_sigma: 3.0
report_age_tau: 10.0
evidence_alpha: 0.99
occupied_evidence_cap: 10.0
free_evidence_cap: 10.0
theta_unknown_evidence: 0.5
theta_occupied: 0.7
theta_free: 0.3
theta_dispute: 0.5
theta_quarantine_trust: 0.25
theta_quarantine_confidence: 0.70
```

### `experiment_maps.yaml`

First required maps:

```yaml
maps:
  - office
  - single_hallway
  - two_path
```

Optional later maps:

```yaml
optional_maps:
  - small_maze
  - random_sandbox
```

## Simulation and Webots Setup

Webots provides the physical simulation environment. It should simulate multiple TurtleBot-style robots, LiDAR scans, robot pose and odometry, checkpoint routes, static walls and objects, fake obstacle attack locations, and different environment layouts.

ROS 2 receives data from Webots, builds local and shared maps, applies the selected fusion model, sends navigation commands through Nav2, and logs the experiment results.

## Main Simulation Data Flow

Expected final data flow:

```text
Webots world
  -> robot controller
  -> ROS bridge
  -> /scan, /odom, /robot_pose
  -> local map builder
  -> shared map update
  -> trust and verification logic
  -> shared confidence map
  -> Nav2-compatible map/costmap
  -> checkpoint navigation
  -> experiment logger
```

For fake obstacle attacks:

```text
fake obstacle injector
  -> /map_updates
  -> map fusion node
  -> temporary or weighted map claim
  -> later LiDAR observation
  -> verification receipt
  -> trust update
  -> map correction or quarantine
```

## Webots Folder

Recommended final world folders:

```text
webots/worlds/office/
webots/worlds/single_hallway/
webots/worlds/two_path/
webots/worlds/small_maze/
webots/worlds/random_sandbox/
```

Current code status:

- `webots/worlds/office/`, `webots/worlds/confusingMaze/`, `webots/worlds/sandbox/`, `webots/worlds/testRvizMap/`, `webots/worlds/testBuildingMapForRobot/`, `webots/worlds/TestFakeObstacle/`, and `webots/worlds/TestCombineRvizMap/` exist now.
- `single_hallway/`, `two_path/`, and `random_sandbox/` are still planned target worlds.

The first experiment should use the office, single-hallway, and two-path maps. The small maze is optional for later stress testing. The random sandbox can be used later for robustness testing, but it should not be part of the first required trial batch.

Each world folder should contain:

```text
<world>.wbt
amcl_map/
  map.yaml
  map.pgm
routes.yaml
attack_locations.yaml
README.md
```

## Required Maps

### Office Map

Purpose: simulate a realistic indoor environment.

The office map should include rooms, hallways, doorways or narrow passages, walls, realistic route constraints, and checkpoints that force the robot to move through different parts of the environment. This map should show whether the method works in a practical indoor navigation environment.

### Single-Hallway Map

Purpose: create a simple worst-case environment.

The single-hallway map should include one main route, a checkpoint path that requires the robot to pass through the hallway, and a fake obstacle injection point that can block the only path. This map tests whether the system can avoid being trapped by a fake obstacle.

### Two-Path Map

Purpose: test rerouting behavior.

The two-path map should include one short path, one longer alternate path, a checkpoint route where the short path is normally preferred, and a fake obstacle injection point on the short path. This map tests whether fake map data causes unnecessary detours.

### Optional Small Maze Map

Purpose: stress test route planning and map confidence.

The maze map can be added later if the first three maps work well. It is useful for testing complex rerouting, but it can make results harder to interpret because many route choices may exist.

### Optional Random Sandbox Map

Purpose: robustness testing.

The random sandbox map can contain scattered walls and objects. It should be used only after the main experiment is stable, because random layouts may make repeated comparisons harder.

## Checkpoint Routes

Each map should define a fixed checkpoint route. Robots should follow the same route under all three models.

Each route should include:

```text
route_name
start_pose
checkpoint_order
goal_tolerance
expected_clean_path_length
expected_clean_completion_time
```

The checkpoint route is important because the project is not only about map accuracy. It is also about whether poisoned map data changes navigation behavior.

The logger should record:

```text
checkpoint_success
checkpoint_delay
time_to_finish_route
path_length_actual
path_length_increase_percent
reroute_taken
stuck_detected
collision_detected
```

## Fake Obstacle Attack Setup

The first version of the project should focus on fake obstacle insertion.

A fake obstacle attack means a compromised robot reports that a free cell is occupied. Example:

```json
{
  "claim_id": "trial_001_claim_0001",
  "reporting_robot_id": "robot_1",
  "target_robot_id": "robot_2",
  "cell_x": 10,
  "cell_y": 12,
  "reported_state": "OCCUPIED",
  "attack_type": "fake_obstacle"
}
```

Attack locations should be map-specific:

| Map | Suggested fake obstacle location |
|---|---|
| Office | Hallway, doorway, or common route segment |
| Single hallway | Middle of the only path |
| Two-path | Short path between checkpoints |

The same attack location should be used across all three fusion models for fair comparison.

## Fake Clearing as Future Work

Fake clearing is when a compromised robot reports that a real obstacle is clear. This is an important safety attack because it can cause collisions or unsafe paths.

However, fake clearing should be treated as a later or secondary experiment. The first version should focus on fake obstacles because they are easier to visualize, easier to inject, and directly affect rerouting and checkpoint delay.

If fake clearing is added later, measure:

```text
false free rate
collision rate
minimum clearance
unsafe path attempts
```

## Fusion Mode Behavior in Webots Trials

### `log_odds`

All robot map reports are treated equally. This behaves like full or equal trust in other robots.

Expected behavior under fake obstacles:

- Fake obstacles may be accepted into the map.
- Robot may reroute unnecessarily.
- Robot may get stuck in the single-hallway map.
- Trust-specific metrics are not available.

### `mate_log_odds`

Incoming map reports are weighted by MATE-style robot trust.

Expected behavior under fake obstacles:

- Each observer maintains `Beta(alpha_ij, beta_ij)` trust PDFs for reporters.
- Optional MATE-style trust propagation pulls stale trust back toward the neutral prior.
- Verification receipts become pseudomeasurements `(v_ijc, c_ijc)`.
- Confirmed claims increase trust and contradicted claims reduce trust, with optional negative bias.
- Log-odds updates are weighted only by the MATE trust mean `T_ij`.
- This mode does not use claim-level `Q`, `R`, caution ramp, suspicious/disputed states, evidence removal, or quarantine.

### `mate_claim_verification`

Incoming map reports are weighted by MATE trust plus claim-level map-poisoning defense terms.

Expected behavior under fake obstacles:

- New claims should start with limited confidence.
- Fake obstacles should be temporary unless verified.
- Later LiDAR observations should produce verification receipts and MATE pseudomeasurements.
- Contradicted claims should reduce attacker trust.
- Contradicted claim contributions should be removed, downgraded, marked suspicious, or disputed.
- The map should maintain occupied/free evidence layers.
- Quarantine may eventually stop a malicious robot from influencing navigation.

## Route Config Example

Example `routes.yaml`:

```yaml
routes:
  main_checkpoint_route:
    start_pose:
      x: 0.0
      y: 0.0
      yaw: 0.0
    checkpoints:
      - {x: 1.0, y: 0.0}
      - {x: 3.0, y: 0.0}
      - {x: 5.0, y: 1.0}
    expected_clean_path_length: 6.5
    expected_clean_completion_time: 45.0
```

The expected clean path length should be measured from a no-attack run or computed from the known map.

## Attack Config Example

Example `attack_locations.yaml`:

```yaml
attacks:
  fake_obstacle_middle_path:
    attack_type: fake_obstacle
    attacker_robot_id: robot_1
    target_robot_id: robot_2
    reported_state: OCCUPIED
    world_position:
      x: 3.0
      y: 0.0
    start_time_sec: 20.0
    duration_sec: 10.0
```

The attack should occur at the same time and place for every fusion mode.

## RViz Visualization

RViz should show:

```text
static map
robot pose
LiDAR scan
local map
shared live map
shared confidence map
cell state map
fake obstacle marker
verification markers
checkpoint route
```

The normal shared `OccupancyGrid` stays separate for navigation. The colored semantic overlay should be a `MarkerArray` display on top of it.

Useful visual distinction:

| Item | Suggested display |
|---|---|
| Real occupied cells | Normal occupancy/costmap color |
| Fake obstacle claim | Marker or highlighted cell |
| Blended multi-robot cell | Mixed robot hue in the overlay |
| Suspicious cell | Distinct overlay color |
| Disputed cell | Distinct overlay color |
| Verified clear cell | Confidence overlay or marker |
| Quarantined source claim | Marker only, not lethal cost |

## Controller Contract

The Webots robot controllers should remain stable during the experiment. Most research logic should live on the ROS side.

The robot controller should provide:

```text
pose
odometry
LiDAR scan
wheel control
checkpoint events if used
```

The ROS side should handle:

```text
map fusion
trust updates
claim verification
fake obstacle injection
navigation adaptation
logging
```

This separation makes the comparison fair because the robot physics and controller behavior stay the same across all three fusion models.

## Scripts

Recommended scripts:

```text
scripts/verify.sh
scripts/quick_test.sh
scripts/runTestFakeObstacle.sh
scripts/run_all_trials.sh
scripts/run_single_trial.sh
scripts/analyze_results.py
scripts/plot_results.py
scripts/watch_topic.sh
scripts/echo_topic_once.sh
```

Current code status:

- `scripts/verify.sh`, `scripts/quick_test.sh`, `scripts/runOffice.sh`, `scripts/runConfusingMaze.sh`, `scripts/runSandbox.sh`, `scripts/runTestBuildingMapForRobot.sh`, `scripts/runTestFakeObstacle.sh`, and `scripts/runTestCombineRvizMap.sh` exist now.
- The remaining script names listed above are still proposed helpers, except `runTestFakeObstacle.sh`, which is the current fake-obstacle smoke test.

### `run_single_trial.sh`

Runs one trial:

```bash
bash scripts/run_single_trial.sh \
  --fusion-mode mate_claim_verification \
  --map office \
  --route main_checkpoint_route \
  --attack fake_obstacle_middle_path \
  --trial-id office_twv_001
```

### `run_all_trials.sh`

Runs all combinations:

```text
3 models x 3 maps x 3-5 trials
```

The optional maze map should not be part of the first required trial batch unless time allows.

## Running the First Experiment Batch

The first batch should run:

```text
3 fusion models x 3 maps x 3-5 trials
```

Recommended maps:

```text
office
single_hallway
two_path
```

Recommended attack:

```text
fake_obstacle
```

Optional later expansion:

```text
small_maze
fake_clearing
late_attacker
noisy_honest_robot
```

## Data Flow Summary

Final expected data flow:

```text
Webots world
  -> robot controller
  -> ROS bridge
  -> robot pose, scan, odom
  -> local map builder
  -> map update publisher
  -> map merge node
  -> trust manager
  -> claim verifier
  -> shared confidence map
  -> navigation adapter
  -> Nav2 route/checkpoint behavior
  -> trial logger
```

For fake obstacle attacks:

```text
fake obstacle injector
  -> /map_updates
  -> map merge node
  -> low-confidence or trust-weighted claim
  -> later LiDAR verification
  -> verification receipt
  -> trust update
  -> map correction or quarantine
```

## Results Folder

Recommended results structure:

```text
results/
  raw/
  processed/
  plots/
  tables/
```

Important output files:

```text
results/raw/map_updates.csv
results/raw/navigation_trials.csv
results/raw/trust_history.csv
results/processed/summary_by_model.csv
results/processed/summary_by_map.csv
results/tables/main_comparison.md
results/plots/map_accuracy.png
results/plots/path_length_increase.png
results/plots/poisoned_data_removal_time.png
```

## Main Metrics Collected from Trials

The trials should support these measurements:

| Metric | Description |
|---|---|
| Final map accuracy | Accuracy of the final map compared with ground truth. |
| False occupied rate | How many fake obstacle cells remain accepted as occupied. |
| Checkpoint success | Whether the robot completes the route. |
| Checkpoint delay | How much longer the attacked trial takes compared with clean navigation. |
| Path length increase | How much farther the robot travels because of fake obstacles. |
| Reroute behavior | Whether the robot changes route, especially in the two-path map. |
| Time to remove poisoned data | Time until a fake obstacle is cleared, downgraded, or marked suspicious. |
| Runtime per update | Computational cost of each fusion method. |

Trust-specific metrics for Models 2 and 3:

```text
trust score over time
trust confidence over time
attacker detection delay
quarantine time
false punishment rate
```

## Main Comparison Tables

Main navigation and map table:

| Method | Final Map Accuracy | False Occupied Rate | Checkpoint Success | Checkpoint Delay | Path Increase | Runtime |
|---|---:|---:|---:|---:|---:|---:|
| Log-odds | | | | | | |
| MATE-weighted log-odds | | | | | | |
| MATE claim verification | | | | | | |

Trust and attack-defense table:

| Method | Poisoned Data Removal Time | Detection Delay | False Punishment Rate | Quarantine Recall |
|---|---:|---:|---:|---:|
| Log-odds | N/A | N/A | N/A | N/A |
| MATE-weighted log-odds | | | | |
| MATE claim verification | | | | |

## Success Criteria

The proposed method is successful if it supports the project claim by showing:

- Higher final map accuracy than the baselines.
- Lower false occupied rate than the baselines.
- Less unnecessary rerouting than the baselines.
- Shorter checkpoint delay under fake obstacle attacks.
- Faster removal of poisoned map data.
- Reasonable runtime per map update.
- Better attacker detection and map recovery than MATE robot trust alone.

The most important comparison is not only whether the map looks better, but whether the robot navigates better under attack.

## Design Rule

Keep Webots controllers stable when possible. The main experimental differences should be implemented in ROS-side fusion, trust, verification, and logging nodes. This keeps the comparison fair because each model uses the same robot movement, maps, sensors, routes, and attacks.
