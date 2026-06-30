# Verification and Experiment Validation Guide

This guide describes how to verify the final ROS 2 + Webots trust-based map-poisoning defense project and its main claim: **Can decentralized trust-weighted map fusion reduce the effects of map-poisoning attacks on multi-robot navigation and final map accuracy?**

The purpose of verification is to confirm that the simulation, bridge, mapping stack, trust logic, fake-obstacle injection, claim-level verification, Nav2 behavior, and experiment logging all work before running the full comparison.

The final experiment compares three models:

```text
1. log_odds
2. beta_log_odds
3. trust_weighted_verification
```

The first required maps are:

```text
1. office
2. single_hallway
3. two_path
```

A small maze map can be added later as an optional stress test.

## 1. Environment Verification

Run from the repository root:

```bash
bash scripts/verify.sh
```

The verification script should check:

- Docker daemon is running.
- Docker Compose config is valid.
- ROS 2 image builds successfully.
- Python source files compile.
- ROS 2 workspace builds with `colcon`.
- Required ROS 2 packages are installed.
- Nav2 packages are installed.
- AMCL is installed.
- RViz is installed.
- Webots bridge ports are available.
- Temporary mapping stack starts.
- Temporary AMCL/Nav2 stack starts.
- Core ROS topics publish data.

Expected result:

```text
All checks passed.
```

## 2. Platform Notes

On Windows:

- Use WSL Ubuntu for commands.
- Docker Desktop runs the Linux ROS 2 container.
- Webots runs as the normal Windows application.
- RViz runs inside Docker and displays through WSLg.
- Avoid PowerShell for project commands unless the script explicitly supports it.

On macOS:

- Use macOS Terminal.
- ROS 2 runs inside Docker.
- Webots runs as the macOS app.
- RViz inside Docker may require XQuartz or another X11 setup.

## 3. Core Topic Verification

The verifier or quick test should confirm that the following topics exist and publish useful data:

```text
/robot_pose
/scan
/odom
/map
/amcl_pose
/live_map
/cmd_vel
```

For multi-robot experiments, also check robot-specific topics:

```text
/robot_1/robot_pose
/robot_1/scan
/robot_1/odom
/robot_1/shared_live_map
/robot_1/shared_confidence_map
/robot_1/cell_state_map

/robot_2/robot_pose
/robot_2/scan
/robot_2/odom
/robot_2/shared_live_map
/robot_2/shared_confidence_map
/robot_2/cell_state_map
```

Trust and verification topics should include:

```text
/map_updates
/verification_receipts
/trust_states
/trial_status
```

Useful commands:

```bash
ros2 topic list
ros2 topic echo /map_updates --once
ros2 topic echo /verification_receipts --once
ros2 topic echo /trust_states --once
ros2 topic info /robot_1/shared_live_map -v
ros2 topic info /robot_2/shared_live_map -v
```

If commands are run inside Docker, first source the ROS environment:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 4. Quick GUI Smoke Test

Run:

```bash
bash scripts/quick_test.sh
```

This should launch a basic Webots + ROS 2 + RViz path and confirm that:

- Webots starts.
- The robot publishes pose and scan data.
- ROS bridge receives robot data.
- RViz displays the map and robot.
- Nav2 or checkpoint routing can command movement.
- The robot can move through at least one checkpoint or route segment.

This test is not the full experiment. It only proves the basic simulation and visualization path works.

## 5. Fake Obstacle Injection Verification

Run the fake obstacle experiment shell:

```bash
bash scripts/run_fake_obstacle_experiment.sh --fusion-mode log_odds --map single_hallway --trial-id smoke_log_odds
```

Then repeat with:

```bash
bash scripts/run_fake_obstacle_experiment.sh --fusion-mode beta_log_odds --map single_hallway --trial-id smoke_beta
bash scripts/run_fake_obstacle_experiment.sh --fusion-mode trust_weighted_verification --map single_hallway --trial-id smoke_trust
```

Expected behavior:

- The fake obstacle injector publishes a `/map_updates` message.
- The victim robot receives the fake occupied-cell claim.
- The map changes differently depending on the selected fusion mode.
- In `log_odds`, the fake obstacle should have the strongest direct effect.
- In `beta_log_odds`, the effect should depend on the attacker's robot trust score.
- In `trust_weighted_verification`, the fake obstacle should initially have limited influence and later be cleared, downgraded, or marked suspicious after verification.

## 6. Fusion Mode Verification

Each fusion mode should be tested with the same attack location and route.

### `log_odds`

Expected checks:

- Standard occupancy update is used.
- No robot trust is applied.
- No quarantine is applied.
- Trust-specific metrics are logged as N/A or unused.

### `beta_log_odds`

Expected checks:

- Robot trust is computed using correct and false report counts.
- Log-odds update is weighted by robot trust.
- Trust updates after verification receipts.
- No claim-level `Q`, `R`, caution ramp, suspicious state, or disputed state is required.

### `trust_weighted_verification`

Expected checks:

- Robot trust is computed.
- Trust confidence is computed.
- Recent trust or combined trust is available.
- Caution ramp is applied.
- Report quality is computed.
- Verification confidence is computed.
- Occupied and free evidence are updated separately.
- Cell state can become unknown, occupied, clear, suspicious, or disputed.
- Quarantine can trigger when trust is low and confidence is high.

## 7. Claim Verification Receipt Checks

A fake obstacle claim should receive a claim ID and be stored as an unverified claim.

When another robot physically observes the same cell, the system should publish a verification receipt:

```text
claim_id
reporting_robot_id
verifying_robot_id
cell_x
cell_y
original_claim_type
verification_result
verification_time
verifier_pose
```

Expected verification results:

- `CONFIRMED` if LiDAR supports the claimed obstacle.
- `CONTRADICTED` if LiDAR shows the claimed obstacle is fake.
- `UNCERTAIN` if the robot could not reliably observe the cell.

Trust counts should update only when the result is confirmed or contradicted. Uncertain results should not punish the reporting robot.

## 8. Map Accuracy Verification

For each map, define ground-truth occupied and free cells from the known map or simulation layout.

At the end of each trial, compute:

```text
final_map_accuracy = correctly_classified_cells / total_evaluated_cells
```

Also compute:

```text
false_occupied_rate = ground_truth_free_cells_predicted_occupied / ground_truth_free_cells
```

For the first version of the project, false occupied rate is more important than false free rate because the main attack is fake obstacle insertion.

Fake clearing can be added later, where false free rate becomes a primary safety metric.

## 9. Navigation Verification

Each map should have a checkpoint route. The robot should attempt the same route under all three fusion modes.

For each trial, verify that the logger records:

```text
checkpoint_success
route_completed
time_to_finish_route
path_length_actual
path_length_clean_reference
path_length_increase_percent
reroute_taken
stuck_detected
collision_detected
```

Navigation is central to the project. The system should not only produce a better map; it should also reduce unnecessary rerouting, route delays, and stuck behavior caused by fake obstacles.

## 10. Trial Logging Verification

Each trial should generate at least three logs:

```text
map_updates.csv
navigation_trials.csv
trust_history.csv
```

Recommended per-update fields:

```text
timestamp
trial_id
fusion_mode
map_name
robot_id
cell_x
cell_y
reported_state
ground_truth_state
is_attack_report
claim_id
P_occ_after
cell_state_after
runtime_ms
```

Recommended trust fields:

```text
timestamp
trial_id
observer_robot_id
reporting_robot_id
trust_lifetime
trust_recent
trust_combined
trust_confidence
caution_lambda
update_ramp
quarantined
```

Recommended navigation fields:

```text
trial_id
fusion_mode
map_name
route_name
attack_location
checkpoint_success
route_completed
time_to_finish_route
path_length_actual
path_length_increase_percent
reroute_taken
final_map_accuracy
final_false_occupied_rate
runtime_per_update
```

## 11. Full Experiment Verification

After the smoke tests pass, run the first full experiment batch:

```bash
bash scripts/run_all_trials.sh
```

The first batch should cover:

```text
3 models x 3 maps x 3-5 trials
```

Models:

```text
log_odds
beta_log_odds
trust_weighted_verification
```

Maps:

```text
office
single_hallway
two_path
```

Attack:

```text
fake_obstacle
```

The maze map is optional and should be added only after the first three maps produce clean results.

## 12. Results Validation

After running the trials, the analysis script should create summary tables by model and by map.

Main comparison table:

| Method | Final Map Accuracy | False Occupied Rate | Checkpoint Success | Checkpoint Delay | Path Increase | Runtime |
|---|---:|---:|---:|---:|---:|---:|
| Log-odds | | | | | | |
| Beta + log-odds | | | | | | |
| Trust-weighted verification | | | | | | |

Trust and attack-defense table:

| Method | Poisoned Data Removal Time | Detection Delay | False Punishment Rate | Quarantine Recall |
|---|---:|---:|---:|---:|
| Log-odds | N/A | N/A | N/A | N/A |
| Beta + log-odds | | | | |
| Trust-weighted verification | | | | |

The proposed defense is successful if it supports the main claim by improving final map accuracy, lowering false occupied rate, reducing checkpoint delay, lowering path length increase, removing poisoned data faster, and remaining computationally practical.

## 13. Common Failure Modes

Check these issues first if verification fails:

- Docker is not running.
- Old containers are still holding bridge ports.
- ROS workspace was not rebuilt after message changes.
- Webots is sending to the wrong bridge port.
- RViz is showing an old topic name.
- `/map_updates` is not publishing.
- Claim IDs are missing or not unique.
- Verification receipts are not being published.
- Trust tables are accidentally shared globally instead of being robot-specific.
- Fake obstacle claims never expire or never get contradicted by LiDAR.
- Ground-truth map alignment is wrong, causing incorrect map accuracy calculations.

## 14. Minimal Passing Criteria

Before writing final results, the project should satisfy:

1. All three fusion modes run on at least one map.
2. Fake obstacle injection works.
3. The robot can follow checkpoints.
4. Map updates are logged.
5. Navigation metrics are logged.
6. Trust values are logged for Models 2 and 3.
7. Verification receipts are logged for Model 3.
8. Final map accuracy and false occupied rate are computed.
9. Path length increase and checkpoint delay are computed.
10. The same attack can be replayed across all three models.
