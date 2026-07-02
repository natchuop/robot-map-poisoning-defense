# Verification and Experiment Validation Guide

This guide describes how to verify the final ROS 2 + Webots trust-based map-poisoning defense project and its main claim: **Can decentralized trust-weighted map fusion reduce the effects of map-poisoning attacks on multi-robot navigation and final map accuracy?**

The purpose of verification is to confirm that the simulation, bridge, mapping stack, trust logic, fake-obstacle injection, claim-level verification, Nav2 behavior, and experiment logging all work before running the full comparison.

The final experiment compares three models:

```text
1. log_odds
2. mate_log_odds
3. mate_claim_verification
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
- Core ROS 2, Nav2, RViz, and Webots-related packages are installed in the container.
- RViz and AMCL executables are available.
- The headless AMCL/Nav2 stack starts and exposes the expected ROS graph.
- The headless multi-robot fake-obstacle stack starts and exposes the expected ROS graph.
- Bridge packets can be sent into the AMCL stack without opening Webots or RViz windows.

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

The lighter `verify.sh` checks only the essential headless connections for the AMCL/Nav2 and multi-robot stacks.

Use `bash scripts/quick_test.sh` and `bash scripts/runTestFakeObstacle.sh` for the deeper GUI and experiment-level topic checks.

Those scripts should confirm that the following topics exist and publish useful data:

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
ros2 topic info /map_updates
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

Run the fake obstacle shared-mapping demo:

```bash
bash scripts/runTestFakeObstacle.sh
```

Expected behavior:

- The fake obstacle injector publishes a `robot_patrol_msgs/msg/MapUpdate` message.
- The victim robot receives the fake occupied-cell claim.
- The map changes differently depending on the selected fusion mode.
- In `log_odds`, the fake obstacle should have the strongest direct effect.
- The claim should show up in the merge-node logs as an accepted `MapUpdate`.
- The log-odds shared map should reflect the claim instead of skipping it just because the cell was previously free.

## 6. Fusion Mode Verification

Each fusion mode should be tested with the same attack location and route.

### `log_odds`

Expected checks:

- Standard occupancy update is used.
- Every robot report is effectively treated as 100% trusted.
- No robot trust is applied.
- No quarantine is applied.
- Trust-specific metrics are logged as N/A or unused.
- Fake obstacle claims are accepted through the same log-odds path as normal shared evidence.

### `mate_log_odds`

Expected checks:

- Robot trust is represented as `Beta(alpha_ij, beta_ij)`. 
- Optional MATE-style trust propagation is applied or explicitly disabled with `omega = 0.0`.
- Verification receipts are converted into pseudomeasurements `(v_ijc, c_ijc)`.
- Confirmed claims increase `alpha`; contradicted claims increase `beta`, optionally with negative bias.
- Log-odds update is weighted only by MATE trust mean `T_ij`.
- No claim-level `Q`, `R`, caution ramp, suspicious state, disputed state, evidence removal, or quarantine is required.

### `mate_claim_verification`

Expected checks:

- MATE-style robot trust is computed.
- Optional lifetime and recent trust are available.
- Trust confidence is computed from MATE trust precision.
- Caution ramp is applied.
- Report quality is computed.
- Verification confidence is computed.
- Occupied and free evidence are updated separately.
- Claim-level evidence removal or downgrading occurs after contradiction.
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

MATE trust PDFs should update only when the result is confirmed or contradicted. A confirmed receipt creates a positive pseudomeasurement, a contradicted receipt creates a negative pseudomeasurement, and an uncertain result should not punish the reporting robot.

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
alpha_lifetime
beta_lifetime
alpha_recent
beta_recent
trust_lifetime
trust_recent
trust_combined
trust_precision
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
mate_log_odds
mate_claim_verification
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
| MATE-weighted log-odds | | | | | | |
| MATE claim verification | | | | | | |

Trust and attack-defense table:

| Method | Poisoned Data Removal Time | Detection Delay | False Punishment Rate | Quarantine Recall |
|---|---:|---:|---:|---:|
| Log-odds | N/A | N/A | N/A | N/A |
| MATE-weighted log-odds | | | | |
| MATE claim verification | | | | |

The proposed defense is successful if it supports the main claim by improving final map accuracy, lowering false occupied rate, reducing checkpoint delay, lowering path length increase, removing poisoned data faster, and remaining computationally practical.

## 13. Common Failure Modes

Check these issues first if verification fails:

- Docker is not running.
- Old containers are still holding bridge ports.
- ROS workspace was not rebuilt after message changes.
- Webots is sending to the wrong bridge port.
- RViz is showing an old topic name.
- If RViz suddenly shows a blank window or WSLg "copy mode" after a reboot, run `wsl --shutdown`, reopen Docker Desktop, and then rerun `bash scripts/quick_test.sh`.
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
