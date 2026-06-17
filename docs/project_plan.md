# Robot Map Poisoning Defense Project Plan

This is the main context file for future AI work on the project.

## Current Working Demo

- TurtleBot3 Burger runs in Webots
- 360-degree LDS-01 LiDAR works
- GPS works
- IMU works
- Current controller is C and does obstacle avoidance
- Controller can print robot `x`, `y`, and heading (`yaw`)

## Project Goal

Study robot-to-robot map poisoning in a Webots + ROS 2 + Python system.

Core idea:

- Multiple robots map the same environment
- Robots share map updates through ROS 2
- One robot becomes compromised and publishes fake map data
- We test trust strategies that reduce the spread of poisoned map updates

This project is about:

- robot-to-robot communication
- shared mapping
- trust decay
- verification
- quarantine

It is not about:

- object classification
- machine learning-based detection

## Main Research Question

How do different trust management strategies affect the resilience of a multi-robot mapping system against robot-to-robot map poisoning attacks?

## Key Design Decisions

- Use GPS + IMU + LiDAR as the localization shortcut
- Use fixed waypoint patrol routes, not random exploration
- Prefer occupancy grids over object classification
- Keep map sharing simple and ROS 2-based

## Mapping Choice

Use occupancy grids for the final system.

Cell states:

- unknown
- free
- occupied

Why:

- easy to share between robots
- easy to compare against ground truth
- matches the poisoning attack well

Point clouds are useful for debugging coordinate transforms early, but occupancy grids are the final target.

## Trust Model

Use differential trust:

- trust increases a little when a report is verified true
- trust decreases a lot when a report is proven false
- trust decays over time
- robots below a threshold can be quarantined or ignored

Example behavior:

```text
Robot A reports obstacle at (10, 12)
Robot B later scans the area
If obstacle exists: small trust gain
If obstacle does not exist: large trust loss
If trust falls below threshold: quarantine or ignore Robot A
```

Suggested starting values:

- true report: `+1`
- false report: `-10`
- decay: `-0.1` per interval or `trust *= 0.99`
- quarantine threshold: `30`

## Simulation Setup

Use a Webots world known to the researcher but unknown to the robots.

The world should have:

- walls
- open pathways
- several obstacles
- overlapping patrol routes
- enough space for multiple robots

Robots start with no map knowledge and discover the world with LiDAR.

## Robot Motion

Use predefined waypoint patrol routes.

Example:

```text
Robot 1: A -> B -> C -> D
Robot 2: D -> E -> F -> A
Robot 3: B -> F -> C -> E
```

Routes should overlap so robots can later verify each other's reports.

## Webots + ROS 2 Architecture

Webots provides:

- robot models
- physics
- LiDAR
- movement
- simulation world

ROS 2 provides:

- publishers and subscribers
- robot-to-robot communication
- shared map updates
- control messages

Each robot should have its own Python controller and ROS 2 node.

Recommended flow:

```text
Webots Robot -> Python Controller -> ROS 2 Node -> Topics/Services -> Other Robots
```

## ROS 2 Data Plan

Start with simple coordinate messages before wiring everything into full mapping.

Useful message shape:

```json
{
  "x": 5,
  "y": 8,
  "z": 0,
  "occupied": true,
  "reporting_robot": "robot_2"
}
```

Useful topics:

- `/scan`
- `/robot_pose`
- `/map`
- `/map_updates`
- `/control/start`
- `/control/attack_trigger`

Useful commands/services:

- start simulation
- stop simulation
- trigger malicious behavior
- publish fake object
- reset trust
- reset maps
- switch trial mode

## LiDAR Mapping Plan

LiDAR gives distance readings, not objects.

Convert scan points to world coordinates:

```text
x = robot_x + distance * cos(robot_heading + lidar_angle)
y = robot_y + distance * sin(robot_heading + lidar_angle)
```

Use these readings to fill an occupancy grid.

## Attack Scenario

One robot is compromised and publishes fake occupied cells or blocked paths.

Example:

```json
{
  "cell_x": 10,
  "cell_y": 12,
  "occupied": true,
  "reporting_robot": "robot_1"
}
```

The real world may be empty at that cell.

Later, an honest robot revisits the area, detects the mismatch, and reduces trust in the reporter.

## Experimental Groups

Keep the world, routes, attack timing, and robot start positions constant.

Only the trust strategy changes.

Compare:

1. No trust
2. Basic trust
3. Differential trust with decay
4. Differential trust with decay + quarantine

Purpose of each:

- no trust: baseline damage from poisoning
- basic trust: simple verification without decay
- differential trust: stale trust matters, false reports hurt more
- quarantine: stop continued poisoning once the attacker is detected

## Metrics

Track:

- fake objects accepted
- time to identify the compromised robot
- map accuracy over time
- trust score over time
- optional navigation impact

Map accuracy should compare each robot’s map with the Webots ground truth.

## Implementation Roadmap

1. Keep robots moving in Webots
2. Get ROS 2 communication working
3. Get LiDAR mapping working
4. Build a local occupancy grid
5. Share map updates through ROS 2
6. Merge maps from multiple robots
7. Add map poisoning
8. Add verification of received reports
9. Add trust scores, decay, and quarantine

## Current ROS 2 / RViz / Nav2 Direction

Recommended path:

- ROS 2 Jazzy in Docker
- `rviz2`
- `sensor_msgs`
- `geometry_msgs`
- `webots_ros2`
- Known static map
- AMCL localization
- Nav2 checkpoint navigation

Run Webots from a ROS-sourced terminal.

Current mapping loop:

- publish `/scan` as `sensor_msgs/LaserScan`
- publish `/robot_pose` as `geometry_msgs/Pose2D`
- build `/map` as `nav_msgs/OccupancyGrid`
- visualize in RViz with fixed frame `map`

Navigation direction:

- Use a known occupancy-grid map for the planned Webots world
- Put checkpoint poses on that known map
- Use AMCL to localize each robot on the known map from LiDAR plus robot motion
- Use Nav2 to move robots from checkpoint to checkpoint
- Use RViz2 for visualization, initial pose setting, goal debugging, maps, paths, TF, and costmaps
- Do not use SLAM for the main implementation path

Why no SLAM for the main path:

- The experiment uses a known map and predefined patrol checkpoints
- SLAM would add map-building complexity that is not central to the map-poisoning defense question
- AMCL keeps localization separate from the trust and map-sharing experiments
- Nav2 still provides navigation either way, so SLAM is not a replacement for Nav2

## Current Controller / Bridge Notes

Preferred controller for ROS integration:

`webots/controllers/testRvizMap/testRvizMap.py`

The Webots controller sends packets over TCP by default; the ROS bridge listens on both UDP and TCP on port `5005`.

It:

- reads GPS, IMU, and LiDAR
- uses a Braitenberg-style avoidance rule
- sends pose and scan data to Docker over TCP on port `5005`
- defaults to `172.28.64.1` first, then `127.0.0.1`, for the host bridge
- also accepts UDP on port `5005` as fallback/debug

World file:

`webots/worlds/testRvizMap/turtlebot3_burger.wbt`

If the robot frame or sensor offset differs, pass mapper parameters such as:

```bash
laser_frame:=lidar
laser_x:=0.10
laser_y:=0.00
laser_yaw:=0.00
```

## First Useful AI Task

Help connect Webots to ROS 2 by publishing `/scan` and `/robot_pose`, then use the `robot_patrol_node` mapper to build and visualize an accumulating occupancy grid in RViz.
