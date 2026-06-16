# Robot-to-Robot Map Poisoning Research Project Context

## 1\. Project Summary

This project investigates cybersecurity vulnerabilities in robot-to-robot communication, specifically how false environmental information can spread between robots through shared maps.

The system will be built using:

* Webots
* ROS2
* Python

Multiple simulated robots will use LiDAR to map an environment. Each robot will build its own local map, then share map updates with other robots using ROS2. One robot will become compromised and publish false map information, such as fake obstacles. The goal is to study how different trust-management strategies reduce the impact of these poisoned map updates.

The project focuses on robot-to-robot communication, map poisoning, trust decay, verification, and quarantine. It does not require advanced machine learning, LLM detection, or object classification.

\---

## 2\. Working Research Topic

### Working Title

**Differential Trust Decay and Quarantine Mechanisms for Mitigating Robot-to-Robot Map Poisoning Attacks**

### Main Research Question

How do different trust management strategies affect the resilience of multi-robot mapping systems against robot-to-robot map poisoning attacks?

### Core Idea

A compromised robot sends fake map updates to other robots. Honest robots decide whether to accept, reject, or later remove those updates based on trust scores, observation verification, trust decay, and quarantine rules.

\---

## 3\. Main Trust Mechanism

The chosen trust idea is:

* Trust decays over time.
* Trust increases when a robot's shared observation is verified.
* Trust decreases sharply when a robot's shared observation is proven false.
* Robots below a trust threshold are temporarily quarantined or ignored.

Example:

```text
Robot A reports obstacle at cell (10, 12)
Robot B later visits/scans that area
If obstacle exists: trust\[A] += small amount
If obstacle does not exist: trust\[A] -= large amount
If trust\[A] < threshold: ignore Robot A's future updates
```

This is called a differential trust model because trust loss is larger than trust gain.

Example trust update values:

```text
Verified true report: +1 trust
Verified false report: -10 trust
Trust decay over time: -0.1 per time interval, or trust \*= 0.99
Quarantine threshold: trust < 30
```

These values are adjustable experiment parameters.

\---

## 4\. Why This Topic Is More Specific Than Basic Trust

A basic trust system might simply say:

```text
if trust < 30:
    ignore robot
```

That is too simple by itself.

This project is more specific because it studies how trust changes over time and under attack:

* How old trust evidence becomes stale.
* How robots regain trust through verified correct observations.
* How robots lose trust when their shared map data is false.
* How quarantine affects the spread of poisoned information.
* Whether a compromised robot can be isolated quickly enough to protect the fleet.

The paper is not just about creating a trust score. It is about comparing trust-management strategies during robot-to-robot map poisoning attacks.

\---

## 5\. Simulation Environment

### Environment Design

Use a known Webots simulation map that is known to the researcher but unknown to the robots.

The environment should include:

* Walls
* Open pathways
* Several obstacles
* Enough space for multiple robots to patrol
* Locations where robots' routes overlap

The robots start with no map knowledge. They discover the environment using LiDAR.

This is valid because the researcher needs a ground-truth map to evaluate map accuracy, but the robots themselves do not know the map at the start.

\---

## 6\. Robot Movement Strategy

Use predefined waypoint patrol routes instead of random movement or frontier exploration.

### Recommended Approach

Use fixed patrol routes for the research experiments.

Example:

```text
Robot 1 route: A -> B -> C -> D
Robot 2 route: D -> E -> F -> A
Robot 3 route: B -> F -> C -> E
```

Routes should overlap so that multiple robots eventually inspect the same areas. This allows robots to verify or disprove each other's map reports.

\---

## 7\. Webots + ROS2 Integration Concept

Webots provides:

* Robot models
* Physics simulation
* LiDAR sensors
* Robot movement
* The 3D environment

ROS2 provides:

* Robot-to-robot communication
* Publishers and subscribers
* Services or commands
* Shared map update messages

Each Webots robot can have its own Python controller file.

Example:

```text
Robot1
└── robot1\_controller.py

Robot2
└── robot2\_controller.py

Robot3
└── robot3\_controller.py
```

Each Python controller can run a ROS2 node.

Each robot controller should handle:

* Moving the robot
* Reading LiDAR
* Updating its local map
* Publishing map updates
* Subscribing to other robots' map updates
* Storing received map updates
* Verifying received updates later
* Updating trust scores
* Applying quarantine rules

Conceptual structure:

```text
Webots Robot
    ↓
Python Controller
    ↓
ROS2 Node
    ↓
ROS2 Topics / Services
    ↓
Other Robots
```

\---

## 8\. LiDAR Mapping Approach

A Webots LiDAR sensor does not automatically say "this is an object." It returns distance readings at different angles.

Example:

```text
angle 0 degrees -> distance 4.2 m
angle 1 degree  -> distance 4.3 m
angle 2 degrees -> distance 4.1 m
```

These distance readings can be converted into x,y coordinates.

Basic conversion:

```text
x = robot\_x + distance \* cos(robot\_heading + lidar\_angle)
y = robot\_y + distance \* sin(robot\_heading + lidar\_angle)
```

These points represent surfaces detected by the LiDAR.

### Occupancy Grid Instead of Object Classification

Do not try to classify whole objects at first.

Instead, use an occupancy grid map.

Each grid cell can be:

* Unknown
* Free
* Occupied

Example:

```text
Cell (10, 12) = occupied
Cell (10, 13) = free
Cell (11, 12) = unknown
```

This is enough for map poisoning research. The attacker can poison the map by claiming that a grid cell is occupied when it is actually empty.

Object classification is unnecessary because the cybersecurity question is whether robots should trust shared map updates, not whether they can identify chairs, boxes, walls, etc.

\---

## 9\. ROS2 Communication Plan

Start with simple ROS2 communication before integrating everything with Webots.

### Initial ROS2 Goal

Create 3-5 ROS2 nodes that can publish and subscribe to simple coordinate data.

Example message content:

```json
{
  "x": 5,
  "y": 8,
  "z": 0,
  "occupied": true,
  "reporting\_robot": "robot\_2"
}
```

When a node receives an object or map coordinate, it should store it in memory.

### Useful ROS2 Topics

Possible topics:

```text
/map\_updates
/robot\_1/map\_updates
/robot\_2/map\_updates
/robot\_3/map\_updates
/control/start
/control/attack\_trigger
```

### Useful ROS2 Commands or Services

Possible commands:

* Start simulation
* Stop simulation
* Trigger malicious robot behavior
* Publish fake object
* Reset trust scores
* Reset maps
* Switch trial mode

\---

## 10\. Attack Scenario

One robot becomes compromised.

The compromised robot publishes fake environmental information, such as:

* Fake obstacles
* False blocked paths
* False occupied cells

Example attack message:

```json
{
  "cell\_x": 10,
  "cell\_y": 12,
  "occupied": true,
  "reporting\_robot": "robot\_1"
}
```

But in the real Webots world, cell (10, 12) is empty.

Victim robots may initially accept this false update and add it to their local maps.

Later, if an honest robot visits that area and sees no obstacle with LiDAR, it can mark the update as false and decrease trust in the reporting robot.

\---

## 11\. Experimental Groups

The project should compare multiple trials while keeping the environment, robot routes, attack timing, and robot starting positions the same.

Only the trust strategy changes between trials.

### Trial 1: No Trust System

All map updates are accepted.

Purpose:

Show the baseline damage caused by map poisoning.

### Trial 2: Basic Trust

Trust increases when reports are verified and decreases when reports are false.

No trust decay.

No quarantine.

Purpose:

Show whether basic trust helps but still allows poisoned data to spread.

### Trial 3: Differential Trust With Decay

Trust gains are small.

Trust losses are large.

Trust decays over time.

No quarantine.

Purpose:

Study whether differential trust and stale-trust decay reduce attacker influence.

### Trial 4: Differential Trust + Quarantine

Trust decays over time.

Trust increases when reports are verified.

Trust decreases sharply when reports are disproven.

Robots below a threshold are temporarily quarantined.

Purpose:

Study whether quarantine prevents continued map poisoning after the attacker is detected.

\---

## 12\. Main Metrics to Measure

### 1\. Fake Objects Accepted

Count how many fake occupied cells or fake obstacles are accepted by honest robots.

Example result table:

```text
Method                         Fake Objects Accepted
No Trust                       40
Basic Trust                    18
Differential Trust             7
Differential Trust + Quarantine 2
```

### 2\. Time to Identify the Compromised Robot

Measure how long it takes for the compromised robot's trust score to fall below the quarantine threshold.

Measured in:

* Seconds
* Simulation steps
* Number of false reports

### 3\. Map Accuracy Over Time

Compare each robot's map against the ground-truth Webots environment.

Measure:

* Correct occupied cells
* Incorrect occupied cells
* Missed obstacles
* False obstacles

### 4\. Trust Rating Per Robot

Track trust scores over time.

Useful graph:

```text
Trust score vs. simulation time
```

This should show:

* Trust decay
* Trust increases after verified reports
* Sharp trust loss after fake reports
* Quarantine point

### 5\. Navigation Impact Optional

If navigation is affected by the map, measure:

* Extra distance traveled
* Number of reroutes
* Time to finish patrol
* Delays caused by fake obstacles

This is optional because the main focus is communication trust and map poisoning.

\---

## 13\. Initial Implementation Plan

The first priority is to build the testing environment and basic robot systems. Trust should be added later.

### Step 1: Get Robots Moving in Webots

Choose a robot model that can:

* Move forward and backward
* Rotate
* Use a LiDAR sensor

Start with keyboard control, then move toward autonomous waypoint following.

Eventually, have multiple robots moving autonomously at the same time.

### Step 2: Get ROS2 Communication Working

Create 3-5 simple ROS2 nodes.

Test:

* Publishers
* Subscribers
* Sending coordinate data
* Saving received coordinate data
* Service messages or command messages

Example test data:

```text
object\_coordinate(x, y, z)
```

### Step 3: Get Webots LiDAR Mapping Working

Add a LiDAR sensor to a robot.

Make the robot rotate 360 degrees.

Place objects in the environment.

Convert LiDAR readings into x,y points.

Track seen points or occupied grid cells in a simple map.

Optional: visualize scans in RViz.

\---

## 14\. Next Six Implementation Steps After the Initial Three

### Step 4: Create a Local Occupancy Grid Map

Convert LiDAR points into occupancy grid cells.

Each robot should maintain its own local map.

Map cell states:

* Unknown
* Free
* Occupied

Deliverable:

```text
Robot explores environment -> local map gradually fills in
```

### Step 5: Share Map Updates Through ROS2

When Robot A discovers an occupied cell, it publishes that update.

Robot B and Robot C subscribe and store the update.

Deliverable:

```text
Robot A sees obstacle -> Robot B and C receive the map update
```

At this stage, accept all updates. Do not add trust yet.

### Step 6: Merge Multiple Robot Maps

Allow each robot to merge received map updates into its own local map.

Keep track of whether a cell was:

* Locally observed
* Received from another robot
* Reported by a specific robot

Deliverable:

```text
Robot 1 maps one area
Robot 2 maps another area
Both robots eventually have a more complete map
```

### Step 7: Implement Map Poisoning

Choose one robot to act as the compromised robot.

Add a malicious mode where it publishes fake occupied cells.

Deliverable:

```text
Compromised robot sends fake obstacle -> other robots add fake obstacle to maps
```

This creates the core cybersecurity attack demonstration.

### Step 8: Build a Verification System

When a robot receives a map update, store the reporter ID and cell location.

Later, when the robot physically scans that area with LiDAR, check whether the report was true or false.

Deliverable:

```text
Received obstacle report -> later verified true or false
```

This is the foundation for trust updates.

### Step 9: Implement the Trust System

Add trust scores for each robot.

Implement:

* Trust gain
* Trust loss
* Trust decay
* Quarantine threshold

Deliverable:

```text
Trust scores change over time based on map update accuracy
```

\---

## 15\. Final Experimental Workflow

Once the full system works, run the same scenario multiple times.

Keep constant:

* Same Webots world
* Same robot starting positions
* Same waypoint routes
* Same attack timing
* Same fake obstacle locations

Change only:

* Trust strategy

Compare:

1. No trust
2. Basic trust
3. Differential trust with decay
4. Differential trust with quarantine

Collect:

* Fake objects accepted
* Time to quarantine
* Map accuracy over time
* Trust scores over time
* Optional navigation impact

\---

## 16\. Expected Final Paper Output

The final paper should present:

### 1\. Problem Statement

Robot-to-robot map sharing creates a cybersecurity risk because compromised robots can publish false environmental updates that spread through the fleet.

### 2\. System Design

Describe the Webots + ROS2 multi-robot mapping system.

### 3\. Attack Model

Describe the compromised robot and the fake map updates.

### 4\. Trust Strategies

Explain the four experimental groups:

* No trust
* Basic trust
* Differential trust
* Differential trust + quarantine

### 5\. Results

Present tables and graphs:

* Fake objects accepted
* Time to identify attacker
* Map accuracy over time
* Trust score over time

### 6\. Conclusion

State whether differential trust decay and quarantine reduced the impact of robot-to-robot map poisoning compared to weaker trust strategies.

\---

## 17\. One-Sentence Project Summary

This project simulates a ROS2/Webots multi-robot mapping system where a compromised robot poisons shared map updates, then evaluates whether differential trust decay and quarantine mechanisms reduce fake map acceptance and improve map accuracy over time.

