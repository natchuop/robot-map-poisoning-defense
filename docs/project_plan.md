# Revised Project Plan: Trust-Based Map Confidence for Robot-to-Robot Map Poisoning Defense

## 1. Project Goal

The goal of this project is to create a trust-based defense system for multi-robot mapping. In the simulation, multiple robots share map updates with each other while exploring or patrolling the same environment. One robot may become compromised and publish fake map data, such as a fake obstacle or a fake blocked path.

This project focuses on building one custom system. The system uses robot trust scores to decide how much confidence should be given to map updates from each robot.

The main idea is:

```text
robot trust -> trust confidence -> map cell confidence -> navigation decision
```

If a trusted robot reports an obstacle, the map cell becomes more likely to be treated as occupied. If an untrusted robot reports an obstacle, the map cell receives little confidence and may be marked as suspicious until another robot verifies it.

## 2. Research Question

How can robot trust and trust confidence be used to update confidence in shared map coordinates so that robots can reduce the effect of fake obstacle injection attacks?

A simpler version:

How can robots decide whether a shared obstacle report is real, fake, or uncertain?

## 3. Three Important Values

This project uses three related but different values.

### 1. Robot Trust

Robot trust means:

```text
How reliable do I think this robot is?
```

For example:

```text
Robot 1 trust in Robot 2 = 0.85
Robot 1 trust in Robot 3 = 0.30
```

A high trust score means the robot has usually sent correct map information. A low trust score means the robot has sent false, suspicious, or inconsistent information.

Trust changes when robots verify each other’s reports.

Example:

```text
Robot 3 reports an obstacle.
Robot 1 later scans that location.

If the obstacle is real:
    Robot 3 trust increases.

If the obstacle is fake:
    Robot 3 trust decreases.
```

### 2. Trust Confidence

Trust confidence means:

```text
How sure am I that this trust score is accurate?
```

This is different from the trust score itself.

Two robots can have the same trust score but different trust confidence.

Example:

```text
Robot 2:
trust = 0.90
trust confidence = 0.15

Robot 4:
trust = 0.90
trust confidence = 0.95

```

Both robots currently look trustworthy, but they should not be treated the same.

Robot 2 may have only sent 2 reports, and both happened to be correct. Its trust score looks high, but there is not much evidence yet. So its trust confidence is low.

Robot 4 may have sent 100 reports, and 90 were correct. Its trust score is also high, but now the system has a lot of evidence. So its trust confidence is high.

This matters because a robot should not become highly influential after only a few correct reports. A malicious robot could behave honestly at first, gain trust too quickly, and then start injecting fake obstacles.

Trust confidence prevents that by asking:

```text
Do I trust this robot?
How much evidence do I have to support that trust?
```

### 3. Map Cell Confidence

Map cell confidence means:

```text
How sure am I that this specific map coordinate is occupied or clear?
```

For example:

```text
Cell (10, 12)
occupied confidence = 0.75
```

This means the system is fairly confident that something is located at that coordinate.

Map confidence is updated using robot trust and trust confidence. A report from a high-trust robot with high trust confidence affects the map strongly. A report from a high-trust robot with low trust confidence affects the map only slightly.

## 4. How the Three Values Work Together

When a robot receives a map update, it should not immediately accept it.

Instead, it should check:

```text
Who sent this update?
How much do I trust that robot?
How confident am I in that trust score?
How much should this update affect the map cell?
```

A simple formula is:

```text
update_weight = robot_trust * trust_confidence
```

Example 1:

```text
Robot 3 trust = 0.90
Trust confidence = 0.10

update_weight = 0.90 * 0.10 = 0.09
```

Even though Robot 3 has high trust, its report only has a small effect because there is not enough evidence yet.

Example 2:

```text
Robot 4 trust = 0.90
Trust confidence = 0.90

update_weight = 0.90 * 0.90 = 0.81
```

Robot 4’s report has a strong effect because the system both trusts Robot 4 and has enough evidence to support that trust.

This helps the system avoid trusting robots too quickly.

## 5. System Overview

The system has two main layers.

### Robot Trust Layer

Each robot keeps a trust table for the other robots.

Example:


| Reporting Robot | Trust Score | Trust Confidence | Status      |
| --------------- | ----------- | ---------------- | ----------- |
| Robot 2         | 0.85        | 0.80             | Trusted     |
| Robot 3         | 0.45        | 0.30             | Uncertain   |
| Robot 4         | 0.20        | 0.85             | Quarantined |


The trust score says how reliable the robot seems.

The trust confidence says how much evidence supports that trust score.

The status tells the system how much influence that robot should have.

### Map Confidence Layer

Each map coordinate or occupancy-grid cell has confidence values.

Example:

```text
Cell (10, 12)
occupied evidence = 4.2
clear evidence = 1.1
occupied confidence = high
state = occupied
```

The cell can be classified as:


| State      | Meaning                           |
| ---------- | --------------------------------- |
| Unknown    | Not enough evidence               |
| Occupied   | Likely a real obstacle            |
| Clear      | Likely empty                      |
| Suspicious | Possible fake or uncertain object |
| Disputed   | Robots disagree about the cell    |


## 6. Attack Scenario

One robot becomes compromised and sends fake map updates.

Example:

```json
{
  "cell_x": 10,
  "cell_y": 12,
  "occupied": true,
  "reporting_robot": "robot_3"
}
```

This means Robot 3 is claiming that there is an obstacle at cell `(10, 12)`, even though the real world may be empty there.

The other robots do not automatically accept the update. Instead, they use Robot 3’s trust score and trust confidence to decide how much the report should affect the map cell.

## 7. Trust and Confidence Logic

When a robot receives a map update, it calculates how much weight to give the update.

Simple formula:

```text
update_weight = robot_trust * trust_confidence
```

A slightly more detailed version could be:

```text
update_weight = robot_trust * trust_confidence * report_quality
```

`report_quality` could include things like whether the reported cell was close enough to the robot’s LiDAR range, whether the report is recent, or whether the report conflicts with already verified map data.

If the report says a cell is occupied:

```text
occupied_evidence += update_weight
```

If the report says a cell is clear:

```text
clear_evidence += update_weight
```

Then the cell’s final state depends on the balance between occupied evidence and clear evidence.

## 8. Verification

Robots verify map updates by physically scanning the reported area with LiDAR.

Example:

1. Robot 3 reports an obstacle at `(10, 12)`.
2. Robot 1 later drives near `(10, 12)`.
3. Robot 1 scans the area with LiDAR.
4. If the obstacle exists, Robot 3’s trust increases.
5. If the obstacle does not exist, Robot 3’s trust decreases.
6. Either way, the amount of evidence about Robot 3 increases, so Robot 3’s trust confidence also increases.

This last part is important:

```text
Correct report:
    trust increases
    trust confidence increases

False report:
    trust decreases
    trust confidence increases
```

Trust confidence increases in both cases because the system has learned more about that robot.

A false report does not mean the system has less confidence. It means the system is becoming more confident that the robot is unreliable.

## 9. Example of Trust Confidence Changing

At the start:

```text
Robot 3 trust = 0.50
Robot 3 trust confidence = 0.00
```

The system is neutral because it has no evidence.

After 2 correct reports:

```text
Robot 3 trust = 0.90
Robot 3 trust confidence = 0.10
```

Robot 3 looks good, but the system is not very sure yet.

After 20 mostly correct reports:

```text
Robot 3 trust = 0.85
Robot 3 trust confidence = 0.80
```

Robot 3 is trusted and the system has enough evidence to rely on it more.

After repeated fake reports:

```text
Robot 3 trust = 0.20
Robot 3 trust confidence = 0.85
```

Robot 3 is not trusted, and now the system is confident that Robot 3 is unreliable.

## 10. Navigation Decision

The robot uses map confidence to decide what to do.


| Map confidence result                        | Robot behavior                       |
| -------------------------------------------- | ------------------------------------ |
| High confidence occupied                     | Treat as real obstacle and reroute   |
| Low confidence occupied                      | Mark suspicious and verify           |
| High confidence clear                        | Treat as clear                       |
| Disputed cell                                | Avoid depending on it until verified |
| Report from low-trust, low-confidence robot  | Add weak evidence only               |
| Report from low-trust, high-confidence robot | Treat as suspicious or ignore        |
| Report from quarantined robot                | Ignore for navigation                |


This connects the trust system directly to robot behavior. The robot is not just calculating trust scores; it is using those scores to decide whether to reroute, ignore a report, or verify the area.

## 11. Quarantine

If a robot repeatedly sends false information, it can be quarantined.

Simple rule:

```text
if trust_score < 0.25 and trust_confidence > 0.70:
    quarantine robot
```

This rule uses both trust and trust confidence.

A robot should not be quarantined just because its trust score is low after only one or two reports. The system should quarantine a robot only when it has enough evidence that the robot is unreliable.

Robot states:


| Status      | Condition                                       | Effect                             |
| ----------- | ----------------------------------------------- | ---------------------------------- |
| Trusted     | High trust and high trust confidence            | Updates have strong influence      |
| Uncertain   | Low trust confidence                            | Updates have limited influence     |
| Suspicious  | Low trust but not enough evidence to quarantine | Updates have very weak influence   |
| Quarantined | Low trust and high trust confidence             | Updates are ignored for navigation |


A quarantined robot’s map updates can still be logged for analysis, but they should not affect the shared map or route planning.

## 12. Experiment Setup

The experiment will run in Webots with ROS 2 communication.

The setup includes:

- Multiple TurtleBot robots
- LiDAR-based map observations
- Occupancy-grid map cells
- ROS 2 map update messages
- AMCL/Nav2 checkpoint navigation
- One compromised robot sending fake obstacle updates
- Honest robots verifying map updates through LiDAR

The robots should follow overlapping patrol routes so they can verify each other’s reported map cells.

## 13. Evaluation Metrics

The project will measure:


| Metric                      | Purpose                                                               |
| --------------------------- | --------------------------------------------------------------------- |
| Fake obstacles accepted     | How much poisoned data entered the map                                |
| Fake obstacles rejected     | How well the system resisted map poisoning                            |
| Time to detect attacker     | How quickly the malicious robot lost trust                            |
| Time to quarantine attacker | When the attacker stopped influencing the map                         |
| Map accuracy                | How close the robot map is to the real map                            |
| Reroute accuracy            | Whether robots rerouted for real obstacles and ignored fake ones      |
| False punishment rate       | Whether honest robots were unfairly distrusted                        |
| Trust confidence growth     | How quickly the system gained enough evidence to rely on trust scores |


## 14. Implementation Steps

1. Keep the current Webots + ROS 2 + Nav2 patrol system working.
2. Add a `/map_updates` topic for robots to share occupied or clear cells.
3. Give each robot a trust table for the other robots.
4. Give each robot a trust confidence value for every trust score.
5. Give each map cell an occupied confidence and clear confidence.
6. Weight incoming map updates using the reporting robot’s trust and trust confidence.
7. Add fake obstacle injection from one compromised robot.
8. Let honest robots verify reported cells using LiDAR.
9. Update robot trust based on correct or false reports.
10. Increase trust confidence as more reports are verified.
11. Use map confidence to decide whether to reroute, ignore, or verify.
12. Add quarantine for robots with low trust and high trust confidence.
13. Log trust scores, trust confidence, map confidence, fake objects, and navigation effects.

## 15. Final Summary

This project creates a custom trust-weighted map confidence system for defending against robot-to-robot map poisoning. The system does not blindly accept shared map updates. Instead, it asks:

1. Which robot sent this update?How much do we trust that robot?
2. How much evidence supports that trust score?
3. 
4. How confident are we that this map cell is actually occupied or clear?
5. Should the robot reroute, ignore the report, or verify it?

The key difference is:

```text
Robot trust = how reliable the robot seems.
Trust confidence = how much evidence supports that trust judgment.
Map confidence = how sure we are that a specific map cell is occupied or clear.
```

The main contribution is using robot trust and trust confidence to control confidence in map coordinates, then using map-coordinate confidence to make navigation decisions.