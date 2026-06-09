# Robot Map Poisoning Defense

A ROS2-based cooperative multi-robot cybersecurity project focused on detecting and containing robot-to-robot map poisoning.

Robots share map and route information with each other. One robot may become compromised and send false map updates, such as fake obstacles, blocked routes, or false route clearing messages. Defense nodes will evaluate shared updates using trust scores, confidence scores, and multi-robot verification.

## Project Goals

- Simulate robot-to-robot map sharing
- Inject malicious or false map updates
- Detect suspicious robot behavior
- Reduce trust in compromised robots
- Quarantine suspicious robots
- Use an LLM security agent to explain suspicious behavior and recommend actions

## Current Status

The repository currently contains a ROS2 Jazzy workspace with the following packages:

```text
attack_node
defense_node
llm_security_agent
map_sharing_msgs
robot_patrol_node