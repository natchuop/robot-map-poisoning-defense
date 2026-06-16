# Webots LiDAR Mapping + ROS2 Next Steps

## Current Status

Completed:
- TurtleBot3 Burger running in Webots
- 360° LDS-01 LiDAR working
- GPS added and verified
- InertialUnit (IMU) added and verified
- Controller prints:
  - robot x position
  - robot y position
  - robot heading (yaw)

Current controller:
- C controller
- Obstacle avoidance behavior
- GPS + IMU pose available
- LiDAR scan data available

## Important Project Decision

Do NOT start with SLAM.

Use:

GPS + IMU + LiDAR

as a localization shortcut.

Reason:
The research focus is robot-to-robot map poisoning and trust management, not localization.

## Mapping Discussion

Two possible map representations:

### Point Cloud

LiDAR hit points stored as:

x,y

Advantages:
- Easy to debug
- Looks similar to the uploaded YouTube video
- Quickly verifies coordinate transforms

Pipeline:

LiDAR
-> world coordinates
-> accumulated point cloud
-> visualization

### Occupancy Grid

Cell states:

- unknown
- free
- occupied

Example:

Cell(10,12) = occupied

Advantages:
- Matches final poisoning attack design
- Easy to share between robots
- Easy to compare trust decisions

Final project should eventually use occupancy grids.

## RViz Discussion

RViz is a ROS visualization tool.

RViz can display:

### LaserScan

Live LiDAR scans.

### PointCloud2

Accumulated point clouds.

### OccupancyGrid

Black = occupied
White = free
Gray = unknown

### Robot Model

Robot pose and transforms.

## Recommended Roadmap

1. GPS + IMU working
2. LiDAR working
3. ROS2 integration
4. Publish LaserScan
5. Open RViz
6. Visualize live scans
7. Build point cloud map
8. Convert to occupancy grid
9. Share maps between robots
10. Add poisoning attack
11. Add trust system

## Python vs C

Recommendation:

Switch to Python controller when integrating ROS2.

Reason:

Python + ROS2 is significantly easier than C + ROS2.

Recommended architecture:

Webots
-> Python Controller
-> ROS2 Node
-> RViz

## ROS2 Integration Plan

Install:

- ROS2 Humble
- rviz2
- sensor_msgs
- geometry_msgs
- webots_ros2

Run Webots from a ROS-sourced terminal:

source /opt/ros/humble/setup.bash
webots

## Initial ROS Topics

Publish:

/scan
/robot_pose

Message types:

sensor_msgs/LaserScan
geometry_msgs/Pose2D

## RViz Setup

Launch:

rviz2

Set:

Fixed Frame = base_link

Add display:

LaserScan

Topic:

/scan

Result:

Live LiDAR visualization in a popup RViz window.

## Long-Term Goal

Final attack workflow:

Robot A discovers obstacle
-> publishes map update

Robot B receives update
-> stores update

Compromised robot publishes fake obstacle

Honest robots initially accept

Later:

Robot revisits area
-> verifies true/false

Trust updated:

true report = small trust gain

false report = large trust loss

Below threshold:

robot quarantined

## Immediate Next Chat Goal

Continue from:

"Help me convert my TurtleBot3 Webots controller from C to Python and integrate ROS2 so I can publish /scan and visualize the LiDAR in RViz."
