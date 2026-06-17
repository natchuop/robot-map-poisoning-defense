# Webots Project

Open this world in Webots:

```text
webots/worlds/testRvizMap/turtlebot3_burger.wbt
```

Controller used by the TurtleBot3:

```text
webots/controllers/testRvizMap/testRvizMap.py
```

Webots looks for the runnable controller under the world-local path:

```text
webots/worlds/controllers/testRvizMap/testRvizMap.py
```

That wrapper forwards to the real controller code above.

The controller:

- reads GPS pose
- reads IMU yaw
- reads LDS-01 LiDAR ranges
- moves the TurtleBot3 with simple obstacle avoidance
- sends newline-delimited JSON packets to the ROS 2 bridge over TCP by default

Default bridge target:

```text
tcp://172.28.64.1:5005
```

You can override targets before launching Webots:

```bash
WEBOTS_BRIDGE_TARGETS=172.28.64.1,127.0.0.1
WEBOTS_BRIDGE_PORT=5005
WEBOTS_BRIDGE_PROTOCOL=tcp
```

Expected robot devices:

- `gps`
- `inertial unit`
- `LDS-01`
- `LDS-01_main_motor`
- `LDS-01_secondary_motor`
- `left wheel motor`
- `right wheel motor`

Run order:

1. Start Docker with `docker compose run --rm --service-ports --name ros2_dev ros2`.
2. Inside Docker, run `bash scripts/start_ros2_stack.sh`.
3. Open the Webots world.
4. Press Play.
5. In another Docker shell, run `ros2 launch robot_patrol_node rviz.launch.py` to open RViz with the saved displays.

Expected Webots console output:

```text
connected to ROS bridge at tcp://172.28.64.1:5005
sent bridge packet
```
