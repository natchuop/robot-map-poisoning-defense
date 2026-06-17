# Webots Project

For AI-facing instructions on creating new worlds and wiring them into Docker, ROS 2, and RViz, see:

```text
webots/set_up_webots_world.md
```

Open this world in Webots:

```text
webots/worlds/testRvizMap/turtlebot3_burger.wbt
```

The matching known AMCL map for this world is kept beside it:

```text
webots/worlds/testRvizMap/amcl_map/arena.yaml
webots/worlds/testRvizMap/amcl_map/arena.pgm
```

`scripts/quick_test.sh` refreshes this map from the known geometry in the world: the circular arena, the wooden box positions, the map origin, and the resolution. It is not automatically parsed from the `.wbt` file, so world geometry changes should be reflected in the generator logic or the map files.

Controller used by the TurtleBot3:

```text
webots/controllers/patrol_robot/patrol_robot.py
```

The world file references it with `controller "patrol_robot"`. Keep behavior code in `webots/controllers/`; only lightweight forwarding wrappers should live under `webots/worlds/controllers/`.

Because the current worlds are nested under `webots/worlds/<world_name>/`, Webots also needs a tiny runnable wrapper here:

```text
webots/worlds/controllers/patrol_robot/patrol_robot.py
```

That wrapper only forwards to the real shared controller. Do not put behavior logic there.

For multiple worlds, keep worlds and controllers as siblings:

```text
webots/
  worlds/
    testRvizMap/
    anotherMap/
  controllers/
    patrol_robot/
```

The controller:

- reads GPS pose
- reads IMU yaw
- reads LDS-01 LiDAR ranges
- moves the TurtleBot3 with simple obstacle avoidance
- sends newline-delimited JSON packets to the ROS 2 bridge over TCP by default

For future multi-map tests, reuse `patrol_robot.py` and vary behavior with map/robot configuration instead of copying controller code per world.

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

```bash
bash scripts/quick_test.sh
```

The quick test starts Docker, builds/sources the ROS 2 workspace, launches the AMCL localization stack, opens this Webots world, waits for bridge packets and TF, then opens RViz with `amcl.rviz`.

The AMCL test publishes:

- `/robot_pose`
- `/scan`
- `/map`
- `/odom`
- `/tf`
- `/tf_static`

Expected Webots console output:

```text
connected to ROS bridge at tcp://172.28.64.1:5005
sent bridge packet
```

