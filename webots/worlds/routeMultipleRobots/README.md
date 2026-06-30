# routeMultipleRobots

Four-robot fake-obstacle demo built from `TestFakeObstacle.wbt`.

Launch it with:

```bash
bash scripts/routeMultipleRobots.sh
```

Robot layout:

- `robot_1` starts at the upper-right corner and keeps the existing keyboard-driven fake-obstacle behavior.
- `robot_2` starts at the lower-left corner and keeps the existing keyboard-driven fake-obstacle behavior.
- `robot_3` starts near the upper-left corner and follows `robot_3_route` autonomously with the `patrol_robot` controller.
- `robot_4` starts near the lower-right corner and follows `robot_4_route` autonomously with the `patrol_robot` controller.

The visible checkpoints are `A` through `F`, shared by both patrol routes.
