# Notes For Future Changes

Keep this file short. It is meant to prevent the same regressions from coming back.

## Repeat Bugs To Avoid

- Do not let RViz show only the current scan or only other robots' data. Each robot must keep its own shared map view, and its own cleared space should remain visible after the scan moves on.
- Do not reapply fake obstacle injections forever. Once real LiDAR evidence clears a fake cell, the injected report must stop overriding the merged map.
- Do not collapse robot trust into one shared value. Robot 1 and Robot 2 need separate trust tables, separate weights, and separate per-robot shared views.
- Do not change the patrol bot controller when fixing shared mapping or fake injection bugs. The patrol bot should stay untouched unless the issue is clearly inside that controller.

## Launch Gotchas

- Webots and RViz can appear broken right after code changes if the ROS 2 workspace has not rebuilt cleanly or an old container is still holding ports `5005` and `5006`.
- If RViz or the overlay nodes fail to start, check the launch logs first. The common failures here were parameter type mismatches, missing helper methods, and stale topic names.
- `scripts/runTestCombineRvizMap.sh` and `scripts/runTestFakeObstacle.sh` are the best smoke tests for launch regressions.

## Fake-Obstacle Gotchas

- Fake obstacle injection must remain temporary.
- The fake marker can be visual, but the merged map must clear it once the robot's own scan proves the cell is free.
- Keep the trust values robot-specific. For the current baseline, Robot 1 trusts Robot 2 at `0.80` and Robot 2 trusts Robot 1 at `0.20`.

