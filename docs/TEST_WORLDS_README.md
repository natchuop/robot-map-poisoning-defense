# Bright brown map-poisoning test worlds

This bundle contains only two test worlds:

1. `simpleCorridor/simple_corridor.wbt`
   - One straight corridor.
   - Fake-obstacle report target: `(0.0, 0.0)`.
   - Run it with `bash scripts/runSimpleCorridor.sh`.
   - Uses the `patrol_robot` controller and auto-starts patrol by default.

2. `twoRoute/two_route.wbt`
   - Exactly two paths between start and goal:
     - upper, shorter route
     - lower, longer backup route
   - Fake-obstacle report target on the upper path: `(0.0, 1.05)`.
   - Run it with `bash scripts/runTwoRoute.sh`.
   - Uses the `patrol_robot` controller and auto-starts patrol by default.

Both floors use the included warm brown checkerboard:
`webots/worlds/textures/bright_brown_checkerboard.png`

The green, red, and yellow squares are non-physical visual markers. Robots can drive through them.
