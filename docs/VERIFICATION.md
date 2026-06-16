# Docker Verification

Last updated: 2026-06-16.

---

## What `verify.sh` checks

From the repo root in WSL Ubuntu:

```bash
bash scripts/verify.sh
```

The script now verifies:

- Docker is running
- Docker image exists or can be built
- `colcon build` succeeds
- ROS 2 demo pub/sub works
- All 5 project packages are discoverable
- `webots_ros2` packages are installed
- Nav2 and localization packages are installed
- TurtleBot3 packages are installed
- Python dependencies `numpy` and `scipy` import correctly
- `rviz2` is installed

Expected summary after a healthy setup:

```text
Results: 10 passed, 0 failed
```

---

## When to run it

Run the script:

- after editing `Dockerfile`
- after a fresh `git pull`
- before pushing setup changes to GitHub
- when a teammate says their container is missing packages

If you are doing a clean rebuild because Docker is bloated or a build failed, run prune first:

```bash
docker system prune -a --volumes -f
docker builder prune -a -f
```

Then rebuild and verify:

```bash
docker compose build
bash scripts/verify.sh
```

---

## Notes

- Run all Docker commands from **WSL Ubuntu**, not PowerShell.
- `docker compose run --rm ros2` is still the right command for interactive development.
- The verification script uses `docker run` one-shot commands because they behave more reliably in scripted checks.
- `rviz2` is checked with `which rviz2` because GUI launch is not reliable in a headless container.
