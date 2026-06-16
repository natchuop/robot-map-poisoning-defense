# Setup Guide

Everything ROS-related (ROS 2, colcon, RViz, `webots_ros2`) runs inside Docker — no local ROS install needed.

**You only need to install three things locally: Git, Docker Desktop, and Webots.**

---

## Platform differences


|                   | Windows                             | Mac                   |
| ----------------- | ----------------------------------- | --------------------- |
| Terminal to use   | **WSL Ubuntu** (not PowerShell/CMD) | Terminal or iTerm2    |
| Extra Docker step | Enable WSL Integration              | Nothing extra         |
| Git install       | Inside WSL                          | Homebrew or installer |
| Everything else   | Identical                           | Identical             |


> **Windows users:** All commands below must be run inside the **WSL Ubuntu terminal**, not PowerShell or Command Prompt. Docker commands called from PowerShell can hang silently.

---

## 0. Windows only — Install WSL2 and Ubuntu

Skip this if you're on Mac or already have WSL2 with Ubuntu.

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart your computer when prompted. Then open **Ubuntu 24.04** from the Start menu and create a Linux username and password.

> From this point on, **all commands must be run in the Ubuntu terminal** — not PowerShell, not CMD. You can find it by searching "Ubuntu" in the Start menu.

Verify:

```bash
wsl --list --verbose   # run in PowerShell to confirm Ubuntu-24.04 is running
```

---

## 1. Install Git

**Windows** — open the Ubuntu terminal from the Start menu:

```bash
sudo apt update && sudo apt install -y git
git --version
```

**Mac:**

```bash
brew install git
git --version
```

No Homebrew? Install from [git-scm.com](https://git-scm.com/).

---

## 2. Install Docker Desktop

1. Download from [docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop/)
2. Install and launch it
3. **Windows only:** Docker Desktop → Settings → Resources → WSL Integration → turn on **Ubuntu-24.04** → Apply & Restart

Verify (run in your terminal):

```bash
docker --version
docker compose version
```

---

## 3. Set up GitHub SSH

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub
```

Copy the output and add it at **GitHub → Settings → SSH and GPG keys → New SSH key**.

Test it:

```bash
ssh -T git@github.com
# Expected: "Hi username! You've successfully authenticated..."
```

---

## 4. Clone the repo

```bash
mkdir -p ~/projects && cd ~/projects
git clone git@github.com:natchuop/robot-map-poisoning-defense.git
cd robot-map-poisoning-defense
```

HTTPS alternative (no SSH key needed):

```bash
git clone https://github.com/natchuop/robot-map-poisoning-defense.git
```

---

## 5. Build the Docker image

From the **repo root** (`~/projects/robot-map-poisoning-defense`):

```bash
cd ~/projects/robot-map-poisoning-defense
docker compose build
```

This downloads Ubuntu 24.04 + ROS 2 Jazzy and installs: `colcon`, `RViz`, `webots_ros2`, demo nodes, git, and pip. **First build takes 5–15 minutes.** Subsequent builds use the cache and are fast.

---

Use plain `docker compose build` for normal work. Do **not** use `--no-cache` unless:

- a previous Docker build failed partway through
- you changed the package install layer in `Dockerfile`
- Docker cache seems corrupted

If Docker storage gets too large and you want a clean reset first:

```bash
docker system prune -a --volumes -f
docker builder prune -a -f
docker compose build
```

## 6. Enter the container and build the workspace

```bash
cd ~/projects/robot-map-poisoning-defense
docker compose run --rm ros2
```

You're now inside the container. ROS 2 is auto-sourced. Build the project packages:

```bash
colcon build
source install/setup.bash
```

Expected output:

```
Summary: 5 packages finished [~2s]
```

### If build/ or install/ are owned by root

If you see permission errors on the host after running `colcon build` inside Docker, fix ownership:

```bash
sudo chown -R $USER:$USER build install log logs
```

Or just delete and rebuild:

```bash
rm -rf build install log logs
colcon build
```

---

## 7. Verify ROS 2 pub/sub

This tests ROS 2 messaging **inside Docker**. You need two terminals, both talking to the **same** container.

> **Run all `docker compose` commands from the repo root.** If you run them from `~`, you'll get `no configuration file provided: not found` and Docker never starts — but `ros2` may still run on your host if you have ROS installed locally. That looks like it worked, but it didn't test Docker.

**Terminal 1** — enter the container, then start the talker:

```bash
cd ~/projects/robot-map-poisoning-defense
docker compose run --name ros2_dev ros2
```

You should land in a container shell (prompt like `root@...:/workspace#`). Confirm you're inside Docker:

```bash
echo $ROS_DISTRO    # should print: jazzy
```

Then start the talker (leave it running):

```bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2** — join that same container:

```bash
docker exec -it ros2_dev bash
source /opt/ros/jazzy/setup.bash
```

Then start the listener:

```bash
ros2 run demo_nodes_py listener
```

Success: the listener prints `I heard: [Hello World: 1]`, `Hello World: 2`, etc.

When done, stop the talker with `Ctrl+C` in Terminal 1, then type `exit` to leave the container.

Clean up the container from your **host WSL terminal** (not inside the container — Docker isn't installed there):

```bash
docker rm -f ros2_dev
```

> **Why `--name ros2_dev` and `docker exec`?**  
> Each `docker compose run` creates a new container with a random name. `--name` gives it a fixed name so Terminal 2 can join it. We omit `--rm` here so the container stays alive while you test across two terminals.

For RViz, `webots_ros2`, and colcon checks, run `bash scripts/verify.sh` from the repo root instead of repeating those manually.

---

## 8. Install Webots (local)

Webots is the simulation app. It runs on your host machine, not inside Docker.

1. Download from [cyberbotics.com/download](https://cyberbotics.com/download)
2. Install and launch it
3. Confirm it opens — **File → Open World** → pick any bundled `.wbt` → press Play

The Webots app runs locally. The ROS 2 bridge (`webots_ros2`) is already in Docker. Wiring them together is a future step.

---

## Checklist — confirm all of this before starting development

- [ ] Git works: `git --version`
- [ ] GitHub SSH works: `ssh -T git@github.com`
- [ ] Docker works: `docker --version` and `docker compose version`
- [ ] Windows: WSL Integration enabled in Docker Desktop
- [ ] Repo cloned and `cd` into it
- [ ] `docker compose build` succeeded
- [ ] `colcon build` inside Docker: `Summary: 5 packages finished`
- [ ] Pub/sub test (step 7): listener prints `Hello World` messages
- [ ] `bash scripts/verify.sh` → `Results: 7 passed, 0 failed` (covers RViz, webots_ros2, colcon, etc.)
- [ ] Webots app opens on your machine

---

## Daily workflow

Run this each time you sit down to work. You do **not** need to repeat the full setup checklist.

### 1. Open your terminal

**Windows:** Open **Ubuntu 24.04** from the Start menu (search "Ubuntu"). Do not use PowerShell or CMD for Docker commands.

**Mac:** Open **Terminal** or iTerm2.

### 2. Start Docker Desktop

Launch **Docker Desktop** and wait until it says it's running (whale icon in the system tray/menu bar).

### 3. Go to the project folder

```bash
cd ~/projects/robot-map-poisoning-defense
```

### 4. Pull teammate changes (optional)

If others may have pushed updates:

```bash
git pull
```

### 5. Enter the Docker container

```bash
docker compose run --rm ros2
```

Your prompt should change to something like `root@...:/workspace#`. ROS 2 is already sourced.

### 6. Build the workspace (if code changed)

If you pulled changes or edited packages:

```bash
colcon build
source install/setup.bash
```

If nothing changed since your last session, you can skip this step.

### 7. Work on the project

You're ready. Run nodes, edit code, test things — all from inside the container.

When you're done for the day, type `exit` to leave the container.

---

### Occasionally (not every session)

**Clean rebuild** (inside the container, if builds act weird):

```bash
rm -rf build install log logs && colcon build && source install/setup.bash
```

**Rebuild the Docker image** (only after `Dockerfile` changes, on the host before step 5):

```bash
cd ~/projects/robot-map-poisoning-defense
docker compose build
```

**Something broken?** Run the verify script on the host:

```bash
cd ~/projects/robot-map-poisoning-defense
bash scripts/verify.sh
```

---

## Troubleshooting


| Problem                           | Fix                                                                                    |
| --------------------------------- | -------------------------------------------------------------------------------------- |
| `docker: permission denied`       | Start Docker Desktop; Windows: enable WSL Integration and use WSL terminal             |
| Docker commands hang silently     | You're in PowerShell — switch to the WSL Ubuntu terminal                               |
| `no configuration file provided`  | `cd ~/projects/robot-map-poisoning-defense` first — `docker compose` needs that folder |
| `No such container: ros2_dev`     | Terminal 1 didn't start the container — fix the `cd` issue above, then retry step 7    |
| `ros2: command not found`         | Enter the container first: `docker compose run --rm ros2`                              |
| Talker and listener don't connect | They're in different containers — use `docker exec` to join the same one (see §7)      |
| `build/` owned by root            | `sudo chown -R $USER:$USER build install log logs`                                     |
| `webots_ros2` packages missing    | `docker compose build --no-cache`                                                      |
| `rviz2` Qt display error          | Expected — no display in Docker yet. Use `which rviz2` to confirm it's installed       |
| SSH clone fails                   | Run `ssh -T git@github.com` to diagnose, or use HTTPS clone                            |


---

## Re-verifying the environment

Run the included verification script from the repo root in WSL:

```bash
bash scripts/verify.sh
```

This checks Docker, colcon build, ROS 2 pub/sub, all packages, and rviz2 — and prints a pass/fail summary. For manual one-liner commands and known shell gotchas, see **[docs/VERIFICATION.md](docs/VERIFICATION.md)**.
