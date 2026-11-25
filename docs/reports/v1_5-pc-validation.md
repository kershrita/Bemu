# Version 1.5: validating on your PC (WSL 2)

The steps to check that version 1.5 (pull request 3) works on the development PC, in two parts:
**automated checks** that print PASS or FAIL, and a **visual check** of the CAD robot. The
recommended way is the **Docker container** (`docker/dev`): one image holds ROS 2 Lyrical, MuJoCo
and the CAD tools, so nothing has to be installed in Ubuntu. The native WSL route (part 5) still
works and is the fallback.

**Status:** updated 2026-11-25 after validating on the development PC (WSL 2, Ubuntu 26.04,
NVIDIA GeForce RTX 5060 Ti). Automated validation (`docker/dev validate full`), WSLg windowing
(`xeyes`, `rviz2`, MuJoCo viewer), and GPU rendering (`D3D12 (NVIDIA GeForce RTX 5060 Ti)`) all
pass. See the validation results recorded at the end of this document. Four container/script issues
found during PC validation were fixed (`docker/validate.sh`, `docker/dev`, `docker/compose.yaml`,
`docker/Dockerfile`).

## 1. What you need (once)

| Need | How to check |
|---|---|
| Windows 11 with WSL 2 and Ubuntu 26.04 | `wsl -l -v` in PowerShell shows `VERSION 2` |
| WSLg (Linux windows on Windows) | in Ubuntu: `echo $DISPLAY` prints `:0`, and `ls /mnt/wslg` lists files |
| The project in your Linux home, **not** under `/mnt/c` (a Windows folder is very slow) | `cd ~/bemu && pwd` prints `/home/<you>/bemu` |
| Docker that works inside Ubuntu: either **Docker Desktop** (Settings → Resources → WSL integration → turn on Ubuntu-26.04), or Docker Engine inside Ubuntu (`sudo apt install docker.io docker-compose-v2`, then `sudo usermod -aG docker $USER`, and restart the Ubuntu window) | `docker run --rm hello-world` prints "Hello from Docker" and `docker compose version` prints a version |
| For GPU rendering (optional): the current NVIDIA Windows driver | `nvidia-smi` works inside Ubuntu |

## 2. Get the version 1.5 code

```bash
cd ~/bemu
git fetch origin
git checkout main
git pull
git log --oneline -3
```

Version 1.5 (pull request 3, from `claude/exciting-tesla-694elw`) is merged into `main` and tagged
`v1.5`. If you have local edits, `git status` shows them first.

## 3. Automated checks in the container

1. **Build the image** (first time 10–20 minutes and about 4 GB; later builds use the cache):

   ```bash
   docker/dev build
   ```

   It finishes with no red "ERROR". It downloads ROS packages, MuJoCo and Python 3.13 for the CAD
   library; it needs internet.

2. **Start the container and open a shell** (the repository appears inside at `/workspace/bemu`):

   ```bash
   docker/dev up
   docker/dev shell
   ```

   The prompt becomes `bemu@...:/workspace/bemu$`. Type `exit` to leave; the container keeps running.

3. **Quick validation** (about 10 minutes the first time: it builds the ROS workspace):

   ```bash
   docker/dev validate quick
   ```

   You should see five lines starting `PASS` and then `ALL PASSED`:

   | Step | What it proves |
   |---|---|
   | environment | ROS, MuJoCo, the MuJoCo bridge and the CAD library are installed |
   | colcon build | all ROS packages build, including the 30+ CAD meshes |
   | ROS package tests | the package tests pass (the simulated launch tests are in `full`) |
   | headless walk | the walker drives the CAD-mass robot for 8 s in the simulator |
   | CAD regenerates | the CAD code gives the same print order, printability report and masses that are committed |

4. **Full validation** (about 7 minutes on the i9 PC; up to 25 minutes on slower hardware):

   ```bash
   docker/dev validate full
   ```

   It adds the whole CI Python test suite (577 tests) and the simulated launch tests of
   `bemu_bringup` (2 tests: controllers active, robot stands, walks and stops, headless). Expect
   `ALL PASSED`. Logs of every step are in `/tmp/bemu-validate/` inside the container.

If a step says FAIL, the last lines of its log are printed; copy them (part 6).

## 4. Visual check: the CAD robot in the simulator

1. **Test that windows work** (a small window with two eyes should open on Windows):

   ```bash
   docker/dev exec xeyes
   ```

   Close it. If nothing opens, see part 7.

2. **Optional, GPU rendering:** `docker/dev --gpu up`, then `docker/dev exec glxinfo -B`. A line
   `OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 5060 Ti)` means the GPU is used; `llvmpipe`
   means software rendering, which also works but is slower. Once started with `docker/dev --gpu up`,
   later commands (`launch`, `teleop`, `rviz`, `exec`) keep using that GPU container.

3. **Launch the robot** (first time it builds the workspace, a few minutes):

   ```bash
   docker/dev launch
   ```

   A MuJoCo window opens. **Check:**
   - [ ] The robot is drawn as the printed parts: blue plates and links, black servos, grey body
         plates, orange feet, a green box on top (the Raspberry Pi) and a green box inside (the
         battery), not capsules and a plain box.
   - [ ] It stands up by itself within a few seconds and stays standing.
   - [ ] There are no red errors repeating in the terminal.

4. **Drive it** (a second Ubuntu terminal):

   ```bash
   docker/dev teleop
   ```

   Click that terminal, then press: `w` to walk forward, `s` back, `a`/`d` sidestep, `q`/`e` turn,
   space to stop, `1`/`2`/`3` for tripod, ripple and wave, `h` stand, `j` sit, `x` quit.
   - [ ] It walks forward and turns without falling over or legs crossing through each other.
   - [ ] The three gaits all work.
   - [ ] Take a screenshot of the window while it walks (Windows key + Shift + S).

5. **RViz (optional):** in a third terminal `docker/dev rviz`. Add a **RobotModel** display with
   **Description Topic** `/robot_description`, and set **Fixed Frame** to `body`. You should see the
   same parts. (The robot must be launched in the other terminal.)

6. **Stop:** `Ctrl+C` in the launch terminal; `docker/dev down` when you are done.

## 5. Native WSL route (no Docker)

Use this if Docker is a problem. The setup is in
[13 — Development environment](../13-dev-environment.md), "Setup, as done on 2026-11-18". After a
`git pull` on this branch:

```bash
source /opt/ros/lyrical/setup.bash
export BEMU_PARAMS=~/bemu/config/robot_params.yaml
cd ~/bemu/ros2_ws && colcon build --symlink-install && source install/setup.bash
colcon test --pytest-args -m "not sim_launch" && colcon test-result --verbose
ros2 launch bemu_bringup robot.launch.py           # then part 4, step 3 and on
ros2 run bemu_teleop keyboard                       # in a second terminal
```

Note: Ubuntu 26.04 ships Python 3.14, for which `build123d` does not have wheels yet, so
`pip install -r requirements-dev.txt` and the CAD pytest suite require a Python 3.13 environment
(already set up at `/opt/cad` inside the Docker container).

Use `~/bemu` as the folder (lower case), as the earlier setup did. A native build and the container's
build do not clash: the container keeps its `build/`, `install/` and `log/` in Docker volumes.

## 6. Validation results (2026-11-25, WSL 2 PC)

- **Environment:** WSL 2 (Ubuntu 26.04), Docker Engine `29.10.1`, Docker Compose `v5.8.0`,
  NVIDIA GeForce RTX 5060 Ti (driver `620.70.03`), WSLg `DISPLAY=:0`.
- **GPU check (`glxinfo -B`):** `D3D12 (NVIDIA GeForce RTX 5060 Ti)` with `docker/dev --gpu up`
  (`llvmpipe` without `--gpu`).
- **GUI smoke checks:** `xeyes` opens under WSLg; `docker/dev launch` starts the MuJoCo window with
  the CAD meshes and all controllers active (`locomotion_node`, `robot_state_publisher`,
  `leg_position_controller`, `joint_state_broadcaster`) and executes `tripod`, `ripple`, and `wave`
  gaits; `docker/dev rviz` opens cleanly with `QT_QPA_PLATFORM=xcb`.
- **Automated validation (`docker/dev validate full`):**

```text
== Bemu validation (full) ==
PASS  environment (5 s)
PASS  colcon build (4 s)
PASS  ROS package tests (no simulated launch) (47 s)
PASS  headless walk (tools/walk.py, 8 s) (0 s)
PASS  CAD regenerates to the committed outputs (10 s)
PASS  CI python tests (build123d, MuJoCo) (321 s)
PASS  simulated launch tests (bemu_bringup) (29 s)
ALL PASSED
```

  Counts from the logs: 446 ROS package tests passed (1 skipped), 577 CI Python/CAD/MuJoCo tests
  passed (5 skipped), 2 simulated launch tests passed (`test_leg_trace_launch.py` and
  `test_robot_launch.py`), `tools/walk.py` walked 0.573 m forward with 77.6 mm min stability margin
  and 0.900 N·m peak torque, and regenerated `print_order.md`, `printability.md`, and
  `mass_properties.json` matched the committed files.

- **Owner visual sign-off:** launch `docker/dev --gpu up && docker/dev launch` and `docker/dev teleop`
  to visually inspect the checkboxes in part 4 and capture a screenshot if desired.

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `permission denied … /var/run/docker.sock` | your user is not in the `docker` group (Docker Engine), or Docker Desktop is off | `sudo usermod -aG docker $USER` and reopen Ubuntu; or start Docker Desktop and turn on its WSL integration |
| `Cannot connect to the Docker daemon` | Docker Engine inside Ubuntu is not running | `sudo service docker start` (or enable systemd in `/etc/wsl.conf`) |
| `docker/dev: Permission denied` | the execute bit was lost | `chmod +x docker/dev docker/validate.sh docker/entrypoint.sh` |
| The image build fails downloading packages | no internet, a proxy, or a ROS mirror hiccup | rerun `docker/dev build`; the cache keeps finished steps |
| `docker/dev exec xeyes` opens nothing, or "cannot open display" | WSLg is not running, or `DISPLAY` is empty | in PowerShell `wsl --shutdown`, reopen Ubuntu, check `echo $DISPLAY` and `ls /mnt/wslg`, then `docker/dev down && docker/dev up` |
| The window opens but is black or very slow | software rendering | use `docker/dev --gpu up`; or `llvmpipe` is what you get and is fine for checking |
| `libGL error: failed to load driver: d3d12` with `--gpu` | the WSL GPU libraries were not passed | check `ls /usr/lib/wsl/lib` exists in Ubuntu, `nvidia-smi` works there, then `docker/dev --gpu down && docker/dev --gpu up`; otherwise run without `--gpu` |
| Files in the repo owned by root | the image was built with another user id | `docker/dev down`, then `docker/dev build` again (it uses your `id -u`) |
| `colcon build` fails right after a branch change | stale build volume | `docker/dev clean` (deletes only the Docker build volumes), then `docker/dev validate quick` |
| Very slow builds and tests | the project folder is under `/mnt/c` | move it to the Linux home (`~/bemu`) |
| `ros2` in Windows PowerShell or native Ubuntu does not see the container's topics | the container has its own network | run `ros2` commands with `docker/dev exec ros2 …` or `docker/dev shell` |
| `rviz2` crashes with `Invalid parentWindowHandle ... GLXWindow::create` | Qt picked Wayland while OGRE uses GLX (X11) | set `QT_QPA_PLATFORM=xcb` (set automatically in `docker/compose.yaml` and `docker/dev rviz`; run `docker/dev --gpu up` to recreate the container with it) |
| CAD step says "differs from the committed cad/export/…" | the branch is behind, or the CAD code changed without regenerating | `git pull`; if it persists, send the diff lines |
