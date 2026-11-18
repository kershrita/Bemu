# 13 — Development environment

What is installed where. Exact commands and pinned versions are written into this
document during roadmap step 0.2, when the environment is actually set up and
tested. Until then this page describes the target.

## Two machines

| Machine | Role | Needs |
|---|---|---|
| Development PC | Simulation, visualisation, CAD, tests; later, training | Windows 11, Intel Core i9 (14900 series), 32 GB memory, NVIDIA RTX 5060 Ti 16 GB. Ubuntu 26.04 runs under WSL 2 |
| Raspberry Pi 5 | The robot's computer, from version 2 | Ubuntu Server 26.04 (64-bit), ROS 2 base install |

## The PC: Windows 11 with WSL 2

The chosen setup is Ubuntu 26.04 running under WSL 2 (Windows Subsystem for
Linux). The PC is far more powerful than the simulation needs.

**Why not ROS 2 directly on Windows?** ROS 2 Lyrical supports Windows 11, but its
Windows install is a binary archive that does not include every package, while the
Ubuntu packages are the recommended route. The Pi 5 runs Ubuntu as well, so using
Ubuntu on the PC means one environment instead of two.

| Point | Detail |
|---|---|
| Install | WSL 2, then the Ubuntu 26.04 distribution from the WSL command line or the Microsoft Store |
| Graphical programs | RViz and the MuJoCo viewer open as ordinary windows on the Windows desktop |
| Graphics card | Check in step 0.2 that 3D rendering inside WSL uses the NVIDIA card. A known failure is WSL silently falling back to slow software rendering |
| Where the code lives | Inside the Linux file system, not on the Windows drive. Builds are much faster there |
| Editor | An editor on Windows connected to WSL, or any editor inside Linux |
| Memory | WSL's memory limit is raised in its settings file if the default proves too small |
| USB devices | The servo adapter (version 2) is passed from Windows into WSL with a USB pass-through tool, or servo setup is done from a Windows tool directly |
| Network to the Pi | WSL sits behind a virtual network by default. Its networking mode is changed so ROS 2 on the PC and on the Pi can find each other. Set up and tested in version 2 |
| Training (version 7) | NVIDIA's computing libraries work inside WSL 2, so training can use the graphics card there |

### Fallbacks

| If | Then |
|---|---|
| 3D viewers are slow or unstable under WSL | Dual-boot Ubuntu 26.04 on the same PC |
| ROS 2 networking between WSL and the Pi is unreliable | Dual-boot Ubuntu, or run the visualiser on the Pi's side through a web tool |
| Training under WSL is troublesome | Dual-boot Ubuntu, or a rented cloud machine |

The repository has no `Dockerfile`. The GitHub check for the ROS packages runs in the
official ROS image (`ros:lyrical-ros-base`) instead.

## Software on the PC

| Software | Purpose | Source |
|---|---|---|
| ROS 2 Lyrical Luth, desktop install | Framework, RViz, command-line tools | Official ROS 2 package repository |
| ros2_control and ros2_controllers | Joint layer | ROS 2 package repository |
| MuJoCo | Physics | Installed with the bridge, or from the MuJoCo releases |
| mujoco_ros2_control | Bridge between MuJoCo and ros2_control | ROS 2 package repository if a binary exists for the distribution, otherwise built from source in the workspace |
| Python 3 with numpy, pytest, ruff | Maths, tests, formatting | System or virtual environment |
| build123d (if chosen) | CAD | Python package |
| A slicer | Preparing prints, from version 1.5 | Any |
| PlotJuggler or Foxglove | Plotting recorded data | Optional |

**Fallback:** if a package needed in version 0 is not available for Lyrical, the
project uses ROS 2 Jazzy on Ubuntu 24.04 instead. The code does not depend on
anything specific to Lyrical. See
[decision 0006](decisions/0006-ros2-distribution.md).

## Setup, as done on 2026-11-18

These are the commands that were run and worked on the development PC (Ubuntu 26.04
under WSL 2). Everything after step 1 runs inside Ubuntu.

1. **WSL 2 and Ubuntu 26.04** (administrator PowerShell):
   `wsl --install -d Ubuntu-26.04`, then check `wsl -l -v` shows `VERSION 2`.
2. **3D rendering on the NVIDIA card.** By default Mesa falls back to software
   rendering (`llvmpipe`) and, when asked for hardware, picks the Intel integrated
   graphics. Select the NVIDIA card explicitly:

   ```bash
   sudo apt update && sudo apt install -y mesa-utils
   echo 'export GALLIUM_DRIVER=d3d12
   export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> ~/.bashrc && source ~/.bashrc
   glxinfo -B | grep "OpenGL renderer"     # D3D12 (NVIDIA GeForce RTX 5060 Ti)
   ```

3. **ROS 2 Lyrical.** Enable the ROS apt source (`ros2-apt-source` package, as in the
   official guide), then:

   ```bash
   sudo apt update && sudo apt install -y ros-dev-tools ros-lyrical-desktop
   echo 'source /opt/ros/lyrical/setup.bash' >> ~/.bashrc && source ~/.bashrc
   ros2 doctor        # all checks pass; "newer version" warnings are harmless
   ```

4. **Control stack and the MuJoCo bridge:**

   ```bash
   sudo apt install -y ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
     ros-lyrical-mujoco-ros2-control ros-lyrical-mujoco-ros2-control-demos
   ```

   If `ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py` crashes
   `robot_state_publisher` with `libsdformat.so.16: cannot open shared object file`,
   the terminal was opened before the packages were installed and does not have the
   vendor library on its path. Open a fresh terminal (or run
   `source /opt/ros/lyrical/setup.bash`); that fixed it on the development PC.

5. **Repository and tests:**

   ```bash
   git clone https://github.com/kershrita/Bemu.git ~/bemu && cd ~/bemu
   sudo apt install -y python3-numpy python3-yaml python3-pytest python3-pip
   pip install --user --break-system-packages mujoco "ruff==0.17.*"
   echo 'export BEMU_PARAMS=~/bemu/config/robot_params.yaml' >> ~/.bashrc && source ~/.bashrc
   python3 -m pytest -m "not sim_launch"
   cd ros2_ws && colcon build --symlink-install && source install/setup.bash
   colcon test && colcon test-result --verbose
   ```

### Things that went wrong, and how to recognise them

| Symptom | Cause | Fix |
|---|---|---|
| `glxinfo` says `llvmpipe` | Mesa defaults to software rendering under WSL | Step 2 |
| Window renders on the Intel chip | Mesa picked the first Direct3D adapter | `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` |
| `libsdformat.so.16` not found | Terminal opened before the packages were installed | Open a fresh terminal |
| Spawners fail with `Failed loading controller`, repeated `Moved backwards in time` | Two simulators running on one ROS network | Stop the old one: `pkill -f "ros2 launch"; pkill -f ros2_control_node; pkill -f robot_state_publisher` |
| `colcon test` says `NO TESTS RAN` | The package does not declare pytest | `extras_require={"test": ["pytest"]}` in `setup.py` (setuptools no longer accepts `tests_require`) |
| Closing the MuJoCo viewer window segfaults | Mesa EGL crash in `glfw.terminate` under WSL | `tools/trace_foot.py` ends the process without that teardown |

## Version record

Recorded on 2026-11-18, on the development PC.

| Component | Version | Date recorded |
|---|---|---|
| Windows build | 10.0.26300.9711 | 2026-11-18 |
| WSL | 3.0.5.0 (kernel 6.18.58.1-1, WSLg 1.0.79) | 2026-11-18 |
| NVIDIA driver | 620.70.03 in WSL (Windows kernel-mode driver 621.14), RTX 5060 Ti | 2026-11-18 |
| Ubuntu | 26.04.1 LTS | 2026-11-18 |
| ROS 2 distribution | Lyrical (`ros-lyrical-desktop` 0.13.0) | 2026-11-18 |
| ros2_control | 6.10.1 | 2026-11-18 |
| MuJoCo (Python, used by the headless tests and the viewer) | 4.0.1 | 2026-11-18 |
| MuJoCo (inside ROS, `mujoco_vendor`) | package 0.1.0; its MuJoCo version is not recorded and may differ from the pip one | 2026-11-18 |
| mujoco_ros2_control | 0.1.2 | 2026-11-18 |
| Python | 3.14.4 | 2026-11-18 |
| ruff | 0.17.x (pinned) | 2026-11-18 |
| build123d | Not used until version 1.5 | |

## The Raspberry Pi 5 (from version 2)

| Step | Detail |
|---|---|
| Operating system | Ubuntu Server 26.04, 64-bit, written with Raspberry Pi Imager |
| Access | SSH with keys; no screen or keyboard on the robot |
| ROS 2 | Base install (no desktop tools) |
| Workspace | The same repository, built on the Pi |
| Network | The same network as the PC; the same ROS domain number on both |
| Clock | Time synchronisation enabled, so data from the Pi and the PC lines up |
| Serial adapter | A device rule that gives the servo adapter a fixed name and low-latency setting |
| Start on boot | A system service that launches the robot stack, added in version 3 |

## Daily workflow

| Task | Command or tool |
|---|---|
| Build | `colcon build --symlink-install` in `ros2_ws/` |
| Test | `colcon test`, then `colcon test-result --verbose` |
| Run the simulated leg (version 0) | `ros2 launch bemu_bringup leg.launch.py shape:=square` (or `circle`, `none`); add `headless:=true` for no window |
| Run on the robot | Version 2 onward; `use_sim:=false` stops with a message in version 0 |
| Watch the robot's state | RViz |
| Record data | `ros2 bag record` |
| Regenerate the model after a parameter change | `python3 tools/generate_model.py` |
| Stand, sit or pose the whole robot without ROS | `python3 tools/stand.py stand` (or `sit`, or `pose --roll 10 --shift 20 0`; add `--viewer` for a window) |
| Walk the whole robot without ROS | `python3 tools/walk.py --gait tripod --vx 0.1 --seconds 10` (also `--vy`, `--omega`; add `--viewer` for a window) |
| Write the full-robot model | `python3 tools/generate_model.py --robot` |
| Trace a foot path without ROS | `python3 tools/trace_foot.py --shape square` (add `--viewer` for a window) |
| Regenerate the printable parts | A script in `cad/` |

## Automated checks

On every change pushed to GitHub:

1. Formatting and static checks.
2. Unit tests for the maths libraries.
3. Build of the ROS workspace in a container.
4. The simulated exit tests of every completed version, run without a screen.

Set up in step 0.1. See [14 — Testing and validation](14-testing-and-validation.md).

## References

| Source | Used here for | Checked |
|---|---|---|
| [ROS 2 Lyrical installation](https://docs.ros.org/en/lyrical/Installation.html) | Platforms ROS 2 Lyrical supports and how each installs | 2026-11-14 |
| [ROS 2 Lyrical: Windows (binary)](https://docs.ros.org/en/lyrical/Installation/Windows-Install-Binary.html) | The Windows binary install does not include every package | 2026-11-14 |
| [ROS 2 Lyrical: installing on Ubuntu](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html) | Installing ROS 2 Lyrical on Ubuntu 26.04 | 2026-11-14 |
| [ROS 2 Lyrical Luth released](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021) | Platform support tiers | 2026-11-14 |
| [Installing Ubuntu 26.04 on WSL](https://www.how2shout.com/?p=89047) | Installing Ubuntu 26.04 under WSL | 2026-11-14 |
| [WSL visualization, NVIDIA forum](https://forums.developer.nvidia.com/t/wsl-visualization/294703) | Reports of graphics problems under WSL with NVIDIA cards | 2026-11-14 |
| [mujoco_ros2_control on ROS Index](https://index.ros.org/r/mujoco_ros2_control/) | The MuJoCo bridge package | 2026-11-13 |
| [build123d documentation](https://build123d.readthedocs.io/) | The CAD library | — |

The full list for the project is in [References](references.md). A dash means the source
is listed for reference and was not re-read on a specific date.
