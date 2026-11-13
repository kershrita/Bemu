# 13 — Development environment

What is installed where. Exact commands and pinned versions are written into this
document during roadmap step 0.2, when the environment is actually set up and
tested. Until then this page describes the target.

## Two machines

| Machine | Role | Needs |
|---|---|---|
| Development PC | Simulation, visualisation, CAD, tests; later, training | Ubuntu 26.04 (native, container or virtual machine), a few GB of disk, any reasonably modern processor |
| Raspberry Pi 5 | The robot's computer, from version 2 | Ubuntu Server 26.04 (64-bit), ROS 2 base install |

The PC's operating system is open question Q1 in [02 — Design](02-design.md).

## The PC, by operating system

| PC runs | Recommended setup | Notes |
|---|---|---|
| Ubuntu 26.04 | Native install | Simplest and fastest |
| Another Linux, or an older Ubuntu | A container based on Ubuntu 26.04 with ROS 2 Lyrical | Graphics windows need the display passed into the container |
| Windows 11 | Ubuntu 26.04 under WSL 2 | Linux graphical programs open as normal windows; simulation speed is close to native |
| macOS | A Linux virtual machine or container | Workable for code and tests; the 3D viewers are slower |

If native Ubuntu 26.04 is not available, a container definition is added to the
repository in step 0.2 so the environment is identical everywhere, including in the
automated checks.

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

## Setup checklist for step 0.2

Each line becomes a tested command in this document.

- [ ] Install or start Ubuntu 26.04.
- [ ] Install ROS 2 Lyrical following the official installation guide.
- [ ] Install ros2_control, ros2_controllers and the MuJoCo bridge.
- [ ] Run one of the MuJoCo bridge's demo launches to confirm the simulator, the
      viewer and the controllers work together.
- [ ] Clone this repository; build the workspace with `colcon build`.
- [ ] Run the tests with `colcon test`.
- [ ] Record every installed version in the table below.

## Version record

Filled in during step 0.2.

| Component | Version | Date recorded |
|---|---|---|
| Ubuntu | | |
| ROS 2 distribution | | |
| ros2_control | | |
| MuJoCo | | |
| mujoco_ros2_control | | |
| Python | | |
| build123d | | |

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
| Run the simulation | `ros2 launch hexapod_bringup robot.launch.py use_sim:=true` |
| Run on the robot | The same, with `use_sim:=false` |
| Watch the robot's state | RViz |
| Record data | `ros2 bag record` |
| Regenerate the model after a parameter change | A script in `tools/` |
| Regenerate the printable parts | A script in `cad/` |

## Automated checks

On every change pushed to GitHub:

1. Formatting and static checks.
2. Unit tests for the maths libraries.
3. Build of the ROS workspace in a container.
4. The simulated exit tests of every completed version, run without a screen.

Set up in step 0.1. See [14 — Testing and validation](14-testing-and-validation.md).
