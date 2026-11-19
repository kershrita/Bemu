# 05 — Software stack

Tools, packages, message contracts and conventions.

## Tools

| Purpose | Choice | Notes |
|---|---|---|
| Operating system | Ubuntu 26.04 | Under WSL 2 on the Windows PC; directly on the Pi 5 |
| Robot framework | ROS 2 Lyrical Luth | Long-term release, supported to May 2031. Fallback: Jazzy on Ubuntu 24.04 |
| Joint layer | `ros2_control`, `ros2_controllers` | Standard controller framework |
| Simulator | MuJoCo | Physics engine |
| Simulator bridge | `mujoco_ros2_control` | From the ros-controls organisation; lists Lyrical as supported |
| Visualisation | RViz, MuJoCo viewer, Foxglove or PlotJuggler | For robot state and plots |
| Language | Python 3 | C++ only for the servo hardware plugin |
| Tests | pytest, launch_testing | See [14 — Testing](14-testing-and-validation.md) |
| CAD | build123d | Python; see [08 — Mechanical design](08-mechanical-design.md) |
| Build | colcon | Standard for ROS 2 |

Versions are pinned in the repository when version 0 is set up, and recorded in
[13 — Development environment](13-dev-environment.md).

## Planned repository layout

Created in version 0, after the design is approved.

```
bemu/
├── README.md
├── docs/                         these documents
├── config/
│   └── robot_params.yaml         the single source of dimensions and limits
├── cad/                          parametric CAD scripts and exported parts
│   ├── src/
│   └── export/                   STL / STEP, generated
├── ros2_ws/
│   └── src/
│       ├── bemu_params/       parameter file loader; pure Python, no ROS dependency
│       ├── bemu_description/  robot model, generated from the parameter file
│       ├── bemu_kinematics/   leg maths; pure Python, no ROS dependency
│       ├── bemu_gait/         gaits, foot paths, the walker core and gait.yaml; pure Python
│       ├── bemu_locomotion/   the walker node (layer 2)
│       ├── bemu_control/      ros2_control configuration (layer 1)
│       ├── bemu_sim/          MuJoCo scene and bridge configuration
│       ├── bemu_hardware/     servo bus driver plugin (version 2)
│       ├── bemu_teleop/       keyboard and gamepad control
│       ├── bemu_msgs/         custom message definitions
│       └── bemu_bringup/      launch files and top-level configuration
├── tools/                        scripts: generate model, torque report, calibration
└── .github/workflows/            automated checks
```

There is no top-level `Dockerfile`: the GitHub check for the ROS packages uses the
official ROS image.

Packages added by later versions follow the same naming:
`bemu_imu`, `bemu_contact`, `bemu_perception`, `bemu_navigation`,
`bemu_rl`, `bemu_behaviors`.

## Package responsibilities

| Package | Layer | Responsibility | Depends on ROS? |
|---|---|---|---|
| `bemu_params` | all | Loads `robot_params.yaml` and converts it to SI units; used by the CAD scripts too | No |
| `bemu_kinematics` | 2 | Forward and inverse kinematics for one leg and for the body | No |
| `bemu_gait` | 2 | Gait phase timing, swing and stance foot paths, the phase-driven `Walker` (velocity command in, 18 joint angles out), gait defaults in `config/gait.yaml` | No |
| `bemu_locomotion` | 2 | ROS node that wraps the `bemu_gait` walker: topics in, joint targets out. The behaviour (stand and sit sequences, command timeout, joint matching by name) is in a ROS-free `LocomotionCore`; the node only moves messages | Yes |
| `bemu_control` | 1 | Controller configuration and joint limits | Yes |
| `bemu_description` | 0–1 | URDF and MuJoCo model, generated from the parameter file | Yes |
| `bemu_sim` | 0 | Simulation world, simulated sensors | Yes |
| `bemu_hardware` | 0 | Real servo driver | Yes (C++) |
| `bemu_teleop` | 4 | Human control | Yes |
| `bemu_msgs` | all | Shared message types | Yes |
| `bemu_bringup` | all | Launch files | Yes |

Keeping the maths in ROS-free libraries means it can be tested in milliseconds,
reused inside the training environment in version 7, and understood without knowing
ROS.

## Naming

### Legs

Viewed from above, front of the robot at the top:

```
        front
   LF ┌───────┐ RF
      │       │
   LM │       │ RM
      │       │
   LR └───────┘ RR
        rear
```

| Code | Leg | Index |
|---|---|---|
| `lf` | Left front | 0 |
| `lm` | Left middle | 1 |
| `lr` | Left rear | 2 |
| `rf` | Right front | 3 |
| `rm` | Right middle | 4 |
| `rr` | Right rear | 5 |

### Joints

| Joint | Movement | Name pattern |
|---|---|---|
| Coxa | Swings the leg forward and back, about a vertical axis | `<leg>_coxa_joint` |
| Femur | Raises and lowers the leg | `<leg>_femur_joint` |
| Tibia | Bends the knee | `<leg>_tibia_joint` |

Example: `lf_femur_joint`. Servo bus address = `leg index × 3 + joint index + 1`,
giving addresses 1 to 18.

### Frames

Following the ROS convention (REP 103): **x forward, y left, z up**, distances in
metres, angles in radians.

| Frame | Location |
|---|---|
| `base_link` | Centre of the body |
| `<leg>_coxa_link`, `<leg>_femur_link`, `<leg>_tibia_link` | Each leg segment |
| `<leg>_foot` | Foot tip |
| `imu_link` | Tilt sensor (version 4) |
| `odom` | Starting point of the robot's travel estimate |

## Message contracts

These are the interfaces between layers. A module may be rewritten freely as long
as it honours them.

### Commands (downward)

| Topic | Type | From → to | Meaning |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Layers 3–4 → walker | Desired forward speed, sideways speed and turn rate |
| `/body_pose_cmd` | `geometry_msgs/PoseStamped` | Layers 3–4 → walker | Desired body height, shift and tilt relative to neutral |
| `/gait_cmd` | `bemu_msgs/GaitCommand` | Layers 3–4 → walker | Gait type, step height, cycle time |
| `/leg_position_controller/commands` | `std_msgs/Float64MultiArray` | Walker → joint controller | 18 joint angle targets, in address order |
| `/stability_margin` | `std_msgs/Float64` | Walker | Stability margin in metres |

### State (upward)

| Topic | Type | From | Meaning |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Joint controller | Angle, speed and effort of every joint |
| `/gait_state` | `bemu_msgs/GaitState` | Walker | Current gait, phase of each leg, which feet are planned to be down |
| `/foot_contacts` | `bemu_msgs/FootContacts` | Contact module (version 5) | Which feet are actually on the ground |
| `/imu/data` | `sensor_msgs/Imu` | Tilt sensor module (version 4) | Orientation, rotation rate, acceleration |
| `/odom` | `nav_msgs/Odometry` | Walker or estimator | Estimated travel |
| `/battery_state` | `sensor_msgs/BatteryState` | Hardware driver (version 3) | Voltage and charge estimate |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | All | Health: servo temperature, load, errors |
| `/tf` | transforms | Robot state publisher | Position of every frame |

### Services and modes

| Name | Type | Meaning |
|---|---|---|
| `/stand`, `/sit` | `std_srvs/Trigger` | Run the stand or sit sequence. `/sit` first stops walking and waits for the feet to settle. Both answer `success=False` with a reason when the robot is in the wrong state |
| `/relax` | `std_srvs/Trigger` | Switch off joint torque |
| `/set_gait` | `bemu_msgs/SetGait` | Change gait |

### Rules for contracts

- Use a standard ROS message type whenever one exists. Custom types only for
  hexapod-specific data.
- A contract change needs a decision record.
- Every topic has exactly one publisher in normal operation. Where several sources
  may command the robot (remote control and navigation), a multiplexer chooses, so
  the walker still sees one `/cmd_vel`.

## Launch structure

Version 1b-2: `ros2 launch bemu_bringup robot.launch.py` (arguments `use_sim`, `headless`,
`params_file`) starts the MuJoCo simulator, `robot_state_publisher`, the two controllers
(`robot_controllers.yaml`) and `locomotion_node`. The walker stands the robot up on start
(parameter `auto_stand`) and treats a `/cmd_vel` older than 0.5 s as zero velocity.
`GaitCommand.step_height` and `cycle_time` are reserved in version 1 (non-zero values are
ignored with a warning); `GaitState.mode` is `sitting`, `standing_up`, `standing`, `stopping`
or `sitting_down`.

One top-level launch file with arguments, rather than many near-identical files.

| Argument | Values | Effect |
|---|---|---|
| `use_sim` | `true` / `false` | MuJoCo or real servos |
| `walker` | `gait` / `policy` | Programmed or learned walker |
| `teleop` | `keyboard` / `gamepad` / `none` | Human control |
| `rviz` | `true` / `false` | Start the visualiser |
| `headless` | `true` / `false` | Run the simulator without a window, for tests |

Each optional module ships its own small launch file that the top-level file
includes when asked.

## Configuration

| What | Where | Used by |
|---|---|---|
| Dimensions, masses, joint limits | `config/robot_params.yaml` | CAD, model generator, kinematics |
| Gait defaults | `bemu_gait/config/gait.yaml` | Walker |
| Controller setup | `bemu_control/config/controllers.yaml` | Joint layer |
| Simulated servo model | `bemu_sim/config/actuators.yaml` | Model generator |
| Servo addresses, offsets, directions | `bemu_hardware/config/servos.yaml` | Servo driver |

Nothing that belongs in configuration is written into code.

## Coding conventions

- Python formatted and checked by `ruff`; type hints on public functions.
- Units in SI everywhere inside the code: metres, radians, seconds, newton-metres.
  Millimetres and degrees only in the parameter file and in documents, converted at
  the boundary.
- Every function in the maths libraries has a test.
- Commit messages: a short summary line in the imperative, then the reason.
- Work on a branch per roadmap step; merge when its tests pass.
- Tag each finished version: `v0`, `v1`, `v1.5`, and so on.

## References

| Source | Used here for | Checked |
|---|---|---|
| [ROS 2 documentation](https://docs.ros.org/) | ROS 2 Lyrical Luth; standard message types | 2026-11-13 |
| [ROS 2 Lyrical vs Jazzy](https://openelab.io/cs/blogs/learn/ros-2-lyrical-vs-jazzy) | Support dates; the Jazzy fallback | 2026-11-13 |
| [ros2_control documentation](https://control.ros.org/) | ros2_control and ros2_controllers | — |
| [mujoco_ros2_control on ROS Index](https://index.ros.org/r/mujoco_ros2_control/) | mujoco_ros2_control: source, supported distributions | 2026-11-13 |
| [REP 103](https://www.ros.org/reps/rep-0103.html) | Axis directions and units for frames | — |
| [build123d documentation](https://build123d.readthedocs.io/) | The CAD tool | — |

The full list for the project is in [References](references.md). A dash means the source
is listed for reference and was not re-read on a specific date.
