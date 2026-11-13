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
hexapod-robot/
├── README.md
├── docs/                         these documents
├── config/
│   └── robot_params.yaml         the single source of dimensions and limits
├── cad/                          parametric CAD scripts and exported parts
│   ├── src/
│   └── export/                   STL / STEP, generated
├── ros2_ws/
│   └── src/
│       ├── bemu_description/  robot model, generated from the parameter file
│       ├── bemu_kinematics/   leg maths; pure Python, no ROS dependency
│       ├── bemu_gait/         gait timing and foot paths; pure Python
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

Packages added by later versions follow the same naming:
`bemu_imu`, `bemu_contact`, `bemu_perception`, `bemu_navigation`,
`bemu_rl`, `bemu_behaviors`.

## Package responsibilities

| Package | Layer | Responsibility | Depends on ROS? |
|---|---|---|---|
| `bemu_kinematics` | 2 | Forward and inverse kinematics for one leg and for the body | No |
| `bemu_gait` | 2 | Gait phase timing, swing and stance foot paths | No |
| `bemu_locomotion` | 2 | Node that combines command, gait and kinematics into joint targets | Yes |
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
| `/stand_up`, `/sit_down` | `std_srvs/Trigger` | Run the stand or sit sequence |
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
| Gait defaults | `bemu_locomotion/config/gait.yaml` | Walker |
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
