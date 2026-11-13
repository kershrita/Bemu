# Glossary

Terms as they are used in these documents.

## Robot and mechanics

| Term | Meaning |
|---|---|
| Hexapod | A six-legged walking robot |
| Joint | A powered pivot. This robot has 18 |
| Degrees of freedom | The number of independent movements. Three per leg here |
| Coxa | The hip segment and the joint that swings the leg forward and back |
| Femur | The upper leg segment and the joint that raises and lowers the leg |
| Tibia | The lower leg segment and the knee joint |
| Stance | The part of a step when the foot is on the ground, pushing |
| Swing | The part of a step when the foot is in the air, returning |
| Gait | The timing pattern of the legs |
| Duty factor | The fraction of a gait cycle a leg spends in stance |
| Tripod gait | Three legs move at a time, in two alternating groups |
| Ripple gait | Two legs move at a time, in three pairs |
| Wave gait | One leg moves at a time |
| Support polygon | The shape formed on the ground by the feet in stance |
| Static stability | Staying upright because the centre of mass is above the support polygon |
| Stability margin | The distance from the point under the centre of mass to the nearest edge of the support polygon |
| Body pose | The body's height, shift and tilt relative to its feet |

## Maths

| Term | Meaning |
|---|---|
| Kinematics | The geometry of motion, ignoring forces |
| Forward kinematics | Joint angles in, foot position out |
| Inverse kinematics | Foot position in, joint angles out |
| Frame | A coordinate system attached to a part of the robot |
| Transform | The position and rotation of one frame relative to another |
| Torque | Turning force. Measured in newton-metres (N·m) or kilogram-centimetres (kg·cm). 1 kg·cm ≈ 0.098 N·m |
| Stall torque | The most torque a servo can produce, at standstill, briefly |
| Rated torque | The torque a servo can sustain continuously |
| Inertia | A body's resistance to being spun up or slowed down |

## Hardware

| Term | Meaning |
|---|---|
| Servo | A motor, gearbox, position sensor and controller in one unit; it moves to a commanded angle |
| Hobby servo | A simple servo controlled by pulse width, with no feedback to the computer |
| Smart servo, bus servo | A servo with a data connection that accepts commands and reports its state. Many share one cable |
| Feedback | Information the servo sends back: position, speed, load, voltage, temperature |
| Half-duplex | A single data line used in both directions, one at a time |
| IMU | Inertial measurement unit: a sensor for tilt, rotation rate and acceleration |
| LiPo | Lithium polymer battery |
| Cell count (2S, 3S) | Number of cells in series. Each is 3.7 V nominal, 4.2 V full |
| Step-down (buck) converter | A circuit that lowers voltage efficiently |
| Heat-set insert | A brass thread pressed into plastic with a soldering iron |
| PETG, PLA, TPU | 3D printing plastics: tough, easy, flexible respectively |

## Software

| Term | Meaning |
|---|---|
| ROS 2 | Robot Operating System 2: a framework in which a robot's software is split into programs that exchange messages |
| Node | One running program in ROS 2 |
| Topic | A named channel that nodes publish messages to and subscribe to |
| Service | A request-and-reply call between nodes |
| Package | A unit of ROS 2 software: code, configuration and launch files |
| Launch file | A script that starts a set of nodes with their settings |
| ros2_control | The ROS 2 framework that connects controllers to hardware through a standard interface |
| Hardware interface | The ros2_control component that talks to real motors or to a simulator |
| Controller | A ros2_control component that takes commands and drives joints |
| URDF | The robot description format used by ROS tools |
| MJCF | The robot description format used by MuJoCo |
| MuJoCo | A physics simulator well suited to legged robots |
| RViz | The ROS 2 3D visualiser |
| TF | The ROS 2 system that tracks every frame's position |
| colcon | The ROS 2 build tool |
| Distribution | A named release of ROS 2 (Jazzy, Lyrical) |
| WSL 2 | Windows Subsystem for Linux: runs a real Linux system inside Windows |
| LTS | Long-term support: a release maintained for several years |
| Parametric CAD | Design in which dimensions are variables, so a part can be regenerated at a different size |
| STL, STEP | File formats for 3D parts: STL for printing, STEP for exchange between CAD programs |

## Learning

| Term | Meaning |
|---|---|
| Reinforcement learning | Training by trial and error against a score |
| Policy | The trained network that maps observations to actions |
| Reward | The score that defines good behaviour |
| PPO | A widely used reinforcement learning algorithm |
| Sim-to-real | Moving a behaviour developed in simulation onto the real robot |
| Domain randomization | Varying the simulation during training so the policy tolerates real-world differences |
| ONNX | A standard file format for trained networks |

## Project terms

| Term | Meaning |
|---|---|
| Version | A roadmap step that adds one capability and has an exit test |
| Exit test | The checklist a version must pass to be finished |
| Purchase gate | The point at which a buying stage is allowed to start |
| Layer | One level of the software structure in [04 — System architecture](04-system-architecture.md) |
| Contract | The agreed message types and topics between layers |
| Decision record | A short note recording why a choice was made |
| Parameter file | `config/robot_params.yaml`, the single source of dimensions |

## References

No external sources. This document draws only on the owner's decisions and the
other documents in this repository. The project's full list is in
[References](references.md).
