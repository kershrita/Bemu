# Version 1b — Walking: gaits, velocity command, and the ROS walker

**Date:** 2026-11-18
**Status:** draft, awaiting the owner's approval
**Size:** a project, delivered as two plans (1b-1 headless, 1b-2 ROS), one design
**Roadmap steps:** 1.4, 1.5, 1.6 in [03 — Roadmap](../../03-roadmap.md)
**Builds on:** [version 1a](2026-11-18-v1a-full-body-design.md)

## 1. Intent

Version 1a made the simulated Bemu stand, sit and pose its body. 1b makes it **walk**:
a velocity command (forward, sideways, turning) drives three programmed gaits (tripod,
ripple, wave), and the robot can change gait while moving. The owner wants to see this
under ROS too, so 1b ends with the walker as a ROS node on the 18-joint robot. Keyboard
and gamepad control, and the torque report, are 1c.

The reasons, from [01 — Vision](../../01-vision-and-scope.md): programmed walking is the
first locomotion module, learned walking later replaces only this module, and the torque
numbers from walking are the evidence for the first purchase. The walker must therefore
be a clean module with the contract in [05](../../05-software-stack.md).

## 2. Goals, non-goals, anti-goals

**Goals**

- Pure-Python gait maths in a new ROS-free package `bemu_gait`: gait definitions, foot
  paths, command handling, gait switching, stability slow-down.
- Headless walking in simulation proving the 1b exit tests (section 6).
- Leg-to-leg and leg-to-body collisions switched on (deferred from 1a).
- `tools/walk.py` to run and watch it (`--viewer`).
- 1b-2: `bemu_msgs`, the 18-joint ROS model and launch, and the walker node
  (`bemu_locomotion`), so the walking runs under ROS 2 on the owner's PC.

**Non-goals (deferred)**

- Keyboard and gamepad (1c), the torque report and logged runs (1c), terrain, sensors,
  closed-loop balance, real hardware.
- Foot-ground-speed matching at touchdown (a refinement named in doc 07).

**Anti-goals**

- Loosening a bar to pass: the 10% drift, 50 mm turn, 5 mm slip and 15 mm stability
  margin are not tuned away. If open-loop walking misses one, find the cause (gait
  timing, touchdown, friction, a modelling error) and report what it takes.
- Raising actuator torque limits or changing `actuators.yaml` to make walking work.
- A switch that stops the robot, or feet that jump, hidden by testing only at low speed.

## 3. Constraints

| Constraint | Effect |
|---|---|
| Everything above the motor layer is identical in simulation and reality | Gait code is ROS-free and takes time and commands, nothing else |
| Single parameter file for dimensions | Gait defaults go in their own file (below); no dimension is repeated |
| Control rate 50 Hz, physics 2 ms | One gait step per control tick |
| Layer rule: a layer talks only to the one below, through a named contract | The walker publishes joint targets to the position controller and nothing else |
| 1a's tests stay green | Enabling collisions must not change sit, stand or pose behaviour |
| ROS code runs only on the owner's PC | 1b-2 is written in the cloud session and validated there by hand, as in version 0 |

## 4. Approaches considered

**Inside the walker.**

| | What | Trade-off |
|---|---|---|
| **Phase-driven (chosen)** | Each leg has a phase 0 to 1; a gait is per-leg offsets plus a duty factor; a foot's position is a function of phase, stride and command, as in doc 07 | Pure functions, testable headless, matches the docs. Open-loop |
| Touchdown-driven | Each leg is a state machine switching on contact | Reacts to rough ground, but needs reliable contact sensing now and is not what doc 07 specifies |

**Switching gaits while moving.**

| | What | Trade-off |
|---|---|---|
| **Blend continuously (chosen)** | Keep each leg's phase; slide its offset and duty factor to the new gait over one cycle | Foot position depends smoothly on phase and duty, so no foot jumps; stability is checked in simulation |
| Switch at a cycle boundary | | Different gaits put each leg at a different point of its step at that instant: feet would jump |
| Stop, then restart | | Breaks the roadmap's "without stopping" |

**Order of work.** One design, two plans. **1b-1** (gaits, collisions, exit tests,
headless) is proven before anything needs the owner's PC; **1b-2** (ROS) follows.

**Owner decisions:** the ROS walker node and 18-joint launch are part of 1b, not
deferred to 1c (2026-11-18); phase-driven; continuous blending; two plans.

## 5. Design

### 5.1 Pipeline and frames

```
command (vx, vy, ω) ─┐
gait + parameters ───┼─► Walker.step(dt) ─► 6 feet, body frame ─► body pose ─► 18 joint angles
body pose offset ────┘        │
                              └─► state: phases, stability margin
```

Feet are planned in the body frame (doc 07, section 4). The walker's output goes
through 1a's `pose_angles` so the body-pose offsets (height, shift, tilt, yaw) while
walking use the code already proven: the planned feet are shifted into the ground-frame
convention 1a uses (add the standing height) and passed as `feet`.

### 5.2 Gaits

Duty factor and per-leg phase offsets (LF, LM, LR, RF, RM, RR), from doc 07:

| Gait | Duty | Offsets |
|---|---|---|
| Tripod | 1/2 | 0, 1/2, 0, 1/2, 0, 1/2 |
| Ripple | 2/3 | 0, 1/3, 2/3, 2/3, 1/3, 0 |
| Wave | 5/6 | 1/2, 2/3, 5/6, 0, 1/6, 1/3 |

A leg's phase is `(t / T + offset) mod 1`; below the duty factor it is in stance. Gait
defaults live in `bemu_gait/config/gait.yaml` (cycle time per gait, step height, maximum
stride; starting values from doc 07: tripod 1.0 s, 30 mm, 60 mm), loaded into a frozen
dataclass. Doc 05 listed this file under `bemu_locomotion`; it moves to `bemu_gait`
because the pure package must read it. Ripple and wave cycle times start at values that
give the same top speed as the tripod; the builder tunes them.

### 5.3 Foot paths and command handling

- **Stance:** the foot moves in a straight line from `r + stride/2` to `r − stride/2`
  over the stance fraction, with `stride_i = (v + ω × r_i) · T_stance` (doc 07), so one
  formula covers forward, sideways, turning and mixes.
- **Swing:** the foot returns along a smooth blend, lifted by `step_height · sin(π u)`.
- **Smoothness:** foot position is continuous in phase, including at the wrap and at the
  stance/swing change.
- **Starting and stopping:** the commanded velocity is slewed with an acceleration
  limit, and the step height is scaled by the stride fraction. At zero command the feet
  stand at their neutral points and do not lift, so the phase can keep running without
  the robot stepping in place. There is no special "stopped" state.
- **Reach limits:** before outputting, the walker checks every foot target. If any leg
  cannot reach, the strides of **all** legs are scaled down together (doc 07), so the
  robot slows instead of distorting its gait. The step is never sent with a leg out of
  reach.
- **Stability slow-down:** if the stability margin falls below a threshold (15 mm,
  doc 07's estimate), the stride is scaled down the same way.

### 5.4 Gait switching

A `gait` change starts a blend lasting one cycle: each leg keeps its current phase
(a continuous variable) and its offset and the duty factor slide linearly, along the
shortest way round the circle, from the old gait's values to the new one's. A request
during a blend replaces the target. Cycle time blends the same way.

### 5.5 Collisions

Legs collide with other legs and with the body; the pairs that cannot or must not
(parent and child links, a coxa and the body, links of the same leg) are excluded in
the model. The 1a tests (stand, sit, twelve poses) must pass unchanged. If a legitimate
1a pose touches, that is a modelling finding to fix in the model, not in the tests.

### 5.6 Headless walking and the tool

A scenario helper drives `RobotSim` with the walker for a number of seconds and returns
measurements: position and heading in the ground frame, stability margin per tick,
per-foot slip during stance, tilt and height, joint-limit proximity, and the largest
reach scale applied. `tools/walk.py --gait tripod --vx 0.1 [--vy] [--omega]
[--seconds] [--viewer]` prints them, with the same viewer-exit workaround as the other
tools.

### 5.7 ROS (plan 1b-2)

- **`bemu_msgs`** (new `ament_cmake` package): `GaitCommand` (gait type, step height,
  cycle time) and `GaitState` (current gait, phase of each leg, which feet are planned
  down), as in doc 05.
- **18-joint model:** `build_robot_urdf` generates the URDF with the `ros2_control`
  block for all 18 joints, from the same code and parameter file as the MuJoCo model;
  `robot_controllers.yaml` configures `joint_state_broadcaster` and a position
  controller over the 18 joints in `joint_order()`; `robot.launch.py` (arguments
  `use_sim`, `headless`, `params_file`) starts the simulator, controllers and walker,
  as doc 13 already names it.
- **`bemu_locomotion` walker node:** subscribes to `/cmd_vel`
  (`TwistStamped`), `/body_pose_cmd` (`PoseStamped`), `/gait_cmd`; reads
  `/joint_states`; publishes 18 targets to `/leg_position_controller/commands` at 50 Hz
  in joint order, `/gait_state`, and the stability margin. Services `/stand` and `/sit`
  (`std_srvs/Trigger`) run 1a's sequences; the node stands up on start (parameter
  `auto_stand`). A stale `/cmd_vel` (no message for 0.5 s) means zero velocity.
- **Joints matched by name,** as in version 0.

## 6. Tests

**Headless (1b-1, run here and in the `python` CI job)**

| Test | Bar |
|---|---|
| Gait definitions | Each gait's phase offsets, duty factor and number of feet down at every instant match the table above |
| Foot paths | Continuous in phase (including wrap and swing/stance change); stance is a straight line at the commanded ground speed; stride follows `(v + ω × r)·T_stance` for forward, sideways, turning and a mix |
| Reach scaling | An over-large command scales all strides together and the step is never sent out of reach |
| Gait switching | Foot positions never jump across a switch, for every ordered pair of gaits |
| **Walks 2 m straight** | Forward distance 2 m with sideways drift under 10% (under 0.2 m); tripod |
| **Turns 360° in place** | Ends within 50 mm of the start, and a heading within 10° of 360° (my call: the roadmap names only the position) |
| **Body pose while walking** | Height ±10 mm and roll and pitch ±5° commanded while walking: no foot slips more than 5 mm during any stance; tilt and height reach the commands |
| **Gait switching while walking** | Tripod to ripple to wave to tripod at a constant command: forward progress in every cycle (never stops), stability margin above 15 mm throughout, no leg out of reach, tilt under 5° |
| Sideways and mixed | Walking sideways and a curve move the body the commanded way; the margin stays above 15 mm |
| Wave and ripple | Each walks 1 m straight with drift under 10%, margin above 15 mm |
| Torque | Peak torque on any physics step stays under the 1.47 N·m limit in every test; it is recorded for 1c |
| 1a regression | All 1a tests pass with collisions on |

**ROS (1b-2, on the owner's PC with `colcon test`)**

| Test | Bar |
|---|---|
| Joint order | Targets published in `joint_order()` whatever order `/joint_states` lists them |
| Command timeout | No `/cmd_vel` for 0.5 s stops the walking |
| Launch | `robot.launch.py` starts every node; both controllers go active; the robot stands |
| Walk | A forward `/cmd_vel` for a few seconds moves the simulated body forward, within the drift bar |
| Contracts | `/gait_state`, `/gait_cmd` and the other topics carry the documented types |

## 7. Deliverables

- `bemu_gait` (gaits, foot paths, command slewing, reach scaling, blending, the walker core, `gait.yaml`).
- Collisions in the model generator; a walking scenario helper in `bemu_sim`; `tools/walk.py`.
- 1b-2: `bemu_msgs`, `build_robot_urdf`, `robot_controllers.yaml`, `robot.launch.py`, `bemu_locomotion`, ROS tests.
- Docs: 05 (packages, where `gait.yaml` lives), 06, 07 (blending rule, stop rule), 13 (daily workflow), 14, roadmap steps 1.4 to 1.6.

## 8. Left to the builder

- Names and layout inside `bemu_gait`; the swing blend shape; slewing limits; ripple and
  wave cycle times; the contact exclusion list in the model; the walking-scenario helper's API.
- How the ROS node schedules its 50 Hz loop.

## 9. Risks and open points

| Risk | Plan |
|---|---|
| Open-loop walking drifts more than 10%, or turns miss 50 mm | Diagnose (touchdown timing, friction, slip, modelling); report what closing the gap needs. Do not loosen the bar |
| Enabling collisions changes the 1a behaviour | The 1a tests are the gate; fix the model's exclusions, not the tests |
| Gait switching is unstable at the moment of the switch | Stability margin and reach are tested through every ordered pair; slow the blend or the stride before changing the bar |
| First ROS run needs fixes (as in version 0) | Expected; 1b-2 is written to be run and corrected on the owner's PC |
| A `rosidl` message package adds a CI step | The `ros` job builds it in the ROS image; check at the first push |

## References

| Source | Used here for |
|---|---|
| [07 — Locomotion](../../07-locomotion.md) | Gaits, foot paths, reach and stability rules, stand and sit |
| [05 — Software stack](../../05-software-stack.md) | Topics, message contracts, package layout |
| [06 — Simulation](../../06-simulation.md), [14 — Testing](../../14-testing-and-validation.md) | Collisions, the walking tests |
| [Version 1a design](2026-11-18-v1a-full-body-design.md) | Body-pose code, `RobotSim`, the viewer pattern |
