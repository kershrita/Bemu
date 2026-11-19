# Version 1a — The full simulated body: model, stand and sit, body pose

**Date:** 2026-11-18
**Status:** implemented 2026-11-18 (approved by the owner; corrected the same day while writing the plan, see §5.2)
**Size:** a project (the first of three plans that make up version 1)
**Roadmap steps:** 1.1, 1.2, 1.3 in [03 — Roadmap](../../03-roadmap.md)

## 1. Intent

Version 0 proved one leg. Version 1 puts six of them on a free-floating body and
makes the robot stand, sit and pose its body in simulation, then walk (1b) and be
driven and measured (1c). This plan, **1a**, delivers the foundation the other two
build on: the full 18-joint model, the body-pose maths, and stand and sit, all
tested **without ROS** so every test runs here, in GitHub's `python` job and on the
owner's PC.

The aim beyond passing tests: by the end of 1a the owner can run one command and
watch the simulated Bemu stand up, tilt, shift and sit down.

## 2. Goals, non-goals, anti-goals

**Goals**

- A generated 18-joint MuJoCo model of the whole robot, from `robot_params.yaml`.
- Pure-Python body-pose kinematics: given a body pose and the feet planted on the
  ground, produce all 18 joint angles; report which leg cannot reach.
- Stand-up and sit-down sequences built on that maths.
- A headless `RobotSim` that the tests (and 1b and 1c) drive.
- A stability-margin function, used here to show body shifts stay safe.
- A viewer script for the owner's PC.

**Non-goals (deferred)**

- Gaits, walking, turning (1b). Keyboard, gamepad, torque report (1c).
- The 18-joint ROS description, controllers and launch. They arrive with the walker
  node in 1b/1c, so the owner will not see the robot stand under ROS until then.
- Leg-to-leg and leg-to-body collisions. [06 — Simulation](../../06-simulation.md)
  says self-collision is enabled; 1a defers it because stand, sit and pose never
  cross legs. It goes in with the gaits in 1b, which can cross them. Doc 06 is
  updated to say so.
- Real hardware, sensors, CAD.

**Anti-goals** (a pass that would still be a failure)

- Tuning actuator gains, torque limits or tolerances until a test passes. If the
  robot cannot stand within the 1.47 N·m limit, that is a finding to report, not a
  number to bump.
- Tests that pass without exercising each leg's mount position and angle: a model
  that is right for one leg and wrong for the other five.

## 3. Constraints

| Constraint | Effect |
|---|---|
| Everything above the motor layer is identical in sim and reality | Body maths lives in ROS-free libraries; the sim is only a stand-in for the motors |
| Dimensions only in `config/robot_params.yaml` | New body size and mass go there, converted to SI by the loader |
| SI units in code; leg codes `lf lm lr rf rm rr`; joint names `<leg>_<coxa\|femur\|tibia>_joint` | As in version 0 |
| Physics step 2 ms, control rate 50 Hz | As in version 0 |
| Python 3.12+, runs in the cloud session and in CI | No ROS imports in the new code |
| Simulated servo model is a placeholder | Torque numbers are indicative until version 2 |

## 4. Approaches considered

| | What | Trade-off |
|---|---|---|
| **A. Headless first** | Model generator, pure body maths, headless `RobotSim`, tests, viewer script. ROS wiring waits for 1b/1c | Fast feedback, fully testable here and in CI. The owner does not see it under ROS until 1b/1c |
| B. Headless plus ROS in 1a | A, plus an 18-joint ROS description, controllers and a stand command node | The owner sees it stand under ROS sooner, but every ROS fix needs the owner's PC (version 0 took about a dozen round trips for one leg), and the wiring is built before the walker that uses it |
| C. ROS first | Start from a walker node | Slow feedback; conflicts with the roadmap's rule that exit tests run without a screen |

**Owner decision: A**, on 2026-11-18.

## 5. Design

### 5.1 Frames and the pose model

- **Body frame:** origin at the body centre on the hip plane, x forward, y left,
  z up. The six hip (coxa) axes are vertical lines through the mount points, with
  `z = 0` in this frame.
- **Ground frame:** fixed to the floor, z up.
- **Leg frame:** as in version 0 (origin on the coxa axis, x along the leg's resting
  direction).

A **body pose** is a position `t = (x, y, h)` in the ground frame (`h` is the height
of the hip plane above the floor) and an orientation `R` (roll, pitch, yaw applied
as `Rz(yaw) · Ry(pitch) · Rx(roll)`). The **planted foot** of leg `i` is the foot
sphere's *centre*, a point `F_i` at height `leg.foot_radius` (8 mm) above the floor.
For standing, `F_i` is the neutral foot position: the mount point plus the stance's
`foot_radius` along the leg's resting direction. (Two different radii share a name:
`stance.foot_radius` is the 120 mm reach; `leg.foot_radius` is the 8 mm sphere.)

For any pose the joint angles follow from three steps already in
[07 — Locomotion](../../07-locomotion.md):

```
P_i  = Rᵀ · (F_i − t)               planted foot, seen from the body
p_i  = Rz(−ψ_i) · (P_i − m_i)       the same point in leg i's frame
angles_i = inverse(leg, p_i)        version 0's inverse kinematics
```

`F_i` is an input, not a constant, so 1b can pass moving feet through the same code.

### 5.2 Body size and the sit pose

New parameter-file entries: `body.size` (x, y, z in mm), `body.mass` (g, the chassis:
everything except the six legs) and `leg.foot_radius` (mm, the foot sphere; version 0
hard-coded 8 mm in the generator). Starting values: **240 × 160 × 56 mm**, **1,390 g**
and **8 mm**. Chassis plus six legs (185 g each, from the existing link masses) is the
2,500 g design mass in [08](../../08-mechanical-design.md).

The body is a box centred on the hip plane. Two heights follow:

- **Standing:** the stance keeps its meaning, "hip axes 90 mm above the foot". The foot
  sphere's centre is 8 mm above the floor, so the hip plane stands **98 mm** above the
  floor and the body's underside is 70 mm clear (doc 08 says about 60 mm).
- **Sitting:** the body rests on the floor, so the hip plane is **28 mm** up, only
  **20 mm** above the foot centres.

That 20 mm is the tight spot. With the feet at the neutral radius, the femur needs
**85.6°** at a 20 mm drop. Checked against the real leg maths: 16 mm passes (89.5°),
15 mm fails (90.4°, past the 90° limit), so the hip must be at least 16 mm above the
foot centres, which means a body at least **48 mm** thick. 56 mm leaves 4.4° of margin.
(The first draft of this design said 40 mm. It forgot the foot sphere; 40 mm fails.)
The loader rejects a body thickness for which the sit pose is unreachable, with an
error naming `body.size`.

### 5.3 Stand and sit

The sim's initial state is the **`sit`** keyframe: body resting on the floor, feet
at their neutral planted positions. (This makes the first step in the locomotion
doc, "move each foot to its neutral position at ground level", already true; the
real robot's power-on posture is a version 3 concern.)

- **Stand up:** ramp `h` from the sit height (28 mm) to the standing height (98 mm) over
  2 s with a smooth profile, producing one set of 18 targets per 20 ms control tick.
- **Sit down:** the same ramp reversed. "Switch off joint torque" at the end is a
  real-robot step and is out of scope in simulation.

Both are pure functions of time, returning targets; the sim just plays them.

### 5.4 The simulated robot

`build_robot_mjcf` generates the model with the same rules as version 0's leg:

- floor plane; the body is a box with a free joint; each leg is the version 0 chain
  placed at its mount with its yaw; capsule links with the parameter-file masses;
  sphere feet with friction 1.0;
- 18 `<position>` actuators, **named exactly as their joints**, same gains and
  force limits from `actuators.yaml`;
- sites `<leg>_hip` and `<leg>_foot` for every leg;
- keyframes `sit` and `stand` (both holding `qpos` and `ctrl`);
- contacts: body and feet collide with the floor; legs do not collide with each
  other or the body yet (see non-goals).

`RobotSim` (headless, `bemu_sim`) mirrors version 0's `LegSim`: starts at `sit`;
`command` takes 18 angles, advances one control period; reads body position and
orientation, joint angles, joint torques, foot positions in the ground frame, and
which feet touch the floor.

### 5.5 Stability margin

A pure function: the shortest distance from the centre of mass, projected onto the
floor, to the edge of the polygon formed by the feet in contact; negative if outside.
Needs `RobotSim` to supply the centre of mass. 1a uses it in the shift test; 1b/1c
use it for the walker and the report.

## 6. Tests (all without ROS, in the `python` CI job)

| Layer | Test | Why it catches something |
|---|---|---|
| Parameters | Body size, mass and foot radius load in SI; a body size whose sit pose is unreachable is rejected naming the key, and the loader's answer agrees with the real inverse kinematics | The sit-pose limit above |
| Body maths | Neutral pose gives each leg the angles that put its foot at the neutral radius | Mount position and yaw per leg |
| Body maths | For poses with height, shift, roll, pitch and yaw, forward kinematics through each mount returns the planted foot in the ground frame within 1e-6 m, for all six legs | Sign and order errors in the transforms |
| Body maths | A pose a leg cannot reach raises an error naming that leg | No silent wrong answers |
| Model | 18 named joints and actuators; `sit` and `stand` keyframes; total mass 2.5 kg; each foot site matches the maths at `stand` within 1e-6 m | The model and the maths agree for every leg |
| Sim | From `sit`, stand-up ends within 3 mm of 98 mm, tilt under 1°, all six feet touching, no foot moved more than 5 mm sideways, peak torque under the 1.47 N·m limit | The headline behaviour |
| Sim | Sit-down ends resting on the floor, hip plane within 3 mm of 28 mm | The reverse path |
| Sim | Height, shift (±20 mm), roll and pitch (±10°), yaw (±15°) each reach the commanded pose without a foot slipping more than 5 mm; stability margin stays positive | Body pose control |

These tolerances are the owner-visible bar. If a test fails, the response is to
find the cause (a modelling mistake, or a real torque or reach limit), not to widen
the number.

## 7. Deliverables

- `bemu_params`: body size, body mass and foot radius, with the sit-reachability check.
- `bemu_kinematics`: body-pose maths, stand and sit profiles, stability margin.
- `bemu_description`: `build_robot_mjcf` and its `write_models` entry.
- `bemu_sim`: `RobotSim`.
- `tools/stand.py` (`--viewer`, `--pose`): prints the metrics above; plays in the
  MuJoCo viewer. The viewer exit workaround from `tools/trace_foot.py` applies.
- Docs: 06 (self-collision deferred, full-model details), 07 (stand-up note),
  08 (body block), 14 (the new tests), roadmap ticks for 1.1 to 1.3.

## 8. Left to the builder

- Module and file layout, names of the new public functions.
- The smooth profile for the height ramp (any profile with zero speed at both ends).
- Settle times and sampling in the sim tests; the actuator values stay as they are
  unless a documented modelling error is found.
- How to place the free-joint body so `sit` starts at rest without a drop.

## 9. Open points and risks

| Risk | Plan |
|---|---|
| The robot cannot stand within the 1.47 N·m limit | Report it with the torque numbers; the roadmap's servo rule decides the response. Do not edit the limit to pass |
| Standing or sit falls apart because of foot friction or contact settings | Diagnose from the sim; the contact parameters are a modelling choice and may be adjusted with a ledger note, unlike torque limits |
| The sit pose is only 4.4° from the femur limit | Covered by the loader check and a test; a different body or foot in version 1.5 CAD must be re-checked |

## Results

Measured in simulation with the placeholder servo values:

| Measure | Result | Bar |
|---|---|---|
| Standing hip height | 96.5 mm (the position actuator sags about 1.5 mm under load) | within 3 mm of 98 mm |
| Sitting hip height | 27.9 mm | within 3 mm of 28 mm |
| Tilt, standing | 0.00° | under 1° |
| Foot slip | 1.4 to 1.9 mm | under 5 mm |
| Peak joint torque | 0.63 to 0.72 N·m, the largest on any 2 ms step | under 1.47 N·m |
| Stability margin | 170 to 186 mm | positive |

All twelve body poses, and a combined roll, pitch and yaw pose, are reached with six feet
planted. 196 tests pass. (The first version of this table said 0.27 to 0.30 N·m: torque was
sampled once per control tick and missed the spike after each new target by about 2×. The
whole-branch review caught it.)

## References

| Source | Used here for |
|---|---|
| [07 — Locomotion](../../07-locomotion.md) | Body-to-leg transform, body pose formula, stand and sit, stability margin |
| [08 — Mechanical design](../../08-mechanical-design.md) | Mount positions, body outline, mass budget |
| [06 — Simulation](../../06-simulation.md) | Time step, contact choices, simulated servo |
| Version 0 code (`bemu_kinematics`, `bemu_description`, `bemu_sim`) | Leg maths, model generator and headless harness this builds on |
