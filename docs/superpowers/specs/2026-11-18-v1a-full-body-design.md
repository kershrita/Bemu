# Version 1a — The full simulated body: model, stand and sit, body pose

**Date:** 2026-11-18
**Status:** draft, awaiting the owner's approval
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
as `Rz(yaw) · Ry(pitch) · Rx(roll)`). The **planted foot** of leg `i` is a point
`F_i` on the ground. For standing, `F_i` is the neutral foot position: the mount
point plus `foot_radius` along the leg's resting direction, at `z = 0`.

For any pose the joint angles follow from three steps already in
[07 — Locomotion](../../07-locomotion.md):

```
P_i  = Rᵀ · (F_i − t)               planted foot, seen from the body
p_i  = Rz(−ψ_i) · (P_i − m_i)       the same point in leg i's frame
angles_i = inverse(leg, p_i)        version 0's inverse kinematics
```

`F_i` is an input, not a constant, so 1b can pass moving feet through the same code.

### 5.2 Body size and the sit pose

New parameter-file entries: `body.size` (x, y, z in mm) and `body.mass` (g, the
chassis: everything except the six legs). Starting values: **240 × 160 × 40 mm** and
**1,390 g**, so chassis plus six legs (185 g each, from the existing link masses)
is the 2,500 g design mass in [08](../../08-mechanical-design.md).

The body is a box centred on the hip plane. Resting on the floor, the hip plane is
therefore 20 mm up. This is not free: with the feet at the neutral radius on the
floor, the femur needs **85.6°** at a 20 mm hip height. Checked against the real
leg maths: it fails at 15 mm (90.4°, past the 90° limit) and passes at 16 mm, so a
thinner body than about 32 mm cannot sit. The 40 mm choice leaves 4.4° of margin.
The loader rejects a body thickness for which the sit pose is unreachable, with an
error naming `body.size`.

### 5.3 Stand and sit

The sim's initial state is the **`sit`** keyframe: body resting on the floor, feet
at their neutral planted positions. (This makes the first step in the locomotion
doc, "move each foot to its neutral position at ground level", already true; the
real robot's power-on posture is a version 3 concern.)

- **Stand up:** ramp `h` from the sit height to the standing height (90 mm) over
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
| Parameters | Body size and mass load in SI; bad or unreachable-sit body size rejected naming the key | The sit-pose limit above |
| Body maths | Neutral pose gives each leg the angles that put its foot at the neutral radius | Mount position and yaw per leg |
| Body maths | For poses with height, shift, roll, pitch and yaw, forward kinematics through each mount returns the planted foot in the ground frame within 1e-6 m, for all six legs | Sign and order errors in the transforms |
| Body maths | A pose a leg cannot reach raises an error naming that leg | No silent wrong answers |
| Model | 18 named joints and actuators; `sit` and `stand` keyframes; total mass 2.5 kg; each foot site matches the maths at `stand` within 1e-6 m | The model and the maths agree for every leg |
| Sim | From `sit`, stand-up ends within 3 mm of 90 mm, tilt under 1°, all six feet touching, no foot moved more than 5 mm sideways, peak torque under the 1.47 N·m limit | The headline behaviour |
| Sim | Sit-down ends resting on the floor, hip plane within 3 mm of 20 mm | The reverse path |
| Sim | Height, shift (±20 mm), roll and pitch (±10°), yaw (±15°) each reach the commanded pose without a foot slipping more than 5 mm; stability margin stays positive | Body pose control |

These tolerances are the owner-visible bar. If a test fails, the response is to
find the cause (a modelling mistake, or a real torque or reach limit), not to widen
the number.

## 7. Deliverables

- `bemu_params`: body size and mass, with the sit-reachability check.
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
| The sit pose is only 4.4° from the femur limit | Covered by the loader check and a test; thicker bodies in version 1.5 CAD must be re-checked |

## References

| Source | Used here for |
|---|---|
| [07 — Locomotion](../../07-locomotion.md) | Body-to-leg transform, body pose formula, stand and sit, stability margin |
| [08 — Mechanical design](../../08-mechanical-design.md) | Mount positions, body outline, mass budget |
| [06 — Simulation](../../06-simulation.md) | Time step, contact choices, simulated servo |
| Version 0 code (`bemu_kinematics`, `bemu_description`, `bemu_sim`) | Leg maths, model generator and headless harness this builds on |
