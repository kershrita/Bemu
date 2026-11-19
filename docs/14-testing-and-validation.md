# 14 — Testing and validation

How each layer is checked, and how the simulation is compared with the real robot.

## Principles

1. **Test the maths without the robot.** Geometry and gait logic are pure functions
   and are tested directly, in milliseconds.
2. **Test behaviour in simulation, automatically.** Each version's exit test is a
   script that runs without a screen.
3. **Test hardware with checklists and measurements.** What cannot be automated is
   written down as a procedure with pass criteria.
4. **A test is written before the thing it tests.**
5. **A failure found on the robot becomes a simulated test** where possible, so it
   cannot return unnoticed.

## Levels

```mermaid
flowchart TB
  U["Unit tests<br/>maths and logic, no ROS<br/>seconds"] --> I["Integration tests<br/>nodes and message contracts<br/>tens of seconds"]
  I --> S["Simulation tests<br/>whole robot in MuJoCo, no screen<br/>minutes"]
  S --> H["Hardware tests<br/>procedures and measurements<br/>manual"]
```

### Unit tests

| Subject | Examples |
|---|---|
| Forward and inverse kinematics | Inverse then forward returns the starting point for thousands of random reachable targets; known poses give known angles |
| Reach and limit handling | Every kind of unreachable target is reported, never silently wrong |
| Body transform | Raising the body by 10 mm lowers every foot target by 10 mm in the body frame |
| Gait timing | In tripod gait exactly three feet are in stance at all times; phases wrap correctly |
| Foot paths | Stance path is a straight line of the right length; swing clears the step height; paths join without a jump |
| Stride scaling | An over-long stride is scaled for all legs together |
| Stability margin | Known foot layouts give known margins |
| Parameter file | Loads, validates ranges, converts units |

Tool: pytest. Property-based tests generate the random cases.

### Integration tests

| Subject | Examples |
|---|---|
| Walker node | A velocity command in produces 18 joint targets out, at the right rate, in the right order |
| Command timeout | Stopping the commands stops the walking |
| Launch | The launch file starts every expected node, in both modes |
| Contracts | Each topic in [05 — Software stack](05-software-stack.md) carries the documented type |

Tool: launch_testing.

### Simulation tests

Each roadmap exit test that can be simulated is a script. Examples from version 1:

| Test | Pass criterion |
|---|---|
| Straight walk, 2 m | Sideways drift under 10% |
| Turn in place, 360° | Ends within 50 mm of the start |
| Body pose while walking | No foot slips more than 5 mm |
| Gait switching | No fall, no joint limit hit |
| Long run, 5 minutes | No drift into instability; no error messages |
| Limits | No joint exceeds its range; no torque exceeds the limit |

Version 1a's simulation tests (`bemu_sim/test/test_stand.py`, `test_pose.py`): stand up to
within 3 mm of 98 mm with under 1° tilt, six feet down, foot slip under 5 mm; sit down to
within 3 mm of 28 mm; hold twelve poses (height ±15 mm, shift ±20 mm, roll and pitch ±10°,
yaw ±15°, and one combined roll, pitch and yaw pose that separates the rotation orders) with
the feet planted and a positive stability margin; torque, taken on every physics step, under
the limit.

Version 1b-1's walking tests (`bemu_sim/test/test_walk_*.py`, `test_collisions.py`): walk 2 m
straight with under 10% sideways drift; turn 360° in place and end within 50 mm; walk
sideways and in a curve; hold body pose changes (height ±10 mm, roll and pitch ±5°) while
walking with no foot slipping more than 5 mm; walk straight in the ripple and the wave;
switch between every ordered pair of gaits, and tripod to ripple to wave and back, without
stopping and with the stability margin above 15 mm; leg-to-leg contact; and torque, taken
on every physics step, under the limit in all of them.

Version 1b-2's tests: headless, the locomotion core (joint matching by name, the stand and sit
sequences, the command timeout, rejected bad commands and unreachable poses), the quaternion
conversion, the 18-joint URDF and controller file, and the message files; with ROS (`colcon
test`, run in the CI `ros` job), the node's topics, services and bad-input handling and the launch
description; and on the development PC only (`sim_launch`), `robot.launch.py` bringing up every
controller, standing the robot and walking it on a `/cmd_vel`.

Version 1c's tests: headless, the key and stick mapping (speed steps and limits, gait clamping,
height limits, deadzone, bad axis values, once-per-press buttons) and the keyboard's refusal
without a terminal; with ROS, the gamepad node and the `teleop` launch choices.

Version 1.5b's tests add: link masses from the CAD and the guard that keeps `robot_params.yaml` equal to
them (and to the CAD body thickness and the 2.5 kg design mass), the coxa mount, and the body (coverage,
bed fit, real-STEP seating at both faces, no contact with a real coxa servo, swinging-leg sweeps,
splices, the insert grid, the mass budget).

Version 1.5a's tests (`bemu_cad/test`): the component library (every value has a source file
and a valid status; bad or missing values are rejected naming the field); the STEP probe (the hole
table equals the library, and the two manufacturer models agree); the servo solid, pocket and mass
properties; the printed leg (axis distances equal the parameter file, no part penetrates a servo,
screw-hole edge distances, print-bed fit, joint sweeps, and a longer femur regenerating with all
checks passing); the tolerance coupon (pockets and holes cut at the stated sizes); and the
exports and the `generate_cad` tool. They need build123d and are skipped in the ROS image job.

The torque report's tests (`bemu_sim/test/test_robot_stats.py`, `test_torque_report*.py`):
per-step joint statistics (peak, RMS, speed, steps at the limit); the 48-scenario matrix, the
mass and scenario validation, the power model and the pass/fail and stability verdict
boundaries; real short stand and walking runs; the Markdown and JSON rendering with failing and
unstable rows; and the tool's quick run and its one-line error on an unwritable output folder.

Every simulation test also records the measurements listed under the torque report
in [06 — Simulation](06-simulation.md), so trends are visible across changes.

### Hardware tests

| Stage | Procedure | Pass criterion |
|---|---|---|
| Before first power | Continuity and polarity check with a multimeter | No shorts; correct polarity at every connector |
| Single servo | Address set; moves to commanded angles; feedback readable | Angle error under 1° |
| Bus timing | Read and write all servos in a loop | Meets the control rate |
| One leg | Foot traces the 60 mm square | Error under 8 mm |
| Protection | Block a joint by hand | Overload detected; leg relaxes |
| Calibration | Neutral stance commanded; foot positions measured | Within 5 mm of the model |
| Power | Run on battery at full walking load | Pi does not brown out; wiring stays cool |
| Low battery | Lower the bench supply voltage | Warning, then sit-down, at the set voltages |
| Endurance | 15 minutes of walking | No servo over its temperature limit |

## Comparing simulation and reality

The same command sequence is run on both, recorded, and overlaid.

| Comparison | When | Acceptable difference |
|---|---|---|
| Single joint step response | Version 2 | Rise time within 10% |
| Foot path of one leg | Version 2 | Within 5 mm |
| Walking speed for a given command | Version 3 | Within 15% |
| Drift over 2 m | Version 3 | Same direction, same order of size |
| Power draw while walking | Version 3 | Within 25% |
| Body tilt on a slope | Version 4 | Within 2° |

When a difference is outside its limit, the model is corrected. The code is not
tuned separately for each world.

## Recorded data

| What | How |
|---|---|
| Every simulated and real test run | Recorded with `ros2 bag` |
| Torque and stability reports | Generated by a script, saved with the version tag |
| Comparison plots | Generated by a script from two recordings |

Large recordings are kept out of git; the scripts and the summary reports are in.

## Definition of done for a version

- [ ] Every exit test in the roadmap passes.
- [ ] All automated checks pass on the main branch.
- [ ] New behaviour has simulated tests.
- [ ] Documents affected by the version are updated.
- [ ] Estimates replaced by measurements are marked as measured.
- [ ] Decision records exist for the significant choices made.
- [ ] The version is tagged.

## References

No external sources. This document draws only on the owner's decisions and the
other documents in this repository. The project's full list is in
[References](references.md).
