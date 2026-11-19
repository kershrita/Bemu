# Version 1a — Full Simulated Body: Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The full 18-joint simulated Bemu stands up, sits down and holds body poses (height, shift, roll, pitch, yaw) without a foot slipping, tested headless without ROS.

**Architecture:** Pure body-pose maths in `bemu_kinematics` (planted feet in the ground frame, through the leg mounts, into version 0's inverse kinematics); a generated 18-joint MuJoCo model sharing the leg chain with version 0's one-leg generator; a headless `RobotSim` like `LegSim`; behaviour proven in simulation.

**Tech Stack:** Python 3.12+, numpy, PyYAML, pytest, ruff, MuJoCo (pip). No ROS imports anywhere in this plan.

**Spec:** [docs/superpowers/specs/2026-11-18-v1a-full-body-design.md](../specs/2026-11-18-v1a-full-body-design.md). Supporting: [07 — Locomotion](../../07-locomotion.md), [08](../../08-mechanical-design.md), [06](../../06-simulation.md), [14](../../14-testing-and-validation.md).

## Global Constraints

- SI inside code (m, rad, s, kg, N·m); mm, deg, g appear only in `config/robot_params.yaml`.
- No dimension, mass or limit is typed in code or tests except through `config/robot_params.yaml`. Only the leg codes `lf lm lr rf rm rr` and the joint names `<leg>_<coxa|femur|tibia>_joint` are fixed names.
- Body frame: origin at the body centre on the hip plane, x forward, y left, z up. Ground frame: fixed to the floor. Leg frame as in version 0.
- Pose rotation: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`. Planted foot = foot sphere *centre*, `leg.foot_radius` above the floor.
- Heights: hip plane at **98 mm** standing (`stance.body_height` + `leg.foot_radius`), **28 mm** sitting (`body.size` z ÷ 2).
- Physics step 2 ms, control rate 50 Hz. Actuator values in `actuators.yaml` do not change unless a documented modelling error is found.
- Actuator torque limit (1.47 N·m) is never raised to make a test pass.
- New maths and the sim import nothing from ROS. Python formatted and linted with `ruff` (line length 100); public functions carry type hints.
- Commit messages: imperative summary line, then the reason. Branch `claude/exciting-tesla-694elw`.

## Review Focus

1. A `BodyPose` with NaN, infinity or `|roll|`, `|pitch|` ≥ 90° → `ValueError`, never a wrong answer. (Task 2)
2. A `feet` mapping missing a leg, or holding a non-finite point → error naming the leg. (Task 2)
3. A body thickness whose sit pose is unreachable, or a non-positive `foot_radius` → `ParamsError` naming the key; the loader and the real inverse kinematics agree on where the boundary is. (Task 1)
4. `RobotSim.command` given a missing leg or a non-finite angle → `ValueError` before any stepping; simulation state unchanged. (Task 6)
5. Fewer than three feet, or collinear feet, in the stability margin → `-inf`, not an exception or NaN. (Task 4)

---

## File Structure

```
config/robot_params.yaml                          + leg.foot_radius, body.size, body.mass
ros2_ws/src/bemu_params/bemu_params/params.py     + BodyParams, LegParams.foot_radius, sit check
ros2_ws/src/bemu_kinematics/bemu_kinematics/
    body.py            new: BodyPose, pose_angles, neutral_feet, foot_ground_position, BodyPoseError
    sequences.py       new: stand/sit heights, stand_up_targets, sit_down_targets, pose_transition
    stability.py       new: stability_margin
ros2_ws/src/bemu_description/bemu_description/
    mjcf.py            modified: leg chain extracted, foot radius from params
    robot_mjcf.py      new: build_robot_mjcf, write_robot_mjcf
    names.py           modified: root, body, joint order
ros2_ws/src/bemu_sim/bemu_sim/
    robot.py           new: RobotSim, play
    report.py          new: PoseReport, measure
tools/stand.py                                    new
tests next to each package, as in version 0
```

---

### Task 1: Parameters — foot radius, body size and mass, sit check

**Files:**
- Modify: `config/robot_params.yaml`, `ros2_ws/src/bemu_params/bemu_params/params.py`, `ros2_ws/src/bemu_params/bemu_params/__init__.py`
- Test: `ros2_ws/src/bemu_params/test/test_params.py`, `ros2_ws/src/bemu_kinematics/test/test_sit_consistency.py`

**Interfaces:**
- Consumes: the existing loader and its helpers (`_number`, `_get`, `ParamsError`).
- Produces:
  ```python
  # LegParams gains a last field:   foot_radius: float   # m (the foot sphere)
  @dataclass(frozen=True)
  class BodyParams:
      size: tuple[float, float, float]   # m (x, y, z); z is the thickness
      mass: float                        # kg, chassis: everything except the six legs
  # RobotParams gains a last field:  body: BodyParams
  ```
  YAML: `leg.foot_radius: 8`, `body.size: {x: 240, y: 160, z: 56}`, `body.mass: 1390`. `BodyParams` is exported from `bemu_params`.

- [ ] **Step 1: Write the failing tests** in `test_params.py`:
  - `test_body_and_foot_load_in_si`: `params.leg.foot_radius == 0.008`, `params.body.size == (0.240, 0.160, 0.056)`, `params.body.mass == 1.39`.
  - Extend the existing `test_bad_file_raises_params_error_naming_the_key` cases with: `("body.size.z", 30, "body.size")`, `("body.size", None, "body.size")`, `("body.mass", -1, "body.mass")`, `("leg.foot_radius", "x", "leg.foot_radius")`, `("leg.foot_radius", 0, "leg.foot_radius")`.
  In `bemu_kinematics/test/test_sit_consistency.py` (it may import both packages):
  - `test_loader_agrees_with_inverse_kinematics_on_the_sit_pose`: for body thickness `t` in 30…80 mm step 2, write an edited copy of the real file; the loader accepts it exactly when `inverse(leg, (stance.foot_radius, 0.0, leg.foot_radius - t/2))` succeeds with limits on. Assert both outcomes occur in the range (at least one accepted, one rejected) and that 48 mm is accepted, 46 mm rejected.

- [ ] **Step 2: Run to verify they fail** — `pytest ros2_ws/src/bemu_params ros2_ws/src/bemu_kinematics/test/test_sit_consistency.py -q`. Expected: FAIL (`foot_radius`, `body` missing).

- [ ] **Step 3: Implement** the new fields, `_parse_body`, and the sit check. The check lives in `params.py` and may not import `bemu_kinematics` (that package imports `bemu_params`; a cycle). It computes the knee-up femur and tibia angles in closed form for the foot centre at `(stance.foot_radius, 0, leg.foot_radius − size_z/2)` and rejects the file if the reach or any joint limit is violated, with a message containing `body.size`. Wire `foot_radius` into `_parse_leg`. Update `config/robot_params.yaml`.

- [ ] **Step 4: Run** the same command plus `pytest -q`. Expected: all PASS (version 0's tests still green).

- [ ] **Step 5: Commit** — `git add config ros2_ws/src/bemu_params ros2_ws/src/bemu_kinematics/test && git commit -m "Add foot radius, body size and mass, and check the sit pose is reachable"`

---

### Task 2: Body-pose kinematics

**Files:**
- Create: `ros2_ws/src/bemu_kinematics/bemu_kinematics/body.py`
- Modify: `ros2_ws/src/bemu_kinematics/bemu_kinematics/__init__.py` (export the new names)
- Test: `ros2_ws/src/bemu_kinematics/test/test_body.py`

**Interfaces:**
- Consumes: `forward`, `inverse`, `JointAngles`, `KinematicsError` (version 0); `RobotParams`, `LEG_CODES` from `bemu_params`.
- Produces:
  ```python
  @dataclass(frozen=True)
  class BodyPose:
      height: float          # m, hip plane above the floor
      x: float = 0.0         # m, ground frame
      y: float = 0.0
      roll: float = 0.0      # rad
      pitch: float = 0.0
      yaw: float = 0.0

  class BodyPoseError(KinematicsError):
      leg: str               # the leg that cannot reach
      cause: KinematicsError # the leg-level error

  def stand_height(params: RobotParams) -> float            # stance.body_height + leg.foot_radius
  def neutral_feet(params: RobotParams) -> dict[str, np.ndarray]   # leg -> (3,) foot centre, ground frame, body at x=y=yaw=0
  def pose_angles(params: RobotParams, pose: BodyPose,
                  feet: Mapping[str, ArrayLike] | None = None) -> dict[str, JointAngles]   # keys in LEG_CODES order
  def foot_ground_position(params: RobotParams, leg_code: str, pose: BodyPose,
                           angles: JointAngles) -> np.ndarray  # (3,), ground frame
  ```
  Maths: `P = Rᵀ·(F − t)`, `p = Rz(−ψ)·(P − m)`, `angles = inverse(leg, p)`; `foot_ground_position` is the exact inverse chain. `feet=None` means `neutral_feet`.

- [ ] **Step 1: Write the failing tests** (`test_body.py`, fixture `params`):
  - `test_neutral_pose_gives_every_leg_the_version_0_neutral_angles`: `pose_angles(params, BodyPose(stand_height(params)))` returns, for all six legs, `(0.0, 0.300494, -1.753631)` within 1e-6.
  - `test_planted_feet_stay_planted_for_poses`: for ≥8 poses (height ±15 mm, shift ±20 mm in x and y, roll and pitch ±10°, yaw ±15°, and one combined), `foot_ground_position(params, leg, pose, angles[leg])` equals `neutral_feet(params)[leg]` within 1e-6 m, for all six legs.
  - `test_feet_override_is_used`: with one foot moved 10 mm in x, only that leg's angles change and its ground position follows.
  - `test_unreachable_pose_names_the_leg`: `BodyPose(height=0.5)` raises `BodyPoseError` whose `.leg` is in `LEG_CODES` and whose message contains it; `.cause` is a `KinematicsError`.
  - Review Focus 1: parametrised `BodyPose` with NaN height, `inf` x, `roll=math.pi/2`, `pitch=-math.pi/2` → `ValueError`.
  - Review Focus 2: `feet` missing `"rr"` → error mentioning `rr`; a foot `[nan, 0, 0.008]` → error mentioning its leg; both are `ValueError`.

- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_kinematics/test/test_body.py -q`. Expected: FAIL (no `body` module).

- [ ] **Step 3: Implement** `body.py`. Validate the pose (finite; `|roll|`, `|pitch|` < π/2) and `feet` before solving; wrap a leg's `KinematicsError` in `BodyPoseError` naming the leg.

- [ ] **Step 4: Run** the file, then `pytest -q`. Expected: all PASS.

- [ ] **Step 5: Commit** — `git add ros2_ws/src/bemu_kinematics && git commit -m "Add body-pose kinematics: planted feet through the leg mounts"`

---

### Task 3: Stand, sit and pose-transition sequences

**Files:**
- Create: `ros2_ws/src/bemu_kinematics/bemu_kinematics/sequences.py`
- Modify: `ros2_ws/src/bemu_kinematics/bemu_kinematics/__init__.py`
- Test: `ros2_ws/src/bemu_kinematics/test/test_sequences.py`

**Interfaces:**
- Consumes: `BodyPose`, `pose_angles`, `stand_height` (Task 2).
- Produces:
  ```python
  def sit_height(params: RobotParams) -> float                       # body.size z / 2
  def ramp(s: float) -> float            # 0→1 for s in [0,1], zero slope at both ends, ramp(0.5) == 0.5; clamps outside
  def stand_up_targets(params, *, duration: float = 2.0, rate: float = 50.0) -> list[dict[str, JointAngles]]
  def sit_down_targets(params, *, duration: float = 2.0, rate: float = 50.0) -> list[dict[str, JointAngles]]
  def pose_transition(params, start: BodyPose, end: BodyPose, *, duration: float = 1.0,
                      rate: float = 50.0) -> list[dict[str, JointAngles]]
  ```
  Each list has `round(duration * rate) + 1` entries (first = `start`, last = `end`); `stand_up_targets` goes from `BodyPose(sit_height)` to `BodyPose(stand_height)`; `sit_down_targets` is its reverse; `pose_transition` interpolates every pose field with `ramp`.

- [ ] **Step 1: Write the failing tests** (`test_sequences.py`):
  - `test_ramp_properties`: `ramp(0)==0`, `ramp(1)==1`, `ramp(0.5)==0.5`, monotonic on a grid, slope within 1e-9·n of zero at both ends, clamps outside [0,1].
  - `test_stand_up_starts_sitting_and_ends_standing`: length 101; `targets[0] == pose_angles(params, BodyPose(sit_height(params)))` and `targets[-1] == pose_angles(params, BodyPose(stand_height(params)))` (compare angle by angle, 1e-9).
  - `test_midpoint_matches_the_pose_at_the_midpoint_height`.
  - `test_sit_down_is_stand_up_reversed`.
  - `test_pose_transition_endpoints_and_length`: roll 0→10° over 1 s: 51 entries, endpoints exact.
  - `test_invalid_duration_or_rate_is_rejected`: duration ≤ 0 or rate ≤ 0 → `ValueError`.

- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_kinematics/test/test_sequences.py -q`. Expected: FAIL.

- [ ] **Step 3: Implement** `sequences.py`; `ramp` is a smoothstep (any profile with the stated properties).

- [ ] **Step 4: Run** the file, then `pytest -q`. Expected: all PASS.

- [ ] **Step 5: Commit** — `git add ros2_ws/src/bemu_kinematics && git commit -m "Add stand, sit and pose-transition sequences"`

---

### Task 4: Stability margin

**Files:**
- Create: `ros2_ws/src/bemu_kinematics/bemu_kinematics/stability.py`
- Modify: `ros2_ws/src/bemu_kinematics/bemu_kinematics/__init__.py`
- Test: `ros2_ws/src/bemu_kinematics/test/test_stability.py`

**Interfaces:**
- Produces: `def stability_margin(com: ArrayLike, feet: ArrayLike) -> float` — `com` is `(2,)` or `(3,)`, `feet` is `(n, 2)` or `(n, 3)`; only x and y are used. Returns the shortest distance from the point to an edge of the convex hull of the feet, positive inside, 0 on an edge, negative outside; `-inf` when fewer than three distinct feet or all collinear.

- [ ] **Step 1: Write the failing tests**: unit square (side 0.2) with the point at the centre → 0.1; point on an edge → 0 (abs 1e-12); point 0.05 outside → −0.05; extra interior feet change nothing; feet given in any order give the same result; 3-D inputs use x, y only; Review Focus 5: two feet, one foot, three collinear feet, duplicates → `-inf`.

- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_kinematics/test/test_stability.py -q`. Expected: FAIL.

- [ ] **Step 3: Implement** with a convex hull of the feet; for a point outside, the margin is minus the distance to the polygon.

- [ ] **Step 4: Run** the file, then `pytest -q`. Expected: all PASS.

- [ ] **Step 5: Commit** — `git add ros2_ws/src/bemu_kinematics && git commit -m "Add the stability margin"`

---

### Task 5: The 18-joint MuJoCo model

**Files:**
- Create: `ros2_ws/src/bemu_description/bemu_description/robot_mjcf.py`
- Modify: `ros2_ws/src/bemu_description/bemu_description/mjcf.py` (extract the leg chain; read the foot radius from `params.leg.foot_radius`, delete the `FOOT_RADIUS` constant), `.../names.py`, `ros2_ws/src/bemu_description/bemu_description/generate.py`, `tools/generate_model.py`
- Test: `ros2_ws/src/bemu_description/test/test_robot_mjcf.py`

**Interfaces:**
- Consumes: `RobotParams`, `ActuatorModel`, `pose_angles`, `BodyPose`, `stand_height`, `sit_height`, `neutral_feet` (Tasks 1–3).
- Produces:
  ```python
  # names.py
  ROOT_JOINT = "root"; BODY = "body"
  def joint_order() -> tuple[str, ...]          # all 18 joint names: LEG_CODES order, coxa, femur, tibia per leg
  # robot_mjcf.py
  def build_robot_mjcf(params: RobotParams, actuators: ActuatorModel) -> str
  # generate.py
  def write_robot_mjcf(params: RobotParams, actuators: ActuatorModel, out_dir: Path) -> Path   # out_dir/robot.xml
  ```
  The model: a `body` box (`body.size`, `body.mass`) on a free joint named `root`, centred on the hip plane; six leg chains (the extracted version 0 chain) at each mount position and yaw, in the body frame; 18 `<position>` actuators named exactly as their joints with `actuators.yaml` values; sites `<leg>_hip` and `<leg>_foot` for every leg; keyframes `sit` and `stand` holding `qpos` (7 for the root, then 18 angles in `joint_order()`) and `ctrl`. Only the body box and the foot spheres collide (with the floor); legs do not collide yet. `tools/generate_model.py --robot` writes `robot.xml`.

- [ ] **Step 1: Write the failing tests** (`test_robot_mjcf.py`, with `mujoco`):
  - `test_names_and_counts`: all `joint_order()` joints and same-named actuators exist; `model.nq == 25`, `model.nv == 24`, `model.nu == 18`; keyframes `sit` and `stand` exist.
  - `test_total_mass_is_the_design_mass`: the `body` subtree mass is `body.mass + 6 × (leg masses)` within 1e-9, i.e. 2.5 kg.
  - `test_every_foot_matches_the_maths_at_stand`: reset to `stand`, `mj_forward`: each `<leg>_foot` site position equals `neutral_feet(params)[leg]` within 1e-6 m; root z equals `stand_height(params)`. This fixes every mount offset, yaw and axis direction.
  - `test_sit_keyframe_rests_on_the_floor`: at `sit` the root z equals `sit_height(params)` and the lowest point of the body box is at the floor within 1e-6 m.
  - `test_joint_ranges_and_force_limits_come_from_params`: each leg's three ranges equal the parameter limits; every actuator `forcerange` is ±`actuators.max_torque`.
  - `test_one_leg_generator_is_unchanged`: `build_leg_mjcf` still loads and version 0's `test_mjcf.py` passes untouched.
  - `test_write_robot_mjcf_creates_the_file(tmp_path)`.

- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_description -q`. Expected: FAIL (`robot_mjcf` missing).

- [ ] **Step 3: Implement**: extract the leg chain from `build_leg_mjcf` into a helper both generators call, then `build_robot_mjcf`. Free-joint placement and a tiny keyframe clearance (so `sit` starts at rest) are your call; keep the asserted heights exact.

- [ ] **Step 4: Run** `pytest -q && python tools/generate_model.py --robot`. Expected: all PASS; `robot.xml` is written.

- [ ] **Step 5: Commit** — `git add ros2_ws/src/bemu_description tools/generate_model.py && git commit -m "Generate the 18-joint robot model from the parameter file"`

---

### Task 6: `RobotSim`

**Files:**
- Create: `ros2_ws/src/bemu_sim/bemu_sim/robot.py`
- Modify: `conftest.py` (add a session fixture `robot_mjcf` = `build_robot_mjcf(params, actuators)`)
- Test: `ros2_ws/src/bemu_sim/test/test_robot_sim.py`

**Interfaces:**
- Consumes: `build_robot_mjcf`, `joint_order`, `ROOT_JOINT`, `BODY`, `foot_site`, `hip_site` (Task 5); `JointAngles`, `neutral_feet`.
- Produces:
  ```python
  CONTROL_RATE: float = 50.0   # re-use bemu_sim.headless.CONTROL_RATE, do not redefine
  class RobotSim:
      def __init__(self, mjcf: str) -> None                              # starts at keyframe "sit"
      def reset(self, keyframe: str = "sit") -> None
      def command(self, targets: Mapping[str, JointAngles]) -> None      # all six legs; one control period
      def body_position(self) -> np.ndarray                              # (3,), hip-plane centre, ground frame
      def body_euler(self) -> tuple[float, float, float]                 # roll, pitch, yaw (Rz·Ry·Rx)
      def tilt(self) -> float                                            # rad between body z-axis and vertical
      def joint_angles(self) -> dict[str, JointAngles]
      def joint_torques(self) -> dict[str, np.ndarray]                   # (3,) N·m per leg
      def foot_positions(self) -> dict[str, np.ndarray]                  # (3,), ground frame, foot centre
      def foot_contacts(self) -> dict[str, bool]                         # foot touching the floor
      def center_of_mass(self) -> np.ndarray                             # (3,)
      @property
      def time(self) -> float                                            # s of simulated time
  def play(sim: RobotSim, targets: Iterable[Mapping[str, JointAngles]]) -> float   # returns peak |torque| over the run, N·m
  ```

- [ ] **Step 1: Write the failing tests**:
  - `test_starts_sitting_at_rest`: after construction, root height equals `sit_height` within 1 mm, tilt < 0.5°, all six `foot_contacts` true.
  - `test_reset_to_stand_puts_feet_on_the_neutral_points`: `reset("stand")`: `foot_positions()` equal `neutral_feet` within 1e-6 m.
  - `test_body_euler_reads_back_a_known_orientation`: `reset("stand")`, write a roll/pitch/yaw quaternion into the free joint, `mj_forward`; `body_euler()` returns it within 1e-9 (uses the same `Rz·Ry·Rx` convention as the maths).
  - `test_command_advances_one_control_period`: `sim.time` rises by 0.02 s.
  - Review Focus 4: a mapping missing `"rm"`, one with an extra key, and one with a NaN angle each raise `ValueError` and leave the state (angles, time) unchanged.
  - `test_play_returns_peak_torque`: playing two held targets returns a float ≥ 0.

- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_sim/test/test_robot_sim.py -q`. Expected: FAIL.

- [ ] **Step 3: Implement** `robot.py` (add a `time` property too). Foot contact = a contact between the foot's geom and the floor.

- [ ] **Step 4: Run** the file, then `pytest -q`. Expected: all PASS.

- [ ] **Step 5: Commit** — `git add conftest.py ros2_ws/src/bemu_sim && git commit -m "Add the headless full-robot simulation"`

---

### Task 7: Stand, sit and pose in simulation; `tools/stand.py`

**Files:**
- Create: `ros2_ws/src/bemu_sim/bemu_sim/report.py`, `tools/stand.py`
- Test: `ros2_ws/src/bemu_sim/test/test_stand.py`, `ros2_ws/src/bemu_sim/test/test_pose.py`, `ros2_ws/src/bemu_sim/test/test_stand_tool.py`

**Interfaces:**
- Consumes: Tasks 2–6.
- Produces:
  ```python
  @dataclass(frozen=True)
  class PoseReport:
      height: float; tilt: float; roll: float; pitch: float; yaw: float   # m, rad
      planted_feet: int            # feet touching the floor
      max_foot_slip: float         # m, largest horizontal distance of any foot from its planted point
      peak_torque: float           # N·m
      stability_margin: float      # m
  def measure(sim: RobotSim, planted: Mapping[str, ArrayLike], peak_torque: float) -> PoseReport
  ```
  `tools/stand.py` — `python tools/stand.py {stand|sit|pose} [--height MM] [--shift X Y] [--roll DEG] [--pitch DEG] [--yaw DEG] [--viewer]`; prints a `PoseReport`; `--viewer` plays in MuJoCo's viewer with the same exit workaround as `tools/trace_foot.py`; one-line error and exit 1 on bad input.

- [ ] **Step 1: Write the failing tests** (a settle step is 50 held commands = 1 s):
  - `test_stand_up` (`test_stand.py`): from `sit`, `play(stand_up_targets)` then settle; report: `|height − stand_height| < 3 mm`, tilt < 1°, `planted_feet == 6`, `max_foot_slip < 5 mm` against `neutral_feet`, `peak_torque < actuators.max_torque`.
  - `test_sit_down`: stand, then `sit_down_targets`, settle: `|height − sit_height| < 3 mm`, tilt < 1°, `planted_feet == 6`.
  - `test_pose_is_reached_without_slipping` (`test_pose.py`, parametrised, from standing, `pose_transition` over 1 s then settle): height ±15 mm; shift +20 mm and −20 mm in x and in y; roll ±10°; pitch ±10°; yaw ±15°. Each: height within 3 mm, x/y within 3 mm of commanded, roll/pitch/yaw within 1° of commanded, `planted_feet == 6`, `max_foot_slip < 5 mm`, `stability_margin > 0`, `peak_torque < actuators.max_torque`.
  - `test_stand_tool_prints_a_report` (`test_stand_tool.py`, loads the tool like `test_tools.py` does): `stand` prints the height line and returns 0; `pose --roll 95` prints one error line and returns 1.
  If a test fails, diagnose (model mistake, or a real reach or torque limit); do not loosen the numbers or raise the torque limit — report a real limit as a finding.

- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_sim -q`. Expected: FAIL (`report` and `tools/stand.py` missing).

- [ ] **Step 3: Implement** `report.measure` and `tools/stand.py` (the trace tool in `tools/trace_foot.py` is the pattern for parsing, error lines and the viewer).

- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check . && python tools/stand.py stand`. Expected: all PASS, clean lint, a report with height ≈ 98 mm.

- [ ] **Step 5: Commit** — `git add ros2_ws/src/bemu_sim tools/stand.py && git commit -m "Stand, sit and hold body poses in simulation"`

---

### Task 8: Documentation and handoff

**Files:**
- Modify: `docs/06-simulation.md`, `docs/07-locomotion.md`, `docs/08-mechanical-design.md`, `docs/13-dev-environment.md`, `docs/14-testing-and-validation.md`, `docs/03-roadmap.md`, `docs/superpowers/specs/2026-11-18-v1a-full-body-design.md`

- [ ] **Step 1: Update the docs**: 06 — self-collision is deferred to 1b, the full model's contents; 07 — stand-up starts from the `sit` keyframe, planted foot = foot centre, the two heights; 08 — body block 240 × 160 × 56 mm, foot radius, clearance; 13 — daily-workflow rows for `tools/stand.py` and `generate_model.py --robot`; 14 — the new tests; 03 — mark version 1 steps 1.1–1.3 done in a note under the step table; the spec — status "implemented", with the final measured numbers (stand height, peak torque, slip) in a short results section.
- [ ] **Step 2: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: all PASS.
- [ ] **Step 3: Commit** — `git add docs && git commit -m "Document version 1a"`; push the branch and confirm the GitHub `python` and `ros` jobs are green.
