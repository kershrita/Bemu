# Version 1b-1 — Headless Walking: Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The simulated Bemu walks, turns and switches gait while moving, driven by a velocity command, proven headless against the version 1 walking exit tests.

**Architecture:** A new ROS-free package `bemu_gait` holds the gaits, foot paths and a phase-driven `Walker` that outputs 18 joint angles through version 1a's `pose_angles`. The model gets leg-to-leg and leg-to-body collisions. A scenario helper in `bemu_sim` drives `RobotSim` with the walker and measures drift, heading, slip and stability.

**Tech Stack:** Python 3.12+, numpy, PyYAML, pytest, ruff, MuJoCo (pip). No ROS imports in this plan.

**Spec:** [docs/superpowers/specs/2026-11-18-v1b-walking-design.md](../specs/2026-11-18-v1b-walking-design.md) (this plan is its 1b-1 part). Builds on [1a](../specs/2026-11-18-v1a-full-body-design.md); maths in [07 — Locomotion](../../07-locomotion.md).

## Global Constraints

- SI inside code (m, rad, s, kg, N·m); mm only in YAML files, converted by the loader.
- No dimension is typed in code or tests except through `config/robot_params.yaml` (leg and body) or `bemu_gait/config/gait.yaml` (gait values).
- Leg codes `lf lm lr rf rm rr`; joint order is `joint_order()` (1a). Feet are planned in the **nominal frame**: the ground frame with the body at x = y = yaw = 0, foot centres at `leg.foot_radius`; 1a's `pose_angles(params, pose, feet)` turns them into joint angles.
- Control rate 50 Hz, physics step 2 ms. Actuator values in `actuators.yaml` do not change; the 1.47 N·m torque limit is never raised.
- Bars (never loosened): sideways drift under 10% on a 2 m walk; 360° turn ends within 50 mm; no foot slips more than 5 mm in any stance; stability margin above 15 mm; peak torque under the limit on any physics step.
- Gait values (spec §5.2): tripod duty 1/2 offsets 0, 1/2, 0, 1/2, 0, 1/2; ripple duty 2/3 offsets 0, 1/3, 2/3, 2/3, 1/3, 0; wave duty 5/6 offsets 1/2, 2/3, 5/6, 0, 1/6, 1/3 (order LF, LM, LR, RF, RM, RR). Starting values: tripod cycle 1.0 s, step height 30 mm, max stride 60 mm.
- New maths imports nothing from ROS. ruff line length 100; public functions carry type hints. Branch `claude/exciting-tesla-694elw`; commit messages: imperative summary, then the reason.

## Review Focus

1. A non-finite or non-numeric command, or a command beyond reach or max stride → `ValueError` for bad input; for large ones all strides scale down together, the step is never out of reach, and feet never jump more than a few millimetres between ticks. (Task 2)
2. An unknown gait name → `ValueError` naming it and the valid names; a request during a blend replaces the target. (Task 2)
3. `step(dt)` with `dt` ≤ 0, NaN, or above 0.1 s (a stalled control loop) → `ValueError`; feet never teleport. (Task 2)
4. A body pose that makes standing unreachable → `BodyPoseError` at `set_body_pose`; one that only makes stepping unreachable → strides scale toward zero and the robot keeps standing. (Task 2)
5. A bad `gait.yaml` (missing key, non-positive value, unknown gait, cycle time missing for a gait) → `GaitParamsError` naming the key. (Task 1)

---

## File Structure

```
ros2_ws/src/bemu_gait/                         new, ROS-free (package.xml, setup.py, setup.cfg, resource/)
    config/gait.yaml
    bemu_gait/{__init__,gaits,params,command,footpath,walker}.py
    test/test_gaits.py, test_footpath.py, test_walker.py
ros2_ws/src/bemu_description/bemu_description/{mjcf,robot_mjcf}.py   modified: collisions
ros2_ws/src/bemu_sim/bemu_sim/walking.py       new: stand_robot, walk, WalkLog, metrics
ros2_ws/src/bemu_sim/test/test_walk_*.py       new
tools/walk.py                                  new
pyproject.toml (pythonpath, isort), conftest.py (gait_params fixture), docs
```

---

### Task 1: The `bemu_gait` package — gaits, parameters, foot paths

**Files:**
- Create: `ros2_ws/src/bemu_gait/` (package files, `config/gait.yaml`, `bemu_gait/gaits.py`, `params.py`, `command.py`, `footpath.py`, `__init__.py`)
- Modify: `pyproject.toml` (add `ros2_ws/src/bemu_gait` to pytest `pythonpath` and `bemu_gait` to isort `known-first-party`), `conftest.py` (fixture `gait_params` = `load_gait_params()`)
- Test: `ros2_ws/src/bemu_gait/test/test_gaits.py`, `test_footpath.py`

**Interfaces:**
- Consumes: `ramp` (1a, `bemu_kinematics`); `LEG_CODES`.
- Produces:
  ```python
  # gaits.py
  @dataclass(frozen=True)
  class Gait: name: str; duty: float; offsets: Mapping[str, float]     # leg -> phase offset in [0,1)
  TRIPOD: Gait; RIPPLE: Gait; WAVE: Gait; GAITS: dict[str, Gait]      # keys "tripod", "ripple", "wave"
  def feet_down(gait: Gait, master_phase: float) -> int               # legs in stance at that master phase
  # command.py
  @dataclass(frozen=True)
  class Command: vx: float = 0.0; vy: float = 0.0; omega: float = 0.0  # m/s forward, m/s left, rad/s counter-clockwise
  # params.py
  class GaitParamsError(ValueError)
  @dataclass(frozen=True)
  class GaitParams:
      cycle_time: Mapping[str, float]      # s, one per gait name
      step_height: float                   # m
      max_stride: float                    # m, per leg per stance
      max_linear_accel: float              # m/s²
      max_angular_accel: float             # rad/s²
      lift_ramp_stride: float              # m: stride at which the full step height is reached
      blend_cycles: float                  # length of a gait blend, in cycles of the old gait
      stability_threshold: float           # m
  def load_gait_params(path: str | os.PathLike | None = None) -> GaitParams   # default: the package's config/gait.yaml
  # footpath.py
  def stride_vector(vx: float, vy: float, omega: float, neutral_xy: ArrayLike, stance_time: float) -> np.ndarray  # (2,)
  def foot_offset(phase: float, duty: float, stride: ArrayLike, lift: float) -> np.ndarray   # (3,) from the neutral point
  ```
  `gait.yaml` keys: `cycle_time_s: {tripod: 1.0, ripple: 0.75, wave: 0.6}`, `step_height_mm: 30`, `max_stride_mm: 60`, `max_linear_accel_mm_s2: 200`, `max_angular_accel_rad_s2: 1.0`, `lift_ramp_stride_mm: 10`, `blend_cycles: 1.0`, `stability_threshold_mm: 15`.
  Foot path: stance (`phase < duty`, `u = phase/duty`): xy `= stride·(0.5 − u)`, z = 0. Swing (`u = (phase − duty)/(1 − duty)`): xy `= stride·(−0.5 + ramp(u))`, z `= lift·sin(π u)`. Stride `= (v + ω × r)·stance_time`, `ω × r = ω·(−r_y, r_x)`.

- [ ] **Step 1: Write the failing tests.** `test_gaits.py`: each gait's `offsets` and `duty` equal the table in Global Constraints; for 200 master phases off the exact boundaries (e.g. `(k + 0.37)/200`) `feet_down` is 3 (tripod), 4 (ripple), 5 (wave); wave swings one leg at a time, 1/6 of a cycle apart in the order RF, RM, RR, LF, LM, LR. `test_footpath.py`: `foot_offset` is continuous in `phase` across 0/1 wrap, the stance/swing change and within each segment (sample 2000 phases; successive differences under a bound proportional to stride + lift); stance xy is a straight line from `+stride/2` to `−stride/2` at constant speed; z is 0 in stance and peaks at `lift` mid-swing; `stride_vector(0.1, 0, 0, ...)` equals `(0.05, 0)` for `stance_time` 0.5 and every neutral point; `stride_vector(0, 0, 1.0, (0.2, 0.0), 0.5)` equals `(0, 0.1)` (tangent); sideways and mixed cases. Params: `load_gait_params()` returns the YAML values in SI (`step_height == 0.030`, `cycle_time["tripod"] == 1.0`, `max_linear_accel == 0.2`); Review Focus 5 parametrised bad files (missing key, `step_height_mm: -1`, a non-numeric value, `cycle_time_s` missing `wave`, an extra unknown gait name) → `GaitParamsError` matching the key.
- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_gait -q`. Expected: FAIL (no package).
- [ ] **Step 3: Implement** the package files and modules as specified; package metadata like `bemu_params` (with `extras_require={"test": ["pytest"]}` and `config/gait.yaml` in `data_files`).
- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: all PASS.
- [ ] **Step 5: Commit** — `git add pyproject.toml conftest.py ros2_ws/src/bemu_gait && git commit -m "Add bemu_gait: gaits, parameters and foot paths"`

---

### Task 2: The `Walker`

**Files:**
- Create: `ros2_ws/src/bemu_gait/bemu_gait/walker.py`; modify `bemu_gait/__init__.py` to export
- Test: `ros2_ws/src/bemu_gait/test/test_walker.py`

**Interfaces:**
- Consumes: Task 1; `BodyPose`, `BodyPoseError`, `JointAngles`, `neutral_feet`, `pose_angles`, `stand_height`, `stability_margin`, `ramp` (1a); `RobotParams`, `LEG_CODES`.
- Produces:
  ```python
  @dataclass(frozen=True)
  class WalkerOutput:
      angles: dict[str, JointAngles]       # all six legs, ready for RobotSim.command
      feet: dict[str, np.ndarray]          # (3,) each, nominal frame, before the body pose
      phases: dict[str, float]             # each leg's phase in [0,1)
      in_stance: dict[str, bool]
      stability_margin: float              # m: COM at (pose.x, pose.y) against the stance feet's xy
      scale: float                         # stride scale applied, 0..1
      gait: str                            # the gait being walked, or blended toward
      blending: bool
  class Walker:
      def __init__(self, params: RobotParams, gait_params: GaitParams, *, gait: str = "tripod") -> None
      def set_command(self, command: Command) -> None
      def set_gait(self, name: str) -> None              # starts a blend; replaces one in progress
      def set_body_pose(self, pose: BodyPose) -> None    # default BodyPose(stand_height(params))
      def step(self, dt: float) -> WalkerOutput          # advance dt seconds, 0 < dt <= 0.1
  ```
  Behaviour (spec §5.3, §5.4): a master phase advances at `1/T`; leg phase `= (master + offset_i) mod 1`; the applied command slews toward the commanded one within the acceleration limits; stride per leg from `stride_vector` with `stance_time = duty·T`; lift `= step_height · ramp(max_i|stride_i| / lift_ramp_stride)`, so zero command means feet stand at the neutral points; a gait change keeps each leg's phase and slides its offset (shortest way round the circle), the duty factor and `T` linearly over `blend_cycles` old cycles; strides and lift are multiplied by a scale found by bisection as the largest value in [0, 1] that keeps every leg reachable (`pose_angles` does not raise), the stability margin at least `stability_threshold` (at scale 0 even if that cannot be met), and the largest stride at most `max_stride`.

- [ ] **Step 1: Write the failing tests** (`test_walker.py`, fixtures `params`, `gait_params`):
  - `test_zero_command_stands_at_the_neutral_points`: after 100 steps of `dt=0.02` with no command, `feet` equal `neutral_feet(params)` within 1e-9 for all gaits, `scale == 1`, all `in_stance`.
  - `test_forward_command_moves_stance_feet_backwards_at_the_commanded_speed`: `Command(vx=0.1)`, after the slew settles, a stance foot's x velocity is `−0.1` within 5% (finite difference over ticks).
  - `test_turning_gives_tangential_strides`: with `Command(omega=0.4)` the stance foot velocity of each leg equals `−ω × r` within 5%.
  - `test_command_is_slewed`: a step from 0 to `vx=0.12` is not reached in one tick; reached within `0.12/max_linear_accel + 0.1` s.
  - `test_gait_switch_never_jumps_a_foot`: for every ordered pair of gaits, walking at `vx=0.08`, call `set_gait` at an arbitrary tick; across 5 cycles the foot displacement per tick never exceeds 8 mm and the transition takes `blend_cycles` of the old cycle (`blending` true, then false).
  - `test_a_request_during_a_blend_replaces_the_target`: `set_gait("ripple")` then `set_gait("wave")` ends on wave, no jump.
  - `test_unreachable_command_scales_all_strides_together` (Review Focus 1): `Command(vx=1.0, omega=3.0)` → `scale < 1`, `angles` valid every tick (no exception), per-tick foot displacement bounded (under 8 mm), and `max_i|stride_i| <= max_stride` in effect (stance foot speed ≤ `max_stride/(duty·T)` + 5%).
  - `test_bad_commands_are_rejected_and_change_nothing` (Review Focus 1): NaN, `inf`, a string → `ValueError`; the next step equals a walker that never received it.
  - `test_unknown_gait_names_the_valid_ones` (Review Focus 2): `ValueError` whose message contains the name and `tripod`, `ripple`, `wave`.
  - `test_bad_dt_is_rejected` (Review Focus 3): `0`, negative, NaN, `0.2` → `ValueError`; walker state unchanged.
  - `test_unreachable_standing_pose_is_rejected_at_once` (Review Focus 4): `set_body_pose(BodyPose(0.5))` → `BodyPoseError`; a pose reachable standing but not stepping (find one near a limit, e.g. a large shift) leaves `step` working with `scale < 1` and no exception.
  - `test_stability_margin_is_the_planners_estimate`: tripod at neutral: margin `≈ stability_margin((0,0), the three stance feet xy)` within 1e-9.
  - `test_body_pose_offset_is_applied`: with `set_body_pose(BodyPose(h, roll=0.1))`, `angles` equal `pose_angles(params, pose, feet)`.

- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_gait/test/test_walker.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement** `Walker` and `WalkerOutput` as specified. The blend interpolates circular offsets by `wrap(new − old)` into (−0.5, 0.5]. Validate all inputs before changing state.
- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: all PASS.
- [ ] **Step 5: Commit** — `git add ros2_ws/src/bemu_gait && git commit -m "Add the phase-driven walker with gait blending"`

---

### Task 3: Collisions between legs, and between legs and the body

**Files:**
- Modify: `ros2_ws/src/bemu_description/bemu_description/mjcf.py` (`leg_body_xml` gains `link_contact: str = ""`), `robot_mjcf.py`
- Test: `ros2_ws/src/bemu_description/test/test_robot_mjcf.py`, `ros2_ws/src/bemu_sim/test/test_collisions.py`

**Interfaces:**
- Consumes: 1a's model generator and `RobotSim`.
- Produces: the same `build_robot_mjcf(params, actuators) -> str`, with contacts: floor `contype=1 conaffinity=0`; body box `contype=2 conaffinity=3`; leg link capsules `contype=2 conaffinity=2`; foot spheres `contype=2 conaffinity=3`; and `<contact><exclude>` entries for, per leg, body–coxa, coxa–femur, femur–tibia and coxa–tibia. Result: feet and body hit the floor; every other pairing of body and legs collides, except the excluded ones.

- [ ] **Step 1: Write the failing tests.** Replace 1a's `test_only_the_floor_collides_with_the_body_and_the_feet` with: `test_a_leg_hits_a_neighbour_and_the_body` (sim): with `RobotSim`, set joint angles so lf and lm swing into each other (lf coxa +45°, lm coxa −45°, others neutral; adjust to angles that really touch and say which in the test) and assert a contact between a geom of `lf_*` and a geom of `lm_*`; set a leg's femur and tibia so it folds into the body and assert a contact with `body_box`; `test_same_leg_links_never_collide`: at the extreme folded leg pose (tibia at its lower limit, femur at its upper limit) no contact between two geoms of one leg; `test_floor_contacts_are_only_the_body_and_the_feet_at_sit_and_stand`: at keyframes `sit` and `stand` every contact involves `floor`. Add a regression check: the whole 1a suite is unchanged.
- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_description ros2_ws/src/bemu_sim/test/test_collisions.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement** the contact bits and exclusions; thread `link_contact` through `leg_body_xml`. If a legitimate 1a pose touches, fix the exclusion or the model, never the 1a tests.
- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q`. Expected: all PASS, including every 1a stand, sit and pose test unchanged.
- [ ] **Step 5: Commit** — `git add ros2_ws && git commit -m "Switch on leg-to-leg and leg-to-body collisions"`

---

### Task 4: Walking in simulation — straight, turning, sideways

**Files:**
- Create: `ros2_ws/src/bemu_sim/bemu_sim/walking.py`
- Modify: `ros2_ws/src/bemu_sim/package.xml` and `setup.py` (depend on `bemu_gait`)
- Test: `ros2_ws/src/bemu_sim/test/test_walk_straight.py`, `test_walk_turn.py`

**Interfaces:**
- Consumes: `Walker`, `Command`, `GaitParams` (Tasks 1–2); `RobotSim`, `play`; `stand_up_targets`; `stability_margin`.
- Produces:
  ```python
  @dataclass(frozen=True)
  class WalkLog:                      # one row per control tick
      t: np.ndarray                   # (n,) s
      position: np.ndarray            # (n, 3) body, ground frame
      yaw: np.ndarray                 # (n,) rad, unwrapped
      tilt: np.ndarray                # (n,) rad
      margin: np.ndarray              # (n,) m, measured from the sim: COM against the touching feet
      feet: np.ndarray                # (n, 6, 3) ground frame, LEG_CODES order
      in_stance: np.ndarray           # (n, 6) bool, the walker's
      scale: np.ndarray               # (n,)
      gait: list[str]
      peak_torque: float              # N·m, largest on any physics step over the run
  def stand_robot(sim: RobotSim, params: RobotParams) -> None        # stand-up then 1 s settle
  @dataclass(frozen=True)
  class WalkState:                    # what a schedule can see before each tick
      t: float                        # s since the walk began
      position: np.ndarray            # (3,) body, ground frame
      yaw: float                      # rad, unwrapped, relative to the start
      heading_change: float           # same as yaw; kept for readability in tests
  def walk(sim: RobotSim, walker: Walker, seconds: float, *,
           schedule: Callable[[WalkState, Walker], None] | None = None) -> WalkLog
      # each tick: schedule(state, walker) if given (it may set the command, gait or body pose);
      # then out = walker.step(1/CONTROL_RATE); sim.command(out.angles); record the row
  def travelled(log: WalkLog) -> tuple[float, float]    # (forward, lateral) of the body from its start, in the start heading's frame
  def heading_change(log: WalkLog) -> float             # rad, last yaw minus first
  def max_stance_slip(log: WalkLog) -> float            # m: largest ground-frame xy movement of any foot within one contiguous stance interval
  ```

- [ ] **Step 1: Write the failing tests** (a standing robot via `stand_robot`; tripod):
  - `test_walks_2_m_straight_with_under_10_percent_drift`: `Command(vx=0.10)` until the body has travelled 2.0 m (limit 40 s): `forward >= 2.0`, `abs(lateral) < 0.2`, `abs(heading_change) < 0.2` rad (my call: a straight walk should not also turn), minimum measured margin above 15 mm, `peak_torque < actuators.max_torque`, `scale.min() > 0.9`.
  - `test_walk_speed_is_close_to_the_command`: over a 10 s steady stretch the mean forward speed is within 25% of 0.10 m/s (a larger gap is a slipping-feet finding: investigate, do not widen).
  - `test_stopping_stands_still`: after commanding zero and 3 s, body speed under 5 mm/s, six feet touching, height within 3 mm of the walking height.
  - `test_turns_360_degrees_in_place`: `Command(omega=0.4)` until `heading_change` reaches `2π − 0.15` (the test, not the walker, decides when to stop), then zero for 3 s: final `abs(heading_change − 2π) < 10°`, body ends within 50 mm of where it started.
  - `test_walks_sideways`: `Command(vy=0.08)` for 12 s: lateral distance positive and within 25% of `0.08 × (12 − ramp)`; forward drift under 10% of the lateral distance.
  - `test_a_curve`: `Command(vx=0.08, omega=0.2)` for 12 s: heading change within 25% of `0.2 × (12 − ramp)`, margin above 15 mm.
  - `test_walk_log_shapes_and_metrics`: small unit tests of `travelled`, `heading_change` and `max_stance_slip` on hand-built logs (a foot that moves 3 mm in one stance → 0.003).
  - `test_max_stance_slip_while_walking`: over a 10 s tripod walk, `max_stance_slip(log) < 0.005`.
- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_sim/test/test_walk_straight.py ros2_ws/src/bemu_sim/test/test_walk_turn.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement** `walking.py`. If a bar is missed once the code is right, diagnose and tune the gait, touchdown or friction, and write what it took in the commit message; do not loosen a bar. Tuning values live in `gait.yaml`.
- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: all PASS.
- [ ] **Step 5: Commit** — `git add ros2_ws && git commit -m "Walk, turn and sidestep in simulation"`

---

### Task 5: Body pose while walking, gait switching, ripple and wave

**Files:**
- Test: `ros2_ws/src/bemu_sim/test/test_walk_pose.py`, `test_walk_gaits.py`; modify `walking.py`/`gait.yaml` only if a test needs tuning

**Interfaces:**
- Consumes: Task 4's `walk`, `WalkLog`, metrics; `Walker.set_body_pose`, `set_gait`.
- Produces: nothing new; this task proves the rest of the exit tests.

- [ ] **Step 1: Write the failing tests:**
  - `test_body_pose_changes_while_walking_do_not_slip_feet` (parametrised): tripod at `vx=0.08`; mid-walk `set_body_pose` for height ±10 mm, roll ±5°, pitch ±5°, and one combined pose; settle 3 s; measured height and tilt reach the commands (3 mm, 1°); `max_stance_slip(log) < 0.005` across the run; margin above 15 mm; torque under the limit.
  - `test_switching_gaits_while_walking_never_stops`: `vx=0.08`; tripod, then ripple at 8 s, wave at 16 s, tripod at 24 s (run 32 s): the body moves forward at least 5 mm in every 1 s window; measured margin above 15 mm throughout; `scale.min() > 0.9`; tilt under 5°; peak torque under the limit; no exception.
  - `test_each_gait_walks_straight` (parametrised ripple, wave): 1 m at `vx=0.08`, lateral drift under 10% of the distance, margin above 15 mm, slip under 5 mm.
  - `test_every_ordered_gait_pair_switches_safely`: all six ordered pairs, switching mid-walk: margin above 15 mm and no exception.
- [ ] **Step 2: Run** the two files. Expected: FAIL or, where the code already suffices, PASS; for any that pass, break the code temporarily (for example flip a gait's offsets) and confirm the test fails before trusting it.
- [ ] **Step 3: Fix what fails** at its cause; if a gait's cycle time, blend length or acceleration needs tuning, change `gait.yaml` and record why.
- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: all PASS.
- [ ] **Step 5: Commit** — `git add ros2_ws && git commit -m "Prove body pose, gait switching, ripple and wave while walking"`

---

### Task 6: `tools/walk.py`, documentation and handoff

**Files:**
- Create: `tools/walk.py`; test `ros2_ws/src/bemu_sim/test/test_walk_tool.py`
- Modify: `docs/05-software-stack.md`, `06-simulation.md`, `07-locomotion.md`, `13-dev-environment.md`, `14-testing-and-validation.md`, `03-roadmap.md`, `docs/superpowers/specs/2026-11-18-v1b-walking-design.md`

**Interfaces:**
- Consumes: Tasks 1–5.
- Produces: `python tools/walk.py [--gait tripod|ripple|wave] [--vx M/S] [--vy M/S] [--omega RAD/S] [--seconds S] [--viewer]` printing: distance and lateral drift, heading change, minimum stability margin, largest stance slip, peak torque, gait; one-line error and exit 1 on bad input; `--viewer` uses the viewer-exit workaround from `tools/stand.py`.

- [ ] **Step 1: Write the failing tests** (`test_walk_tool.py`, loading the tool like `test_stand_tool.py`): a default tripod run prints the metrics lines and returns 0; `--gait nonsense` prints one error line starting `walk: ` and returns 1; `--vx nan` likewise.
- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_sim/test/test_walk_tool.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement** the tool; then update the docs: 05 (`bemu_gait` in the layout and package table, `gait.yaml` lives there), 06 (collisions now on, with the exclusions), 07 (the blend rule, the stop rule, measured results), 13 (`tools/walk.py` row), 14 (the 1b-1 tests), 03 (steps 1.4 to 1.6 done headless; the ROS walker is 1b-2), the spec (status of the 1b-1 half, with measured numbers).
- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check . && python tools/walk.py --vx 0.1 --seconds 10`. Expected: all PASS; a metrics report.
- [ ] **Step 5: Commit** — `git add tools docs ros2_ws && git commit -m "Add tools/walk.py and document headless walking"`; push the branch and confirm the GitHub `python` and `ros` jobs are green.
