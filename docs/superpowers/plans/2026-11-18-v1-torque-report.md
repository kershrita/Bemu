# Version 1 — Torque Report: Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A script that runs a fixed matrix of simulated walks and writes a torque report (Markdown and JSON) with per-joint peak and RMS torque, joint speed, an electrical power estimate, stability margin and foot slip, and a verdict per row against the servo decision rule.

**Architecture:** `RobotSim` gains per-joint statistics taken on every physics step. A new `bemu_sim.torque_report` module runs scenarios (gait × motion × speed × total mass) with the torque clip raised to the servo's stall torque, so peaks show demand; it computes power and verdicts and renders the report. `tools/torque_report.py` runs the matrix and writes `docs/reports/v1-torque-report.md` and `.json`.

**Tech Stack:** Python 3.12+, numpy, MuJoCo (pip), existing `bemu_*` packages. No ROS.

**Spec:** [docs/superpowers/specs/2026-11-18-v1-torque-report-design.md](../specs/2026-11-18-v1-torque-report-design.md). Context: [06 — Simulation](../../06-simulation.md) "The torque report", [10 — Components](../../10-components.md).

## Global Constraints

- SI in code (N·m, rad/s, W, V, A, kg, m); mm, g only in YAML files, converted by the loader.
- The servo constants live in `config/robot_params.yaml`: rated torque 0.98 N·m (doc 10), supply 12 V, stall current 2.7 A, idle current 0.1 A (the last three are placeholders to confirm against the exact sub-model; say so in the file's comments).
- The simulator's design torque limit (1.47 N·m, `actuators.yaml`) is not changed. The report raises the clip to the servo's stall torque (2.94 N·m) **inside the report only**; walking tests are untouched.
- Statistics are taken on every 2 ms physics step, never once per control tick.
- Verdict rule (roadmap): the worst joint's peak torque under 50% of stall torque; the worst RMS under the rated torque; peak speed reported against the 4.7 rad/s no-load speed (a ceiling, not a pass mark).
- Matrix: gaits tripod, ripple, wave; forward at 25%, 50%, 100% of top speed; sideways and turning in place at top speed; standing; total robot masses 2.0, 2.2, 2.5 kg (the chassis mass changes; the six legs stay 185 g each). Peak over the whole run; RMS, mean torque and power over the steady part after a 6 s warm-up.
- ruff line length 100; public functions carry type hints; no ROS imports. Branch `claude/exciting-tesla-694elw`; commit messages: imperative summary, then the reason.

## Review Focus

1. A report measured under the 1.47 N·m clip can never fail: the report's actuator model has `max_torque` equal to the servo's stall torque. (Task 3)
2. A total mass at or below the six legs' mass, or non-finite → `ValueError` naming the mass. (Task 3)
3. An unknown gait or motion, or a non-positive speed fraction → `ValueError` naming it. (Task 3)
4. A row where the robot did not stay stable (minimum margin under 15 mm) is flagged `UNSTABLE`, not silently reported. (Task 3 verdict, Task 4 text)
5. An unwritable output directory → one line `torque_report: ...` on stderr and exit 1, no traceback. (Task 4)

---

## File Structure

```
config/robot_params.yaml                      + servo.rated_torque_nm, supply_voltage_v, stall_current_a, idle_current_a
ros2_ws/src/bemu_params/bemu_params/params.py + ServoParams fields
ros2_ws/src/bemu_sim/bemu_sim/robot.py        + JointStats, RobotSim.stats()/clear_stats()
ros2_ws/src/bemu_sim/bemu_sim/torque_report.py   new: scenarios, run, power, verdict, render, json
tools/torque_report.py                        new
docs/reports/v1-torque-report.md, .json       generated and committed
```

---

### Task 1: Servo parameters

**Files:**
- Modify: `config/robot_params.yaml`, `ros2_ws/src/bemu_params/bemu_params/params.py`
- Test: `ros2_ws/src/bemu_params/test/test_params.py`

**Interfaces:**
- Produces: `ServoParams` gains, after the existing fields, `rated_torque: float` (N·m), `supply_voltage: float` (V), `stall_current: float` (A), `idle_current: float` (A). YAML keys `servo.rated_torque_nm: 0.98`, `servo.supply_voltage_v: 12`, `servo.stall_current_a: 2.7`, `servo.idle_current_a: 0.1`.

- [ ] **Step 1: Write the failing tests** in `test_params.py`: `test_servo_report_constants_load` (`params.servo.rated_torque == 0.98`, `supply_voltage == 12.0`, `stall_current == 2.7`, `idle_current == 0.1`); extend the bad-file table with `("servo.rated_torque_nm", None, "servo.rated_torque_nm")`, `("servo.supply_voltage_v", -1, "servo.supply_voltage_v")`, `("servo.stall_current_a", 0, "servo.stall_current_a")`, and `("servo.idle_current_a", 5, "servo.idle_current_a")` (idle current must be below stall current); and `("servo.rated_torque_nm", 3.5, "servo.rated_torque_nm")` (rated above stall is impossible).
- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_params -q`. Expected: FAIL.
- [ ] **Step 3: Implement** the fields, their parsing in `_parse_servo` (positive numbers; `idle_current < stall_current`; `rated_torque <= stall_torque`; errors name the dotted key), and the YAML entries with a comment that current and supply are placeholders.
- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: all PASS.
- [ ] **Step 5: Commit** — `git add config ros2_ws/src/bemu_params && git commit -m "Add the servo constants the torque report needs"`

---

### Task 2: Per-joint statistics in `RobotSim`

**Files:**
- Modify: `ros2_ws/src/bemu_sim/bemu_sim/robot.py`
- Test: `ros2_ws/src/bemu_sim/test/test_robot_stats.py`

**Interfaces:**
- Consumes: `joint_order()` (1a).
- Produces:
  ```python
  @dataclass(frozen=True)
  class JointStats:
      joints: tuple[str, ...]            # joint_order()
      peak_torque: np.ndarray            # (18,) N·m, max |τ| on any physics step
      rms_torque: np.ndarray             # (18,) N·m
      mean_abs_torque: np.ndarray        # (18,) N·m
      peak_speed: np.ndarray             # (18,) rad/s, max |joint velocity|
      steps_at_limit: np.ndarray         # (18,) int: physics steps with |τ| >= 0.999 × the actuator's force limit
      steps: int                         # physics steps counted
  # RobotSim:
  def stats(self) -> JointStats
  def clear_stats(self) -> None          # clear_peak() stays and does the same
  ```
  `peak_torque` (the overall float) stays and equals the largest entry of the per-joint peaks. With no steps counted every array is zeros.

- [ ] **Step 1: Write the failing tests** (`test_robot_stats.py`, fixtures `robot_mjcf`, `params`, `actuators`): `test_statistics_match_a_hand_computation`: drive one sim with `command` for 3 control periods holding a pose slightly off `stand`; drive an identical second sim by writing `ctrl` and calling `mujoco.mj_step` itself while collecting `actuator_force` and joint `qvel` after every step; the first sim's `stats()` equals the hand-collected peak, RMS, mean |τ| and peak speed per joint (1e-12), `steps == 30`; `test_clear_resets_everything`; `test_steps_at_limit_counts_saturation`: with an actuator model whose `max_torque` is 0.05 N·m (via `dataclasses.replace`) a few ticks at `stand` give `steps_at_limit.sum() > 0` and no entry above `steps`; with `max_torque` 2.94 it is all zeros; `test_overall_peak_is_the_largest_joint_peak`; existing `peak_torque`/`clear_peak` behaviour unchanged (the 1a tests keep passing).
- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_sim/test/test_robot_stats.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement** the accumulation inside `command`'s physics loop (running max, sums of |τ| and τ², counts) and `stats()`; map joints to actuators and dofs by `joint_order()`.
- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: all PASS.
- [ ] **Step 5: Commit** — `git add ros2_ws/src/bemu_sim && git commit -m "Record per-joint torque and speed statistics on every physics step"`

---

### Task 3: The scenario matrix, power model and verdicts

**Files:**
- Create: `ros2_ws/src/bemu_sim/bemu_sim/torque_report.py`
- Test: `ros2_ws/src/bemu_sim/test/test_torque_report.py`

**Interfaces:**
- Consumes: `RobotSim`, `JointStats` (Task 2); `ServoParams` fields (Task 1); `Walker`, `Command`, `GAITS`, `GaitParams`; `stand_robot`, `walk`, `max_stance_slip` (`bemu_sim.walking`); `build_robot_mjcf`; `neutral_feet`.
- Produces:
  ```python
  MOTIONS = ("stand", "forward", "sideways", "turn")
  @dataclass(frozen=True)
  class Scenario: gait: str; motion: str; fraction: float; total_mass: float   # fraction of the gait's top speed
  @dataclass(frozen=True)
  class ScenarioResult:
      scenario: Scenario; command: tuple[float, float, float]            # vx, vy, omega
      joints: tuple[str, ...]
      peak_torque: np.ndarray; rms_torque: np.ndarray; mean_abs_torque: np.ndarray
      peak_speed: np.ndarray; steps_at_limit: np.ndarray                  # all (18,)
      min_margin: float | None; max_slip: float | None                    # None for "stand"
      power: float                                                        # W, estimated
  @dataclass(frozen=True)
  class Verdict:
      peak_ok: bool; rms_ok: bool; stable: bool
      peak_vs_stall: float; rms_vs_rated: float; speed_vs_max: float      # fractions: worst peak/stall, worst RMS/rated, peak speed/no-load
      steps_at_limit: int
  MASSES = (2.0, 2.2, 2.5)
  def top_speeds(params, gait_params) -> dict[str, dict[str, float]]     # gait -> {"forward","sideways": m/s, "turn": rad/s}
  def build_matrix(params, gait_params) -> list[Scenario]               # 48 scenarios, grouped by mass, then gait, then motion
  def params_for_mass(params, total_mass: float) -> RobotParams         # chassis = total − 6 legs
  def report_actuators(actuators, servo) -> ActuatorModel               # max_torque = servo.stall_torque
  def run_scenario(params, actuators, gait_params, scenario, *, warmup: float = 6.0, seconds: float = 24.0) -> ScenarioResult
  def electrical_power(servo, mean_abs_torque) -> float                 # Σ V·(I_idle + |τ|/k_t), k_t = stall torque / stall current
  def verdict(result, servo) -> Verdict
  ```
  Top speeds: forward and sideways `max_stride / (duty × cycle_time)`; turn = forward ÷ the largest horizontal distance of a neutral foot from the body centre. `run_scenario` stands the robot up, then for `stand` holds the pose 3 s, otherwise walks `warmup` s then `seconds` s at `fraction × top` on the chosen axis; peak and `steps_at_limit` combine both phases, RMS, mean |τ|, power, margin and slip come from the steady phase. `peak_ok`: `max(peak_torque) < 0.5 × servo.stall_torque`; `rms_ok`: `max(rms_torque) < servo.rated_torque`; `stable`: `min_margin is None or min_margin > 0.015`; `speed_vs_max`: `max(peak_speed) / servo.max_speed`; `peak_vs_stall`: `max(peak) / stall`.

- [ ] **Step 1: Write the failing tests:** `test_top_speeds`: tripod forward 0.12, ripple 0.06, wave 0.024 (abs 1e-9), turn for tripod `0.12 / 0.2348…` (within 1e-3); `test_matrix_has_48_scenarios_in_order`: 48, each (gait, motion, fraction) combination present exactly once per mass (stand: 1 per mass; forward: 3 fractions × 3 gaits; sideways and turn: 1 × 3 gaits), masses exactly `MASSES`; `test_params_for_mass`: 2.5 kg gives chassis 1.39 kg, 2.0 kg gives 0.89 kg (1e-9); Review Focus 2: `0.5`, `1.11`, `nan`, `-1` → `ValueError` matching `mass`; `test_report_actuators_clip_at_stall` (Review Focus 1): `max_torque == servo.stall_torque`, other fields unchanged; `test_electrical_power`: 18 joints at mean |τ| = `stall_torque / stall_current` give `18 × 12 × (0.1 + 1.0)` = 237.6 W (1e-9); all zeros give `18 × 12 × 0.1` = 21.6 W; a wrong-length array → `ValueError`; `test_verdict_boundaries`: a result with worst peak exactly `0.5 × stall` is not `peak_ok`, just under is; worst RMS exactly the rated torque is not `rms_ok`; `min_margin` 0.014 → `stable` False, `None` → True; Review Focus 3: unknown gait, unknown motion, `fraction <= 0` → `ValueError` naming it; `test_a_stand_scenario_runs` (real, short: `seconds` irrelevant, hold 3 s): shapes `(18,)`, `peak_torque.max() > 0.2`, `min_margin is None`, `steps_at_limit.sum() == 0`; `test_a_walking_scenario_runs` (real, `warmup=1.0, seconds=3.0`, tripod forward 0.5, 2.5 kg): `rms <= peak` elementwise, `min_margin > 0.015`, `max_slip is not None`, `power` between the idle floor (21.6 W) and the all-stall ceiling (18 × 12 × 2.7).
- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_sim/test/test_torque_report.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement** the module as specified.
- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: all PASS.
- [ ] **Step 5: Commit** — `git add ros2_ws/src/bemu_sim && git commit -m "Add the torque report scenarios, power model and verdicts"`

---

### Task 4: Rendering, the tool, and the generated report

**Files:**
- Modify: `ros2_ws/src/bemu_sim/bemu_sim/torque_report.py`
- Create: `tools/torque_report.py`, `docs/reports/v1-torque-report.md`, `docs/reports/v1-torque-report.json`
- Test: `ros2_ws/src/bemu_sim/test/test_torque_report_render.py`, `test_torque_report_tool.py`
- Modify: `docs/06-simulation.md`, `docs/10-components.md`, `docs/13-dev-environment.md`, `docs/14-testing-and-validation.md`, the spec

**Interfaces:**
- Produces:
  ```python
  def render_markdown(results: Sequence[ScenarioResult], params, actuators, *, commit: str, date: str) -> str
  def to_json(results: Sequence[ScenarioResult], params, actuators, *, commit: str, date: str) -> dict   # all 18 joints per result, plus the verdicts, the constants and their placeholder flag
  ```
  Markdown: title; method (the clip at stall, per-step statistics, warm-up, the placeholder constants called out); for each mass a table with one row per scenario — gait, motion and speed, worst peak torque and its joint, peak as % of stall, worst RMS and its joint, RMS as % of rated, peak speed, power, steps at the design limit, min margin, slip, and `PASS`/`FAIL` (peak and RMS) plus `UNSTABLE` where `stable` is False; a rule-check summary (how many rows pass; the largest peak and RMS across the matrix with their scenario); a full 18-joint table (peak, RMS) for each gait's top-speed forward row at 2.5 kg. `tools/torque_report.py [--out DIR] [--quick]`: runs `build_matrix` (48 runs; `--quick` runs one stand, one tripod forward and one wave turn at 2.5 kg with short durations as a smoke check), prints progress, writes `v1-torque-report.md` and `.json` into `DIR` (default `docs/reports/`), with the current `git` commit and date; one-line error `torque_report: ...` and exit 1 on a bad output directory or any `ValueError`.

- [ ] **Step 1: Write the failing tests:** `test_render_markdown` (fabricated results built by hand, including one with peak over 50% of stall and one with margin 0.01): the text has the headings, the placeholder-constants note, a `FAIL` for the over-peak row and `UNSTABLE` for the low-margin row, the worst joint's name and the percentage (e.g. `1.50` N·m over a 2.94 stall prints `51%`), the 18-joint table with all 18 joint names; `test_to_json_round_trips` (`json.loads(json.dumps(...))` equals it, has all 18 joint torques per result, verdict fields, and a `placeholders` list naming `stall_current_a`, `idle_current_a`, `supply_voltage_v`); `test_tool_quick_run_writes_both_files(tmp_path)` (`--quick --out tmp_path`: returns 0, both files exist, the Markdown names the three smoke scenarios); `test_tool_reports_an_unwritable_output_dir_in_one_line` (Review Focus 5): `--out` a path under a file → return 1, one stderr line starting `torque_report: `.
- [ ] **Step 2: Run** `pytest ros2_ws/src/bemu_sim/test/test_torque_report_render.py ros2_ws/src/bemu_sim/test/test_torque_report_tool.py -q`. Expected: FAIL.
- [ ] **Step 3: Implement** `render_markdown`, `to_json` and `tools/torque_report.py`.
- [ ] **Step 4: Run** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: all PASS.
- [ ] **Step 5: Generate the report:** `python tools/torque_report.py` (a few minutes). Read the output; if a row fails its rule, that is a finding to report, not a number to adjust. Expected: `docs/reports/v1-torque-report.md` and `.json` written; the report lists all 48 rows.
- [ ] **Step 6: Update the docs:** 06 (where the report lives, the method: clip at stall, per-step statistics, and the verdict rule); 10 (the servo table's first-draft "peak as % of stall" next to the simulated worst case, and the placeholder currents flagged as to be confirmed); 13 (a `tools/torque_report.py` row); 14 (what the new tests cover); the spec (status implemented, with the headline results).
- [ ] **Step 7: Commit** — `git add tools docs ros2_ws && git commit -m "Generate the version 1 torque report"`; push the branch.
