# Version 0 — Foundations: Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One simulated leg of Bemu traces a 60 mm square and a 30 mm-radius circle with its foot, driven by tested leg maths, first in a plain Python simulation and then through ROS 2 and `ros2_control`.

**Architecture:** A single parameter file feeds three ROS-free Python libraries: a parameter loader, leg kinematics, and a model generator that writes the MuJoCo model (MJCF) and the ROS model (URDF). A headless MuJoCo harness proves the foot-tracing test without ROS. The same model then runs under `mujoco_ros2_control`, with a small node publishing joint targets to a standard position controller.

**Tech Stack:** Python 3.12+, numpy, PyYAML, pytest, ruff, MuJoCo (Python package for the headless tests; `mujoco_vendor` under ROS), ROS 2 Lyrical on Ubuntu 26.04 (WSL 2), `ros2_control`, `ros2_controllers`, `mujoco_ros2_control`.

**Spec:** [docs/02-design.md](../02-design.md), with the version 0 section of [docs/03-roadmap.md](../03-roadmap.md). Supporting detail: [05](../05-software-stack.md), [06](../06-simulation.md), [07](../07-locomotion.md), [13](../13-dev-environment.md).

## Global Constraints

- Inside code, every quantity is SI: metres, radians, seconds, kilograms, newton-metres. Millimetres, degrees and grams appear only in `config/robot_params.yaml` and are converted by the loader.
- No dimension, mass or joint limit is typed anywhere except `config/robot_params.yaml`. Tests read it too.
- Leg codes: `lf lm lr rf rm rr`. Joint names: `<leg>_coxa_joint`, `<leg>_femur_joint`, `<leg>_tibia_joint`. Foot frame: `<leg>_foot`.
- Leg frame: origin on the coxa axis, x outward, y left of x seen from above, z up (REP 103 handedness).
- Angle zero and sign: coxa 0 = leg straight out, positive counter-clockwise from above; femur 0 = horizontal, positive raised; tibia 0 = in line with femur, positive raised (a bent knee is negative).
- Starting values, verbatim from the design: `Lc = 45 mm`, `Lf = 65 mm`, `Lt = 110 mm`; limits coxa `[-45°, 45°]`, femur `[-60°, 90°]`, tibia `[-150°, -20°]`; neutral foot at radius `120 mm`, height `-90 mm`.
- Physics step 2 ms. Control rate 50 per second (10 physics steps per control cycle).
- Maths libraries (`bemu_params`, `bemu_kinematics`, `bemu_description`, and `bemu_sim.headless`) import nothing from ROS.
- Python is formatted and linted with `ruff`; public functions carry type hints; every maths function has a test.
- ROS packages are named `bemu_*`. Topic names and types follow [05 — Software stack](../05-software-stack.md).
- Commit messages: imperative summary line, then the reason.
- The repository is private; no licence file is added in this version.

## Review Focus

1. A foot target containing NaN or infinity, or not exactly three numbers → `inverse` raises `ValueError`; it never returns NaN angles. (Task 2)
2. A parameter file with a missing key, a non-numeric value, or a lower limit not below its upper limit → `ParamsError` whose message names the offending key. (Task 1)
3. A parameter edit that makes the neutral stance unreachable (for example `tibia_length: 20`) → `load_params` fails with `ParamsError`, not a later crash in the simulator. (Task 1)
4. A path with one point outside reach handed to `track_path` → it raises before stepping the simulation, and the message gives the index of the bad point. (Task 4)
5. `/joint_states` listing the joints in a different order from the command order → the foot-trace node matches by joint name, not by position in the list. (Task 6)

## Where each task runs

| Tasks | Needs | Can run in |
|---|---|---|
| 1–4 | Python only | Any machine with Python 3.12+, including a cloud session |
| 5 | The Windows PC | The owner's PC, by hand |
| 6 | ROS 2 with the MuJoCo bridge | The owner's PC (WSL 2) |
| 7 | GitHub and the PC | Both |

## File Structure

```
bemu/
├── pyproject.toml                      ruff and pytest settings for the whole repository
├── requirements-dev.txt                numpy, pyyaml, pytest, ruff, mujoco>=3.4
├── .github/workflows/checks.yml        lint + pure-Python tests (Task 1); ROS build job (Task 7)
├── config/robot_params.yaml            the single source of dimensions and limits
├── tools/
│   ├── generate_model.py               writes leg.xml and leg.urdf from the parameter file
│   └── trace_foot.py                   runs the headless foot trace; optional viewer
└── ros2_ws/src/
    ├── bemu_params/                    ament_python, ROS-free
    │   ├── bemu_params/__init__.py     re-exports the public names
    │   ├── bemu_params/params.py       dataclasses, load_params, ParamsError
    │   └── test/test_params.py
    ├── bemu_kinematics/                ament_python, ROS-free
    │   ├── bemu_kinematics/leg.py      forward, inverse, JointAngles, errors
    │   ├── bemu_kinematics/paths.py    square_path, circle_path
    │   └── test/test_leg.py, test/test_paths.py
    ├── bemu_description/               ament_python, ROS-free library
    │   ├── bemu_description/names.py   joint and frame names
    │   ├── bemu_description/actuators.py  ActuatorModel, load_actuator_model
    │   ├── bemu_description/mjcf.py    build_leg_mjcf
    │   ├── bemu_description/urdf.py    build_leg_urdf
    │   ├── bemu_description/generate.py   write_models
    │   └── test/test_mjcf.py, test/test_urdf.py
    ├── bemu_sim/                       ament_python
    │   ├── bemu_sim/headless.py        LegSim, track_path, TrackingResult (ROS-free)
    │   ├── config/actuators.yaml       simulated servo model
    │   └── test/test_tracking.py
    ├── bemu_control/                   ament_python, configuration only
    │   └── config/leg_controllers.yaml
    └── bemu_bringup/                   ament_python
        ├── launch/leg.launch.py
        ├── bemu_bringup/foot_trace_node.py
        ├── bemu_bringup/joint_order.py    ROS-free helper
        └── test/test_leg_trace_launch.py, test/test_joint_order.py
```

Each ament_python package also has `package.xml`, `setup.py`, `setup.cfg` and `resource/<name>`. `bemu_params` is an addition to the layout in the design: the loader must be importable by the CAD scripts later without pulling in kinematics.

---

### Task 1: Tooling and the parameter file

**Files:**
- Create: `pyproject.toml`, `requirements-dev.txt`, `.github/workflows/checks.yml`
- Create: `config/robot_params.yaml`
- Create: `ros2_ws/src/bemu_params/` (package files, `bemu_params/params.py`, `bemu_params/__init__.py`)
- Create: `conftest.py` (repository root)
- Test: `ros2_ws/src/bemu_params/test/test_params.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  ```python
  class ParamsError(ValueError): ...

  @dataclass(frozen=True)
  class LegParams:
      coxa_length: float            # m
      femur_length: float           # m
      tibia_length: float           # m
      coxa_limits: tuple[float, float]    # rad, (lower, upper)
      femur_limits: tuple[float, float]
      tibia_limits: tuple[float, float]
      coxa_mass: float              # kg
      femur_mass: float
      tibia_mass: float

  @dataclass(frozen=True)
  class MountParams:
      x: float; y: float; yaw: float      # m, m, rad

  @dataclass(frozen=True)
  class StanceParams:
      foot_radius: float; body_height: float   # m

  @dataclass(frozen=True)
  class ServoParams:
      model: str
      mass: float                   # kg
      stall_torque: float           # N·m
      max_speed: float              # rad/s

  @dataclass(frozen=True)
  class RobotParams:
      leg: LegParams
      mounts: dict[str, MountParams]      # keys: lf lm lr rf rm rr
      stance: StanceParams
      servo: ServoParams

  LEG_CODES: tuple[str, ...] = ("lf", "lm", "lr", "rf", "rm", "rr")

  def load_params(path: str | os.PathLike | None = None) -> RobotParams
  ```
  `load_params(None)` reads the path in the environment variable `BEMU_PARAMS`.
- Root `conftest.py` provides two fixtures: `params_path` (the repository's `config/robot_params.yaml`) and `params` (`load_params(params_path)`).

- [ ] **Step 1: Write `pyproject.toml`, `requirements-dev.txt` and the package skeleton**

  `pyproject.toml`: `[tool.ruff]` line length 100, target `py312`; `[tool.ruff.lint]` select `E, F, I, UP, B`. `[tool.pytest.ini_options]`: `testpaths = ["ros2_ws/src"]`, `addopts = "--import-mode=importlib"`, `pythonpath` listing `ros2_ws/src/bemu_params`, `ros2_ws/src/bemu_kinematics`, `ros2_ws/src/bemu_description`, `ros2_ws/src/bemu_sim`, and a marker `sim_launch: needs ROS 2 and the MuJoCo bridge`.
  `requirements-dev.txt`: `numpy`, `pyyaml`, `pytest`, `ruff`, `mujoco>=3.4`.

- [ ] **Step 2: Write `config/robot_params.yaml`**

  ```yaml
  units: {length: mm, angle: deg, mass: g}

  leg:
    coxa_length: 45
    femur_length: 65
    tibia_length: 110
    joint_limits:
      coxa:  [-45, 45]
      femur: [-60, 90]
      tibia: [-150, -20]
    link_mass:            # estimates until the CAD exists (version 1.5)
      coxa: 75            # bracket plus the femur servo
      femur: 25
      tibia: 85           # knee servo plus the lower leg and foot

  body:
    mounts:
      lf: {x: 100,  y: 60,  yaw: 45}
      lm: {x: 0,    y: 80,  yaw: 90}
      lr: {x: -100, y: 60,  yaw: 135}
      rf: {x: 100,  y: -60, yaw: -45}
      rm: {x: 0,    y: -80, yaw: -90}
      rr: {x: -100, y: -60, yaw: -135}

  stance:
    foot_radius: 120
    body_height: 90

  servo:
    model: STS3215-12V
    mass: 55
    stall_torque_nm: 2.94
    max_speed_rad_s: 4.7
  ```

- [ ] **Step 3: Write the failing tests**

  ```python
  def test_loads_and_converts_to_si(params):
      assert params.leg.coxa_length == pytest.approx(0.045)
      assert params.leg.femur_length == pytest.approx(0.065)
      assert params.leg.tibia_length == pytest.approx(0.110)
      assert params.leg.tibia_limits == pytest.approx((-2.617994, -0.349066), abs=1e-6)
      assert params.leg.coxa_mass == pytest.approx(0.075)
      assert params.stance.foot_radius == pytest.approx(0.120)
      assert params.servo.stall_torque == pytest.approx(2.94)

  def test_six_mounts_in_order(params):
      assert tuple(params.mounts) == LEG_CODES
      assert params.mounts["lf"].yaw == pytest.approx(math.radians(45))
      assert params.mounts["rr"].x == pytest.approx(-0.100)

  def test_env_var_is_used_when_no_path(params_path, monkeypatch):
      monkeypatch.setenv("BEMU_PARAMS", str(params_path))
      assert load_params().leg.coxa_length == pytest.approx(0.045)

  def test_no_path_and_no_env_var_is_an_error(monkeypatch):
      monkeypatch.delenv("BEMU_PARAMS", raising=False)
      with pytest.raises(ParamsError, match="BEMU_PARAMS"):
          load_params()

  # Review Focus 2 and 3: each case edits a copy of the real file (helper: edited(tmp_path, params_path, **changes))
  @pytest.mark.parametrize("dotted_key, value, expected_in_message", [
      ("leg.femur_length", None, "leg.femur_length"),            # key removed
      ("leg.femur_length", "long", "leg.femur_length"),          # not a number
      ("leg.femur_length", -5, "leg.femur_length"),              # not positive
      ("leg.joint_limits.tibia", [-20, -150], "leg.joint_limits.tibia"),  # lower >= upper
      ("body.mounts.rr", None, "body.mounts"),                   # five mounts
      ("leg.tibia_length", 20, "stance"),                        # neutral stance out of reach
      ("units.length", "inch", "units"),                         # unsupported unit
  ])
  def test_bad_file_raises_params_error_naming_the_key(tmp_path, params_path, dotted_key, value, expected_in_message): ...
  ```

- [ ] **Step 4: Run the tests to verify they fail**

  Run: `pytest ros2_ws/src/bemu_params -v`
  Expected: FAIL, `bemu_params` has no `load_params`.

- [ ] **Step 5: Implement `bemu_params/params.py`**

  Stance reachability check, the only geometry in this package: with `r = foot_radius − coxa_length` and `d = hypot(r, body_height)`, require `r > 0` and `|Lf − Lt| ≤ d ≤ Lf + Lt`. Only the units shown in the file are accepted.

- [ ] **Step 6: Run the tests and the linter**

  Run: `pytest ros2_ws/src/bemu_params -v && ruff check . && ruff format --check .`
  Expected: all PASS, no lint findings.

- [ ] **Step 7: Add `.github/workflows/checks.yml`**

  One job `python`, on push and pull request, `ubuntu-latest`, Python 3.13: `pip install -r requirements-dev.txt`, `ruff check .`, `ruff format --check .`, `pytest -m "not sim_launch"`.

- [ ] **Step 8: Commit**

  ```bash
  git add pyproject.toml requirements-dev.txt conftest.py config .github ros2_ws/src/bemu_params
  git commit -m "Add the parameter file, its loader, and repository checks"
  ```

---

### Task 2: Leg kinematics

**Files:**
- Create: `ros2_ws/src/bemu_kinematics/` (package files, `bemu_kinematics/leg.py`, `bemu_kinematics/__init__.py`)
- Test: `ros2_ws/src/bemu_kinematics/test/test_leg.py`

**Interfaces:**
- Consumes: `LegParams` from Task 1.
- Produces:
  ```python
  class JointAngles(NamedTuple):
      coxa: float; femur: float; tibia: float        # rad

  class KinematicsError(ValueError): ...
  class UnreachableError(KinematicsError):
      reason: str        # "too_far" | "too_close" | "behind_hip"
  class JointLimitError(KinematicsError):
      joint: str         # "coxa" | "femur" | "tibia"

  def forward(leg: LegParams, angles: JointAngles) -> np.ndarray          # shape (3,), m, leg frame
  def inverse(leg: LegParams, foot: ArrayLike, *, check_limits: bool = True) -> JointAngles
  ```

- [ ] **Step 1: Write the failing tests**

  ```python
  NEUTRAL_FOOT = (0.120, 0.0, -0.090)
  NEUTRAL_ANGLES = (0.0, 0.300494, -1.753631)

  def test_inverse_neutral_stance(params):
      assert inverse(params.leg, NEUTRAL_FOOT) == pytest.approx(NEUTRAL_ANGLES, abs=1e-6)

  def test_inverse_femur_horizontal_tibia_down(params):
      assert inverse(params.leg, (0.110, 0.0, -0.110)) == pytest.approx((0.0, 0.0, -math.pi / 2), abs=1e-9)

  def test_forward_neutral_stance(params):
      assert forward(params.leg, JointAngles(*NEUTRAL_ANGLES)) == pytest.approx(NEUTRAL_FOOT, abs=1e-6)

  def test_coxa_rotates_the_foot_about_z(params):
      foot = forward(params.leg, JointAngles(math.radians(30), 0.300494, -1.753631))
      assert math.hypot(foot[0], foot[1]) == pytest.approx(0.120, abs=1e-6)
      assert math.atan2(foot[1], foot[0]) == pytest.approx(math.radians(30), abs=1e-9)
      assert foot[2] == pytest.approx(-0.090, abs=1e-6)

  def test_round_trip_10000_random_poses(params):
      # numpy default_rng(0); sample angles uniformly inside the joint limits; keep poses whose
      # horizontal reach beyond the femur axis is at least 5 mm; require 10,000 kept poses
      # for each: inverse(forward(a)) == a within 1e-9 rad, forward(inverse(p)) == p within 1e-8 m

  @pytest.mark.parametrize("foot, reason", [
      ((0.300, 0.0, 0.0), "too_far"),
      ((0.055, 0.0, -0.010), "too_close"),
      ((0.030, 0.0, -0.090), "behind_hip"),
  ])
  def test_unreachable_targets_are_reported(params, foot, reason):
      with pytest.raises(UnreachableError) as e:
          inverse(params.leg, foot)
      assert e.value.reason == reason

  def test_joint_limit_is_reported_with_the_joint_name(params):
      with pytest.raises(JointLimitError) as e:
          inverse(params.leg, (0.100, 0.150, -0.090))      # coxa would need 56.3 degrees
      assert e.value.joint == "coxa"

  def test_limits_can_be_skipped(params):
      a = inverse(params.leg, (0.100, 0.150, -0.090), check_limits=False)
      assert a.coxa == pytest.approx(0.982794, abs=1e-6)

  def test_full_extension_does_not_fail_on_rounding(params):
      a = inverse(params.leg, (0.220, 0.0, 0.0), check_limits=False)   # d == Lf + Lt exactly
      assert a == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)

  # Review Focus 1
  @pytest.mark.parametrize("foot", [
      (float("nan"), 0.0, -0.09), (0.12, float("inf"), -0.09), (0.12, 0.0), (0.12, 0.0, -0.09, 1.0),
  ])
  def test_malformed_targets_raise_value_error(params, foot):
      with pytest.raises(ValueError):
          inverse(params.leg, foot)
  ```

- [ ] **Step 2: Run the tests to verify they fail**

  Run: `pytest ros2_ws/src/bemu_kinematics -v`
  Expected: FAIL, `bemu_kinematics.leg` not found.

- [ ] **Step 3: Implement `forward` and `inverse` in `bemu_kinematics/leg.py`**

  The formulas are in [07 — Locomotion, section 1](../07-locomotion.md): the knee-up solution, `acos` arguments clamped to `[-1, 1]`. Order of checks in `inverse`: shape and finiteness, `behind_hip` (`r ≤ 0`), `too_far`, `too_close`, then joint limits in the order coxa, femur, tibia. The reach tests allow a tolerance of `1e-12` m so exact full extension passes; the limit checks allow `1e-9` rad so an angle exactly on a limit passes.

- [ ] **Step 4: Run the tests to verify they pass**

  Run: `pytest ros2_ws/src/bemu_kinematics -v`
  Expected: all PASS.

- [ ] **Step 5: Commit**

  ```bash
  git add ros2_ws/src/bemu_kinematics
  git commit -m "Add forward and inverse kinematics for one leg"
  ```

---

### Task 3: Model generator for one leg

**Files:**
- Create: `ros2_ws/src/bemu_description/` (package files; `names.py`, `actuators.py`, `mjcf.py`, `urdf.py`, `generate.py`)
- Create: `ros2_ws/src/bemu_sim/config/actuators.yaml` and the `bemu_sim` package files
- Create: `tools/generate_model.py`
- Modify: `conftest.py` (add fixture `actuators`: `load_actuator_model` on `ros2_ws/src/bemu_sim/config/actuators.yaml` with `params.servo`)
- Test: `ros2_ws/src/bemu_description/test/test_mjcf.py`, `test_urdf.py`

**Interfaces:**
- Consumes: `RobotParams`, `load_params` (Task 1); `forward`, `inverse`, `JointAngles` (Task 2).
- Produces:
  ```python
  # names.py
  def joint_names(leg_code: str) -> tuple[str, str, str]     # ("lf_coxa_joint", "lf_femur_joint", "lf_tibia_joint")
  def foot_site(leg_code: str) -> str                        # "lf_foot"
  def hip_site(leg_code: str) -> str                         # "lf_hip", at the leg-frame origin

  # actuators.py
  @dataclass(frozen=True)
  class ActuatorModel:
      kp: float; kv: float                 # N·m/rad, N·m·s/rad
      max_torque: float                    # N·m
      armature: float; damping: float; frictionloss: float
  def load_actuator_model(path: str | os.PathLike, servo: ServoParams) -> ActuatorModel

  # mjcf.py
  def build_leg_mjcf(params: RobotParams, actuators: ActuatorModel, leg_code: str = "lf") -> str

  # urdf.py
  def build_leg_urdf(params: RobotParams, leg_code: str = "lf", *,
                     mujoco_model_path: str, headless: bool = False) -> str

  # generate.py
  def write_models(params: RobotParams, actuators: ActuatorModel, out_dir: Path,
                   *, headless: bool = False) -> dict[str, Path]     # {"mjcf": .../leg.xml, "urdf": .../leg.urdf}
  ```
- The MJCF contains a keyframe named `neutral` holding the neutral joint angles as both `qpos` and `ctrl`.

**Decisions:**
- The MuJoCo model is generated directly from the parameter file. The bridge's URDF-to-MJCF converter is documented as highly experimental, and the design allows writing both formats directly.
- `actuators.yaml`: `kp: 20.0`, `kv: 0.5`, `max_torque_fraction_of_stall: 0.5`, `armature: 0.002`, `damping: 0.05`, `frictionloss: 0.02`. `max_torque = fraction × servo.stall_torque` (1.47 N·m).
- `mjcf.py` and `urdf.py` build XML text and do not import `mujoco`; only the tests and `bemu_sim.headless` do.
- The leg hangs from a fixed stand: the hip (leg-frame origin) is a fixed body 0.20 m above a floor plane, leg x-axis along world x. No free joint.
- One `<position>` actuator per joint, **named exactly as the joint it drives**. This is the rule `mujoco_ros2_control` uses to pair an actuator with a `ros2_control` joint. Attributes: `kp`, `kv`, `ctrlrange` = joint range, `forcerange` = ±`max_torque`.
- Links are capsules of radius 12 mm with the masses from the parameter file; the foot is a sphere of radius 8 mm with friction 1.0.
- `<option timestep="0.002"/>`.
- URDF `<ros2_control name="BemuLeg" type="system">`: plugin `mujoco_ros2_control/MujocoSystemInterface`; hardware params `mujoco_model` (absolute path), `headless`, `initial_keyframe` = `neutral`. Each joint: command interface `position`; state interfaces `position` (with `initial_value` = the neutral angle), `velocity`, `effort`.

- [ ] **Step 1: Write the failing tests**

  ```python
  # test_mjcf.py  (model = mujoco.MjModel.from_xml_string(build_leg_mjcf(params, actuators)))
  def test_model_loads_with_three_named_joints_and_actuators(model):
      for name in joint_names("lf"):
          assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0
          assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) >= 0
      assert model.nu == 3 and model.nq == 3

  def test_timestep_and_joint_ranges_come_from_params(model, params):
      assert model.opt.timestep == pytest.approx(0.002)
      # jnt_range of each joint == the matching limits in params.leg, abs=1e-9

  def test_force_limit_is_half_of_stall(model):
      assert model.actuator_forcerange[:, 1] == pytest.approx([1.47, 1.47, 1.47])

  @pytest.mark.parametrize("angles", [
      (0.0, 0.300494, -1.753631), (0.5, 0.0, -1.570796), (-0.6, 0.876369, -2.416219), (0.2, -0.290826, -0.881021),
  ])
  def test_foot_site_matches_forward_kinematics(model, params, angles):
      # set data.qpos to angles, mj_forward; foot site position minus hip site position
      # == forward(params.leg, JointAngles(*angles)) within 1e-6 m

  def test_neutral_keyframe(model):
      key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "neutral")
      assert model.key_qpos[key] == pytest.approx([0.0, 0.300494, -1.753631], abs=1e-6)

  # test_urdf.py  (root = ElementTree.fromstring(build_leg_urdf(params, mujoco_model_path="/tmp/leg.xml")))
  def test_ros2_control_block(root):
      hw = root.find("ros2_control/hardware")
      assert hw.findtext("plugin") == "mujoco_ros2_control/MujocoSystemInterface"
      assert {p.get("name"): p.text for p in hw.findall("param")} == {
          "mujoco_model": "/tmp/leg.xml", "headless": "false", "initial_keyframe": "neutral"}

  def test_joints_limits_and_interfaces(root, params):
      # the three revolute joints exist with <limit lower/upper> equal to params (abs=1e-6)
      # each ros2_control joint has command_interface "position" and state interfaces
      # position, velocity, effort; position's initial_value equals the neutral angle

  def test_write_models_creates_both_files(tmp_path, params, actuators): ...
  ```

- [ ] **Step 2: Run the tests to verify they fail**

  Run: `pytest ros2_ws/src/bemu_description -v`
  Expected: FAIL, `bemu_description` not found.

- [ ] **Step 3: Implement the five modules**

  `test_foot_site_matches_forward_kinematics` fixes every axis direction and offset; build the body tree until it passes. Hinge axes for femur and tibia point along the leg frame's −y so that a positive angle raises the link.

- [ ] **Step 4: Implement `tools/generate_model.py`**

  `python tools/generate_model.py [--out DIR] [--headless]`; default `--out` is `build/model`. Prints the two paths. `build/` is added to `.gitignore`.

- [ ] **Step 5: Run the tests and the tool**

  Run: `pytest ros2_ws/src/bemu_description -v && python tools/generate_model.py`
  Expected: all PASS; two paths printed; both files exist.

- [ ] **Step 6: Commit**

  ```bash
  git add conftest.py ros2_ws/src/bemu_description ros2_ws/src/bemu_sim tools/generate_model.py .gitignore
  git commit -m "Generate the one-leg MuJoCo and ROS models from the parameter file"
  ```

---

### Task 4: Headless simulation and the foot trace

**Files:**
- Create: `ros2_ws/src/bemu_kinematics/bemu_kinematics/paths.py`
- Create: `ros2_ws/src/bemu_sim/bemu_sim/headless.py`
- Create: `tools/trace_foot.py`
- Test: `ros2_ws/src/bemu_kinematics/test/test_paths.py`, `ros2_ws/src/bemu_sim/test/test_tracking.py`

**Interfaces:**
- Consumes: `inverse`, `forward`, `JointAngles` (Task 2); `build_leg_mjcf`, `joint_names`, `foot_site`, `hip_site`, `load_actuator_model` (Task 3).
- Produces:
  ```python
  # paths.py — closed loops, points evenly spaced along the perimeter, first point not repeated at the end
  def square_path(center: ArrayLike, side: float, n: int, *, plane: str = "xz") -> np.ndarray   # (n, 3)
  def circle_path(center: ArrayLike, radius: float, n: int, *, plane: str = "xy") -> np.ndarray # (n, 3)
  # plane is "xz" or "xy"; any other value raises ValueError

  # headless.py
  CONTROL_RATE: float = 50.0

  @dataclass(frozen=True)
  class TrackingResult:
      max_error: float; rms_error: float       # m
      target: np.ndarray; actual: np.ndarray   # (n, 3), leg frame
      peak_torque: np.ndarray                  # (3,), N·m, absolute

  class LegSim:
      def __init__(self, mjcf: str, leg_code: str = "lf") -> None      # starts at the "neutral" keyframe
      def command(self, angles: JointAngles) -> None                   # sets targets, advances one control period
      def foot_position(self) -> np.ndarray                            # (3,), leg frame
      def joint_angles(self) -> JointAngles
      def joint_torques(self) -> np.ndarray                            # (3,)

  def track_path(sim: LegSim, leg: LegParams, path: np.ndarray, *, settle_time: float = 1.0) -> TrackingResult
  ```
  `track_path` first converts every point with `inverse`, then moves to the first point over `settle_time`, then sends one point per control period, recording the foot after each.

- [ ] **Step 1: Write the failing tests**

  ```python
  # test_paths.py
  def test_square_has_corners_and_stays_in_plane():
      p = square_path((0.120, 0.0, -0.090), 0.060, 400)
      assert p.shape == (400, 3) and np.all(p[:, 1] == 0.0)
      assert p[:, 0].min() == pytest.approx(0.090) and p[:, 0].max() == pytest.approx(0.150)
      assert p[:, 2].min() == pytest.approx(-0.120) and p[:, 2].max() == pytest.approx(-0.060)
      # consecutive points, including last-to-first, are 0.240 / 400 apart (abs=1e-9)

  def test_circle_has_constant_radius_in_xy():
      p = circle_path((0.120, 0.0, -0.090), 0.030, 300)
      assert np.hypot(p[:, 0] - 0.120, p[:, 1]) == pytest.approx(0.030) and np.all(p[:, 2] == -0.090)

  def test_unknown_plane_is_rejected(): ...       # plane="yz" -> ValueError

  def test_every_point_of_both_test_paths_is_reachable(params): ...   # inverse() succeeds with limits on

  # test_tracking.py   (sim fixture: LegSim(build_leg_mjcf(params, actuators)))
  def test_holds_neutral_stance(sim, params):
      for _ in range(100): sim.command(JointAngles(0.0, 0.300494, -1.753631))      # 2 s
      assert np.linalg.norm(sim.foot_position() - (0.120, 0.0, -0.090)) < 0.002

  def test_traces_60_mm_square_within_5_mm(sim, params, actuators):
      r = track_path(sim, params.leg, square_path((0.120, 0.0, -0.090), 0.060, 400))   # 8 s
      assert r.max_error < 0.005
      assert np.all(r.peak_torque < actuators.max_torque)

  def test_traces_30_mm_circle_within_5_mm(sim, params):
      r = track_path(sim, params.leg, circle_path((0.120, 0.0, -0.090), 0.030, 300))   # 6 s
      assert r.max_error < 0.005

  # Review Focus 4
  def test_unreachable_point_is_rejected_before_simulating(sim, params):
      path = square_path((0.120, 0.0, -0.090), 0.060, 400); path[137] = (0.300, 0.0, 0.0)
      before = sim.joint_angles()
      with pytest.raises(UnreachableError, match="137"):
          track_path(sim, params.leg, path)
      assert sim.joint_angles() == before
  ```

- [ ] **Step 2: Run the tests to verify they fail**

  Run: `pytest ros2_ws/src/bemu_kinematics/test/test_paths.py ros2_ws/src/bemu_sim -v`
  Expected: FAIL, modules not found.

- [ ] **Step 3: Implement `paths.py` and `headless.py`**

  One control period is `round(1 / (CONTROL_RATE × timestep))` physics steps (10). Foot position is the foot site minus the hip site. If a tracking test misses 5 mm, tune `kp`, `kv` and `damping` in `actuators.yaml`; do not loosen the threshold or slow the path.

- [ ] **Step 4: Implement `tools/trace_foot.py`**

  `python tools/trace_foot.py --shape square|circle [--viewer]`. Prints max error, RMS error and peak torque per joint. `--viewer` opens MuJoCo's passive viewer and plays in real time.

- [ ] **Step 5: Run the tests to verify they pass**

  Run: `pytest -m "not sim_launch" -v && ruff check . && ruff format --check .`
  Expected: all PASS.

- [ ] **Step 6: Commit and push; confirm the GitHub check is green**

  ```bash
  git add ros2_ws/src/bemu_kinematics ros2_ws/src/bemu_sim tools/trace_foot.py
  git commit -m "Trace a square and a circle with the simulated foot, without ROS"
  ```

---

### Task 5: Development environment on the Windows PC

Done by hand on the owner's PC. The deliverable is a working environment and a filled-in version record.

**Files:**
- Modify: `docs/13-dev-environment.md` (the "Version record" table and the setup checklist, with the commands actually used)

**Interfaces:**
- Produces: Ubuntu 26.04 under WSL 2 with ROS 2 Lyrical, `ros2_control`, `ros2_controllers`, `mujoco_ros2_control`; the repository at `~/bemu`; `BEMU_PARAMS` set.

- [ ] **Step 1: Install WSL 2 and Ubuntu 26.04**

  In an administrator PowerShell: `wsl --list --online` to confirm the distribution name, then `wsl --install -d Ubuntu-26.04`. Restart if asked; create the Linux user.
  Check: `wsl -l -v` shows `Ubuntu-26.04` with `VERSION 2`.

- [ ] **Step 2: Confirm 3D rendering uses the NVIDIA card**

  Inside Ubuntu: `sudo apt update && sudo apt install -y mesa-utils && glxinfo -B | grep "OpenGL renderer"`.
  Expected: the line names the NVIDIA card (through D3D12). If it says `llvmpipe`, update the NVIDIA Windows driver and WSL (`wsl --update`), restart WSL, and repeat. If it still says `llvmpipe`, stop and record it: this is risk R15, and the fallback is dual-boot Ubuntu.

- [ ] **Step 3: Install ROS 2 Lyrical**

  Follow "Installing on Ubuntu" in the ROS 2 Lyrical documentation (linked under References) through `sudo apt install ros-lyrical-desktop`, then `sudo apt install ros-dev-tools`. Add `source /opt/ros/lyrical/setup.bash` to `~/.bashrc`.
  Check: `ros2 doctor` reports no errors; `ros2 run demo_nodes_cpp talker` prints messages.

- [ ] **Step 4: Install the control stack and the MuJoCo bridge**

  `sudo apt install ros-lyrical-ros2-control ros-lyrical-ros2-controllers ros-lyrical-mujoco-ros2-control ros-lyrical-mujoco-ros2-control-demos`
  If the bridge packages are not found, clone `mujoco_ros2_control` into `~/bemu/ros2_ws/src/third_party/`, run `rosdep install --from-paths src -y --ignore-src`, and build it with the workspace.
  Check: `ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py` opens the simulator window with a robot; `ros2 control list_controllers` in a second terminal lists active controllers; the same launch with `headless:=true` runs with no window.

- [ ] **Step 5: Clone the repository and run the Python tests**

  ```bash
  git clone https://github.com/kershrita/Bemu.git ~/bemu && cd ~/bemu
  sudo apt install -y python3-numpy python3-yaml python3-pytest python3-pip
  pip install --user --break-system-packages mujoco ruff
  echo 'export BEMU_PARAMS=~/bemu/config/robot_params.yaml' >> ~/.bashrc && source ~/.bashrc
  pytest -m "not sim_launch"
  python3 tools/trace_foot.py --shape square --viewer
  ```
  No virtual environment: ROS 2 builds against the system Python, so the two extra tools are installed for the user beside it.
  Expected: all tests PASS; the viewer shows the leg tracing a square.

- [ ] **Step 6: Record the versions and commit**

  Fill every row of the version record in `docs/13-dev-environment.md` (`wsl --version`, `nvidia-smi`, `lsb_release -d`, `echo $ROS_DISTRO`, `apt show ros-lyrical-ros2-control`, `apt show ros-lyrical-mujoco-vendor`, `python3 --version`, `pip show mujoco`). Replace the checklist with the commands actually used. If the MuJoCo version under ROS differs from the pip one, note both.

  ```bash
  git add docs/13-dev-environment.md
  git commit -m "Record the development environment as installed"
  ```

---

### Task 6: The leg under ROS 2

**Files:**
- Create: `ros2_ws/src/bemu_control/` (package files, `config/leg_controllers.yaml`)
- Create: `ros2_ws/src/bemu_bringup/` (package files, `launch/leg.launch.py`, `bemu_bringup/foot_trace_node.py`, `bemu_bringup/joint_order.py`)
- Modify: `pyproject.toml` (add `ros2_ws/src/bemu_bringup` to pytest's `pythonpath`)
- Test: `ros2_ws/src/bemu_bringup/test/test_leg_trace_launch.py`, `ros2_ws/src/bemu_bringup/test/test_joint_order.py`

**Interfaces:**
- Consumes: `load_params`, `inverse`, `forward`, `square_path`, `circle_path`, `write_models`, `load_actuator_model`, `joint_names`.
- Produces:
  - Controller `joint_state_broadcaster` (`joint_state_broadcaster/JointStateBroadcaster`) publishing `/joint_states`.
  - Controller `leg_position_controller` (`forward_command_controller/ForwardCommandController`, `interface_name: position`, joints in the order of `joint_names("lf")`), subscribing to `/leg_position_controller/commands` (`std_msgs/Float64MultiArray`).
  - `controller_manager` `update_rate: 50`.
  - Launch file `leg.launch.py` with arguments `use_sim` (default `true`), `headless` (default `false`), `shape` (`square` | `circle` | `none`, default `none`), `params_file` (default: `$BEMU_PARAMS`).
  - Node `foot_trace_node` with parameters `shape`, `params_file`; publishes `/foot_trace/max_error` (`std_msgs/Float64`, latched) when the trace ends, then exits with code 0.

**Decisions:**
- The launch file calls `write_models` into a temporary directory at launch time and passes the URDF text to `robot_state_publisher`. The simulator is the bridge's own node: package `mujoco_ros2_control`, executable `ros2_control_node`, parameters `{"use_sim_time": True}` plus `leg_controllers.yaml`. Controllers are started with `controller_manager` `spawner` nodes.
- `use_sim:=false` makes the launch file fail immediately with the message `Real hardware arrives in version 2`.
- `foot_trace_node` runs on simulated time, sends one target per 20 ms timer tick, and measures the foot from `/joint_states` through `forward`.

- [ ] **Step 1: Write the failing launch test**

  ```python
  pytest.importorskip("launch_testing")     # first statement: lets the file be collected where ROS is absent

  @pytest.mark.sim_launch
  @pytest.mark.launch_test
  def generate_test_description():
      # include leg.launch.py with headless:=true, shape:=square; then ReadyToTest

  class TestLegTrace(unittest.TestCase):
      def test_joint_states_has_the_three_leg_joints(self): ...
          # within 20 s a /joint_states message arrives whose names, as a set, equal joint_names("lf")
      def test_both_controllers_are_active(self): ...
          # list_controllers service: joint_state_broadcaster and leg_position_controller are "active"
      def test_square_trace_error_under_5_mm(self): ...
          # within 60 s a /foot_trace/max_error message arrives with data < 0.005
  ```

  And, in `test_joint_order.py`, a unit test for the node's joint-order handling (Review Focus 5), runnable without ROS:

  ```python
  def test_angles_from_joint_state_are_matched_by_name():
      names = ["lf_tibia_joint", "lf_coxa_joint", "lf_femur_joint"]
      positions = [-1.753631, 0.0, 0.300494]
      assert angles_by_name(names, positions, "lf") == pytest.approx((0.0, 0.300494, -1.753631))
  ```
  `angles_by_name(names: Sequence[str], positions: Sequence[float], leg_code: str) -> JointAngles` lives in `bemu_bringup/joint_order.py`, which imports nothing from ROS. A missing joint name raises `KeyError` naming it.

- [ ] **Step 2: Run the tests to verify they fail**

  Run: `cd ~/bemu/ros2_ws && colcon build --symlink-install && source install/setup.bash && colcon test --packages-select bemu_bringup && colcon test-result --verbose`
  Expected: FAIL, launch file not found.

- [ ] **Step 3: Write `leg_controllers.yaml`, `leg.launch.py` and `foot_trace_node.py`**

- [ ] **Step 4: Run it by hand with the window open**

  Run: `ros2 launch bemu_bringup leg.launch.py shape:=square`
  Expected: the simulator window shows the leg tracing a square; the terminal prints the maximum error, below 5 mm. Repeat with `shape:=circle`.
  Then with `shape:=none`, in a second terminal: `ros2 topic pub --once /leg_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, -1.5708]"` moves the leg to femur horizontal, tibia down.

- [ ] **Step 5: Run the tests to verify they pass**

  Run: `colcon test --packages-select bemu_bringup bemu_control && colcon test-result --verbose`
  Expected: all PASS.

- [ ] **Step 6: Commit**

  ```bash
  git add ros2_ws/src/bemu_control ros2_ws/src/bemu_bringup
  git commit -m "Run the simulated leg through ros2_control and trace the foot paths"
  ```

---

### Task 7: ROS build check, documentation pass, tag

**Files:**
- Modify: `.github/workflows/checks.yml`
- Modify: `docs/03-roadmap.md`, `docs/05-software-stack.md`, `docs/06-simulation.md`, `docs/13-dev-environment.md`, `README.md`
- Create: `docs/decisions/0010-generate-mjcf-directly.md`; modify `docs/decisions/README.md`

**Interfaces:**
- Consumes: everything above.
- Produces: tag `v0`.

- [ ] **Step 1: Add the `ros` job to the workflow**

  Container `ros:lyrical-ros-base`. Steps: checkout; `rosdep install --from-paths ros2_ws/src -y --ignore-src`; `pip install --break-system-packages mujoco`; `colcon build` in `ros2_ws`; `colcon test --pytest-args -m "not sim_launch"`; `colcon test-result --verbose`. The launch test stays a PC-only check in this version.
  Expected: both jobs green on GitHub.

- [ ] **Step 2: Check the roadmap's exit test, item by item**

  | Exit test | Proven by |
  |---|---|
  | A fresh clone builds and all tests pass with one command | Task 5 step 5 on the PC; both GitHub jobs |
  | Round trip within 0.01 mm for 10,000 random reachable points | `test_round_trip_10000_random_poses` (tolerance 1e-8 m) |
  | Unreachable targets rejected with a clear error | `test_unreachable_targets_are_reported`, `test_joint_limit_is_reported_with_the_joint_name` |
  | Foot follows a 60 mm square with less than 5 mm error | `test_traces_60_mm_square_within_5_mm` and `test_square_trace_error_under_5_mm` |

  Tick the boxes in `docs/03-roadmap.md`.

- [ ] **Step 3: Update the documents**

  - `05`: add `bemu_params` to the layout and the package table; note that there is no top-level `Dockerfile`, the GitHub check uses the official ROS image.
  - `06`: state that both model formats are generated directly; record the tuned actuator values if they changed.
  - `13`: confirm the daily-workflow commands match what was built (`leg.launch.py`, `tools/generate_model.py`, `tools/trace_foot.py`).
  - `03`: change step 0.1's wording from "licence" to "no licence file while the repository is private".
  - `README.md`: status line becomes "version 0 complete".
  - Decision record 0010: generate the MuJoCo model directly from the parameter file instead of converting from URDF; alternatives: the bridge's converter (experimental), hand-written MJCF.

- [ ] **Step 4: Commit, tag, push**

  ```bash
  git add -A
  git commit -m "Close version 0: ROS build check and documentation pass"
  git tag v0
  git push origin main --tags
  ```

---

## References

| Source | Used here for | Checked |
|---|---|---|
| [mujoco_ros2_control hardware interface configuration](https://control.ros.org/lyrical/doc/mujoco_ros2_control/mujoco_ros2_control/docs/hardware_interface.html) | Plugin name, hardware parameters (`mujoco_model`, `headless`, `initial_keyframe`), the actuator-to-joint naming rule, the node to launch, `use_sim_time` | 2026-11-14 |
| [mujoco_ros2_control package page](https://index.ros.org/p/mujoco_ros2_control/) | The URDF-to-MJCF converter is marked highly experimental; released for Lyrical | 2026-11-14 |
| [mujoco_ros2_control demos and tutorials](https://control.ros.org/rolling/doc/mujoco_ros2_control/mujoco_ros2_control_demos/doc/tutorials.html) | Demo launch name and the `headless:=true` argument used to verify the install | 2026-11-14 |
| [ROS 2 Lyrical: installing on Ubuntu](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html) | The ROS 2 install procedure in Task 5 | 2026-11-14 |
| [Installing Ubuntu 26.04 on WSL](https://www.how2shout.com/?p=89047) | The WSL distribution name and install route | 2026-11-14 |
| [WSL visualization, NVIDIA forum](https://forums.developer.nvidia.com/t/wsl-visualization/294703) | The software-rendering failure checked in Task 5 step 2 | 2026-11-14 |
| [MuJoCo documentation](https://mujoco.readthedocs.io/) | MJCF elements used by the generator | — |

The full list for the project is in [References](../references.md).
