# Version 1b-2: The ROS Walker Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (the owner chose native execution). Steps use checkbox (`- [ ]`) syntax.

**Goal:** Walk the full 18-joint simulated robot through ROS 2: `robot.launch.py` starts the MuJoCo simulator, controllers and a walker node that takes `/cmd_vel`, `/body_pose_cmd` and `/gait_cmd` and publishes joint targets.

**Architecture:** All decisions live in a ROS-free `LocomotionCore` (stand/sit state machine, command timeout, joint matching by name, wrapping 1b-1's `Walker`), tested headless here. A thin `rclpy` node translates topics to the core. The 18-joint URDF is generated from the same parameter file and name tables as the MuJoCo model. ROS-dependent tests run in the CI `ros` job and on the owner's PC.

**Tech Stack:** Python 3.13, ROS 2 Lyrical (rclpy, ament_python, ament_cmake + rosidl for messages), ros2_control, mujoco_ros2_control, pytest, launch_testing.

**Spec:** `docs/superpowers/specs/2026-11-18-v1b-walking-design.md` section 5.7 and the ROS table in section 6. Version 0's leg launch (`bemu_bringup/launch/leg.launch.py`, `foot_trace_node.py`) is the pattern.

## Global Constraints

- Joint order is `bemu_description.names.joint_order()` (18 names: legs in `LEG_CODES` order, coxa, femur, tibia). Joints are matched by name, never by position in a message.
- Control rate 50 Hz; `/cmd_vel` is `geometry_msgs/TwistStamped`; a `/cmd_vel` older than 0.5 s means zero velocity.
- Targets go to `/leg_position_controller/commands` (`std_msgs/Float64MultiArray`, 18 values).
- Services `/stand` and `/sit` are `std_srvs/Trigger`; the node stands up on start when parameter `auto_stand` is true (default true).
- Launch arguments `use_sim` (default true; false raises "Real hardware arrives in version 2"), `headless` (default false), `params_file` (default `$BEMU_PARAMS`).
- The container has no ROS: everything not needing `rclpy`, `launch` or `rosidl` is tested here with `pytest -m "not sim_launch"`; ROS tests use `pytest.importorskip` so the `python` CI job skips them. The `ros` CI job runs them minus `sim_launch`.
- Python tests use `ruff` (line length 100) and keep `ruff check . && ruff format --check .` clean.
- No new tuning values: walking parameters stay in `gait.yaml`.

## Review Focus

1. `/joint_states` that omits a joint, lists extra joints, or lists them shuffled: the node must never publish wrong or partial targets (skip the tick, warn once).
2. `/cmd_vel` with NaN or infinity: rejected with a warning, previous command kept, node survives.
3. A `/body_pose_cmd` the legs cannot reach: rejected with a warning, the old pose kept.
4. `/stand` when already standing or mid-move, `/sit` while walking, `/sit` when already sitting: each returns `success=False` or a clear message; none restarts a sequence or jerks the legs.
5. Simulated time that jumps backwards (sim restart) or `/cmd_vel` stamps: the timeout uses the node's receive time, not the message stamp.

---

### Task 1: `bemu_msgs`

**Files:**
- Create: `ros2_ws/src/bemu_msgs/{package.xml,CMakeLists.txt,msg/GaitCommand.msg,msg/GaitState.msg}`
- Test: `ros2_ws/src/bemu_msgs/test/test_msg_files.py`

**Interfaces:**
- Produces (rosidl types `bemu_msgs/msg/GaitCommand`, `GaitState`):
  - `GaitCommand`: `string gait`, `float64 step_height`, `float64 cycle_time` (0 means "use gait.yaml"; non-zero values are reserved in version 1: the node warns and ignores them).
  - `GaitState`: `std_msgs/Header header`, `string gait`, `string mode` (`sitting|standing_up|standing|stopping|sitting_down`), `float64[6] phase` and `bool[6] foot_down` (both in `LEG_CODES` order `lf, lm, lr, rf, rm, rr`; check the real order in `bemu_params`), `float64 stability_margin` (m).

- [ ] **Step 1: Write the failing test** `test_msg_files.py` (pure Python, no ROS): parse each `.msg` line by line; assert `GaitCommand` field names/types equal the list above, `GaitState` likewise, `phase` and `foot_down` have length 6, and `package.xml` declares `rosidl_default_generators`, `rosidl_default_runtime`, `std_msgs`, `member_of_group rosidl_interface_packages`. Also assert `LEG_CODES` has 6 entries (the array length is tied to it).
- [ ] **Step 2:** `pytest ros2_ws/src/bemu_msgs -q`. Expected: FAIL (files missing).
- [ ] **Step 3:** Create the package (standard `ament_cmake` + `rosidl_generate_interfaces(${PROJECT_NAME} "msg/GaitCommand.msg" "msg/GaitState.msg" DEPENDENCIES std_msgs)`).
- [ ] **Step 4:** `pytest ros2_ws/src/bemu_msgs -q && ruff check . && ruff format --check .`. Expected: PASS. (The rosidl build itself runs in the `ros` CI job and on the owner's PC; record that in the ledger.)
- [ ] **Step 5: Commit** `Add the bemu_msgs package`.

### Task 2: The 18-joint URDF and robot model files

**Files:**
- Modify: `ros2_ws/src/bemu_description/bemu_description/urdf.py` (add `build_robot_urdf`), `generate.py` (add `write_robot_models`)
- Test: `ros2_ws/src/bemu_description/test/test_robot_urdf.py`

**Interfaces:**
- Produces:
  ```python
  def build_robot_urdf(params: RobotParams, *, mujoco_model_path: str, headless: bool = False) -> str
  def write_robot_models(params, actuators, out_dir: Path, *, headless: bool = False) -> dict[str, Path]  # keys "mjcf" (robot.xml), "urdf" (robot.urdf)
  ```
  The URDF: link `body` (mass `params.body.mass`) and for each leg the same coxa/femur/tibia links and revolute joints as `build_leg_urdf` (reuse its per-leg XML by extracting a shared helper rather than copying), the leg's `{leg}_hip_link` fixed to `body` at the mount's `(x, y, 0)` with `rpy = (0, 0, yaw)`; one `ros2_control` system `BemuRobot` with the `MujocoSystemInterface` plugin, `mujoco_model` = the robot MJCF's absolute path, `headless`, `initial_keyframe` `sit`, and 18 `<joint>` blocks in `joint_order()` with position command and position/velocity/effort states, each position state's `initial_value` the sit-pose angle (`pose_angles(params, BodyPose(sit_height(params)))`).

- [ ] **Step 1: Write the failing tests:** parse with `xml.etree`: 18 revolute joints named exactly `joint_order()`; the `ros2_control` joint list equals `joint_order()` in order; 6 fixed hip joints whose origins equal each mount's `(x, y, yaw)` for every leg; `initial_keyframe` is `sit`; each `initial_value` equals the sit-pose angle for that joint (1e-9); `headless` flag follows the argument; `mujoco_model` path is the one given (and XML-escaped when it contains `&`); a leg's joint limits and axes equal `build_leg_urdf`'s for `lf` (no drift between the two builders); `write_robot_models` writes both files, the URDF names `robot.xml` by absolute path, and `robot.xml` loads in MuJoCo (`mujoco.MjModel.from_xml_path`) with 18 actuators.
- [ ] **Step 2:** `pytest ros2_ws/src/bemu_description -q`. Expected: FAIL (names missing).
- [ ] **Step 3:** Implement `build_robot_urdf` and `write_robot_models`; refactor `build_leg_urdf` to share the per-leg link/joint/ros2_control XML helper (the existing leg tests must still pass unchanged).
- [ ] **Step 4:** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: PASS.
- [ ] **Step 5: Commit** `Add the 18-joint URDF and robot model writer`.

### Task 3: `robot_controllers.yaml`

**Files:**
- Create: `ros2_ws/src/bemu_control/config/robot_controllers.yaml`
- Modify: `ros2_ws/src/bemu_control/setup.py` (install the file), `package.xml` if a dependency is missing
- Test: `ros2_ws/src/bemu_control/test/test_robot_controllers.py`

**Interfaces:** Produces the controller config: `controller_manager` at `update_rate: 50` with `joint_state_broadcaster` and `leg_position_controller` (`forward_command_controller/ForwardCommandController`), whose `joints` are the 18 names of `joint_order()` and `interface_name: position`.

- [ ] **Step 1: Write the failing test:** load the YAML; `joints == list(joint_order())`; update rate 50; controller types as above; `setup.py` lists the file under `share/bemu_control/config` (read as text); the existing `leg_controllers.yaml` is untouched.
- [ ] **Step 2:** run it. Expected: FAIL.
- [ ] **Step 3:** Write the YAML (generate the joint list from `joint_order()` once, then paste it as literal text; the test is what keeps it in step).
- [ ] **Step 4:** `pytest ros2_ws/src/bemu_control -q && ruff check .`. Expected: PASS.
- [ ] **Step 5: Commit** `Add the 18-joint controller configuration`.

### Task 4: `LocomotionCore` (the whole behaviour, ROS-free)

**Files:**
- Create: `ros2_ws/src/bemu_locomotion/{package.xml,setup.py,setup.cfg,resource/bemu_locomotion,bemu_locomotion/__init__.py,bemu_locomotion/core.py}`
- Test: `ros2_ws/src/bemu_locomotion/test/test_core.py`

**Interfaces:**
- Consumes: `bemu_gait.Walker`, `Command`, `GaitParams`; `bemu_kinematics.{BodyPose, pose_angles, sit_height, stand_height, stand_up_targets, sit_down_targets, neutral_feet}`; `bemu_description.names.joint_order`; `bemu_params.{RobotParams, LEG_CODES}`.
- Produces:
  ```python
  class Mode(str, Enum): SITTING="sitting"; STANDING_UP="standing_up"; STANDING="standing"; STOPPING="stopping"; SITTING_DOWN="sitting_down"
  @dataclass(frozen=True)
  class Tick:
      command: tuple[float, ...]        # 18 joint targets in joint_order()
      mode: Mode
      gait: str
      phase: tuple[float, ...]          # 6, LEG_CODES order (0.0s when not walking)
      foot_down: tuple[bool, ...]       # 6 (all True when not walking)
      stability_margin: float           # m (the walker's; 0.0 when not walking)
  class LocomotionCore:
      def __init__(self, params, gait_params, *, auto_stand=True, cmd_timeout=0.5, rate=50.0)
      def on_joint_state(self, names: Sequence[str], positions: Sequence[float]) -> bool  # False (and nothing stored) if a joint is missing or a value is not finite; extra joints ignored; any order
      def set_velocity(self, vx, vy, omega, now: float) -> None   # ValueError if not finite
      def set_body_pose(self, pose: BodyPose) -> None             # ValueError if unreachable (state unchanged)
      def set_gait(self, name: str) -> None                       # ValueError if unknown
      def request_stand(self) -> tuple[bool, str]
      def request_sit(self) -> tuple[bool, str]
      def tick(self, now: float) -> Tick | None                   # None until a complete joint state has arrived
  ```
  Behaviour: dt per tick is `1/rate`. `SITTING` publishes the sit-pose targets. `request_stand` succeeds only from `SITTING` and only when the measured joints are within 0.3 rad of the sit pose (else `(False, "legs are not in the sit pose")`); with `auto_stand` the first complete joint state starts the stand-up. `STANDING_UP` plays `stand_up_targets(params)` one per tick, then becomes `STANDING`. In `STANDING` the walker steps every tick; velocity is the last `set_velocity` unless `now - last_velocity_time > cmd_timeout`, then zero. `request_sit` from `STANDING`: velocity to zero, pose back to `BodyPose(stand_height)`, mode `STOPPING`; `STOPPING` ends (after at least 1.5 s) when every foot is in stance and within 3 mm of its neutral position for 10 consecutive ticks, then plays `sit_down_targets` as `SITTING_DOWN`, then `SITTING`; if that is not reached within 10 s the request is cancelled (back to `STANDING`, velocity timeout rules apply) and a message is kept for the node to log. `request_sit` from `SITTING`/`SITTING_DOWN`/`STOPPING` returns `(False, ...)`; `request_stand` from any non-sitting mode returns `(False, ...)`. Velocity and pose commands are accepted but ignored (no effect, no error) unless `STANDING`.

- [ ] **Step 1: Write the failing tests** (`conftest.py` fixtures `params`, `gait_params` exist at the repo root; build a `sit_state()` helper giving shuffled names plus one extra joint): joint order of every `Tick.command` equals `joint_order()` with the joint state given in a shuffled order; `tick` is `None` before any state; a state missing one joint or with a NaN returns `False` and leaves `tick` as it was (Review Focus 1); auto-stand: first tick after a sit state starts `STANDING_UP`, after `round(2.0*50)+1` ticks the mode is `STANDING` and the last command equals `pose_angles(params, BodyPose(stand_height))` (1e-9); `request_stand` with legs far from the sit pose is refused; `request_stand` twice and while standing is refused (Review Focus 4); velocity ignored while `SITTING`; command timeout: after `set_velocity(0.1,0,0, now=0)` the walker is moving at `now=0.4` and zero from `now=0.6` (assert via `Tick.phase` advancing and the applied stride, or via displacement of the commanded targets; choose the observable that distinguishes them) and a backwards `now` does not extend it (Review Focus 5); NaN/inf velocity raises `ValueError` and keeps the previous velocity (Review Focus 2); unreachable pose raises and the previous pose still applies (Review Focus 3); `set_gait("nope")` raises, a valid gait changes `Tick.gait` (blending is the walker's concern); `request_sit` while walking: mode `STOPPING`, then `SITTING_DOWN`, then `SITTING`, with the targets at the end equal to the sit pose (1e-6) and no tick-to-tick joint jump above 0.05 rad throughout; `request_sit` when already sitting is refused; a sit that cannot settle in 10 s is cancelled back to `STANDING` (force it by calling `set_velocity` each tick with a valid command is *not* allowed to cancel the sit: `STOPPING` zeroes velocity, so instead monkeypatch the settle test to never succeed).
- [ ] **Step 2:** `pytest ros2_ws/src/bemu_locomotion -q`. Expected: FAIL (module missing).
- [ ] **Step 3:** Implement `core.py` as specified. Build the walker once; `stand_up_targets`/`sit_down_targets` lists are computed once in `__init__`; targets are flattened to `joint_order()` through the leg dict (`angles[leg]` is a `JointAngles` in coxa, femur, tibia order).
- [ ] **Step 4:** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .`. Expected: PASS.
- [ ] **Step 5: Commit** `Add the ROS-free locomotion core`.

### Task 5: The `rclpy` node

**Files:**
- Create: `ros2_ws/src/bemu_locomotion/bemu_locomotion/node.py`; modify `setup.py` (entry point `locomotion_node = bemu_locomotion.node:main`, install nothing else), `package.xml` (depends: `bemu_params`, `bemu_kinematics`, `bemu_description`, `bemu_gait`, `bemu_msgs`, `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`; test_depend `python3-pytest`)
- Test: `ros2_ws/src/bemu_locomotion/test/test_node.py` (skipped without `rclpy`)

**Interfaces:**
- Consumes: `LocomotionCore`, `Tick` (Task 4); `bemu_msgs/msg/{GaitCommand,GaitState}` (Task 1).
- Produces: node `locomotion_node` with parameters `params_file` (string, default `""` = `load_params` default), `auto_stand` (bool, true), `cmd_timeout` (double, 0.5); subscriptions `/joint_states` (`JointState`), `/cmd_vel` (`TwistStamped`: `linear.x`, `linear.y`, `angular.z`), `/body_pose_cmd` (`PoseStamped`: `position.z` is the hip height, `position.x/y` the body shift, orientation → roll/pitch/yaw), `/gait_cmd` (`GaitCommand`); publishers `/leg_position_controller/commands` (`Float64MultiArray`), `/gait_state` (`GaitState`), `/stability_margin` (`Float64`); services `/stand`, `/sit` (`Trigger`). A 50 Hz timer calls `core.tick(now)` with `now` the node's monotonic clock reading (receive time, not message stamps); `set_velocity` is also stamped with receive time. Core errors (`ValueError`) become one warning each, never a crash. A non-zero `GaitCommand.step_height` or `cycle_time` logs a one-time warning that they are reserved in version 1. A missing/partial `/joint_states` logs one warning per streak.

- [ ] **Step 1: Write the failing tests** (rclpy, `ROS_DOMAIN_ID=78` set before `rclpy.init`, node spun in a thread or with `spin_once`; a helper publishes a shuffled `JointState` at the sit pose at 50 Hz): the node publishes 18-value commands whose entries match `joint_order()` (compare the first command against the sit/stand-up first target); message types on `/gait_state`, `/stability_margin`, `/leg_position_controller/commands` are the documented ones (use `node.get_publishers_info_by_topic`); `/stand` when already auto-standing returns `success=False`; after the stand-up completes a `TwistStamped` forward command makes commands change with time and `/gait_state.mode == "standing"` and `foot_down` has 6 entries; stopping the `/cmd_vel` publisher makes the commanded motion settle within 3 s (commands stop changing); a NaN `TwistStamped` does not kill the node (it keeps publishing); an unreachable `PoseStamped` (height 1.0 m) is warned about and ignored.
- [ ] **Step 2:** `pytest ros2_ws/src/bemu_locomotion -q`. Expected here: SKIPPED for `test_node.py` (no ROS in this container), PASS for `test_core.py`. State in the ledger that Step 1/2 for this task are RED/GREEN only on a machine with ROS; here the node file is checked with `python -m py_compile` and `ruff`.
- [ ] **Step 3:** Implement `node.py`: construct the core from `load_params(params_file or None)` and `load_gait_params()`; convert quaternion to roll/pitch/yaw with the same `R = Rz·Ry·Rx` convention as `BodyPose` (write and unit-test `quaternion_to_rpy` in `core.py`'s sibling `convert.py`, tested headless, including a 10° roll round trip and non-unit quaternion normalisation); `main()` follows `foot_trace_node.main` (exit code 1 and one stderr line `locomotion_node: ...` on `ParamsError`/`GaitParamsError`).
- [ ] **Step 4:** `pytest -m "not sim_launch" -q && ruff check . && ruff format --check .` here; on the owner's PC `colcon build && colcon test`. Expected: PASS.
- [ ] **Step 5: Commit** `Add the locomotion node`.

### Task 6: `robot.launch.py` and the simulated launch test

**Files:**
- Create: `ros2_ws/src/bemu_bringup/launch/robot.launch.py`; modify `ros2_ws/src/bemu_bringup/setup.py` (install it), `package.xml` (`exec_depend` `bemu_locomotion`, `bemu_msgs`)
- Test: `ros2_ws/src/bemu_bringup/test/test_robot_launch.py` (`sim_launch` marker, runs on the owner's PC only), plus `test_robot_launch_description.py` (no simulator: imports the launch file's `_setup` pieces)

**Interfaces:** Consumes `write_robot_models`, `robot_controllers.yaml`, `locomotion_node`. Produces a launch with arguments `use_sim`, `headless`, `params_file`; starts the MuJoCo `ros2_control_node` (parameters: `use_sim_time`, the controllers file), `robot_state_publisher`, the broadcaster spawner, the position-controller spawner (after the broadcaster exits, as in `leg.launch.py`), and `locomotion_node` (after the controller spawner exits with code 0; otherwise shut everything down with a log message, as the leg launch does) with `use_sim_time`, `params_file`.

- [ ] **Step 1: Write the failing tests.** Description test (skipped without `launch`): `use_sim:=false` raises `RuntimeError` mentioning version 2; the setup function with `headless:=true` produces the simulator node, state publisher, both spawners and the locomotion node registration; the generated URDF in the temp directory has 18 `ros2_control` joints. Launch test (`ROS_DOMAIN_ID=79`, `headless:=true`): `/joint_states` lists all 18 joints; both controllers `active`; within 8 s of the controllers going active the measured joints are within 0.05 rad of the stand pose (the robot stands); `/gait_state.mode == "standing"`; publish `TwistStamped` vx 0.06 for 6 s: the coxa/femur joints of at least four legs move through more than 0.1 rad range, `/stability_margin` stays above 0.015, and after stopping `/cmd_vel` the commands stop changing within 3 s (the 0.5 s timeout plus the slew-down).
- [ ] **Step 2:** run them on the owner's PC; here only the description test can run if `launch` is importable, else skipped. Expected: FAIL/ SKIP (document which).
- [ ] **Step 3:** Implement `robot.launch.py` by adapting `leg.launch.py` (do not import from it; share nothing but the style); the URDF comes from `write_robot_models` in a temp directory.
- [ ] **Step 4:** Owner's PC: `colcon build --symlink-install && colcon test --event-handlers console_direct+`, then `ros2 launch bemu_bringup robot.launch.py` with a window and a manual `ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped ...`. Expected: PASS; fix first-run issues in this task (version 0 needed several).
- [ ] **Step 5: Commit** `Add robot.launch.py and its launch test`.

### Task 7: Documentation and CI

**Files:**
- Modify: `docs/05-software-architecture.md` (new packages, topic and service contract), `docs/13-dev-environment.md` (launch and teleop-by-hand commands, new troubleshooting rows as found), `docs/14-testing-and-validation.md` (the ROS tests), `docs/03-roadmap.md` (steps 1.4 to 1.6 ticked where evidence exists), the 1b spec (status of 1b-2); `.github/workflows/checks.yml` only if the `ros` job needs another system package.

- [ ] **Step 1:** Update the docs with the exact commands from Task 6 and the message definitions from Task 1.
- [ ] **Step 2:** `ruff check . && ruff format --check . && pytest -m "not sim_launch" -q`. Expected: PASS.
- [ ] **Step 3: Commit** `Document the ROS walker`; push; read the GitHub `ros` job result for the pushed commit (Actions) and fix a red build in this task.
