"""Stand up, sit down or hold a body pose with the full simulated robot, and report how it went.

Usage: python tools/stand.py {stand|sit|pose} [--height MM] [--shift X Y] [--roll DEG]
                                            [--pitch DEG] [--yaw DEG] [--viewer]

`pose` stands up first, then moves the body to the pose with the feet planted. --height is the
hip plane's height above the floor in mm (98 is standing); --shift moves the body in mm.
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for package in ("bemu_params", "bemu_kinematics", "bemu_description", "bemu_sim"):
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / package))

from bemu_description.actuators import load_actuator_model  # noqa: E402
from bemu_description.robot_mjcf import build_robot_mjcf  # noqa: E402
from bemu_kinematics import (  # noqa: E402
    BodyPose,
    KinematicsError,
    neutral_feet,
    pose_transition,
    sit_down_targets,
    stand_height,
    stand_up_targets,
)
from bemu_params import ParamsError, load_params  # noqa: E402
from bemu_sim.headless import CONTROL_RATE  # noqa: E402
from bemu_sim.report import measure  # noqa: E402
from bemu_sim.robot import RobotSim, play  # noqa: E402

PARAMS_FILE = REPO_ROOT / "config" / "robot_params.yaml"
ACTUATORS_FILE = REPO_ROOT / "ros2_ws" / "src" / "bemu_sim" / "config" / "actuators.yaml"
SETTLE_STEPS = 50  # 1 s of holding the last target


def main() -> int:
    try:
        return _run()
    except (ParamsError, KinematicsError, ValueError, OSError) as exc:
        print(f"stand: {exc}", file=sys.stderr)
        return 1


def _run() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("action", choices=("stand", "sit", "pose"))
    parser.add_argument("--height", type=float, help="hip plane height above the floor, mm")
    parser.add_argument("--shift", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"))
    parser.add_argument("--roll", type=float, default=0.0, help="degrees")
    parser.add_argument("--pitch", type=float, default=0.0, help="degrees")
    parser.add_argument("--yaw", type=float, default=0.0, help="degrees")
    parser.add_argument("--viewer", action="store_true", help="play in MuJoCo's viewer")
    args = parser.parse_args()

    params = load_params(PARAMS_FILE)
    actuators = load_actuator_model(ACTUATORS_FILE, params.servo)
    sim = RobotSim(build_robot_mjcf(params, actuators))

    stand = stand_height(params)
    steps = list(stand_up_targets(params))
    if args.action == "sit":
        steps += sit_down_targets(params)
    elif args.action == "pose":
        pose = BodyPose(
            height=stand if args.height is None else args.height / 1000.0,
            x=args.shift[0] / 1000.0,
            y=args.shift[1] / 1000.0,
            roll=math.radians(args.roll),
            pitch=math.radians(args.pitch),
            yaw=math.radians(args.yaw),
        )
        steps += pose_transition(params, BodyPose(stand), pose)
    # Resolve every target before anything moves: an impossible pose fails here, with its leg.
    steps += [steps[-1]] * SETTLE_STEPS

    if args.viewer:
        return _play_in_viewer(sim, steps, params)
    return _print(sim, params, play(sim, steps))


def _print(sim: RobotSim, params, peak: float) -> int:
    report = measure(sim, neutral_feet(params), peak)
    print(f"height: {report.height * 1000:.1f} mm")
    print(f"tilt: {math.degrees(report.tilt):.2f} deg")
    print(
        f"roll, pitch, yaw: {math.degrees(report.roll):.2f}, "
        f"{math.degrees(report.pitch):.2f}, {math.degrees(report.yaw):.2f} deg"
    )
    print(f"planted feet: {report.planted_feet}")
    print(f"max foot slip: {report.max_foot_slip * 1000:.2f} mm")
    print(f"peak torque: {report.peak_torque:.3f} N·m")
    print(f"stability margin: {report.stability_margin * 1000:.1f} mm")
    return 0


def _play_in_viewer(sim: RobotSim, steps, params) -> int:
    import mujoco.viewer

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        peak = 0.0
        for step in steps:
            started = time.perf_counter()
            peak = max(peak, play(sim, [step]))
            viewer.sync()
            time.sleep(max(0.0, 1.0 / CONTROL_RATE - (time.perf_counter() - started)))
            if not viewer.is_running():
                break
        _print(sim, params, peak)
        sys.stdout.flush()
        # Closing the viewer window segfaults in Mesa's EGL under WSL (glfw.terminate); the report
        # is already printed, so end the process here. Do not replace this with a normal return.
        os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
