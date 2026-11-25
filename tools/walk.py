"""Walk the full simulated robot with a velocity command and report how it went.

Usage: python tools/walk.py [--gait tripod|ripple|wave] [--vx M/S] [--vy M/S] [--omega RAD/S]
                            [--seconds S] [--viewer]

Stands the robot up, then walks it for the given time. --vx is forward, --vy to the left, --omega
counter-clockwise. The tripod's top speed is 0.12 m/s, the ripple's 0.06, the wave's 0.024.
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for package in ("bemu_params", "bemu_kinematics", "bemu_description", "bemu_gait", "bemu_sim"):
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / package))

from bemu_description.actuators import load_actuator_model  # noqa: E402
from bemu_description.robot_mjcf import build_robot_mjcf  # noqa: E402
from bemu_description.visuals import load_visuals  # noqa: E402
from bemu_gait import GAITS, Command, GaitParamsError, Walker, load_gait_params  # noqa: E402
from bemu_kinematics import KinematicsError  # noqa: E402
from bemu_params import ParamsError, load_params  # noqa: E402
from bemu_sim.headless import CONTROL_RATE  # noqa: E402
from bemu_sim.robot import RobotSim  # noqa: E402
from bemu_sim.walking import (  # noqa: E402
    WalkLog,
    heading_change,
    max_stance_slip,
    max_swing_drag,
    stand_robot,
    travelled,
    walk,
)

PARAMS_FILE = REPO_ROOT / "config" / "robot_params.yaml"
ACTUATORS_FILE = REPO_ROOT / "ros2_ws" / "src" / "bemu_sim" / "config" / "actuators.yaml"


class ViewerClosed(Exception):
    pass


def main() -> int:
    try:
        return _run()
    except (ParamsError, GaitParamsError, KinematicsError, ValueError, OSError) as exc:
        print(f"walk: {exc}", file=sys.stderr)
        return 1


def _run() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--gait", default="tripod", help="tripod, ripple or wave")
    parser.add_argument("--vx", type=float, default=0.08)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--omega", type=float, default=0.0)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--viewer", action="store_true", help="play in MuJoCo's viewer")
    args = parser.parse_args()
    if args.gait not in GAITS:
        raise ValueError(f"unknown gait {args.gait!r}; valid gaits: {', '.join(GAITS)}")
    if not (math.isfinite(args.seconds) and args.seconds > 0):
        raise ValueError(f"--seconds must be positive, got {args.seconds}")

    params = load_params(PARAMS_FILE)
    actuators = load_actuator_model(ACTUATORS_FILE, params.servo)
    walker = Walker(params, load_gait_params(), gait=args.gait)
    walker.set_command(Command(vx=args.vx, vy=args.vy, omega=args.omega))  # validates the numbers
    sim = RobotSim(
        build_robot_mjcf(params, actuators, visuals=load_visuals() if args.viewer else None)
    )
    stand_robot(sim, params)

    if args.viewer:
        return _walk_in_viewer(sim, walker, args)
    return _print(walk(sim, walker, args.seconds), args)


def _print(log: WalkLog, args) -> int:
    forward, lateral = travelled(log)
    print(f"gait: {args.gait}")
    print(f"forward: {forward:.3f} m")
    print(f"lateral: {lateral:.3f} m")
    print(f"heading change: {math.degrees(heading_change(log)):.1f} deg")
    print(f"min stability margin: {log.margin.min() * 1000:.1f} mm")
    print(f"max stance slip: {max_stance_slip(log) * 1000:.1f} mm")
    print(f"max swing drag: {max_swing_drag(log) * 1000:.1f} mm")
    print(f"peak torque: {log.peak_torque:.3f} N·m")
    print(f"min stride scale: {log.scale.min():.2f}")
    return 0


def _walk_in_viewer(sim: RobotSim, walker: Walker, args) -> int:
    import mujoco.viewer

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:

        def pace(state, walker):
            if not viewer.is_running():
                raise ViewerClosed
            viewer.sync()
            time.sleep(1.0 / CONTROL_RATE)

        try:
            log = walk(sim, walker, args.seconds, schedule=pace)
        except ViewerClosed:
            sys.stdout.flush()
            os._exit(0)
        _print(log, args)
        sys.stdout.flush()
        # Closing the viewer window segfaults in Mesa's EGL under WSL (glfw.terminate); the report
        # is printed, so end the process here. Do not replace this with a normal return.
        os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
