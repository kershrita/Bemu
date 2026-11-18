"""Trace a square or a circle with the simulated foot and report the error.

Usage: python tools/trace_foot.py --shape square|circle [--viewer]
"""

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for package in ("bemu_params", "bemu_kinematics", "bemu_description", "bemu_sim"):
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / package))

from bemu_description.actuators import load_actuator_model  # noqa: E402
from bemu_description.mjcf import build_leg_mjcf  # noqa: E402
from bemu_kinematics.paths import circle_path, square_path  # noqa: E402
from bemu_params import load_params  # noqa: E402
from bemu_sim.headless import CONTROL_RATE, LegSim, track_path  # noqa: E402

CENTER = (0.120, 0.0, -0.090)
SHAPES = {
    "square": lambda: square_path(CENTER, 0.060, 400),
    "circle": lambda: circle_path(CENTER, 0.030, 300),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--shape", choices=sorted(SHAPES), required=True)
    parser.add_argument("--viewer", action="store_true", help="play in MuJoCo's viewer")
    args = parser.parse_args()

    params = load_params(REPO_ROOT / "config" / "robot_params.yaml")
    actuators = load_actuator_model(
        REPO_ROOT / "ros2_ws" / "src" / "bemu_sim" / "config" / "actuators.yaml", params.servo
    )
    sim = LegSim(build_leg_mjcf(params, actuators))

    if args.viewer:
        return _play_in_viewer(sim, params.leg, SHAPES[args.shape]())

    result = track_path(sim, params.leg, SHAPES[args.shape]())
    print(f"shape: {args.shape} ({len(result.target)} points)")
    print(f"max error: {result.max_error * 1000:.3f} mm")
    print(f"rms error: {result.rms_error * 1000:.3f} mm")
    for name, torque in zip(("coxa", "femur", "tibia"), result.peak_torque, strict=True):
        print(f"peak torque {name}: {torque:.4f} N·m (limit {actuators.max_torque:.3f} N·m)")
    return 0


def _play_in_viewer(sim: LegSim, leg, path) -> int:
    """Play the path at real time, one point per control period."""
    import mujoco.viewer

    from bemu_kinematics import inverse

    angles = [inverse(leg, point) for point in path]
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        for target in angles:
            started = time.perf_counter()
            sim.command(target)
            viewer.sync()
            time.sleep(max(0.0, 1.0 / CONTROL_RATE - (time.perf_counter() - started)))
            if not viewer.is_running():
                break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
