"""Generate the MuJoCo (MJCF) and ROS (URDF) models from config/robot_params.yaml.

By default the one-leg models; with --robot, the full 18-joint MuJoCo model (robot.xml).
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for package in ("bemu_params", "bemu_kinematics", "bemu_description", "bemu_sim"):
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / package))

from bemu_description.actuators import load_actuator_model  # noqa: E402
from bemu_description.generate import write_models, write_robot_mjcf  # noqa: E402
from bemu_kinematics import KinematicsError  # noqa: E402
from bemu_params import ParamsError, load_params  # noqa: E402


def main() -> None:
    try:
        _run()
    except (ParamsError, KinematicsError, OSError) as exc:
        print(f"generate_model: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


def _run() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "build" / "model")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--robot", action="store_true", help="write the full robot model only")
    args = parser.parse_args()
    params = load_params(REPO_ROOT / "config" / "robot_params.yaml")
    actuators = load_actuator_model(
        REPO_ROOT / "ros2_ws" / "src" / "bemu_sim" / "config" / "actuators.yaml", params.servo
    )
    if args.robot:
        print(write_robot_mjcf(params, actuators, args.out))
        return
    for path in write_models(params, actuators, args.out, headless=args.headless).values():
        print(path)


if __name__ == "__main__":
    main()
