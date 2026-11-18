"""Generate the one-leg MuJoCo (MJCF) and ROS (URDF) models from config/robot_params.yaml."""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for package in ("bemu_params", "bemu_kinematics", "bemu_description", "bemu_sim"):
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / package))

from bemu_description.actuators import load_actuator_model  # noqa: E402
from bemu_description.generate import write_models  # noqa: E402
from bemu_params import load_params  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "build" / "model")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    params = load_params(REPO_ROOT / "config" / "robot_params.yaml")
    actuators = load_actuator_model(
        REPO_ROOT / "ros2_ws" / "src" / "bemu_sim" / "config" / "actuators.yaml", params.servo
    )
    for path in write_models(params, actuators, args.out, headless=args.headless).values():
        print(path)


if __name__ == "__main__":
    main()
