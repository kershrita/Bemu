from pathlib import Path

import pytest

from bemu_description.actuators import ActuatorModel, load_actuator_model
from bemu_params import RobotParams, load_params

REPO_ROOT = Path(__file__).resolve().parent


@pytest.fixture
def params_path() -> Path:
    return REPO_ROOT / "config" / "robot_params.yaml"


@pytest.fixture
def params(params_path: Path) -> RobotParams:
    return load_params(params_path)


@pytest.fixture
def actuators(params: RobotParams) -> ActuatorModel:
    return load_actuator_model(
        REPO_ROOT / "ros2_ws" / "src" / "bemu_sim" / "config" / "actuators.yaml", params.servo
    )
