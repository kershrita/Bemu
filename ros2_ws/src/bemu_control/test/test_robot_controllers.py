from pathlib import Path

import yaml

from bemu_description.names import joint_order

PACKAGE = Path(__file__).resolve().parents[1]


def load():
    return yaml.safe_load((PACKAGE / "config" / "robot_controllers.yaml").read_text())


def test_position_controller_covers_the_eighteen_joints_in_order():
    config = load()
    assert config["leg_position_controller"]["ros__parameters"]["joints"] == list(joint_order())
    assert config["leg_position_controller"]["ros__parameters"]["interface_name"] == "position"


def test_controller_manager_section():
    manager = load()["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 50
    assert manager["joint_state_broadcaster"]["type"] == (
        "joint_state_broadcaster/JointStateBroadcaster"
    )
    assert manager["leg_position_controller"]["type"] == (
        "forward_command_controller/ForwardCommandController"
    )


def test_the_file_is_installed_and_the_leg_file_is_untouched():
    setup = (PACKAGE / "setup.py").read_text()
    assert "config/robot_controllers.yaml" in setup and "config/leg_controllers.yaml" in setup
    leg = yaml.safe_load((PACKAGE / "config" / "leg_controllers.yaml").read_text())
    assert len(leg["leg_position_controller"]["ros__parameters"]["joints"]) == 3
