from pathlib import Path

import yaml

from bemu_description.names import joint_names

CONFIG = Path(__file__).resolve().parent.parent / "config" / "leg_controllers.yaml"


def _load():
    return yaml.safe_load(CONFIG.read_text())


def test_update_rate_is_50_hz_and_both_controllers_are_declared():
    cm = _load()["controller_manager"]["ros__parameters"]
    assert cm["update_rate"] == 50
    assert cm["joint_state_broadcaster"]["type"] == "joint_state_broadcaster/JointStateBroadcaster"
    assert (
        cm["leg_position_controller"]["type"]
        == "forward_command_controller/ForwardCommandController"
    )


def test_position_controller_joint_order_matches_the_model():
    cfg = _load()["leg_position_controller"]["ros__parameters"]
    assert tuple(cfg["joints"]) == joint_names("lf")
    assert cfg["interface_name"] == "position"
