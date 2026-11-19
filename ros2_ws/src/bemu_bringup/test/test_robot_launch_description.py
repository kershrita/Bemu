import pytest

pytest.importorskip("launch")
pytest.importorskip("ament_index_python")

import importlib.util  # noqa: E402
from pathlib import Path  # noqa: E402

from launch import LaunchContext  # noqa: E402
from launch.actions import LogInfo, RegisterEventHandler  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "robot.launch.py"
PARAMS = Path(__file__).resolve().parents[4] / "config" / "robot_params.yaml"


def load_launch():
    spec = importlib.util.spec_from_file_location("robot_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(**arguments):
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {"use_sim": "true", "headless": "true", "params_file": str(PARAMS), **arguments}
    )
    return ctx


def test_real_hardware_is_refused():
    with pytest.raises(RuntimeError, match="version 2"):
        load_launch()._setup(context(use_sim="false"))


def test_the_launch_starts_the_simulator_state_publisher_spawners_and_walker():
    actions = load_launch()._setup(context())
    nodes = [a for a in actions if isinstance(a, Node)]
    handlers = [a for a in actions if isinstance(a, RegisterEventHandler)]
    assert len(nodes) == 3  # simulator, robot_state_publisher, broadcaster spawner
    assert len(handlers) == 2  # position-controller spawner, then the walker
    assert "bemu_locomotion" in LAUNCH_FILE.read_text()


def test_the_launch_file_is_installed():
    setup = (LAUNCH_FILE.parents[1] / "setup.py").read_text()
    assert "launch/robot.launch.py" in setup and "launch/leg.launch.py" in setup


def test_teleop_choices():
    module = load_launch()
    none = module._setup(context(teleop="none"))
    pad = module._setup(context(teleop="gamepad"))
    keys = module._setup(context(teleop="keyboard"))
    assert (
        len([a for a in pad if isinstance(a, Node)])
        == len([a for a in none if isinstance(a, Node)]) + 2
    )
    assert any(isinstance(a, LogInfo) for a in keys)
    assert not any(isinstance(a, LogInfo) for a in none)
    with pytest.raises(RuntimeError, match="teleop"):
        module._setup(context(teleop="bogus"))
