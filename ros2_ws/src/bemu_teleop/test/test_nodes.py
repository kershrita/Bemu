import pytest

pytest.importorskip("rclpy")

import math  # noqa: E402
import os  # noqa: E402
import time  # noqa: E402
from pathlib import Path  # noqa: E402

import rclpy  # noqa: E402
from bemu_msgs.msg import GaitCommand, GaitState  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402

from bemu_teleop.gamepad import GamepadNode  # noqa: E402

os.environ["ROS_DOMAIN_ID"] = "80"
PARAMS_FILE = str(Path(__file__).resolve().parents[4] / "config" / "robot_params.yaml")


class Harness:
    def __init__(self) -> None:
        self.node = GamepadNode(parameter_overrides=[Parameter("params_file", value=PARAMS_FILE)])
        self.peer = rclpy.create_node("gamepad_test_peer")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.peer)
        self.twists, self.gaits = [], []
        self.peer.create_subscription(TwistStamped, "/cmd_vel", self.twists.append, 50)
        self.peer.create_subscription(GaitCommand, "/gait_cmd", self.gaits.append, 50)
        self.joy = self.peer.create_publisher(Joy, "/joy", 10)
        self.state = self.peer.create_publisher(GaitState, "/gait_state", 10)

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def send(self, axes, buttons):
        msg = Joy()
        msg.axes = [float(a) for a in axes]
        msg.buttons = list(buttons)
        self.joy.publish(msg)
        self.spin(0.3)

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        self.peer.destroy_node()


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def pad():
    harness = Harness()
    harness.spin(0.5)
    yield harness
    harness.close()


def test_forward_stick_commands_the_tripod_top_speed(pad):
    pad.send([0, 1.0, 0, 0, 0, 0], [0] * 8)
    assert pad.twists[-1].twist.linear.x == pytest.approx(0.12)


def test_the_current_gait_sets_the_scale_and_the_wave_button_fires_once(pad):
    state = GaitState()
    state.gait = "ripple"
    pad.state.publish(state)
    pad.spin(0.3)
    pad.send([0, 1.0, 0, 0, 0, 0], [0] * 8)
    assert pad.twists[-1].twist.linear.x == pytest.approx(0.06)
    wave = [0] * 8
    wave[5] = 1
    pad.send([0] * 6, wave)
    pad.send([0] * 6, wave)  # held
    assert [g.gait for g in pad.gaits] == ["wave"]


def test_centred_sticks_publish_an_explicit_zero(pad):
    pad.send([0, 1.0, 0, 0, 0, 0], [0] * 8)
    pad.send([0] * 6, [0] * 8)
    assert pad.twists[-1].twist.linear.x == 0.0


@pytest.mark.parametrize("axes", [[math.nan, 0, 0, 0, 0, 0], [0.0, 0.0]])
def test_bad_joy_messages_are_ignored(pad, axes):
    pad.send([0, 1.0, 0, 0, 0, 0], [0] * 8)
    count = len(pad.twists)
    pad.send(axes, [0] * 8)
    assert len(pad.twists) == count
    pad.send([0, 1.0, 0, 0, 0, 0], [0] * 8)
    assert len(pad.twists) == count + 1
