import pytest

pytest.importorskip("rclpy")

import math  # noqa: E402
import os  # noqa: E402
import time  # noqa: E402
from pathlib import Path  # noqa: E402

import rclpy  # noqa: E402
from bemu_msgs.msg import GaitState  # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64, Float64MultiArray  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from bemu_description.names import joint_order  # noqa: E402
from bemu_kinematics import BodyPose, pose_angles, sit_height  # noqa: E402
from bemu_locomotion.node import LocomotionNode  # noqa: E402
from bemu_params import LEG_CODES, load_params  # noqa: E402

os.environ["ROS_DOMAIN_ID"] = "78"  # a private network: never talk to a running simulator
PARAMS_FILE = str(Path(__file__).resolve().parents[4] / "config" / "robot_params.yaml")


class Harness:
    def __init__(self, auto_stand: bool) -> None:
        self.node = LocomotionNode(
            parameter_overrides=[
                Parameter("params_file", value=PARAMS_FILE),
                Parameter("auto_stand", value=auto_stand),
            ]
        )
        self.peer = rclpy.create_node("locomotion_test_peer")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.peer)
        params = load_params(PARAMS_FILE)
        sit = pose_angles(params, BodyPose(sit_height(params)))
        self.sit = [a for leg in LEG_CODES for a in sit[leg]]
        self.commands, self.states, self.margins = [], [], []
        self.peer.create_subscription(
            Float64MultiArray, "/leg_position_controller/commands", self.commands.append, 50
        )
        self.peer.create_subscription(GaitState, "/gait_state", self.states.append, 50)
        self.peer.create_subscription(Float64, "/stability_margin", self.margins.append, 50)
        self.joint_pub = self.peer.create_publisher(JointState, "/joint_states", 10)
        self.twist_pub = self.peer.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.pose_pub = self.peer.create_publisher(PoseStamped, "/body_pose_cmd", 10)
        self.peer.create_timer(0.02, self._publish_joints)

    def _publish_joints(self) -> None:
        msg = JointState()
        msg.name = list(joint_order())[::-1] + ["unrelated_joint"]  # shuffled, with an extra
        msg.position = self.sit[::-1] + [0.0]
        self.joint_pub.publish(msg)

    def spin(self, seconds: float, until=None) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
            if until is not None and until():
                return True
        return until is None

    def call(self, name: str) -> Trigger.Response:
        client = self.peer.create_client(Trigger, name)
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(Trigger.Request())
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        return future.result()

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
def manual():
    harness = Harness(auto_stand=False)
    yield harness
    harness.close()


@pytest.fixture
def automatic():
    harness = Harness(auto_stand=True)
    yield harness
    harness.close()


def test_commands_are_in_joint_order_and_types_match(manual):
    assert manual.spin(5.0, until=lambda: len(manual.commands) > 3)
    assert list(manual.commands[-1].data) == pytest.approx(manual.sit, abs=1e-9)
    state = manual.states[-1]
    assert state.mode == "sitting" and len(state.phase) == 6 and len(state.foot_down) == 6
    assert isinstance(manual.margins[-1], Float64)


def test_stand_service_and_a_second_request(manual):
    manual.spin(1.0)
    assert manual.call("/stand").success
    assert manual.spin(6.0, until=lambda: manual.states[-1].mode == "standing")
    assert not manual.call("/stand").success


def test_walking_and_the_command_timeout(automatic):
    assert automatic.spin(8.0, until=lambda: automatic.states[-1].mode == "standing")

    def publish_forward():
        msg = TwistStamped()
        msg.twist.linear.x = 0.08
        automatic.twist_pub.publish(msg)

    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        publish_forward()
        automatic.spin(0.02)
    walking = [list(c.data) for c in automatic.commands[-50:]]
    assert max(max(row) - min(row) for row in zip(*walking, strict=True)) > 0.05
    automatic.spin(4.0)  # no more commands: the timeout stops the walk
    tail = [list(c.data) for c in automatic.commands[-10:]]
    assert max(max(row) - min(row) for row in zip(*tail, strict=True)) < 1e-6


def test_bad_commands_do_not_stop_the_node(automatic):
    assert automatic.spin(8.0, until=lambda: automatic.states[-1].mode == "standing")
    bad = TwistStamped()
    bad.twist.linear.x = math.nan
    automatic.twist_pub.publish(bad)
    pose = PoseStamped()
    pose.pose.position.z = 1.0
    pose.pose.orientation.w = 1.0
    automatic.pose_pub.publish(pose)
    count = len(automatic.commands)
    automatic.spin(1.0)
    assert len(automatic.commands) > count + 20
