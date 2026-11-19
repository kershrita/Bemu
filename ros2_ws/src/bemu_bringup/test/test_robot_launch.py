import pytest

pytest.importorskip("launch_testing")

import os  # noqa: E402
import time  # noqa: E402
import unittest  # noqa: E402

import launch  # noqa: E402
import launch_testing.actions  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from bemu_msgs.msg import GaitState  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

from bemu_description.names import joint_order  # noqa: E402
from bemu_kinematics import BodyPose, pose_angles, stand_height  # noqa: E402
from bemu_params import LEG_CODES, load_params  # noqa: E402

# A private ROS network, so a simulator left running in another terminal cannot collide.
os.environ["ROS_DOMAIN_ID"] = "79"


@pytest.mark.sim_launch
@pytest.mark.launch_test
def generate_test_description():
    include = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory("bemu_bringup") + "/launch/robot.launch.py"
        ),
        launch_arguments={"headless": "true"}.items(),
    )
    return launch.LaunchDescription([include, launch_testing.actions.ReadyToTest()])


def _wait(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


class TestRobotLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("robot_launch_test")
        cls.joints, cls.states, cls.margins = [], [], []
        cls.node.create_subscription(JointState, "/joint_states", cls.joints.append, 10)
        cls.node.create_subscription(GaitState, "/gait_state", cls.states.append, 10)
        cls.node.create_subscription(Float64, "/stability_margin", cls.margins.append, 10)
        cls.twist = cls.node.create_publisher(TwistStamped, "/cmd_vel", 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout):
        return _wait(
            lambda: (rclpy.spin_once(self.node, timeout_sec=0.05), predicate())[1], timeout
        )

    def _by_name(self):
        latest = self.joints[-1]
        return dict(zip(latest.name, latest.position, strict=True))

    def test_1_all_joints_and_both_controllers(self):
        self.assertTrue(self._spin_until(lambda: bool(self.joints), 30.0))
        self.assertEqual(set(self.joints[-1].name), set(joint_order()))
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))

        def active():
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            if future.result() is None:
                return False
            states = {c.name: c.state for c in future.result().controller}
            return all(
                states.get(n) == "active"
                for n in ("joint_state_broadcaster", "leg_position_controller")
            )

        self.assertTrue(_wait(active, 30.0))

    def test_2_the_robot_stands(self):
        self.assertTrue(
            self._spin_until(lambda: bool(self.states) and self.states[-1].mode == "standing", 30.0)
        )
        params = load_params()
        stand = pose_angles(params, BodyPose(stand_height(params)))
        expected = {
            name: angle
            for leg in LEG_CODES
            for name, angle in zip(
                (f"{leg}_coxa_joint", f"{leg}_femur_joint", f"{leg}_tibia_joint"),
                stand[leg],
                strict=True,
            )
        }
        self._spin_until(lambda: False, 1.0)
        measured = self._by_name()
        for name, angle in expected.items():
            self.assertAlmostEqual(measured[name], angle, delta=0.05, msg=name)

    def test_3_a_forward_command_walks_and_stopping_it_stops(self):
        start = len(self.joints)
        end = time.monotonic() + 6.0
        while time.monotonic() < end:
            msg = TwistStamped()
            msg.twist.linear.x = 0.06
            self.twist.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        window = self.joints[start:]
        moved = 0
        for name in joint_order():
            if not name.endswith("_coxa_joint"):
                continue
            series = [dict(zip(m.name, m.position, strict=True))[name] for m in window]
            moved += max(series) - min(series) > 0.1
        self.assertGreaterEqual(moved, 4)
        self.assertGreater(min(m.data for m in self.margins[-300:]), 0.015)
        self._spin_until(lambda: False, 3.0)  # the 0.5 s timeout plus the slew down
        before = self._by_name()
        self._spin_until(lambda: False, 1.0)
        after = self._by_name()
        self.assertLess(max(abs(before[n] - after[n]) for n in before), 0.01)
