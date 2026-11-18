import pytest

pytest.importorskip("launch_testing")

import time  # noqa: E402
import unittest  # noqa: E402

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

from bemu_description.names import joint_names  # noqa: E402


@pytest.mark.sim_launch
@pytest.mark.launch_test
def generate_test_description():
    include = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory("bemu_bringup") + "/launch/leg.launch.py"
        ),
        launch_arguments={"headless": "true", "shape": "square"}.items(),
    )
    return launch.LaunchDescription([include, launch_testing.actions.ReadyToTest()])


def _wait(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


class TestLegTrace(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("leg_trace_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout):
        return _wait(
            lambda: (rclpy.spin_once(self.node, timeout_sec=0.05), predicate())[1], timeout
        )

    def test_joint_states_has_the_three_leg_joints(self):
        got = []
        self.node.create_subscription(JointState, "/joint_states", got.append, 10)
        self.assertTrue(self._spin_until(lambda: bool(got), 20.0))
        self.assertEqual(set(got[-1].name), set(joint_names("lf")))

    def test_both_controllers_are_active(self):
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.assertTrue(client.wait_for_service(timeout_sec=20.0))

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

    def test_square_trace_error_under_5_mm(self):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        got = []
        self.node.create_subscription(Float64, "/foot_trace/max_error", got.append, qos)
        self.assertTrue(self._spin_until(lambda: bool(got), 60.0))
        self.assertLess(got[0].data, 0.005)
