"""Message builders and the action dispatcher shared by the two teleop nodes (needs ROS)."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from bemu_msgs.msg import GaitCommand
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Trigger

from bemu_teleop.mapping import Action


def gait_file() -> Path | None:
    """gait.yaml as installed by colcon (the source-tree default does not exist there)."""
    try:
        path = Path(get_package_share_directory("bemu_gait")) / "config" / "gait.yaml"
    except PackageNotFoundError:
        return None
    return path if path.exists() else None


class Output:
    """Publishers and service clients for /cmd_vel, /body_pose_cmd, /gait_cmd, /stand, /sit."""

    def __init__(self, node) -> None:
        self._node = node
        self._twist = node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self._pose = node.create_publisher(PoseStamped, "/body_pose_cmd", 10)
        self._gait = node.create_publisher(GaitCommand, "/gait_cmd", 10)
        self._stand = node.create_client(Trigger, "/stand")
        self._sit = node.create_client(Trigger, "/sit")

    def velocity(self, vx: float, vy: float, omega: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, omega
        self._twist.publish(msg)

    def height(self, height: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.position.z = height
        msg.pose.orientation.w = 1.0
        self._pose.publish(msg)

    def gait(self, name: str) -> None:
        self._gait.publish(GaitCommand(gait=name))

    def dispatch(self, action: Action) -> None:
        """Send the non-motion request an Action stands for. Service answers are logged."""
        if action.kind == "gait":
            self.gait(action.gait)
        elif action.kind == "height":
            self.height(action.height)
        elif action.kind in ("stand", "sit"):
            client = self._stand if action.kind == "stand" else self._sit
            if not client.service_is_ready():
                self._node.get_logger().warn(f"/{action.kind} is not available (is the walker up?)")
                return
            future = client.call_async(Trigger.Request())
            future.add_done_callback(self._log_answer(action.kind))

    def _log_answer(self, name: str):
        def done(future) -> None:
            result = future.result()
            if result is not None and not result.success:
                self._node.get_logger().warn(f"/{name}: {result.message}")

        return done
