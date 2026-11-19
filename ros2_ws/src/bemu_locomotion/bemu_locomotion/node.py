"""ROS node: velocity, body pose and gait commands in, 18 joint targets out, at 50 Hz.

All decisions are made by LocomotionCore; this file only moves messages in and out. Time for
the command timeout is this node's own monotonic clock (when a message arrived), never a
message stamp or simulated time, so a restarted simulator cannot extend or cut a command.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from bemu_msgs.msg import GaitCommand, GaitState
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import Trigger

from bemu_gait import GaitParamsError, load_gait_params
from bemu_locomotion.convert import pose_from_msg_values
from bemu_locomotion.core import LocomotionCore, Tick
from bemu_params import ParamsError, load_params

RATE = 50.0


def _gait_file() -> Path | None:
    """gait.yaml as installed by colcon (the source-tree default does not exist there)."""
    try:
        path = Path(get_package_share_directory("bemu_gait")) / "config" / "gait.yaml"
    except PackageNotFoundError:
        return None
    return path if path.exists() else None


class LocomotionNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("locomotion_node", **kwargs)
        self.declare_parameter("params_file", "")
        self.declare_parameter("auto_stand", True)
        self.declare_parameter("cmd_timeout", 0.5)
        params_file = self.get_parameter("params_file").value or None
        self._core = LocomotionCore(
            load_params(params_file),
            load_gait_params(_gait_file()),
            auto_stand=self.get_parameter("auto_stand").value,
            cmd_timeout=self.get_parameter("cmd_timeout").value,
            rate=RATE,
        )
        self._joint_state_bad = False
        self._warned_reserved = False

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(PoseStamped, "/body_pose_cmd", self._on_body_pose, 10)
        self.create_subscription(GaitCommand, "/gait_cmd", self._on_gait_cmd, 10)
        self._commands = self.create_publisher(
            Float64MultiArray, "/leg_position_controller/commands", 10
        )
        self._state = self.create_publisher(GaitState, "/gait_state", 10)
        self._margin = self.create_publisher(Float64, "/stability_margin", 10)
        self.create_service(Trigger, "/stand", self._on_stand)
        self.create_service(Trigger, "/sit", self._on_sit)
        self.create_timer(1.0 / RATE, self._on_tick)

    # -- inputs ---------------------------------------------------------------------------------

    def _on_joint_state(self, msg: JointState) -> None:
        ok = self._core.on_joint_state(msg.name, msg.position)
        if not ok and not self._joint_state_bad:
            self.get_logger().warning(
                "joint state is missing a joint or has a bad value; skipping it"
            )
        self._joint_state_bad = not ok

    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        twist = msg.twist
        try:
            self._core.set_velocity(
                twist.linear.x, twist.linear.y, twist.angular.z, now=time.monotonic()
            )
        except ValueError as exc:
            self.get_logger().warning(f"/cmd_vel ignored: {exc}", throttle_duration_sec=2.0)

    def _on_body_pose(self, msg: PoseStamped) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        try:
            self._core.set_body_pose(pose_from_msg_values((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
        except ValueError as exc:
            self.get_logger().warning(f"/body_pose_cmd ignored: {exc}", throttle_duration_sec=2.0)

    def _on_gait_cmd(self, msg: GaitCommand) -> None:
        if (msg.step_height or msg.cycle_time) and not self._warned_reserved:
            self._warned_reserved = True
            self.get_logger().warning(
                "step_height and cycle_time are reserved in version 1; ignored"
            )
        try:
            self._core.set_gait(msg.gait)
        except ValueError as exc:
            self.get_logger().warning(f"/gait_cmd ignored: {exc}", throttle_duration_sec=2.0)

    def _on_stand(self, _request, response):
        response.success, response.message = self._core.request_stand()
        return response

    def _on_sit(self, _request, response):
        response.success, response.message = self._core.request_sit()
        return response

    # -- the 50 Hz step ---------------------------------------------------------------------------

    def _on_tick(self) -> None:
        tick = self._core.tick(time.monotonic())
        for notice in self._core.pop_notices():
            self.get_logger().warning(notice)
        if tick is None:
            return
        self._publish(tick)

    def _publish(self, tick: Tick) -> None:
        self._commands.publish(Float64MultiArray(data=list(tick.command)))
        state = GaitState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.gait = tick.gait
        state.mode = tick.mode.value
        state.phase = list(tick.phase)
        state.foot_down = list(tick.foot_down)
        state.stability_margin = tick.stability_margin
        self._state.publish(state)
        self._margin.publish(Float64(data=tick.stability_margin))


def main(args=None) -> int:
    rclpy.init(args=args)
    try:
        node = LocomotionNode()
    except (ParamsError, GaitParamsError, ValueError) as exc:
        print(f"locomotion_node: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
