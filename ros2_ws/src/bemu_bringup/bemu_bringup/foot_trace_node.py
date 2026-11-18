"""ROS node: trace a square or circle with the simulated foot through ros2_control.

Reads /joint_states, publishes targets to /leg_position_controller/commands once per 20 ms
tick, and when the trace ends publishes the maximum foot error (latched) on
/foot_trace/max_error and exits with code 0.
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from bemu_bringup.foot_trace import FootTrace
from bemu_bringup.joint_order import angles_by_name
from bemu_kinematics.paths import circle_path, square_path
from bemu_params import ParamsError, load_params

LEG = "lf"
TICK = 0.02  # s, matches the 50 Hz controller update rate
LINGER_TICKS = 150  # keep the node alive 3 s after publishing so late subscribers get the result


class FootTraceNode(Node):
    def __init__(self) -> None:
        super().__init__("foot_trace_node")
        self.declare_parameter("shape", "square")
        self.declare_parameter("params_file", "")
        shape = self.get_parameter("shape").value
        params_file = self.get_parameter("params_file").value or None
        if shape not in ("square", "circle"):
            raise ValueError(f"shape must be 'square' or 'circle', got {shape!r}")

        params = load_params(params_file)
        centre = (params.stance.foot_radius, 0.0, -params.stance.body_height)
        path = (
            square_path(centre, 0.060, 400)
            if shape == "square"
            else circle_path(centre, 0.030, 300)
        )
        self._trace = FootTrace(params.leg, path)

        self._latest: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._command_pub = self.create_publisher(
            Float64MultiArray, "/leg_position_controller/commands", 10
        )
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._error_pub = self.create_publisher(Float64, "/foot_trace/max_error", latched)
        self._linger = 0
        self._published = False
        self.create_timer(TICK, self._on_tick)

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest = msg

    def _on_tick(self) -> None:
        if self._published:
            self._linger += 1
            if self._linger >= LINGER_TICKS:
                rclpy.shutdown()
            return
        if self._latest is None:
            return
        # Match joints by name: /joint_states may list them in any order.
        angles = angles_by_name(self._latest.name, self._latest.position, LEG)
        command = self._trace.step(angles)
        if command is None:
            error = self._trace.max_error
            self._error_pub.publish(Float64(data=error))
            self.get_logger().info(f"foot trace finished: max error {error * 1e3:.2f} mm")
            self._published = True
            return
        self._command_pub.publish(Float64MultiArray(data=[float(a) for a in command]))


def main(args=None) -> int:
    rclpy.init(args=args)
    try:
        node = FootTraceNode()
    except (ParamsError, ValueError) as exc:
        print(f"foot_trace_node: {exc}", file=sys.stderr)
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
