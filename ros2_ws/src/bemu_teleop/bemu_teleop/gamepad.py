"""ROS node: a gamepad (sensor_msgs/Joy from the joy package) drives the walker."""

from __future__ import annotations

import dataclasses
import sys

import rclpy
from bemu_msgs.msg import GaitState
from rclpy.node import Node
from sensor_msgs.msg import Joy

from bemu_gait import GaitParamsError, load_gait_params
from bemu_params import ParamsError, load_params
from bemu_teleop.mapping import JoyConfig, joy_actions, joy_command, top_speeds
from bemu_teleop.ros_io import Output, gait_file


class GamepadNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("gamepad_teleop", **kwargs)
        self.declare_parameter("params_file", "")
        defaults = JoyConfig()
        for field in dataclasses.fields(JoyConfig):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        self._config = JoyConfig(
            **{f.name: self.get_parameter(f.name).value for f in dataclasses.fields(JoyConfig)}
        )
        self._params = load_params(self.get_parameter("params_file").value or None)
        self._gait_params = load_gait_params(gait_file())
        self._gait = "tripod"
        self._buttons: list[int] = []
        self._out = Output(self)
        self.create_subscription(GaitState, "/gait_state", self._on_gait_state, 10)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)

    def _on_gait_state(self, msg: GaitState) -> None:
        self._gait = msg.gait

    def _on_joy(self, msg: Joy) -> None:
        buttons = [int(b) for b in msg.buttons]
        try:
            command = joy_command(
                msg.axes,
                buttons,
                self._config,
                top_speeds(self._params, self._gait_params, self._gait),
            )
        except ValueError as exc:
            self.get_logger().warn(f"joystick message ignored: {exc}", throttle_duration_sec=2.0)
            return
        for action in joy_actions(buttons, self._buttons, self._config):
            self._out.dispatch(action)
        self._buttons = buttons
        self._out.velocity(*command)


def main(args=None) -> int:
    rclpy.init(args=args)
    try:
        node = GamepadNode()
    except (ParamsError, GaitParamsError, ValueError) as exc:
        print(f"gamepad: {exc}", file=sys.stderr)
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
