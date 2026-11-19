"""The ROS side of the keyboard teleop: raw terminal in, commands out."""

from __future__ import annotations

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from bemu_gait import GaitParamsError, load_gait_params
from bemu_params import ParamsError, load_params
from bemu_teleop.mapping import KeyboardMapper
from bemu_teleop.ros_io import Output, gait_file

RATE = 10.0  # Hz: the velocity is republished at this rate so the walker's timeout never fires


class KeyboardNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("keyboard_teleop", **kwargs)
        self.declare_parameter("params_file", "")
        params = load_params(self.get_parameter("params_file").value or None)
        self._keys = KeyboardMapper(params, load_gait_params(gait_file()))
        self._out = Output(self)
        self.quit = False
        self.create_timer(1.0 / RATE, self._on_timer)

    def _on_timer(self) -> None:
        while select.select([sys.stdin], [], [], 0)[0]:
            key = os.read(sys.stdin.fileno(), 1).decode(errors="ignore")
            if not key:
                self.quit = True
                break
            action = self._keys.press(key)
            if action is not None:
                if action.kind == "quit":
                    self.quit = True
                else:
                    self._out.dispatch(action)
        self._out.velocity(*self._keys.velocity)


def run(args, help_text: str) -> int:
    rclpy.init(args=args)
    try:
        node = KeyboardNode()
    except (ParamsError, GaitParamsError, ValueError) as exc:
        print(f"keyboard: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 1
    print(help_text)
    saved = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and not node.quit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        node._out.velocity(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
