"""Drive the walker from the keyboard. Run it in its own terminal: ros2 run bemu_teleop keyboard

Keys step a latched speed (a terminal has no key release): w/s forward and back, a/d left and
right, q/e turn left and right, space stops. 1/2/3 pick tripod, ripple or wave; r/f raise and
lower the body 5 mm; h stands up, j sits down, x quits.
"""

from __future__ import annotations

import sys

HELP = __doc__.split("\n", 2)[2].strip()


def main(args=None) -> int:
    if not sys.stdin.isatty():
        print("keyboard: stdin is not a terminal; run it in a terminal", file=sys.stderr)
        return 1
    from bemu_teleop.keyboard_node import run  # ROS is only needed from here on

    return run(args, HELP)


if __name__ == "__main__":
    sys.exit(main())
