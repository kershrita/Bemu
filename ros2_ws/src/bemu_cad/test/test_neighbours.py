"""The CAD says two neighbouring legs are free of each other up to a combined 45 degrees toward each
other (test_body.py). The walker must stay inside that, or two printed legs would collide."""

import math
from pathlib import Path

import pytest

pytest.importorskip("build123d")

from bemu_cad.fit import FREE_TOWARD_SUM, NEIGHBOURS  # noqa: E402
from bemu_gait.command import Command  # noqa: E402
from bemu_gait.params import load_gait_params  # noqa: E402
from bemu_gait.walker import Walker  # noqa: E402
from bemu_kinematics import BodyPose, stand_height  # noqa: E402
from bemu_params import load_params  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
COMMANDS = [
    Command(vx=0.12),
    Command(vx=-0.12),
    Command(vy=0.12),
    Command(vy=-0.12),
    Command(omega=0.8),
    Command(omega=-0.8),
    Command(vx=0.12, omega=0.5),
    Command(vx=0.12, omega=-0.5),
]


@pytest.mark.parametrize("gait", ["tripod", "ripple", "wave"])
def test_the_gaits_keep_neighbouring_legs_inside_the_free_angle(gait):
    params = load_params(REPO / "config" / "robot_params.yaml")
    gait_params = load_gait_params()
    worst = 0.0
    for command in COMMANDS:
        walker = Walker(params, gait_params, gait=gait)
        walker.set_body_pose(BodyPose(stand_height(params)))
        walker.set_command(command)
        for _ in range(1200):
            out = walker.step(0.005)
            for a, b, sign in NEIGHBOURS:
                toward = sign * (out.angles[a].coxa - out.angles[b].coxa)
                worst = max(worst, math.degrees(toward))
    assert worst <= FREE_TOWARD_SUM, f"{gait}: neighbours reach {worst:.1f} deg toward each other"
    assert worst > 10.0  # the check sees real motion
