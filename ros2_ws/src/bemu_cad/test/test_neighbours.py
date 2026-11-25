"""The CAD says two neighbouring legs are free of each other up to a combined 45 degrees toward each
other (test_body.py). The walker must stay inside that, or two printed legs would collide. This test
needs no CAD library."""

import math
from pathlib import Path

import pytest

from bemu_cad.neighbours import FREE_TOWARD_SUM, NEIGHBOURS, toward_sum
from bemu_gait.command import Command
from bemu_gait.params import load_gait_params
from bemu_gait.walker import Walker
from bemu_kinematics import BodyPose, stand_height
from bemu_params import load_params

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
    Command(vy=0.12, omega=0.5),
    Command(vx=-0.12, vy=-0.12, omega=-0.5),
]
POSES = [
    {},
    {"x": 0.03},
    {"y": 0.03},
    {"x": -0.03, "y": -0.03, "yaw": math.radians(-10)},
    {"yaw": math.radians(10)},
]


def _worst(params, gait_params, gait, command, pose_args, switch_to=None):
    walker = Walker(params, gait_params, gait=gait)
    try:
        walker.set_body_pose(BodyPose(stand_height(params), **pose_args))
    except Exception:  # an unreachable pose is rejected by the kinematics; nothing to check
        return None
    walker.set_command(command)
    worst = 0.0
    for i in range(1200):
        if switch_to and i == 400:
            walker.set_gait(switch_to)  # a gait change while walking blends the two
        out = walker.step(0.005)
        for a, b, sign in NEIGHBOURS:
            worst = max(worst, toward_sum(out.angles, a, b, sign))
    return worst


@pytest.mark.parametrize("gait", ["tripod", "ripple", "wave"])
def test_the_gaits_keep_neighbouring_legs_inside_the_free_angle(gait):
    params = load_params(REPO / "config" / "robot_params.yaml")
    gait_params = load_gait_params()
    results = [
        _worst(params, gait_params, gait, command, pose) for command in COMMANDS for pose in POSES
    ]
    worst = max(r for r in results if r is not None)
    assert sum(r is not None for r in results) >= 30  # most poses are reachable and were checked
    assert worst <= FREE_TOWARD_SUM, f"{gait}: neighbours reach {worst:.1f} deg toward each other"
    assert worst > 10.0  # the check sees real motion


@pytest.mark.parametrize("pair", [("tripod", "wave"), ("wave", "ripple"), ("ripple", "tripod")])
def test_a_gait_change_while_walking_stays_inside_the_free_angle(pair):
    params = load_params(REPO / "config" / "robot_params.yaml")
    gait_params = load_gait_params()
    for command in (Command(vx=0.12), Command(vx=0.12, omega=0.5), Command(omega=-0.8)):
        worst = _worst(params, gait_params, pair[0], command, {}, switch_to=pair[1])
        assert worst <= FREE_TOWARD_SUM, f"{pair} {command}: {worst:.1f} deg"
