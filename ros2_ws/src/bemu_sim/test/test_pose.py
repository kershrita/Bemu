import math

import pytest

from bemu_kinematics import (
    BodyPose,
    neutral_feet,
    pose_transition,
    stand_height,
    stand_up_targets,
)
from bemu_sim.report import measure
from bemu_sim.robot import RobotSim, play

DEG = math.pi / 180.0
SETTLE_STEPS = 50


@pytest.fixture
def standing(robot_mjcf, params):
    sim = RobotSim(robot_mjcf)
    play(sim, stand_up_targets(params))
    play(sim, [stand_up_targets(params)[-1]] * SETTLE_STEPS)
    return sim


def poses(h):
    return {
        "height+15mm": BodyPose(h + 0.015),
        "height-15mm": BodyPose(h - 0.015),
        "shift+x": BodyPose(h, x=0.020),
        "shift-x": BodyPose(h, x=-0.020),
        "shift+y": BodyPose(h, y=0.020),
        "shift-y": BodyPose(h, y=-0.020),
        "roll+": BodyPose(h, roll=10 * DEG),
        "roll-": BodyPose(h, roll=-10 * DEG),
        "pitch+": BodyPose(h, pitch=10 * DEG),
        "pitch-": BodyPose(h, pitch=-10 * DEG),
        "yaw+": BodyPose(h, yaw=15 * DEG),
        "yaw-": BodyPose(h, yaw=-15 * DEG),
    }


@pytest.mark.parametrize("name", list(poses(0.098)))
def test_pose_is_reached_without_slipping(standing, params, actuators, name):
    pose = poses(stand_height(params))[name]
    start = BodyPose(stand_height(params))
    targets = pose_transition(params, start, pose)
    peak = play(standing, targets)
    peak = max(peak, play(standing, [targets[-1]] * SETTLE_STEPS))
    report = measure(standing, neutral_feet(params), peak)

    assert report.height == pytest.approx(pose.height, abs=0.003)
    x, y = standing.body_position()[:2]
    assert x == pytest.approx(pose.x, abs=0.003)
    assert y == pytest.approx(pose.y, abs=0.003)
    assert report.roll == pytest.approx(pose.roll, abs=1.0 * DEG)
    assert report.pitch == pytest.approx(pose.pitch, abs=1.0 * DEG)
    assert report.yaw == pytest.approx(pose.yaw, abs=1.0 * DEG)
    assert report.planted_feet == 6
    assert report.max_foot_slip < 0.005
    assert report.stability_margin > 0.0
    assert report.peak_torque < actuators.max_torque
