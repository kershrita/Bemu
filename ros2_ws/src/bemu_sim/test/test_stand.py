import math

import pytest

from bemu_kinematics import (
    neutral_feet,
    sit_down_targets,
    sit_height,
    stand_height,
    stand_up_targets,
)
from bemu_params import LEG_CODES
from bemu_sim.report import measure
from bemu_sim.robot import RobotSim, play

SETTLE_STEPS = 50  # 1 s of holding the last target


def play_and_settle(sim, targets):
    peak = play(sim, targets)
    return max(peak, play(sim, [targets[-1]] * SETTLE_STEPS))


@pytest.fixture
def sim(robot_mjcf):
    return RobotSim(robot_mjcf)


def test_stand_up(sim, params, actuators):
    peak = play_and_settle(sim, stand_up_targets(params))
    report = measure(sim, neutral_feet(params), peak)
    assert report.height == pytest.approx(stand_height(params), abs=0.003)
    assert report.tilt < math.radians(1.0)
    assert report.planted_feet == 6
    assert report.max_foot_slip < 0.005
    assert report.peak_torque < actuators.max_torque
    assert report.stability_margin > 0.0


def test_sit_down(sim, params, actuators):
    play_and_settle(sim, stand_up_targets(params))
    peak = play_and_settle(sim, sit_down_targets(params))
    report = measure(sim, neutral_feet(params), peak)
    assert report.height == pytest.approx(sit_height(params), abs=0.003)
    assert report.tilt < math.radians(1.0)
    assert report.planted_feet == 6
    assert report.peak_torque < actuators.max_torque


def test_standing_holds_still(sim, params):
    play_and_settle(sim, stand_up_targets(params))
    before = sim.body_position()
    play(sim, [stand_up_targets(params)[-1]] * 100)
    assert abs(sim.body_position() - before).max() < 0.0005
    assert set(sim.foot_contacts()) == set(LEG_CODES)
