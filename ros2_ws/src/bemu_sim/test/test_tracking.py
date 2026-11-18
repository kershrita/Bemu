import numpy as np
import pytest

from bemu_description.mjcf import build_leg_mjcf
from bemu_kinematics import JointAngles, UnreachableError
from bemu_kinematics.paths import circle_path, square_path
from bemu_sim.headless import LegSim, track_path

CENTER = (0.120, 0.0, -0.090)


@pytest.fixture
def sim(params, actuators):
    return LegSim(build_leg_mjcf(params, actuators))


def test_holds_neutral_stance(sim):
    for _ in range(100):  # 2 s
        sim.command(JointAngles(0.0, 0.300494, -1.753631))
    assert np.linalg.norm(sim.foot_position() - CENTER) < 0.002


def test_traces_60_mm_square_within_5_mm(sim, params, actuators):
    r = track_path(sim, params.leg, square_path(CENTER, 0.060, 400))  # 8 s
    assert r.target.shape == r.actual.shape == (400, 3)
    assert r.max_error < 0.005
    assert np.all(r.peak_torque < actuators.max_torque)


def test_traces_30_mm_circle_within_5_mm(sim, params, actuators):
    r = track_path(sim, params.leg, circle_path(CENTER, 0.030, 300))  # 6 s
    assert r.max_error < 0.005
    assert np.all(r.peak_torque < actuators.max_torque)


def test_unreachable_point_is_rejected_before_simulating(sim, params):
    path = square_path(CENTER, 0.060, 400)
    path[137] = (0.300, 0.0, 0.0)
    before = sim.joint_angles()
    with pytest.raises(UnreachableError, match="137") as info:
        track_path(sim, params.leg, path)
    assert info.value.reason == "too_far"
    assert sim.joint_angles() == before
