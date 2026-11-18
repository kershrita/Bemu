import numpy as np
import pytest

from bemu_bringup.foot_trace import FootTrace
from bemu_description.mjcf import build_leg_mjcf
from bemu_kinematics import UnreachableError
from bemu_kinematics.paths import circle_path, square_path
from bemu_sim.headless import LegSim

CENTER = (0.120, 0.0, -0.090)


def run_closed_loop(sim, trace):
    """What the ROS node does each tick: read the joints, ask for the next command, send it."""
    commands = 0
    while (cmd := trace.step(sim.joint_angles())) is not None:
        sim.command(cmd)
        commands += 1
    return commands


@pytest.fixture
def sim(params, actuators):
    return LegSim(build_leg_mjcf(params, actuators))


def test_square_is_traced_within_5_mm_in_closed_loop(sim, params):
    path = square_path(CENTER, 0.060, 400)
    trace = FootTrace(params.leg, path)
    commands = run_closed_loop(sim, trace)
    assert trace.done
    assert commands == 50 + 400  # settle ticks, then one command per path point
    assert trace.max_error < 0.005


def test_circle_is_traced_within_5_mm_in_closed_loop(sim, params):
    trace = FootTrace(params.leg, circle_path(CENTER, 0.030, 300))
    run_closed_loop(sim, trace)
    assert trace.max_error < 0.005


def test_unreachable_path_is_rejected_when_the_trace_is_built(params):
    path = square_path(CENTER, 0.060, 400)
    path[10] = (0.3, 0.0, 0.0)
    with pytest.raises(UnreachableError, match="10"):
        FootTrace(params.leg, path)


def test_max_error_is_not_available_before_the_trace_ends(sim, params):
    trace = FootTrace(params.leg, square_path(CENTER, 0.060, 40))
    trace.step(sim.joint_angles())
    assert not trace.done
    with pytest.raises(RuntimeError):
        _ = trace.max_error


def test_step_after_done_keeps_returning_none(sim, params):
    trace = FootTrace(params.leg, square_path(CENTER, 0.060, 8), settle_ticks=2)
    run_closed_loop(sim, trace)
    assert trace.step(sim.joint_angles()) is None
    assert np.isfinite(trace.max_error)
