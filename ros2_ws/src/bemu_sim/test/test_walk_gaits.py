import itertools

import numpy as np
import pytest

from bemu_gait import GAITS, Command, Walker
from bemu_sim.robot import RobotSim
from bemu_sim.walking import stand_robot, travelled, walk

SPEED = 0.02  # below the wave's top speed: max stride / stance time = 0.06 / 2.5 s = 0.024 m/s
MARGIN = 0.015


def make(robot_mjcf, params, gait_params, gait="tripod"):
    sim = RobotSim(robot_mjcf)
    stand_robot(sim, params)
    return sim, Walker(params, gait_params, gait=gait)


def switching(plan, speed=SPEED):
    """A schedule: constant command, and `plan` = [(time, gait), ...] gait changes."""
    pending = list(plan)

    def schedule(state, walker):
        walker.set_command(Command(vx=speed))
        while pending and state.t >= pending[0][0]:
            walker.set_gait(pending.pop(0)[1])

    return schedule


@pytest.mark.parametrize(
    ("gait", "speed", "seconds"), [("ripple", 0.05, 30.0), ("wave", 0.02, 60.0)]
)
def test_each_gait_walks_straight(robot_mjcf, params, gait_params, actuators, gait, speed, seconds):
    sim, walker = make(robot_mjcf, params, gait_params, gait)
    log = walk(sim, walker, seconds, schedule=switching([], speed))
    forward, lateral = travelled(log)
    assert forward > 1.0
    assert abs(lateral) < 0.1 * forward
    assert log.margin.min() > MARGIN
    assert log.peak_torque < actuators.max_torque
    assert log.scale.min() > 0.9


def test_switching_gaits_while_walking_never_stops(robot_mjcf, params, gait_params, actuators):
    sim, walker = make(robot_mjcf, params, gait_params)
    plan = [(8.0, "ripple"), (16.0, "wave"), (28.0, "tripod")]
    log = walk(sim, walker, 40.0, schedule=switching(plan))
    for start in np.arange(2.0, 39.0, 1.0):  # every 1 s window after the start-up
        window = (log.t >= start) & (log.t < start + 1.0)
        progress = log.position[window][-1, 0] - log.position[window][0, 0]
        assert progress > 0.005, f"stalled at t={start:.0f} s"
    assert log.margin.min() > MARGIN
    assert log.tilt.max() < np.radians(5)
    assert log.peak_torque < actuators.max_torque
    assert set(log.gait) == {"tripod", "ripple", "wave"}
    assert log.gait[-1] == "tripod"
    assert log.scale.min() > 0.9


@pytest.mark.parametrize(("old", "new"), list(itertools.permutations(GAITS, 2)))
def test_every_ordered_gait_pair_switches_safely(
    robot_mjcf, params, gait_params, actuators, old, new
):
    sim, walker = make(robot_mjcf, params, gait_params, old)
    log = walk(sim, walker, 24.0, schedule=switching([(8.0, new)]))
    assert log.gait[-1] == new
    assert log.margin.min() > MARGIN
    assert log.tilt.max() < np.radians(5)
    assert log.peak_torque < actuators.max_torque
    assert travelled(log)[0] > 0.2
