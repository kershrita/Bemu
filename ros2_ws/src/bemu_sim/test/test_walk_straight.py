import math

import numpy as np
import pytest

from bemu_gait import Command, Walker
from bemu_sim.robot import RobotSim
from bemu_sim.walking import max_stance_slip, stand_robot, travelled, walk

MARGIN = 0.015


@pytest.fixture
def robot(robot_mjcf, params, gait_params):
    sim = RobotSim(robot_mjcf)
    stand_robot(sim, params)
    return sim, Walker(params, gait_params)


def constant(command):
    def schedule(state, walker):
        walker.set_command(command)

    return schedule


def test_walks_2_m_straight_with_under_10_percent_drift(robot, actuators, gait_params):
    sim, walker = robot

    def schedule(state, walker):
        start = schedule.start
        if start is None:
            schedule.start = state.position.copy()
            start = schedule.start
        walked = float(np.linalg.norm(state.position[:2] - start[:2]))
        walker.set_command(Command(vx=0.10) if walked < 2.0 else Command())

    schedule.start = None
    log = walk(sim, walker, 45.0, schedule=schedule)
    forward, lateral = travelled(log)
    assert forward >= 2.0
    assert abs(lateral) < 0.2
    assert abs(log.yaw[-1] - log.yaw[0]) < 0.2  # a straight walk should not also turn
    assert log.margin.min() > MARGIN
    assert log.peak_torque < actuators.max_torque
    assert log.scale.min() > 0.9


def test_walk_speed_is_close_to_the_command(robot):
    sim, walker = robot
    log = walk(sim, walker, 20.0, schedule=constant(Command(vx=0.10)))
    steady = (log.t >= 8.0) & (log.t <= 18.0)
    t, x = log.t[steady], log.position[steady, 0]
    speed = (x[-1] - x[0]) / (t[-1] - t[0])
    assert speed == pytest.approx(0.10, rel=0.25)


def test_stopping_stands_still(robot):
    sim, walker = robot
    height = float(sim.body_position()[2])

    def schedule(state, walker):
        walker.set_command(Command(vx=0.10) if state.t < 12.0 else Command())

    log = walk(sim, walker, 20.0, schedule=schedule)
    last_second = log.t >= log.t[-1] - 1.0
    moved = np.linalg.norm(log.position[last_second][-1, :2] - log.position[last_second][0, :2])
    assert moved < 0.005
    assert all(sim.foot_contacts().values())
    assert float(sim.body_position()[2]) == pytest.approx(height, abs=0.003)


def test_max_stance_slip_while_walking(robot):
    sim, walker = robot
    log = walk(sim, walker, 12.0, schedule=constant(Command(vx=0.10)))
    assert max_stance_slip(log) < 0.005


def test_walks_sideways(robot, gait_params):
    sim, walker = robot
    log = walk(sim, walker, 12.0, schedule=constant(Command(vy=0.08)))
    forward, lateral = travelled(log)
    expected = 0.08 * (12.0 - 0.08 / gait_params.max_linear_accel / 2)
    assert lateral > 0
    assert lateral == pytest.approx(expected, rel=0.25)
    assert abs(forward) < 0.1 * lateral
    assert log.margin.min() > MARGIN


def test_a_curve(robot, gait_params):
    sim, walker = robot
    log = walk(sim, walker, 12.0, schedule=constant(Command(vx=0.08, omega=0.2)))
    expected = 0.2 * (12.0 - 0.2 / gait_params.max_angular_accel / 2)
    assert log.yaw[-1] - log.yaw[0] == pytest.approx(expected, rel=0.25)
    assert log.margin.min() > MARGIN
    assert math.isfinite(travelled(log)[0])


def test_tripod_walks_sideways_without_slipping(robot, gait_params):
    sim, walker = robot
    log = walk(sim, walker, 15.0, schedule=constant(Command(vy=0.06)))
    assert max_stance_slip(log) < 0.005
    assert log.margin.min() > MARGIN


@pytest.mark.parametrize(
    ("gait", "command"),
    [
        ("ripple", Command(omega=0.3)),
        ("wave", Command(omega=0.1)),
        ("ripple", Command(vy=0.05)),
    ],
)
def test_other_gaits_turn_and_sidestep_without_slipping(
    robot_mjcf, params, gait_params, actuators, gait, command
):
    sim = RobotSim(robot_mjcf)
    stand_robot(sim, params)
    walker = Walker(params, gait_params, gait=gait)
    log = walk(sim, walker, 20.0, schedule=constant(command))
    assert max_stance_slip(log) < 0.005
    assert log.margin.min() > MARGIN
    assert log.peak_torque < actuators.max_torque
    if command.omega:
        assert abs(log.yaw[-1] - log.yaw[0]) == pytest.approx(command.omega * 19.0, rel=0.25)
