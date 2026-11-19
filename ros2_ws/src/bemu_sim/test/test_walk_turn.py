import math

import numpy as np

from bemu_gait import Command, Walker
from bemu_sim.robot import RobotSim
from bemu_sim.walking import heading_change, stand_robot, walk


def test_turns_360_degrees_in_place(robot_mjcf, params, gait_params, actuators):
    sim = RobotSim(robot_mjcf)
    stand_robot(sim, params)
    walker = Walker(params, gait_params)

    def schedule(state, walker):
        # the test, not the walker, decides when a full turn is done
        walker.set_command(
            Command(omega=0.4) if state.heading_change < 2 * math.pi - 0.15 else Command()
        )

    log = walk(sim, walker, 40.0, schedule=schedule)
    assert abs(heading_change(log) - 2 * math.pi) < math.radians(10)
    assert np.linalg.norm(log.position[-1, :2] - log.position[0, :2]) < 0.05
    assert log.margin.min() > 0.015
    assert log.peak_torque < actuators.max_torque
