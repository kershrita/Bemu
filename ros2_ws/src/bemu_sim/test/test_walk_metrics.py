import math

import numpy as np
import pytest

from bemu_params import LEG_CODES
from bemu_sim.walking import (
    WalkLog,
    heading_change,
    max_stance_slip,
    max_swing_drag,
    travelled,
    window,
)


def log_from(position, yaw=None, feet=None, in_stance=None, contact=None):
    n = len(position)
    return WalkLog(
        t=np.arange(n) * 0.02,
        position=np.asarray(position, dtype=float),
        yaw=np.zeros(n) if yaw is None else np.asarray(yaw, dtype=float),
        tilt=np.zeros(n),
        margin=np.full(n, 0.1),
        feet=np.zeros((n, 6, 3)) if feet is None else feet,
        in_stance=np.ones((n, 6), dtype=bool) if in_stance is None else in_stance,
        contact=np.ones((n, 6), dtype=bool) if contact is None else contact,
        scale=np.ones(n),
        gait=["tripod"] * n,
        peak_torque=0.0,
    )


def test_travelled_is_measured_in_the_start_heading_frame():
    # facing +y (yaw 90 degrees): moving +y is forward, moving -x is to the left
    yaw = np.full(3, math.pi / 2)
    log = log_from([(1, 1, 0.09), (1, 3, 0.09), (0, 3, 0.09)], yaw=yaw)
    forward, lateral = travelled(log)
    assert forward == pytest.approx(2.0)
    assert lateral == pytest.approx(1.0)


def test_heading_change_is_last_minus_first():
    assert heading_change(log_from([(0, 0, 0)] * 3, yaw=[0.2, 1.0, 2.2])) == pytest.approx(2.0)


def test_max_stance_slip_is_the_movement_within_one_stance():
    feet = np.zeros((6, 6, 3))
    feet[:, 0, 0] = [0.0, 0.001, 0.003, 0.0, 0.0, 0.0]  # leg lf drifts 3 mm in its first stance
    stance = np.ones((6, 6), dtype=bool)
    stance[3, 0] = False  # lf swings at row 3: the stance interval ends there
    log = log_from([(0, 0, 0)] * 6, feet=feet, in_stance=stance)
    assert max_stance_slip(log) == pytest.approx(0.003)


def test_a_foot_that_only_moves_while_swinging_has_no_slip():
    feet = np.zeros((6, 6, 3))
    feet[:, 1, 0] = [0.0, 0.0, 0.05, 0.1, 0.1, 0.1]
    stance = np.ones((6, 6), dtype=bool)
    stance[2:4, 1] = False
    assert max_stance_slip(log_from([(0, 0, 0)] * 6, feet=feet, in_stance=stance)) == pytest.approx(
        0.0
    )


def test_a_foot_not_yet_touching_the_floor_is_not_slipping():
    feet = np.zeros((6, 6, 3))
    feet[:, 2, 0] = [0.0, 0.004, 0.004, 0.004, 0.004, 0.004]  # settles 4 mm onto the floor
    contact = np.ones((6, 6), dtype=bool)
    contact[0, 2] = False  # at row 0 the foot is still in the air
    assert max_stance_slip(log_from([(0, 0, 0)] * 6, feet=feet, contact=contact)) == pytest.approx(
        0.0
    )
    # the same movement while touching is slip
    assert max_stance_slip(log_from([(0, 0, 0)] * 6, feet=feet)) == pytest.approx(0.004)


def test_log_fields_cover_every_leg():
    assert LEG_CODES == ("lf", "lm", "lr", "rf", "rm", "rr")


def test_swing_drag_is_a_foot_scraping_along_the_floor_while_it_should_be_swinging():
    feet = np.zeros((6, 6, 3))
    feet[:, 3, 0] = [0.0, 0.004, 0.009, 0.009, 0.009, 0.009]
    stance = np.ones((6, 6), dtype=bool)
    stance[:3, 3] = False  # leg rf is flagged swing for the first three rows, touching all along
    log = log_from([(0, 0, 0)] * 6, feet=feet, in_stance=stance)
    assert max_swing_drag(log) == pytest.approx(0.009)
    assert max_stance_slip(log) == pytest.approx(0.0)  # ... and that is not planted-foot slip


def test_swing_drag_ignores_a_foot_that_is_in_the_air():
    feet = np.zeros((6, 6, 3))
    feet[:, 3, 0] = [0.0, 0.05, 0.1, 0.1, 0.1, 0.1]
    stance = np.ones((6, 6), dtype=bool)
    stance[:3, 3] = False
    contact = np.ones((6, 6), dtype=bool)
    contact[:3, 3] = False  # swinging in the air, as it should
    assert (
        max_swing_drag(log_from([(0, 0, 0)] * 6, feet=feet, in_stance=stance, contact=contact))
        == 0.0
    )


def test_window_keeps_the_rows_in_the_time_range():
    log = log_from([(i, 0, 0) for i in range(10)])
    part = window(log, 0.04, 0.12)  # rows at t = 0, 0.02, ... 0.18
    assert list(part.t) == pytest.approx([0.04, 0.06, 0.08, 0.10])
    assert part.position.shape == (4, 3) and part.feet.shape == (4, 6, 3)
    assert len(part.gait) == 4 and part.peak_torque == log.peak_torque
