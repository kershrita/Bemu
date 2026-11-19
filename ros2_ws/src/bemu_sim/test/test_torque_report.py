import dataclasses
import math

import numpy as np
import pytest

from bemu_gait import GAITS
from bemu_sim.torque_report import (
    MASSES,
    MOTIONS,
    Scenario,
    ScenarioResult,
    build_matrix,
    electrical_power,
    params_for_mass,
    report_actuators,
    run_scenario,
    top_speeds,
    verdict,
)


def test_top_speeds(params, gait_params):
    speeds = top_speeds(params, gait_params)
    assert tuple(speeds) == tuple(GAITS)
    assert speeds["tripod"]["forward"] == pytest.approx(0.12, abs=1e-9)
    assert speeds["ripple"]["forward"] == pytest.approx(0.06, abs=1e-9)
    assert speeds["wave"]["forward"] == pytest.approx(0.024, abs=1e-9)
    for gait in GAITS:
        assert speeds[gait]["sideways"] == speeds[gait]["forward"]
    farthest = math.hypot(0.1 + 0.12 * math.cos(math.pi / 4), 0.06 + 0.12 * math.sin(math.pi / 4))
    assert speeds["tripod"]["turn"] == pytest.approx(0.12 / farthest, abs=1e-3)


def test_matrix_has_48_scenarios_in_order(params, gait_params):
    matrix = build_matrix(params, gait_params)
    assert len(matrix) == 48
    assert len(set(matrix)) == 48
    assert sorted({s.total_mass for s in matrix}) == sorted(MASSES)
    for mass in MASSES:
        mine = [s for s in matrix if s.total_mass == mass]
        assert len(mine) == 16
        assert sum(s.motion == "stand" for s in mine) == 1
        for gait in GAITS:
            forward = sorted(s.fraction for s in mine if s.gait == gait and s.motion == "forward")
            assert forward == [0.25, 0.5, 1.0]
            for motion in ("sideways", "turn"):
                assert [s.fraction for s in mine if s.gait == gait and s.motion == motion] == [1.0]
    masses_in_order = [s.total_mass for s in matrix]
    assert masses_in_order == sorted(masses_in_order)


def test_params_for_mass(params):
    assert params_for_mass(params, 2.5).body.mass == pytest.approx(1.39, abs=1e-9)
    assert params_for_mass(params, 2.0).body.mass == pytest.approx(0.89, abs=1e-9)
    assert params_for_mass(params, 2.5).leg == params.leg


@pytest.mark.parametrize("mass", [0.5, 1.11, float("nan"), float("inf"), -1.0])
def test_a_mass_that_cannot_hold_the_legs_is_rejected(params, mass):
    with pytest.raises(ValueError, match="mass"):
        params_for_mass(params, mass)


def test_report_actuators_clip_at_the_stall_torque(params, actuators):
    report = report_actuators(actuators, params.servo)
    assert report.max_torque == params.servo.stall_torque
    assert report.kp == actuators.kp and report.kv == actuators.kv
    assert actuators.max_torque == pytest.approx(1.47)  # the design limit is untouched


def test_electrical_power(params):
    servo = params.servo
    kt = servo.stall_torque / servo.stall_current
    assert electrical_power(servo, np.full(18, kt)) == pytest.approx(18 * 12 * (0.03 + 1.0))
    assert electrical_power(servo, np.zeros(18)) == pytest.approx(18 * 12 * 0.03)
    with pytest.raises(ValueError):
        electrical_power(servo, np.zeros(17))


def fake(joints, *, peak=0.5, rms=0.3, margin=0.08, at_limit=0):
    peaks = np.full(18, 0.1)
    peaks[3] = peak
    rmss = np.full(18, 0.05)
    rmss[5] = rms
    return ScenarioResult(
        scenario=Scenario("tripod", "forward", 1.0, 2.5),
        command=(0.12, 0.0, 0.0),
        joints=joints,
        peak_torque=peaks,
        rms_torque=rmss,
        mean_abs_torque=np.full(18, 0.1),
        peak_speed=np.full(18, 2.0),
        steps_at_limit=np.full(18, at_limit),
        min_margin=margin,
        max_slip=0.004,
        power=100.0,
    )


def test_verdict_boundaries(params):
    from bemu_description.names import joint_order

    servo = params.servo
    half = 0.5 * servo.stall_torque
    assert verdict(fake(joint_order(), peak=half - 1e-6), servo).peak_ok
    assert not verdict(fake(joint_order(), peak=half), servo).peak_ok
    assert verdict(fake(joint_order(), rms=servo.rated_torque - 1e-6), servo).rms_ok
    assert not verdict(fake(joint_order(), rms=servo.rated_torque), servo).rms_ok
    assert not verdict(fake(joint_order(), margin=0.014), servo).stable
    assert verdict(fake(joint_order(), margin=None), servo).stable
    v = verdict(fake(joint_order(), peak=1.0, rms=0.49, at_limit=3), servo)
    assert v.peak_vs_stall == pytest.approx(1.0 / servo.stall_torque)
    assert v.rms_vs_rated == pytest.approx(0.49 / servo.rated_torque)
    assert v.speed_vs_max == pytest.approx(2.0 / servo.max_speed)
    assert v.steps_at_limit == 3 * 18


@pytest.mark.parametrize(
    ("args", "word"),
    [
        (("gallop", "forward", 1.0, 2.5), "gallop"),
        (("tripod", "hop", 1.0, 2.5), "hop"),
        (("tripod", "forward", 0.0, 2.5), "fraction"),
        (("tripod", "forward", -0.5, 2.5), "fraction"),
        (("tripod", "forward", float("nan"), 2.5), "fraction"),
    ],
)
def test_a_bad_scenario_is_rejected_naming_the_problem(args, word):
    with pytest.raises(ValueError, match=word):
        Scenario(*args)


def test_motions_are_the_documented_ones():
    assert MOTIONS == ("stand", "forward", "sideways", "turn")


def test_a_stand_scenario_runs(params, actuators, gait_params):
    result = run_scenario(params, actuators, gait_params, Scenario("tripod", "stand", 0.0, 2.5))
    assert result.peak_torque.shape == (18,) and result.rms_torque.shape == (18,)
    assert result.peak_torque.max() > 0.2
    assert result.min_margin is None and result.max_slip is None
    assert result.steps_at_limit.sum() == 0
    assert (result.rms_torque <= result.peak_torque + 1e-12).all()


def test_a_walking_scenario_runs(params, actuators, gait_params):
    scenario = Scenario("tripod", "forward", 0.5, 2.5)
    result = run_scenario(params, actuators, gait_params, scenario, warmup=1.0, seconds=3.0)
    assert (result.rms_torque <= result.peak_torque + 1e-12).all()
    assert result.min_margin > 0.015
    assert result.max_slip is not None
    assert result.command == pytest.approx((0.06, 0.0, 0.0))
    assert 18 * 12 * 0.03 < result.power < 18 * 12 * 2.7


def test_run_scenario_clips_at_stall_and_counts_the_design_limit(params, actuators, gait_params):
    # Review finding: a walk that needs more than the design clip must show it, not be clipped,
    # and the steps over the design limit must be counted.
    low = dataclasses.replace(actuators, max_torque=0.3)
    scenario = Scenario("tripod", "forward", 0.5, 2.5)
    result = run_scenario(params, low, gait_params, scenario, warmup=1.0, seconds=2.0)
    assert result.peak_torque.max() > 0.3 + 1e-6  # not clipped at the design limit
    assert result.steps_at_limit.sum() > 0  # counted against the design limit
