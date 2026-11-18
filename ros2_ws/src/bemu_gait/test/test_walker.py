import math

import numpy as np
import pytest

from bemu_gait import GAITS, Command, Walker
from bemu_kinematics import BodyPose, BodyPoseError, neutral_feet, pose_angles, stand_height
from bemu_kinematics import stability_margin as margin_of
from bemu_params import LEG_CODES

DT = 0.02


def make(params, gait_params, gait="tripod"):
    return Walker(params, gait_params, gait=gait)


def run(walker, ticks):
    return [walker.step(DT) for _ in range(ticks)]


def per_tick_displacement(outputs):
    worst = 0.0
    for a, b in zip(outputs, outputs[1:], strict=False):
        for leg in LEG_CODES:
            worst = max(worst, float(np.linalg.norm(b.feet[leg] - a.feet[leg])))
    return worst


@pytest.mark.parametrize("gait", list(GAITS))
def test_zero_command_stands_at_the_neutral_points(params, gait_params, gait):
    outputs = run(make(params, gait_params, gait), 100)
    neutral = neutral_feet(params)
    for out in outputs:
        assert out.scale == 1.0
        for leg in LEG_CODES:
            assert out.feet[leg] == pytest.approx(neutral[leg], abs=1e-9)


def test_forward_command_moves_stance_feet_backwards_at_the_commanded_speed(params, gait_params):
    walker = make(params, gait_params)
    walker.set_command(Command(vx=0.1))
    outputs = run(walker, 150)
    checked = 0
    for a, b in zip(outputs[100:], outputs[101:], strict=False):
        if a.in_stance["lf"] and b.in_stance["lf"]:
            velocity = (b.feet["lf"] - a.feet["lf"]) / DT
            assert velocity[:2] == pytest.approx((-0.1, 0.0), rel=0.05, abs=1e-4)
            checked += 1
    assert checked > 10


def test_turning_gives_tangential_strides(params, gait_params):
    omega = 0.4
    walker = make(params, gait_params)
    walker.set_command(Command(omega=omega))
    outputs = run(walker, 150)
    neutral = neutral_feet(params)
    for leg in LEG_CODES:
        rx, ry = neutral[leg][:2]
        expected = np.array([omega * ry, -omega * rx])  # minus omega x r
        for a, b in zip(outputs[100:], outputs[101:], strict=False):
            if a.in_stance[leg] and b.in_stance[leg]:
                velocity = (b.feet[leg] - a.feet[leg])[:2] / DT
                assert velocity == pytest.approx(expected, rel=0.05, abs=1e-4), leg


def test_command_is_slewed_not_jumped(params, gait_params):
    walker = make(params, gait_params)
    walker.set_command(Command(vx=0.12))
    outputs = run(walker, 160)
    first = (outputs[1].feet["lf"] - outputs[0].feet["lf"])[0] / DT
    assert abs(first) < 0.2 * 0.12
    settle = 0.12 / gait_params.max_linear_accel + 0.1
    ticks = math.ceil(settle / DT)
    a, b = outputs[ticks], outputs[ticks + 1]
    leg = next(leg for leg in LEG_CODES if a.in_stance[leg] and b.in_stance[leg])
    later = (b.feet[leg] - a.feet[leg])[0] / DT
    assert later == pytest.approx(-0.12, rel=0.05)


def reference_displacement(params, gait_params):
    """The largest per-tick foot displacement of each gait walking on its own."""
    worst = 0.0
    for gait in GAITS:
        walker = make(params, gait_params, gait)
        walker.set_command(Command(vx=0.04))
        worst = max(worst, per_tick_displacement(run(walker, 400)[100:]))
    return worst


@pytest.mark.parametrize("old", list(GAITS))
@pytest.mark.parametrize("new", list(GAITS))
def test_gait_switch_never_jumps_a_foot(params, gait_params, old, new):
    reference = reference_displacement(params, gait_params)
    for start_tick in (131, 150, 177):  # different points of the cycle
        walker = make(params, gait_params, old)
        walker.set_command(Command(vx=0.04))
        outputs = run(walker, start_tick)
        walker.set_gait(new)
        outputs += run(walker, 300)
        assert per_tick_displacement(outputs) <= 1.5 * reference, (old, new, start_tick)


def test_a_switch_takes_blend_cycles_of_the_old_cycle(params, gait_params):
    walker = make(params, gait_params, "tripod")
    walker.set_command(Command(vx=0.05))
    run(walker, 10)
    walker.set_gait("ripple")
    outputs = run(walker, 130)
    blending = [o.blending for o in outputs]
    assert blending[0] and blending[80]
    assert not blending[-1]
    steps = sum(blending)
    assert steps * DT == pytest.approx(
        gait_params.blend_cycles * gait_params.cycle_time["tripod"], abs=0.1
    )
    assert outputs[-1].gait == "ripple"


def test_a_request_during_a_blend_replaces_the_target(params, gait_params):
    reference = reference_displacement(params, gait_params)
    walker = make(params, gait_params)
    walker.set_command(Command(vx=0.04))
    outputs = run(walker, 120)
    walker.set_gait("ripple")
    outputs += run(walker, 15)
    walker.set_gait("wave")
    outputs += run(walker, 250)
    assert outputs[-1].gait == "wave" and not outputs[-1].blending
    assert per_tick_displacement(outputs) <= 1.5 * reference


def test_unreachable_command_scales_all_strides_together(params, gait_params):
    walker = make(params, gait_params)
    walker.set_command(Command(vx=1.0, omega=3.0))
    outputs = run(walker, 200)  # no exception means every step was reachable
    assert min(o.scale for o in outputs) < 1.0
    assert per_tick_displacement(outputs) < 0.012
    limit = gait_params.max_stride / (0.5 * gait_params.cycle_time["tripod"])
    for a, b in zip(outputs[150:], outputs[151:], strict=False):
        for leg in LEG_CODES:
            if a.in_stance[leg] and b.in_stance[leg]:
                speed = float(np.linalg.norm(b.feet[leg][:2] - a.feet[leg][:2])) / DT
                assert speed <= limit * 1.05, leg


@pytest.mark.parametrize(
    "bad", [Command(vx=float("nan")), Command(omega=float("inf")), "go", {"vx": 1}, None]
)
def test_bad_commands_are_rejected_and_change_nothing(params, gait_params, bad):
    hurt, clean = make(params, gait_params), make(params, gait_params)
    with pytest.raises(ValueError):
        hurt.set_command(bad)
    for walker in (hurt, clean):
        walker.set_command(Command(vx=0.05))
    a, b = run(hurt, 30), run(clean, 30)
    for x, y in zip(a, b, strict=True):
        for leg in LEG_CODES:
            assert x.feet[leg] == pytest.approx(y.feet[leg], abs=1e-12)


def test_unknown_gait_names_the_valid_ones(params, gait_params):
    walker = make(params, gait_params)
    with pytest.raises(ValueError) as info:
        walker.set_gait("gallop")
    for word in ("gallop", "tripod", "ripple", "wave"):
        assert word in str(info.value)
    with pytest.raises(ValueError, match="gallop"):
        make(params, gait_params, "gallop")


@pytest.mark.parametrize("dt", [0.0, -0.02, float("nan"), float("inf"), 0.2])
def test_bad_dt_is_rejected_and_changes_nothing(params, gait_params, dt):
    hurt, clean = make(params, gait_params), make(params, gait_params)
    for walker in (hurt, clean):
        walker.set_command(Command(vx=0.05))
        run(walker, 20)
    with pytest.raises(ValueError):
        hurt.step(dt)
    a, b = run(hurt, 20), run(clean, 20)
    for x, y in zip(a, b, strict=True):
        assert x.feet["lm"] == pytest.approx(y.feet["lm"], abs=1e-12)


def test_unreachable_standing_pose_is_rejected_at_once(params, gait_params):
    walker = make(params, gait_params)
    with pytest.raises(BodyPoseError):
        walker.set_body_pose(BodyPose(0.5))
    with pytest.raises(ValueError):
        walker.set_body_pose(BodyPose(float("nan")))
    run(walker, 5)  # the walker still works with its old pose


def test_a_pose_that_only_blocks_stepping_scales_strides_and_keeps_standing(params, gait_params):
    walker = make(params, gait_params)
    walker.set_body_pose(BodyPose(stand_height(params), x=0.06))  # standing is fine
    walker.set_command(Command(vx=0.1))
    outputs = run(walker, 200)  # no exception
    assert min(o.scale for o in outputs) < 1.0


def test_stability_margin_uses_the_feet_on_the_ground(params, gait_params):
    walker = make(params, gait_params)
    walker.set_command(Command(vx=0.04))
    outputs = run(walker, 200)
    foot_radius = params.leg.foot_radius
    lifted_seen = False
    for out in outputs:
        down = [out.feet[leg][:2] for leg in LEG_CODES if out.feet[leg][2] - foot_radius < 0.002]
        lifted_seen |= len(down) < 6
        assert out.stability_margin == pytest.approx(
            margin_of((0.0, 0.0), np.array(down)), abs=1e-9
        )
    assert lifted_seen
    assert outputs[0].stability_margin > 0.05


def test_body_pose_offset_is_applied(params, gait_params):
    pose = BodyPose(stand_height(params), roll=0.1, x=0.01)
    walker = make(params, gait_params)
    walker.set_body_pose(pose)
    out = walker.step(DT)
    expected = pose_angles(params, pose, out.feet)
    for leg in LEG_CODES:
        assert tuple(out.angles[leg]) == pytest.approx(tuple(expected[leg]), abs=1e-12)
