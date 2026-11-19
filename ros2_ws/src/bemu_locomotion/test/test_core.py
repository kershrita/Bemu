import math

import pytest

from bemu_description.names import joint_order
from bemu_kinematics import BodyPose, pose_angles, sit_height, stand_height
from bemu_locomotion.core import LocomotionCore, Mode
from bemu_params import LEG_CODES

RATE = 50.0
DT = 1.0 / RATE


def flat(angles):
    return [a for leg in LEG_CODES for a in angles[leg]]


@pytest.fixture
def sit(params):
    return flat(pose_angles(params, BodyPose(sit_height(params))))


@pytest.fixture
def stand(params):
    return flat(pose_angles(params, BodyPose(stand_height(params))))


def feed(core, values, *, shuffled=True, extra=True):
    names = list(joint_order())
    values = list(values)
    if shuffled:
        names, values = names[::-1], values[::-1]
    if extra:
        names, values = [*names, "unrelated_joint"], [*values, 9.9]
    return core.on_joint_state(names, values)


def make(params, gait_params, sit, **kwargs):
    core = LocomotionCore(params, gait_params, **kwargs)
    assert feed(core, sit)
    return core


def run(core, ticks, start=0.0):
    out = None
    for k in range(ticks):
        out = core.tick(start + k * DT)
    return out


STAND_UP_TICKS = round(2.0 * RATE) + 1


def standing(params, gait_params, sit, **kwargs):
    core = make(params, gait_params, sit, **kwargs)
    run(core, STAND_UP_TICKS)
    assert core.mode == Mode.STANDING
    return core


def test_no_tick_before_a_joint_state(params, gait_params):
    assert LocomotionCore(params, gait_params).tick(0.0) is None


def test_commands_are_in_joint_order_whatever_the_state_order(params, gait_params, sit):
    core = LocomotionCore(params, gait_params, auto_stand=False)
    feed(core, sit, shuffled=True)
    tick = core.tick(0.0)
    assert tick.mode == Mode.SITTING
    assert tick.command == pytest.approx(sit, abs=1e-9)


@pytest.mark.parametrize("bad", ["missing", "nan", "inf", "length"])
def test_an_incomplete_or_bad_joint_state_is_rejected(params, gait_params, sit, bad):
    core = LocomotionCore(params, gait_params, auto_stand=False)
    names, values = list(joint_order()), list(sit)
    if bad == "missing":
        del names[3], values[3]
    elif bad == "nan":
        values[2] = math.nan
    elif bad == "inf":
        values[2] = math.inf
    else:
        values = values[:-1]
    assert core.on_joint_state(names, values) is False
    assert core.tick(0.0) is None  # nothing stored
    assert feed(core, sit)
    assert core.on_joint_state(names, values) is False  # a bad one later keeps the good one
    assert core.tick(0.1).command == pytest.approx(sit, abs=1e-9)


def test_auto_stand_ends_at_the_stand_pose(params, gait_params, sit, stand):
    core = make(params, gait_params, sit)
    first = core.tick(0.0)
    assert first.mode == Mode.STANDING_UP
    last = run(core, STAND_UP_TICKS - 1, start=DT)
    assert last.command == pytest.approx(stand, abs=1e-9)
    assert core.mode == Mode.STANDING


def test_without_auto_stand_it_sits_until_asked(params, gait_params, sit):
    core = make(params, gait_params, sit, auto_stand=False)
    assert run(core, 10).mode == Mode.SITTING
    assert core.request_stand() == (True, "standing up")
    assert core.tick(1.0).mode == Mode.STANDING_UP


def test_stand_is_refused_from_the_wrong_state_or_pose(params, gait_params, sit):
    core = make(params, gait_params, sit, auto_stand=False)
    far = list(sit)
    far[1] += 0.5
    feed(core, far)
    ok, message = core.request_stand()
    assert not ok and "sit pose" in message
    feed(core, sit)
    assert core.request_stand()[0]
    assert not core.request_stand()[0]  # already standing up
    run(core, STAND_UP_TICKS)
    assert core.mode == Mode.STANDING
    assert not core.request_stand()[0]  # already standing


def test_stand_needs_a_joint_state(params, gait_params):
    core = LocomotionCore(params, gait_params, auto_stand=False)
    ok, message = core.request_stand()
    assert not ok and "joint" in message


def moved(a, b):
    return max(abs(x - y) for x, y in zip(a, b, strict=True))


def test_velocity_walks_and_the_timeout_stops_it(params, gait_params, sit):
    core = standing(params, gait_params, sit)
    t = 10.0
    core.set_velocity(0.1, 0.0, 0.0, now=t)
    first = core.tick(t)
    walking = [core.tick(t + (k + 1) * DT) for k in range(20)]  # 0.4 s, still fresh
    assert moved(first.command, walking[-1].command) > 0.01
    # After the timeout the commanded velocity is zero: the stride slews down and the feet
    # end up planted: the targets stop changing.
    for k in range(20, 300):
        last = core.tick(t + (k + 1) * DT)
    again = core.tick(t + 301 * DT)
    assert moved(last.command, again.command) < 1e-6


def test_a_clock_that_goes_backwards_does_not_extend_the_timeout(params, gait_params, sit):
    core = standing(params, gait_params, sit)
    core.set_velocity(0.1, 0.0, 0.0, now=100.0)
    for k in range(400):
        core.tick(3.0 + k * DT)  # the clock restarted far earlier than the command's stamp
    quiet = core.tick(3.0 + 401 * DT)
    again = core.tick(3.0 + 402 * DT)
    assert moved(quiet.command, again.command) < 1e-6


def test_bad_velocity_is_rejected_and_the_old_one_kept(params, gait_params, sit):
    core = standing(params, gait_params, sit)
    core.set_velocity(0.1, 0.0, 0.0, now=0.0)
    for bad in (math.nan, math.inf):
        with pytest.raises(ValueError):
            core.set_velocity(bad, 0.0, 0.0, now=0.0)
    before = core.tick(0.01)
    after = core.tick(0.03)
    assert moved(before.command, after.command) > 0.0  # still walking on the 0.1 m/s command


def test_velocity_is_ignored_while_sitting(params, gait_params, sit):
    core = make(params, gait_params, sit, auto_stand=False)
    core.set_velocity(0.1, 0.0, 0.0, now=0.0)
    first = core.tick(0.0)
    second = core.tick(DT)
    assert moved(first.command, second.command) == 0.0


def test_body_pose_applies_and_an_unreachable_one_is_rejected(params, gait_params, sit):
    core = standing(params, gait_params, sit)
    low = BodyPose(stand_height(params) - 0.01)
    core.set_body_pose(low)
    out = run(core, 200, start=20.0)
    assert out.command == pytest.approx(flat(pose_angles(params, low)), abs=1e-3)
    with pytest.raises(ValueError):
        core.set_body_pose(BodyPose(1.0))
    out2 = run(core, 50, start=40.0)
    assert out2.command == pytest.approx(out.command, abs=1e-3)


def test_gait_selection(params, gait_params, sit):
    core = standing(params, gait_params, sit)
    assert core.tick(10.0).gait == "tripod"
    core.set_gait("ripple")
    assert core.tick(10.02).gait == "ripple"
    with pytest.raises(ValueError):
        core.set_gait("nope")


def test_tick_reports_six_phases_and_feet(params, gait_params, sit):
    core = standing(params, gait_params, sit)
    core.set_velocity(0.1, 0.0, 0.0, now=0.0)
    out = run(core, 100)
    assert len(out.phase) == 6 and len(out.foot_down) == 6
    assert all(0.0 <= p < 1.0 for p in out.phase)
    assert out.stability_margin > 0.0


def test_sit_while_walking_stops_then_sits(params, gait_params, sit):
    core = standing(params, gait_params, sit)
    core.set_velocity(0.1, 0.0, 0.0, now=0.0)
    run(core, 150)
    assert core.request_sit() == (True, "stopping, then sitting down")
    assert core.mode == Mode.STOPPING
    previous = core.tick(5.0).command
    modes = {Mode.STOPPING}
    t = 5.0
    for _ in range(2000):
        t += DT
        out = core.tick(t)
        assert moved(previous, out.command) < 0.05  # no jump
        previous = out.command
        modes.add(out.mode)
        if core.mode == Mode.SITTING:
            break
    assert {Mode.STOPPING, Mode.SITTING_DOWN} <= modes
    assert core.mode == Mode.SITTING
    assert previous == pytest.approx(sit, abs=1e-6)
    assert not core.request_sit()[0]  # already sitting


def test_sit_is_refused_when_not_standing(params, gait_params, sit):
    core = make(params, gait_params, sit)
    run(core, 5)
    assert core.mode == Mode.STANDING_UP
    assert not core.request_sit()[0]
    run(core, STAND_UP_TICKS)
    assert core.request_sit()[0]
    assert not core.request_sit()[0]  # already stopping


def test_a_sit_that_cannot_settle_is_cancelled(params, gait_params, sit, monkeypatch):
    core = standing(params, gait_params, sit)
    monkeypatch.setattr(core, "_settled", lambda out: False)
    assert core.request_sit()[0]
    t = 0.0
    for _ in range(int(11 / DT)):
        t += DT
        core.tick(t)
    assert core.mode == Mode.STANDING
    assert any("settle" in n for n in core.pop_notices())
    assert core.pop_notices() == []
