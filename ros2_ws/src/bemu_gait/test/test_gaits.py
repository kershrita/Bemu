import pytest

from bemu_gait import GAITS, RIPPLE, TRIPOD, WAVE, feet_down
from bemu_params import LEG_CODES


def offsets(gait):
    return tuple(gait.offsets[leg] for leg in LEG_CODES)


def test_gait_table_matches_the_locomotion_doc():
    assert tuple(GAITS) == ("tripod", "ripple", "wave")
    assert (TRIPOD.duty, offsets(TRIPOD)) == (1 / 2, (0, 1 / 2, 0, 1 / 2, 0, 1 / 2))
    assert (RIPPLE.duty, offsets(RIPPLE)) == (2 / 3, (0, 1 / 3, 2 / 3, 2 / 3, 1 / 3, 0))
    assert (WAVE.duty, offsets(WAVE)) == (5 / 6, (1 / 2, 2 / 3, 5 / 6, 0, 1 / 6, 1 / 3))
    assert [g.name for g in GAITS.values()] == ["tripod", "ripple", "wave"]


@pytest.mark.parametrize(("gait", "down"), [(TRIPOD, 3), (RIPPLE, 4), (WAVE, 5)])
def test_number_of_feet_down_is_constant(gait, down):
    for k in range(200):
        assert feet_down(gait, (k + 0.37) / 200) == down, (gait.name, k)


def swing_start(gait, leg):
    """Master phase at which a leg lifts off: where its own phase crosses the duty factor."""
    return (gait.duty - gait.offsets[leg]) % 1.0


def test_wave_lifts_one_leg_at_a_time_rear_to_front_on_the_left_then_the_right():
    order = sorted(LEG_CODES, key=lambda leg: swing_start(WAVE, leg))
    assert order == ["lr", "lm", "lf", "rr", "rm", "rf"]
    starts = [swing_start(WAVE, leg) for leg in order]
    gaps = [b - a for a, b in zip(starts, starts[1:], strict=False)]
    assert gaps == pytest.approx([1 / 6] * 5)


def test_tripod_legs_move_in_two_groups_of_three():
    groups = {swing_start(TRIPOD, leg) for leg in LEG_CODES}
    assert len(groups) == 2
    assert {leg for leg in LEG_CODES if swing_start(TRIPOD, leg) == min(groups)} in (
        {"lf", "lr", "rm"},
        {"lm", "rf", "rr"},
    )
