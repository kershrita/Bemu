import math

import pytest

from bemu_kinematics import stand_height
from bemu_teleop.mapping import (
    Action,
    JoyConfig,
    KeyboardMapper,
    joy_actions,
    joy_command,
    top_speeds,
)


def test_top_speeds(params, gait_params):
    assert top_speeds(params, gait_params, "tripod")[:2] == pytest.approx((0.12, 0.12))
    assert top_speeds(params, gait_params, "tripod")[2] == pytest.approx(0.12 / 0.2349, abs=1e-3)
    assert top_speeds(params, gait_params, "ripple")[0] == pytest.approx(0.06)
    assert top_speeds(params, gait_params, "wave")[0] == pytest.approx(0.024)
    with pytest.raises(ValueError):
        top_speeds(params, gait_params, "nope")


@pytest.fixture
def keys(params, gait_params):
    return KeyboardMapper(params, gait_params)


def test_forward_steps_and_stops_at_the_top_speed(keys):
    for _ in range(3):
        keys.press("w")
    assert keys.velocity[0] == pytest.approx(0.6 * 0.12)
    for _ in range(10):
        keys.press("w")
    assert keys.velocity[0] == pytest.approx(0.12)
    for _ in range(20):
        keys.press("s")
    assert keys.velocity[0] == pytest.approx(-0.12)


def test_direction_signs_and_stop(keys):
    keys.press("a")
    keys.press("q")
    assert keys.velocity[1] > 0 and keys.velocity[2] > 0  # left and counter-clockwise
    keys.press("d")
    keys.press("d")
    keys.press("e")
    keys.press("e")
    assert keys.velocity[1] < 0 and keys.velocity[2] < 0
    keys.press(" ")
    assert keys.velocity == (0.0, 0.0, 0.0)


def test_unknown_keys_do_nothing(keys):
    keys.press("w")
    before = keys.velocity
    assert keys.press("z") is None and keys.press("") is None and keys.press("\x1b") is None
    assert keys.velocity == before


def test_gait_keys_and_clamping_to_the_new_top_speed(keys):
    for _ in range(5):
        keys.press("w")
    assert keys.velocity[0] == pytest.approx(0.12)
    assert keys.press("2") == Action("gait", gait="ripple")
    assert keys.gait == "ripple"
    assert keys.velocity[0] == pytest.approx(0.06)
    assert keys.press("3") == Action("gait", gait="wave")
    assert keys.velocity[0] == pytest.approx(0.024)
    assert keys.press("1") == Action("gait", gait="tripod")


def test_body_height_keys_clamp_at_ten_millimetres(params, keys):
    base = stand_height(params)
    assert keys.press("r") == Action("height", height=pytest.approx(base + 0.005))
    keys.press("r")
    assert keys.press("r").height == pytest.approx(base + 0.010)
    for _ in range(6):
        action = keys.press("f")
    assert action.height == pytest.approx(base - 0.010)


def test_stand_sit_quit(keys):
    assert keys.press("h") == Action("stand")
    assert keys.press("j") == Action("sit")
    assert keys.press("x") == Action("quit")


TOP = (0.12, 0.12, 0.5)
CONFIG = JoyConfig()


def axes(x=0.0, y=0.0, turn=0.0):
    return [x, y, 0.0, turn, 0.0, 0.0]


def buttons(*pressed):
    out = [0] * 8
    for i in pressed:
        out[i] = 1
    return out


def test_sticks_map_to_speeds():
    assert joy_command(axes(), buttons(), CONFIG, TOP) == (0.0, 0.0, 0.0)
    assert joy_command(axes(y=1.0), buttons(), CONFIG, TOP) == pytest.approx((0.12, 0.0, 0.0))
    assert joy_command(axes(x=-1.0), buttons(), CONFIG, TOP) == pytest.approx((0.0, -0.12, 0.0))
    assert joy_command(axes(turn=1.0), buttons(), CONFIG, TOP) == pytest.approx((0.0, 0.0, 0.5))
    assert joy_command(axes(y=0.05), buttons(), CONFIG, TOP)[0] == 0.0
    assert joy_command(axes(y=0.55), buttons(), CONFIG, TOP)[0] == pytest.approx(
        (0.55 - 0.1) / 0.9 * 0.12
    )
    slow = joy_command(axes(y=1.0), buttons(CONFIG.button_slow), CONFIG, TOP)
    assert slow[0] == pytest.approx(0.3 * 0.12)


@pytest.mark.parametrize("bad", [math.nan, math.inf, 1.5, -2.0])
def test_bad_axis_values_are_rejected(bad):
    with pytest.raises(ValueError):
        joy_command(axes(y=bad), buttons(), CONFIG, TOP)


def test_a_short_axes_list_is_rejected():
    with pytest.raises(ValueError):
        joy_command([0.0, 0.0], buttons(), CONFIG, TOP)


def test_buttons_fire_once_on_the_press():
    none = buttons()
    stand = buttons(CONFIG.button_stand)
    assert joy_actions(stand, none, CONFIG) == [Action("stand")]
    assert joy_actions(stand, stand, CONFIG) == []
    both = buttons(CONFIG.button_stand, CONFIG.button_wave)
    assert joy_actions(both, stand, CONFIG) == [Action("gait", gait="wave")]
    assert joy_actions(buttons(CONFIG.button_sit), none, CONFIG) == [Action("sit")]
    assert joy_actions([], none, CONFIG) == []  # a short button list is not an error
