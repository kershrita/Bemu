import math

import pytest
import yaml

from bemu_params import LEG_CODES, ParamsError, load_params


def edited(tmp_path, params_path, **changes):
    """Copy the real file with dotted-key edits applied; a value of None removes the key."""
    data = yaml.safe_load(params_path.read_text())
    for dotted, value in changes.items():
        *parents, last = dotted.split(".")
        node = data
        for part in parents:
            node = node[part]
        if value is None:
            del node[last]
        else:
            node[last] = value
    out = tmp_path / "robot_params.yaml"
    out.write_text(yaml.safe_dump(data))
    return out


def test_loads_and_converts_to_si(params):
    assert params.leg.coxa_length == pytest.approx(0.045)
    assert params.leg.femur_length == pytest.approx(0.065)
    assert params.leg.tibia_length == pytest.approx(0.110)
    assert params.leg.tibia_limits == pytest.approx((-2.617994, -0.349066), abs=1e-6)
    assert params.leg.coxa_mass == pytest.approx(0.075)
    assert params.stance.foot_radius == pytest.approx(0.120)
    assert params.servo.stall_torque == pytest.approx(2.94)


def test_body_and_foot_load_in_si(params):
    assert params.leg.foot_radius == pytest.approx(0.008)
    assert params.body.size == pytest.approx((0.240, 0.160, 0.056))
    assert params.body.mass == pytest.approx(1.39)


def test_six_mounts_in_order(params):
    assert tuple(params.mounts) == LEG_CODES
    assert params.mounts["lf"].yaw == pytest.approx(math.radians(45))
    assert params.mounts["rr"].x == pytest.approx(-0.100)


def test_env_var_is_used_when_no_path(params_path, monkeypatch):
    monkeypatch.setenv("BEMU_PARAMS", str(params_path))
    assert load_params().leg.coxa_length == pytest.approx(0.045)


def test_no_path_and_no_env_var_is_an_error(monkeypatch):
    monkeypatch.delenv("BEMU_PARAMS", raising=False)
    with pytest.raises(ParamsError, match="BEMU_PARAMS"):
        load_params()


@pytest.mark.parametrize(
    ("dotted_key", "value", "expected_in_message"),
    [
        ("leg.femur_length", None, "leg.femur_length"),
        ("leg.femur_length", "long", "leg.femur_length"),
        ("leg.femur_length", -5, "leg.femur_length"),
        ("leg.joint_limits.tibia", [-20, -150], "leg.joint_limits.tibia"),
        ("leg.joint_limits.coxa", [float("nan"), 45], "leg.joint_limits.coxa"),
        ("leg.joint_limits.coxa", [-45, float("nan")], "leg.joint_limits.coxa"),
        ("leg.joint_limits.coxa", [float("-inf"), float("inf")], "leg.joint_limits.coxa"),
        ("body.mounts.rr", None, "body.mounts"),
        ("leg.tibia_length", 20, "stance"),
        ("units.length", "inch", "units"),
        ("body.size.z", 30, "body.size"),
        ("body.size", None, "body.size"),
        ("body.mass", -1, "body.mass"),
        ("leg.foot_radius", "x", "leg.foot_radius"),
        ("leg.foot_radius", 0, "leg.foot_radius"),
    ],
)
def test_bad_file_raises_params_error_naming_the_key(
    tmp_path, params_path, dotted_key, value, expected_in_message
):
    path = edited(tmp_path, params_path, **{dotted_key: value})
    with pytest.raises(ParamsError, match=expected_in_message):
        load_params(path)
