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
    assert 0.060 < params.leg.coxa_mass < 0.090  # from the CAD (version 1.5b)
    assert params.stance.foot_radius == pytest.approx(0.120)
    assert params.servo.stall_torque == pytest.approx(2.94)


def test_body_and_foot_load_in_si(params):
    assert params.leg.foot_radius == pytest.approx(0.008)
    assert params.body.size == pytest.approx((0.240, 0.160, 0.05808))  # z from the CAD
    legs = params.leg.coxa_mass + params.leg.femur_mass + params.leg.tibia_mass
    assert params.body.mass == pytest.approx(2.5 - 6 * legs, abs=1e-6)  # the chassis budget


def test_servo_report_constants_load(params):
    assert params.servo.rated_torque == pytest.approx(0.98)
    assert params.servo.supply_voltage == pytest.approx(12.0)
    assert params.servo.stall_current == pytest.approx(2.7)
    assert params.servo.idle_current == pytest.approx(0.03)


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
        ("body.size.z", 250, "body.size"),
        ("servo.rated_torque_nm", None, "servo.rated_torque_nm"),
        ("servo.rated_torque_nm", 3.5, "servo.rated_torque_nm"),
        ("servo.supply_voltage_v", -1, "servo.supply_voltage_v"),
        ("servo.stall_current_a", 0, "servo.stall_current_a"),
        ("servo.idle_current_a", 5, "servo.idle_current_a"),
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


def test_link_inertia_from_the_cad_is_loaded_in_si(params):
    femur = params.leg.link_inertia["femur"]
    assert 0.06 < femur.com[0] < 0.08  # the knee servo puts the centre of mass far out
    assert femur.inertia[1] > 1e-5  # about the COM, kg m^2


def test_a_link_inertia_that_is_not_physical_is_rejected(tmp_path, params_path):
    text = params_path.read_text()
    bad = text.replace("inertia_g_mm2: [20295,", "inertia_g_mm2: [-20295,", 1)
    assert bad != text
    path = tmp_path / "p.yaml"
    path.write_text(bad)
    with pytest.raises(ParamsError, match="link_inertia.femur"):
        load_params(path)


def test_the_inertia_block_is_optional(tmp_path, params_path):
    lines = [
        line
        for line in params_path.read_text().splitlines()
        if "com_mm" not in line and "link_inertia" not in line and "# about the COM" not in line
    ]
    path = tmp_path / "p.yaml"
    path.write_text("\n".join(lines) + "\n")
    assert load_params(path).leg.link_inertia is None
