"""The loader's sit-pose check must agree with the real inverse kinematics."""

import yaml

from bemu_kinematics import KinematicsError, inverse
from bemu_params import ParamsError, load_params


def with_thickness(tmp_path, params_path, thickness_mm):
    data = yaml.safe_load(params_path.read_text())
    data["body"]["size"]["z"] = thickness_mm
    out = tmp_path / f"robot_params_{thickness_mm}.yaml"
    out.write_text(yaml.safe_dump(data))
    return out


def loader_accepts(path):
    try:
        load_params(path)
    except ParamsError:
        return False
    return True


def sit_pose_is_reachable(params, thickness_mm):
    foot = (params.stance.foot_radius, 0.0, params.leg.foot_radius - thickness_mm / 2000.0)
    try:
        inverse(params.leg, foot)
    except KinematicsError:
        return False
    return True


def test_loader_agrees_with_inverse_kinematics_on_the_sit_pose(tmp_path, params, params_path):
    outcomes = {}
    for thickness in range(30, 82, 2):
        accepted = loader_accepts(with_thickness(tmp_path, params_path, thickness))
        assert accepted == sit_pose_is_reachable(params, thickness), thickness
        outcomes[thickness] = accepted
    assert any(outcomes.values()) and not all(outcomes.values())
    assert outcomes[48] is True
    assert outcomes[46] is False


def test_the_shipped_body_can_sit(params):
    assert sit_pose_is_reachable(params, params.body.size[2] * 1000.0)
