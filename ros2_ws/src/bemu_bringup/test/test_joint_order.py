import pytest

from bemu_bringup.joint_order import angles_by_name


def test_angles_from_joint_state_are_matched_by_name():
    names = ["lf_tibia_joint", "lf_coxa_joint", "lf_femur_joint"]
    positions = [-1.753631, 0.0, 0.300494]
    assert angles_by_name(names, positions, "lf") == pytest.approx((0.0, 0.300494, -1.753631))


def test_extra_joints_are_ignored():
    names = ["rf_coxa_joint", "lf_femur_joint", "lf_coxa_joint", "lf_tibia_joint"]
    positions = [9.0, 0.3, 0.1, -1.7]
    assert angles_by_name(names, positions, "lf") == pytest.approx((0.1, 0.3, -1.7))


def test_missing_joint_raises_key_error_naming_it():
    with pytest.raises(KeyError, match="lf_tibia_joint"):
        angles_by_name(["lf_coxa_joint", "lf_femur_joint"], [0.0, 0.0], "lf")


def test_mismatched_lengths_are_rejected():
    with pytest.raises(ValueError, match="positions"):
        angles_by_name(["lf_coxa_joint"], [0.0, 1.0], "lf")
