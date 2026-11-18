import mujoco
import numpy as np
import pytest

from bemu_description.mjcf import build_leg_mjcf
from bemu_description.names import foot_site, hip_site, joint_names
from bemu_kinematics import JointAngles, forward

NEUTRAL = (0.0, 0.300494, -1.753631)


@pytest.fixture
def model(params, actuators):
    return mujoco.MjModel.from_xml_string(build_leg_mjcf(params, actuators))


def test_model_loads_with_three_named_joints_and_actuators(model):
    for name in joint_names("lf"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) >= 0
    assert model.nu == 3 and model.nq == 3


def test_actuator_drives_the_joint_of_the_same_name(model):
    for name in joint_names("lf"):
        act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert model.actuator_trnid[act, 0] == joint


def test_timestep_and_joint_ranges_come_from_params(model, params):
    assert model.opt.timestep == pytest.approx(0.002)
    leg = params.leg
    expected = [leg.coxa_limits, leg.femur_limits, leg.tibia_limits]
    for name, limits in zip(joint_names("lf"), expected, strict=True):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        assert model.jnt_range[joint] == pytest.approx(limits, abs=1e-9)
        assert model.actuator_ctrlrange[act] == pytest.approx(limits, abs=1e-9)


def test_actuator_gains_and_joint_dynamics_come_from_actuator_model(model, actuators):
    assert model.actuator_gainprm[:, 0] == pytest.approx([actuators.kp] * 3)
    assert model.actuator_biasprm[:, 1] == pytest.approx([-actuators.kp] * 3)
    assert model.actuator_biasprm[:, 2] == pytest.approx([-actuators.kv] * 3)
    assert model.dof_armature == pytest.approx([actuators.armature] * 3)
    assert model.dof_damping == pytest.approx([actuators.damping] * 3)
    assert model.dof_frictionloss == pytest.approx([actuators.frictionloss] * 3)


def test_force_limit_is_half_of_stall(model, params, actuators):
    assert actuators.max_torque == pytest.approx(0.5 * params.servo.stall_torque)
    assert model.actuator_forcerange[:, 1] == pytest.approx([1.47, 1.47, 1.47])
    assert model.actuator_forcerange[:, 0] == pytest.approx([-1.47, -1.47, -1.47])


def test_link_masses_come_from_params(model, params):
    leg = params.leg
    body_masses = model.body_mass[1:]
    assert body_masses == pytest.approx([leg.coxa_mass, leg.femur_mass, leg.tibia_mass])


def test_hip_is_fixed_above_a_floor_and_foot_has_friction(model):
    hip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, hip_site("lf"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.site_xpos[hip] == pytest.approx([0.0, 0.0, 0.20])
    floor = [g for g in range(model.ngeom) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE]
    assert len(floor) == 1
    foot_geoms = [
        g for g in range(model.ngeom) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
    ]
    assert len(foot_geoms) == 1
    assert model.geom_size[foot_geoms[0], 0] == pytest.approx(0.008)
    assert model.geom_friction[foot_geoms[0], 0] == pytest.approx(1.0)


@pytest.mark.parametrize(
    "angles",
    [
        (0.0, 0.300494, -1.753631),
        (0.5, 0.0, -1.570796),
        (-0.6, 0.876369, -2.416219),
        (0.2, -0.290826, -0.881021),
    ],
)
def test_foot_site_matches_forward_kinematics(model, params, angles):
    data = mujoco.MjData(model)
    data.qpos[:] = angles
    mujoco.mj_forward(model, data)
    foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, foot_site("lf"))
    hip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, hip_site("lf"))
    relative = data.site_xpos[foot] - data.site_xpos[hip]
    expected = forward(params.leg, JointAngles(*angles))
    assert np.allclose(relative, expected, atol=1e-6)


def test_neutral_keyframe(model):
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "neutral")
    assert key >= 0
    assert model.key_qpos[key] == pytest.approx(NEUTRAL, abs=1e-6)
    assert model.key_ctrl[key] == pytest.approx(NEUTRAL, abs=1e-6)
