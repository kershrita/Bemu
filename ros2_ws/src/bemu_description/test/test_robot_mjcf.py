import mujoco
import numpy as np
import pytest

from bemu_description.generate import write_robot_mjcf
from bemu_description.mjcf import build_leg_mjcf
from bemu_description.names import BODY, ROOT_JOINT, foot_site, joint_order
from bemu_description.robot_mjcf import build_robot_mjcf
from bemu_kinematics import neutral_feet, sit_height, stand_height
from bemu_params import LEG_CODES


@pytest.fixture
def model(params, actuators):
    return mujoco.MjModel.from_xml_string(build_robot_mjcf(params, actuators))


def at_keyframe(model, name):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(
        model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, name)
    )
    mujoco.mj_forward(model, data)
    return data


def test_joint_order_lists_all_18_joints_leg_by_leg():
    names = joint_order()
    assert len(names) == len(set(names)) == 18
    assert names[:3] == ("lf_coxa_joint", "lf_femur_joint", "lf_tibia_joint")
    assert names[-3:] == ("rr_coxa_joint", "rr_femur_joint", "rr_tibia_joint")


def test_names_and_counts(model):
    for name in joint_order():
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0
        act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        assert act >= 0
        assert model.actuator_trnid[act, 0] == model.joint(name).id
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ROOT_JOINT) >= 0
    assert (model.nq, model.nv, model.nu) == (25, 24, 18)
    for key in ("sit", "stand"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key) >= 0


def test_actuator_order_follows_joint_order(model):
    for i, name in enumerate(joint_order()):
        assert model.actuator(i).name == name


def test_total_mass_is_the_design_mass(model, params):
    leg = params.leg
    expected = params.body.mass + 6 * (leg.coxa_mass + leg.femur_mass + leg.tibia_mass)
    assert model.body_subtreemass[model.body(BODY).id] == pytest.approx(expected, abs=1e-9)
    assert expected == pytest.approx(2.5)


def test_every_foot_matches_the_maths_at_stand(model, params):
    data = at_keyframe(model, "stand")
    assert data.qpos[2] == pytest.approx(stand_height(params), abs=1e-9)
    for leg, planted in neutral_feet(params).items():
        site = data.site_xpos[model.site(foot_site(leg)).id]
        assert site == pytest.approx(planted, abs=1e-6), leg


def test_sit_keyframe_rests_on_the_floor(model, params):
    data = at_keyframe(model, "sit")
    assert data.qpos[2] == pytest.approx(sit_height(params), abs=1e-9)
    box = model.geom("body_box")
    assert data.geom_xpos[box.id][2] - box.size[2] == pytest.approx(0.0, abs=1e-6)
    for leg in LEG_CODES:
        assert data.site_xpos[model.site(foot_site(leg)).id][2] == pytest.approx(
            params.leg.foot_radius, abs=1e-6
        )


def test_keyframes_hold_ctrl_equal_to_the_joint_angles(model):
    for name in ("sit", "stand"):
        key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, name)
        assert model.key_ctrl[key] == pytest.approx(model.key_qpos[key][7:])


def test_joint_ranges_and_force_limits_come_from_params(model, params, actuators):
    limits = (params.leg.coxa_limits, params.leg.femur_limits, params.leg.tibia_limits)
    for i, name in enumerate(joint_order()):
        assert tuple(model.joint(name).range) == pytest.approx(limits[i % 3], abs=1e-9)
        assert tuple(model.actuator(i).forcerange) == pytest.approx(
            (-actuators.max_torque, actuators.max_torque)
        )


def test_only_the_body_and_the_feet_collide_with_the_floor(model):
    floor = model.geom("floor").id
    for i in range(model.ngeom):
        if i == floor:
            continue
        hits = bool(model.geom_contype[floor] & model.geom_conaffinity[i]) or bool(
            model.geom_contype[i] & model.geom_conaffinity[floor]
        )
        name = model.geom(i).name
        assert hits == (name == "body_box" or name.endswith("_foot_tip")), name


def test_one_leg_generator_still_loads(params, actuators):
    model = mujoco.MjModel.from_xml_string(build_leg_mjcf(params, actuators))
    assert (model.nq, model.nu) == (3, 3)


def test_write_robot_mjcf_creates_the_file(tmp_path, params, actuators):
    path = write_robot_mjcf(params, actuators, tmp_path / "out")
    assert path.name == "robot.xml" and path.is_file()
    assert isinstance(np.asarray(mujoco.MjModel.from_xml_path(str(path)).nq), np.ndarray)


def test_link_centres_of_mass_and_inertia_come_from_the_cad(params, actuators):
    import mujoco

    model = mujoco.MjModel.from_xml_string(build_robot_mjcf(params, actuators))
    for link, joint in (("coxa", "coxa"), ("femur", "femur"), ("tibia", "tibia")):
        body = model.body(f"lf_{link}")
        item = params.leg.link_inertia[joint]
        assert body.mass[0] == pytest.approx(getattr(params.leg, f"{joint}_mass"), rel=1e-6)
        # MuJoCo reports the principal frame; the offset of the centre of mass is what matters
        assert abs(body.ipos[0] - item.com[0]) < 1e-9
        assert abs(body.ipos[2] - item.com[2]) < 1e-9
        assert sum(body.inertia) == pytest.approx(
            (item.inertia[0] + item.inertia[1] + item.inertia[2]), rel=1e-6
        )
