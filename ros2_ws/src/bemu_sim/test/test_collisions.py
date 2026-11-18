"""Legs collide with other legs and with the body; a leg's own links never collide."""

import math

import mujoco
import pytest

from bemu_description.names import joint_names
from bemu_params import LEG_CODES
from bemu_sim.robot import RobotSim, play


@pytest.fixture
def sim(robot_mjcf):
    return RobotSim(robot_mjcf)


def body_of(model, geom):
    return model.body(model.geom_bodyid[geom]).name


def contact_bodies(sim):
    """The pairs of bodies in contact, other than with the floor (floor is world: body 0)."""
    model, data = sim.model, sim.data
    pairs = set()
    for c in data.contact[: data.ncon]:
        a, b = body_of(model, c.geom1), body_of(model, c.geom2)
        if "world" not in (a, b):
            pairs.add(frozenset((a, b)))
    return pairs


def set_joints(sim, **angles):
    for name, value in angles.items():
        sim.data.qpos[sim.model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(sim.model, sim.data)


def excluded(model, name1, name2):
    a, b = sorted((model.body(name1).id, model.body(name2).id))
    return (a << 16) + b in {int(x) for x in model.exclude_signature}


def test_neighbouring_legs_collide_when_they_swing_into_each_other(sim):
    sim.reset("stand")
    assert contact_bodies(sim) == set()
    set_joints(sim, lf_coxa_joint=math.radians(45), lm_coxa_joint=-math.radians(45))
    touching = contact_bodies(sim)
    assert touching, "the two legs should touch"
    for pair in touching:
        legs = {name.split("_")[0] for name in pair}
        assert legs == {"lf", "lm"}


@pytest.mark.parametrize("leg", LEG_CODES)
def test_a_legs_own_links_and_its_hip_are_excluded_but_other_pairs_are_not(sim, leg):
    model = sim.model
    coxa, femur, tibia = f"{leg}_coxa", f"{leg}_femur", f"{leg}_tibia"
    for a, b in (("body", coxa), (coxa, femur), (femur, tibia), (coxa, tibia)):
        assert excluded(model, a, b), (a, b)
    # these can touch: they are what the collisions are for
    assert not excluded(model, "body", femur)
    assert not excluded(model, "body", tibia)
    other = "rm" if leg != "rm" else "lm"
    for mine in (coxa, femur, tibia):
        for theirs in (f"{other}_coxa", f"{other}_femur", f"{other}_tibia"):
            assert not excluded(model, mine, theirs)


@pytest.mark.parametrize("keyframe", ["sit", "stand"])
def test_only_the_floor_is_touched_at_sit_and_stand(sim, params, keyframe):
    sim.reset(keyframe)
    play(sim, [sim.joint_angles()] * 25)
    assert contact_bodies(sim) == set()


@pytest.mark.parametrize("leg", LEG_CODES)
def test_a_fully_folded_leg_does_not_hit_itself(sim, params, leg):
    sim.reset("stand")
    coxa, femur, tibia = joint_names(leg)
    set_joints(
        sim,
        **{
            femur: params.leg.femur_limits[1],
            tibia: params.leg.tibia_limits[0],
        },
    )
    assert contact_bodies(sim) == set()
