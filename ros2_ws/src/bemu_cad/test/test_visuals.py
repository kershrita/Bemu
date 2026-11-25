"""The simulator's meshes must be the CAD: same geometry, same place in the model."""

import math
import struct
from pathlib import Path

import mujoco
import numpy as np
import pytest

pytest.importorskip("build123d")

from build123d import Pos  # noqa: E402

from bemu_cad.body import build_body  # noqa: E402
from bemu_cad.components import load_components  # noqa: E402
from bemu_cad.leg import build_leg  # noqa: E402
from bemu_cad.visual import build_visuals  # noqa: E402
from bemu_description.actuators import load_actuator_model  # noqa: E402
from bemu_description.robot_mjcf import build_robot_mjcf  # noqa: E402
from bemu_description.visuals import SOURCE_DIR, load_visuals  # noqa: E402
from bemu_params import load_params  # noqa: E402

REPO = Path(__file__).resolve().parents[4]


def stl_bounds(path: Path):
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:300]:  # ASCII
        pts = [
            [float(v) for v in line.split()[1:4]]
            for line in data.decode().splitlines()
            if line.strip().startswith("vertex")
        ]
        arr = np.array(pts)
    else:
        count = struct.unpack("<I", data[80:84])[0]
        raw = np.frombuffer(
            data[84:], dtype=np.dtype([("v", "<f4", (12,)), ("a", "<u2")]), count=count
        )
        arr = raw["v"][:, 3:].reshape(-1, 3)
    return arr.min(axis=0), arr.max(axis=0)


@pytest.fixture(scope="module")
def world():
    params = load_params(REPO / "config" / "robot_params.yaml")
    components = load_components()
    leg = build_leg(params, components)
    body = build_body(params, components, leg)
    meshes, manifest = build_visuals(leg, body)
    return params, components, leg, body, meshes, manifest


def test_the_committed_meshes_are_up_to_date_with_the_cad(world):
    """Change the CAD without regenerating the meshes and this fails (run tools/generate_cad.py)."""
    *_, meshes, manifest = world
    assert {p.stem for p in SOURCE_DIR.glob("*.stl")} == set(meshes)
    for stem, solid in meshes.items():
        box = solid.bounding_box()
        low, high = stl_bounds(SOURCE_DIR / f"{stem}.stl")
        assert low == pytest.approx([box.min.X, box.min.Y, box.min.Z], abs=0.01), stem
        assert high == pytest.approx([box.max.X, box.max.Y, box.max.Z], abs=0.01), stem
    saved = load_visuals(SOURCE_DIR)
    assert sorted(saved.mesh_names) == sorted(meshes)


@pytest.mark.parametrize("code", ["lf", "rr"])
def test_the_cad_parts_sit_where_the_model_puts_them(world, code):
    """At zero joint angles every mesh in the simulator is where the CAD assembly has it."""
    params, components, leg, body, _, _ = world
    actuators = load_actuator_model(
        REPO / "ros2_ws" / "src" / "bemu_sim" / "config" / "actuators.yaml", params.servo
    )
    model = mujoco.MjModel.from_xml_string(
        build_robot_mjcf(params, actuators, visuals=load_visuals(SOURCE_DIR))
    )
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[3] = 1.0  # the body at the origin, identity orientation
    mujoco.mj_forward(model, data)
    t_hip = body.hip_transforms[code]
    cases = {
        "coxa_bracket": (f"{code}_coxa", leg.parts),
        "femur_plate_horn": (f"{code}_femur", leg.parts),
        "tibia_plate_horn": (f"{code}_tibia", leg.parts),
    }
    for name, (body_name, parts) in cases.items():
        part = next(p for p in parts if p.name == name)
        centre = part.solid.center()
        expected = (t_hip * Pos(centre.X, centre.Y, centre.Z)).position
        body_id = model.body(body_name).id
        mesh_id = model.mesh(name).id
        geoms = [
            g
            for g in range(model.ngeom)
            if model.geom_bodyid[g] == body_id and model.geom_dataid[g] == mesh_id
        ]
        assert len(geoms) == 1, name
        got = data.geom_xpos[geoms[0]] * 1000.0
        assert got == pytest.approx([expected.X, expected.Y, expected.Z], abs=0.05), (code, name)
    # a body plate and a hip plate
    plate = next(p for p in body.parts if p.name == "top_plate_centre")
    centre = plate.solid.center()
    geom = next(
        g for g in range(model.ngeom) if model.geom_dataid[g] == model.mesh("top_plate_centre").id
    )
    assert data.geom_xpos[geom] * 1000.0 == pytest.approx([centre.X, centre.Y, centre.Z], abs=0.05)
    assert math.isfinite(float(data.geom_xpos.sum()))
