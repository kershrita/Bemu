import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import pytest

from bemu_description.robot_mjcf import build_robot_mjcf
from bemu_description.urdf import build_robot_urdf
from bemu_description.visuals import SOURCE_DIR, assets_xml, load_visuals
from bemu_params import LEG_CODES

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def visuals():
    loaded = load_visuals(SOURCE_DIR)
    assert loaded is not None, "run python3 tools/generate_cad.py to write the meshes"
    return loaded


def test_the_manifest_lists_every_link_and_every_mesh_file_exists(visuals):
    assert set(visuals.links) == {"body", "coxa", "femur", "tibia"}
    assert {m.mesh for m in visuals.links["femur"]} >= {"femur_plate_horn", "knee_servo"}
    assert {m.mesh for m in visuals.links["tibia"]} >= {"tibia_plate_back", "foot"}
    assert {m.mesh for m in visuals.per_leg} == {"coxa_servo", "hip_plate"}
    for name in visuals.mesh_names:
        assert (SOURCE_DIR / f"{name}.stl").stat().st_size > 100


def test_a_missing_mesh_file_is_reported_by_name(tmp_path, visuals):
    (tmp_path / "meshes.yaml").write_text((SOURCE_DIR / "meshes.yaml").read_text())
    with pytest.raises(FileNotFoundError, match=r"\.stl"):
        load_visuals(tmp_path)


def test_no_manifest_means_no_visuals(tmp_path):
    assert load_visuals(tmp_path) is None


def test_the_model_with_cad_meshes_compiles_and_has_the_same_physics(params, actuators, visuals):
    plain = mujoco.MjModel.from_xml_string(build_robot_mjcf(params, actuators))
    shown = mujoco.MjModel.from_xml_string(build_robot_mjcf(params, actuators, visuals=visuals))
    assert shown.nmesh == len(visuals.mesh_names)
    assert shown.body_subtreemass[1] == pytest.approx(plain.body_subtreemass[1], rel=1e-9)
    assert shown.nbody == plain.nbody and shown.njnt == plain.njnt and shown.nu == plain.nu
    for geom in range(shown.ngeom):
        if shown.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH:  # CAD meshes never collide
            assert shown.geom_contype[geom] == 0 and shown.geom_conaffinity[geom] == 0
            assert shown.geom_group[geom] == 2
            assert shown.geom_mass[geom] == 0.0 if hasattr(shown, "geom_mass") else True
    # the collision shapes stay, hidden in group 3 behind the meshes
    colliding = [
        g for g in range(shown.ngeom) if shown.geom_contype[g] or shown.geom_conaffinity[g]
    ]
    assert len(colliding) == sum(
        1 for g in range(plain.ngeom) if plain.geom_contype[g] or plain.geom_conaffinity[g]
    )
    assert all(
        shown.geom_group[g] == 3
        for g in colliding
        if shown.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE
    )


def test_every_leg_carries_its_cad_parts(params, actuators, visuals):
    model = mujoco.MjModel.from_xml_string(build_robot_mjcf(params, actuators, visuals=visuals))
    for code in LEG_CODES:
        for link, wanted in (("femur", "femur_plate_horn"), ("tibia", "tibia_plate_horn")):
            body = model.body(f"{code}_{link}").id
            names = {
                model.mesh(model.geom_dataid[g]).name
                for g in range(model.ngeom)
                if model.geom_bodyid[g] == body and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
            }
            assert wanted in names, (code, link)


def test_urdf_visuals_point_at_installed_meshes(params, visuals):
    root = ET.fromstring(build_robot_urdf(params, mujoco_model_path="/x.xml", visuals=visuals))
    files = [m.get("filename") for m in root.iter("mesh")]
    assert len(files) > 100
    for filename in files:
        assert filename.startswith("package://bemu_description/meshes/")
        assert (SOURCE_DIR / filename.rsplit("/", 1)[1]).is_file()
    plain = ET.fromstring(build_robot_urdf(params, mujoco_model_path="/x.xml"))
    assert not list(plain.iter("visual"))


def test_the_meshes_are_installed_with_the_package():
    assert 'glob("meshes/*")' in (PACKAGE / "setup.py").read_text()
    assert "<asset>" in assets_xml(load_visuals(SOURCE_DIR))
