"""Render preview images of the robot built from the CAD meshes (no GPU needed).

Usage: python tools/render_preview.py [--out DIR] [--pose stand|sit]

Poses the simulator's model (the same meshes the viewer and RViz show) and draws it with
matplotlib in three views. Needs matplotlib (pip install matplotlib).
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for package in ("bemu_params", "bemu_kinematics", "bemu_description", "bemu_gait", "bemu_sim"):
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / package))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from bemu_description.actuators import load_actuator_model  # noqa: E402
from bemu_description.robot_mjcf import build_robot_mjcf  # noqa: E402
from bemu_description.visuals import load_visuals  # noqa: E402
from bemu_params import load_params  # noqa: E402

PARAMS_FILE = REPO_ROOT / "config" / "robot_params.yaml"
ACTUATORS_FILE = REPO_ROOT / "ros2_ws" / "src" / "bemu_sim" / "config" / "actuators.yaml"
VIEWS = {"iso": (24, -55), "top": (90, -90), "side": (0, -90)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "docs" / "images")
    parser.add_argument("--pose", choices=("stand", "sit"), default="stand")
    args = parser.parse_args()
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError:
        print("render_preview: matplotlib is needed (pip install matplotlib)", file=sys.stderr)
        return 1

    params = load_params(PARAMS_FILE)
    actuators = load_actuator_model(ACTUATORS_FILE, params.servo)
    visuals = load_visuals()
    if visuals is None:
        print("render_preview: no meshes; run tools/generate_cad.py first", file=sys.stderr)
        return 1
    model = mujoco.MjModel.from_xml_string(build_robot_mjcf(params, actuators, visuals=visuals))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(args.pose).id)
    mujoco.mj_forward(model, data)

    polys, colors = [], []
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[g]
        v0, nv = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        f0, nf = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
        verts = model.mesh_vert[v0 : v0 + nv]
        faces = model.mesh_face[f0 : f0 + nf]
        world = verts @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
        tri = world[faces]
        normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        normal /= np.linalg.norm(normal, axis=1, keepdims=True) + 1e-12
        shade = 0.55 + 0.45 * np.clip(normal @ np.array([0.3, -0.5, 0.8]), -1, 1)
        base = model.geom_rgba[g][:3]
        polys += list(tri)
        colors += [tuple(np.clip(base * s, 0, 1)) + (1.0,) for s in shade]
    args.out.mkdir(parents=True, exist_ok=True)
    allv = np.concatenate([np.asarray(p) for p in polys])
    centre, span = (allv.max(0) + allv.min(0)) / 2, (allv.max(0) - allv.min(0)).max() / 2
    for name, (elev, azim) in VIEWS.items():
        fig = plt.figure(figsize=(9, 6.5), dpi=130)
        ax = fig.add_subplot(111, projection="3d")
        ax.add_collection3d(Poly3DCollection(polys, facecolors=colors, linewidths=0))
        ax.set_xlim(centre[0] - span, centre[0] + span)
        ax.set_ylim(centre[1] - span, centre[1] + span)
        ax.set_zlim(centre[2] - span, centre[2] + span)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        path = args.out / f"robot-{args.pose}-{name}.png"
        fig.savefig(path, bbox_inches="tight", facecolor="white")
        plt.close(fig)
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
