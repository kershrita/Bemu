"""The CAD visual meshes of the robot, for the MuJoCo viewer and RViz.

`tools/generate_cad.py` writes the STL files and `meshes.yaml` into the package's `meshes/` folder
from the same CAD as the print files. Nothing here needs a CAD library: the manifest says which
link a mesh belongs to, where it sits in that link's frame and what colour it is.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

MM = 1e-3
SOURCE_DIR = Path(__file__).resolve().parents[1] / "meshes"


@dataclass(frozen=True)
class VisualMesh:
    mesh: str
    pos: tuple[float, float, float]  # m, in the link's frame
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class Visuals:
    directory: Path
    links: dict[str, tuple[VisualMesh, ...]]  # body, coxa, femur, tibia
    per_leg: tuple[VisualMesh, ...]  # placed at every leg's mount, in the leg frame

    @property
    def mesh_names(self) -> list[str]:
        names = [m.mesh for items in self.links.values() for m in items]
        names += [m.mesh for m in self.per_leg]
        return sorted(set(names))


def default_mesh_dir() -> Path:
    """The installed meshes folder under ROS, else the one in the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("bemu_description")) / "meshes"
        if installed.is_dir():
            return installed
    except Exception:  # not under ROS, or the package is not installed
        pass
    return SOURCE_DIR


def load_visuals(directory: str | Path | None = None) -> Visuals | None:
    """The visuals in `directory` (default: the package's), or None when there are none."""
    directory = Path(directory) if directory is not None else default_mesh_dir()
    manifest = directory / "meshes.yaml"
    if not manifest.is_file():
        return None
    raw = yaml.safe_load(manifest.read_text(encoding="utf-8"))

    def mesh(entry: dict) -> VisualMesh:
        pos = entry.get("pos_mm", [0.0, 0.0, 0.0])
        return VisualMesh(
            entry["mesh"],
            tuple(float(v) * MM for v in pos),
            tuple(float(v) for v in entry["color"]),
        )

    links = {name: tuple(mesh(e) for e in entries) for name, entries in raw["links"].items()}
    per_leg = tuple(mesh(e) for e in raw.get("per_leg_body", []))
    for name in {m.mesh for items in links.values() for m in items} | {m.mesh for m in per_leg}:
        if not (directory / f"{name}.stl").is_file():
            raise FileNotFoundError(f"{directory}: manifest names {name}.stl, which is missing")
    return Visuals(directory, links, per_leg)


def _rgba(color) -> str:
    return " ".join(f"{v:g}" for v in color)


def assets_xml(visuals: Visuals) -> str:
    """The <asset> block of mesh files (millimetre STL, scaled to metres)."""
    lines = [
        f'<mesh name="{name}" file="{name}.stl" scale="0.001 0.001 0.001" inertia="exact"/>'
        for name in visuals.mesh_names
    ]
    return "<asset>\n    " + "\n    ".join(lines) + "\n  </asset>"


def geoms_xml(visuals: Visuals, link: str) -> str:
    """Visual-only geoms (no contact, no mass) for one link, in group 2 so the viewer shows them."""
    out = []
    for item in visuals.links.get(link, ()):
        x, y, z = item.pos
        out.append(
            f'<geom type="mesh" mesh="{item.mesh}" pos="{x!r} {y!r} {z!r}" '
            f'rgba="{_rgba(item.rgba)}" contype="0" conaffinity="0" group="2" density="0"/>'
        )
    return "\n      ".join(out)


def per_leg_geoms_xml(visuals: Visuals, pos: str, euler: str) -> str:
    """The hip plate and coxa servo of one leg, placed at its mount in the body frame."""
    out = [
        f'<geom type="mesh" mesh="{m.mesh}" pos="{pos}" euler="{euler}" rgba="{_rgba(m.rgba)}" '
        f'contype="0" conaffinity="0" group="2" density="0"/>'
        for m in visuals.per_leg
    ]
    return "\n      ".join(out)
