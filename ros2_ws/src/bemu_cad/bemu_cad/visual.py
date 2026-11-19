"""Visual meshes for the simulator and RViz, from the same CAD as the print files.

`build_visuals` returns the meshes (one solid per file, in millimetres) and a manifest that says
which link of the model each mesh belongs to, where it sits in that link's frame, and its colour.
The model generators in bemu_description read the manifest and the STL files; they need no CAD
library.

Frames: leg meshes are in the leg frame at zero joint angles, so a mesh on the femur link is shifted
by minus the femur joint's x, and one on the tibia link by minus the knee's x. Body meshes are in
the body frame. The hip plates and coxa servos are in the leg frame and are placed at each mount.
"""

from __future__ import annotations

from bemu_cad.body import BodyParts
from bemu_cad.export_body import canonical_solid
from bemu_cad.leg import LegParts

COLORS = {
    "printed": [0.31, 0.56, 0.82, 1.0],
    "plate": [0.78, 0.80, 0.84, 1.0],
    "servo": [0.12, 0.12, 0.13, 1.0],
    "foot": [0.95, 0.62, 0.12, 1.0],
}
LINK_OF_GROUP = {"fixed": "coxa", "femur": "femur", "tibia": "tibia"}
SERVO_LINK = {"femur_servo": "coxa", "knee_servo": "femur"}


def build_visuals(leg: LegParts, body: BodyParts) -> tuple[dict, dict]:
    """(meshes, manifest): meshes maps a file stem to a solid in mm; the manifest is plain data."""
    meshes: dict = {}
    links: dict[str, list] = {"body": [], "coxa": [], "femur": [], "tibia": []}
    per_leg: list = []
    shift = {
        "coxa": -leg.axes_mm["coxa"][0],
        "femur": -leg.axes_mm["femur"][0],
        "tibia": -leg.axes_mm["knee"][0],
    }

    def add(link: str, stem: str, solid, color, pos=(0.0, 0.0, 0.0)):
        meshes[stem] = solid
        links[link].append({"mesh": stem, "pos_mm": [float(v) for v in pos], "color": color})

    for part in leg.parts:
        link = LINK_OF_GROUP[part.group]
        color = COLORS["foot"] if part.name == "foot" else COLORS["printed"]
        add(link, part.name, part.solid, color, (shift[link], 0.0, 0.0))
    for name, link in SERVO_LINK.items():
        add(link, name, leg.servo_solids[name], COLORS["servo"], (shift[link], 0.0, 0.0))

    meshes["coxa_servo"] = leg.servo_solids["coxa_servo"]
    per_leg.append({"mesh": "coxa_servo", "color": COLORS["servo"]})
    done_family = set()
    for part in body.parts:
        family = part.family or part.name
        if family == "hip_plate":
            if family not in done_family:
                meshes["hip_plate"] = canonical_solid(body, part)
                per_leg.append({"mesh": "hip_plate", "color": COLORS["printed"]})
                done_family.add(family)
        elif family.startswith("post"):
            if family not in done_family:
                meshes[family] = canonical_solid(body, part)
                done_family.add(family)
            box = part.solid.bounding_box()
            pos = ((box.min.X + box.max.X) / 2, (box.min.Y + box.max.Y) / 2, box.min.Z)
            links["body"].append(
                {"mesh": family, "pos_mm": [float(v) for v in pos], "color": COLORS["printed"]}
            )
        else:
            add("body", part.name, part.solid, COLORS["plate"])
    manifest = {"links": links, "per_leg_body": per_leg, "unit": "mm"}
    return meshes, manifest
