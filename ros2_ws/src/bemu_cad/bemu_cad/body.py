"""The printed body: a top and a bottom plate (each split in three to fit the bed), a support plate
under each coxa servo, posts, splice plates and a 20 mm M3 insert grid on the top plate.

Body frame (millimetres): x forward, y left, z up, the hip plane at z = 0. The legs are mounted at
the `mounts` of robot_params.yaml: a leg frame maps to the body by Pos(x, y, 0) * Rot(Z=yaw). Each
coxa servo hangs horn-down: its back shelf seats against the top plate (idler and hub in a recess),
its horn-side shelf against a small support plate that stands on posts from the bottom plate, so
the servo is held at both faces against the tilt moment of a loaded leg.

Battery tray and Raspberry Pi plate (trays.py): the pack lies along x on the bottom plate between
the posts, the Pi sits on the top deck. Their dimensions are unverified listings until the parts are
bought and measured (components.yaml); the regulator and wiring are a placeholder mass.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from build123d import Box, Cylinder, Polygon, Pos, Rot, Solid, extrude

from bemu_cad.components import Components
from bemu_cad.export import part_mass_properties
from bemu_cad.leg import PLATE, Hardware, HoleSpec, LegParts, Part, Window
from bemu_cad.servo import M_TO_MM, servo_pocket
from bemu_cad.trays import (
    BATTERY_MOUNTS,
    PI_MOUNTS,
    battery_box,
    battery_tray,
    pi_box,
    pi_plate,
)
from bemu_params import LEG_CODES, RobotParams

TOP_THICKNESS = 4.0  # mm: the idler recess leaves 1.75 mm of floor; the hub goes through a hole
BOTTOM_THICKNESS = 4.0
JOINT_X = (-40.0, 40.0)  # where the plates are split
SPLICE = {"length": 30.0, "width": 12.0, "y": (-50.0, 50.0), "pitch": 8.0}
POST_DIAMETER = 8.0
POST_PILOT = (2.5, 10.0)  # mm diameter and depth of the screw pilots in the post ends
# The posts stay out of the 50 mm wide battery channel along x (|y| < 25) and out of the hip plates.
MAIN_POSTS = [(-88.0, -15.0), (-88.0, 15.0), (88.0, -15.0), (88.0, 15.0)] + [
    (-21.0, -29.0),
    (-21.0, 29.0),
    (21.0, -29.0),
    (21.0, 29.0),
]
HIP_POSTS_LEG = [(-42.0, -8.0), (-42.0, 8.0)]  # in each leg frame
HIP_PLATE = (-48.0, 14.0, -14.0, 14.0)  # x0, x1, y0, y1 in the leg frame
FOOTPRINT_PAD = 5.0  # mm beyond the coxa servo's outline
GRID_PITCH = 20.0
GRID_X = tuple(-90.0 + 20.0 * i for i in range(10))
GRID_Y = (-60.0, -40.0, -20.0, 0.0, 20.0, 40.0, 60.0)
INSERT_HOLE = 4.1  # mm, from the unverified insert entry; the coupon decides
PI_KG = 0.075  # placeholder until the board is weighed
REGULATOR_WIRING_KG = 0.120  # placeholder
FASTENERS_KG = 0.030  # body screws and inserts: an estimate
DECAGON = [
    (120.0, -40.0),
    (120.0, 40.0),
    (100.0, 60.0),
    (0.0, 80.0),
    (-100.0, 60.0),
    (-120.0, 40.0),
    (-120.0, -40.0),
    (-100.0, -60.0),
    (0.0, -80.0),
    (100.0, -60.0),
]


@dataclass(frozen=True)
class BodyParts:
    parts: tuple[Part, ...]
    hip_transforms: dict  # leg code -> body <- leg frame
    layout_mm: dict
    printed_mass_kg: float
    electronics_mass_kg: float
    coxa_servo_kg: float = 0.055
    hardware: tuple = ()  # screw heads that sit near moving parts, for the sweep checks
    screws: tuple = ()  # (where, thread, count, length mm, nut)

    @property
    def mass_kg(self) -> float:
        """Printed parts, the six coxa servos and the electronics placeholders."""
        return self.printed_mass_kg + 6 * self.coxa_servo_kg + self.electronics_mass_kg


def _prism(points, z0: float, z1: float) -> Solid:
    return Pos(0, 0, z0) * extrude(Polygon(*points, align=None), amount=z1 - z0)


def _box(x0, x1, y0, y1, z0, z1) -> Solid:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _cyl(x: float, y: float, diameter: float, z0: float, z1: float) -> Solid:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(diameter / 2, z1 - z0)


def build_body(params: RobotParams, components: Components, leg: LegParts) -> BodyParts:
    servo = components.servo
    layout = leg.layout_mm
    allowance = leg.allowance_mm
    m2 = components.fields["screw_m2"]["clearance_hole"].value
    m3 = components.fields["screw_m3"]["clearance_hole"].value
    window_d = servo.horn_disc_diameter * M_TO_MM + 1.0
    z_seat = layout["coxa_back_shelf_z"]  # top plate underside
    z_top = (z_seat, z_seat + TOP_THICKNESS)
    z_bot = (-z_seat - BOTTOM_THICKNESS, -z_seat)  # symmetric about the hip plane
    z_hip = (layout["coxa_support_bottom_z"], layout["coxa_support_top_z"])
    t_servo = leg.servo_transforms["coxa_servo"]
    pocket = servo_pocket(servo, allowance * 1e-3)

    hip_t = {}
    for code in LEG_CODES:
        mount = params.mounts[code]
        hip_t[code] = Pos(mount.x * M_TO_MM, mount.y * M_TO_MM, 0) * Rot(Z=math.degrees(mount.yaw))

    # ---- plate outline: the decagon plus a padded coxa-servo footprint at every hip ----------
    near = layout["servo_near_end"]
    far = layout["servo_far_end"]
    half_w = servo.size[1] * M_TO_MM / 2
    foot = [
        (-far - FOOTPRINT_PAD, -half_w - FOOTPRINT_PAD),
        (near + FOOTPRINT_PAD, -half_w - FOOTPRINT_PAD),
        (near + FOOTPRINT_PAD, half_w + FOOTPRINT_PAD),
        (-far - FOOTPRINT_PAD, half_w + FOOTPRINT_PAD),
    ]

    def plate(z0: float, z1: float) -> Solid:
        solid = _prism(DECAGON, z0, z1)
        for t in hip_t.values():
            pts = [(t * Pos(x, y, 0)).position for x, y in foot]
            solid += _prism([(p.X, p.Y) for p in pts], z0, z1)
        return solid

    top = plate(*z_top)
    bottom = plate(*z_bot)

    # ---- top plate: coxa servo seats, post and splice holes, the insert grid -------------------
    top_holes: list[HoleSpec] = []
    bottom_holes: list[HoleSpec] = []
    seat_holes = []
    hub_hole = servo.rear_hub_diameter * M_TO_MM + 2 * allowance + 1.0
    for t in hip_t.values():
        top -= t * (t_servo * pocket)  # the shelf, strip, boss, idler and hub relief
        hub = (t * Pos(0, 0, 0)).position
        top -= _cyl(hub.X, hub.Y, hub_hole, z_top[0] - 1, z_top[1] + 1)  # the hub passes through
        for h in servo.case_holes:
            if h.side == "back":
                p = (t * t_servo * Pos(h.x * M_TO_MM, h.y * M_TO_MM, 0)).position
                seat_holes.append((p.X, p.Y))
    for x, y in seat_holes:
        top -= _cyl(x, y, m2, z_top[0] - 1, z_top[1] + 1)
        top_holes.append(HoleSpec(x, y, m2))

    def drill(solid, holes, x, y, d, z0, z1):
        holes.append(HoleSpec(x, y, d))
        return solid - _cyl(x, y, d, z0 - 1, z1 + 1)

    for x, y in MAIN_POSTS:
        top = drill(top, top_holes, x, y, m3, *z_top)
        bottom = drill(bottom, bottom_holes, x, y, m3, *z_bot)
    for x, y in BATTERY_MOUNTS:  # heat-set inserts for the battery tray screws
        bottom = drill(bottom, bottom_holes, x, y, INSERT_HOLE, *z_bot)
    hip_post_xy = {}
    for code, t in hip_t.items():
        for x, y in HIP_POSTS_LEG:
            p = (t * Pos(x, y, 0)).position
            hip_post_xy[(code, x, y)] = (p.X, p.Y)
            bottom = drill(bottom, bottom_holes, p.X, p.Y, m3, *z_bot)
    joint_holes = []
    for jx in JOINT_X:
        for cy in SPLICE["y"]:
            for dx in (-SPLICE["pitch"], SPLICE["pitch"]):
                joint_holes.append((jx + dx, cy))
    for x, y in joint_holes:
        top = drill(top, top_holes, x, y, m3, *z_top)
        bottom = drill(bottom, bottom_holes, x, y, m3, *z_bot)

    taken = [(h.a, h.b) for h in top_holes]
    footprints = []
    for t in hip_t.values():
        pts = [(t * Pos(x, y, 0)).position for x, y in foot]
        footprints.append([(p.X, p.Y) for p in pts])

    def in_footprint(x: float, y: float, margin: float = 6.0) -> bool:
        for t in hip_t.values():
            local = (t.inverse() * Pos(x, y, 0)).position
            if (
                -far - FOOTPRINT_PAD - margin <= local.X <= near + FOOTPRINT_PAD + margin
                and abs(local.Y) <= half_w + FOOTPRINT_PAD + margin
            ):
                return True
        return False

    for gx in GRID_X:
        for gy in GRID_Y:
            if in_footprint(gx, gy) or any(math.hypot(gx - a, gy - b) < 9.0 for a, b in taken):
                continue
            top = drill(top, top_holes, gx, gy, INSERT_HOLE, *z_top)
    del footprints
    for x, y in PI_MOUNTS:  # the Pi plate's screws go into these grid inserts
        if not any(
            abs(h.a - x) < 1e-6 and abs(h.b - y) < 1e-6 and h.diameter == INSERT_HOLE
            for h in top_holes
        ):
            top = drill(top, top_holes, x, y, INSERT_HOLE, *z_top)

    def split(solid, holes, name, z, prefix):
        pieces = []
        bounds = [
            (-300.0, JOINT_X[0], "rear"),
            (JOINT_X[0], JOINT_X[1], "centre"),
            (JOINT_X[1], 300.0, "front"),
        ]
        for x0, x1, label in bounds:
            piece = solid & _box(x0, x1, -300, 300, z[0] - 1, z[1] + 1)
            box = piece.bounding_box()
            mine = tuple(h for h in holes if x0 <= h.a < x1 or (label == "front" and h.a >= x1))
            pieces.append(
                Part(
                    f"{prefix}_{label}",
                    piece,
                    1,
                    "PETG",
                    (box.min.X, box.max.X, box.min.Y, box.max.Y),
                    mine,
                    "z",
                    "body",
                    family=f"{prefix}_{label}",
                )
            )
        return pieces

    parts: list[Part] = []
    hardware: list[Hardware] = []
    head_d = components.fields["screw_m2"]["head_diameter"].value
    head_h = components.fields["screw_m2"]["head_height"].value
    parts += split(top, top_holes, "top", z_top, "top_plate")
    parts += split(bottom, bottom_holes, "bottom", z_bot, "bottom_plate")

    # ---- support plates under the coxa servos --------------------------------------------------
    hx0, hx1, hy0, hy1 = HIP_PLATE
    horn_holes_leg = []
    for h in servo.case_holes:
        if h.side == "horn":
            p = (t_servo * Pos(h.x * M_TO_MM, h.y * M_TO_MM, 0)).position
            horn_holes_leg.append((p.X, p.Y))
    for code, t in hip_t.items():
        plate_leg = _box(hx0, hx1, hy0, hy1, *z_hip)
        plate_leg -= t_servo * pocket
        plate_leg -= _cyl(0.0, 0.0, window_d, z_hip[0] - 1, z_hip[1] + 1)
        holes = []
        for x, y in horn_holes_leg:
            plate_leg -= _cyl(x, y, m2, z_hip[0] - 1, z_hip[1] + 1)
            holes.append(HoleSpec(x, y, m2))
        for x, y in HIP_POSTS_LEG:
            plate_leg -= _cyl(x, y, m3, z_hip[0] - 1, z_hip[1] + 1)
            holes.append(HoleSpec(x, y, m3))
        for k, (x, y) in enumerate(horn_holes_leg):  # M2 heads hang under the plate
            hardware.append(
                Hardware(
                    f"screw_hip_{code}_{k}",
                    "body",
                    t * _cyl(x, y, head_d, z_hip[0] - head_h, z_hip[0]),
                )
            )
        parts.append(
            Part(
                f"hip_plate_{code}",
                t * plate_leg,
                1,
                "PETG",
                (hx0, hx1, hy0, hy1),
                tuple(holes),
                "z",
                "body",
                windows=(Window(0.0, 0.0, window_d),),
                family="hip_plate",
            )
        )

    # ---- posts ---------------------------------------------------------------------------------
    pilot_d, pilot_depth = POST_PILOT

    def post(name, x, y, z0, z1, family):
        solid = _cyl(x, y, POST_DIAMETER, z0, z1)
        solid -= _cyl(x, y, pilot_d, z0 - 1, z0 + pilot_depth)
        solid -= _cyl(x, y, pilot_d, z1 - pilot_depth, z1 + 1)
        return Part(
            name, solid, 1, "PETG", (x - 4, x + 4, y - 4, y + 4), (), "z", "body", family=family
        )

    for i, (x, y) in enumerate(MAIN_POSTS):
        parts.append(post(f"post_main_{i}", x, y, z_bot[1], z_top[0], "post_main"))
    for (code, _lx, ly), (x, y) in hip_post_xy.items():
        side = "a" if ly < 0 else "b"
        parts.append(post(f"post_hip_{code}_{side}", x, y, z_bot[1], z_hip[0], "post_hip"))

    # ---- splice plates -------------------------------------------------------------------------
    length, width, pitch = SPLICE["length"], SPLICE["width"], SPLICE["pitch"]
    for level, (zl, zh) in (
        ("top", (z_top[0] - PLATE, z_top[0])),
        ("bottom", (z_bot[1], z_bot[1] + PLATE)),
    ):
        index = 0
        for jx in JOINT_X:
            for cy in SPLICE["y"]:
                solid = _box(
                    jx - length / 2, jx + length / 2, cy - width / 2, cy + width / 2, zl, zh
                )
                holes = []
                for dx in (-pitch, pitch):
                    solid -= _cyl(jx + dx, cy, m3, zl - 1, zh + 1)
                    holes.append(HoleSpec(jx + dx, cy, m3))
                parts.append(
                    Part(
                        f"splice_{level}_{index}",
                        solid,
                        1,
                        "PETG",
                        (jx - length / 2, jx + length / 2, cy - width / 2, cy + width / 2),
                        tuple(holes),
                        "z",
                        "body",
                        family=f"splice_{level}",
                    )
                )
                index += 1

    # ---- battery tray and Pi plate ----------------------------------------------------------
    parts.append(battery_tray(components, z_bot[1], m3))
    parts.append(pi_plate(components, z_top[1], m3))
    hardware.append(Hardware("battery_pack", "body", battery_box(components, z_bot[1])))
    hardware.append(Hardware("pi_board", "body", pi_box(components, z_top[1])))

    printed = sum(part_mass_properties(p)["mass_kg"] for p in parts)
    battery_kg = components.fields["battery"]["mass"].value / 1000.0
    seat = len(seat_holes)
    posts = len(MAIN_POSTS) + 2 * 6
    grid = sum(
        1
        for p in parts
        if p.family.startswith("top_plate")
        for h in p.holes
        if h.diameter == INSERT_HOLE
    )
    screws = (
        ("top plate to coxa servo (M2)", "M2", seat, 8.0, False),
        ("hip plate to coxa servo (M2)", "M2", 4 * 6, 6.0, False),
        ("post ends into the plates and hip plates (M3)", "M3", 2 * posts, 10.0, False),
        ("splice plates (M3, with nuts)", "M3", 4 * 2 * 4, 12.0, True),
        ("insert grid: heat-set inserts M3 (not screws)", "insert", grid, 0.0, False),
        (
            "battery tray to the bottom plate: heat-set inserts M3 (not screws)",
            "insert",
            4,
            0,
            False,
        ),
        ("battery tray to the bottom plate (M3)", "M3", 4, 8.0, False),
        ("Pi plate to the top deck (M3, into the grid inserts)", "M3", 4, 8.0, False),
        ("Raspberry Pi 5 to the plate (M2.5, self-tapping)", "M2.5", 4, 8.0, False),
    )
    return BodyParts(
        parts=tuple(parts),
        hip_transforms=hip_t,
        layout_mm={
            "top_plate_z": z_top,
            "bottom_plate_z": z_bot,
            "hip_plate_z": z_hip,
            "joint_x": JOINT_X,
            "half_height": z_top[1],
        },
        printed_mass_kg=printed,
        electronics_mass_kg=PI_KG + battery_kg + REGULATOR_WIRING_KG + FASTENERS_KG,
        coxa_servo_kg=servo.mass,
        hardware=tuple(hardware),
        screws=screws,
    )
