"""The printed leg: coxa bracket, femur plates, knee spacers, tibia plates, foot bridge and foot.

Layout (millimetres, leg frame: x radial outward from the hip, y lateral, z up; every joint axis
lies in the z = 0 plane at zero angle, and the femur and knee axes run along y):

* The coxa servo is vertical, horn down, its body inboard (toward -x), above a base plate that
  sits on its horn. The base plate carries a clevis (two walls) that holds the femur servo.
* The femur servo's far end points outward. Its two horns (it is a dual-axis servo) drive the two
  femur plates, which sit outside the clevis walls and carry the knee servo through spacers.
* The knee servo's two horns drive the two tibia plates, outside the femur plates, joined by the
  foot bridge; the foot is a hemisphere with a stem into the bridge, its centre the foot centre.
* Every servo is centred on y = 0, so the foot lies in the leg's radial plane.
* Screw heads sit on the outer faces of walls and plates, so every sliding gap is the running
  clearance plus a head height; the heads are modelled and checked.
* The rear (second) horn's boss has a relief for the servo's fixed hub; horn-side bosses have a
  relief for the stock centre screw's head.

Parts that must not move relative to each other form three groups (fixed, femur, tibia) used by the
checks. Hard stops and cable slots come with the body (version 1.5b).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from build123d import Box, Cylinder, Location, Polygon, Pos, Rot, Solid, Sphere, extrude

from bemu_cad.components import Components
from bemu_cad.servo import M_TO_MM, servo_pocket, servo_solid
from bemu_params import RobotParams

PLATE = 3.0  # mm, plate and wall thickness
COXA_SUPPORT = 3.0  # mm, the body's plate under the coxa servo
RUNNING_CLEARANCE = 0.6  # mm, between parts that rotate against each other
BOSS_DIAMETER = 18.0  # mm, enters the horn window of the next plate
WINDOW_MARGIN = 1.0  # mm, the window is the horn disc plus this
HORN_SCREW_RELIEF = (6.6, 2.0)  # mm diameter and depth for the stock centre screw head
HUB_RELIEF_MARGIN = 1.0  # mm added to the hub diameter
HUB_RELIEF_EXTRA = 0.4  # mm deeper than the hub stands proud of the idler
FOOT_BRIDGE_DEPTH = 14.0  # mm, bridge block above the foot centre
FOOT_STEM = (5.0, 4.0)  # mm diameter and length of the stem into the bridge
BRIDGE_PILOT = (1.6, 8.0)  # mm diameter and depth of the screw pilots in the bridge ends
MIN_EDGE_FACTOR = 1.5  # edge distance of a hole, in hole diameters
FALLBACK_ALLOWANCE = 0.2e-3  # m, until the tolerance coupon sets it


@dataclass(frozen=True)
class HoleSpec:
    a: float  # mm, first in-plane coordinate (x of the leg frame)
    b: float  # mm, second in-plane coordinate (z or y of the leg frame)
    diameter: float  # mm
    face: str = ""  # holes on different faces of one part never conflict


@dataclass(frozen=True)
class Window:
    """A round cut-out through a plate (the horn window), checked against the holes near it."""

    a: float
    b: float
    diameter: float
    face: str = ""


@dataclass(frozen=True)
class Part:
    name: str
    solid: Solid
    count: int
    material: str
    outline: tuple[float, float, float, float]  # mm, in-plane bounds (a0, a1, b0, b1)
    holes: tuple[HoleSpec, ...]
    print_axes: str  # which leg axis points up when printing: "y", "z"
    group: str  # fixed, femur or tibia
    windows: tuple[Window, ...] = ()
    face_outlines: dict[str, tuple[float, float, float, float]] = field(default_factory=dict)
    polygon: tuple[tuple[float, float], ...] = ()  # exact outline when it is not a rectangle
    print_flipped: bool = False  # print with the flat face down: turn the part over
    family: str = ""  # parts printed as copies of one file share a family (default: the name)


@dataclass(frozen=True)
class Hardware:
    """A screw head placed on a part's outer face (checked for clearance, not for strength)."""

    name: str
    group: str
    solid: Solid


@dataclass(frozen=True)
class LegParts:
    parts: tuple[Part, ...]
    servo_solids: dict[str, Solid]  # coxa_servo, femur_servo, knee_servo, placed in the leg frame
    servo_groups: dict[str, str]
    servo_transforms: dict[str, Location]  # servo frame -> leg frame
    hardware: tuple[Hardware, ...]
    axes_mm: dict[str, tuple[float, float, float]]  # coxa, femur, knee, foot positions
    allowance_mm: float
    layout_mm: dict[str, float]  # named planes and distances, for the checks
    screws: tuple[tuple[str, str, int, float], ...]  # (joint, thread, count, length mm)

    def measured_axis_x(self, name: str, solid: Solid) -> float:
        """The output axis x of a placed servo, measured from its bounding box, not the layout."""
        box = solid.bounding_box()
        near = self.layout_mm["servo_near_end"]
        if name == "coxa_servo":  # body inboard: the axis is `near` from the outer end
            return box.max.X - near
        return box.min.X + near


def _box(x0, x1, y0, y1, z0, z1) -> Solid:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _cylinder_y(x: float, z: float, diameter: float, y0: float, y1: float) -> Solid:
    return Pos(x, (y0 + y1) / 2, z) * (Rot(X=90) * Cylinder(diameter / 2, y1 - y0))


def _cylinder_z(x: float, y: float, diameter: float, z0: float, z1: float) -> Solid:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(diameter / 2, z1 - z0)


def _screw_length(stack: float, engagement: float = 3.0) -> float:
    """The shortest standard ISO screw length (mm) that crosses `stack` and engages `engagement`."""
    for length in (4, 5, 6, 8, 10, 12, 14, 16, 20):
        if length >= stack + engagement - 1e-9:
            return float(length)
    return float(math.ceil(stack + engagement))


def build_leg(
    params: RobotParams, components: Components, *, allowance: float | None = None
) -> LegParts:
    if allowance is None:
        printing = components.fields.get("printing", {}).get("pocket_allowance")
        allowance = printing.value * 1e-3 if printing else FALLBACK_ALLOWANCE
    if not (math.isfinite(allowance) and allowance > 0):
        raise ValueError(f"allowance must be a positive length in metres, got {allowance!r}")
    servo = components.servo
    a = allowance * M_TO_MM
    near = servo.output_axis[0] * M_TO_MM
    length = servo.size[0] * M_TO_MM
    far = length - near
    width = servo.size[1] * M_TO_MM
    z0, z1 = (v * M_TO_MM for v in servo.case_z)
    case_z1 = z1
    thickness = z1 - z0  # the case along the output axis, back shelf to lid strip
    z_centre = (z0 + z1) / 2
    horn_tip = servo.horn_z[1] * M_TO_MM - z1  # horn tip beyond the horn-side case face
    idler_face = servo.rear_idler_z[0] * M_TO_MM  # the rear horn's outer face
    hub_tip = servo.rear_hub_z[0] * M_TO_MM
    hub_diameter = servo.rear_hub_diameter * M_TO_MM
    half_w, half_t = width / 2, thickness / 2
    leg = params.leg
    x_f = leg.coxa_length * M_TO_MM
    x_k = x_f + leg.femur_length * M_TO_MM
    x_foot = x_k + leg.tibia_length * M_TO_MM
    window = servo.horn_disc_diameter * M_TO_MM + WINDOW_MARGIN
    m3 = components.fields["screw_m3"]["clearance_hole"].value
    m2 = components.fields["screw_m2"]["clearance_hole"].value
    head_d = components.fields["screw_m2"]["head_diameter"].value
    head_h = components.fields["screw_m2"]["head_height"].value
    gap = RUNNING_CLEARANCE + head_h  # screw heads sit in every sliding gap

    raw = servo_solid(servo)
    pocket = servo_pocket(servo, allowance)  # the volume the servo (plus allowance) occupies
    horn_hole_pattern = [(h.x * M_TO_MM, h.y * M_TO_MM) for h in servo.horn_holes]

    # ---- servos, placed ------------------------------------------------------------------
    z_b = -(half_w + a)  # base plate top: just under the femur servo
    # the coxa servo hangs from the body, horn down; a support plate (COXA_SUPPORT thick) sits on
    # its horn-side shelf and the base plate's boss reaches up through that plate's window
    boss_len = RUNNING_CLEARANCE + COXA_SUPPORT - horn_tip + HORN_SCREW_RELIEF[1]
    if boss_len <= 0:
        raise ValueError("the coxa support plate does not leave room for the base-plate boss")
    z_tip = z_b + boss_len  # the coxa horn tip
    c = z_tip + servo.horn_z[1] * M_TO_MM
    t_coxa = Pos(0, 0, c) * Rot(Y=180)
    t_femur = Pos(x_f, 0, 0) * Rot(X=-90) * Pos(0, 0, -z_centre)
    t_knee = Pos(x_k, 0, 0) * Rot(X=-90) * Pos(0, 0, -z_centre)
    coxa_servo = t_coxa * raw
    femur_servo = t_femur * raw
    knee_servo = t_knee * raw

    def case_holes(side: str, x_axis: float):
        """Bracket holes of one face, as (x, z) in the leg frame for a servo whose far end is +x."""
        return [
            (x_axis + h.x * M_TO_MM, -h.y * M_TO_MM) for h in servo.case_holes if h.side == side
        ]

    hardware: list[Hardware] = []

    def head(name: str, group: str, x: float, z: float, y_face: float, sign: int):
        lo, hi = sorted((y_face, y_face + sign * head_h))
        hardware.append(Hardware(name, group, _cylinder_y(x, z, head_d, lo, hi)))

    # ---- coxa bracket ----------------------------------------------------------------------
    y_in = half_t + a  # clevis wall inner face
    y_out = y_in + PLATE
    wall_x0 = x_f - near - a - PLATE
    wall_x1 = x_f + far + a + PLATE
    base_x0 = -14.0
    z_base1 = z_b
    z_base0 = z_base1 - PLATE
    z_top = half_w + a + PLATE
    bracket = _box(base_x0, wall_x1, -y_out, y_out, z_base0, z_base1)
    bracket += _cylinder_z(0.0, 0.0, BOSS_DIAMETER, z_base1 - 1.0, z_tip)  # boss up to the horn
    d_relief, depth_relief = HORN_SCREW_RELIEF
    bracket -= _cylinder_z(0.0, 0.0, d_relief, z_tip - depth_relief, z_tip + 1.0)
    for sign in (1, -1):
        lo, hi = (y_in, y_out) if sign > 0 else (-y_out, -y_in)
        bracket += _box(wall_x0, wall_x1, lo, hi, z_base0, z_top)
        bracket -= _cylinder_y(x_f, 0.0, window, lo - 1, hi + 1)
    bracket -= t_femur * pocket  # relief for the lid strip, the rear boss, the idler and the hub
    holes_base = [HoleSpec(x, y, m3, "base") for x, y in horn_hole_pattern]
    holes_base.append(HoleSpec(0.0, 0.0, m3, "base"))
    for h in holes_base:
        bracket -= _cylinder_z(h.a, h.b, h.diameter, z_base0 - 1, z_tip + 1)
    horn_wall = case_holes("horn", x_f)
    back_wall = case_holes("back", x_f)
    for xs, (lo, hi) in ((horn_wall, (y_in, y_out)), (back_wall, (-y_out, -y_in))):
        for x, z in xs:
            bracket -= _cylinder_y(x, z, m2, lo - 1, hi + 1)
    wall_holes = [HoleSpec(x, z, m2, "horn") for x, z in horn_wall] + [
        HoleSpec(x, z, m2, "back") for x, z in back_wall
    ]
    for i, (x, z) in enumerate(horn_wall):
        head(f"screw_wall_horn_{i}", "fixed", x, z, y_out, 1)
    for i, (x, z) in enumerate(back_wall):
        head(f"screw_wall_back_{i}", "fixed", x, z, -y_out, -1)

    # ---- femur plates ----------------------------------------------------------------------
    y_p0 = y_out + gap  # inner face of a femur plate, outside the clevis wall and its screw heads
    plate_x0 = x_f - 14.0
    plate_x1 = x_k + 36.5
    plate_z = 14.0
    horn_y = half_t + horn_tip  # horn tip, +y side
    # the rear horn's outer face and the hub tip, in the leg frame (negative y)
    back_y = idler_face - z_centre
    hub_y = hub_tip - z_centre
    relief_depth_back = abs(hub_y - back_y) + HUB_RELIEF_EXTRA

    def horn_boss(plate, x: float, sign: int, inner_y: float, tip_y: float, rear: bool):
        """A boss from a plate's inner face to the horn face, with the right relief at its end."""
        lo, hi = sorted((inner_y, tip_y))
        plate += _cylinder_y(x, 0.0, BOSS_DIAMETER, lo, hi)
        if rear:  # relief for the fixed hub, cut into the boss end
            r_lo, r_hi = sorted((tip_y, tip_y + sign * relief_depth_back))
            plate -= _cylinder_y(x, 0.0, hub_diameter + HUB_RELIEF_MARGIN, r_lo, r_hi)
        else:  # relief for the stock centre screw head
            d, depth = HORN_SCREW_RELIEF
            r_lo, r_hi = sorted((tip_y, tip_y + sign * depth))
            plate -= _cylinder_y(x, 0.0, d, r_lo, r_hi)
        return plate

    def femur_plate(sign: int, side: str):
        lo, hi = (y_p0, y_p0 + PLATE) if sign > 0 else (-(y_p0 + PLATE), -y_p0)
        plate = _box(plate_x0, plate_x1, lo, hi, -plate_z, plate_z)
        rear = side == "back"
        tip = back_y if rear else horn_y
        plate = horn_boss(plate, x_f, sign, sign * y_p0, tip, rear)
        plate -= _cylinder_y(x_k, 0.0, window, lo - 1, hi + 1)  # the tibia boss passes through
        holes = [HoleSpec(x_f + dx, dz, m3) for dx, dz in horn_hole_pattern]
        holes.append(HoleSpec(x_f, 0.0, m3))
        holes += [HoleSpec(x, z, m2) for x, z in case_holes(side, x_k)]
        for h in holes:
            plate -= _cylinder_y(h.a, h.b, h.diameter, lo - 8, hi + 8)
        return plate, holes

    femur_horn, femur_horn_holes = femur_plate(1, "horn")
    femur_back, femur_back_holes = femur_plate(-1, "back")
    for i, (x, z) in enumerate(case_holes("horn", x_k)):
        head(f"screw_knee_horn_{i}", "femur", x, z, y_p0 + PLATE, 1)
    for i, (x, z) in enumerate(case_holes("back", x_k)):
        head(f"screw_knee_back_{i}", "femur", x, z, -(y_p0 + PLATE), -1)

    # ---- knee spacers ----------------------------------------------------------------------
    def knee_spacer(sign: int, side: str):
        face = half_t  # the servo's case face on this side
        lo, hi = (face, y_p0) if sign > 0 else (-y_p0, -face)
        sp = _box(x_k - near, x_k + far + 1.5, lo, hi, -plate_z, plate_z)
        sp -= t_knee * pocket
        sp -= _cylinder_y(x_k, 0.0, window, lo - 1, hi + 1)
        holes = [HoleSpec(x, z, m2) for x, z in case_holes(side, x_k)]
        for h in holes:
            sp -= _cylinder_y(h.a, h.b, h.diameter, lo - 1, hi + 1)
        return sp, holes

    spacer_horn, spacer_horn_holes = knee_spacer(1, "horn")
    spacer_back, spacer_back_holes = knee_spacer(-1, "back")

    # ---- tibia plates, foot bridge, foot ---------------------------------------------------
    y_t0 = y_p0 + PLATE + gap  # inner face of a tibia plate, outside the femur plate and heads
    outline = [
        (x_k - 14.0, -14.0),
        (x_k + 30.0, -14.0),
        (x_foot - 20.0, -4.0),
        (x_foot + 14.0, -4.0),
        (x_foot + 14.0, 14.0),
        (x_k - 14.0, 14.0),
    ]
    bridge_holes = [(x_foot - 8.0, 5.0), (x_foot + 8.0, 5.0)]
    plates = {}
    for sign, side in ((1, "horn"), (-1, "back")):
        lo, hi = (y_t0, y_t0 + PLATE) if sign > 0 else (-(y_t0 + PLATE), -y_t0)
        prism = Pos(0, lo, 0) * (Rot(X=90) * extrude(Polygon(*outline, align=None), amount=-PLATE))
        box = prism.bounding_box()
        plate = Pos(0, lo - box.min.Y, 0) * prism
        rear = side == "back"
        tip = back_y if rear else horn_y
        plate = horn_boss(plate, x_k, sign, sign * y_t0, tip, rear)
        holes = [HoleSpec(x_k + dx, dz, m3) for dx, dz in horn_hole_pattern]
        holes.append(HoleSpec(x_k, 0.0, m3))
        holes += [HoleSpec(x, z, m2) for x, z in bridge_holes]
        for h in holes:
            plate -= _cylinder_y(h.a, h.b, h.diameter, lo - 8, hi + 8)
        plates[side] = (plate, holes)
    for i, (x, z) in enumerate(bridge_holes):
        head(f"screw_bridge_horn_{i}", "tibia", x, z, y_t0 + PLATE, 1)
        head(f"screw_bridge_back_{i}", "tibia", x, z, -(y_t0 + PLATE), -1)

    bridge = _box(x_foot - 12.0, x_foot + 12.0, -y_t0, y_t0, 0.0, FOOT_BRIDGE_DEPTH)
    pilot_d, pilot_depth = BRIDGE_PILOT
    bridge_pilots = []
    for x, z in bridge_holes:
        for sign in (1, -1):
            end = sign * y_t0
            bridge -= _cylinder_y(x, z, pilot_d, *sorted((end, end - sign * pilot_depth)))
        bridge_pilots += [HoleSpec(x, z, pilot_d, "horn"), HoleSpec(x, z, pilot_d, "back")]
    stem_d, stem_len = FOOT_STEM
    bridge -= _cylinder_z(x_foot, 0.0, stem_d + 0.2, -1.0, stem_len + 0.2)  # pocket for the stem
    foot = Pos(x_foot, 0, 0) * Sphere(leg.foot_radius * M_TO_MM)
    foot = foot & _box(x_foot - 20, x_foot + 20, -20, 20, -20, 0.0)  # the lower hemisphere
    foot += _cylinder_z(x_foot, 0.0, stem_d, 0.0, stem_len)

    def part(name, solid, count, outline, holes, print_axes, group, **extra):
        material = extra.pop("material", "PETG")
        return Part(name, solid, count, material, outline, tuple(holes), print_axes, group, **extra)

    knee_outline = (x_k - near, x_k + far + 1.5, -plate_z, plate_z)
    femur_outline = (plate_x0, plate_x1, -plate_z, plate_z)
    knee_window = Window(x_k, 0.0, window)
    polygon = tuple(outline)
    parts = (
        part(
            "coxa_bracket",
            bracket,
            1,
            (wall_x0, wall_x1, z_base0, z_top),
            holes_base + wall_holes,
            "z",
            "fixed",
            windows=(
                Window(x_f, 0.0, window, "horn"),
                Window(x_f, 0.0, window, "back"),
            ),
            face_outlines={
                "base": (base_x0, wall_x1, -y_out, y_out),
                "horn": (wall_x0, wall_x1, z_base0, z_top),
                "back": (wall_x0, wall_x1, z_base0, z_top),
            },
        ),
        part(
            "femur_plate_horn",
            femur_horn,
            1,
            femur_outline,
            femur_horn_holes,
            "y",
            "femur",
            windows=(knee_window,),
        ),
        part(
            "femur_plate_back",
            femur_back,
            1,
            femur_outline,
            femur_back_holes,
            "y",
            "femur",
            windows=(knee_window,),
        ),
        part(
            "knee_spacer_horn",
            spacer_horn,
            1,
            knee_outline,
            spacer_horn_holes,
            "y",
            "femur",
            windows=(knee_window,),
        ),
        part(
            "knee_spacer_back",
            spacer_back,
            1,
            knee_outline,
            spacer_back_holes,
            "y",
            "femur",
            windows=(knee_window,),
        ),
        part(
            "tibia_plate_horn",
            plates["horn"][0],
            1,
            (x_k - 14.0, x_foot + 14.0, -14.0, 14.0),
            plates["horn"][1],
            "y",
            "tibia",
            polygon=polygon,
        ),
        part(
            "tibia_plate_back",
            plates["back"][0],
            1,
            (x_k - 14.0, x_foot + 14.0, -14.0, 14.0),
            plates["back"][1],
            "y",
            "tibia",
            polygon=polygon,
        ),
        part(
            "foot_bridge",
            bridge,
            1,
            (x_foot - 12.0, x_foot + 12.0, 0.0, FOOT_BRIDGE_DEPTH),
            bridge_pilots,
            "z",
            "tibia",
        ),
        part(
            "foot",
            foot,
            1,
            (x_foot - 8.0, x_foot + 8.0, -8.0, 0.0),
            (),
            "z",
            "tibia",
            material="TPU",
            print_flipped=True,
        ),
    )
    # screws: stack = plate + boss (horn screws) or plate + spacer (case screws) + engagement
    boss_horn_f = y_p0 - horn_y
    boss_back_f = y_p0 + back_y  # back_y is negative: boss from -y_p0 to back_y
    boss_horn_t = y_t0 - horn_y
    boss_back_t = y_t0 + back_y
    screws = (
        ("coxa horn", "M3", 4, _screw_length(PLATE)),  # base plate on the horn, no boss
        ("femur horn side", "M3", 4, _screw_length(PLATE + boss_horn_f)),
        ("femur back side", "M3", 4, _screw_length(PLATE + boss_back_f)),
        ("knee horn side", "M3", 4, _screw_length(PLATE + boss_horn_t)),
        ("knee back side", "M3", 4, _screw_length(PLATE + boss_back_t)),
        ("clevis walls", "M2", 8, _screw_length(PLATE)),
        ("knee servo", "M2", 8, _screw_length(PLATE + (y_p0 - half_t))),
        ("foot bridge", "M2", 4, _screw_length(PLATE, engagement=pilot_depth - 2.0)),
    )
    return LegParts(
        parts=parts,
        servo_solids={
            "coxa_servo": coxa_servo,
            "femur_servo": femur_servo,
            "knee_servo": knee_servo,
        },
        servo_groups={"coxa_servo": "fixed", "femur_servo": "fixed", "knee_servo": "femur"},
        servo_transforms={"coxa_servo": t_coxa, "femur_servo": t_femur, "knee_servo": t_knee},
        hardware=tuple(hardware),
        axes_mm={
            "coxa": (0.0, 0.0, 0.0),
            "femur": (x_f, 0.0, 0.0),
            "knee": (x_k, 0.0, 0.0),
            "foot": (x_foot, 0.0, 0.0),
        },
        allowance_mm=a,
        layout_mm={
            "servo_near_end": near,
            "servo_far_end": far,
            "wall_inner_y": y_in,
            "wall_outer_y": y_out,
            "femur_plate_inner_y": y_p0,
            "tibia_plate_inner_y": y_t0,
            "running_clearance": RUNNING_CLEARANCE,
            "screw_head_height": head_h,
            "case_half_thickness": half_t,
            "horn_tip_y": horn_y,
            "rear_idler_y": back_y,
            "hub_tip_y": hub_y,
            "base_plate_top_z": z_b,
            "coxa_horn_tip_z": z_tip,
            "coxa_support_top_z": c - case_z1,
            "coxa_support_bottom_z": c - case_z1 - COXA_SUPPORT,
            "coxa_back_shelf_z": c - z0,
            "coxa_hub_tip_z": c - servo.rear_hub_z[0] * M_TO_MM,
            "coxa_idler_top_z": c - servo.rear_idler_z[0] * M_TO_MM,
        },
        screws=screws,
    )
