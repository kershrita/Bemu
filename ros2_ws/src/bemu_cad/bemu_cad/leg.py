"""The printed leg: coxa bracket, femur plates, knee spacers, tibia plates, foot bridge and foot.

Layout (millimetres, leg frame: x radial outward from the hip, y lateral, z up; every joint axis
lies in the z = 0 plane at zero angle, and the femur and knee axes run along y):

* The coxa servo is vertical, horn down, its body inboard (toward -x), above a base plate that
  sits on its horn. The base plate carries a clevis (two walls) that holds the femur servo.
* The femur servo's far end points outward. Its two horns (it is a dual-axis servo) drive the two
  femur plates, which sit outside the clevis walls and carry the knee servo through 3.8 mm spacers.
* The knee servo's two horns drive the two tibia plates, outside the femur plates, joined by the
  foot bridge; the foot is a hemisphere under the bridge whose centre is the foot centre.
* Every servo is centred on y = 0, so the foot lies in the leg's radial plane.

Parts that must not move relative to each other form three groups (fixed, femur, tibia) used by the
sweep check. Hard stops and cable slots come with the body (version 1.5b).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from build123d import Box, Cylinder, Polygon, Pos, Rot, Solid, Sphere, extrude

from bemu_cad.components import Components
from bemu_cad.servo import M_TO_MM, REAR_HORN_RECESS, servo_solid
from bemu_params import RobotParams

PLATE = 3.0  # mm, plate and wall thickness
GAP = 0.6  # mm, sliding clearance between parts that rotate against each other
BOSS_DIAMETER = 18.0  # mm, enters the horn window of the next plate
WINDOW_MARGIN = 1.0  # mm, the window is the horn disc plus this
FOOT_BRIDGE_DEPTH = 14.0  # mm, bridge block above the foot centre
MIN_EDGE_FACTOR = 1.5  # edge distance of a hole, in hole diameters
DEFAULT_ALLOWANCE = 0.2e-3  # m, until the tolerance coupon sets it


@dataclass(frozen=True)
class HoleSpec:
    a: float  # mm, first in-plane coordinate (x of the leg frame)
    b: float  # mm, second in-plane coordinate (z or y of the leg frame)
    diameter: float  # mm
    face: str = ""  # holes on different faces of one part never conflict


@dataclass(frozen=True)
class Part:
    name: str
    solid: Solid
    count: int
    material: str
    outline: tuple[float, float, float, float]  # mm, in-plane bounds (a0, a1, b0, b1) for the holes
    holes: tuple[HoleSpec, ...]
    print_axes: str  # which leg axis points up when printing: "y", "z"
    group: str  # fixed, femur or tibia


@dataclass(frozen=True)
class LegParts:
    parts: tuple[Part, ...]
    servo_solids: dict[str, Solid]  # coxa_servo, femur_servo, knee_servo, placed in the leg frame
    servo_groups: dict[str, str]
    axes_mm: dict[str, tuple[float, float, float]]  # coxa, femur, knee, foot positions
    allowance_mm: float
    joint_offsets_mm: dict[str, float] = field(default_factory=dict)

    def measured_axis_x(self, name: str, solid: Solid) -> float:
        """The output axis x of a placed servo, measured from its bounding box, not the layout."""
        box = solid.bounding_box()
        near = self.joint_offsets_mm["servo_near_end"]
        far = self.joint_offsets_mm["servo_far_end"]
        if name == "coxa_servo":  # body inboard: the axis is `near` from the outer end
            return box.max.X - near
        return box.min.X + near if far > 0 else box.max.X - near


def _box(x0, x1, y0, y1, z0, z1) -> Solid:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _cylinder_y(x: float, z: float, diameter: float, y0: float, y1: float) -> Solid:
    return Pos(x, (y0 + y1) / 2, z) * (Rot(X=90) * Cylinder(diameter / 2, y1 - y0))


def _cylinder_z(x: float, y: float, diameter: float, z0: float, z1: float) -> Solid:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(diameter / 2, z1 - z0)


def build_leg(
    params: RobotParams, components: Components, *, allowance: float = DEFAULT_ALLOWANCE
) -> LegParts:
    if not (math.isfinite(allowance) and allowance > 0):
        raise ValueError(f"allowance must be a positive length in metres, got {allowance!r}")
    servo = components.servo
    a = allowance * M_TO_MM
    near = servo.output_axis[0] * M_TO_MM
    length = servo.size[0] * M_TO_MM
    far = length - near
    width = servo.size[1] * M_TO_MM
    z0, z1 = (v * M_TO_MM for v in servo.case_z)
    thickness = z1 - z0  # the case along the output axis
    z_centre = (z0 + z1) / 2
    horn_tip = servo.horn_z[1] * M_TO_MM - z1  # horn tip beyond the horn-side case face
    rear_recess = REAR_HORN_RECESS  # the rear horn face sits inside the back face
    half_w, half_t = width / 2, thickness / 2
    leg = params.leg
    x_f = leg.coxa_length * M_TO_MM
    x_k = x_f + leg.femur_length * M_TO_MM
    x_foot = x_k + leg.tibia_length * M_TO_MM
    window = servo.horn_disc_diameter * M_TO_MM + WINDOW_MARGIN
    m3 = components.fields["screw_m3"]["clearance_hole"].value
    m2 = components.fields["screw_m2"]["clearance_hole"].value
    spacer = PLATE + GAP + a  # fills the gap between a servo face and the plate beyond the wall

    raw = servo_solid(servo)
    centred = Pos(0, 0, -z_centre) * raw  # case centred on the output axis plane

    # ---- servos, placed ------------------------------------------------------------------
    horn_hole_pattern = [(7.0, 0.0), (-7.0, 0.0), (0.0, 7.0), (0.0, -7.0)]
    z_tip = -(half_w + a)  # base plate top = coxa horn tip
    c = z_tip + servo.horn_z[1] * M_TO_MM
    coxa_servo = Pos(0, 0, c) * Rot(Y=180) * raw
    femur_servo = Pos(x_f, 0, 0) * Rot(X=-90) * centred
    knee_servo = Pos(x_k, 0, 0) * Rot(X=-90) * centred

    def case_holes(side: str, x_axis: float):
        """Bracket holes of one face, as (x, z) in the leg frame for a servo whose far end is +x."""
        return [
            (x_axis + h.x * M_TO_MM, -h.y * M_TO_MM) for h in servo.case_holes if h.side == side
        ]

    # ---- coxa bracket ----------------------------------------------------------------------
    y_in = half_t + a  # clevis wall inner face
    y_out = y_in + PLATE
    wall_x0 = x_f - near - a - PLATE
    wall_x1 = x_f + far + a + PLATE
    base_x0 = -14.0
    z_base1 = z_tip
    z_base0 = z_base1 - PLATE
    z_top = half_w + a + PLATE
    bracket = _box(base_x0, wall_x1, -y_out, y_out, z_base0, z_base1)
    for sign in (1, -1):
        lo, hi = (y_in, y_out) if sign > 0 else (-y_out, -y_in)
        bracket += _box(wall_x0, wall_x1, lo, hi, z_base0, z_top)
        bracket -= _cylinder_y(x_f, 0.0, window, lo - 1, hi + 1)
    holes_base = [HoleSpec(x, y, m3) for x, y in horn_hole_pattern] + [HoleSpec(0.0, 0.0, m3)]
    for h in holes_base:
        bracket -= _cylinder_z(h.a, h.b, h.diameter, z_base0 - 1, z_base1 + 1)
    horn_wall = case_holes("horn", x_f)
    back_wall = case_holes("back", x_f)
    for xs, (lo, hi) in ((horn_wall, (y_in, y_out)), (back_wall, (-y_out, -y_in))):
        for x, z in xs:
            bracket -= _cylinder_y(x, z, m2, lo - 1, hi + 1)
    wall_holes = [HoleSpec(x, z, m2, "horn") for x, z in horn_wall] + [
        HoleSpec(x, z, m2, "back") for x, z in back_wall
    ]

    # ---- femur plates ----------------------------------------------------------------------
    y_p0 = y_out + GAP  # inner face of a femur plate, outside the clevis wall
    plate_x0 = x_f - 14.0
    plate_x1 = x_k + 36.5
    plate_z = 14.0

    def femur_plate(sign: int, side: str, horn_face_y: float):
        lo, hi = (y_p0, y_p0 + PLATE) if sign > 0 else (-(y_p0 + PLATE), -y_p0)
        plate = _box(plate_x0, plate_x1, lo, hi, -plate_z, plate_z)
        # boss from the plate's inner face to the horn
        boss_lo, boss_hi = sorted((sign * y_p0, horn_face_y))
        plate += _cylinder_y(x_f, 0.0, BOSS_DIAMETER, boss_lo, boss_hi)
        plate -= _cylinder_y(x_k, 0.0, window, lo - 1, hi + 1)  # the tibia boss passes through
        holes = [HoleSpec(x_f + dx, dz, m3) for dx, dz in horn_hole_pattern]
        holes.append(HoleSpec(x_f, 0.0, m3))
        holes += [HoleSpec(x, z, m2) for x, z in case_holes(side, x_k)]
        for h in holes:
            plate -= _cylinder_y(h.a, h.b, h.diameter, min(lo, boss_lo) - 1, max(hi, boss_hi) + 1)
        return plate, holes

    horn_y = half_t + horn_tip  # horn tip, +y side
    back_y = -(half_t - rear_recess)  # rear horn face, -y side
    femur_horn, femur_horn_holes = femur_plate(1, "horn", horn_y)
    femur_back, femur_back_holes = femur_plate(-1, "back", back_y)

    # ---- knee spacers ----------------------------------------------------------------------
    def knee_spacer(sign: int, side: str):
        lo, hi = (half_t, half_t + spacer) if sign > 0 else (-(half_t + spacer), -half_t)
        sp = _box(x_k - near, x_k + far + 1.5, lo, hi, -plate_z, plate_z)
        sp -= _cylinder_y(x_k, 0.0, window, lo - 1, hi + 1)
        holes = [HoleSpec(x, z, m2) for x, z in case_holes(side, x_k)]
        for h in holes:
            sp -= _cylinder_y(h.a, h.b, h.diameter, lo - 1, hi + 1)
        return sp, holes

    spacer_horn, spacer_horn_holes = knee_spacer(1, "horn")
    spacer_back, spacer_back_holes = knee_spacer(-1, "back")

    # ---- tibia plates, foot bridge, foot ---------------------------------------------------
    y_t0 = y_p0 + PLATE + GAP  # inner face of a tibia plate, outside the femur plate
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
    for sign, face_y, side in ((1, horn_y, "horn"), (-1, back_y, "back")):
        lo, hi = (y_t0, y_t0 + PLATE) if sign > 0 else (-(y_t0 + PLATE), -y_t0)
        prism = Pos(0, lo, 0) * (Rot(X=90) * extrude(Polygon(*outline, align=None), amount=-PLATE))
        plate = prism
        # make sure the prism spans exactly [lo, hi] in y
        box = plate.bounding_box()
        plate = Pos(0, lo - box.min.Y, 0) * plate
        boss_lo, boss_hi = sorted((sign * y_t0, face_y))
        plate += _cylinder_y(x_k, 0.0, BOSS_DIAMETER, boss_lo, boss_hi)
        holes = [HoleSpec(x_k + dx, dz, m3) for dx, dz in horn_hole_pattern]
        holes.append(HoleSpec(x_k, 0.0, m3))
        holes += [HoleSpec(x, z, m2) for x, z in bridge_holes]
        for h in holes:
            plate -= _cylinder_y(h.a, h.b, h.diameter, min(lo, boss_lo) - 1, max(hi, boss_hi) + 1)
        plates[side] = (plate, holes)

    bridge = _box(x_foot - 12.0, x_foot + 12.0, -y_t0, y_t0, 0.0, FOOT_BRIDGE_DEPTH)
    foot = Pos(x_foot, 0, 0) * Sphere(leg.foot_radius * M_TO_MM)
    foot = foot & _box(x_foot - 20, x_foot + 20, -20, 20, -20, 0.0)  # the lower hemisphere

    def part(name, solid, count, outline, holes, print_axes, group, material="PETG"):
        return Part(name, solid, count, material, outline, tuple(holes), print_axes, group)

    pl_x0, pl_x1 = plate_x0, plate_x1
    parts = (
        part(
            "coxa_bracket", bracket, 1, (wall_x0, wall_x1, z_base0, z_top), wall_holes, "z", "fixed"
        ),
        part(
            "femur_plate_horn",
            femur_horn,
            1,
            (pl_x0, pl_x1, -plate_z, plate_z),
            femur_horn_holes,
            "y",
            "femur",
        ),
        part(
            "femur_plate_back",
            femur_back,
            1,
            (pl_x0, pl_x1, -plate_z, plate_z),
            femur_back_holes,
            "y",
            "femur",
        ),
        part(
            "knee_spacer_horn",
            spacer_horn,
            1,
            (x_k - near, x_k + far + 1.5, -plate_z, plate_z),
            spacer_horn_holes,
            "y",
            "femur",
        ),
        part(
            "knee_spacer_back",
            spacer_back,
            1,
            (x_k - near, x_k + far + 1.5, -plate_z, plate_z),
            spacer_back_holes,
            "y",
            "femur",
        ),
        part(
            "tibia_plate_horn",
            plates["horn"][0],
            1,
            (x_k - 14.0, x_foot + 14.0, -14.0, 14.0),
            plates["horn"][1],
            "y",
            "tibia",
        ),
        part(
            "tibia_plate_back",
            plates["back"][0],
            1,
            (x_k - 14.0, x_foot + 14.0, -14.0, 14.0),
            plates["back"][1],
            "y",
            "tibia",
        ),
        part(
            "foot_bridge", bridge, 1, (x_foot - 12.0, x_foot + 12.0, -y_t0, y_t0), (), "z", "tibia"
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
        ),
    )
    return LegParts(
        parts=parts,
        servo_solids={
            "coxa_servo": coxa_servo,
            "femur_servo": femur_servo,
            "knee_servo": knee_servo,
        },
        servo_groups={"coxa_servo": "fixed", "femur_servo": "fixed", "knee_servo": "femur"},
        axes_mm={
            "coxa": (0.0, 0.0, 0.0),
            "femur": (x_f, 0.0, 0.0),
            "knee": (x_k, 0.0, 0.0),
            "foot": (x_foot, 0.0, 0.0),
        },
        allowance_mm=a,
        joint_offsets_mm={"servo_near_end": near, "servo_far_end": far},
    )
