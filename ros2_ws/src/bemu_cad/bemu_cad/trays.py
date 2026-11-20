"""The battery tray and the Raspberry Pi plate (both printed, both bolted into heat-set inserts).

Frame: the body frame of body.py (x forward, y left, z up, the hip plane at z = 0).

Battery: lies along x on the bottom plate, centred at x = y = 0, between the main posts. The tray is
a base with two low end walls and two side lips around a pocket (the pack plus a clearance), strap
flanges with slots for a hook-and-loop strap or cable ties, and four screw holes over four heat-set
inserts in the bottom plate. Everything about the pack comes from `battery` in components.yaml.

Pi: sits on the top deck, on a plate with four standoffs at the board's hole pattern, bolted to four
insert holes of the 20 mm grid. The board and the pack are returned as boxes so the sweep tests can
check that no leg touches them.
"""

from __future__ import annotations

from build123d import Box, Cylinder, Pos

from bemu_cad.leg import HoleSpec, Part

BATTERY_CLEARANCE = 0.5  # mm of play around the pack, on each side
TRAY_BASE = 3.0
TRAY_WALL = 1.5
END_WALL_HALF_WIDTH = 17.0  # the end walls stop short of the hip posts
END_WALL_HEIGHT = 12.0  # above the base
LIP_HEIGHT = 8.0
FLANGE_Y = 34.0  # strap flange half width
FLANGE_X = (28.0, 56.0)  # |x| range of the flange (slots then screw holes)
SLOT = (12.0, 3.0)  # strap slot size (along x, along y): fits a 10 mm strap
SLOT_GROOVE = 2.0  # mm deep channel under the base joining each slot pair, for the strap
SLOT_X = 37.0
SLOT_Y = 27.0
MOUNT_X = 50.0
MOUNT_Y = 28.0
BATTERY_MOUNTS = [(sx * MOUNT_X, sy * MOUNT_Y) for sx in (-1, 1) for sy in (-1, 1)]

PI_PLATE = (96.0, 66.0, 3.0)  # x, y, thickness
PI_STANDOFF = (6.0, 6.0)  # diameter, height
PI_MOUNTS = [(sx * 10.0, sy * 20.0) for sx in (-1, 1) for sy in (-1, 1)]  # grid insert holes
PI_NOTCH = (16.0, 25.5, 23.0)  # |x| from, |x| to, |y| from: clears the centre posts' screws


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _cyl(x, y, diameter, z0, z1):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(diameter / 2, z1 - z0)


def battery_box(components, z0: float):
    """The pack itself (mm), resting on the tray base; z0 is the top of the bottom plate."""
    length, width, height = components.fields["battery"]["size"].value
    return _box(
        -length / 2, length / 2, -width / 2, width / 2, z0 + TRAY_BASE, z0 + TRAY_BASE + height
    )


def battery_tray(components, z0: float, m3: float) -> Part:
    length, width, _ = components.fields["battery"]["size"].value
    half_l = length / 2 + BATTERY_CLEARANCE
    half_w = width / 2 + BATTERY_CLEARANCE
    outer_l = half_l + TRAY_WALL
    outer_w = half_w + TRAY_WALL
    top = z0 + TRAY_BASE
    solid = _box(-outer_l, outer_l, -outer_w, outer_w, z0, top)
    for sx in (-1, 1):  # end walls, with a notch for the leads
        x0, x1 = sorted((sx * half_l, sx * outer_l))
        wall = _box(x0, x1, -END_WALL_HALF_WIDTH, END_WALL_HALF_WIDTH, z0, top + END_WALL_HEIGHT)
        wall -= _box(x0 - 1, x1 + 1, -10.0, 10.0, top + 3.0, top + END_WALL_HEIGHT + 1)
        solid += wall
    for sy in (-1, 1):  # side lips
        y0, y1 = sorted((sy * half_w, sy * outer_w))
        solid += _box(-half_l, half_l, y0, y1, z0, top + LIP_HEIGHT)
    holes = []
    for sx in (-1, 1):  # flanges: strap slots and screw holes
        for sy in (-1, 1):
            xs = sorted((sx * FLANGE_X[0], sx * FLANGE_X[1]))
            ys = sorted((sy * (outer_w - 1.0), sy * FLANGE_Y))
            solid += _box(xs[0], xs[1], ys[0], ys[1], z0, top)
    for sx in (-1, 1):
        for sy in (-1, 1):
            solid -= _box(
                sx * SLOT_X - SLOT[0] / 2,
                sx * SLOT_X + SLOT[0] / 2,
                sy * SLOT_Y - SLOT[1] / 2,
                sy * SLOT_Y + SLOT[1] / 2,
                z0 - 1,
                top + 1,
            )
        solid -= _box(  # the channel under the base that joins the two slots of one side pair
            sx * SLOT_X - SLOT[0] / 2,
            sx * SLOT_X + SLOT[0] / 2,
            -SLOT_Y - SLOT[1] / 2,
            SLOT_Y + SLOT[1] / 2,
            z0 - 1,
            z0 + SLOT_GROOVE,
        )
    for x, y in BATTERY_MOUNTS:
        solid -= _cyl(x, y, m3, z0 - 1, top + 1)
        holes.append(HoleSpec(x, y, m3))
    f0, f1 = FLANGE_X
    polygon = (
        (-outer_l, -outer_w),
        (-f1, -outer_w),
        (-f1, -FLANGE_Y),
        (-f0, -FLANGE_Y),
        (-f0, -outer_w),
        (f0, -outer_w),
        (f0, -FLANGE_Y),
        (f1, -FLANGE_Y),
        (f1, -outer_w),
        (outer_l, -outer_w),
        (outer_l, outer_w),
        (f1, outer_w),
        (f1, FLANGE_Y),
        (f0, FLANGE_Y),
        (f0, outer_w),
        (-f0, outer_w),
        (-f0, FLANGE_Y),
        (-f1, FLANGE_Y),
        (-f1, outer_w),
        (-outer_l, outer_w),
    )
    return Part(
        "battery_tray",
        solid,
        1,
        "PETG",
        (-outer_l, outer_l, -FLANGE_Y, FLANGE_Y),
        tuple(holes),
        "z",
        "body",
        polygon=polygon,
        family="battery_tray",
    )


def pi_box(components, z_top: float):
    """The Raspberry Pi 5 board with its tallest connectors on the top deck. The standoffs (the hole
    pattern) are at the centre of the plate; the board hangs further past the hole pattern at its
    connector end, which is turned toward +x."""
    pi = components.fields["raspberry_pi_5"]
    length, width = pi["board"].value
    pitch_x = pi["hole_pattern"].value[0]
    offset = (
        length / 2 - pi["hole_inset"].value - pitch_x / 2
    )  # board centre past the pattern centre
    height = pi["height"].value
    base = z_top + PI_PLATE[2] + PI_STANDOFF[1]
    return _box(
        offset - length / 2, offset + length / 2, -width / 2, width / 2, base, base + height
    )


def pi_plate(components, z_top: float, m3: float) -> Part:
    px, py, thick = PI_PLATE
    nx0, nx1, ny0 = PI_NOTCH
    pilot = components.fields["raspberry_pi_5"]["pilot_diameter"].value
    pitch_x, pitch_y = components.fields["raspberry_pi_5"]["hole_pattern"].value
    z1 = z_top + thick
    solid = _box(-px / 2, px / 2, -py / 2, py / 2, z_top, z1)
    for sx in (-1, 1):
        for sy in (-1, 1):
            xs = sorted((sx * nx0, sx * nx1))
            ys = sorted((sy * ny0, sy * (py / 2 + 1)))
            solid -= _box(xs[0], xs[1], ys[0], ys[1], z_top - 1, z1 + 1)
    holes = []
    for x, y in PI_MOUNTS:
        solid -= _cyl(x, y, m3, z_top - 1, z1 + 1)
        holes.append(HoleSpec(x, y, m3))
    diameter, height = PI_STANDOFF
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * pitch_x / 2, sy * pitch_y / 2
            solid += _cyl(x, y, diameter, z1 - 0.01, z1 + height)
            solid -= _cyl(x, y, pilot, z1 + height - 5.0, z1 + height + 1)
    hx, hy = px / 2, py / 2
    polygon = (
        (-hx, -hy), (-nx1, -hy), (-nx1, -ny0), (-nx0, -ny0), (-nx0, -hy),
        (nx0, -hy), (nx0, -ny0), (nx1, -ny0), (nx1, -hy), (hx, -hy),
        (hx, hy), (nx1, hy), (nx1, ny0), (nx0, ny0), (nx0, hy),
        (-nx0, hy), (-nx0, ny0), (-nx1, ny0), (-nx1, hy), (-hx, hy),
    )  # fmt: skip
    return Part(
        "pi_plate",
        solid,
        1,
        "PETG",
        (-px / 2, px / 2, -py / 2, py / 2),
        tuple(holes),
        "z",
        "body",
        polygon=polygon,
        family="pi_plate",
    )
