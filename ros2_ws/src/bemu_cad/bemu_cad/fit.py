"""Fit checks for the printed leg: interference, screw-hole edges, print bed, joint sweeps."""

from __future__ import annotations

import itertools
import math

from build123d import Pos, Rot

from bemu_cad.leg import MIN_EDGE_FACTOR, LegParts, Part

MIN_LIGAMENT = 1.2  # mm of material between two holes
MIN_WALL = 1.6  # mm: four 0.4 mm perimeters
CONTACT_VOLUME = 1e-3  # mm^3: faces that only touch intersect in numerical dust


def _rotate_about_y(solid, x: float, angle: float):
    """Rotate about the line through (x, *, 0) along -y by `angle` radians (positive raises +x)."""
    return Pos(x, 0, 0) * Rot(Y=-math.degrees(angle)) * Pos(-x, 0, 0) * solid


def _posed(leg: LegParts, femur_angle: float, knee_angle: float) -> dict[str, tuple[str, object]]:
    """Every solid in the leg, moved for the two joint angles: name -> (group, solid)."""
    x_f = leg.axes_mm["femur"][0]
    x_k = leg.axes_mm["knee"][0]
    posed = {}
    items = [(p.name, p.group, p.solid) for p in leg.parts]
    items += [(n, leg.servo_groups[n], s) for n, s in leg.servo_solids.items()]
    items += [(h.name, h.group, h.solid) for h in leg.hardware]
    for name, group, solid in items:
        if group == "tibia":
            solid = _rotate_about_y(solid, x_k, knee_angle)
        if group in ("tibia", "femur"):
            solid = _rotate_about_y(solid, x_f, femur_angle)
        posed[name] = (group, solid)
    return posed


def interference_volumes(leg: LegParts, femur_angle: float, knee_angle: float) -> dict[str, float]:
    """Intersection volume (mm^3) of every pair of solids that can collide, for one pose: parts of
    different groups, any printed part against any servo or screw head, and servo against servo."""
    posed = _posed(leg, femur_angle, knee_angle)
    kind = {p.name: "printed" for p in leg.parts}
    kind |= dict.fromkeys(leg.servo_solids, "servo")
    kind |= {h.name: "hardware" for h in leg.hardware}
    volumes = {}
    for (name_a, (group_a, solid_a)), (name_b, (group_b, solid_b)) in itertools.combinations(
        posed.items(), 2
    ):
        if group_a == group_b and kind[name_a] == kind[name_b] != "servo":
            continue  # printed parts (or screw heads) of one group are fixed to each other;
            # a printed part against a servo or head, and two servos, are checked
        overlap = solid_a & solid_b
        volumes[f"{name_a}|{name_b}"] = float(overlap.volume) if overlap else 0.0
    return volumes


def _segment_distance(
    px: float, py: float, a: tuple[float, float], b: tuple[float, float]
) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t = (
        0.0
        if dx == dy == 0
        else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    )
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _edge_distance(part: Part, hole) -> float:
    if part.polygon:
        points = part.polygon
        return min(
            _segment_distance(hole.a, hole.b, points[i], points[(i + 1) % len(points)])
            for i in range(len(points))
        )
    a0, a1, b0, b1 = part.face_outlines.get(hole.face, part.outline)
    return min(hole.a - a0, a1 - hole.a, hole.b - b0, b1 - hole.b)


def edge_distance_problems(leg: LegParts) -> list[str]:
    """Holes too close to a part's edge, to each other, or to a round window."""
    problems = []
    for part in leg.parts:
        for hole in part.holes:
            edge = _edge_distance(part, hole)
            need = MIN_EDGE_FACTOR * hole.diameter
            if edge < need - 1e-9:
                problems.append(
                    f"{part.name}: hole at ({hole.a:.2f}, {hole.b:.2f}) is {edge:.2f} mm from the "
                    f"edge, needs {need:.2f} mm"
                )
            for window in part.windows:
                if window.face != hole.face:
                    continue
                gap = (
                    math.hypot(hole.a - window.a, hole.b - window.b)
                    - (hole.diameter + window.diameter) / 2
                )
                if gap < MIN_LIGAMENT - 1e-9:
                    problems.append(
                        f"{part.name}: hole at ({hole.a:.2f}, {hole.b:.2f}) leaves {gap:.2f} mm "
                        f"to the window, needs {MIN_LIGAMENT} mm"
                    )
        for window in part.windows:
            a0, a1, b0, b1 = part.face_outlines.get(window.face, part.outline)
            rim = (
                min(window.a - a0, a1 - window.a, window.b - b0, b1 - window.b)
                - window.diameter / 2
            )
            if rim < MIN_WALL - 1e-9:
                problems.append(f"{part.name}: window leaves a {rim:.2f} mm rim, needs {MIN_WALL}")
        for h1, h2 in itertools.combinations(part.holes, 2):
            if h1.face != h2.face:
                continue
            gap = math.hypot(h1.a - h2.a, h1.b - h2.b) - (h1.diameter + h2.diameter) / 2
            if gap < MIN_LIGAMENT - 1e-9:
                problems.append(f"{part.name}: holes {gap:.2f} mm apart, need {MIN_LIGAMENT} mm")
    return problems


def thickness_problems(leg: LegParts) -> list[str]:
    """Plates thinner than four perimeters."""
    problems = []
    for part in leg.parts:
        box = part.solid.bounding_box()
        thin = min(box.size.X, box.size.Y, box.size.Z)
        if part.name != "foot" and thin < MIN_WALL:
            problems.append(f"{part.name}: thinnest dimension {thin:.2f} mm, need {MIN_WALL} mm")
    return problems


def bed_overshoot(part: Part, bed: tuple[float, float, float]) -> list[str]:
    """Messages (empty when it fits) for the part in its print orientation on a bed (x, y, z)."""
    box = part.solid.bounding_box()
    size = {"x": box.size.X, "y": box.size.Y, "z": box.size.Z}
    up = part.print_axes
    footprint = sorted(v for k, v in size.items() if k != up)
    height = size[up]
    problems = []
    for have, allowed, label in zip(footprint, sorted(bed[:2]), ("short", "long"), strict=True):
        if have > allowed + 1e-9:
            problems.append(
                f"{part.name}: {label} side {have:.1f} mm exceeds the bed by "
                f"{have - allowed:.1f} mm"
            )
    if height > bed[2] + 1e-9:
        problems.append(
            f"{part.name}: height {height:.1f} mm exceeds the bed by {height - bed[2]:.1f} mm"
        )
    return problems


def foot_radius_mm(leg: LegParts, femur_angle: float, knee_angle: float) -> float:
    """Horizontal distance (mm) of the foot centre from the coxa axis, with the leg in its plane."""
    x_f, x_k, x_foot = (leg.axes_mm[k][0] for k in ("femur", "knee", "foot"))
    return (
        x_f
        + (x_k - x_f) * math.cos(femur_angle)
        + (x_foot - x_k) * math.cos(femur_angle + knee_angle)
    )


def sweep_collisions(
    leg: LegParts,
    limits: dict[str, tuple[float, float]],
    *,
    steps: int = 3,
    min_foot_radius: float = 60.0,
) -> list[str]:
    """Collisions over a grid of femur and knee angles between the joint limits (radians).

    Poses with the foot closer than `min_foot_radius` mm to the hip axis (folded under the body)
    are skipped: the gaits never go there (their closest foot is about 85 mm out), and the joint
    limits alone would allow it. Pass a large negative number to see those collisions too.
    """
    femur = _grid(limits["femur"], steps)
    knee = _grid(limits["tibia"], steps)
    found = []
    for f, k in itertools.product(femur, knee):
        if foot_radius_mm(leg, f, k) < min_foot_radius:
            continue
        for pair, volume in interference_volumes(leg, f, k).items():
            if volume > CONTACT_VOLUME:
                found.append(
                    f"{pair}: {volume:.1f} mm^3 at femur {math.degrees(f):.0f} deg, "
                    f"knee {math.degrees(k):.0f} deg"
                )
    return found


def _grid(limits: tuple[float, float], steps: int) -> list[float]:
    lo, hi = limits
    if steps < 2:
        return [lo]
    return [lo + (hi - lo) * i / (steps - 1) for i in range(steps)]


from bemu_cad.neighbours import FREE_TOWARD_SUM, NEIGHBOURS  # noqa: E402,F401


def _box(solid) -> tuple:
    b = solid.bounding_box()
    return (b.min.X, b.max.X, b.min.Y, b.max.Y, b.min.Z, b.max.Z)


def _touch(a: tuple, b: tuple) -> bool:
    return (
        a[0] <= b[1]
        and b[0] <= a[1]
        and a[2] <= b[3]
        and b[2] <= a[3]
        and a[4] <= b[5]
        and b[4] <= a[5]
    )


def _moved(cache: dict, posed: dict, hip_transforms: dict, code: str, angle: float) -> dict:
    """Every solid of one leg swung by `angle` degrees about its coxa axis and placed on the body,
    with its bounding box; kept in `cache` per (leg, angle)."""
    key = (code, angle)
    if key not in cache:
        t = hip_transforms[code]
        swing = Rot(Z=angle)
        items = {
            name: t * (swing * solid if name != "coxa_servo" else solid)
            for name, (_, solid) in posed.items()
        }
        cache[key] = {name: (solid, _box(solid)) for name, solid in items.items()}
    return cache[key]


def neighbour_collisions(
    leg: LegParts,
    hip_transforms: dict,
    toward_pairs,
    poses: list[tuple[float, float]],
    pairs: tuple = NEIGHBOURS,
) -> list[str]:
    """Part-to-part intersections between two neighbouring legs for every (a, b) in `toward_pairs`
    (degrees each leg swings toward the other; negative swings away) and every femur and knee pose
    (both legs in the same pose: the worst case is two legs reaching for the same spot). The coxa
    servo is part of the body, so it stays put. Parts whose boxes do not touch are skipped."""
    found = []
    for femur, knee in poses:
        posed = _posed(leg, femur, knee)
        cache: dict = {}

        for a, b, sign in pairs:
            for toward_a, toward_b in toward_pairs:
                set_a = _moved(cache, posed, hip_transforms, a, sign * toward_a)
                set_b = _moved(cache, posed, hip_transforms, b, -sign * toward_b)
                for name_a, (solid_a, box_a) in set_a.items():
                    for name_b, (solid_b, box_b) in set_b.items():
                        if not _touch(box_a, box_b):
                            continue
                        overlap = solid_a & solid_b
                        volume = float(overlap.volume) if overlap else 0.0
                        if volume > CONTACT_VOLUME:
                            found.append(
                                f"{a}.{name_a}|{b}.{name_b}: {volume:.1f} mm^3 with the legs "
                                f"{toward_a:.0f} and {toward_b:.0f} deg toward each other, femur "
                                f"{math.degrees(femur):.0f}, knee {math.degrees(knee):.0f} deg"
                            )
    return found
