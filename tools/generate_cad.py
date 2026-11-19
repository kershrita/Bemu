"""Generate the printed parts, the tolerance coupon, mass properties and the print order.

Usage: python tools/generate_cad.py [--out DIR] [--coupon-only] [--allowance MM] [--sync-masses]

Reads config/robot_params.yaml and config/components.yaml and writes STEP and STL files, a
mass_properties.json and a print_order.md into DIR (default cad/export/). --coupon-only writes just
the tolerance coupon, the first thing to print. --sync-masses rewrites the three link_mass numbers
(and the chassis budget and body thickness) in config/robot_params.yaml from the CAD.
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for package in ("bemu_params", "bemu_cad"):
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / package))

from bemu_cad.body import build_body  # noqa: E402
from bemu_cad.components import ComponentsError, load_components  # noqa: E402
from bemu_cad.export import export_all, export_coupon  # noqa: E402
from bemu_cad.leg import build_leg  # noqa: E402
from bemu_cad.masses import link_inertias, link_masses, sync_robot_params  # noqa: E402
from bemu_params import ParamsError, load_params  # noqa: E402

PARAMS_FILE = REPO_ROOT / "config" / "robot_params.yaml"
DEFAULT_OUT = REPO_ROOT / "cad" / "export"


def main() -> int:
    try:
        return _run()
    except (ParamsError, ComponentsError, ValueError, OSError) as exc:
        print(f"generate_cad: {exc}", file=sys.stderr)
        return 1


def _run() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="output directory")
    parser.add_argument(
        "--coupon-only", action="store_true", help="write only the tolerance coupon"
    )
    parser.add_argument(
        "--allowance",
        type=float,
        default=None,
        help="servo pocket allowance in mm (default: printing.pocket_allowance in components.yaml)",
    )
    parser.add_argument(
        "--sync-masses",
        action="store_true",
        help="write the CAD link masses into config/robot_params.yaml and stop",
    )
    args = parser.parse_args()

    components = load_components()
    if args.sync_masses:
        masses = link_masses(build_leg(load_params(PARAMS_FILE), components), components)
        leg = build_leg(load_params(PARAMS_FILE), components)
        body = build_body(load_params(PARAMS_FILE), components, leg)
        thickness = 2 * body.layout_mm["half_height"]
        sync_robot_params(PARAMS_FILE, masses, thickness, link_inertias(leg, components))
        print(
            f"link masses (g): coxa {masses.coxa * 1000:.1f}, femur {masses.femur * 1000:.1f}, "
            f"tibia {masses.tibia * 1000:.1f}, body thickness {thickness:.2f} mm written to "
            f"{PARAMS_FILE}"
        )
        return 0
    args.out.mkdir(parents=True, exist_ok=True)  # fails early, before any CAD work
    if args.coupon_only:
        written = export_coupon(components, args.out)
    else:
        allowance = None if args.allowance is None else args.allowance * 1e-3
        written = export_all(load_params(PARAMS_FILE), components, args.out, allowance=allowance)
    for name in sorted(written):
        print(f"wrote {args.out / name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
