"""Generate the printed parts, the tolerance coupon, mass properties and the print order.

Usage: python tools/generate_cad.py [--out DIR] [--coupon-only]

Reads config/robot_params.yaml and config/components.yaml and writes STEP and STL files, a
mass_properties.json and a print_order.md into DIR (default cad/export/). --coupon-only writes just
the tolerance coupon, the first thing to print.
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for package in ("bemu_params", "bemu_cad"):
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / package))

from bemu_cad.components import ComponentsError, load_components  # noqa: E402
from bemu_cad.export import export_all, export_coupon  # noqa: E402
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
    args = parser.parse_args()

    components = load_components()
    args.out.mkdir(parents=True, exist_ok=True)  # fails early, before any CAD work
    if args.coupon_only:
        written = export_coupon(components, args.out)
    else:
        written = export_all(load_params(PARAMS_FILE), components, args.out)
    for name in sorted(written):
        print(f"wrote {args.out / name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
