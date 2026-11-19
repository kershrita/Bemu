"""Run the version 1 torque report and write it to docs/reports/.

Usage: python tools/torque_report.py [--out DIR] [--quick]

Runs the 48-scenario matrix (a stand plus walking, sideways and turning at three robot masses) with
the torque clip at the servo's stall torque, and writes v1-torque-report.md and .json into DIR
(default docs/reports/). --quick runs three short scenarios as a smoke check and writes
v1-torque-report-quick.* instead, so the real report is never overwritten.
"""

import argparse
import json
import subprocess
import sys
from datetime import date
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for package in ("bemu_params", "bemu_kinematics", "bemu_description", "bemu_gait", "bemu_sim"):
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / package))

from bemu_description.actuators import load_actuator_model  # noqa: E402
from bemu_gait import GaitParamsError, load_gait_params  # noqa: E402
from bemu_kinematics import KinematicsError  # noqa: E402
from bemu_params import ParamsError, load_params  # noqa: E402
from bemu_sim.torque_report import (  # noqa: E402
    Scenario,
    build_matrix,
    render_markdown,
    run_scenario,
    to_json,
)

PARAMS_FILE = REPO_ROOT / "config" / "robot_params.yaml"
ACTUATORS_FILE = REPO_ROOT / "ros2_ws" / "src" / "bemu_sim" / "config" / "actuators.yaml"
DEFAULT_OUT = REPO_ROOT / "docs" / "reports"


def main() -> int:
    try:
        return _run()
    except (ParamsError, GaitParamsError, KinematicsError, ValueError, OSError) as exc:
        print(f"torque_report: {exc}", file=sys.stderr)
        return 1


def _commit() -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        return out.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _run() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="output directory")
    parser.add_argument(
        "--quick", action="store_true", help="three short scenarios, as a smoke check"
    )
    args = parser.parse_args()

    params = load_params(PARAMS_FILE)
    actuators = load_actuator_model(ACTUATORS_FILE, params.servo)
    gait_params = load_gait_params()
    args.out.mkdir(parents=True, exist_ok=True)

    if args.quick:
        scenarios = [
            Scenario("tripod", "stand", 0.0, 2.5),
            Scenario("tripod", "forward", 1.0, 2.5),
            Scenario("wave", "turn", 1.0, 2.5),
        ]
        timing = {"warmup": 2.0, "seconds": 4.0}
    else:
        scenarios = build_matrix(params, gait_params)
        timing = {}

    results = []
    for number, scenario in enumerate(scenarios, 1):
        print(
            f"[{number}/{len(scenarios)}] {scenario.gait} {scenario.motion} "
            f"{scenario.fraction:g} @ {scenario.total_mass} kg",
            flush=True,
        )
        results.append(run_scenario(params, actuators, gait_params, scenario, **timing))

    commit, today = _commit(), date.today().isoformat()
    markdown = render_markdown(
        results, params, actuators, commit=commit, date=today, quick=args.quick
    )
    data = to_json(results, params, actuators, commit=commit, date=today)
    stem = "v1-torque-report-quick" if args.quick else "v1-torque-report"  # never the real report
    (args.out / f"{stem}.json").write_text(json.dumps(data, indent=2) + "\n")
    (args.out / f"{stem}.md").write_text(markdown + "\n")
    print(f"wrote {args.out / stem}.md and .json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
