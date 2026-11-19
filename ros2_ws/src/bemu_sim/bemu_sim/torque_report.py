"""The torque report: simulated walks, set against the servo decision rule (docs/06, docs/10).

Scenarios run with the torque clip raised to the servo's stall torque, so peaks show what a walk
asks for; the 50%-of-stall rule is then applied here, not hidden by the simulator's design limit.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from bemu_description.actuators import ActuatorModel
from bemu_description.names import joint_order
from bemu_description.robot_mjcf import build_robot_mjcf
from bemu_gait import GAITS, Command, GaitParams, Walker
from bemu_kinematics import BodyPose, neutral_feet, pose_angles, stand_height, stand_up_targets
from bemu_params import RobotParams, ServoParams
from bemu_sim.headless import CONTROL_RATE
from bemu_sim.robot import RobotSim, play
from bemu_sim.walking import max_stance_slip, stand_robot, walk

MOTIONS = ("stand", "forward", "sideways", "turn")
MASSES = (2.0, 2.2, 2.5)  # kg, total robot mass
FRACTIONS = (0.25, 0.5, 1.0)  # of a gait's top forward speed
STAND_SECONDS = 3.0
MARGIN_BAR = 0.015  # m


@dataclass(frozen=True)
class Scenario:
    gait: str
    motion: str
    fraction: float  # of the gait's top speed on the motion's axis; 0 for "stand"
    total_mass: float  # kg

    def __post_init__(self) -> None:
        if self.gait not in GAITS:
            raise ValueError(f"unknown gait {self.gait!r}; valid gaits: {', '.join(GAITS)}")
        if self.motion not in MOTIONS:
            raise ValueError(f"unknown motion {self.motion!r}; valid motions: {', '.join(MOTIONS)}")
        if self.motion != "stand" and not (math.isfinite(self.fraction) and self.fraction > 0):
            raise ValueError(f"fraction must be positive, got {self.fraction!r}")


@dataclass(frozen=True)
class ScenarioResult:
    scenario: Scenario
    command: tuple[float, float, float]  # vx, vy, omega
    joints: tuple[str, ...]
    peak_torque: np.ndarray  # (18,) N·m, over the whole run
    rms_torque: np.ndarray  # (18,) N·m, steady part
    mean_abs_torque: np.ndarray  # (18,) N·m, steady part
    peak_speed: np.ndarray  # (18,) rad/s, over the whole run
    steps_at_limit: np.ndarray  # (18,) physics steps at the actuator limit, whole run
    min_margin: float | None  # m; None when standing
    max_slip: float | None  # m; None when standing
    power: float  # W, estimated, steady part


@dataclass(frozen=True)
class Verdict:
    peak_ok: bool  # worst joint's peak under 50% of the stall torque
    rms_ok: bool  # worst RMS under the rated torque
    stable: bool  # minimum margin above 15 mm (or standing)
    peak_vs_stall: float  # worst peak / stall torque
    rms_vs_rated: float  # worst RMS / rated torque
    speed_vs_max: float  # peak joint speed / the no-load speed
    steps_at_limit: int  # physics steps, all joints, at the actuator limit


def top_speeds(params: RobotParams, gait_params: GaitParams) -> dict[str, dict[str, float]]:
    """Each gait's fastest forward and sideways speed (m/s) and turn rate (rad/s)."""
    farthest = max(math.hypot(*foot[:2]) for foot in neutral_feet(params).values())
    speeds = {}
    for name, gait in GAITS.items():
        straight = gait_params.max_stride / (gait.duty * gait_params.cycle_time[name])
        speeds[name] = {"forward": straight, "sideways": straight, "turn": straight / farthest}
    return speeds


def build_matrix(params: RobotParams, gait_params: GaitParams) -> list[Scenario]:
    """16 scenarios per mass: one stand, then per gait forward at three speeds, sideways, turn."""
    matrix = []
    for mass in MASSES:
        matrix.append(Scenario("tripod", "stand", 0.0, mass))
        for gait in GAITS:
            matrix += [Scenario(gait, "forward", f, mass) for f in FRACTIONS]
            matrix.append(Scenario(gait, "sideways", 1.0, mass))
            matrix.append(Scenario(gait, "turn", 1.0, mass))
    return matrix


def params_for_mass(params: RobotParams, total_mass: float) -> RobotParams:
    """The same robot with the chassis mass that makes the total `total_mass` (legs unchanged)."""
    legs = 6 * (params.leg.coxa_mass + params.leg.femur_mass + params.leg.tibia_mass)
    if not (math.isfinite(total_mass) and total_mass - legs >= 0.001):
        raise ValueError(
            f"total mass must leave the chassis at least 1 g above the six legs' "
            f"{legs:.3f} kg, got {total_mass!r}"
        )
    chassis = dataclasses.replace(params.body, mass=total_mass - legs)
    return dataclasses.replace(params, body=chassis)


def report_actuators(actuators: ActuatorModel, servo: ServoParams) -> ActuatorModel:
    """The actuator model with its torque clip at the servo's stall torque, so peaks are demand."""
    return dataclasses.replace(actuators, max_torque=servo.stall_torque)


def electrical_power(servo: ServoParams, mean_abs_torque: ArrayLike) -> float:
    """Estimated electrical power (W): the sum over joints of V (I_idle + |tau| / k_t),
    with k_t = stall torque / stall current."""
    torque = np.asarray(mean_abs_torque, dtype=float)
    if torque.shape != (len(joint_order()),):
        raise ValueError(
            f"expected one mean torque per joint ({len(joint_order())}), got {torque.shape}"
        )
    kt = servo.stall_torque / servo.stall_current
    return float(np.sum(servo.supply_voltage * (servo.idle_current + torque / kt)))


def verdict(result: ScenarioResult, servo: ServoParams) -> Verdict:
    peak = float(result.peak_torque.max())
    rms = float(result.rms_torque.max())
    return Verdict(
        peak_ok=peak < 0.5 * servo.stall_torque,
        rms_ok=rms < servo.rated_torque,
        stable=result.min_margin is None or result.min_margin > MARGIN_BAR,
        peak_vs_stall=peak / servo.stall_torque,
        rms_vs_rated=rms / servo.rated_torque,
        speed_vs_max=float(result.peak_speed.max()) / servo.max_speed,
        steps_at_limit=int(result.steps_at_limit.sum()),
    )


def run_scenario(
    params: RobotParams,
    actuators: ActuatorModel,
    gait_params: GaitParams,
    scenario: Scenario,
    *,
    warmup: float = 6.0,
    seconds: float = 24.0,
) -> ScenarioResult:
    """Run one scenario. Peak torque and speed cover the whole run; RMS, mean torque, power,
    margin and slip cover the steady part after the warm-up."""
    robot = params_for_mass(params, scenario.total_mass)
    sim = RobotSim(
        build_robot_mjcf(robot, report_actuators(actuators, params.servo)),
        count_limit=actuators.max_torque,  # steps over the design limit, though clipped at stall
    )
    joints = joint_order()

    if scenario.motion == "stand":
        up = stand_up_targets(robot)
        play(sim, up)
        first = sim.stats()
        hold = pose_angles(robot, BodyPose(stand_height(robot)))
        play(sim, [hold] * round(STAND_SECONDS * CONTROL_RATE))
        steady, margin, slip, command = sim.stats(), None, None, (0.0, 0.0, 0.0)
    else:
        stand_robot(sim, robot)
        speed = scenario.fraction * top_speeds(params, gait_params)[scenario.gait][scenario.motion]
        command = {
            "forward": (speed, 0.0, 0.0),
            "sideways": (0.0, speed, 0.0),
            "turn": (0.0, 0.0, speed),
        }[scenario.motion]
        walker = Walker(robot, gait_params, gait=scenario.gait)
        walker.set_command(Command(*command))
        walk(sim, walker, warmup)
        first = sim.stats()
        log = walk(sim, walker, seconds)
        steady = sim.stats()
        margin, slip = float(log.margin.min()), max_stance_slip(log)

    return ScenarioResult(
        scenario=scenario,
        command=command,
        joints=joints,
        peak_torque=np.maximum(first.peak_torque, steady.peak_torque),
        rms_torque=steady.rms_torque,
        mean_abs_torque=steady.mean_abs_torque,
        peak_speed=np.maximum(first.peak_speed, steady.peak_speed),
        steps_at_limit=first.steps_at_limit + steady.steps_at_limit,
        min_margin=margin,
        max_slip=slip,
        power=electrical_power(params.servo, steady.mean_abs_torque),
    )


PLACEHOLDERS = ["stall_current_a", "idle_current_a", "supply_voltage_v"]


def _label(scenario: Scenario) -> str:
    if scenario.motion == "stand":
        return "stand"
    if scenario.motion == "forward":
        return f"{scenario.gait} forward {scenario.fraction:.0%}"
    return f"{scenario.gait} {scenario.motion}"


def _speed(result: ScenarioResult) -> str:
    vx, vy, omega = result.command
    if result.scenario.motion == "turn":
        return f"{omega:.2f} rad/s"
    if result.scenario.motion == "stand":
        return "0"
    return f"{math.hypot(vx, vy) * 1000:.0f} mm/s"


def _status(result: ScenarioResult, servo: ServoParams) -> str:
    v = verdict(result, servo)
    parts = [
        "PASS" if v.peak_ok and v.rms_ok else "FAIL",
    ]
    if not v.stable:
        parts.append("UNSTABLE")
    return " ".join(parts)


def _worst(joints: Sequence[str], values: np.ndarray) -> tuple[str, float]:
    index = int(np.argmax(values))
    return joints[index], float(values[index])


def render_markdown(
    results: Sequence[ScenarioResult],
    params: RobotParams,
    actuators: ActuatorModel,
    *,
    commit: str,
    date: str,
    quick: bool = False,
) -> str:
    servo = params.servo
    out = [
        "# Version 1 torque report" + (" (quick smoke run, not the report)" if quick else ""),
        "",
        f"Generated {date} at commit `{commit}` by `tools/torque_report.py`.",
        "",
        "## Method",
        "",
        "Each scenario runs the full MuJoCo robot with the walker, with the actuator torque clip "
        f"raised to the servo's stall torque ({servo.stall_torque:.2f} N·m) instead of the "
        f"{actuators.max_torque:.2f} N·m design limit, so the peaks show what the motion demands. "
        "Torque and speed statistics are taken at every 2 ms physics step. Peak torque and speed "
        "cover the whole run; RMS torque, power, stability margin and slip cover the steady part "
        "after a warm-up walk. Speeds are fractions of each gait's top speed.",
        "",
        "Rule: a row passes when its worst peak is under 50% of stall "
        f"({0.5 * servo.stall_torque:.2f} N·m) "
        f"and its worst RMS is under the rated torque ({servo.rated_torque:.2f} N·m). "
        "`UNSTABLE` marks a stability margin of 15 mm or less.",
        "",
        "Placeholder constants (not datasheet values, to be confirmed on the real servo): "
        f"`stall_current_a` = {servo.stall_current} A, `idle_current_a` = {servo.idle_current} A, "
        f"`supply_voltage_v` = {servo.supply_voltage} V. They only feed the power estimate; "
        "torque, speed and the verdicts do not depend on them.",
        "",
    ]
    passing = sum(1 for r in results if _status(r, servo) == "PASS")
    out += [
        "## Summary",
        "",
        f"{passing} of {len(results)} rows pass the peak and RMS rules "
        "and keep a stability margin above 15 mm.",
    ]
    at_limit = sum(verdict(r, servo).steps_at_limit > 0 for r in results)
    peak_row = max(results, key=lambda r: float(r.peak_torque.max()))
    rms_row = max(results, key=lambda r: float(r.rms_torque.max()))
    out += [
        f"- Largest peak: {peak_row.peak_torque.max():.2f} N·m "
        f"({verdict(peak_row, servo).peak_vs_stall:.0%} of stall), "
        f"{_label(peak_row.scenario)} at {peak_row.scenario.total_mass} kg.",
        f"- Largest RMS: {rms_row.rms_torque.max():.2f} N·m "
        f"({verdict(rms_row, servo).rms_vs_rated:.0%} of rated), "
        f"{_label(rms_row.scenario)} at {rms_row.scenario.total_mass} kg.",
        f"- Rows with steps over the design limit: {at_limit}.",
        f"- {sum(not verdict(r, servo).stable for r in results)} unstable rows.",
        "",
    ]
    for mass in sorted({r.scenario.total_mass for r in results}):
        out += [
            f"## {mass} kg",
            "",
            "| Scenario | Speed | Peak (joint) | Peak % stall | RMS (joint) | RMS % rated "
            "| Max speed % | Power W | Steps at limit | Min margin mm | Slip mm | Verdict |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|",
        ]
        for r in (x for x in results if x.scenario.total_mass == mass):
            v = verdict(r, servo)
            pj, pv = _worst(r.joints, r.peak_torque)
            rj, rv = _worst(r.joints, r.rms_torque)
            margin = "-" if r.min_margin is None else f"{r.min_margin * 1000:.0f}"
            slip = "-" if r.max_slip is None else f"{r.max_slip * 1000:.1f}"
            out.append(
                f"| {_label(r.scenario)} | {_speed(r)} | {pv:.2f} N·m ({pj}) | "
                f"{v.peak_vs_stall:.0%} | {rv:.2f} N·m ({rj}) | {v.rms_vs_rated:.0%} | "
                f"{v.speed_vs_max:.0%} | {r.power:.0f} | {v.steps_at_limit} | {margin} | "
                f"{slip} | {_status(r, servo)} |"
            )
        out.append("")
    tops = [
        r
        for r in results
        if r.scenario.total_mass == 2.5
        and r.scenario.motion == "forward"
        and r.scenario.fraction == 1.0
    ]
    for r in tops:
        out += [
            f"## Per-joint torque: {r.scenario.gait} forward at top speed, 2.5 kg",
            "",
            "| Joint | Peak N·m | RMS N·m |",
            "|---|---|---|",
        ]
        out += [
            f"| {name} | {r.peak_torque[i]:.3f} | {r.rms_torque[i]:.3f} |"
            for i, name in enumerate(r.joints)
        ]
        out.append("")
    return "\n".join(out)


def to_json(
    results: Sequence[ScenarioResult],
    params: RobotParams,
    actuators: ActuatorModel,
    *,
    commit: str,
    date: str,
) -> dict:
    servo = params.servo

    def per_joint(r: ScenarioResult, values: np.ndarray) -> dict[str, float]:
        return {name: float(values[i]) for i, name in enumerate(r.joints)}

    rows = []
    for r in results:
        rows.append(
            {
                "gait": r.scenario.gait,
                "motion": r.scenario.motion,
                "fraction": r.scenario.fraction,
                "total_mass": r.scenario.total_mass,
                "command": list(r.command),
                "peak_torque": per_joint(r, r.peak_torque),
                "rms_torque": per_joint(r, r.rms_torque),
                "mean_abs_torque": per_joint(r, r.mean_abs_torque),
                "peak_speed": per_joint(r, r.peak_speed),
                "steps_at_limit": {n: int(r.steps_at_limit[i]) for i, n in enumerate(r.joints)},
                "min_margin": r.min_margin,
                "max_slip": r.max_slip,
                "power": r.power,
                "verdict": dataclasses.asdict(verdict(r, servo)),
            }
        )
    return {
        "commit": commit,
        "date": date,
        "servo": {
            "model": servo.model,
            "stall_torque": servo.stall_torque,
            "rated_torque": servo.rated_torque,
            "max_speed": servo.max_speed,
            "stall_current_a": servo.stall_current,
            "idle_current_a": servo.idle_current,
            "supply_voltage_v": servo.supply_voltage,
        },
        "design_limit_torque": actuators.max_torque,
        "placeholders": list(PLACEHOLDERS),
        "results": rows,
    }
