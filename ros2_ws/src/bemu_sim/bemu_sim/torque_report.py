"""The torque report: simulated walks, set against the servo decision rule (docs/06, docs/10).

Scenarios run with the torque clip raised to the servo's stall torque, so peaks show what a walk
asks for; the 50%-of-stall rule is then applied here, not hidden by the simulator's design limit.
"""

from __future__ import annotations

import dataclasses
import math
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
    sim = RobotSim(build_robot_mjcf(robot, report_actuators(actuators, params.servo)))
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
