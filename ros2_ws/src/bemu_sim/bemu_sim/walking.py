"""Drive the full simulated robot with the walker and measure how it walked (headless)."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from bemu_gait import Walker
from bemu_kinematics import stability_margin, stand_up_targets
from bemu_params import LEG_CODES, RobotParams
from bemu_sim.headless import CONTROL_RATE
from bemu_sim.robot import RobotSim, play

SETTLE_STEPS = 50  # 1 s of holding the standing pose


@dataclass(frozen=True)
class WalkLog:
    """One row per control tick."""

    t: np.ndarray  # (n,) s
    position: np.ndarray  # (n, 3) body, ground frame
    yaw: np.ndarray  # (n,) rad, unwrapped
    tilt: np.ndarray  # (n,) rad
    margin: np.ndarray  # (n,) m, measured from the sim: COM against the feet that touch
    feet: np.ndarray  # (n, 6, 3) foot centres, ground frame, LEG_CODES order
    in_stance: np.ndarray  # (n, 6) bool, the walker's flags
    contact: np.ndarray  # (n, 6) bool, the foot touches the floor in the simulation
    scale: np.ndarray  # (n,) stride scale the walker applied
    gait: list[str]
    peak_torque: float  # N·m, the largest on any physics step over the run


@dataclass(frozen=True)
class WalkState:
    """What a schedule sees before each tick."""

    t: float  # s since the walk began
    position: np.ndarray  # (3,) body, ground frame
    yaw: float  # rad, unwrapped
    heading_change: float  # rad since the walk began


def stand_robot(sim: RobotSim, params: RobotParams) -> None:
    """Stand the robot up from `sit`, then hold the pose for a second."""
    targets = stand_up_targets(params)
    play(sim, targets)
    play(sim, [targets[-1]] * SETTLE_STEPS)


def walk(
    sim: RobotSim,
    walker: Walker,
    seconds: float,
    *,
    schedule: Callable[[WalkState, Walker], None] | None = None,
) -> WalkLog:
    """Run the walker against the simulator for `seconds`, recording a row per control tick.

    `schedule(state, walker)` runs before each tick and may change the command, gait or pose.
    """
    dt = 1.0 / CONTROL_RATE
    ticks = round(seconds * CONTROL_RATE)
    sim.clear_peak()
    yaw0 = prev = sim.body_euler()[2]
    unwrapped = yaw0
    rows = {
        name: [] for name in ("t", "position", "yaw", "tilt", "margin", "feet", "stance", "contact")
    }
    scale, gait = [], []
    for k in range(ticks):
        if schedule is not None:
            schedule(WalkState(k * dt, sim.body_position(), unwrapped, unwrapped - yaw0), walker)
        out = walker.step(dt)
        sim.command(out.angles)

        now = sim.body_euler()[2]
        unwrapped += (now - prev + math.pi) % (2 * math.pi) - math.pi
        prev = now
        feet = sim.foot_positions()
        contacts = sim.foot_contacts()
        touching = [feet[leg] for leg, down in contacts.items() if down]
        rows["t"].append((k + 1) * dt)
        rows["position"].append(sim.body_position())
        rows["yaw"].append(unwrapped)
        rows["tilt"].append(sim.tilt())
        rows["margin"].append(
            stability_margin(sim.center_of_mass(), np.array(touching).reshape(-1, 3))
        )
        rows["feet"].append(np.array([feet[leg] for leg in LEG_CODES]))
        rows["stance"].append([out.in_stance[leg] for leg in LEG_CODES])
        rows["contact"].append([contacts[leg] for leg in LEG_CODES])
        scale.append(out.scale)
        gait.append(out.gait)
    return WalkLog(
        t=np.array(rows["t"]),
        position=np.array(rows["position"]),
        yaw=np.array(rows["yaw"]),
        tilt=np.array(rows["tilt"]),
        margin=np.array(rows["margin"]),
        feet=np.array(rows["feet"]),
        in_stance=np.array(rows["stance"], dtype=bool),
        contact=np.array(rows["contact"], dtype=bool),
        scale=np.array(scale),
        gait=gait,
        peak_torque=sim.peak_torque,
    )


def travelled(log: WalkLog) -> tuple[float, float]:
    """(forward, lateral) distance of the body from its start, in the start heading's frame."""
    dx, dy = (log.position[-1, :2] - log.position[0, :2]).tolist()
    c, s = math.cos(log.yaw[0]), math.sin(log.yaw[0])
    return dx * c + dy * s, -dx * s + dy * c


def heading_change(log: WalkLog) -> float:
    """Rad turned from the first row to the last."""
    return float(log.yaw[-1] - log.yaw[0])


def max_stance_slip(log: WalkLog) -> float:
    """Largest ground-frame movement (m) of any foot within one contiguous loaded stance.

    Loaded: the walker has the leg in stance and the foot touches the floor. A foot still settling
    onto the floor at touchdown is not loaded, so its drift is not slip.
    """
    worst = 0.0
    for leg in range(log.feet.shape[1]):
        start = None
        for i, down in enumerate(log.in_stance[:, leg] & log.contact[:, leg]):
            if not down:
                start = None
                continue
            if start is None:
                start = i
            moved = float(np.linalg.norm(log.feet[i, leg, :2] - log.feet[start, leg, :2]))
            worst = max(worst, moved)
    return worst
