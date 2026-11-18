"""The foot-trace logic, free of ROS: the node calls `step` once per control tick.

Each call takes the joint angles measured now (the result of the previous command) and
returns the next command, or None when the trace is finished. The tick sequence matches
`bemu_sim.headless.track_path`: a settle move to the first point, then one point per tick.
"""

from __future__ import annotations

import numpy as np

from bemu_kinematics import JointAngles, JointLimitError, UnreachableError, forward, inverse
from bemu_params import LegParams


class FootTrace:
    def __init__(self, leg: LegParams, path: np.ndarray, *, settle_ticks: int = 50) -> None:
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[1] != 3 or len(path) == 0:
            raise ValueError(f"path must have shape (n, 3) with n > 0, got {path.shape}")
        if settle_ticks < 1:
            raise ValueError(f"settle_ticks must be at least 1, got {settle_ticks}")
        self._leg = leg
        self._path = path
        self._settle_ticks = settle_ticks
        self._targets = self._solve(leg, path)  # raises before anything moves
        self._tick = 0  # commands issued so far
        self._start: np.ndarray | None = None
        self._errors: list[float] = []

    @staticmethod
    def _solve(leg: LegParams, path: np.ndarray) -> list[JointAngles]:
        solved = []
        for i, point in enumerate(path):
            try:
                solved.append(inverse(leg, point))
            except UnreachableError as exc:
                raise UnreachableError(f"path point {i}: {exc}", exc.reason) from exc
            except JointLimitError as exc:
                raise JointLimitError(f"path point {i}: {exc}", exc.joint) from exc
        return solved

    @property
    def done(self) -> bool:
        return len(self._errors) == len(self._targets)

    @property
    def max_error(self) -> float:
        if not self.done:
            raise RuntimeError("the trace has not finished")
        return max(self._errors)

    def step(self, measured: JointAngles) -> JointAngles | None:
        n_settle = self._settle_ticks
        # `measured` is the result of the last command issued. If that was a path point,
        # score it (once) against the foot position it was meant to reach.
        issued = self._tick - n_settle  # path commands issued so far
        if issued >= 1 and len(self._errors) < issued:
            foot = forward(self._leg, measured)
            self._errors.append(float(np.linalg.norm(foot - self._path[issued - 1])))

        if self._tick >= n_settle + len(self._targets):
            return None

        if self._start is None:
            self._start = np.array(measured)
        if self._tick < n_settle:
            frac = (self._tick + 1) / n_settle
            goal = np.array(self._targets[0])
            cmd = JointAngles(*(self._start + (goal - self._start) * frac))
        else:
            cmd = self._targets[self._tick - n_settle]
        self._tick += 1
        return cmd
