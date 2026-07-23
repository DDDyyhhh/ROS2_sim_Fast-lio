"""Pure command arbitration for the simulation-only motion path."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class MuxCommand:
    """A planar velocity command selected by the simulation arbiter."""

    linear_x: float = 0.0
    angular_z: float = 0.0


class CommandMux:
    """Prioritize an active plan and fail closed on stale inputs."""

    def __init__(self, timeout=0.4):
        if timeout <= 0.0:
            raise ValueError('mux timeout must be positive')
        self.timeout = float(timeout)
        self.plan_active = False
        self._teleop = MuxCommand()
        self._plan = MuxCommand()
        self._teleop_at = None
        self._plan_at = None

    def set_plan_active(self, active):
        self.plan_active = bool(active)

    def accept_teleop(self, command, now):
        self._teleop = self._normalize(command)
        self._teleop_at = float(now)

    def accept_plan(self, command, now):
        self._plan = self._normalize(command)
        self._plan_at = float(now)

    def output(self, now):
        now = float(now)
        if self.plan_active:
            if self._fresh(self._plan_at, now):
                return self._plan
            return MuxCommand()
        if self._fresh(self._teleop_at, now):
            return self._teleop
        return MuxCommand()

    @staticmethod
    def _normalize(command):
        values = (float(command.linear_x), float(command.angular_z))
        if not all(isfinite(value) for value in values):
            return MuxCommand()
        return MuxCommand(*values)

    def _fresh(self, observed_at, now):
        return observed_at is not None and now - observed_at <= self.timeout
