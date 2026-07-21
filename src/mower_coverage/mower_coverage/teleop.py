"""Dead-man and timeout safety for simulation-only web teleoperation."""

from dataclasses import dataclass


@dataclass
class TeleopCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


class TeleopGate:
    """Allow a command only while capture is active and input is fresh."""

    def __init__(self, timeout=0.4, max_linear=0.6, max_angular=1.2):
        if timeout <= 0.0 or max_linear <= 0.0 or max_angular <= 0.0:
            raise ValueError('teleop limits must be positive')
        self.timeout = float(timeout)
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.allowed = False
        self._command = TeleopCommand()
        self._last_command_at = None

    def set_allowed(self, allowed):
        self.allowed = bool(allowed)
        if not self.allowed:
            self._command = TeleopCommand()

    def accept(self, command, now):
        if not isinstance(command, TeleopCommand):
            raise ValueError('teleop command must be TeleopCommand')
        self._command = TeleopCommand(
            max(-self.max_linear, min(self.max_linear, float(command.linear_x))),
            max(-self.max_angular, min(self.max_angular, float(command.angular_z))),
        )
        self._last_command_at = float(now)

    def output(self, now):
        if not self.allowed or self._last_command_at is None:
            return TeleopCommand()
        if float(now) - self._last_command_at > self.timeout:
            return TeleopCommand()
        return TeleopCommand(
            self._command.linear_x,
            self._command.angular_z,
        )
