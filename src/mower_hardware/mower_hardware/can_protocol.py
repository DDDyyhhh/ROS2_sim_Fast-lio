"""Pure offline decoder for the delivered RK3588 ↔ STM32 CAN protocol.

This module deliberately has no SocketCAN, ROS, or ``python-can`` dependency.  It
can decode captured frames and ``candump -L`` text, but it never opens an
interface or creates an outgoing frame.
"""

from dataclasses import dataclass
import re
from typing import Iterable, Optional, Tuple, Union


CONTROL_CAN_ID = 0x005
FEEDBACK_CAN_ID = 0x507
STANDARD_CAN_MAX_ID = 0x7FF
CAN_FRAME_DLC = 8
CAN_BITRATE = 500_000
CONTROL_PERIOD_S = 0.05
CONTROL_TIMEOUT_S = 0.5
UNLOCK_MAGIC_KEY = 0xAA

_RESERVED_STATUS_MASK = 0xC8
_CANDUMP_LINE = re.compile(
    r"^\s*(?:\((?P<timestamp>[0-9]+(?:\.[0-9]+)?)\)\s+)?"
    r"(?P<interface>\S+)\s+"
    r"(?P<can_id>[0-9A-Fa-f]{1,3})#(?P<data>[0-9A-Fa-f]*)\s*$"
)


class CanProtocolError(ValueError):
    """Raised when a frame violates the known protocol boundary."""


@dataclass(frozen=True)
class CanFrame:
    """One captured classic-CAN data frame."""

    can_id: int
    data: bytes
    timestamp: Optional[float] = None
    interface: Optional[str] = None

    def __post_init__(self):
        if not isinstance(self.can_id, int):
            raise TypeError('can_id must be an integer')
        if not 0 <= self.can_id <= STANDARD_CAN_MAX_ID:
            raise CanProtocolError(
                f'CAN ID must be an 11-bit standard ID: {self.can_id!r}')
        if not isinstance(self.data, (bytes, bytearray)):
            raise TypeError('data must be bytes-like')
        if not isinstance(self.data, bytes):
            object.__setattr__(self, 'data', bytes(self.data))


@dataclass(frozen=True)
class ControlCommand:
    """Decoded fields from the RK3588 → STM32 control frame.

    Raw values are kept as received.  In particular, this decoder does not
    reproduce the STM32's downstream clamping of blade/lifter levels.
    """

    ctrl_mode: int
    vx_raw: int
    vy_raw: int
    omega_raw: int
    blade_level: int
    lifter_level: int
    bag_target: int
    magic_key: int

    @property
    def vx_mps(self):
        return self.vx_raw * 0.1

    @property
    def omega_rad_s(self):
        return self.omega_raw * 0.1

    @property
    def bag_open_requested(self):
        return self.bag_target != 0

    @property
    def unlock_requested(self):
        return self.magic_key == UNLOCK_MAGIC_KEY


@dataclass(frozen=True)
class FeedbackStatus:
    """Decoded fields from the STM32 → RK3588 status frame."""

    vx_raw: int
    omega_raw: int
    ctrl_mode: int
    battery_soc: int
    bms_temperature_raw: int
    status_flags: int

    @property
    def vx_mps(self):
        return self.vx_raw / 1000.0

    @property
    def omega_rad_s(self):
        return self.omega_raw / 1000.0

    @property
    def bms_temperature_c(self):
        return None if self.bms_temperature_raw == 0xFF else self.bms_temperature_raw

    @property
    def bag_open(self):
        return bool(self.status_flags & 0x01)

    @property
    def bag_connected(self):
        return bool(self.status_flags & 0x02)

    @property
    def bag_full(self):
        return bool(self.status_flags & 0x04)

    @property
    def blade_level(self):
        return (self.status_flags & 0x30) >> 4


CanMessage = Union[ControlCommand, FeedbackStatus]


@dataclass(frozen=True)
class DecodedCanFrame:
    """A captured frame together with its known protocol payload."""

    frame: CanFrame
    message: CanMessage


@dataclass(frozen=True)
class ReplayRecord:
    """One recognized line from an offline capture replay."""

    line_number: int
    decoded: DecodedCanFrame


def _require_payload(frame: CanFrame, expected_id: int):
    if frame.can_id != expected_id:
        raise CanProtocolError(
            f'expected CAN ID 0x{expected_id:03X}, got 0x{frame.can_id:03X}')
    if len(frame.data) != CAN_FRAME_DLC:
        raise CanProtocolError(
            f'CAN ID 0x{expected_id:03X} requires DLC=8, got {len(frame.data)}')


def _signed_byte(value: int) -> int:
    return value - 0x100 if value & 0x80 else value


def decode_control_frame(frame: CanFrame) -> ControlCommand:
    """Decode one `0x005` frame without applying downstream clamping."""

    _require_payload(frame, CONTROL_CAN_ID)
    data = frame.data
    ctrl_mode = data[0]
    if ctrl_mode not in (0, 1, 2):
        raise CanProtocolError(f'unsupported ctrl_mode: {ctrl_mode}')

    vy_raw = _signed_byte(data[2])
    if vy_raw != 0:
        raise CanProtocolError(
            f'vy_raw must be 0 for the differential chassis, got {vy_raw}')

    magic_key = data[7]
    if magic_key not in (0, UNLOCK_MAGIC_KEY):
        raise CanProtocolError(
            f'magic_key must be 0x00 or 0xAA, got 0x{magic_key:02X}')

    return ControlCommand(
        ctrl_mode=ctrl_mode,
        vx_raw=_signed_byte(data[1]),
        vy_raw=vy_raw,
        omega_raw=_signed_byte(data[3]),
        blade_level=data[4],
        lifter_level=data[5],
        bag_target=data[6],
        magic_key=magic_key,
    )


def decode_feedback_frame(frame: CanFrame) -> FeedbackStatus:
    """Decode one `0x507` frame and reject undefined status bits."""

    _require_payload(frame, FEEDBACK_CAN_ID)
    data = frame.data
    ctrl_mode = data[4]
    if ctrl_mode not in (0, 1, 2):
        raise CanProtocolError(f'unsupported feedback ctrl_mode: {ctrl_mode}')

    status_flags = data[7]
    if status_flags & _RESERVED_STATUS_MASK:
        raise CanProtocolError(
            f'feedback status contains reserved bits: 0x{status_flags:02X}')

    return FeedbackStatus(
        vx_raw=int.from_bytes(data[0:2], byteorder='big', signed=True),
        omega_raw=int.from_bytes(data[2:4], byteorder='big', signed=True),
        ctrl_mode=ctrl_mode,
        battery_soc=data[5],
        bms_temperature_raw=data[6],
        status_flags=status_flags,
    )


def decode_frame(frame: CanFrame) -> Optional[DecodedCanFrame]:
    """Decode a delivered frame, or return ``None`` for an unknown ID.

    Unsupported IDs such as `0x508` and `0x509` are intentionally not mapped
    to a guessed message type.
    """

    if frame.can_id == CONTROL_CAN_ID:
        message = decode_control_frame(frame)
    elif frame.can_id == FEEDBACK_CAN_ID:
        message = decode_feedback_frame(frame)
    else:
        return None
    return DecodedCanFrame(frame=frame, message=message)


def parse_candump_line(line: str) -> CanFrame:
    """Parse one classic-CAN line emitted by ``candump -L``.

    A timestamp is optional to also support small hand-written fixtures.  CAN
    FD, remote frames, and malformed data are rejected instead of being
    silently interpreted as this classic-CAN protocol.
    """

    match = _CANDUMP_LINE.match(line)
    if match is None:
        raise CanProtocolError(f'invalid candump line: {line.rstrip()}')

    timestamp = match.group('timestamp')
    return CanFrame(
        can_id=int(match.group('can_id'), 16),
        data=bytes.fromhex(match.group('data')),
        timestamp=None if timestamp is None else float(timestamp),
        interface=match.group('interface'),
    )


def replay_candump(lines: Iterable[str]) -> Tuple[ReplayRecord, ...]:
    """Decode recognized frames from a capture without touching any CAN device.

    Blank lines and frames with IDs outside the delivered `0x005`/`0x507`
    interface are skipped.  A malformed line or malformed recognized frame is
    raised so an offline regression cannot hide a broken capture.
    """

    records = []
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        frame = parse_candump_line(line)
        decoded = decode_frame(frame)
        if decoded is not None:
            records.append(ReplayRecord(line_number, decoded))
    return tuple(records)


def validate_control_replay(records: Iterable[ReplayRecord]) -> None:
    """Audit the documented startup and control timing rules offline.

    This validates only facts represented by the delivered frames: the
    `ctrl_mode=1` then `2` handshake, feedback confirmation before motion,
    control-frame timeout, and non-consecutive unlock keys.  Collision, E-stop,
    and remote-takeover causes cannot be audited because `0x507` has no such
    fields.
    """

    saw_control = False
    standby_requested = False
    standby_confirmed = False
    run_requested = False
    run_confirmed = False
    exited_api = False
    unlock_active = False
    last_control_timestamp = None

    for record in records:
        message = record.decoded.message
        if isinstance(message, FeedbackStatus):
            if message.ctrl_mode == 1 and standby_requested:
                standby_confirmed = True
            if message.ctrl_mode == 2 and run_requested:
                run_confirmed = True
            continue

        command = message
        if not isinstance(command, ControlCommand):
            raise CanProtocolError(
                f'line {record.line_number}: unknown decoded message type')
        if command.magic_key == UNLOCK_MAGIC_KEY:
            if unlock_active:
                raise CanProtocolError(
                    f'line {record.line_number}: repeated unlock magic key')
            unlock_active = True
        else:
            unlock_active = False

        timestamp = record.decoded.frame.timestamp
        if timestamp is None:
            raise CanProtocolError(
                f'line {record.line_number}: control replay needs timestamps')
        if last_control_timestamp is not None:
            gap = timestamp - last_control_timestamp
            if gap < 0:
                raise CanProtocolError(
                    f'line {record.line_number}: control timestamp moved backwards')
            if gap > CONTROL_TIMEOUT_S:
                raise CanProtocolError(
                    f'line {record.line_number}: control gap {gap:.3f}s '
                    f'exceeds {CONTROL_TIMEOUT_S:.3f}s')
        last_control_timestamp = timestamp

        has_motion = command.vx_raw != 0 or command.omega_raw != 0
        if exited_api:
            raise CanProtocolError(
                f'line {record.line_number}: control continued after API exit')
        if not saw_control:
            if command.ctrl_mode != 1:
                raise CanProtocolError(
                    f'line {record.line_number}: replay must start in API standby')
            saw_control = True
            standby_requested = True
        elif command.ctrl_mode == 1:
            if run_confirmed:
                raise CanProtocolError(
                    f'line {record.line_number}: API standby after API run')
            standby_requested = True
        elif command.ctrl_mode == 2:
            if not standby_confirmed:
                raise CanProtocolError(
                    f'line {record.line_number}: API run before standby feedback')
            run_requested = True
            if has_motion and not run_confirmed:
                raise CanProtocolError(
                    f'line {record.line_number}: motion before run feedback')
        elif command.ctrl_mode == 0:
            if not run_confirmed:
                raise CanProtocolError(
                    f'line {record.line_number}: API exit before run feedback')
            if has_motion:
                raise CanProtocolError(
                    f'line {record.line_number}: motion in API exit frame')
            exited_api = True

        if command.ctrl_mode != 2 and has_motion:
            raise CanProtocolError(
                f'line {record.line_number}: non-zero motion outside API run')

    if not saw_control:
        raise CanProtocolError('replay contains no control frame')
    if not standby_confirmed:
        raise CanProtocolError('replay lacks API standby feedback')
    if not run_requested or not run_confirmed:
        raise CanProtocolError('replay lacks API run feedback')


__all__ = [
    'CAN_BITRATE',
    'CAN_FRAME_DLC',
    'CONTROL_CAN_ID',
    'CONTROL_PERIOD_S',
    'CONTROL_TIMEOUT_S',
    'FEEDBACK_CAN_ID',
    'UNLOCK_MAGIC_KEY',
    'CanFrame',
    'CanProtocolError',
    'ControlCommand',
    'DecodedCanFrame',
    'FeedbackStatus',
    'ReplayRecord',
    'decode_control_frame',
    'decode_feedback_frame',
    'decode_frame',
    'parse_candump_line',
    'replay_candump',
    'validate_control_replay',
]
