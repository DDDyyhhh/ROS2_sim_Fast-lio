"""Dependency-free NMEA, NTRIP and RTK health primitives."""

from __future__ import annotations

import base64
import datetime as dt
import json
import math
import os
import select
import socket
import termios
import time
from threading import Lock


QUALITY_NAMES = {
    0: "NO_FIX",
    1: "SINGLE",
    2: "DGPS",
    4: "RTK_FIXED",
    5: "RTK_FLOAT",
}


class NtripProtocolError(RuntimeError):
    """The caster returned a response that cannot provide RTCM corrections."""


def nmea_checksum(body: str) -> int:
    checksum = 0
    for byte in body.encode("ascii"):
        checksum ^= byte
    return checksum


def _decimal_degrees(value: str, hemisphere: str) -> float | None:
    if not value or hemisphere not in {"N", "S", "E", "W"}:
        return None
    try:
        degrees = int(float(value) // 100)
        minutes = float(value) - degrees * 100.0
        if not 0.0 <= minutes < 60.0:
            return None
        result = degrees + minutes / 60.0
        return -result if hemisphere in {"S", "W"} else result
    except (TypeError, ValueError):
        return None


def parse_gga(line: str) -> dict[str, float | int | None] | None:
    """Parse a checksummed GGA sentence, including no-fix GGA samples."""
    if not line.startswith("$") or "*" not in line:
        return None
    payload, supplied = line[1:].split("*", 1)
    try:
        if nmea_checksum(payload) != int(supplied[:2], 16):
            return None
    except (TypeError, ValueError):
        return None
    fields = payload.split(",")
    if not fields[0].endswith("GGA") or len(fields) < 10:
        return None
    try:
        quality = int(fields[6] or 0)
        satellites = int(fields[7] or 0)
        hdop = float(fields[8]) if fields[8] else math.nan
        altitude = float(fields[9]) if fields[9] else math.nan
    except (TypeError, ValueError, IndexError):
        return None
    latitude = _decimal_degrees(fields[2], fields[3])
    longitude = _decimal_degrees(fields[4], fields[5])
    return {
        "quality": quality,
        "satellites": satellites,
        "hdop": hdop,
        "altitude": altitude,
        "latitude": latitude,
        "longitude": longitude,
    }


def _coordinate(value: float, latitude: bool) -> tuple[str, str]:
    absolute = abs(value)
    degrees = int(absolute)
    minutes = (absolute - degrees) * 60.0
    width = 2 if latitude else 3
    hemisphere = (
        ("N" if value >= 0 else "S")
        if latitude
        else ("E" if value >= 0 else "W")
    )
    return f"{degrees:0{width}d}{minutes:08.5f}", hemisphere


def make_gga(sample: dict[str, float | int | None]) -> bytes:
    """Build the position report sent to the caster."""
    latitude = sample.get("latitude")
    longitude = sample.get("longitude")
    if latitude is None or longitude is None:
        raise ValueError("a valid position is required for NTRIP GGA")
    lat, lat_hemi = _coordinate(float(latitude), True)
    lon, lon_hemi = _coordinate(float(longitude), False)
    now = dt.datetime.now(dt.timezone.utc)
    time_field = now.strftime("%H%M%S") + f".{now.microsecond // 10000:02d}"
    quality = max(1, int(sample.get("quality") or 1))
    satellites = max(0, int(sample.get("satellites") or 0))
    hdop_value = float(sample.get("hdop") or 0.0)
    altitude_value = float(sample.get("altitude") or 0.0)
    hdop = hdop_value if math.isfinite(hdop_value) else 0.0
    altitude = altitude_value if math.isfinite(altitude_value) else 0.0
    body = (
        f"GPGGA,{time_field},{lat},{lat_hemi},{lon},{lon_hemi},"
        f"{quality},{satellites},{hdop:.2f},{altitude:.4f},M,0.0,M,,"
    )
    return f"${body}*{nmea_checksum(body):02X}\r\n".encode("ascii")


def parse_ntrip_response(data: bytes) -> bytes | None:
    """Return initial RTCM bytes, or None until the response is complete."""
    legacy_ok = b"ICY 200 OK\r\n"
    if data.startswith(legacy_ok):
        return data[len(legacy_ok) :]

    first_line = data.split(b"\r\n", 1)[0]
    if first_line.startswith((b"ERROR", b"SOURCETABLE")):
        raise NtripProtocolError(
            f"NTRIP caster rejected request: {first_line.decode('ascii', 'replace')}"
        )
    separator = b"\r\n\r\n"
    if separator not in data:
        return None
    header, body = data.split(separator, 1)
    status_line = header.split(b"\r\n", 1)[0]
    if b" 200 " not in status_line:
        raise NtripProtocolError(
            f"NTRIP request rejected: {status_line.decode('ascii', 'replace')}"
        )
    return body


def open_serial(path: str, baud: int) -> int:
    speed = getattr(termios, f"B{baud}", None)
    if speed is None:
        raise ValueError(f"unsupported baud rate: {baud}")
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        settings = termios.tcgetattr(fd)
        settings[0] = 0
        settings[1] = 0
        settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        settings[2] &= ~getattr(termios, "CRTSCTS", 0)
        settings[3] = 0
        settings[4] = speed
        settings[5] = speed
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 1
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        termios.tcflush(fd, termios.TCIFLUSH)
        return fd
    except Exception:
        os.close(fd)
        raise


def write_all(fd: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        try:
            written = os.write(fd, view)
            view = view[written:]
        except BlockingIOError:
            select.select([], [fd], [], 0.5)


def connect_ntrip(
    host: str,
    port: int,
    mountpoint: str,
    username: str,
    password: str,
    timeout_s: float = 10.0,
) -> tuple[socket.socket, bytes]:
    """Perform one NTRIP handshake without logging credentials."""
    connection = socket.create_connection((host, port), timeout=timeout_s)
    credentials = base64.b64encode(f"{username}:{password}".encode()).decode()
    request = (
        f"GET /{mountpoint} HTTP/1.0\r\n"
        "User-Agent: NTRIP mower-rk3588/1.0\r\n"
        "Accept: */*\r\n"
        "Connection: close\r\n"
        f"Authorization: Basic {credentials}\r\n\r\n"
    ).encode("ascii")
    try:
        connection.sendall(request)
        response = bytearray()
        connection.settimeout(timeout_s)
        while True:
            chunk = connection.recv(4096)
            if not chunk:
                first_line = response.split(b"\r\n", 1)[0] if response else b""
                if first_line:
                    raise NtripProtocolError(
                        "NTRIP caster closed with incomplete response: "
                        f"{first_line.decode('ascii', 'replace')}"
                    )
                raise NtripProtocolError("NTRIP caster closed before response")
            response.extend(chunk)
            if len(response) > 64 * 1024:
                raise NtripProtocolError("NTRIP response header is too large")
            body = parse_ntrip_response(bytes(response))
            if body is not None:
                break
        connection.setblocking(False)
        return connection, body
    except Exception:
        connection.close()
        raise


class RtkHealth:
    """Thread-safe health state used by the ROS node and its status contract."""

    def __init__(
        self,
        stale_after_s: float = 3.0,
        correction_timeout_s: float = 15.0,
    ) -> None:
        self._lock = Lock()
        self.stale_after_s = stale_after_s
        self.correction_timeout_s = correction_timeout_s
        self.serial_state = "DISCONNECTED"
        self.ntrip_state = "DISABLED"
        self.last_error: str | None = None
        self.current: dict[str, float | int | None] | None = None
        self.last_gga_at: float | None = None
        self.nmea_lines = 0
        self.valid_gga = 0
        self.invalid_gga = 0
        self.rtcm_bytes = 0
        self.ntrip_connected_at: float | None = None
        self.last_rtcm_at: float | None = None

    def observe_nmea_line(self) -> None:
        with self._lock:
            self.nmea_lines += 1

    def observe_invalid_gga(self) -> None:
        with self._lock:
            self.invalid_gga += 1

    def observe_gga(self, sample: dict[str, float | int | None], now: float) -> None:
        with self._lock:
            self.current = dict(sample)
            self.last_gga_at = now
            self.valid_gga += 1

    def set_serial(self, connected: bool, error: str | None = None) -> None:
        with self._lock:
            self.serial_state = "CONNECTED" if connected else "DISCONNECTED"
            if not connected:
                self.current = None
                self.last_gga_at = None
            if error:
                self.last_error = error

    def set_ntrip(
        self,
        state: str,
        error: str | None = None,
        now: float | None = None,
    ) -> None:
        with self._lock:
            self.ntrip_state = state
            self.last_error = error
            self.last_rtcm_at = None
            self.ntrip_connected_at = (
                time.monotonic() if now is None else now
            ) if state == "CONNECTED" else None

    def add_rtcm(self, count: int, now: float | None = None) -> None:
        with self._lock:
            self.rtcm_bytes += count
            if count > 0:
                self.last_rtcm_at = time.monotonic() if now is None else now

    def corrections_stale(self, now: float) -> bool:
        with self._lock:
            if self.ntrip_state != "CONNECTED":
                return False
            reference = self.last_rtcm_at or self.ntrip_connected_at
            return (
                reference is not None
                and now - reference > self.correction_timeout_s
            )

    def latest_position(self, now: float | None = None) -> dict[str, float | int | None] | None:
        with self._lock:
            if self.current is None:
                return None
            if self.current.get("latitude") is None:
                return None
            if now is not None and self.last_gga_at is not None:
                if now - self.last_gga_at > self.stale_after_s:
                    return None
            return dict(self.current)

    def snapshot(self, now: float) -> dict[str, object]:
        with self._lock:
            current = dict(self.current) if self.current else {}
            age = None if self.last_gga_at is None else max(0.0, now - self.last_gga_at)
            fresh = age is not None and age <= self.stale_after_s
            quality = int(current.get("quality") or 0)
            solution = QUALITY_NAMES.get(quality, "OTHER") if fresh else "NO_FIX"
            state = solution if self.serial_state == "CONNECTED" else "SERIAL_UNAVAILABLE"
            hdop = current.get("hdop")
            if isinstance(hdop, float) and not math.isfinite(hdop):
                hdop = None
            rtcm_reference = self.last_rtcm_at or self.ntrip_connected_at
            rtcm_age = (
                None if rtcm_reference is None else max(0.0, now - rtcm_reference)
            )
            corrections_fresh = (
                self.ntrip_state == "CONNECTED"
                and self.last_rtcm_at is not None
                and rtcm_age is not None
                and rtcm_age <= self.correction_timeout_s
            )
            trusted = (
                state == "RTK_FIXED"
                and self.ntrip_state == "CONNECTED"
                and fresh
                and corrections_fresh
            )
            return {
                "state": state,
                "solution": solution,
                "quality": quality,
                "satellites": int(current.get("satellites") or 0),
                "hdop": hdop,
                "ntrip": self.ntrip_state,
                "corrections_fresh": corrections_fresh,
                "global_position_trusted": trusted,
                "serial": self.serial_state,
                "rtcm_bytes": self.rtcm_bytes,
                "nmea_lines": self.nmea_lines,
                "valid_gga": self.valid_gga,
                "invalid_gga": self.invalid_gga,
                "last_gga_age_s": age,
                "last_rtcm_age_s": rtcm_age,
                "last_error": self.last_error,
            }

    def snapshot_json(self, now: float) -> str:
        return json.dumps(self.snapshot(now), ensure_ascii=False, sort_keys=True)
