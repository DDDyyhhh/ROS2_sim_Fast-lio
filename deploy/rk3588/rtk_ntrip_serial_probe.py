#!/usr/bin/env python3
"""Probe an NTRIP caster and UM982 through one serial-port owner.

The probe deliberately does not start ROS or any motion-control component. It
opens the receiver serial device once, writes RTCM received from NTRIP, and
reads NMEA from the same file descriptor. Credentials are read only from the
environment and are never printed.
"""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import math
import os
import select
import statistics
import socket
import termios
import time
from collections import Counter


QUALITY_NAMES = {
    0: "invalid",
    1: "single",
    2: "dgps",
    4: "rtk-fixed",
    5: "rtk-float",
}


def nmea_checksum(body: str) -> int:
    checksum = 0
    for byte in body.encode("ascii"):
        checksum ^= byte
    return checksum


def make_gga(latitude: float, longitude: float, height: float) -> bytes:
    def coordinate(value: float, latitude_value: bool) -> tuple[str, str]:
        absolute = abs(value)
        degrees = int(absolute)
        minutes = (absolute - degrees) * 60.0
        width = 2 if latitude_value else 3
        hemisphere = (
            ("N" if value >= 0 else "S")
            if latitude_value
            else ("E" if value >= 0 else "W")
        )
        return f"{degrees:0{width}d}{minutes:08.5f}", hemisphere

    now = dt.datetime.now(dt.timezone.utc)
    time_field = now.strftime("%H%M%S") + f".{now.microsecond // 10000:02d}"
    lat, lat_hemi = coordinate(latitude, True)
    lon, lon_hemi = coordinate(longitude, False)
    body = (
        f"GPGGA,{time_field},{lat},{lat_hemi},{lon},{lon_hemi},"
        f"1,19,0.8,{height:.4f},M,0.0,M,,"
    )
    return f"${body}*{nmea_checksum(body):02X}\r\n".encode("ascii")


def decimal_degrees(value: str, hemisphere: str) -> float | None:
    if not value or not hemisphere:
        return None
    try:
        degrees = int(float(value) // 100)
        minutes = float(value) - degrees * 100.0
        result = degrees + minutes / 60.0
        return -result if hemisphere in {"S", "W"} else result
    except ValueError:
        return None


def parse_gga(line: str) -> dict[str, float | int] | None:
    if not line.startswith("$") or "*" not in line:
        return None
    payload, supplied = line[1:].split("*", 1)
    try:
        if nmea_checksum(payload) != int(supplied[:2], 16):
            return None
    except (ValueError, TypeError):
        return None
    fields = payload.split(",")
    if not fields[0].endswith("GGA") or len(fields) < 10:
        return None
    try:
        quality = int(fields[6])
        satellites = int(fields[7])
        hdop = float(fields[8])
        altitude = float(fields[9])
    except (ValueError, IndexError):
        return None
    latitude = decimal_degrees(fields[2], fields[3])
    longitude = decimal_degrees(fields[4], fields[5])
    if latitude is None or longitude is None:
        return None
    return {
        "quality": quality,
        "satellites": satellites,
        "hdop": hdop,
        "altitude": altitude,
        "latitude": latitude,
        "longitude": longitude,
    }


def configure_serial(path: str, baud: int) -> int:
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


def parse_ntrip_response(data: bytes) -> bytes | None:
    """Return correction bytes, or None while the response is incomplete."""
    legacy_ok = b"ICY 200 OK\r\n"
    if data.startswith(legacy_ok):
        return data[len(legacy_ok) :]

    first_line = data.split(b"\r\n", 1)[0]
    if first_line.startswith((b"ERROR", b"SOURCETABLE")):
        message = first_line.decode("ascii", "replace")
        raise RuntimeError(f"NTRIP caster rejected request: {message}")

    separator = b"\r\n\r\n"
    if separator not in data:
        return None
    header, body = data.split(separator, 1)
    status_line = header.split(b"\r\n", 1)[0]
    status = status_line.decode("ascii", "replace")
    if b" 200 " not in status_line:
        raise RuntimeError(f"NTRIP request rejected: {status}")
    return body


def connect_ntrip(
    host: str,
    port: int,
    mountpoint: str,
    username: str,
    password: str,
) -> tuple[socket.socket, bytes]:
    connection = socket.create_connection((host, port), timeout=10.0)
    credentials = base64.b64encode(f"{username}:{password}".encode()).decode()
    request = (
        f"GET /{mountpoint} HTTP/1.0\r\n"
        "User-Agent: NTRIP mower-rk3588-probe/1.0\r\n"
        "Accept: */*\r\n"
        "Connection: close\r\n"
        f"Authorization: Basic {credentials}\r\n\r\n"
    ).encode("ascii")
    connection.sendall(request)
    response = bytearray()
    while True:
        chunk = connection.recv(4096)
        if not chunk:
            if response:
                first_line = response.split(b"\r\n", 1)[0].decode(
                    "ascii", "replace"
                )
                raise RuntimeError(
                    f"NTRIP caster closed with incomplete response: {first_line}"
                )
            raise RuntimeError("NTRIP caster closed before sending a response")
        response.extend(chunk)
        if len(response) > 64 * 1024:
            raise RuntimeError("NTRIP response header is too large")
        body = parse_ntrip_response(bytes(response))
        if body is not None:
            break
    connection.setblocking(False)
    return connection, body


def write_all(fd: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        try:
            written = os.write(fd, view)
            view = view[written:]
        except BlockingIOError:
            select.select([], [fd], [], 0.5)


def position_spread(samples: list[dict[str, float | int]]) -> tuple[float, float, float]:
    if not samples:
        return 0.0, 0.0, 0.0
    reference_lat = float(samples[0]["latitude"])
    reference_lon = float(samples[0]["longitude"])
    east_north = []
    for sample in samples:
        north = (float(sample["latitude"]) - reference_lat) * 111_320.0
        east = (
            (float(sample["longitude"]) - reference_lon)
            * 111_320.0
            * math.cos(math.radians(reference_lat))
        )
        east_north.append((east, north))
    max_radius = max(math.hypot(east, north) for east, north in east_north)
    east_values = [east for east, _ in east_north]
    north_values = [north for _, north in east_north]
    return (
        max(east_values) - min(east_values),
        max(north_values) - min(north_values),
        max_radius,
    )


def _east_north(samples: list[dict[str, float | int]]) -> list[tuple[float, float]]:
    if not samples:
        return []
    reference_lat = float(samples[0]["latitude"])
    reference_lon = float(samples[0]["longitude"])
    return [
        (
            (float(sample["longitude"]) - reference_lon)
            * 111_320.0
            * math.cos(math.radians(reference_lat)),
            (float(sample["latitude"]) - reference_lat) * 111_320.0,
        )
        for sample in samples
    ]


def fixed_only_statistics(
    samples: list[dict[str, float | int]], max_gap_s: float = 2.5
) -> dict[str, float | int | None]:
    """Return scatter and continuity metrics for quality=4 samples only."""
    fixed = [sample for sample in samples if int(sample["quality"]) == 4]
    east_north = _east_north(fixed)
    if east_north:
        east_values = [east for east, _ in east_north]
        north_values = [north for _, north in east_north]
        east_mean = statistics.fmean(east_values)
        north_mean = statistics.fmean(north_values)
        radial = [
            math.hypot(east - east_mean, north - north_mean)
            for east, north in east_north
        ]
        std_east = statistics.pstdev(east_values)
        std_north = statistics.pstdev(north_values)
        std_radial = statistics.pstdev(radial)
        fixed_spread = (
            max(east_values) - min(east_values),
            max(north_values) - min(north_values),
            max(math.hypot(east, north) for east, north in east_north),
        )
    else:
        std_east = std_north = std_radial = 0.0
        fixed_spread = (0.0, 0.0, 0.0)

    first_fixed = next(
        (float(sample["elapsed_s"]) for sample in samples if int(sample["quality"]) == 4),
        None,
    )
    longest_duration = 0.0
    current_start = None
    previous_fixed = None
    for sample in samples:
        if int(sample["quality"]) != 4:
            current_start = None
            previous_fixed = None
            continue
        elapsed = float(sample.get("elapsed_s", 0.0))
        if previous_fixed is None or elapsed - previous_fixed > max_gap_s:
            current_start = elapsed
        previous_fixed = elapsed
        if current_start is not None:
            longest_duration = max(longest_duration, elapsed - current_start)

    return {
        "count": len(fixed),
        "east_span": fixed_spread[0],
        "north_span": fixed_spread[1],
        "max_from_first": fixed_spread[2],
        "std_east": std_east,
        "std_north": std_north,
        "std_radial": std_radial,
        "first_fixed_s": first_fixed,
        "longest_continuous_s": longest_duration,
    }


def run(args: argparse.Namespace) -> int:
    username = os.environ.get("CORS_USER")
    password = os.environ.get("CORS_PASS")
    if not username or not password:
        raise RuntimeError("set CORS_USER and CORS_PASS in the environment")

    print(
        f"connecting caster={args.host}:{args.port}/{args.mountpoint} "
        f"serial={args.device} baud={args.baud} duration={args.duration}s"
    )
    serial_fd = configure_serial(args.device, args.baud)
    connection = None
    samples: list[dict[str, float | int]] = []
    quality_counts: Counter[int] = Counter()
    nmea_lines = 0
    invalid_nmea = 0
    rtcm_bytes = 0
    serial_buffer = bytearray()
    started = time.monotonic()
    stop_reason = "duration complete"
    try:
        connection, initial_body = connect_ntrip(
            args.host, args.port, args.mountpoint, username, password
        )
        print("NTRIP status=200; single serial owner active")
        if initial_body:
            write_all(serial_fd, initial_body)
            rtcm_bytes += len(initial_body)
        next_gga = 0.0
        deadline = started + args.duration
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_gga:
                try:
                    connection.sendall(
                        make_gga(args.latitude, args.longitude, args.height)
                    )
                except OSError as error:
                    stop_reason = f"NTRIP send failed: {type(error).__name__}"
                    break
                next_gga = now + 1.0
            readable, _, _ = select.select([serial_fd, connection], [], [], 0.2)
            if connection in readable:
                try:
                    correction = connection.recv(8192)
                except OSError as error:
                    stop_reason = f"NTRIP receive failed: {type(error).__name__}"
                    break
                if not correction:
                    stop_reason = "NTRIP caster closed the correction stream"
                    break
                write_all(serial_fd, correction)
                rtcm_bytes += len(correction)
            if serial_fd in readable:
                received = os.read(serial_fd, 8192)
                if not received:
                    raise RuntimeError("UM982 serial device returned EOF")
                serial_buffer.extend(received)
                while b"\n" in serial_buffer:
                    raw_line, _, remainder = serial_buffer.partition(b"\n")
                    serial_buffer = bytearray(remainder)
                    marker = raw_line.find(b"$")
                    if marker < 0:
                        continue
                    line = raw_line[marker:].rstrip(b"\r").decode("ascii", "replace")
                    nmea_lines += 1
                    sample = parse_gga(line)
                    if sample is None:
                        if "GGA" in line:
                            invalid_nmea += 1
                        continue
                    samples.append(sample)
                    sample["elapsed_s"] = time.monotonic() - started
                    quality = int(sample["quality"])
                    quality_counts[quality] += 1
                    print(
                        f"GGA quality={quality}({QUALITY_NAMES.get(quality, 'other')}) "
                        f"sat={sample['satellites']} hdop={sample['hdop']:.2f} "
                        f"lat={sample['latitude']:.8f} "
                        f"lon={sample['longitude']:.8f}"
                    )
    finally:
        if connection is not None:
            connection.close()
        os.close(serial_fd)

    elapsed = time.monotonic() - started
    east_span, north_span, max_radius = position_spread(samples)
    print(f"ELAPSED_S={elapsed:.1f}")
    print(f"STOP_REASON={stop_reason}")
    print(f"RTCM_BYTES={rtcm_bytes}")
    print(f"NMEA_LINES={nmea_lines} VALID_GGA={len(samples)} INVALID_GGA={invalid_nmea}")
    print(f"GGA_QUALITY_COUNTS={dict(sorted(quality_counts.items()))}")
    print(
        f"POSITION_SPREAD_M=east:{east_span:.3f},north:{north_span:.3f},"
        f"max_from_first:{max_radius:.3f}"
    )
    fixed_stats = fixed_only_statistics(samples)
    first_fixed = fixed_stats["first_fixed_s"]
    first_fixed_text = "none" if first_fixed is None else f"{first_fixed:.1f}"
    print(f"FIXED_ONLY_COUNT={fixed_stats['count']}")
    print(f"FIRST_FIXED_S={first_fixed_text}")
    print(
        "FIXED_ONLY_SPREAD_M="
        f"east:{fixed_stats['east_span']:.3f},"
        f"north:{fixed_stats['north_span']:.3f},"
        f"max_from_first:{fixed_stats['max_from_first']:.3f}"
    )
    print(
        "FIXED_ONLY_STD_M="
        f"east:{fixed_stats['std_east']:.3f},"
        f"north:{fixed_stats['std_north']:.3f},"
        f"radial:{fixed_stats['std_radial']:.3f}"
    )
    print(f"LONGEST_CONTINUOUS_FIXED_S={fixed_stats['longest_continuous_s']:.1f}")
    if rtcm_bytes > 0 and samples and stop_reason == "duration complete":
        return 0
    if rtcm_bytes > 0 and samples:
        return 3
    return 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="114.111.30.20")
    parser.add_argument("--port", type=int, default=8002)
    parser.add_argument("--mountpoint", default="RTCM33GRCEJ")
    parser.add_argument("--latitude", type=float, default=23.39633658)
    parser.add_argument("--longitude", type=float, default=113.16141518)
    parser.add_argument("--height", type=float, default=13.9)
    parser.add_argument("--duration", type=int, default=60)
    return parser.parse_args()


if __name__ == "__main__":
    try:
        raise SystemExit(run(parse_args()))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"ERROR: {error}")
        raise SystemExit(1)
