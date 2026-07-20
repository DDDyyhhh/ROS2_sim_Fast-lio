"""Single-owner ROS 2 node for UM982 NMEA plus NTRIP RTCM."""

from __future__ import annotations

import math
import os
import select
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from .rtk_ntrip_core import (
    NtripProtocolError,
    RtkHealth,
    connect_ntrip,
    make_gga,
    open_serial,
    parse_gga,
    write_all,
)


class RtkNtripNode(Node):
    """Own the UM982 fd and caster socket in one worker thread."""

    def __init__(self) -> None:
        super().__init__("um982_rtk_ntrip")
        self._device = self.declare_parameter("device", "/dev/um982").value
        self._baud = int(self.declare_parameter("baud", 115200).value)
        self._frame_id = str(self.declare_parameter("frame_id", "rtk_link").value)
        self._caster_host = str(
            self.declare_parameter("caster_host", "114.111.30.20").value
        )
        self._caster_port = int(self.declare_parameter("caster_port", 8002).value)
        self._mountpoint = str(
            self.declare_parameter("mountpoint", "RTCM33GRCEJ").value
        )
        self._user_env = str(self.declare_parameter("user_env", "CORS_USER").value)
        self._pass_env = str(self.declare_parameter("pass_env", "CORS_PASS").value)
        self._gga_interval = float(self.declare_parameter("gga_interval", 10.0).value)
        self._reconnect_interval = float(
            self.declare_parameter("reconnect_interval", 5.0).value
        )
        self._serial_retry_interval = float(
            self.declare_parameter("serial_retry_interval", 2.0).value
        )
        self._ntrip_timeout = float(
            self.declare_parameter("ntrip_timeout", 10.0).value
        )
        self._nmea_timeout = float(
            self.declare_parameter("nmea_timeout", 3.0).value
        )
        self._correction_timeout = float(
            self.declare_parameter("correction_timeout", 15.0).value
        )

        self._fix_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self._nmea_pub = self.create_publisher(String, "/rtk/nmea", 20)
        self._status_pub = self.create_publisher(String, "/rtk/status", 10)
        self._health = RtkHealth(
            stale_after_s=self._nmea_timeout,
            correction_timeout_s=self._correction_timeout,
        )
        self._serial_fd: int | None = None
        self._caster = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._io_loop, name="rtk-io", daemon=True)
        self._last_status_at = 0.0
        self._last_log_state: tuple[str, str, str] | None = None
        self._thread.start()

    def destroy_node(self) -> bool:
        self._stop.set()
        self._close_caster()
        self._close_serial()
        self._thread.join(timeout=2.0)
        return super().destroy_node()

    def _close_serial(self) -> None:
        if self._serial_fd is not None:
            try:
                os.close(self._serial_fd)
            except OSError:
                pass
            self._serial_fd = None
        self._health.set_serial(False)

    def _close_caster(self) -> None:
        if self._caster is not None:
            try:
                self._caster.close()
            except OSError:
                pass
            self._caster = None

    def _publish_status(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last_status_at < 1.0:
            return
        message = String()
        message.data = self._health.snapshot_json(now)
        self._status_pub.publish(message)
        self._last_status_at = now
        snapshot = self._health.snapshot(now)
        state_key = (
            str(snapshot["state"]),
            str(snapshot["ntrip"]),
            str(snapshot["serial"]),
        )
        if state_key != self._last_log_state:
            self.get_logger().info(
                f"RTK state={state_key[0]} ntrip={state_key[1]} serial={state_key[2]}"
            )
            self._last_log_state = state_key

    def _publish_sample(self, sample: dict[str, Any]) -> None:
        raw = String()
        raw.data = str(sample.pop("_line", ""))
        self._nmea_pub.publish(raw)

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame_id
        quality = int(sample.get("quality") or 0)
        fix.status.status = (
            NavSatStatus.STATUS_FIX if quality > 0 else NavSatStatus.STATUS_NO_FIX
        )
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = float(sample["latitude"]) if sample["latitude"] is not None else math.nan
        fix.longitude = (
            float(sample["longitude"]) if sample["longitude"] is not None else math.nan
        )
        altitude = sample.get("altitude")
        fix.altitude = float(altitude) if altitude is not None else math.nan
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._fix_pub.publish(fix)

    def _handle_serial_bytes(self, data: bytes, serial_buffer: bytearray) -> None:
        serial_buffer.extend(data)
        while b"\n" in serial_buffer:
            raw_line, _, remainder = serial_buffer.partition(b"\n")
            serial_buffer[:] = remainder
            marker = raw_line.find(b"$")
            if marker < 0:
                continue
            line = raw_line[marker:].rstrip(b"\r").decode("ascii", "replace")
            self._health.observe_nmea_line()
            sample = parse_gga(line)
            if sample is None:
                if "GGA" in line:
                    self._health.observe_invalid_gga()
                raw = String()
                raw.data = line
                self._nmea_pub.publish(raw)
                continue
            sample["_line"] = line
            self._health.observe_gga(sample, time.monotonic())
            self._publish_sample(sample)

    def _connect_caster(self) -> None:
        username = os.environ.get(self._user_env, "")
        password = os.environ.get(self._pass_env, "")
        if not username or not password:
            self._health.set_ntrip("DISABLED", f"missing {self._user_env}/{self._pass_env}")
            return
        position = self._health.latest_position(time.monotonic())
        if position is None:
            self._health.set_ntrip("DISCONNECTED", "waiting for valid GGA position")
            return
        self._health.set_ntrip("CONNECTING")
        caster, initial_body = connect_ntrip(
            self._caster_host,
            self._caster_port,
            self._mountpoint,
            username,
            password,
            timeout_s=self._ntrip_timeout,
        )
        self._caster = caster
        self._health.set_ntrip("CONNECTED", now=time.monotonic())
        if initial_body:
            write_all(self._serial_fd, initial_body)
            self._health.add_rtcm(len(initial_body), now=time.monotonic())

    def _send_gga(self) -> None:
        position = self._health.latest_position(time.monotonic())
        if self._caster is None or position is None:
            return
        self._caster.sendall(make_gga(position))

    def _handle_caster(self) -> None:
        correction = self._caster.recv(8192)
        if not correction:
            raise NtripProtocolError("NTRIP caster closed correction stream")
        write_all(self._serial_fd, correction)
        self._health.add_rtcm(len(correction), now=time.monotonic())

    def _io_loop(self) -> None:
        serial_buffer = bytearray()
        next_serial_retry = 0.0
        next_caster_retry = 0.0
        next_gga = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if self._serial_fd is None and now >= next_serial_retry:
                try:
                    self._serial_fd = open_serial(self._device, self._baud)
                    self._health.set_serial(True)
                    serial_buffer.clear()
                    next_caster_retry = now
                except (OSError, ValueError) as error:
                    self._health.set_serial(False, f"serial open failed: {type(error).__name__}")
                    next_serial_retry = now + self._serial_retry_interval

            if self._serial_fd is None:
                self._publish_status()
                self._stop.wait(0.2)
                continue

            if self._caster is None and now >= next_caster_retry:
                try:
                    self._connect_caster()
                    next_gga = now
                    next_caster_retry = now + self._reconnect_interval
                except (OSError, NtripProtocolError, ValueError) as error:
                    self._close_caster()
                    self._health.set_ntrip("DISCONNECTED", str(error))
                    next_caster_retry = now + self._reconnect_interval

            readable = [self._serial_fd]
            if self._caster is not None:
                readable.append(self._caster)
            try:
                ready, _, _ = select.select(readable, [], [], 0.2)
            except OSError as error:
                self._close_serial()
                self._health.set_serial(False, f"serial select failed: {type(error).__name__}")
                next_serial_retry = time.monotonic() + self._serial_retry_interval
                continue

            if self._caster is not None and self._caster in ready:
                try:
                    self._handle_caster()
                except (OSError, NtripProtocolError) as error:
                    self._close_caster()
                    self._health.set_ntrip("DISCONNECTED", str(error))
                    next_caster_retry = time.monotonic() + self._reconnect_interval

            if self._serial_fd is not None and self._serial_fd in ready:
                try:
                    data = os.read(self._serial_fd, 8192)
                    if not data:
                        raise OSError("UM982 serial EOF")
                    self._handle_serial_bytes(data, serial_buffer)
                except (OSError, ValueError) as error:
                    self._close_serial()
                    self._close_caster()
                    self._health.set_ntrip("DISCONNECTED", str(error))
                    next_serial_retry = time.monotonic() + self._serial_retry_interval

            if self._caster is not None and time.monotonic() >= next_gga:
                try:
                    self._send_gga()
                    next_gga = time.monotonic() + self._gga_interval
                except OSError as error:
                    self._close_caster()
                    self._health.set_ntrip("DISCONNECTED", str(error))
                    next_caster_retry = time.monotonic() + self._reconnect_interval

            if self._caster is not None and self._health.corrections_stale(
                time.monotonic()
            ):
                self._close_caster()
                self._health.set_ntrip("DISCONNECTED", "RTCM stream stale")
                next_caster_retry = time.monotonic() + self._reconnect_interval

            self._publish_status()
        self._close_caster()
        self._close_serial()
        self._publish_status(force=True)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RtkNtripNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
