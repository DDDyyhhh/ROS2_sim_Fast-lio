#!/usr/bin/env python3
"""Offline RTK health, protocol and failure-mode regressions."""

import importlib.util
import json
import os
import pty
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from mower_hardware.rtk_ntrip_core import (  # noqa: E402
    NtripProtocolError,
    RtkHealth,
    nmea_checksum,
    parse_gga,
    parse_ntrip_response,
    open_serial,
)


PROBE_PATH = PACKAGE_ROOT.parents[1] / 'deploy' / 'rk3588' / (
    'rtk_ntrip_serial_probe.py'
)
PROBE_SPEC = importlib.util.spec_from_file_location('rtk_probe', PROBE_PATH)
PROBE = importlib.util.module_from_spec(PROBE_SPEC)
assert PROBE_SPEC.loader is not None
PROBE_SPEC.loader.exec_module(PROBE)


def sentence(body):
    return f'${body}*{nmea_checksum(body):02X}'


def gga(quality, latitude='2323.77990', longitude='11309.68290', seconds='000000'):
    return sentence(
        f'GNGGA,{seconds}.00,{latitude},N,{longitude},E,{quality},20,0.60,30.0,M,0.0,M,,'
    )


class RtkProtocolTest(unittest.TestCase):
    def test_serial_open_configures_a_port(self):
        master_fd, slave_fd = pty.openpty()
        try:
            configured_fd = open_serial(os.ttyname(slave_fd), 115200)
            os.close(configured_fd)
        finally:
            os.close(master_fd)
            os.close(slave_fd)

    def test_parse_gga_accepts_fixed_and_no_fix(self):
        fixed = parse_gga(gga(4))
        no_fix = parse_gga(gga(0, latitude='', longitude=''))

        self.assertEqual(fixed['quality'], 4)
        self.assertAlmostEqual(fixed['latitude'], 23.3963316667, places=7)
        self.assertIsNone(no_fix['latitude'])
        self.assertEqual(no_fix['quality'], 0)

    def test_bad_nmea_checksum_is_invalid(self):
        line = gga(4)
        bad = line[:-2] + ('00' if line[-2:] != '00' else 'FF')
        self.assertIsNone(parse_gga(bad))

    def test_ntrip_old_and_http_handshakes(self):
        self.assertEqual(parse_ntrip_response(b'ICY 200 OK\r\n\xd3\x00'), b'\xd3\x00')
        self.assertEqual(
            parse_ntrip_response(b'HTTP/1.0 200 OK\r\nServer: test\r\n\r\n\xd3'),
            b'\xd3',
        )
        with self.assertRaises(NtripProtocolError):
            parse_ntrip_response(b'ERROR - Bad Password\r\n')


class RtkHealthFailureModeTest(unittest.TestCase):
    def setUp(self):
        self.health = RtkHealth(stale_after_s=3.0)
        self.health.set_serial(True)

    def observe(self, quality, now):
        self.health.observe_gga(
            {
                'quality': quality,
                'satellites': 20,
                'hdop': 0.6,
                'altitude': 30.0,
                'latitude': 23.3963317,
                'longitude': 113.1613817,
            },
            now,
        )

    def test_caster_disconnect_revokes_global_trust(self):
        self.health.set_ntrip('CONNECTED', now=0.0)
        self.health.add_rtcm(1, now=0.0)
        self.observe(4, 0.0)
        self.assertTrue(self.health.snapshot(1.0)['global_position_trusted'])

        self.health.set_ntrip('DISCONNECTED', 'caster closed')
        snapshot = self.health.snapshot(1.2)
        self.assertEqual(snapshot['solution'], 'RTK_FIXED')
        self.assertFalse(snapshot['global_position_trusted'])
        self.assertEqual(snapshot['ntrip'], 'DISCONNECTED')

    def test_degrade_and_recover_solution_are_visible(self):
        self.health.set_ntrip('CONNECTED', now=0.0)
        self.health.add_rtcm(1, now=0.0)
        self.observe(4, 0.0)
        self.assertEqual(self.health.snapshot(0.5)['state'], 'RTK_FIXED')

        self.observe(5, 1.0)
        self.assertEqual(self.health.snapshot(1.5)['state'], 'RTK_FLOAT')
        self.assertFalse(self.health.snapshot(1.5)['global_position_trusted'])

        self.observe(4, 2.0)
        self.assertTrue(self.health.snapshot(2.5)['global_position_trusted'])

    def test_serial_unplug_and_reenumeration_are_fail_closed_then_recover(self):
        self.health.set_ntrip('CONNECTED', now=0.0)
        self.health.add_rtcm(1, now=0.0)
        self.observe(4, 0.0)
        self.health.set_serial(False, 'serial EOF')
        unplugged = self.health.snapshot(0.5)
        self.assertEqual(unplugged['state'], 'SERIAL_UNAVAILABLE')
        self.assertFalse(unplugged['global_position_trusted'])

        self.health.set_serial(True)
        self.health.set_ntrip('CONNECTED', now=1.0)
        self.health.add_rtcm(1, now=1.0)
        self.observe(4, 1.0)
        self.assertTrue(self.health.snapshot(1.5)['global_position_trusted'])

    def test_stale_corrections_revoke_trust_before_socket_failure(self):
        health = RtkHealth(stale_after_s=3.0, correction_timeout_s=2.0)
        health.set_serial(True)
        health.set_ntrip('CONNECTED', now=0.0)
        health.add_rtcm(1, now=0.0)
        health.observe_gga(
            {
                'quality': 4,
                'satellites': 20,
                'hdop': 0.6,
                'altitude': 30.0,
                'latitude': 23.3963317,
                'longitude': 113.1613817,
            },
            0.0,
        )

        snapshot = health.snapshot(2.5)

        self.assertFalse(snapshot['corrections_fresh'])
        self.assertFalse(snapshot['global_position_trusted'])
        self.assertTrue(health.corrections_stale(2.5))

    def test_status_json_is_valid_when_gga_has_no_hdop(self):
        self.observe(0, 0.0)
        self.health.current['hdop'] = float('nan')

        payload = json.loads(self.health.snapshot_json(0.5))

        self.assertIsNone(payload['hdop'])

    def test_restart_starts_without_stale_fix_or_bytes(self):
        self.health.set_ntrip('CONNECTED', now=0.0)
        self.observe(4, 0.0)
        self.health.add_rtcm(123)

        restarted = RtkHealth(stale_after_s=3.0)
        restarted.set_serial(True)
        snapshot = restarted.snapshot(0.1)
        self.assertEqual(snapshot['state'], 'NO_FIX')
        self.assertFalse(snapshot['global_position_trusted'])
        self.assertEqual(snapshot['rtcm_bytes'], 0)


class FixedOnlyStatisticsTest(unittest.TestCase):
    def test_fixed_only_metrics_exclude_float_and_single_samples(self):
        samples = [
            {'quality': 1, 'latitude': 23.39630, 'longitude': 113.16130,
             'elapsed_s': 0.0},
            {'quality': 4, 'latitude': 23.39630, 'longitude': 113.16130,
             'elapsed_s': 1.0},
            {'quality': 4, 'latitude': 23.39631, 'longitude': 113.16130,
             'elapsed_s': 2.0},
            {'quality': 5, 'latitude': 23.39632, 'longitude': 113.16130,
             'elapsed_s': 3.0},
            {'quality': 4, 'latitude': 23.39630, 'longitude': 113.16130,
             'elapsed_s': 7.0},
        ]

        metrics = PROBE.fixed_only_statistics(samples)

        self.assertEqual(metrics['count'], 3)
        self.assertEqual(metrics['first_fixed_s'], 1.0)
        self.assertAlmostEqual(metrics['longest_continuous_s'], 1.0)
        self.assertGreater(metrics['north_span'], 1.0)
        self.assertGreater(metrics['std_north'], 0.0)


if __name__ == '__main__':
    unittest.main()
