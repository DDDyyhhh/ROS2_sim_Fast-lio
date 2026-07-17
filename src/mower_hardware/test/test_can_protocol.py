#!/usr/bin/env python3
"""Offline regression tests for the delivered CAN protocol."""

import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from mower_hardware.can_protocol import (  # noqa: E402
    CAN_BITRATE,
    CAN_FRAME_DLC,
    CONTROL_CAN_ID,
    CONTROL_PERIOD_S,
    CONTROL_TIMEOUT_S,
    FEEDBACK_CAN_ID,
    CanFrame,
    CanProtocolError,
    ControlCommand,
    FeedbackStatus,
    decode_control_frame,
    decode_feedback_frame,
    replay_candump,
    validate_control_replay,
)


class CanProtocolTest(unittest.TestCase):
    def test_matrix_constants_match_delivered_protocol(self):
        self.assertEqual(CAN_BITRATE, 500_000)
        self.assertEqual(CAN_FRAME_DLC, 8)
        self.assertEqual(CONTROL_CAN_ID, 0x005)
        self.assertEqual(FEEDBACK_CAN_ID, 0x507)
        self.assertEqual(CONTROL_PERIOD_S, 0.05)
        self.assertEqual(CONTROL_TIMEOUT_S, 0.5)

    def test_decodes_control_frame_scaling_and_targets(self):
        frame = CanFrame(
            CONTROL_CAN_ID,
            bytes.fromhex('02 FB 00 FD 01 03 01 00'),
        )

        command = decode_control_frame(frame)

        self.assertIsInstance(command, ControlCommand)
        self.assertEqual(command.ctrl_mode, 2)
        self.assertEqual(command.vx_raw, -5)
        self.assertAlmostEqual(command.vx_mps, -0.5)
        self.assertEqual(command.vy_raw, 0)
        self.assertEqual(command.omega_raw, -3)
        self.assertAlmostEqual(command.omega_rad_s, -0.3)
        self.assertEqual(command.blade_level, 1)
        self.assertEqual(command.lifter_level, 3)
        self.assertTrue(command.bag_open_requested)
        self.assertFalse(command.unlock_requested)

    def test_exposes_unlock_key_without_treating_it_as_normal(self):
        frame = CanFrame(
            CONTROL_CAN_ID,
            bytes.fromhex('01 00 00 00 00 01 00 AA'),
        )

        command = decode_control_frame(frame)

        self.assertTrue(command.unlock_requested)
        self.assertEqual(command.magic_key, 0xAA)

    def test_decodes_feedback_big_endian_and_status_bits(self):
        frame = CanFrame(
            FEEDBACK_CAN_ID,
            bytes.fromhex('01 F4 FE D4 02 50 23 13'),
        )

        status = decode_feedback_frame(frame)

        self.assertIsInstance(status, FeedbackStatus)
        self.assertEqual(status.vx_raw, 500)
        self.assertAlmostEqual(status.vx_mps, 0.5)
        self.assertEqual(status.omega_raw, -300)
        self.assertAlmostEqual(status.omega_rad_s, -0.3)
        self.assertEqual(status.ctrl_mode, 2)
        self.assertEqual(status.battery_soc, 80)
        self.assertEqual(status.bms_temperature_c, 35)
        self.assertTrue(status.bag_open)
        self.assertTrue(status.bag_connected)
        self.assertFalse(status.bag_full)
        self.assertEqual(status.blade_level, 1)

    def test_unknown_temperature_is_not_reported_as_255_degrees(self):
        status = decode_feedback_frame(
            CanFrame(FEEDBACK_CAN_ID, bytes.fromhex('00 00 00 00 01 01 FF 00')))

        self.assertIsNone(status.bms_temperature_c)

    def test_rejects_bad_frame_boundaries_and_undefined_fields(self):
        with self.assertRaises(CanProtocolError):
            decode_control_frame(CanFrame(CONTROL_CAN_ID, b'\x00' * 7))
        with self.assertRaises(CanProtocolError):
            decode_feedback_frame(CanFrame(FEEDBACK_CAN_ID, b'\x00' * 9))
        with self.assertRaises(CanProtocolError):
            decode_control_frame(
                CanFrame(CONTROL_CAN_ID, bytes.fromhex('02 00 01 00 00 01 00 00')))
        with self.assertRaises(CanProtocolError):
            decode_control_frame(
                CanFrame(CONTROL_CAN_ID, bytes.fromhex('02 00 00 00 00 01 00 01')))
        with self.assertRaises(CanProtocolError):
            decode_feedback_frame(
                CanFrame(FEEDBACK_CAN_ID, bytes.fromhex('00 00 00 00 02 01 00 08')))

    def test_replays_candump_and_skips_unsupported_ids(self):
        records = replay_candump([
            '(1.000000) can0 507#01F4FED402502313',
            '(1.040000) can0 508#0000000000000000',
            '(1.050000) can0 005#0205000001030000',
            '',
        ])

        self.assertEqual(len(records), 2)
        self.assertEqual(records[0].line_number, 1)
        self.assertEqual(records[0].decoded.frame.timestamp, 1.0)
        self.assertIsInstance(records[0].decoded.message, FeedbackStatus)
        self.assertIsInstance(records[1].decoded.message, ControlCommand)
        self.assertEqual(records[1].decoded.frame.interface, 'can0')

    def test_audits_safe_startup_before_motion(self):
        records = replay_candump([
            '(1.000000) can0 005#0100000000010000',
            '(1.040000) can0 507#0000000001500000',
            '(1.050000) can0 005#0200000000010000',
            '(1.090000) can0 507#0000000002500000',
            '(1.100000) can0 005#0205000001030000',
        ])

        validate_control_replay(records)

    def test_audit_rejects_extended_looking_id(self):
        with self.assertRaises(CanProtocolError):
            replay_candump(['(1.0) can0 00000507#01F4FED402502313'])

    def test_audit_rejects_unconfirmed_run_and_timeout(self):
        records = replay_candump([
            '(1.000000) can0 005#0100000000010000',
            '(1.050000) can0 507#0000000001500000',
            '(1.100000) can0 005#0205000001030000',
        ])
        with self.assertRaises(CanProtocolError):
            validate_control_replay(records)

        records = replay_candump([
            '(1.000000) can0 005#0100000000010000',
            '(1.050000) can0 507#0000000001500000',
            '(1.100000) can0 005#0200000000010000',
            '(1.150000) can0 507#0000000002500000',
            '(1.701000) can0 005#0200000000010000',
        ])
        with self.assertRaises(CanProtocolError):
            validate_control_replay(records)

    def test_audit_rejects_consecutive_unlock_frames(self):
        records = replay_candump([
            '(1.000000) can0 005#01000000000100AA',
            '(1.050000) can0 005#01000000000100AA',
        ])

        with self.assertRaises(CanProtocolError):
            validate_control_replay(records)

    def test_rejects_malformed_candump_line(self):
        with self.assertRaises(CanProtocolError):
            replay_candump(['(1.0) can0 507#01F4FE'])
        with self.assertRaises(CanProtocolError):
            replay_candump(['(1.0) can0 507##01F4FED402502313'])


if __name__ == '__main__':
    unittest.main()
