#!/usr/bin/env python3
"""Behavior tests for the simulation teleop safety gate."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_coverage.teleop import TeleopCommand, TeleopGate


class TeleopGateTests(unittest.TestCase):
    def test_gate_blocks_until_capture_is_active(self):
        gate = TeleopGate()
        gate.accept(TeleopCommand(0.3, 0.4), now=1.0)

        self.assertEqual(gate.output(1.1), TeleopCommand())

        gate.set_allowed(True)
        self.assertEqual(gate.output(1.1), TeleopCommand(0.3, 0.4))

    def test_release_and_timeout_stop_motion(self):
        gate = TeleopGate(timeout=0.4)
        gate.set_allowed(True)
        gate.accept(TeleopCommand(0.3, 0.0), now=1.0)
        self.assertEqual(gate.output(1.2), TeleopCommand(0.3, 0.0))
        self.assertEqual(gate.output(1.41), TeleopCommand())

        gate.accept(TeleopCommand(0.3, 0.0), now=2.0)
        gate.set_allowed(False)
        self.assertEqual(gate.output(2.1), TeleopCommand())

    def test_commands_are_clamped(self):
        gate = TeleopGate(max_linear=0.5, max_angular=1.0)
        gate.set_allowed(True)
        gate.accept(TeleopCommand(2.0, -2.0), now=1.0)

        self.assertEqual(gate.output(1.1), TeleopCommand(0.5, -1.0))


if __name__ == '__main__':
    unittest.main()
