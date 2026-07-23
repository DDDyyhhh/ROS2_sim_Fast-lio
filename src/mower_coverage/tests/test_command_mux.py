"""Behavior tests for the simulation command arbiter."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_coverage.command_mux import CommandMux, MuxCommand


class CommandMuxTests(unittest.TestCase):
    def test_teleop_is_selected_when_no_plan_is_active(self):
        mux = CommandMux(timeout=0.4)
        mux.accept_teleop(MuxCommand(0.4, -0.2), now=1.0)

        self.assertEqual(mux.output(1.2), MuxCommand(0.4, -0.2))

    def test_active_plan_has_priority_over_teleop(self):
        mux = CommandMux(timeout=0.4)
        mux.accept_teleop(MuxCommand(0.4, 0.0), now=1.0)
        mux.accept_plan(MuxCommand(0.2, 0.3), now=1.0)
        mux.set_plan_active(True)

        self.assertEqual(mux.output(1.2), MuxCommand(0.2, 0.3))

    def test_stale_owner_fails_closed_without_fallback(self):
        mux = CommandMux(timeout=0.4)
        mux.accept_teleop(MuxCommand(0.4, 0.0), now=1.0)
        mux.accept_plan(MuxCommand(0.2, 0.3), now=1.0)
        mux.set_plan_active(True)

        self.assertEqual(mux.output(1.41), MuxCommand())


if __name__ == '__main__':
    unittest.main()
