#!/usr/bin/env python3
"""Behavior tests for the localization safety gate."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_coverage.localization.health import assess_localization


def _healthy_observation():
    return {
        'fused_pose_valid': True,
        'fused_pose_fresh': True,
        'fused_pose_in_map': True,
        'local_odom_valid': True,
        'local_odom_fresh': True,
        'mid360_valid': True,
        'mid360_fresh': True,
        'imu_valid': True,
        'imu_fresh': True,
        'pointcloud_valid': True,
        'pointcloud_fresh': True,
        'pointcloud_time_monotonic': True,
        'rtk_fixed': True,
        'rtk_fresh': True,
    }


class LocalizationHealthTests(unittest.TestCase):
    def test_healthy_fused_chain_is_green_and_allowed(self):
        result = assess_localization(_healthy_observation())

        self.assertEqual(result.state, 'GREEN')
        self.assertTrue(result.capture_allowed)
        self.assertTrue(result.execution_allowed)
        self.assertEqual(result.reasons, ())

    def test_rtk_loss_with_continuous_local_chain_is_yellow_but_stopped(self):
        observation = _healthy_observation()
        observation['rtk_fixed'] = False
        observation['rtk_fresh'] = False

        result = assess_localization(observation)

        self.assertEqual(result.state, 'YELLOW')
        self.assertFalse(result.capture_allowed)
        self.assertFalse(result.execution_allowed)
        self.assertIn('RTK is not fixed and fresh', result.reasons)

    def test_mid360_failure_is_red(self):
        observation = _healthy_observation()
        observation['mid360_valid'] = False

        result = assess_localization(observation)

        self.assertEqual(result.state, 'RED')
        self.assertFalse(result.capture_allowed)
        self.assertFalse(result.execution_allowed)
        self.assertIn('Mid-360/FAST-LIO is invalid', result.reasons)

    def test_non_monotonic_pointcloud_time_is_red(self):
        observation = _healthy_observation()
        observation['pointcloud_time_monotonic'] = False

        result = assess_localization(observation)

        self.assertEqual(result.state, 'RED')
        self.assertIn('point cloud time is not monotonic', result.reasons)

    def test_fused_pose_outside_map_is_red(self):
        observation = _healthy_observation()
        observation['fused_pose_in_map'] = False

        result = assess_localization(observation)

        self.assertEqual(result.state, 'RED')
        self.assertIn('fused pose is not in map frame', result.reasons)

    def test_missing_or_non_boolean_signal_fails_closed(self):
        observation = _healthy_observation()
        del observation['imu_fresh']
        observation['pointcloud_valid'] = 'yes'

        result = assess_localization(observation)

        self.assertEqual(result.state, 'RED')
        self.assertFalse(result.capture_allowed)
        self.assertFalse(result.execution_allowed)
        self.assertIn('missing health signal: imu_fresh', result.reasons)
        self.assertIn('health signal must be boolean: pointcloud_valid',
                      result.reasons)


if __name__ == '__main__':
    unittest.main()
