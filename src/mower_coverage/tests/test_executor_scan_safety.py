#!/usr/bin/env python3
"""Regression tests for safe executor scan freshness handling."""

import time

from mower_coverage.execution.multi_area_executor import MultiAreaExecutor
from sensor_msgs.msg import LaserScan


def _node(mode='safe'):
    node = object.__new__(MultiAreaExecutor)
    node.execution_mode = mode
    node.scan_timeout = 0.5
    node.scan_received = False
    node.scan_valid = False
    node.last_scan_monotonic = None
    node.obstacle_scan_angle = 60.0
    node.latest_scan = None
    node.obstacle_detected = False
    node.obstacle_angle = 0.0
    node.obstacle_distance = 999.0
    return node


def test_safe_mode_rejects_missing_or_stale_scan():
    node = _node()

    assert not node._scan_is_fresh()

    node.scan_received = True
    node.scan_valid = True
    node.last_scan_monotonic = time.monotonic() - 1.0

    assert not node._scan_is_fresh()


def test_safe_mode_accepts_recent_valid_scan_and_direct_mode_bypasses_scan():
    node = _node()
    node.scan_received = True
    node.scan_valid = True
    node.last_scan_monotonic = time.monotonic()

    assert node._scan_is_fresh()
    assert _node('direct')._scan_is_fresh()


def test_safe_mode_rejects_nan_scan_but_accepts_no_return_scan():
    node = _node()
    invalid = LaserScan()
    invalid.angle_min = -1.0
    invalid.angle_max = 1.0
    invalid.angle_increment = 0.1
    invalid.range_min = 0.2
    invalid.range_max = 10.0
    invalid.ranges = [float('nan')] * 20

    node.scan_callback(invalid)

    assert not node.scan_valid
    assert not node._scan_is_fresh()

    valid = LaserScan()
    valid.angle_min = -1.0
    valid.angle_max = 1.0
    valid.angle_increment = 0.1
    valid.range_min = 0.2
    valid.range_max = 10.0
    valid.ranges = [float('inf')] * 20

    node.scan_callback(valid)

    assert node.scan_valid
    assert node._scan_is_fresh()

    invalid.ranges = [float('-inf')]
    node.scan_callback(invalid)

    assert not node.scan_valid
    assert not node._scan_is_fresh()
