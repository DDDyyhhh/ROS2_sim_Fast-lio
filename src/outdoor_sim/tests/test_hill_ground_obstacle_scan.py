#!/usr/bin/env python3
"""Regression tests for hill point-cloud ground filtering."""

import sys
import unittest
from pathlib import Path

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from hill_ground_obstacle_scan import (  # noqa: E402
    build_scan_ranges,
    extract_obstacle_points,
)


class HillGroundObstacleScanTest(unittest.TestCase):
    @staticmethod
    def make_ground_points():
        points = []
        for x in np.arange(-2.0, 2.01, 0.2):
            for y in np.arange(-2.0, 2.01, 0.2):
                # A sloped surface whose height exceeds the old 1 m cutoff.
                z = 0.2 + 0.55 * x - 0.25 * y
                points.append((x, y, z))
        return np.asarray(points, dtype=float)

    def test_sloped_ground_is_filtered_without_absolute_height_cutoff(self):
        ground = self.make_ground_points()

        obstacles = extract_obstacle_points(
            ground,
            min_range=0.35,
            ground_cell_size=0.25,
            ground_clearance=0.12,
        )

        self.assertEqual(len(obstacles), 0)

    def test_sub_one_meter_obstacle_is_kept_above_sloped_ground(self):
        ground = self.make_ground_points()
        obstacle_base = np.asarray([
            (0.8, -0.2, 0.2 + 0.55 * 0.8 - 0.25 * -0.2),
            (1.0, -0.2, 0.2 + 0.55 * 1.0 - 0.25 * -0.2),
            (0.8, 0.0, 0.2 + 0.55 * 0.8),
            (1.0, 0.0, 0.2 + 0.55 * 1.0),
        ])
        obstacle = obstacle_base.copy()
        obstacle[:, 2] += 0.25  # 25 cm high: below the old 1 m workaround.
        points = np.vstack((ground, obstacle))

        detected = extract_obstacle_points(
            points,
            min_range=0.35,
            ground_cell_size=0.25,
            ground_clearance=0.12,
        )

        for point in obstacle:
            self.assertTrue(
                np.any(np.all(np.isclose(detected, point), axis=1)),
                msg=f"low obstacle was filtered: {point}",
            )

    def test_points_without_ground_support_are_kept_fail_safe(self):
        obstacle = np.asarray([(1.0, 0.0, 0.25)], dtype=float)

        detected = extract_obstacle_points(
            obstacle,
            min_range=0.35,
            ground_cell_size=0.25,
            ground_clearance=0.12,
        )

        np.testing.assert_allclose(detected, obstacle)

    def test_conversion_failure_publishes_a_stop_scan(self):
        ranges = build_scan_ranges(
            np.empty((0, 3)),
            count=8,
            angle_min=-1.0,
            angle_increment=0.25,
            range_min=0.35,
            range_max=30.0,
            emergency=True,
        )

        np.testing.assert_allclose(ranges, np.full(8, 0.35))


if __name__ == "__main__":
    unittest.main()
