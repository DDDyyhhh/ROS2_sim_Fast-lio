#!/usr/bin/env python3
"""Regression contracts for the sensor_full ROS graph.

This test intentionally uses only the standard library so it can run before a
ROS launch or a pytest installation is available.
"""

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class SensorChainContractTest(unittest.TestCase):
    def test_pointcloud_converter_receives_velodyne_topic(self):
        launch = read("src/outdoor_sim/launch/hill_nav2_local.launch.py")

        self.assertIn('(\"cloud_in\", \"/velodyne_points\")', launch)
        self.assertNotIn(
            '(\"/velodyne_points\", \"/velodyne_points\")', launch)

    def test_safe_executor_uses_sensor_data_qos_for_scan(self):
        source = read(
            "src/mower_coverage/mower_coverage/execution/multi_area_executor.py"
        )

        self.assertIn(
            "from rclpy.qos import qos_profile_sensor_data", source)
        self.assertRegex(
            source,
            r"LaserScan,\s*'/scan',\s*self\.scan_callback,\s*"
            r"qos_profile_sensor_data",
        )

    def test_localization_uses_existing_body_frame(self):
        launch = read("src/outdoor_sim/launch/hill_ekf.launch.py")

        self.assertEqual(launch.count('"base_link_frame": "body"'), 1)

    def test_navsat_output_is_connected_to_ekf_input(self):
        launch = read("src/outdoor_sim/launch/hill_ekf.launch.py")

        self.assertIn('(\"/odometry/gps\", \"/odom/gps\")', launch)
        self.assertIn('"transform_timeout": 0.2', launch)

    def test_fast_lio_handles_pointcloud_without_time_field(self):
        source = read("src/fast_lio/src/preprocess.cpp")

        self.assertIn("bool has_point_field", source)
        self.assertIn("velodyne_ros::PointXYZIR", source)
        self.assertRegex(source, r'has_point_field\(\*msg,\s*"time"\)')
        self.assertIn("time = 0.0f", source)


if __name__ == "__main__":
    unittest.main()
