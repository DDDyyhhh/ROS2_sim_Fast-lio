#!/usr/bin/env python3
"""Full simulated sensor, localization, Web, planning, and execution profile."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    outdoor = get_package_share_directory('outdoor_sim')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(outdoor, 'launch', 'hill_full.launch.py'))),
    ])
