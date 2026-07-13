#!/usr/bin/env python3
"""Web multi-area mowing with simulation and direct execution."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    outdoor = get_package_share_directory('outdoor_sim')
    mower = get_package_share_directory('mower_coverage')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(outdoor, 'launch', 'hill_sim_launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(mower, 'launch', 'rosbridge_bridge.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mower, 'launch', 'hill_coverage.launch.py')),
            launch_arguments={'execution_mode': 'direct'}.items(),
        ),
    ])
