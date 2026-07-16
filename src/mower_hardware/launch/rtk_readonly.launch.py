#!/usr/bin/env python3
"""Start the UM982 NMEA reader and remote Web UI without motion control.

This profile deliberately starts no planner, executor, CAN bridge, simulator,
or motor controller.  It is the first real-hardware bring-up profile and only
owns the UM982 -> /gps/fix and Web/rosbridge read-only path.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _env(name, default):
    return EnvironmentVariable(name, default_value=default)


def generate_launch_description():
    um982_device = LaunchConfiguration('um982_device')
    baud = LaunchConfiguration('baud')
    frame_id = LaunchConfiguration('frame_id')
    http_port = LaunchConfiguration('http_port')
    websocket_port = LaunchConfiguration('websocket_port')

    mower_share = get_package_share_directory('mower_coverage')
    web_dir = os.path.join(mower_share, 'web_frontend')

    nmea_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='um982_nmea_driver',
        output='screen',
        parameters=[{
            'port': um982_device,
            'baud': baud,
            'frame_id': frame_id,
            'use_sim_time': False,
        }],
        remappings=[
            ('fix', '/gps/fix'),
            ('nmea_sentence', '/rtk/nmea'),
        ],
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': websocket_port,
            'use_sim_time': False,
        }],
    )

    web_server = Node(
        package='mower_coverage',
        executable='web_server',
        name='web_frontend_server',
        output='screen',
        parameters=[{
            'port': http_port,
            'web_dir': web_dir,
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'um982_device',
            default_value=_env('UM982_DEVICE', '/dev/um982'),
            description='UM982 serial device inside the container'),
        DeclareLaunchArgument(
            'baud',
            default_value=_env('UM982_BAUD', '115200'),
            description='UM982 serial baud rate'),
        DeclareLaunchArgument(
            'frame_id',
            default_value=_env('UM982_FRAME_ID', 'rtk_link'),
            description='Frame attached to the RTK antenna'),
        DeclareLaunchArgument(
            'http_port',
            default_value=_env('HTTP_PORT', '8080'),
            description='HTTP port for the Web UI'),
        DeclareLaunchArgument(
            'websocket_port',
            default_value=_env('ROSBRIDGE_PORT', '9090'),
            description='rosbridge WebSocket port'),
        nmea_driver,
        rosbridge,
        web_server,
    ])
