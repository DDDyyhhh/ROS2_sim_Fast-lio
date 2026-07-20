#!/usr/bin/env python3
"""Start the single-owner UM982/NTRIP node and remote Web UI without motion control.

This profile deliberately starts no planner, executor, CAN bridge, simulator,
or motor controller.  It is the first real-hardware bring-up profile and only
owns the UM982 serial fd, NTRIP corrections, /gps/fix and Web/rosbridge path.
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
    caster_host = LaunchConfiguration('caster_host')
    caster_port = LaunchConfiguration('caster_port')
    mountpoint = LaunchConfiguration('mountpoint')
    gga_interval = LaunchConfiguration('gga_interval')
    correction_timeout = LaunchConfiguration('correction_timeout')
    http_port = LaunchConfiguration('http_port')
    websocket_port = LaunchConfiguration('websocket_port')

    mower_share = get_package_share_directory('mower_coverage')
    web_dir = os.path.join(mower_share, 'web_frontend')

    rtk_driver = Node(
        package='mower_hardware',
        executable='rtk_ntrip_node',
        name='um982_rtk_ntrip',
        output='screen',
        parameters=[{
            'device': um982_device,
            'baud': baud,
            'frame_id': frame_id,
            'caster_host': caster_host,
            'caster_port': caster_port,
            'mountpoint': mountpoint,
            'gga_interval': gga_interval,
            'correction_timeout': correction_timeout,
            'use_sim_time': False,
        }],
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
            'caster_host',
            default_value=_env('CORS_HOST', '114.111.30.20'),
            description='NTRIP caster host'),
        DeclareLaunchArgument(
            'caster_port',
            default_value=_env('CORS_PORT', '8002'),
            description='NTRIP caster port'),
        DeclareLaunchArgument(
            'mountpoint',
            default_value=_env('CORS_MOUNTPOINT', 'RTCM33GRCEJ'),
            description='NTRIP mountpoint'),
        DeclareLaunchArgument(
            'gga_interval',
            default_value=_env('CORS_GGA_INTERVAL', '10.0'),
            description='Seconds between caster GGA reports'),
        DeclareLaunchArgument(
            'correction_timeout',
            default_value=_env('CORS_DATA_TIMEOUT', '15.0'),
            description='Seconds without RTCM before the caster is considered stale'),
        DeclareLaunchArgument(
            'http_port',
            default_value=_env('HTTP_PORT', '8080'),
            description='HTTP port for the Web UI'),
        DeclareLaunchArgument(
            'websocket_port',
            default_value=_env('ROSBRIDGE_PORT', '9090'),
            description='rosbridge WebSocket port'),
        rtk_driver,
        rosbridge,
        web_server,
    ])
