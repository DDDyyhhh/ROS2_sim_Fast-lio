#!/usr/bin/env python3
"""Simulation profile for remote mission capture.

This profile starts no coverage planner/executor and owns the simulation
teleop command path only. Real RTK topics, when present in the same ROS graph,
are read by the Web UI as a separate antenna/status stream.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_outdoor = get_package_share_directory('outdoor_sim')
    pkg_mower = get_package_share_directory('mower_coverage')
    pkg_fast_lio = get_package_share_directory('fast_lio')

    use_sim_time = LaunchConfiguration('use_sim_time')
    with_fastlio = LaunchConfiguration('with_fastlio')
    with_ekf = LaunchConfiguration('with_ekf')
    with_lidar_scan = LaunchConfiguration('with_lidar_scan')
    with_odom_to_tf = LaunchConfiguration('with_odom_to_tf')
    pose_topic = LaunchConfiguration('pose_topic')
    mission_file = LaunchConfiguration('mission_file')
    with_sim_rtk = LaunchConfiguration('with_sim_rtk')
    simulation_cmd_vel_topic = LaunchConfiguration('simulation_cmd_vel_topic')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_outdoor, 'launch', 'hill_sim_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'with_odom_to_tf': with_odom_to_tf,
            'cmd_vel_topic': simulation_cmd_vel_topic,
        }.items(),
    )
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fast_lio, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': 'mid360.yaml',
            'rviz': 'false',
        }.items(),
        condition=IfCondition(with_fastlio),
    )
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_outdoor, 'launch', 'hill_ekf.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(with_ekf),
    )
    lidar_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_outdoor, 'launch', 'hill_nav2_local.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(with_lidar_scan),
    )

    # Use the normal mower package bridge, but deliberately omit the planner
    # and executor. The simulation command topic is private to this profile.
    web_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mower, 'launch', 'rosbridge_bridge.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    capture = Node(
        package='mower_coverage',
        executable='remote_capture_node',
        name='remote_capture',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pose_topic': pose_topic,
            'capture_frame_id': 'odom',
            'allow_local_odom': True,
            'mission_file': mission_file,
            'initial_health_state': 'RED',
        }],
    )
    simulated_rtk = Node(
        package='mower_coverage',
        executable='simulation_rtk',
        name='simulation_rtk',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_fix_topic': '/gps/fix',
            'fix_topic': '/rtk/gps/fix',
            'status_topic': '/rtk/status',
            'health_topic': '/localization/health',
        }],
        condition=IfCondition(with_sim_rtk),
    )
    teleop = Node(
        package='mower_coverage',
        executable='simulation_teleop',
        name='simulation_teleop',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_topic': simulation_cmd_vel_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('with_fastlio', default_value='false'),
        DeclareLaunchArgument('with_ekf', default_value='false'),
        DeclareLaunchArgument('with_lidar_scan', default_value='false'),
        DeclareLaunchArgument('with_odom_to_tf', default_value='false'),
        DeclareLaunchArgument('pose_topic', default_value='/odom'),
        DeclareLaunchArgument('with_sim_rtk', default_value='true'),
        DeclareLaunchArgument(
            'simulation_cmd_vel_topic', default_value='/simulation/cmd_vel'),
        DeclareLaunchArgument(
            'mission_file',
            default_value=os.path.join(
                os.path.expanduser('~'),
                '.local/state/mower_coverage/remote_capture_mission.yaml')),
        sim_launch,
        fast_lio,
        ekf,
        lidar_scan,
        web_bridge,
        simulated_rtk,
        capture,
        teleop,
    ])
