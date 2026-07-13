#!/usr/bin/env python3
"""
coverage_planning.launch.py — 全覆盖路径规划启动文件

启动节点：
  1. area_definer — 区域定义
  2. boustrophedon_planner — 牛耕式路径规划（作为节点）
  3. path_executor — 路径执行器
  4. coverage_monitor — 覆盖率监控

用法：
  ros2 launch mower_coverage coverage_planning.launch.py
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mower_coverage')
    config_file = os.path.join(pkg_dir, 'config', 'coverage_params.yaml')
    state_dir = Path.home() / '.local' / 'state' / 'mower_coverage'

    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    frame_id = LaunchConfiguration('frame_id', default='map')
    mode = LaunchConfiguration('mode', default='direct')
    cutting_width = LaunchConfiguration('cutting_width', default='0.5')
    area_file = LaunchConfiguration('area_file')
    checkpoint_file = LaunchConfiguration('checkpoint_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='使用仿真时钟')
    declare_frame_id = DeclareLaunchArgument(
        'frame_id', default_value='map',
        description='坐标系')
    declare_mode = DeclareLaunchArgument(
        'mode', default_value='direct',
        description='执行模式 (direct/nav2)')
    declare_cutting_width = DeclareLaunchArgument(
        'cutting_width', default_value='0.5',
        description='割幅宽度 (m)')
    declare_area_file = DeclareLaunchArgument(
        'area_file', default_value=str(state_dir / 'mowing_area.yaml'))
    declare_checkpoint_file = DeclareLaunchArgument(
        'checkpoint_file',
        default_value=str(state_dir / 'coverage_checkpoint.json'))

    # 1. 区域定义节点
    area_definer = Node(
        package='mower_coverage',
        executable='area_definer',
        name='area_definer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'area_file': area_file,
        }],
    )

    # 2. 路径规划演示节点
    coverage_demo = Node(
        package='mower_coverage',
        executable='coverage_demo',
        name='coverage_demo',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'cutting_width': cutting_width,
            'overlap': 0.1,
            'path_angle': 0.0,
            'use_default_area': True,
        }],
    )

    # 3. 路径执行器
    path_executor = Node(
        package='mower_coverage',
        executable='path_executor',
        name='path_executor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'mode': mode,
            'max_linear_speed': 1.0,
            'goal_tolerance': 0.3,
            'checkpoint_file': checkpoint_file,
        }],
    )

    # 4. 覆盖率监控
    coverage_monitor = Node(
        package='mower_coverage',
        executable='coverage_monitor',
        name='coverage_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'cutting_width': cutting_width,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_frame_id,
        declare_mode,
        declare_cutting_width,
        declare_area_file,
        declare_checkpoint_file,
        area_definer,
        coverage_demo,
        path_executor,
        coverage_monitor,
    ])
