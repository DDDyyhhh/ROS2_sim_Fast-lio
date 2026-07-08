#!/usr/bin/env python3
"""
hill_coverage.launch.py — 斜坡场景全覆盖规划启动文件

启动节点：
  1. multi_area_definer — 多区域定义与管理
  2. hill_boustrophedon — 多区域牛耕式路径规划
  3. multi_area_executor — 多区域路径执行

用法：
  # 先启动仿真：
  ros2 launch outdoor_sim hill_sim_launch.py

  # 再启动覆盖规划（本文件）：
  ros2 launch mower_coverage hill_coverage.launch.py

  # 加载预设区域：
  ros2 service call /multi_area/load std_srvs/srv/Trigger

  # 开始执行：
  ros2 service call /multi_area/plan_and_start std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mower_coverage')
    config_file = os.path.join(pkg_dir, 'config', 'hill_coverage_params.yaml')
    areas_file = os.path.join(pkg_dir, 'config', 'areas_example.yaml')

    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    frame_id = LaunchConfiguration('frame_id', default='map')
    cutting_width = LaunchConfiguration('cutting_width', default='0.5')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='使用仿真时钟')
    declare_frame_id = DeclareLaunchArgument(
        'frame_id', default_value='map',
        description='坐标系')
    declare_cutting_width = DeclareLaunchArgument(
        'cutting_width', default_value='0.5',
        description='割幅宽度 (m)')

    # 1. 多区域定义节点
    multi_area_definer = Node(
        package='mower_coverage',
        executable='multi_area_definer',
        name='multi_area_definer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'area_file': os.path.expanduser('~/hill_mowing_areas.yaml'),
        }],
    )

    # 2. 多区域牛耕式规划器
    hill_boustrophedon = Node(
        package='mower_coverage',
        executable='hill_boustrophedon',
        name='hill_boustrophedon',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'cutting_width': cutting_width,
            'overlap': 0.1,
            'waypoint_spacing': 0.3,
            'start_from_corner': True,
            'terrain_slope_threshold': 20.0,
            'terrain_slow_factor': 0.5,
        }],
    )

    # 3. 多区域执行器
    multi_area_executor = Node(
        package='mower_coverage',
        executable='multi_area_executor',
        name='multi_area_executor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'goal_tolerance': 0.3,
            'max_linear_speed': 1.0,
            'max_angular_speed': 1.0,
            'checkpoint_file': os.path.expanduser('~/hill_coverage_checkpoint.json'),
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_frame_id,
        declare_cutting_width,
        multi_area_definer,
        hill_boustrophedon,
        multi_area_executor,
    ])
