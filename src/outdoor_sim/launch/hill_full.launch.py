#!/usr/bin/env python3
"""
hill_full.launch.py — 斜坡场景一键启动（全集成）

一键启动整个系统:
  1. Ignition Fortress 仿真（斜坡地形 30×30m）
  2. ROS2 桥接（LiDAR / IMU / GPS / cmd_vel）
  3. FAST-LIO SLAM（3D LiDAR+IMU 里程计）
  4. EKF 融合（FAST-LIO + IMU + RTK GPS → /odometry/filtered）
  5. 点云→激光转换（/velodyne_points → /scan）
  6. rosbridge WebSocket（Web 前端通信）
  7. 全覆盖规划 + 执行

启动后操作:
  1. 浏览器打开 http://localhost:8080
  2. 在卫星地图上画区域 → 点"发送到割草机"
  3. 点"规划路径" → "开始执行"

也可以分步操作（ROS2 服务）:
  ros2 service call /multi_area/plan std_srvs/srv/Trigger
  ros2 service call /multi_area/plan_and_start std_srvs/srv/Trigger
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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 各组件开关（可单独关闭）
    with_fastlio = LaunchConfiguration('with_fastlio', default='true')
    with_ekf = LaunchConfiguration('with_ekf', default='true')
    with_lidar_scan = LaunchConfiguration('with_lidar_scan', default='true')
    with_rosbridge = LaunchConfiguration('with_rosbridge', default='true')
    with_odom_to_tf = LaunchConfiguration('with_odom_to_tf', default='false')

    return LaunchDescription([
        # ==============================================================
        # Launch 参数
        # ==============================================================
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('with_fastlio', default_value='true',
                              description='启动 FAST-LIO SLAM'),
        DeclareLaunchArgument('with_ekf', default_value='true',
                              description='启动 EKF 融合定位'),
        DeclareLaunchArgument('with_lidar_scan', default_value='true',
                              description='启动 LiDAR→/scan 转换'),
        DeclareLaunchArgument('with_rosbridge', default_value='true',
                              description='启动 Web 前端桥接'),
        DeclareLaunchArgument(
            "with_odom_to_tf", default_value="false",
            description="EKF 已发布 odom→body；仅关闭 EKF 时再启用 /odom TF 中继",
        ),

        # ==============================================================
        # 1. 斜坡仿真 + 传感器桥接
        # ==============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_outdoor, 'launch', 'hill_sim_launch.py'),
            ),
            launch_arguments={
                "with_odom_to_tf": with_odom_to_tf,
            }.items(),
        ),

        # ==============================================================
        # 2. FAST-LIO SLAM（可选）
        # ==============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_fast_lio, 'launch', 'mapping.launch.py'),
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'config_file': 'mid360.yaml',
                'rviz': 'false',
            }.items(),
            condition=IfCondition(with_fastlio),
        ),

        # ==============================================================
        # 3. EKF 融合定位（可选）
        # ==============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_outdoor, 'launch', 'hill_ekf.launch.py'),
            ),
            condition=IfCondition(with_ekf),
        ),

        # ==============================================================
        # 4. LiDAR → /scan（可选，供避障使用）
        # ==============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_outdoor, 'launch', 'hill_nav2_local.launch.py'),
            ),
            condition=IfCondition(with_lidar_scan),
        ),

        # ==============================================================
        # 5. rosbridge + Web 前端（可选）
        # ==============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_mower, 'launch', 'rosbridge_bridge.launch.py'),
            ),
            condition=IfCondition(with_rosbridge),
        ),

        # ==============================================================
        # 6. 全覆盖规划 + 执行
        # ==============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_mower, 'launch', 'hill_coverage.launch.py'),
            ),
        ),
    ])
