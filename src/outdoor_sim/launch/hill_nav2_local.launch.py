#!/usr/bin/env python3
"""
hill_nav2_local.launch.py — 斜坡场景 LiDAR 避障启动

使用 pointcloud_to_laserscan 将 LiDAR 点云转成激光扫描数据，
提供给 Nav2 局部代价地图做避障。

TODO: 安装 ros-humble-pointcloud-to-laserscan 后生效
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # =============================================================
        # pointcloud_to_laserscan: /velodyne_points → /scan
        # Nav2 代价地图需要 /scan 格式，但 LiDAR 发的是 PointCloud2
        # =============================================================
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            parameters=[{
                "use_sim_time": use_sim_time,
                "target_frame": "lidar_link",
                "transform_tolerance": 0.01,
                "min_height": 0.05,
                "max_height": 2.0,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.2,
                "range_max": 30.0,
                "inf_epsilon": 1.0,
            }],
            remappings=[
                ("cloud_in", "/velodyne_points"),
                ("/scan", "/scan"),
            ],
        ),
    ])
