#!/usr/bin/env python3
"""
hill_nav2_local.launch.py — 斜坡场景 LiDAR 避障启动

使用坡面地面分割节点将 LiDAR 点云转成激光扫描数据，
提供给 Nav2 局部代价地图做避障。
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
        # hill_ground_obstacle_scan: /velodyne_points → /scan
        # Nav2 代价地图需要 /scan 格式，但 LiDAR 发的是 PointCloud2
        # 不使用绝对 min_height：坡面地面可能高于 1m，低矮障碍也必须保留
        # =============================================================
        Node(
            package="outdoor_sim",
            executable="hill_ground_obstacle_scan.py",
            name="hill_ground_obstacle_scan",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": "/velodyne_points",
                "output_topic": "/scan",
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.2,
                "range_max": 30.0,
                "ground_fit_range": 8.0,
                "ground_cell_size": 0.25,
                "ground_clearance": 0.03,
                "min_ground_points": 3,
                "min_ground_fit_points": 64,
                "min_ground_cells": 32,
                "ground_support_radius": 1.0,
                "min_ground_cell_span": 0.02,
                "self_filter_x": 0.25,
                "self_filter_y": 0.20,
            }],
        ),
    ])
