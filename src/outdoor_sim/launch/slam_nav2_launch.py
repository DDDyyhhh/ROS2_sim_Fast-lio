#!/usr/bin/env python3
"""
slam_nav2_launch.py — 未知环境自主探索与导航联合启动

同时拉起 FAST-LIO 3D SLAM、PointCloud→LaserScan、Nav2 导航堆栈。
FAST-LIO 提供实时里程计（camera_init→body），Nav2 在其上做动态避障与路径规划。

TF 树结构：
  map  ──(static)──→  odom  ──(static)──→  camera_init  ──(FAST-LIO)──→  body
                     (Nav2 costmap 定位)      (FAST-LIO 里程计根帧)

★ 启动时序：
  1. 静态 TF + FAST-LIO 立即启动
  2. tf_waiter 等待 camera_init→body 变换就绪（最多 30s）
  3. Nav2 + PointCloud→LaserScan + RViz 在 TF 树完整后才启动
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (GroupAction, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    outdoor_sim_share = get_package_share_directory("outdoor_sim")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    fast_lio_dir = get_package_share_directory("fast_lio")

    slam_params_file = os.path.join(outdoor_sim_share, "config", "nav2_slam_params.yaml")
    rviz_config_file = os.path.join(outdoor_sim_share, "config", "nav2_3d_view.rviz")

    # ── 第 1 组：立即启动（静态 TF + FAST-LIO）──────────────────────

    # 1a. 静态 TF 桥接：将 FAST-LIO 的 camera_init 融入 Nav2 的 map→odom 体系
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
    )
    odom_to_camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
        parameters=[{"use_sim_time": True}],
    )

    # 1b. FAST-LIO 配置目录与文件名（分离传入，适配 mapping.launch.py 的参数签名）
    fast_lio_config_dir = os.path.join(fast_lio_dir, "config")
    fast_lio_config_file = "mid360_sim.yaml"

    # 1c. 启动 FAST-LIO 3D SLAM
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, "launch", "mapping.launch.py")
        ),
        launch_arguments={
            "config_path": fast_lio_config_dir,
            "config_file": fast_lio_config_file,
            "use_sim_time": "true",
            "rviz": "false",  # 关闭 FAST-LIO 自带的 RViz，由外面统一管理
        }.items(),
    )

    # ── 第 2 组：TF 等待门控 ────────────────────────────────────────

    # 等待 camera_init→body 变换可用（防止 Nav2 在 TF 树断裂时崩溃）
    tf_waiter = Node(
        package="outdoor_sim",
        executable="tf_waiter.py",
        name="tf_waiter",
        parameters=[{"use_sim_time": True, "timeout": 30.0}],
        output="screen",
    )

    # ── 第 3 组：TF 就绪后方可启动（Nav2 + 降维 + RViz）────────────

    # 3a. 3D 点云 → 2D LaserScan 降维（供 Nav2 costmap 使用）
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[
            ("cloud_in", "/velodyne_points"),
            ("scan", "/scan"),
        ],
        parameters=[{
            "target_frame": "lidar_link",
            "min_height": 0.1,
            "max_height": 1.0,
            "use_sim_time": True,
        }],
    )

    # 3b. Nav2 导航堆栈（planner/controller，无 AMCL / map_server）
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": slam_params_file,
            "autostart": "true",
        }.items(),
    )

    # 3c. RViz2 可视化
    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "rviz_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "rviz_config": rviz_config_file,
        }.items(),
    )

    # 将第 3 组包裹为 GroupAction，等待 tf_waiter 退出后统一启动
    nav2_group = GroupAction(
        [
            pointcloud_to_laserscan,
            nav2_navigation,
            nav2_rviz,
        ],
        # 保持 use_sim_time 参数继承
        scoped=False,
    )

    # 注册事件处理器：tf_waiter 正常退出 → 启动 nav2_group
    tf_ready_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=tf_waiter,
            on_exit=[nav2_group],
        )
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),  # 全局强制仿真时钟同步
        map_to_odom,
        odom_to_camera_init,
        fast_lio,
        tf_waiter,
        tf_ready_handler,
    ])
