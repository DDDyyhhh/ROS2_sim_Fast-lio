#!/usr/bin/env python3
"""
nav2_sim_launch.py — Nav2 导航堆栈启动文件 (配合仿真)

功能：
  1. 启动 pointcloud_to_laserscan 将 3D 点云转为 2D LaserScan
  2. 启动 Nav2 bringup (map_server, AMCL, planner, controller, …)
  3. 启动带 Nav2 面板的 RViz2

前置条件：
  sim_launch.py 已在另一个终端中运行 (Ignition + ros_gz_bridge + robot_state_publisher)

用法：
  ros2 launch outdoor_sim nav2_sim_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # 获取功能包路径
    # ------------------------------------------------------------------
    outdoor_sim_share = get_package_share_directory("outdoor_sim")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # ------------------------------------------------------------------
    # 资源路径
    # ------------------------------------------------------------------
    default_map_yaml = os.path.join(
        os.path.expanduser("~"), "ros2_ws", "fastlio_map.yaml"
    )
    default_params_file = os.path.join(
        outdoor_sim_share, "config", "nav2_params.yaml"
    )
    default_rviz_config = os.path.join(
        outdoor_sim_share, "config", "nav2_3d_view.rviz"
    )

    # ------------------------------------------------------------------
    # Launch 参数
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_yaml_file = LaunchConfiguration("map", default=default_map_yaml)
    params_file = LaunchConfiguration("params_file", default=default_params_file)
    autostart = LaunchConfiguration("autostart", default="true")
    rviz_config_file = LaunchConfiguration("rviz_config", default=default_rviz_config)

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_map_yaml = DeclareLaunchArgument(
        "map",
        default_value=default_map_yaml,
        description="Full path to the map yaml file to load",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the Nav2 ROS 2 parameters file",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Full path to the RViz config file to use",
    )

    # ------------------------------------------------------------------
    # 1. pointcloud_to_laserscan —— 将 3D 点云转为 2D LaserScan
    #
    #    输入: /velodyne_points (sensor_msgs/PointCloud2)
    #    输出: /scan (sensor_msgs/LaserScan)
    #
    #    参数说明:
    #      target_frame        — 激光扫描的参考系 (lidar_link)
    #      min/max_height      — Z 轴高度滤波 (去除地面/天花板点)
    #      angle_min/angle_max — 水平视场角 (-π ~ +π = 360°)
    # ------------------------------------------------------------------
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", "/velodyne_points"),
            ("scan", "/scan"),
        ],
        parameters=[{
            "target_frame": "lidar_link",
            "transform_tolerance": 0.01,
            "min_height": 0.1,
            "max_height": 1.0,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.1,
            "range_max": 40.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
            "use_sim_time": use_sim_time,
        }],
    )

    # ------------------------------------------------------------------
    # 2. pcd_to_pointcloud —— 将离线 3D PCD 地图发布到 ROS 2 点云话题
    #
    #    输入: /home/orangepi/ros2_ws/my_3d_map.pcd (磁盘文件)
    #    输出: /pcl_ros/pcd/points (sensor_msgs/PointCloud2)
    #
    #    参数说明:
    #      file_name          — PCD 文件路径
    #      tf_frame           — 点云的参考坐标系 (map)
    #      publishing_period_ms — 发布间隔 (5000ms = 5s, 节约 RK3588 CPU)
    #      qos_reliability    — QoS 可靠性策略 (best_effort)
    # ------------------------------------------------------------------
    pcd_publisher = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="pcd_publisher",
        output="screen",
        parameters=[{
            "file_name": "/home/orangepi/ros2_ws/my_3d_map.pcd",
            "tf_frame": "map",
            "publishing_period_ms": 5000,
            "qos_reliability": "best_effort",
        }],
    )

    # ------------------------------------------------------------------
    # 4. Nav2 bringup —— 完整的导航堆栈
    #
    #    包含: map_server, AMCL 定位, planner_server, controller_server,
    #         behavior_server, bt_navigator, velocity_smoother, …
    #
    #    传入参数:
    #      map           — PGM 地图文件路径 (通过 map 启动参数覆盖)
    #      use_sim_time  — 使用仿真时钟 (True)
    #      slam          — False (用已有的地图, 不开 SLAM)
    #      params_file   — 自定义 Nav2 参数文件 (适配 4WD 小车)
    #      autostart     — 自动启动导航
    # ------------------------------------------------------------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "slam": "False",
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 3. Nav2 RViz2 —— 带导航面板的可视化界面
    #
    #    包含: 2D Pose Estimate, 2D Nav Goal, 全局/局部代价地图, TF, …
    # ------------------------------------------------------------------
    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "rviz_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": "",
            "rviz_config": rviz_config_file,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 组装 LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription([
        # 声明参数
        declare_use_sim_time,
        declare_map_yaml,
        declare_params_file,
        declare_autostart,
        declare_rviz_config,

        # 实际动作
        pointcloud_to_laserscan,
        pcd_publisher,
        nav2_bringup,
        nav2_rviz,
    ])
