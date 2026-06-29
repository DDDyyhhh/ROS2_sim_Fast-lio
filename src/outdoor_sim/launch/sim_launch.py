#!/usr/bin/env python3
"""
sim_launch.py — 户外仿真 Launch 文件 (Ignition Fortress / ros_gz)

功能：
  1. 启动 Ignition Gazebo，加载 grass_terrain.world (headless + 取消暂停)
  2. 启动 robot_state_publisher，解析 robot_sensors.xacro 发布 TF 树
  3. 运行 ros_gz_sim create，将机器人模型投入 Ignition 世界
  4. 运行 ros_gz_bridge parameter_bridge，将 Ignition 话题映射回 ROS 2：
       /clock           → rosgraph_msgs/Clock
       /lidar/points    → sensor_msgs/PointCloud2  (remapped → /velodyne_points)
       /imu/data        → sensor_msgs/Imu
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ------------------------------------------------------------------
    # 获取功能包路径
    # ------------------------------------------------------------------
    pkg_share = get_package_share_directory("outdoor_sim")

    # ------------------------------------------------------------------
    # 资源路径
    # ------------------------------------------------------------------
    world_file = os.path.join(pkg_share, "worlds", "grass_terrain.world")
    xacro_file = os.path.join(pkg_share, "urdf", "robot_sensors.xacro")

    # ------------------------------------------------------------------
    # Launch 参数
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_path = LaunchConfiguration("world", default=world_file)

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=world_file,
        description="Full path to the Ignition .world file to load",
    )

    # ------------------------------------------------------------------
    # 1. 启动 Ignition Gazebo (headless + 取消暂停)
    #    gz sim -s -r <world_file>
    #    -s = server only (headless)
    #    -r = run on start (unpause)
    # ------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            # 传递 world 路径给 gz sim，拼接为: gz sim -s -r <world>
            "gz_args": [" -s -r ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. robot_state_publisher —— 解析 xacro 发布 TF
    # ------------------------------------------------------------------
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": ParameterValue(
                Command(["xacro", " ", xacro_file]), value_type=str
            ),
        }],
    )

    # ------------------------------------------------------------------
    # 3. ros_gz_sim create —— 将机器人模型投入 Ignition 世界
    # ------------------------------------------------------------------
    create_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="create_outdoor_bot",
        output="screen",
        arguments=[
            "-name", "outdoor_bot",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5",
        ],
    )

    # ------------------------------------------------------------------
    # 4. ros_gz_bridge parameter_bridge —— Ignition → ROS 2 话题桥接
    #
    #    桥接映射：
    #      Ignition 话题           → ROS 2 类型                      → ROS 2 话题 (remapped)
    #      /clock                  → rosgraph_msgs/Clock             → /clock
    #      /lidar/points           → sensor_msgs/PointCloud2         → /velodyne_points
    #      /imu/data               → sensor_msgs/Imu                 → /imu/data
    #
    #    语法: <topic>@<ROS_type>[<Ignition_type>
    #          [ 表示 Ignition → ROS 单向桥接
    # ------------------------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/lidar/points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # ★ 核心新增：将 ROS2 端的 geometry_msgs/Twist 控制命令桥接到 Ignition 仿真端
            "/model/outdoor_bot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            # ★ 关节状态桥接：让 RViz2 能看到小车 3D 模型
            "/world/outdoor_flat_features_world/model/outdoor_bot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
        remappings=[
            ('/lidar/points/points', '/velodyne_points'),
            # ★ 核心重映射：允许在外部通过标准的 /cmd_vel 直接控制小车
            ('/model/outdoor_bot/cmd_vel', '/cmd_vel'),
            # ★ 关节状态重映射：RViz2 通过 /joint_states 驱动 TF 模型
            ('/world/outdoor_flat_features_world/model/outdoor_bot/joint_state', '/joint_states'),
        ],
        output="screen"
    )

    # ------------------------------------------------------------------
    # 组装 LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gz_sim,
        robot_state_pub,
        create_entity,
        bridge,
    ])
