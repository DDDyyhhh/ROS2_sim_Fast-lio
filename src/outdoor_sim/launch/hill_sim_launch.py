#!/usr/bin/env python3
"""
hill_sim_launch.py — 斜坡地形仿真启动文件 (Ignition Fortress / ros_gz)

功能：
  1. 启动 Ignition Gazebo，加载 hill_terrain_30x30.world (起伏地形)
  2. 启动 robot_state_publisher，解析 robot_with_gps.urdf 发布 TF 树
  3. 运行 ros_gz_sim create，将机器人模型投入 Ignition 世界
  4. 运行 ros_gz_bridge，桥接 ROS2 ↔ Ignition 话题
     （含 Mid-360 LiDAR + RTK GPS 传感器桥接）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    world_file = os.path.join(pkg_share, "worlds", "hill_terrain_30x30.world")
    urdf_file = os.path.join(pkg_share, "urdf", "robot_with_gps.urdf")

    # ------------------------------------------------------------------
    # Launch 参数
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_path = LaunchConfiguration("world", default=world_file)
    with_odom_to_tf = LaunchConfiguration("with_odom_to_tf", default="true")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic", default="/cmd_vel")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=world_file,
        description="Full path to the .world file to load",
    )
    declare_with_odom_to_tf = DeclareLaunchArgument(
        "with_odom_to_tf", default_value="true",
        description="转发 /odom 为 TF；EKF profile 应关闭以避免重复 odom→body",
    )
    declare_cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic", default_value="/cmd_vel",
        description="ROS command topic bridged to the simulation only",
    )

    # ------------------------------------------------------------------
    # 1. 启动 Ignition Gazebo (headless + 取消暂停)
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
            "gz_args": [" -s -r ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. robot_state_publisher —— 解析 URDF 发布 TF
    #    注意：robot_with_gps.urdf 是独立 URDF（非 xacro），可直接加载
    # ------------------------------------------------------------------
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        arguments=['--ros-args', '--log-level', 'ERROR'],
        parameters=[{
            "use_sim_time": True,
            "robot_description": ParameterValue(
                Command(["cat ", urdf_file]), value_type=str
            ),
        }],
    )

    # ------------------------------------------------------------------
    # 3. ros_gz_sim create —— 将机器人投入 Ignition 世界
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
            "-z", "1.0",        # 斜坡地形起点略高
        ],
    )

    # ------------------------------------------------------------------
    # 4. ros_gz_bridge —— Ignition → ROS2 话题桥接
    #    斜坡世界名称: hill_terrain_30x30
    # ------------------------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": False}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/lidar/points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/model/outdoor_bot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            # ★ GPS 桥接：NavSat → NavSatFix
            "/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
            # 关节状态
            "/world/hill_terrain_30x30/model/outdoor_bot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
        ],
        remappings=[
            ('/clock', '/clock_raw'),
            ('/lidar/points/points', '/velodyne_points'),
            ('/model/outdoor_bot/cmd_vel', cmd_vel_topic),
            ('/gps/fix', '/gps/fix'),
            ('/world/hill_terrain_30x30/model/outdoor_bot/joint_state', '/joint_states'),
        ],
        output="screen"
    )

    # ------------------------------------------------------------------
    # 5. 时钟滤波器
    # ------------------------------------------------------------------
    clock_filter_node = Node(
        package="outdoor_sim",
        executable="clock_filter.py",
        name="clock_filter",
        output="screen",
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # 6. TF 中继：odom → /tf
    # ------------------------------------------------------------------
    odom_to_tf_node = Node(
        package="outdoor_sim",
        executable="odom_to_tf.py",
        name="odom_to_tf",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(with_odom_to_tf),
    )

    # ------------------------------------------------------------------
    # 7. map → odom 静态 TF（供 RViz 使用）
    # ------------------------------------------------------------------
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_with_odom_to_tf,
        declare_cmd_vel_topic,
        gz_sim,
        robot_state_pub,
        create_entity,
        bridge,
        clock_filter_node,
        odom_to_tf_node,
        map_to_odom,
    ])
