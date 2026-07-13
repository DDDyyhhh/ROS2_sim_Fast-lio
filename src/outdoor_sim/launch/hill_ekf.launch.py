#!/usr/bin/env python3
"""
hill_ekf.launch.py — 斜坡场景 RTK+LiDAR 融合定位

启动内容:
  1. navsat_transform_node: /gps/fix (经纬度) → /odom/gps (UTM坐标)
  2. ekf_node: 融合 FAST-LIO (/Odometry) + IMU + GPS → /odometry/filtered
  3. 静态 TF: camera_init → odom（FAST-LIO 坐标系对齐）
  4. 静态 TF: map → odom（地图坐标系对齐）

依赖:
  - ros-humble-robot-localization
  - FAST-LIO 正在运行，发布 /Odometry
  - 仿真正在运行，发布 /gps/fix, /imu/data

★ 参数直接写在 launch 文件里（不通过 YAML 文件），避免 ROS2 launch 系统重复传参导致崩溃
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # ============ ekf_node 参数 ============
    ekf_params = {
        "frequency": 30.0,
        "sensor_timeout": 0.1,
        "two_d_mode": False,              # ❗斜坡 → 3D 模式
        "publish_acceleration": False,
        "world_frame": "odom",
        "base_link_frame": "body",

        # FAST-LIO 里程计（差分模式，相对 SLAM）
        "odom0": "/Odometry",
        "odom0_config": [True, True, True, True, True, True,
                         False, False, False, False, False, False,
                         False, False, False, False, False, False],
        "odom0_differential": True,
        "odom0_relative": False,
        "odom0_queue_size": 10,

        # IMU — 只融合姿态角 + 角速度
        "imu0": "/imu/data",
        "imu0_config": [False, False, False, True, True, True,
                        False, False, False, True, True, True,
                        False, False, False, False, False, False],
        "imu0_differential": False,
        "imu0_relative": True,
        "imu0_queue_size": 10,
        "imu0_remove_gravitational_acceleration": True,

        # GPS 里程计（navsat_transform 输出）
        "odom1": "/odom/gps",
        "odom1_config": [True, True, False, False, False, True,
                         False, False, False, False, False, False,
                         False, False, False, False, False, False],
        "odom1_differential": False,
        "odom1_relative": False,
        "odom1_queue_size": 10,
    }

    # ============ navsat_transform_node 参数 ============
    navsat_params = {
        "frequency": 30.0,
        "delay": 3.0,
        "transform_timeout": 0.2,
        "magnetic_declination_radians": 0.0,
        "yaw_offset": 0.0,
        "zero_altitude": True,
        "publish_filtered_gps": True,
        "use_odometry_yaw": False,
        "wait_for_datum": True,
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # =============================================================
        # 1. FAST-LIO 坐标系对齐: camera_init → odom
        #    FAST-LIO 使用 camera_init 作为世界坐标系，
        #    EKF 使用 odom 作为世界坐标系，需要静态 TF 转换
        # =============================================================
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_init_to_odom",
            arguments=["0", "0", "0", "0", "0", "0",
                       "camera_init", "odom"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # =============================================================
        # 2. map → odom 静态 TF（EKF 输出的 world_frame 是 odom）
        # =============================================================
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static",
            arguments=["0", "0", "0", "0", "0", "0",
                       "map", "odom"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # =============================================================
        # 3. navsat_transform_node: GPS 经纬度 → UTM 里程计
        #    订阅 /gps/fix → 发布 /odom/gps
        # =============================================================
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            parameters=[navsat_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/imu", "/imu/data"),
                ("/gps/fix", "/gps/fix"),
                ("/odometry/filtered", "/odometry/filtered"),
                ("/odometry/gps", "/odom/gps"),
            ],
        ),

        # =============================================================
        # 4. ekf_node: 融合里程计 → 最优位姿
        #    订阅 /Odometry (FAST-LIO) + /imu/data + /odom/gps
        #    发布 /odometry/filtered
        #    ★ ROS2 Humble 中可执行文件名是 ekf_node（不是 ekf_localization_node）
        # =============================================================
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_localization",
            parameters=[ekf_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/odometry/filtered", "/odometry/filtered"),
            ],
        ),
    ])
