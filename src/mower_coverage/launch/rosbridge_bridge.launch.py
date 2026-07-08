#!/usr/bin/env python3
"""
rosbridge_bridge.launch.py — Web 前端 ↔ ROS2 桥接

启动 rosbridge_websocket，让网页前端能通过 WebSocket 与 ROS2 通信。
同时启动一个简单的 HTTP 服务器托管前端页面。

前端画完多边形区域后，通过 rosbridge 发送到 ROS2：
  1. 区域坐标 → 调用 /multi_area 服务
  2. 触发规划 → /multi_area/plan
  3. 触发执行 → /multi_area/plan_and_start

依赖:
  - ros-humble-rosbridge-suite
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    websocket_port = LaunchConfiguration("websocket_port", default="9090")
    http_port = LaunchConfiguration("http_port", default="8080")

    # Web 前端文件路径（在同级 web_frontend 目录）
    web_dir = os.path.join(
        os.path.dirname(__file__), "..", "web_frontend"
    )
    web_dir = os.path.abspath(web_dir)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("websocket_port", default_value="9090"),
        DeclareLaunchArgument("http_port", default_value="8080"),

        # =============================================================
        # rosbridge_websocket: WebSocket 服务器
        # 网页前端通过此端口与 ROS2 通信
        # =============================================================
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            parameters=[{
                "port": websocket_port,
                "use_sim_time": use_sim_time,
            }],
        ),

        # =============================================================
        # HTTP 服务器: 托管 Web 前端页面
        # 浏览器访问 http://localhost:{http_port} 打开前端
        # =============================================================
        Node(
            package="mower_coverage",
            executable="web_server",
            name="web_frontend_server",
            parameters=[{
                "port": http_port,
                "web_dir": web_dir,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
