#!/usr/bin/env python3
"""
odom_to_tf.py — 将 /odom (nav_msgs/Odometry) 中的位姿实时转发为 /tf

问题背景：
  Ignition Fortress 的 DiffDrive 插件通过 <update_odom_to_tf>true</update_odom_to_tf>
  在 Ignition 端发布 Pose_V 到 /tf 话题。ros_gz_bridge 将此桥接到 ROS 2 /tf，
  但在实测中发现桥接后的 TF 不更新（卡在初始化时刻的位姿）。

  单独桥接的 /odom 话题（nav_msgs/Odometry）更新正常（20Hz），
  因此本节点订阅 /odom，将其中的 pose 重新广播为 ROS 2 TF，
  绕过桥接层可能存在的丢帧/卡顿问题。
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅来自 Ignition 桥的 /odom (Odometry)
        # QoS 匹配 Ignition 桥的行为：BEST_EFFORT + VOLATILE
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos
        )

        self.get_logger().info("odom_to_tf 中继节点已启动 — 将 /odom 位姿转发为 /tf")

        # ★ 反转 Yaw 配置：DiffDrive 里程计自转方向反向时，对四元数取共轭
        self.declare_parameter("reverse_yaw", True)
        self.reverse_yaw = self.get_parameter("reverse_yaw").value

        # --- 时间戳单调性跟踪 (丢弃乱序包，保护 TF 树) ---
        self.last_time_sec = 0
        self.last_time_nanosec = 0

    def odom_callback(self, msg: Odometry):
        # 1. NaN 防御
        import math
        if math.isnan(msg.pose.pose.position.x):
            return

        # 2. 时间戳严格单调性过滤
        current_sec = msg.header.stamp.sec
        current_nanosec = msg.header.stamp.nanosec

        if current_sec < self.last_time_sec - 1:
            self.get_logger().warn("检测到 Odom 时钟重置，重新同步！")
            self.last_time_sec = current_sec
            self.last_time_nanosec = current_nanosec
        elif current_sec < self.last_time_sec or (current_sec == self.last_time_sec and current_nanosec <= self.last_time_nanosec):
            return

        self.last_time_sec = current_sec
        self.last_time_nanosec = current_nanosec

        # 3. 构造 TF 变换
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id

        # ★ 核心参数：根据诊断结果，设置是否需要反转自转方向
        REVERSE_YAW = False  # 如果情况 A（没反），设为 False；如果情况 B（确实反了），设为 True

        if REVERSE_YAW:
            # 镜像空间修复：翻转 Yaw 角的同时，必须同步翻转 Y 轴位移，保持 SE(2) 群数学一致性！
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = -msg.pose.pose.position.y  # 翻转 Y 轴位移
            t.transform.translation.z = msg.pose.pose.position.z

            t.transform.rotation.x = -msg.pose.pose.orientation.x
            t.transform.rotation.y = -msg.pose.pose.orientation.y
            t.transform.rotation.z = -msg.pose.pose.orientation.z
            t.transform.rotation.w = msg.pose.pose.orientation.w
        else:
            # 正常模式：完全信任原始里程计的旋转与平移
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation

        # 4. 广播 TF
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
