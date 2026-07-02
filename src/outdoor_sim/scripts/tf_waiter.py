#!/usr/bin/env python3
"""
tf_waiter.py — 等待 FAST-LIO 发布 camera_init→body TF 变换

用法：
  在 slam_nav2_launch.py 中以此节点为门控：只有等待到 TF 变换可用后，
  才启动 Nav2 导航堆栈，确保 TF 树完整（map→odom→camera_init→body）。

退出条件：
  - 成功：检测到 camera_init→body 变换可用，exit 0
  - 超时：30 秒后仍不可用，打印警告并 exit 0（不阻塞 Nav2 启动）
"""

import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TFWaiter(Node):
    def __init__(self):
        super().__init__("tf_waiter")

        # use_sim_time 由 launch file 通过 SetParameter 传入，不在此重复声明
        # timeout 参数声明（带默认值）
        self.declare_parameter("timeout", 30.0)
        self.timeout = self.get_parameter("timeout").value

        # TF 缓冲区 — 自动订阅 /tf 和 /tf_static
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 要等待的 transform：camera_init → body
        self.target_parent = "camera_init"
        self.target_child = "body"

        self.get_logger().info(
            f"tf_waiter 已启动 (timeout={self.timeout:.0f}s)"
        )

    def run(self):
        """轮询等待目标 TF 变换，返回 True=成功 False=超时"""
        rate = self.create_rate(5.0)  # 5 Hz 轮询
        elapsed = 0.0
        step = 0.2  # 每步约 0.2s

        self.get_logger().info(
            f"等待 TF 变换 '{self.target_parent}' → '{self.target_child}' "
            f"（超时 {self.timeout:.0f}s）..."
        )

        while rclpy.ok() and elapsed < self.timeout:
            rclpy.spin_once(self, timeout_sec=step)

            try:
                if self.tf_buffer.can_transform(
                    self.target_child, self.target_parent, rclpy.time.Time()
                ):
                    # 变换可用！
                    ts = self.tf_buffer.lookup_transform(
                        self.target_child, self.target_parent, rclpy.time.Time()
                    )
                    self.get_logger().info(
                        f"✅ TF 变换可用！"
                        f" '{self.target_parent}' → '{self.target_child}' @ "
                        f"t={ts.header.stamp.sec}.{ts.header.stamp.nanosec:09d}"
                    )
                    return True
            except Exception:
                # can_transform 可能抛异常（空树等），忽略继续等
                pass

            elapsed += step

        # 超时
        self.get_logger().warn(
            f"⚠️ 等待 TF 变换 '{self.target_parent}' → '{self.target_child}' "
            f"超时（{self.timeout:.0f}s），继续启动 Nav2..."
        )
        return False


def main(args=None):
    rclpy.init(args=args)
    node = TFWaiter()
    success = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
