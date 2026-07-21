#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockFilter(Node):
    def __init__(self):
        super().__init__("clock_filter")
        self.last_sec = 0
        self.last_nanosec = 0

        qos = rclpy.qos.QoSProfile(depth=10)
        # 订阅桥接过来的"脏"时钟
        self.sub = self.create_subscription(Clock, "/clock_raw", self.clock_cb, qos)
        # 发布纯净的单调时钟
        self.pub = self.create_publisher(Clock, "/clock", qos)
        self.get_logger().info("全局时钟单调滤波器已启动 (防 TF 塌缩)")

    def clock_cb(self, msg: Clock):
        curr_sec = msg.clock.sec
        curr_nano = msg.clock.nanosec

        # 如果时间倒退超过 1 秒，说明仿真器重启了，释放锁
        if curr_sec < self.last_sec - 1:
            self.last_sec = curr_sec
            self.last_nanosec = curr_nano
            self.pub.publish(msg)
            self.get_logger().warn("检测到仿真时钟重置，重新同步！")
        # 否则严格要求时间单调递增
        elif curr_sec > self.last_sec or (curr_sec == self.last_sec and curr_nano > self.last_nanosec):
            self.pub.publish(msg)
            self.last_sec = curr_sec
            self.last_nanosec = curr_nano


def main():
    rclpy.init()
    node = ClockFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass


if __name__ == "__main__":
    main()
