#!/usr/bin/env python3
"""
coverage_monitor.py — 覆盖率监控与可视化节点

功能：
  1. 跟踪已覆盖区域（刀盘经过的地方）
  2. 计算已完成覆盖率百分比
  3. 发布覆盖热力图 MarkerArray
  4. 发布覆盖率统计数据

用法：
  ros2 run mower_coverage coverage_monitor
  ros2 topic echo /coverage/statistics
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, ColorRGBA, Float32
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class CoverageMonitor(Node):
    """覆盖率监控"""

    def __init__(self):
        super().__init__('coverage_monitor')

        # 参数
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('cutting_width', 0.5)   # 割幅
        self.declare_parameter('grid_resolution', 0.2)  # 栅格分辨率

        self.frame_id = self.get_parameter('frame_id').value
        self.cutting_width = self.get_parameter('cutting_width').value
        self.resolution = self.get_parameter('grid_resolution').value

        # 覆盖栅格图 (以刀盘经过的轨迹点膨胀为割幅宽度的圆)
        self._coverage_map = {}  # key: (gx, gy), value: True
        self._robot_path: list = []

        # 发布器
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage/coverage_map', 10)
        self.coverage_percent_pub = self.create_publisher(Float32, '/coverage/percentage', 10)
        self.stat_pub = self.create_publisher(Float32, '/coverage/statistics', 10)

        # 订阅器
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 定期发布
        self.create_timer(1.0, self.publish_coverage)
        self.create_timer(5.0, self.publish_statistics)

        self.get_logger().info('覆盖率监控已启动')

    def odom_callback(self, msg: Odometry):
        """记录机器人位置，标记已覆盖区域"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._robot_path.append((x, y))

        # 以割幅宽度为准，标记栅格
        radius_cells = int(math.ceil(self.cutting_width / 2 / self.resolution))
        cx = int(x / self.resolution)
        cy = int(y / self.resolution)

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx*dx + dy*dy <= radius_cells*radius_cells:
                    self._coverage_map[(cx + dx, cy + dy)] = True

    def publish_coverage(self):
        """发布覆盖热力图"""
        markers = MarkerArray()

        if not self._coverage_map:
            return

        # 覆盖点云（用 Cube 或 Points 表示）
        covered = Marker()
        covered.header.frame_id = self.frame_id
        covered.ns = 'coverage'
        covered.id = 0
        covered.type = Marker.CUBE_LIST
        covered.action = Marker.ADD
        covered.pose.orientation.w = 1.0
        covered.scale.x = self.resolution
        covered.scale.y = self.resolution
        covered.scale.z = 0.05
        covered.color = ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.4)

        for (gx, gy) in self._coverage_map.keys():
            covered.points.append(Point(
                x=gx * self.resolution,
                y=gy * self.resolution,
                z=0.01
            ))
        markers.markers.append(covered)

        # 机器人轨迹
        if len(self._robot_path) > 1:
            path_line = Marker()
            path_line.header.frame_id = self.frame_id
            path_line.ns = 'coverage_path'
            path_line.id = 1
            path_line.type = Marker.LINE_STRIP
            path_line.action = Marker.ADD
            path_line.pose.orientation.w = 1.0
            path_line.scale.x = self.cutting_width * 0.8
            path_line.color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=0.3)
            for x, y in self._robot_path[::5]:  # 每5个点取一个（减少数据量）
                path_line.points.append(Point(x=x, y=y, z=0.0))
            markers.markers.append(path_line)

        self.marker_pub.publish(markers)

    def publish_statistics(self):
        """发布覆盖率统计"""
        if not self._coverage_map:
            return

        total_coverage = len(self._coverage_map) * self.resolution * self.resolution
        msg = Float32()
        msg.data = float(total_coverage)
        self.coverage_percent_pub.publish(msg)

        # Log
        if len(self._robot_path) > 0:
            distance = sum(
                math.dist(self._robot_path[i], self._robot_path[i-1])
                for i in range(1, len(self._robot_path))
            )
            self.get_logger().info(
                f'已覆盖: {total_coverage:.1f}m² | '
                f'行驶距离: {distance:.1f}m | '
                f'栅格数: {len(self._coverage_map)}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
