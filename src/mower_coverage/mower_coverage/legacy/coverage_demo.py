#!/usr/bin/env python3
"""
coverage_demo.py — 全覆盖路径规划演示节点

整合 area_definer → boustrophedon_planner → path_executor 的流水线。

用法：
  1. 启动仿真:        ros2 launch outdoor_sim sim_launch.py
  2. 启动 SLAM+导航:   ros2 launch outdoor_sim slam_nav2_launch.py
  3. 启动本演示:       ros2 run mower_coverage coverage_demo
  4. 在 RViz 中查看区域和路径

流程：
  1. 加载预定义区域（或使用默认矩形）
  2. 调用规划器生成牛耕式路径
  3. 发布路径到 /coverage/path
  4. 启动 path_executor 执行
"""

import os
import yaml
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from shapely.geometry import Polygon as ShapelyPolygon, box

from mower_coverage.boustrophedon_planner import BoustrophedonPlanner
from mower_coverage.area_definer import AreaDefiner


class CoverageDemo(Node):
    """全覆盖路径规划演示节点"""

    def __init__(self):
        super().__init__('coverage_demo')

        # 参数
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('cutting_width', 0.5)
        self.declare_parameter('overlap', 0.1)
        self.declare_parameter('path_angle', 0.0)
        self.declare_parameter('use_default_area', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.cutting_width = self.get_parameter('cutting_width').value
        self.overlap = self.get_parameter('overlap').value
        self.path_angle = self.get_parameter('path_angle').value

        # 规划器
        self.planner = BoustrophedonPlanner(
            cutting_width=self.cutting_width,
            overlap=self.overlap,
            angle=self.path_angle
        )

        # 区域定义（使用 area_definer 加载或默认）
        self.area_polygon = None
        self.obstacles = []

        # 发布器
        self.path_pub = self.create_publisher(Path, '/coverage/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage/path_markers', 10)

        # 服务
        self.srv_plan = self.create_service(Trigger, '/coverage/plan', self.plan_callback)
        self.srv_plan_and_start = self.create_service(Trigger, '/coverage/plan_and_start',
                                                       self.plan_and_start_callback)

        # 是否使用默认区域
        if self.get_parameter('use_default_area').value:
            self.setup_default_area()

        self.get_logger().info('全覆盖演示节点已启动')
        self.get_logger().info(f'  割幅: {self.cutting_width}m, 重叠: {self.overlap*100:.0f}%')
        self.get_logger().info('  Services: /coverage/plan, /coverage/plan_and_start')

    def setup_default_area(self):
        """设置默认矩形区域 (50m×50m 草坪)"""
        self.get_logger().info('使用默认区域: 50m × 50m 矩形')

        # 主区域
        self.area_polygon = box(-23, -23, 23, 23)

        # 内岛障碍物（模拟树/花坛）
        self.obstacles = [
            box(-8, -8, -4, -4),     # 左下障碍物
            box(5, 5, 8, 7),         # 右上障碍物
            box(-15, 10, -12, 12),   # 左上障碍物
            box(10, -12, 13, -9),    # 右下障碍物
        ]

        self.get_logger().info(f'  区域: {23*2}m × {23*2}m')
        self.get_logger().info(f'  障碍物: {len(self.obstacles)} 个')

    def plan_callback(self, request, response):
        """生成规划路径"""
        if self.area_polygon is None:
            response.success = False
            response.message = "未设置作业区域"
            return response

        try:
            path = self.planner.plan(self.area_polygon, obstacles=self.obstacles)
            self._publish_path(path)
            self._publish_path_markers(path)

            total_dist = sum(math.dist(path[i], path[i+1]) for i in range(len(path)-1))
            self.get_logger().info(
                f'✅ 规划完成: {len(path)} 个路径点, '
                f'总距离 {total_dist:.1f}m'
            )

            response.success = True
            response.message = f'规划完成: {len(path)} 个点, {total_dist:.1f}m'
        except Exception as e:
            self.get_logger().error(f'规划失败: {e}')
            response.success = False
            response.message = str(e)

        return response

    def plan_and_start_callback(self, request, response):
        """规划并开始执行"""
        # 先规划
        plan_response = self.plan_callback(request, response)
        if not plan_response.success:
            return plan_response

        # 然后启动执行器
        client = self.create_client(Trigger, '/coverage/start')
        if not client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = '路径执行器未启动'
            return response

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('路径执行器已启动')
            response.message += ' + 开始执行'
        else:
            response.message += ' (但启动执行失败)'

        return response

    def _publish_path(self, waypoints):
        """发布为 nav_msgs/Path"""
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info(f'路径已发布到 /coverage/path ({len(waypoints)} 个点)')

    def _publish_path_markers(self, waypoints):
        """发布路径 Marker 用于 RViz 可视化"""
        markers = MarkerArray()

        # 路径线
        line = Marker()
        line.header.frame_id = self.frame_id
        line.ns = 'planned_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.08
        line.color.r = 0.0
        line.color.g = 0.5
        line.color.b = 1.0
        line.color.a = 1.0

        for x, y in waypoints:
            line.points.append(Point(x=x, y=y, z=0.02))
        markers.markers.append(line)

        # 方向箭头（每 10 个点显示一个方向）
        for i in range(0, len(waypoints) - 1, 10):
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i+1]
            dx, dy = x2 - x1, y2 - y1
            angle = math.atan2(dy, dx)

            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.ns = 'path_arrows'
            arrow.id = i // 10
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x1
            arrow.pose.position.y = y1
            arrow.pose.position.z = 0.03
            arrow.pose.orientation.w = math.cos(angle / 2)
            arrow.pose.orientation.z = math.sin(angle / 2)
            arrow.scale.x = 0.3
            arrow.scale.y = 0.08
            arrow.scale.z = 0.08
            arrow.color.r = 0.0
            arrow.color.g = 0.8
            arrow.color.b = 1.0
            arrow.color.a = 0.6
            markers.markers.append(arrow)

        self.marker_pub.publish(markers)
        self.get_logger().info(f'已发布 {len(markers.markers)} 个可视化Marker')


def main(args=None):
    rclpy.init(args=args)
    node = CoverageDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
