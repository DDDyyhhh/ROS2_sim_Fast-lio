#!/usr/bin/env python3
"""
area_definer.py — 割草区域定义节点

功能：
  1. 通过 RViz 2D Nav Goal 类似的交互方式框定区域
  2. 从 YAML 文件加载/保存区域多边形
  3. 发布区域可视化 MarkerArray 用于 RViz 显示
  4. 通过服务请求提供当前区域定义给规划器

使用方式：
  ros2 run mower_coverage area_definer
  ros2 service call /define_area mower_coverage/srv/DefineArea "{polygon: [{x: -10, y: -10}, {x: 10, y: -10}, ...]}"
  ros2 service call /save_area std_srvs/srv/Trigger
  ros2 service call /load_area std_srvs/srv/Trigger
"""

import os
import yaml
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, PolygonStamped, Polygon as RosPolygon
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from shapely.geometry import Polygon as ShapelyPolygon


class AreaDefiner(Node):
    """区域定义节点"""

    def __init__(self):
        super().__init__('area_definer')

        # 参数
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('area_file', os.path.expanduser('~/mowing_area.yaml'))
        self.declare_parameter('area_color_r', 0.0)
        self.declare_parameter('area_color_g', 1.0)
        self.declare_parameter('area_color_b', 0.0)
        self.declare_parameter('area_color_a', 0.5)

        self.frame_id = self.get_parameter('frame_id').value
        self.area_file = self.get_parameter('area_file').value

        # 当前区域多边形 (外边界)
        self.outer_boundary: List[Tuple[float, float]] = []
        # 内岛障碍物列表
        self.inner_holes: List[List[Tuple[float, float]]] = []

        # 发布器
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage/area_markers', qos)
        self.polygon_pub = self.create_publisher(PolygonStamped, '/coverage/area_polygon', qos)

        # 服务 (PolygonSetter 服务暂时禁用，使用话题接口替代)
        # 简化: 用 Trigger 做占位，实际通过话题 /coverage/area_polygon 传递
        self.srv_set_area = self.create_service(Trigger, '/coverage/set_area',
                                                 self.set_area_dummy)
        self.srv_save_area = self.create_service(Trigger, '/coverage/save_area',
                                                  self.save_area_callback)
        self.srv_load_area = self.create_service(Trigger, '/coverage/load_area',
                                                  self.load_area_callback)
        self.srv_clear_area = self.create_service(Trigger, '/coverage/clear_area',
                                                   self.clear_area_callback)

        # 定时发布可视化
        self.create_timer(1.0, self.publish_visualization)

        self.get_logger().info('区域定义节点已启动')
        self.get_logger().info(f'  frame_id: {self.frame_id}')
        self.get_logger().info(f'  area_file: {self.area_file}')
        self.get_logger().info('  Services: /coverage/set_area, /coverage/save_area, /coverage/load_area')

    def set_area_dummy(self, request, response):
        """占位：设置区域多边形（通过话题 /coverage/area_polygon 实际传递）"""
        if not self.outer_boundary:
            response.success = False
            response.message = "未设置区域，请先发布 /coverage/area_polygon"
            return response

        n = len(self.outer_boundary)
        self._publish_polygon()
        self.get_logger().info(f'已设置区域多边形 ({n} 个顶点)')
        response.success = True
        response.message = f'当前区域 {n} 个顶点'
        return response

    def save_area_callback(self, request, response):
        """保存区域到 YAML 文件"""
        if not self.outer_boundary:
            response.success = False
            response.message = "区域为空，无法保存"
            return response

        data = {
            'frame_id': self.frame_id,
            'outer_boundary': [[float(x), float(y)] for x, y in self.outer_boundary],
            'inner_holes': [[[float(x), float(y)] for x, y in hole]
                           for hole in self.inner_holes]
        }

        try:
            with open(self.area_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            self.get_logger().info(f'区域已保存到 {self.area_file}')
            response.success = True
            response.message = f'已保存到 {self.area_file}'
        except Exception as e:
            self.get_logger().error(f'保存失败: {e}')
            response.success = False
            response.message = str(e)
        return response

    def load_area_callback(self, request, response):
        """从 YAML 文件加载区域"""
        if not os.path.exists(self.area_file):
            response.success = False
            response.message = f'文件不存在: {self.area_file}'
            return response

        try:
            with open(self.area_file, 'r') as f:
                data = yaml.safe_load(f)

            if 'outer_boundary' not in data:
                raise ValueError('YAML 文件中缺少 outer_boundary')

            self.outer_boundary = [(p[0], p[1]) for p in data['outer_boundary']]
            self.inner_holes = [[(p[0], p[1]) for p in hole]
                               for hole in data.get('inner_holes', [])]
            self.frame_id = data.get('frame_id', 'map')

            self._publish_polygon()
            n = len(self.outer_boundary)
            self.get_logger().info(f'已加载区域 ({n} 个顶点)')
            response.success = True
            response.message = f'已加载 {n} 个顶点的多边形'
        except Exception as e:
            self.get_logger().error(f'加载失败: {e}')
            response.success = False
            response.message = str(e)
        return response

    def clear_area_callback(self, request, response):
        """清除当前区域"""
        self.outer_boundary = []
        self.inner_holes = []
        self._publish_empty_markers()
        self.get_logger().info('区域已清除')
        response.success = True
        response.message = '区域已清除'
        return response

    def get_shapely_polygon(self) -> ShapelyPolygon:
        """返回 shapely Polygon（供规划器使用）"""
        if not self.outer_boundary:
            return None
        holes = [hole for hole in self.inner_holes if len(hole) >= 3]
        return ShapelyPolygon(self.outer_boundary, holes)

    def publish_visualization(self):
        """定时发布 RViz 可视化 Marker"""
        if not self.outer_boundary:
            return

        markers = MarkerArray()

        # 区域填充
        fill_marker = Marker()
        fill_marker.header.frame_id = self.frame_id
        fill_marker.ns = 'area'
        fill_marker.id = 0
        fill_marker.type = Marker.TRIANGLE_LIST
        fill_marker.action = Marker.ADD
        fill_marker.pose.orientation.w = 1.0
        fill_marker.scale.x = 1.0
        fill_marker.scale.y = 1.0
        fill_marker.scale.z = 1.0
        fill_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.2)

        # 简单三角剖分（对凸多边形有效）
        pts = self.outer_boundary
        if len(pts) >= 3:
            for i in range(1, len(pts) - 1):
                p0 = Point(x=pts[0][0], y=pts[0][1], z=0.0)
                p1 = Point(x=pts[i][0], y=pts[i][1], z=0.0)
                p2 = Point(x=pts[i+1][0], y=pts[i+1][1], z=0.0)
                fill_marker.points.extend([p0, p1, p2])
        markers.markers.append(fill_marker)

        # 边界线
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.ns = 'area'
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.08  # 线宽
        line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        for x, y in pts:
            line_marker.points.append(Point(x=x, y=y, z=0.01))
        # 闭合
        line_marker.points.append(Point(x=pts[0][0], y=pts[0][1], z=0.01))
        markers.markers.append(line_marker)

        # 顶点标记
        for i, (x, y) in enumerate(pts):
            v_marker = Marker()
            v_marker.header.frame_id = self.frame_id
            v_marker.ns = 'area_vertices'
            v_marker.id = i
            v_marker.type = Marker.SPHERE
            v_marker.action = Marker.ADD
            v_marker.pose.position.x = x
            v_marker.pose.position.y = y
            v_marker.pose.position.z = 0.02
            v_marker.pose.orientation.w = 1.0
            v_marker.scale.x = 0.2
            v_marker.scale.y = 0.2
            v_marker.scale.z = 0.2
            v_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            if i == 0:  # 起始顶点用红色标记
                v_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            markers.markers.append(v_marker)

        self.marker_pub.publish(markers)

    def _publish_polygon(self):
        """发布多边形到话题"""
        msg = PolygonStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.outer_boundary:
            p = Point(x=x, y=y, z=0.0)
            msg.polygon.points.append(p)
        self.polygon_pub.publish(msg)

    def _publish_empty_markers(self):
        """发布空的 MarkerArray 隐藏旧标记"""
        markers = MarkerArray()
        for i in range(2):
            m = Marker()
            m.action = Marker.DELETEALL
            markers.markers.append(m)
        self.marker_pub.publish(markers)

    def _is_polygon_valid(self, pts: List[Tuple[float, float]]) -> bool:
        """检查多边形是否有效"""
        if len(pts) < 3:
            return False
        polygon = ShapelyPolygon(pts)
        return polygon.is_valid and not polygon.is_empty


def main(args=None):
    rclpy.init(args=args)
    node = AreaDefiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
