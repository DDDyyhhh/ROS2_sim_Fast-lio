#!/usr/bin/env python3
"""
multi_area_definer.py — 多区域定义节点

功能：
  - 管理多个割草区域（每个含内岛）
  - 支持 YAML 加载/保存
  - 支持 RViz PublishPoint 交互画区域
  - RViz Marker 可视化
  - 提供区域管理服务

区域数据格式：
  {
    "name": str,
    "points": [[x1,y1], ...],        # 外边界
    "inner_rings": [[[x1,y1], ...]],  # 内岛列表（可选）
    "color": [r, g, b],              # 显示颜色
    "cutting_angle": float,          # 牛耕式角度
    "max_speed": float,              # 区域最大速度
  }

使用方式：
  ros2 run mower_coverage multi_area_definer
"""

import os
import yaml
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import ColorRGBA, Header, String as StringMsg
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from shape_msgs.msg import Mesh

from shapely.geometry import Polygon, Point as ShapelyPoint

from mower_coverage.state_paths import ensure_parent, readable_path, state_file
from .loader import load_mission_file, mission_to_legacy_areas


class MultiAreaDefiner(Node):
    """多区域定义节点"""

    def __init__(self):
        super().__init__('multi_area_definer')

        # 参数（检查是否已被 ROS 基础设施声明）
        for param_name, default in [('frame_id', 'map'),
                                     ('area_file', state_file('mowing_areas.yaml'))]:
            if not self.has_parameter(param_name):
                self.declare_parameter(param_name, default)

        self.frame_id = self.get_parameter('frame_id').value
        self.area_file = self.get_parameter('area_file').value

        # 状态
        self.areas = {}           # name → area dict
        self.drawing_points = []   # 当前绘制中的点列表
        self.drawing_name = None   # 当前绘制区域名
        self.color_cycle = [
            (0.0, 0.8, 0.0),     # 绿色
            (0.0, 0.5, 1.0),     # 蓝色
            (1.0, 0.8, 0.0),     # 黄色
            (1.0, 0.4, 0.0),     # 橙色
            (0.8, 0.0, 0.8),     # 紫色
        ]
        self.next_color_idx = 0

        # --- 发布者 ---
        self.marker_pub = self.create_publisher(
            MarkerArray, '/multi_area/visualization', 10)
        self.debug_pub = self.create_publisher(
            Marker, '/multi_area/debug_point', 10)

        # --- 订阅者：RViz PublishPoint ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)
        self.click_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_callback, qos)

        # --- 服务 ---

        # 添加/完成/取消绘制
        self.srv_start_draw = self.create_service(
            Trigger, '/multi_area/start_draw', self.start_draw_cb)
        self.srv_finish = self.create_service(
            Trigger, '/multi_area/finish_polygon', self.finish_polygon_cb)
        self.srv_cancel = self.create_service(
            Trigger, '/multi_area/cancel_polygon', self.cancel_polygon_cb)

        # 区域管理
        self.srv_list = self.create_service(
            Trigger, '/multi_area/list_areas', self.list_areas_cb)
        self.srv_remove = self.create_service(
            Trigger, '/multi_area/remove_area', self.remove_area_cb)
        self.srv_clear = self.create_service(
            Trigger, '/multi_area/clear_all', self.clear_all_cb)

        # 持久化
        self.srv_save = self.create_service(
            Trigger, '/multi_area/save', self.save_cb)
        self.srv_load = self.create_service(
            Trigger, '/multi_area/load', self.load_cb)

        # --- 订阅 Web 前端发来的区域（JS→JSON→std_msgs/String） ---
        self.web_area_sub = self.create_subscription(
            StringMsg, '/web/areas', self.web_areas_callback, 10)

        # GPS 原点（用于从卫星地图 GPS 转本地坐标）
        # 初始中心与 app.js 中的 CONFIG.initialCenter 保持一致
        self._gps_origin_lat = 22.5431
        self._gps_origin_lng = 114.0579

        # 启动定时可视化
        self.timer = self.create_timer(1.0, self.publish_visualization)

        self.get_logger().info('多区域定义节点已启动')
        self.get_logger().info(f'  区域文件: {self.area_file}')
        self.get_logger().info('  在 RViz 中用 Publish Point 点击地图画点')
        self.get_logger().info('  服务: /multi_area/{start_draw, finish_polygon, cancel_polygon, save, load, list_areas, remove_area, clear_all}')

    # ==============================================================
    # Web 前端区域回调（GPS 坐标 → 本地坐标转换）
    # ==============================================================
    def _gps_to_local(self, lat, lng):
        """GPS (lat/lng) → 本地 x/y (米)，基于 GPS 原点做等距圆柱投影"""
        deg_to_rad = math.pi / 180.0
        lat_rad = self._gps_origin_lat * deg_to_rad
        dx = (lng - self._gps_origin_lng) * 111320.0 * math.cos(lat_rad)
        dy = (lat - self._gps_origin_lat) * 110540.0
        return dx, dy

    def _validate_web_payload(self, data):
        """Validate the Web batch before it can mutate the active mission."""
        if not isinstance(data, dict):
            raise ValueError('payload 必须是 JSON 对象')
        if data.get('action') != 'set_areas':
            raise ValueError(f'未知 action: {data.get("action", "")}')

        web_areas = data.get('areas')
        if not isinstance(web_areas, list) or not web_areas:
            raise ValueError('areas 必须是非空数组')

        for area_idx, area in enumerate(web_areas):
            if not isinstance(area, dict):
                raise ValueError(f'areas[{area_idx}] 必须是对象')

            name = area.get('name')
            if name is not None and not isinstance(name, str):
                raise ValueError(f'areas[{area_idx}].name 必须是字符串')

            points = area.get('points')
            if not isinstance(points, list) or len(points) < 3:
                raise ValueError(
                    f'areas[{area_idx}].points 必须是至少 3 个点的数组')

            for point_idx, point in enumerate(points):
                if not isinstance(point, dict):
                    raise ValueError(
                        f'areas[{area_idx}].points[{point_idx}] 必须是对象')
                for axis in ('x', 'y'):
                    value = point.get(axis)
                    if (isinstance(value, bool)
                            or not isinstance(value, (int, float))
                            or not math.isfinite(value)):
                        raise ValueError(
                            f'areas[{area_idx}].points[{point_idx}].{axis} '
                            '必须是有限数字')

            local_points = [
                self._gps_to_local(point['y'], point['x'])
                for point in points
            ]
            try:
                polygon = Polygon(local_points)
            except Exception as e:
                raise ValueError(
                    f'areas[{area_idx}] 多边形无法构造: {e}') from e
            if not polygon.is_valid or polygon.area < 0.01:
                raise ValueError(
                    f'areas[{area_idx}] 多边形无效或面积太小')

        return web_areas

    def web_areas_callback(self, msg: StringMsg):
        """接收 Web 前端发来的区域数据（JSON），用 Shapely 匹配障碍物归属"""
        import json
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Web 区域 JSON 解析失败: {e}')
            return

        try:
            web_areas = self._validate_web_payload(data)
        except ValueError as e:
            self.get_logger().error(f'Web 区域数据校验失败: {e}')
            return

        self.get_logger().info(f'📩 Web 收到 {len(web_areas)} 个条目')

        # ★ 全新方案：先在局部变量中构建整批区域，成功后一次性替换旧任务。
        next_areas = {}
        next_color_idx = 0

        # 第一遍：将 GPS 坐标全部转为本地坐标，分离区域和障碍物
        parsed_areas = []     # { name, local_pts, poly, color }
        parsed_obstacles = [] # { name, local_pts, poly }

        for wa in web_areas:
            name = wa.get('name', f'item_{len(parsed_areas) + len(parsed_obstacles) + 1}')
            raw_pts = wa.get('points', [])
            if len(raw_pts) < 3:
                self.get_logger().warn(f'[{name}] 点数不足，跳过')
                continue

            local_pts = []
            for pt in raw_pts:
                lng = pt.get('x', 0.0)
                lat = pt.get('y', 0.0)
                x, y = self._gps_to_local(lat, lng)
                local_pts.append((x, y))

            try:
                poly = Polygon(local_pts)
                if not poly.is_valid or poly.area < 0.01:
                    self.get_logger().warn(f'[{name}] 多边形无效或面积太小 ({poly.area:.2f} m²)')
                    continue
            except Exception as e:
                self.get_logger().error(f'[{name}] 多边形验证失败: {e}')
                continue

            is_obs = wa.get('is_obstacle', False)
            if is_obs:
                parsed_obstacles.append({'name': name, 'local_pts': local_pts, 'poly': poly})
                self.get_logger().info(f'  ⛔ 障碍物 [{name}]: {len(local_pts)} 点, 面积 {poly.area:.1f} m²')
            else:
                color = self.color_cycle[next_color_idx % len(self.color_cycle)]
                next_color_idx += 1
                parsed_areas.append({'name': name, 'local_pts': local_pts, 'poly': poly, 'color': color})
                self.get_logger().info(f'  🟢 区域 [{name}]: {len(local_pts)} 点, 面积 {poly.area:.1f} m²')

        # 第二遍：用 Shapely 判断每个障碍物归属哪个区域
        # 对于每个障碍物，找包含其代表点（guaranteed inside）的割草区域
        assigned_obs = set()  # 跟踪已分配的障碍物索引
        for area_item in parsed_areas:
            matching_rings = []
            for obs_idx, obs_item in enumerate(parsed_obstacles):
                try:
                    # ★ 使用 Shapely 的 representative_point() 做包含测试
                    # 这比手动 ray casting 更健壮，且不受顶点边界精度影响
                    rep_pt = obs_item['poly'].representative_point()
                    if area_item['poly'].contains(rep_pt):
                        matching_rings.append(obs_item['local_pts'])
                        assigned_obs.add(obs_idx)
                        self.get_logger().info(
                            f'    ↳ [障碍物 {obs_item["name"]}] 归属于 [区域 {area_item["name"]}]')
                except Exception as e:
                    self.get_logger().error(f'  障碍物归属判断失败: {e}')

            area_entry = {
                'name': area_item['name'],
                'points': area_item['local_pts'],
                'inner_rings': matching_rings,
                'color': list(area_item['color']),
                'cutting_angle': 0.0,
                'max_speed': 1.0,
            }
            next_areas[area_item['name']] = area_entry
            inner_count = len(matching_rings)
            if inner_count:
                self.get_logger().info(f'  → 区域 [{area_item["name"]}]: 含 {inner_count} 个内岛障碍物')

        # 未归属任何区域的障碍物 → 作为独立障碍物区域（max_speed=0，规划器会跳过）
        for obs_idx, obs_item in enumerate(parsed_obstacles):
            if obs_idx not in assigned_obs:
                obs_entry = {
                    'name': obs_item['name'],
                    'points': obs_item['local_pts'],
                    'inner_rings': [],
                    'color': [1.0, 0.0, 0.0],
                    'cutting_angle': 0.0,
                    'max_speed': 0.0,
                }
                next_areas[obs_item['name']] = obs_entry
                self.get_logger().info(f'  → 独立障碍物区域 [{obs_item["name"]}]（未归属任何割草区域）')

        if not next_areas:
            self.get_logger().error('Web 区域数据校验失败: 没有有效的区域条目')
            return

        self.areas = next_areas
        self.next_color_idx = next_color_idx

        self.get_logger().info(f'[Web] 完成: {len(parsed_areas)} 个区域, '
                               f'{len(parsed_obstacles)} 个障碍物 '
                               f'（共 {len(self.areas)} 个条目）')
        self.publish_visualization()
        self._save_hill_yaml()

    def _save_hill_yaml(self):
        """保存区域到 hill_boustrophedon 读取的 YAML 文件"""
        hill_file = state_file('hill_mowing_areas.yaml')
        data = {'areas': []}
        for name, area in self.areas.items():
            entry = {
                'name': name,
                'points': self._tuples_to_lists(area['points']),
                'inner_rings': self._tuples_to_lists(area.get('inner_rings', [])),
                'color': self._tuples_to_lists(area.get('color', [0.0, 1.0, 0.0])),
                'cutting_angle': area.get('cutting_angle', 0.0),
                'max_speed': area.get('max_speed', 1.0),
            }
            data['areas'].append(entry)
        try:
            ensure_parent(hill_file)
            with open(hill_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=None, allow_unicode=True)
            self.get_logger().info(f'已保存 {len(data["areas"])} 个区域到 {hill_file}')
        except Exception as e:
            self.get_logger().error(f'保存到 {hill_file} 失败: {e}')

    # ==============================================================
    # RViz PublishPoint 回调
    # ==============================================================
    def clicked_callback(self, msg: PointStamped):
        """接收 RViz 的点击点"""
        if self.drawing_name is None:
            return

        x, y = msg.point.x, msg.point.y
        self.drawing_points.append((x, y))
        self.get_logger().info(f'  添加点 ({x:.2f}, {y:.2f}) 到 [{self.drawing_name}] (共 {len(self.drawing_points)} 点)')

        # 发布临时标记
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f'draw_{self.drawing_name}'
        marker.id = len(self.drawing_points)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        self.debug_pub.publish(marker)

    # ==============================================================
    # 服务回调
    # ==============================================================
    def start_draw_cb(self, req, resp):
        """开始绘制新区域"""
        if self.drawing_name is not None:
            resp.success = False
            resp.message = f'正在绘制 [{self.drawing_name}]，请先 finish 或 cancel'
            return resp

        # 自动生成区域名
        name = f'area_{len(self.areas) + 1}'
        self.drawing_name = name
        self.drawing_points = []
        resp.success = True
        resp.message = f'开始绘制 [{name}]，请在 RViz 中点击地图添加点'
        self.get_logger().info(f'开始绘制新区域 [{name}]')
        return resp

    def finish_polygon_cb(self, req, resp):
        """完成当前多边形绘制"""
        if self.drawing_name is None:
            resp.success = False
            resp.message = '没有正在绘制的区域'
            return resp

        if len(self.drawing_points) < 3:
            resp.success = False
            resp.message = f'至少需要 3 个点 (当前 {len(self.drawing_points)})'
            return resp

        # 验证多边形有效性
        try:
            poly = Polygon(self.drawing_points)
            if not poly.is_valid or poly.area < 0.01:
                resp.success = False
                resp.message = '多边形无效或面积太小'
                return resp
        except Exception as e:
            resp.success = False
            resp.message = f'多边形错误: {str(e)}'
            return resp

        # 创建区域
        color = self.color_cycle[self.next_color_idx % len(self.color_cycle)]
        self.next_color_idx += 1

        area = {
            'name': self.drawing_name,
            'points': list(self.drawing_points),
            'inner_rings': [],
            'color': list(color),
            'cutting_angle': 0.0,
            'max_speed': 1.0,
        }
        self.areas[self.drawing_name] = area

        name = self.drawing_name
        self.drawing_name = None
        self.drawing_points = []

        resp.success = True
        resp.message = f'区域 [{name}] 创建完成 ({len(self.areas)} 个区域)'
        self.get_logger().info(resp.message)
        self.publish_visualization()
        return resp

    def cancel_polygon_cb(self, req, resp):
        """取消当前绘制"""
        if self.drawing_name is None:
            resp.success = False
            resp.message = '没有正在绘制的区域'
            return resp
        self.get_logger().info(f'取消绘制 [{self.drawing_name}]')
        self.drawing_name = None
        self.drawing_points = []
        resp.success = True
        resp.message = '已取消绘制'
        return resp

    def list_areas_cb(self, req, resp):
        """列出所有区域"""
        names = list(self.areas.keys())
        if not names:
            resp.success = True
            resp.message = '没有已定义的区域'
        else:
            details = []
            for name in names:
                a = self.areas[name]
                n_pts = len(a['points'])
                n_rings = len(a.get('inner_rings', []))
                details.append(f'{name}({n_pts}点,{n_rings}内岛)')
            resp.success = True
            resp.message = f'区域 ({len(names)}): ' + ', '.join(details)
        self.get_logger().info(resp.message)
        return resp

    def remove_area_cb(self, req, resp):
        """移除区域（通过参数指定名称）"""
        # 由于使用 Trigger，通过日志方式提示
        resp.success = False
        resp.message = '请使用服务参数指定区域名，或调用 clear_all'
        return resp

    def clear_all_cb(self, req, resp):
        """清空所有区域 + 删除 YAML 文件"""
        count = len(self.areas)
        self.areas.clear()
        self.next_color_idx = 0
        # 删除 YAML 文件，确保下次规划不会读到旧数据
        for f in [state_file('hill_mowing_areas.yaml'),
                  state_file('mowing_areas.yaml'),
                  os.path.expanduser('~/hill_mowing_areas.yaml'),
                  os.path.expanduser('~/mowing_areas.yaml')]:
            try:
                if os.path.exists(f):
                    os.remove(f)
                    self.get_logger().info(f'已删除 {f}')
            except Exception as e:
                self.get_logger().error(f'删除 {f} 失败: {e}')
        resp.success = True
        resp.message = f'已清空 {count} 个区域并删除 YAML 文件'
        self.get_logger().info(resp.message)
        self.publish_visualization()
        return resp

    @staticmethod
    def _tuples_to_lists(obj):
        """递归将 tuple 转换为 list（YAML 序列化兼容）"""
        if isinstance(obj, tuple):
            return [MultiAreaDefiner._tuples_to_lists(x) for x in obj]
        elif isinstance(obj, list):
            return [MultiAreaDefiner._tuples_to_lists(x) for x in obj]
        elif isinstance(obj, dict):
            return {k: MultiAreaDefiner._tuples_to_lists(v) for k, v in obj.items()}
        return obj

    def save_cb(self, req, resp):
        """保存区域到 YAML"""
        data = {'areas': []}
        for name, area in self.areas.items():
            entry = {
                'name': name,
                'points': self._tuples_to_lists(area['points']),
                'inner_rings': self._tuples_to_lists(area.get('inner_rings', [])),
                'color': self._tuples_to_lists(area['color']),
                'cutting_angle': area.get('cutting_angle', 0.0),
                'max_speed': area.get('max_speed', 1.0),
            }
            data['areas'].append(entry)

        try:
            ensure_parent(self.area_file)
            with open(self.area_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=None, allow_unicode=True)
            resp.success = True
            resp.message = f'已保存 {len(data["areas"])} 个区域到 {self.area_file}'
        except Exception as e:
            resp.success = False
            resp.message = f'保存失败: {str(e)}'

        self.get_logger().info(resp.message)
        return resp

    def load_cb(self, req, resp):
        """从 YAML 加载区域"""
        load_file = readable_path(self.area_file, 'mowing_areas.yaml')
        if not os.path.exists(load_file):
            resp.success = False
            resp.message = f'文件不存在: {self.area_file}'
            self.get_logger().error(resp.message)
            return resp

        try:
            data = mission_to_legacy_areas(load_mission_file(load_file))

            loaded = 0
            for entry in data['areas']:
                name = entry.get('name', f'area_{loaded}')
                # 验证多边形
                pts = entry.get('points', [])
                if len(pts) < 3:
                    continue
                try:
                    poly = Polygon(pts)
                    if not poly.is_valid or poly.area < 0.01:
                        continue
                except Exception:
                    continue

                area = {
                    'name': name,
                    'points': [(float(x), float(y)) for x, y in pts],
                    'inner_rings': [
                        [(float(x), float(y)) for x, y in ring]
                        for ring in entry.get('inner_rings', [])
                    ],
                    'color': entry.get('color', [0.0, 1.0, 0.0]),
                    'cutting_angle': entry.get('cutting_angle', 0.0),
                    'max_speed': entry.get('max_speed', 1.0),
                }
                self.areas[name] = area
                loaded += 1

            self.next_color_idx = loaded
            resp.success = True
            resp.message = f'已加载 {loaded} 个区域'
            self.get_logger().info(resp.message)
            self.publish_visualization()

        except Exception as e:
            resp.success = False
            resp.message = f'加载失败: {str(e)}'
            self.get_logger().error(resp.message)

        return resp

    # ==============================================================
    # 三角剖分工具（耳切法）
    # ==============================================================
    @staticmethod
    def _triangulate_polygon(pts):
        """
        耳切法三角剖分简单多边形
        返回 [(x, y), ...] 每3个一组构成一个三角形
        """
        n = len(pts)
        if n < 3:
            return []
        if n == 3:
            return [p for p in pts]

        # 确保逆时针
        vertices = list(pts)
        if not MultiAreaDefiner._is_ccw(vertices):
            vertices = vertices[::-1]

        result = []
        indices = list(range(n))

        while len(indices) >= 3:
            ear_found = False
            for i in range(len(indices)):
                prev = indices[i - 1]
                curr = indices[i]
                nxt = indices[(i + 1) % len(indices)]

                a, b, c = vertices[prev], vertices[curr], vertices[nxt]

                # 凸顶点检查（逆时针方向为正）
                cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
                if cross <= 1e-8:
                    continue  # 不是凸顶点

                # 检查是否有其他点在三角形内部
                is_ear = True
                for j_idx in indices:
                    if j_idx in (prev, curr, nxt):
                        continue
                    if MultiAreaDefiner._point_in_triangle(vertices[j_idx], a, b, c):
                        is_ear = False
                        break

                if is_ear:
                    result.extend([a, b, c])
                    indices.pop(i)
                    ear_found = True
                    break

            if not ear_found:
                # 容错：从剩余顶点 fan 剖分
                for i in range(1, len(indices) - 1):
                    result.extend([vertices[indices[0]], vertices[indices[i]], vertices[indices[i + 1]]])
                break

        return result

    @staticmethod
    def _is_ccw(pts):
        """检查多边形顶点是否为逆时针方向"""
        area = 0.0
        for i in range(len(pts)):
            j = (i + 1) % len(pts)
            area += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1]
        return area > 0

    @staticmethod
    def _point_in_triangle(p, a, b, c):
        """重心坐标法检查点 p 是否在三角形 (a,b,c) 内部"""
        def area(pa, pb, pc):
            return abs((pb[0] - pa[0]) * (pc[1] - pa[1]) -
                       (pc[0] - pa[0]) * (pb[1] - pa[1])) * 0.5

        total = area(a, b, c)
        if total < 1e-12:
            return False

        s1 = area(p, b, c) / total
        s2 = area(a, p, c) / total
        s3 = area(a, b, p) / total

        eps = 1e-6
        return (s1 > eps and s2 > eps and s3 > eps and
                s1 < 1 - eps and s2 < 1 - eps and s3 < 1 - eps)

    # ==============================================================
    # 可视化
    # ==============================================================
    def publish_visualization(self):
        """发布所有区域为 RViz MarkerArray"""
        markers = MarkerArray()

        for idx, (name, area) in enumerate(self.areas.items()):
            color = area.get('color', [0.0, 1.0, 0.0])
            pts = area.get('points', [])

            if len(pts) < 3:
                continue

            # --- 区域填充（半透明多边形，耳切法三角剖分） ---
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'area_fill'
            marker.id = idx
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color = ColorRGBA(
                r=color[0], g=color[1], b=color[2], a=0.3)

            # 三角剖分多边形（保证点数为3的倍数）
            tri_pts = self._triangulate_polygon(pts)
            for x, y in tri_pts:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.0
                marker.points.append(p)

            markers.markers.append(marker)

            # --- 边界线 ---
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = 'area_boundary'
            line.id = idx
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.05  # 线宽
            line.color = ColorRGBA(
                r=color[0], g=color[1], b=color[2], a=1.0)

            for x, y in pts + [pts[0]]:  # 闭合
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.01
                line.points.append(p)
            markers.markers.append(line)

            # --- 区域名称标签 ---
            center = Polygon(pts).centroid
            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'area_label'
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = center.x
            text.pose.position.y = center.y
            text.pose.position.z = 0.5
            text.scale.z = 0.4
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = name
            markers.markers.append(text)

            # --- 内岛（红色禁入区） ---
            for ring_idx, ring in enumerate(area.get('inner_rings', [])):
                if len(ring) < 3:
                    continue

                inner = Marker()
                inner.header.frame_id = self.frame_id
                inner.header.stamp = self.get_clock().now().to_msg()
                inner.ns = f'inner_ring_{name}'
                inner.id = ring_idx
                inner.type = Marker.LINE_STRIP
                inner.action = Marker.ADD
                inner.pose.orientation.w = 1.0
                inner.scale.x = 0.08
                inner.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

                for x, y in ring + [ring[0]]:
                    p = Point()
                    p.x = x
                    p.y = y
                    p.z = 0.02
                    inner.points.append(p)
                markers.markers.append(inner)

                # 内岛填充（半透明红，耳切法三角剖分）
                inner_fill = Marker()
                inner_fill.header.frame_id = self.frame_id
                inner_fill.header.stamp = self.get_clock().now().to_msg()
                inner_fill.ns = f'inner_fill_{name}'
                inner_fill.id = ring_idx
                inner_fill.type = Marker.TRIANGLE_LIST
                inner_fill.action = Marker.ADD
                inner_fill.pose.orientation.w = 1.0
                inner_fill.scale.x = 1.0
                inner_fill.scale.y = 1.0
                inner_fill.scale.z = 1.0
                inner_fill.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)

                tri_pts = self._triangulate_polygon(ring)
                for x, y in tri_pts:
                    p = Point()
                    p.x = x
                    p.y = y
                    p.z = 0.01
                    inner_fill.points.append(p)
                markers.markers.append(inner_fill)

        # --- 当前绘制点 ---
        if self.drawing_points:
            for i, (x, y) in enumerate(self.drawing_points):
                pt = Marker()
                pt.header.frame_id = self.frame_id
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.ns = 'drawing_points'
                pt.id = i
                pt.type = Marker.SPHERE
                pt.action = Marker.ADD
                pt.pose.position.x = x
                pt.pose.position.y = y
                pt.pose.position.z = 0.0
                pt.scale.x = 0.2
                pt.scale.y = 0.2
                pt.scale.z = 0.2
                pt.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
                markers.markers.append(pt)

            # 绘制中的连线
            if len(self.drawing_points) >= 2:
                line = Marker()
                line.header.frame_id = self.frame_id
                line.header.stamp = self.get_clock().now().to_msg()
                line.ns = 'drawing_line'
                line.id = 0
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.05
                line.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)

                for x, y in self.drawing_points:
                    p = Point()
                    p.x = x
                    p.y = y
                    p.z = 0.01
                    line.points.append(p)
                markers.markers.append(line)

        try:
            self.marker_pub.publish(markers)
        except Exception as e:
            self.get_logger().error(f'可视化发布失败: {e}')


def main():
    rclpy.init()
    node = MultiAreaDefiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
