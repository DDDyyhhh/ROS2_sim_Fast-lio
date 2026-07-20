#!/usr/bin/env python3
"""
hill_boustrophedon.py — 多区域牛耕式全覆盖路径规划器

扩展自 boustrophedon_planner.py，支持：
  - 多个不相连区域（MultiPolygon）
  - 每个区域含内岛
  - 区域间导航路径
  - 地形感知速度标注

输入：区域列表（含内岛）
输出：每个区域的牛耕式路径 + 区域间导航路径

用法：
  ros2 run mower_coverage hill_boustrophedon
"""

import math
import json
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from nav_msgs.msg import Path
from shapely.geometry import Polygon, LineString, Point as ShapelyPoint, box
from shapely.ops import unary_union

import numpy as np

from mower_coverage.state_paths import readable_path, state_file
from mower_coverage.mission.loader import (
    load_mission_file,
    mission_to_legacy_areas,
)


def polyline_to_path(waypoints, frame_id, stamp):
    """路径点列表 → nav_msgs/Path"""
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = stamp
    for x, y, speed in waypoints:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = stamp
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        path_msg.poses.append(ps)
    return path_msg


def waypoints_to_poses(waypoints):
    """路径点列表 → PoseArray"""
    arr = PoseArray()
    for x, y, speed in waypoints:
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = 0.0
        p.orientation.w = 1.0
        arr.poses.append(p)
    return arr


class HillBoustrophedon(Node):
    """多区域牛耕式规划器"""

    def __init__(self):
        super().__init__('hill_boustrophedon')

        # 参数（安全声明，避免重复）
        param_defaults = [
            ('frame_id', 'map'),
            ('cutting_width', 0.5),
            ('overlap', 0.1),
            ('waypoint_spacing', 0.3),
            ('inner_ring_inflate', 0.3),
            ('start_from_corner', True),
            ('terrain_slope_threshold', 20.0),
            ('terrain_slow_factor', 0.5),
            ('inter_area_speed', 0.5),
            ('area_file', state_file('hill_mowing_areas.yaml')),
        ]
        for name, default in param_defaults:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

        self.frame_id = self.get_parameter('frame_id').value
        self.cutting_width = self.get_parameter('cutting_width').value
        self.overlap = self.get_parameter('overlap').value
        self.spacing = self.get_parameter('waypoint_spacing').value
        self.inner_inflate = self.get_parameter('inner_ring_inflate').value
        self.slope_threshold = self.get_parameter('terrain_slope_threshold').value
        self.slow_factor = self.get_parameter('terrain_slow_factor').value
        self.transit_speed = self.get_parameter('inter_area_speed').value
        self.area_file = self.get_parameter('area_file').value

        self.swath_spacing = self.cutting_width * (1.0 - self.overlap)

        # 发布者
        self.path_pub = self.create_publisher(Path, '/coverage/multi_path', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/coverage/multi_path_poses', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage/multi_path_markers', 10)

        # 服务
        self.srv_plan = self.create_service(
            Trigger, '/multi_area/plan', self.plan_cb)
        self.srv_clear_path = self.create_service(
            Trigger, '/multi_area/clear_path', self.clear_path_cb)

        self.get_logger().info('多区域牛耕式规划器已启动')
        self.get_logger().info(f'  割幅: {self.cutting_width}m, 重叠: {self.overlap*100:.0f}%')

        # 保存最后规划的路径（供执行器获取 + RViz 周期可视化）
        self.last_waypoints = []  # [(x, y, speed), ...]
        self.last_obstacles = []  # 当前规划中的膨胀障碍物，用于前端路径降采样保形

        # 可视化定时器（1Hz — RViz Volatile QoS 需要周期性消息）
        self.viz_timer = self.create_timer(1.0, self.republish_path_viz)

    # ==============================================================
    # 核心规划算法
    # ==============================================================
    def plan_area(self, area_polygon, inner_rings=None, angle=0.0,
                  max_speed=1.0):
        """
        规划单个区域的牛耕式路径

        Returns:
            [(x, y, speed), ...] 速度标注的路径点
        """
        obstacles = []
        if inner_rings:
            self.get_logger().info(f'    ↳ 处理 {len(inner_rings)} 个内岛/障碍物')
            for ring in inner_rings:
                inner_poly = Polygon(ring)
                if inner_poly.is_valid and inner_poly.area > 0.01:
                    inflated = inner_poly.buffer(self.inner_inflate)
                    obstacles.append(inflated)
                    self.get_logger().info(
                        f'      ↳ 障碍物面积: {inner_poly.area:.1f} m², '
                        f'膨胀后: {inflated.area:.1f} m²')
                else:
                    self.get_logger().warn(
                        f'      ↳ 跳过无效障碍物 (valid={inner_poly.is_valid}, '
                        f'area={inner_poly.area:.3f})')

        # 从区域中挖掉障碍物
        work_area = area_polygon
        obs_bounds = None  # 障碍物合并 bounding box，用于绕障过渡
        if obstacles:
            combined = unary_union(obstacles)
            # difference/intersection 的浮点误差可能让扫描线端点落入
            # 障碍物约几个纳米；预留极小几何余量，避免后处理把整段
            # 后续覆盖误判为障碍物内部。
            work_area = area_polygon.difference(combined.buffer(1e-4))
            obs_bounds = combined.bounds  # (min_x, min_y, max_x, max_y)

        if work_area.is_empty:
            return []

        # 旋转坐标系（同时旋转障碍物，确保后处理时坐标系一致）
        if abs(angle) > 1e-6:
            work_area = self._rotate_polygon(work_area, -angle)
            obstacles = [self._rotate_polygon(obs, -angle) for obs in obstacles]

        min_x, min_y, max_x, max_y = work_area.bounds
        span = max_y - min_y
        num_swaths = max(1, int(math.ceil(span / self.swath_spacing)))

        waypoints = []

        for i in range(num_swaths):
            pos = min_y + i * self.swath_spacing
            if pos > max_y:
                break

            scan_line = LineString([(min_x - 20, pos), (max_x + 20, pos)])
            intersection = work_area.intersection(scan_line)

            if intersection.is_empty:
                continue

            segments = []
            if intersection.geom_type == 'LineString':
                segments = [list(intersection.coords)]
            elif intersection.geom_type == 'MultiLineString':
                segments = [list(seg.coords) for seg in intersection.geoms]

            # ★ 确保段按从左到右排序（Shapely 返回的顺序不确定）
            segments.sort(key=lambda seg: seg[0][0])

            # ★ 奇数扫描线（右→左）反转段顺序，使路径从右到左自然流动
            # 偶数扫描线：左→右（seg[0]=左侧段, seg[1]=右侧段）
            # 奇数扫描线：右→左（seg[1]=右侧段, seg[0]=左侧段）
            # 避免奇数线时从左侧尽头直接跳到右侧尽头穿越障碍物
            segs_to_process = segments[::-1] if i % 2 == 1 else segments

            for si, seg in enumerate(segs_to_process):
                if len(seg) < 2:
                    continue

                # 段内排序：交替方向
                if i % 2 == 0:
                    seg.sort(key=lambda p: p[0])  # 左→右
                else:
                    seg.sort(key=lambda p: p[0], reverse=True)  # 右→左

                seg_line = LineString(seg)

                # ★ 浮点精度保护：即使段来自 work_area.intersection()，
                # 也可能因 difference() 边界误差而轻微穿越障碍物，跳过这种段
                if obstacles:
                    combined_check = unary_union(obstacles)
                    if seg_line.crosses(combined_check) or seg_line.within(combined_check) or combined_check.contains(seg_line):
                        continue

                for dist in np.arange(0, seg_line.length, self.spacing):
                    pt = seg_line.interpolate(dist)
                    waypoints.append((pt.x, pt.y, max_speed))

                end_pt = list(seg_line.coords)[-1]
                waypoints.append((end_pt[0], end_pt[1], max_speed))

            # 平滑U型转弯（半圆弧，避免尖角损伤草坪）
            if i < num_swaths - 1 and waypoints:
                last_pt = waypoints[-1]
                next_pos = min_y + (i + 1) * self.swath_spacing

                R = self.swath_spacing / 2.0          # 转弯半径
                center_y = last_pt[1] + R
                num_arc_pts = 12                      # 弧线插值点数

                if i % 2 == 0:
                    # 右侧转弯（L→R → R→L）：弧线向右凸出
                    for j in range(1, num_arc_pts + 1):
                        t = -math.pi / 2 + j * math.pi / num_arc_pts
                        px = last_pt[0] + R * math.cos(t)
                        py = center_y + R * math.sin(t)
                        waypoints.append((px, py, self.transit_speed))
                else:
                    # 左侧转弯（R→L → L→R）：弧线向左凸出
                    for j in range(1, num_arc_pts + 1):
                        t = 3 * math.pi / 2 - j * math.pi / num_arc_pts
                        px = last_pt[0] + R * math.cos(t)
                        py = center_y + R * math.sin(t)
                        waypoints.append((px, py, self.transit_speed))

        # ★ 后处理：修复所有穿越障碍物的路径连线（扫描线间过渡 + U 型转弯）
        if obstacles:
            waypoints = self._fix_obstacle_crossings(waypoints, obstacles, max_speed)

        # 还原旋转
        rotated = abs(angle) > 1e-6
        if rotated:
            waypoints = [(self._rotate_point((x, y), angle), s)
                         for x, y, s in waypoints]
            waypoints = [(x, y, s) for (x, y), s in waypoints]

        # ★ 还原旋转后（或如果没有旋转），再次检查原始坐标系的障碍物穿越
        # 原因：(1) 旋转后坐标系的修复可能在还原后又产生穿越
        #       (2) work_area.difference() 的浮点精度误差可能导致扫描线段本身就贴边
        # 如果有旋转，必须再检一次原始坐标系；如果没有旋转，也检一次以捕获浮点误差
        if inner_rings and (rotated or obstacles):
            original_obstacles = []
            for ring in inner_rings:
                inner_poly = Polygon(ring)
                if inner_poly.is_valid and inner_poly.area > 0.01:
                    original_obstacles.append(inner_poly.buffer(self.inner_inflate))
            if original_obstacles:
                waypoints = self._fix_obstacle_crossings(waypoints, original_obstacles, max_speed)

        return waypoints

    def _fix_obstacle_crossings(self, waypoints, obstacles, max_speed):
        """
        后处理修复：扫描所有相邻路径点，如果连线穿越障碍物则插入绕障过渡点

        过渡路径：沿当前侧水平移至安全区 → 垂直升到障碍物上方 →
                 水平跨越 → 垂直降到目标高度 → 水平移至目标点

        多障碍物场景：每次只绕**实际穿越的那个（或那几个）**障碍物，
        不用全局合并框（否则绕障点可能落在另一个障碍物里）
        """
        if not obstacles or len(waypoints) < 2:
            return waypoints

        combined = unary_union(obstacles)

        fixed = [waypoints[0]]
        fix_count = 0

        def point_inside_obstacle(pt):
            # work_area.difference() 产生的扫描线端点可能正好落在膨胀
            # 障碍物边界上；边界点不是内部点，不能因此丢掉后续整段覆盖。
            return combined.contains(ShapelyPoint(pt[0], pt[1]))

        def segment_hits_obstacle(a, b):
            """检测线段是否穿越或过于接近障碍物"""
            line = LineString([a[:2], b[:2]])
            # 标准穿越检测
            if line.crosses(combined) or line.within(combined) or combined.contains(line):
                return True
            # 浮点精度保护：如果线段距离障碍物 < 0.05m，且不是仅在
            # 边界端点接触，视为穿越。扫描线端点贴边是 difference()
            # 的正常结果，不能把它当作整条线段不安全。
            safety_margin = 0.05
            if combined.distance(line) < safety_margin and not line.touches(combined):
                # 路径点可能因几何内缩只在障碍物外约几个微米；若该段
                # 从障碍物边界向外离开，不应因安全余量把它判成不可连接。
                start_distance = combined.distance(ShapelyPoint(a[:2]))
                end_distance = combined.distance(ShapelyPoint(b[:2]))
                return end_distance <= start_distance + 1e-9
            return False

        def get_blocking_obstacles(a, b):
            """返回实际阻挡这条线段的障碍物列表（用于计算绕障边界）"""
            line = LineString([a[:2], b[:2]])
            blocking = []
            for obs in obstacles:
                if line.crosses(obs) or line.within(obs) or obs.contains(line):
                    blocking.append(obs)
            return blocking

        def candidate_route_is_safe(candidates):
            """候选绕障点及其新增线段都必须避开全部障碍物。"""
            route = [fixed[-1], *candidates]
            if any(point_inside_obstacle(pt) for pt in route):
                return False
            return all(
                not segment_hits_obstacle(route[i], route[i + 1])
                for i in range(len(route) - 1)
            )

        def append_point(points, pt):
            if points and points[-1][:2] == pt[:2]:
                return
            points.append(pt)

        for i in range(1, len(waypoints)):
            prev = fixed[-1]
            curr = waypoints[i]

            # U 型转弯或旧过渡点可能落进膨胀障碍物内部；这些点不能保留，
            # 否则前端和执行器都会在红色区域内连出黄色路径。
            if point_inside_obstacle(curr):
                fix_count += 1
                continue

            if not segment_hits_obstacle(prev, curr):
                append_point(fixed, curr)
                continue

            # 连线穿越障碍物 → 找出实际阻挡的障碍物，只绕这些障碍物
            blocking = get_blocking_obstacles(prev, curr)
            if not blocking:
                # 理论上不应该发生（segment_hits_obstacle 已返回 True）
                append_point(fixed, curr)
                continue

            # 用实际阻挡障碍物的合并框计算绕障参数
            blocking_union = unary_union(blocking)
            obs_min_x, obs_min_y, obs_max_x, obs_max_y = blocking_union.bounds

            # 动态调整安全距离，直到至少有一个候选点不在障碍物内
            vertical_clearance = 1.0   # 初始向上绕过障碍物的高度
            horizontal_margin = 0.5    # 初始左右安全距离
            max_attempts = 10

            for attempt in range(max_attempts):
                clear_y = obs_max_y + vertical_clearance
                safe_left = obs_min_x - horizontal_margin
                safe_right = obs_max_x + horizontal_margin

                def safe_side_x(x):
                    obs_mid_x = (obs_min_x + obs_max_x) / 2.0
                    return safe_left if x < obs_mid_x else safe_right

                px, py = prev[0], prev[1]
                cx, cy = curr[0], curr[1]
                prev_side_x = safe_side_x(px)
                curr_side_x = safe_side_x(cx)

                candidate_routes = [
                    [
                        (prev_side_x, py, self.transit_speed),
                        (prev_side_x, clear_y, self.transit_speed),
                        (curr_side_x, clear_y, self.transit_speed),
                        (curr_side_x, cy, self.transit_speed),
                        curr,
                    ],
                    [
                        # 起点可能贴着障碍物下边界，先向下离开再横向
                        # 绕行；否则第一条“先横移”的候选会立即穿障。
                        (px, obs_min_y - vertical_clearance, self.transit_speed),
                        (prev_side_x, obs_min_y - vertical_clearance, self.transit_speed),
                        (curr_side_x, obs_min_y - vertical_clearance, self.transit_speed),
                        (curr_side_x, cy, self.transit_speed),
                        curr,
                    ],
                ]

                # 候选点和新增线段必须一起验证；只验证点会让绕过当前
                # 障碍物的竖直/水平段穿过另一个分离障碍物。
                route_added = False
                for candidates in candidate_routes:
                    if candidate_route_is_safe(candidates):
                        for pt in candidates:
                            append_point(fixed, pt)
                        route_added = True
                        break

                if route_added:
                    break

                # 候选点或候选线段不安全，增大安全距离再试
                vertical_clearance += 0.5
                horizontal_margin += 0.3
            else:
                # 极端情况：尝试了 max_attempts 次仍失败，记录警告并跳过这个点
                self.get_logger().warn(
                    f'      ↳ 无法为 ({prev[0]:.1f},{prev[1]:.1f})→({curr[0]:.1f},{curr[1]:.1f}) '
                    f'生成安全绕障路径，跳过此段')
                continue

        if fix_count > 0:
            self.get_logger().info(f'    ↳ 后处理修复 {fix_count} 处障碍物穿越/内部点')

        return fixed

    def plan_transit(self, from_pt, to_polygon):
        """
        区域间导航路径（直线）
        Returns:
            [(x, y, speed), ...]
        """
        # 找到目标区域最近点
        boundary = to_polygon.exterior
        nearest = boundary.interpolate(boundary.project(ShapelyPoint(from_pt[0], from_pt[1])))

        return [
            (from_pt[0], from_pt[1], self.transit_speed),
            (nearest.x, nearest.y, self.transit_speed),
        ]

    # ==============================================================
    # 服务回调
    # ==============================================================
    def plan_cb(self, req, resp):
        """
        规划所有区域路径
        从 YAML 文件读取区域数据并规划完整路径
        """
        # 读取区域 YAML 文件
        area_file = readable_path(self.area_file, 'hill_mowing_areas.yaml')
        if not os.path.exists(area_file):
            resp.success = False
            resp.message = f'区域文件不存在: {area_file}'
            self.get_logger().error(resp.message)
            return resp

        try:
            data = mission_to_legacy_areas(load_mission_file(area_file))
            if not data['areas']:
                resp.success = False
                resp.message = 'YAML 文件中没有区域数据'
                return resp

            # 格式化为 areas_dict
            areas_dict = {}
            for entry in data['areas']:
                name = entry.get('name', f'area_{len(areas_dict)}')
                pts = entry.get('points', [])
                if len(pts) < 3:
                    continue
                areas_dict[name] = {
                    'points': [(float(x), float(y)) for x, y in pts],
                    'inner_rings': [
                        [(float(x), float(y)) for x, y in ring]
                        for ring in entry.get('inner_rings', [])
                    ],
                    'cutting_angle': entry.get('cutting_angle', 0.0),
                    'max_speed': entry.get('max_speed', 1.0),
                }

            if not areas_dict:
                resp.success = False
                resp.message = '没有有效的区域'
                return resp

            # 执行规划
            waypoints = self.plan_from_areas(areas_dict)
            self.get_logger().info(f'规划完成: 共 {len(waypoints)} 个路径点')
            resp.success = True
            resp.message = f'规划完成: {len(areas_dict)} 个区域, {len(waypoints)} 个路径点'

        except Exception as e:
            resp.success = False
            resp.message = f'规划失败: {str(e)}'
            self.get_logger().error(resp.message)
            import traceback
            self.get_logger().error(traceback.format_exc())

        return resp

    def clear_path_cb(self, req, resp):
        """清除已规划的路径 — 停止 1Hz 重发"""
        self.last_waypoints = []
        resp.success = True
        resp.message = '已清除规划路径'
        self.get_logger().info('🧹 已清除规划路径（停止重发）')
        return resp

    def plan_from_areas(self, areas_dict):
        """
        从区域字典生成完整路径
        areas_dict: {name: area_info}

        Returns:
            [(x, y, speed), ...] 完整路径点列表
        """
        planned_areas = []
        all_obstacles = []
        area_names = list(areas_dict.keys())

        for name in area_names:
            area_info = areas_dict[name]
            pts = area_info.get('points', [])
            if len(pts) < 3:
                continue

            poly = Polygon(pts)
            if not poly.is_valid:
                continue

            inner_rings = area_info.get('inner_rings', [])
            angle = area_info.get('cutting_angle', 0.0)
            max_speed = area_info.get('max_speed', 1.0)

            # ★ 跳过障碍物区域（max_speed <= 0 表示是障碍物，非割草区域）
            if max_speed <= 0.0:
                self.get_logger().info(f'  跳过障碍物区域 [{name}]（max_speed={max_speed}）')
                continue

            self.get_logger().info(f'  规划区域 [{name}]: {len(inner_rings)} 个内岛, max_speed={max_speed}')

            for ring in inner_rings:
                inner_poly = Polygon(ring)
                if inner_poly.is_valid and inner_poly.area > 0.01:
                    all_obstacles.append(inner_poly.buffer(self.inner_inflate))

            # 先完成所有区域规划。这样后续区域间连接可以看到全局障碍物，
            # 同时不会把尚未处理的区域覆盖路径混进同一次绕障状态机。
            wp = self.plan_area(poly, inner_rings, angle, max_speed)
            self.get_logger().info(f'  区域 [{name}]: {len(wp)} 个路径点')
            planned_areas.append((name, wp))

        all_waypoints = []
        previous_wp = None
        previous_name = None
        connection_count = 0
        for name, wp in planned_areas:
            if not wp:
                continue

            if previous_wp is None:
                all_waypoints.extend(wp)
            else:
                # 只修复区域间 seam。区域内路径已经由 plan_area() 使用
                # 本区域障碍物处理过，不能再对完整路径做流式后处理。
                connection = self._fix_obstacle_crossings(
                    [previous_wp[-1], wp[0]],
                    all_obstacles,
                    self.transit_speed)
                if connection[-1][:2] != wp[0][:2]:
                    message = f'区域 [{previous_name}] → [{name}] 无法生成安全导航路径'
                    self.get_logger().error(message)
                    raise RuntimeError(message)
                all_waypoints.extend(connection[1:])
                all_waypoints.extend(wp[1:])
                connection_count += 1
                self.get_logger().info(
                    f'  区域 [{previous_name}] → [{name}]: 导航路径')

            previous_wp = wp
            previous_name = name

        # 区域内路径与各个 seam 已分别绕障，完整路径无需再次统一流式处理。
        if all_obstacles and all_waypoints:
            self.get_logger().info(
                f'  已对 {connection_count} 个区域间连接执行全局绕障检查')

        self.last_waypoints = all_waypoints
        self.last_obstacles = all_obstacles
        self.publish_path(all_waypoints)
        return all_waypoints

    # ==============================================================
    # 发布
    # ==============================================================
    def _segment_hits_obstacle(self, a, b):
        """判断两点连线是否进入当前规划障碍物。"""
        if not getattr(self, 'last_obstacles', None):
            return False
        line = LineString([a[:2], b[:2]])
        for obstacle in self.last_obstacles:
            if line.crosses(obstacle) or line.within(obstacle) or obstacle.contains(line):
                return True
        return False

    def _downsample(self, wps, step=10):
        """降采样路径点（保留首尾，并保留绕障必要拐点）。"""
        if len(wps) <= step * 2:
            return wps

        result = [wps[0]]
        last_keep_idx = 0

        for i in range(1, len(wps) - 1):
            prev_pt = wps[i - 1]
            curr_pt = wps[i]
            next_pt = wps[i + 1]

            speed_changed = curr_pt[2] != prev_pt[2] or curr_pt[2] != next_pt[2]
            would_skip_into_obstacle = self._segment_hits_obstacle(result[-1], next_pt)
            due_by_step = i - last_keep_idx >= step

            if speed_changed or would_skip_into_obstacle or due_by_step:
                result.append(curr_pt)
                last_keep_idx = i

        if result[-1] != wps[-1]:
            result.append(wps[-1])

        return result

    def publish_path(self, waypoints):
        """发布规划好的路径（降采样后发布，减轻 rosbridge 负担）"""
        stamp = self.get_clock().now().to_msg()

        # 降采样路径点（原始路径保留在 last_waypoints 供执行器使用）
        viz_wps = self._downsample(waypoints, step=10)

        # nav_msgs/Path（供前端 rosbridge 使用）
        path_msg = polyline_to_path(viz_wps, self.frame_id, stamp)
        self.path_pub.publish(path_msg)

        # PoseArray（兼容层）
        arr = PoseArray()
        arr.header.frame_id = self.frame_id
        arr.header.stamp = stamp
        for x, y, s in viz_wps:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = 0.0
            p.orientation.w = 1.0
            arr.poses.append(p)
        self.poses_pub.publish(arr)

        # MarkerArray（可视化 — 也降采样）
        markers = MarkerArray()
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = stamp
        line.ns = 'planned_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.05
        line.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8)

        for x, y, s in viz_wps:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            line.points.append(p)
        markers.markers.append(line)

        self.marker_pub.publish(markers)

        self.get_logger().info(f'已发布路径: {len(waypoints)} 点 → 降采样 {len(viz_wps)} 点')

    def republish_path_viz(self):
        """1Hz 重发路径 — RViz Volatile QoS 需要周期性消息"""
        if not self.last_waypoints:
            return

        stamp = self.get_clock().now().to_msg()
        viz_wps = self._downsample(self.last_waypoints, step=10)

        # 重发 nav_msgs/Path（给前端 rosbridge 等晚加入订阅者）
        path_msg = polyline_to_path(viz_wps, self.frame_id, stamp)
        self.path_pub.publish(path_msg)

        # PoseArray
        arr = PoseArray()
        arr.header.frame_id = self.frame_id
        arr.header.stamp = stamp
        for x, y, s in viz_wps:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = 0.0
            p.orientation.w = 1.0
            arr.poses.append(p)
        self.poses_pub.publish(arr)

        # MarkerArray（RViz 可视化）
        markers = MarkerArray()
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = stamp
        line.ns = 'planned_path'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.05
        line.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8)

        for x, y, s in viz_wps:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            line.points.append(p)
        markers.markers.append(line)

        self.marker_pub.publish(markers)

    # ==============================================================
    # 几何工具
    # ==============================================================
    def _rotate_polygon(self, polygon, angle):
        """旋转多边形（保留内岛环）"""
        # 旋转外边界
        exterior_pts = list(polygon.exterior.coords)
        rotated_exterior = [self._rotate_point(p, angle) for p in exterior_pts]
        # 旋转内岛环
        interiors = []
        for interior in polygon.interiors:
            interior_pts = list(interior.coords)
            rotated_interior = [self._rotate_point(p, angle) for p in interior_pts]
            interiors.append(rotated_interior)
        return Polygon(rotated_exterior, interiors)

    def _rotate_point(self, point, angle):
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        x = point[0] * cos_a - point[1] * sin_a
        y = point[0] * sin_a + point[1] * cos_a
        return (x, y)


def main():
    rclpy.init()
    node = HillBoustrophedon()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
