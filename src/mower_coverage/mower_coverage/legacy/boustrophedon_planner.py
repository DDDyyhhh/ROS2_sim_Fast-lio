#!/usr/bin/env python3
"""
boustrophedon_planner.py — 牛耕式全覆盖路径规划器

输入：
  - 多边形区域（凸多边形的顶点列表）
  - 割幅宽度（m）
  - 重叠系数
  - 起始点（可选）
  - 障碍物多边形列表（可选内岛）

输出：
  - 覆盖路径点序列（可发布为 nav_msgs/Path 或 geometry_msgs/PoseArray）

算法：
  1. 确定割草方向（默认沿 x 轴，可通过旋转多边形实现任意角度）
  2. 在垂直于行进方向上等距生成平行条带
  3. 每条带与多边形求交 → 得到覆盖段
  4. 用交替方向连接这些段（牛耕式）
  5. 对障碍物：从段中切除被障碍物覆盖的部分
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from shapely.geometry import Polygon, Point, LineString, box
from shapely.ops import unary_union


class BoustrophedonPlanner:
    """牛耕式全覆盖路径规划器"""

    def __init__(self, cutting_width: float = 0.5, overlap: float = 0.1,
                 angle: float = 0.0, start_from_corner: bool = True):
        """
        Args:
            cutting_width: 割幅宽度（米），默认 0.5m (50cm)
            overlap: 重叠比例 (0~1)，默认 0.1 (10% 重叠)
            angle: 路径角度（弧度），0=沿x轴
            start_from_corner: 是否从角落开始
        """
        self.cutting_width = cutting_width
        self.swath_spacing = cutting_width * (1.0 - overlap)
        self.angle = angle
        self.start_from_corner = start_from_corner

    def plan(self, area_polygon: Polygon, obstacles: Optional[List[Polygon]] = None,
             start_pose: Optional[Tuple[float, float]] = None) -> List[Tuple[float, float]]:
        """
        生成全覆盖路径

        Args:
            area_polygon: 作业区域多边形 (shapely Polygon)
            obstacles: 障碍物多边形列表
            start_pose: 起始位姿 (x, y)，不指定则自动选角点

        Returns:
            路径点列表 [(x1,y1), (x2,y2), ...]
        """
        if area_polygon is None or area_polygon.is_empty:
            raise ValueError("作业区域不能为空")

        # 如果有多边形，从区域中挖掉障碍物
        if obstacles and len(obstacles) > 0:
            combined_obstacle = unary_union(obstacles)
            work_area = area_polygon.difference(combined_obstacle)
        else:
            work_area = area_polygon

        # 获取多边形的边界框
        min_x, min_y, max_x, max_y = work_area.bounds

        # 旋转坐标系（支持任意方向割草）
        if abs(self.angle) > 1e-6:
            work_area = self._rotate_polygon(work_area, -self.angle)
            min_x, min_y, max_x, max_y = work_area.bounds

        # 计算扫描线数量
        span = max_y - min_y if self._is_horizontal() else max_x - min_x
        num_swaths = max(1, int(math.ceil(span / self.swath_spacing)))

        # 生成路径点
        waypoints = []
        for i in range(num_swaths):
            # 当前条带的位置
            pos = min_y + i * self.swath_spacing if self._is_horizontal() else min_x + i * self.swath_spacing
            if pos > (max_y if self._is_horizontal() else max_x):
                break

            # 生成条带段: 与多边形求交
            if self._is_horizontal():
                scan_line = LineString([(min_x - 10, pos), (max_x + 10, pos)])
            else:
                scan_line = LineString([(pos, min_y - 10), (pos, max_y + 10)])

            intersection = work_area.intersection(scan_line)

            if intersection.is_empty:
                continue

            # 处理可能的多个线段
            segments = []
            if intersection.geom_type == 'LineString':
                segments = [list(intersection.coords)]
            elif intersection.geom_type == 'MultiLineString':
                segments = [list(seg.coords) for seg in intersection.geoms]

            # 对每个线段生成路径点
            for seg in segments:
                if len(seg) < 2:
                    continue
                # 按行进方向排序
                if i % 2 == 0:  # 偶数行：从左到右/从下到上
                    seg.sort(key=lambda p: p[0])
                else:  # 奇数行：从右到左（反向）
                    seg.sort(key=lambda p: p[0], reverse=True)

                # 采样路径点（每 0.3m 一个点）
                seg_line = LineString(seg)
                for dist in np.arange(0, seg_line.length, 0.3):
                    pt = seg_line.interpolate(dist)
                    waypoints.append((pt.x, pt.y))
                # 确保包含终点
                end_pt = list(seg_line.coords)[-1]
                waypoints.append(end_pt)

            # 行间转弯连接点（在条带之间添加过渡点）
            if i < num_swaths - 1:
                if waypoints:
                    last_pt = waypoints[-1]
                    next_pos = min_y + (i + 1) * self.swath_spacing if self._is_horizontal() else min_x + (i + 1) * self.swath_spacing

                    if self._is_horizontal():
                        # 鱼尾转（在条带末端掉头）
                        turn_start = (last_pt[0], pos)
                        turn_end = (last_pt[0] if i % 2 == 0 else min_x + 10, next_pos)
                    else:
                        turn_start = (pos, last_pt[1])
                        turn_end = (next_pos, last_pt[1] if i % 2 == 0 else min_y + 10)

                    # 插入转弯路径点（弓形转弯）
                    waypoints.append(turn_start)
                    waypoints.append(turn_end)

        # 还原坐标系旋转
        if abs(self.angle) > 1e-6:
            waypoints = [self._rotate_point(p, self.angle) for p in waypoints]

        return waypoints

    def _is_horizontal(self) -> bool:
        """判断扫描方向（默认水平 x 轴方向）"""
        return True  # 沿 x 轴行进，y 方向递进

    def _rotate_polygon(self, polygon: Polygon, angle: float) -> Polygon:
        """旋转多边形"""
        pts = list(polygon.exterior.coords)
        rotated = [self._rotate_point(p, angle) for p in pts]
        return Polygon(rotated)

    def _rotate_point(self, point: Tuple[float, float], angle: float) -> Tuple[float, float]:
        """绕原点旋转点"""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        x = point[0] * cos_a - point[1] * sin_a
        y = point[0] * sin_a + point[1] * cos_a
        return (x, y)


def main():
    """测试用：生成一个矩形区域的牛耕式路径并打印"""
    import sys

    # 默认测试参数
    width = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0
    height = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
    cutting = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5

    print(f"测试参数: 区域 {width}m × {height}m, 割幅 {cutting}m")

    # 创建矩形区域
    area = box(-width/2, -height/2, width/2, height/2)

    # 创建内岛障碍物
    obstacles = [
        box(-8, -8, -4, -4),   # 一个障碍物
        box(5, 5, 8, 7),       # 另一个障碍物
    ]

    planner = BoustrophedonPlanner(cutting_width=cutting, overlap=0.1)
    path = planner.plan(area, obstacles=obstacles)

    print(f"生成 {len(path)} 个路径点")
    print("前 10 个路径点:")
    for i, pt in enumerate(path[:10]):
        print(f"  {i}: ({pt[0]:.3f}, {pt[1]:.3f})")
    print(f"  路径总长度: {sum(math.dist(path[i], path[i+1]) for i in range(len(path)-1)):.1f}m")


if __name__ == "__main__":
    main()
