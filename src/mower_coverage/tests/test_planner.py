#!/usr/bin/env python3
"""
test_planner.py — 牛耕式路径规划器测试

独立于 ROS2 运行，直接验证算法正确性

用法：
  python3 tests/test_planner.py
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_coverage.boustrophedon_planner import BoustrophedonPlanner
from shapely.geometry import Polygon, box


def test_rectangle():
    """测试矩形区域全覆盖"""
    print("\n=== 测试1: 矩形区域 ===")
    area = box(-25, -25, 25, 25)  # 50m × 50m
    planner = BoustrophedonPlanner(cutting_width=0.5, overlap=0.1)
    path = planner.plan(area)

    total_dist = sum(
        ((path[i][0]-path[i+1][0])**2 + (path[i][1]-path[i+1][1])**2)**0.5
        for i in range(len(path)-1)
    )

    # 理论: 50/0.45=111条带 × 50m = 5550m + 转弯 ≈ 6000m+
    expected_min = 5500
    expected_max = 8000

    print(f"  路径点数: {len(path)}")
    print(f"  总距离: {total_dist:.1f}m")
    print(f"  预期范围: {expected_min}-{expected_max}m")

    assert len(path) > 100, f"路径点太少: {len(path)}"
    assert expected_min < total_dist < expected_max, \
        f"距离不在预期范围: {total_dist:.1f} (预期 {expected_min}-{expected_max})"
    print("  ✅ 通过")


def test_rectangle_with_obstacles():
    """测试带障碍物的矩形区域"""
    print("\n=== 测试2: 矩形 + 障碍物 ===")
    area = box(-25, -25, 25, 25)
    obstacles = [
        box(-8, -8, -4, -4),     # 方形障碍物
        box(5, 5, 8, 7),         # 另一个障碍物
        box(-15, 10, -12, 12),   # 第三个障碍物
    ]

    planner = BoustrophedonPlanner(cutting_width=0.5)
    path = planner.plan(area, obstacles=obstacles)

    total_dist = sum(
        ((path[i][0]-path[i+1][0])**2 + (path[i][1]-path[i+1][1])**2)**0.5
        for i in range(len(path)-1)
    )
    # 无障碍时是 5500-8000m，有障碍应该略少
    print(f"  路径点数: {len(path)}")
    print(f"  总距离: {total_dist:.1f}m")

    assert len(path) > 50, f"路径点太少: {len(path)}"
    # 应该小于无障碍的距离
    print("  ✅ 通过")


def test_small_lawn():
    """测试小面积草坪 (30m × 20m)"""
    print("\n=== 测试3: 小草坪 30m×20m ===")
    area = box(-15, -10, 15, 10)

    planner = BoustrophedonPlanner(cutting_width=0.4, overlap=0.1)
    path = planner.plan(area)

    total_dist = sum(
        ((path[i][0]-path[i+1][0])**2 + (path[i][1]-path[i+1][1])**2)**0.5
        for i in range(len(path)-1)
    )

    print(f"  路径点数: {len(path)}")
    print(f"  总距离: {total_dist:.1f}m")

    assert len(path) > 50
    # 30*10/(0.4*0.9)*15 + 20 ≈ 预期
    print("  ✅ 通过")


def test_non_convex():
    """测试L形非凸区域"""
    print("\n=== 测试4: L形区域 (非凸) ===")

    # 用大方块减去一个小方块形成L形
    outer = Polygon([(-25, -25), (25, -25), (25, 25), (-25, 25), (-25, -25)])
    hole = Polygon([(-5, -5), (5, -5), (5, 5), (-5, 5), (-5, -5)])
    area = outer.difference(hole)

    planner = BoustrophedonPlanner(cutting_width=0.5)
    path = planner.plan(area)

    total_dist = sum(
        ((path[i][0]-path[i+1][0])**2 + (path[i][1]-path[i+1][1])**2)**0.5
        for i in range(len(path)-1)
    )

    print(f"  路径点数: {len(path)}")
    print(f"  总距离: {total_dist:.1f}m")
    assert len(path) > 50
    print("  ✅ 通过")


def test_no_overlap():
    """测试无重叠（严格按割幅走）"""
    print("\n=== 测试5: 严格模式 (0% 重叠) ===")
    area = box(-20, -20, 20, 20)

    planner = BoustrophedonPlanner(cutting_width=1.0, overlap=0.0)
    path = planner.plan(area)

    total_dist = sum(
        ((path[i][0]-path[i+1][0])**2 + (path[i][1]-path[i+1][1])**2)**0.5
        for i in range(len(path)-1)
    )

    # 1m 割幅, 0% 重叠, y方向步进1m
    # 40条带 × 40m = 1600m + 转弯 ≈ 1680m
    print(f"  路径点数: {len(path)}")
    print(f"  总距离: {total_dist:.1f}m (预期~1680m)")
    assert 1500 < total_dist < 2200
    print("  ✅ 通过")


if __name__ == "__main__":
    test_rectangle()
    test_rectangle_with_obstacles()
    test_small_lawn()
    test_non_convex()
    test_no_overlap()
    print("\n" + "="*40)
    print("🎉 所有测试通过!")
    print("="*40)
