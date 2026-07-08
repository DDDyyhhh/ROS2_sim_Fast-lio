#!/usr/bin/env python3
"""
test_hill_boustrophedon_obstacles.py — 回归测试：发布给前端/执行器的路径不能因降采样穿越障碍物

独立于 ROS2 节点运行，只测试几何工具逻辑。
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from mower_coverage.hill_boustrophedon import HillBoustrophedon


def path_crosses_obstacle(path, obstacle):
    for i in range(1, len(path)):
        line = LineString([path[i - 1][:2], path[i][:2]])
        if line.crosses(obstacle) or line.within(obstacle) or obstacle.contains(line):
            return True
    return False


def path_has_points_inside(path, obstacle):
    return any(obstacle.covers(Point(p[:2])) for p in path)


def test_downsample_preserves_obstacle_detour_points():
    """降采样不能删除绕障拐点，否则前端黄线会重新直穿红色障碍物。"""
    obstacle = box(-1.0, -1.0, 1.0, 1.0)

    # 原始路径从左下绕到障碍物上方再到右下；完整路径不穿越障碍物。
    # 点数故意超过 step*2，让旧的“每 10 个点取一个”降采样生效；
    # 如果降采样只保留首尾/固定间隔点，会把中间绕障拐点删掉，黄线直穿红区。
    path = [
        (-5.0, 0.0, 1.0),
        (-4.7, 0.0, 1.0),
        (-4.4, 0.0, 1.0),
        (-4.1, 0.0, 1.0),
        (-3.8, 0.0, 1.0),
        (-3.5, 0.0, 1.0),
        (-3.2, 0.0, 1.0),
        (-2.9, 0.0, 1.0),
        (-2.6, 0.0, 1.0),
        (-2.3, 0.0, 1.0),
        (-2.0, 0.0, 1.0),
        (-2.0, 2.0, 0.5),
        (-1.0, 2.0, 0.5),
        (0.0, 2.0, 0.5),
        (1.0, 2.0, 0.5),
        (2.0, 2.0, 0.5),
        (2.0, 0.0, 0.5),
        (2.3, 0.0, 1.0),
        (2.6, 0.0, 1.0),
        (2.9, 0.0, 1.0),
        (3.2, 0.0, 1.0),
        (3.5, 0.0, 1.0),
        (3.8, 0.0, 1.0),
        (4.1, 0.0, 1.0),
        (4.4, 0.0, 1.0),
        (4.7, 0.0, 1.0),
        (5.0, 0.0, 1.0),
    ]
    assert not path_crosses_obstacle(path, obstacle)

    planner = object.__new__(HillBoustrophedon)
    planner.last_obstacles = [obstacle]

    sampled = planner._downsample(path, step=10)

    assert not path_crosses_obstacle(sampled, obstacle), sampled

def test_current_field_geometry_keeps_path_outside_inflated_obstacle():
    """当前截图对应几何：最终路径点和线段都不能进入膨胀后的红色障碍物。"""
    area = Polygon([
        (-16.064584472828553, 20.27933832675238),
        (16.19704779044271, 18.636085213949514),
        (14.542605111848422, -16.830837575490634),
        (-17.16754625953213, -16.55696140995076),
    ])
    obstacle_ring = [
        (-5.586447497683488, 4.805364513330872),
        (-6.827279506629205, -7.929864648551259),
        (5.167429923406933, -9.98393486216365),
        (6.546132157516944, 4.531488727548307),
    ]
    inflated_obstacle = Polygon(obstacle_ring).buffer(0.3)

    planner = object.__new__(HillBoustrophedon)
    planner.inner_inflate = 0.3
    planner.swath_spacing = 0.45
    planner.spacing = 0.3
    planner.transit_speed = 0.5
    planner.get_logger = lambda: type('Logger', (), {
        'info': lambda self, *args, **kwargs: None,
        'warn': lambda self, *args, **kwargs: None,
    })()

    path = HillBoustrophedon.plan_area(planner, area, [obstacle_ring], 0.0, 1.0)

    assert not path_has_points_inside(path, inflated_obstacle)
    assert not path_crosses_obstacle(path, inflated_obstacle)

def test_multiple_areas_create_transit_without_crashing():
    """多个区域规划时需要区域间导航函数，不能因为方法缺失而报错。"""
    planner = object.__new__(HillBoustrophedon)
    planner.inner_inflate = 0.3
    planner.swath_spacing = 0.45
    planner.spacing = 0.3
    planner.transit_speed = 0.5
    planner.last_waypoints = []
    planner.last_obstacles = []
    planner.publish_path = lambda waypoints: None
    planner.get_logger = lambda: type('Logger', (), {
        'info': lambda self, *args, **kwargs: None,
        'warn': lambda self, *args, **kwargs: None,
    })()

    areas = {
        '区域 1': {
            'points': [(-6, -2), (-2, -2), (-2, 2), (-6, 2)],
            'inner_rings': [],
            'cutting_angle': 0.0,
            'max_speed': 1.0,
        },
        '区域 2': {
            'points': [(2, -2), (6, -2), (6, 2), (2, 2)],
            'inner_rings': [],
            'cutting_angle': 0.0,
            'max_speed': 1.0,
        },
    }

    path = HillBoustrophedon.plan_from_areas(planner, areas)

    assert path
    assert len(path) > 2


def test_one_area_two_obstacles_no_path_inside():
    """一个区域内有两个障碍物时，路径不能因为用全局合并框绕障而进入另一个障碍物。"""
    planner = object.__new__(HillBoustrophedon)
    planner.inner_inflate = 0.3
    planner.swath_spacing = 0.45
    planner.spacing = 0.3
    planner.transit_speed = 0.5
    planner.get_logger = lambda: type('Logger', (), {
        'info': lambda self, *args, **kwargs: None,
        'warn': lambda self, *args, **kwargs: None,
    })()

    # 一个大区域，里面两个分离的障碍物
    area = Polygon([(-10, -5), (10, -5), (10, 5), (-10, 5)])
    obstacle_1 = [(-6, -2), (-4, -2), (-4, 2), (-6, 2)]  # 左侧障碍物
    obstacle_2 = [(4, -2), (6, -2), (6, 2), (4, 2)]      # 右侧障碍物
    inner_rings = [obstacle_1, obstacle_2]

    inflated_obstacles = [Polygon(ring).buffer(0.3) for ring in inner_rings]
    combined = unary_union(inflated_obstacles)

    path = HillBoustrophedon.plan_area(planner, area, inner_rings, 0.0, 1.0)

    # 路径点不能进入任何一个膨胀障碍物
    assert not path_has_points_inside(path, combined), \
        "路径点进入了障碍物内部（可能是绕第一个障碍物时用了全局合并框，导致绕障点落进第二个障碍物）"

    # 路径线段不能穿越任何一个膨胀障碍物
    assert not path_crosses_obstacle(path, combined), \
        "路径线段穿越了障碍物（段间过渡或后处理绕障失败）"


if __name__ == "__main__":
    test_downsample_preserves_obstacle_detour_points()
    test_current_field_geometry_keeps_path_outside_inflated_obstacle()
    test_multiple_areas_create_transit_without_crashing()
    test_one_area_two_obstacles_no_path_inside()
    print("✅ hill_boustrophedon 障碍物回归测试通过")
