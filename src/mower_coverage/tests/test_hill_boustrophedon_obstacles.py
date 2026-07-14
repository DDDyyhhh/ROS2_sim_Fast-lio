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


def test_multi_area_transit_avoids_next_area_obstacle():
    """区域间 transit 不能穿过下一个区域膨胀后的障碍物。"""
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

    # 第一个区域的终点在 (-2, 1.6)，第二个区域的最近边界点为
    # (2, 1.6)。障碍物贴近第二个区域左边界，膨胀后会延伸到 x=1.8，
    # 因此当前的区域间直线会直接穿过它。
    obstacle = [(2.1, 1.0), (3.0, 1.0), (3.0, 2.0), (2.1, 2.0)]
    areas = {
        '区域 1': {
            'points': [(-6, -2), (-2, -2), (-2, 2), (-6, 2)],
            'inner_rings': [],
            'cutting_angle': 0.0,
            'max_speed': 1.0,
        },
        '区域 2': {
            'points': [(2, -2), (6, -2), (6, 2), (2, 2)],
            'inner_rings': [obstacle],
            'cutting_angle': 0.0,
            'max_speed': 1.0,
        },
    }

    path = planner.plan_from_areas(areas)
    inflated_obstacle = Polygon(obstacle).buffer(0.3)

    assert not path_has_points_inside(path, inflated_obstacle), path
    assert not path_crosses_obstacle(path, inflated_obstacle), path

    # 规划器发布给 Web/RViz 的是降采样路径，降采样后也必须保持安全。
    sampled = planner._downsample(path, step=10)
    assert not path_has_points_inside(sampled, inflated_obstacle), sampled
    assert not path_crosses_obstacle(sampled, inflated_obstacle), sampled


def test_global_detour_avoids_all_separated_obstacles():
    """全局绕障新增的线段也不能穿过另一个分离障碍物。"""
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

    # 第一个障碍物触发全局 transit 绕障；第二个障碍物位于候选绕障
    # 竖直段中。只检查候选点会误以为安全，但新增线段会穿过第二个障碍物。
    inner_rings = [
        [(2.05, 1.0), (3.0, 1.0), (3.0, 1.8), (2.05, 1.8)],
        [(3.5, -0.4), (4.5, -0.4), (4.5, 0.6), (3.5, 0.6)],
    ]
    areas = {
        '区域 1': {
            'points': [(-8, -2), (-2, -2), (-2, 2), (-8, 2)],
            'inner_rings': [],
            'cutting_angle': 0.0,
            'max_speed': 1.0,
        },
        '区域 2': {
            'points': [(2, -2), (12, -2), (12, 2), (2, 2)],
            'inner_rings': inner_rings,
            'cutting_angle': 0.0,
            'max_speed': 1.0,
        },
    }

    path = planner.plan_from_areas(areas)
    inflated_obstacles = [Polygon(ring).buffer(0.3) for ring in inner_rings]
    combined = unary_union(inflated_obstacles)

    assert not path_has_points_inside(path, combined), path
    assert not path_crosses_obstacle(path, combined), path

    sampled = planner._downsample(path, step=10)
    assert not path_has_points_inside(sampled, combined), sampled
    assert not path_crosses_obstacle(sampled, combined), sampled


def test_angled_obstacle_does_not_delete_area_two_coverage():
    """斜四边形障碍物不能让后处理删除区域后半段覆盖路径。"""
    planner = object.__new__(HillBoustrophedon)
    planner.inner_inflate = 0.3
    planner.swath_spacing = 0.45
    planner.spacing = 0.3
    planner.transit_speed = 0.5
    planner.get_logger = lambda: type('Logger', (), {
        'info': lambda self, *args, **kwargs: None,
        'warn': lambda self, *args, **kwargs: None,
    })()

    # 来自 Web 复现的区域 2 几何：当前后处理会在障碍物下边界卡住，
    # 只留下 y≈10m 以下的少量路径，而 y≈20m 以上区域完全没有覆盖。
    area = Polygon([
        (-13.714977842826407, 23.85120281683264),
        (-15.231542502868553, 7.432372433621168),
        (9.998578666292058, 9.48472719863723),
        (9.3092310923502, 26.177202358157103),
        (-12.336282696403769, 23.85120281683264),
    ])
    obstacle = [
        (-6.821502112174298, 17.936834187142523),
        (1.8642773057589763, 19.85481215042057),
        (-4.4777203631097064, 9.990922907985933),
        (-10.681848522742113, 11.360907906788),
    ]

    path = planner.plan_area(area, [obstacle], 0.0, 1.0)
    inflated_obstacle = Polygon(obstacle).buffer(0.3)

    assert len(path) > 1000, f'区域覆盖路径过少: {len(path)}'
    assert max(point[1] for point in path) > 20.0, path[-20:]
    assert not path_has_points_inside(path, inflated_obstacle)
    assert not path_crosses_obstacle(path, inflated_obstacle)


def test_two_areas_keep_second_area_coverage_with_obstacles():
    """两个各自带障碍物的区域不能因全局后处理丢掉第二个区域。"""
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
            'points': [
                (17.581401969424945, 13.452612292496156),
                (16.478445851702404, -6.249999714315564),
                (41.43282798631861, -8.028708992990943),
                (40.88134993330165, 10.989787183188184),
            ],
            'inner_rings': [[
                (22.82044352057109, 5.653664748198537),
                (22.131095945168152, -2.282110526382226),
                (31.092614396184757, -2.692581773530449),
                (27.094398471997433, 6.885077785108962),
            ]],
            'cutting_angle': 0.0,
            'max_speed': 1.0,
        },
        '区域 2': {
            'points': [
                (-13.714977842826407, 23.85120281683264),
                (-15.231542502868553, 7.432372433621168),
                (9.998578666292058, 9.48472719863723),
                (9.3092310923502, 26.177202358157103),
                (-12.336282696403769, 23.85120281683264),
            ],
            'inner_rings': [[
                (-6.821502112174298, 17.936834187142523),
                (1.8642773057589763, 19.85481215042057),
                (-4.4777203631097064, 9.9909229071097064),
                (-10.681848522742113, 11.360907906788),
            ]],
            'cutting_angle': 0.0,
            'max_speed': 1.0,
        },
    }

    path = planner.plan_from_areas(areas)
    area_two = Polygon(areas['区域 2']['points'])
    obstacles = [
        Polygon(ring).buffer(0.3)
        for area in areas.values()
        for ring in area['inner_rings']
    ]
    combined = unary_union(obstacles)
    area_two_points = [point for point in path
                       if area_two.buffer(0.001).covers(Point(point[:2]))]

    assert len(area_two_points) > 1000, (
        f'区域 2 覆盖路径过少: {len(area_two_points)} / {len(path)}')
    assert max(point[1] for point in area_two_points) > 20.0
    assert not path_has_points_inside(path, combined)
    assert not path_crosses_obstacle(path, combined)


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
    test_multi_area_transit_avoids_next_area_obstacle()
    test_global_detour_avoids_all_separated_obstacles()
    test_angled_obstacle_does_not_delete_area_two_coverage()
    test_two_areas_keep_second_area_coverage_with_obstacles()
    test_one_area_two_obstacles_no_path_inside()
    print("✅ hill_boustrophedon 障碍物回归测试通过")
