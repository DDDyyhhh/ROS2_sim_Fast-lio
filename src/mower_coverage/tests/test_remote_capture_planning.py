"""Regression tests for the remote-capture planning profile."""

from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]


def _corridor_mission(bidirectional=False):
    return {
        'objects': [
            {
                'id': 'area-1',
                'type': 'work_area',
                'status': 'confirmed',
                'geometry': [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0],
                             [0.0, 2.0], [0.0, 0.0]],
            },
            {
                'id': 'corridor-1',
                'type': 'corridor',
                'status': 'confirmed',
                'geometry': [[2.0, 1.0], [4.0, 1.0], [6.0, 1.0],
                             [8.0, 1.0]],
                'width': 1.0,
                'from_work_area_id': 'area-1',
                'to_work_area_id': 'area-2',
                'bidirectional': bidirectional,
            },
            {
                'id': 'area-2',
                'type': 'work_area',
                'status': 'confirmed',
                'geometry': [[8.0, 0.0], [10.0, 0.0], [10.0, 2.0],
                             [8.0, 2.0], [8.0, 0.0]],
            },
        ],
        'order': ['area-1', 'corridor-1', 'area-2'],
    }


def test_safe_remote_capture_profile_starts_the_lidar_scan_chain():
    launch = (
        REPO_ROOT / 'src' / 'mower_coverage' / 'launch'
        / 'remote_capture_sim.launch.py'
    ).read_text()

    declaration = (
        "DeclareLaunchArgument('with_lidar_scan', default_value='true')"
    )
    assert declaration in launch
    assert '_validate_private_simulation_topics' in launch


def test_capture_planning_handoff_keeps_corridor_semantics_in_web():
    app = (
        REPO_ROOT / 'src' / 'mower_coverage' / 'web_frontend' / 'app.js'
    ).read_text()

    assert "name: '/web/mission'" in app
    assert "action: 'set_mission'" in app
    assert '当前旧规划器暂不支持通道' not in app
    assert 'safety_stop_reason' in app


def test_mission_planner_consumes_recorded_corridor_centerline():
    from mower_coverage.planning.hill_boustrophedon import HillBoustrophedon

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

    mission = _corridor_mission()

    path = planner.plan_from_mission(
        mission,
        robot_length=0.4,
        robot_width=0.3,
        safety_margin=0.1,
    )

    assert path
    assert any(point[:2] == (4.0, 1.0) for point in path)
    assert any(point[:2] == (6.0, 1.0) for point in path)
    sampled = planner._downsample(path, step=10)
    assert any(point[:2] == (4.0, 1.0) for point in sampled)
    assert any(point[:2] == (6.0, 1.0) for point in sampled)


def test_one_way_corridor_rejects_reverse_execution_order():
    from mower_coverage.mission.model import validate_mission

    mission = _corridor_mission()
    mission['order'] = ['area-2', 'corridor-1', 'area-1']

    result = validate_mission(
        mission,
        {'robot_length': 0.4, 'robot_width': 0.3, 'safety_margin': 0.1},
    )

    assert not result.valid
    assert any('direction' in issue for issue in result.issues)


def test_planner_rejects_cross_area_order_without_explicit_corridor():
    from mower_coverage.planning.hill_boustrophedon import HillBoustrophedon

    planner = object.__new__(HillBoustrophedon)
    planner.inner_inflate = 0.3
    planner.transit_speed = 0.5
    planner.last_waypoints = []
    planner.last_obstacles = []
    planner.publish_path = lambda waypoints: None

    mission = _corridor_mission()
    mission['objects'] = [
        item for item in mission['objects'] if item['type'] != 'corridor'
    ]
    mission['order'] = ['area-1', 'area-2']

    with pytest.raises(ValueError, match='explicit corridor'):
        planner.plan_from_mission(
            mission,
            robot_length=0.4,
            robot_width=0.3,
            safety_margin=0.1,
        )
