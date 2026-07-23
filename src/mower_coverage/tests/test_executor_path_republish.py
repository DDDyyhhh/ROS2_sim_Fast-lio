"""Regression tests for repeated planner path publications."""

import json

from nav_msgs.msg import Path
from std_msgs.msg import String

from mower_coverage.execution.multi_area_executor import MultiAreaExecutor


def _path(points):
    message = Path()
    for x, y in points:
        pose = type('PoseStampedLike', (), {})()
        pose.pose = type('PoseLike', (), {})()
        pose.pose.position = type('PointLike', (), {})()
        pose.pose.position.x = x
        pose.pose.position.y = y
        message.poses.append(pose)
    return message


def test_republished_completed_path_does_not_reset_progress():
    node = object.__new__(MultiAreaExecutor)
    node.waypoints = []
    node.current_index = 0
    node.total_waypoints = 0
    node.executing = False
    node.path_received = False
    node.path_signature = None
    node.get_logger = lambda: type('Logger', (), {
        'info': lambda self, *args, **kwargs: None,
    })()

    message = _path([(0.0, 0.0), (1.0, 0.0)])
    node.path_callback(message)
    node.current_index = 2

    node.path_callback(message)

    assert node.current_index == 2
    assert node.total_waypoints == 2


def test_changed_path_still_resets_progress():
    node = object.__new__(MultiAreaExecutor)
    node.waypoints = []
    node.current_index = 0
    node.total_waypoints = 0
    node.executing = False
    node.path_received = False
    node.path_signature = None
    node.get_logger = lambda: type('Logger', (), {
        'info': lambda self, *args, **kwargs: None,
    })()

    node.path_callback(_path([(0.0, 0.0), (1.0, 0.0)]))
    node.current_index = 1
    node.path_callback(_path([(0.0, 0.0), (2.0, 0.0)]))

    assert node.current_index == 0


def test_path_metadata_excludes_corridor_points_from_coverage():
    node = object.__new__(MultiAreaExecutor)
    node.waypoints = []
    node.current_index = 0
    node.total_waypoints = 0
    node.executing = False
    node.path_received = False
    node.path_signature = None
    node.pending_path_metadata = None
    node.get_logger = lambda: type('Logger', (), {
        'info': lambda self, *args, **kwargs: None,
        'warn': lambda self, *args, **kwargs: None,
    })()

    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    node.path_callback(_path(points))
    metadata = String()
    metadata.data = json.dumps({
        'path_signature': [[x, y] for x, y in points],
        'waypoint_types': ['coverage', 'transit', 'transit', 'coverage'],
    })
    node.path_metadata_callback(metadata)

    assert node.waypoint_is_coverage == [True, False, False, True]
    assert node.coverage_total == 2
