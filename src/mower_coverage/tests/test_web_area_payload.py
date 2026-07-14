#!/usr/bin/env python3
"""Behavioral regression tests for the public /web/areas callback seam."""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from std_msgs.msg import String as StringMsg

from mower_coverage.mission.multi_area_definer import MultiAreaDefiner


class _Logger:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.infos = []

    def error(self, message):
        self.errors.append(message)

    def warn(self, message):
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)


def _message(payload):
    return StringMsg(data=json.dumps(payload))


def _valid_payload():
    return {
        'action': 'set_areas',
        'areas': [{
            'name': 'valid_area',
            'points': [
                {'x': 114.05790, 'y': 22.54310},
                {'x': 114.05791, 'y': 22.54310},
                {'x': 114.05791, 'y': 22.54311},
                {'x': 114.05790, 'y': 22.54311},
            ],
        }],
    }


def _harness():
    node = object.__new__(MultiAreaDefiner)
    node.areas = {'previous_area': {'name': 'previous_area'}}
    node.next_color_idx = 4
    node.color_cycle = [(0.0, 0.8, 0.0)]
    node._gps_origin_lat = 22.5431
    node._gps_origin_lng = 114.0579
    node.logger = _Logger()
    node.get_logger = lambda: node.logger
    node.publish_visualization = lambda: None
    node._save_hill_yaml = lambda: None
    return node


def test_invalid_point_shape_is_rejected_atomically_and_next_valid_batch_works():
    node = _harness()
    previous = dict(node.areas)
    invalid = {
        'action': 'set_areas',
        'areas': [{
            'name': 'broken_area',
            'points': [[114.05790, 22.54310],
                       [114.05791, 22.54310],
                       [114.05791, 22.54311]],
        }],
    }

    node.web_areas_callback(_message(invalid))

    assert node.areas == previous
    assert any('Web 区域数据校验失败' in message
               for message in node.logger.errors)

    node.web_areas_callback(_message(_valid_payload()))

    assert list(node.areas) == ['valid_area']


def test_invalid_polygon_is_rejected_atomically():
    node = _harness()
    previous = dict(node.areas)
    invalid = {
        'action': 'set_areas',
        'areas': [{
            'name': 'self_intersecting_area',
            'points': [
                {'x': 114.05790, 'y': 22.54310},
                {'x': 114.05791, 'y': 22.54311},
                {'x': 114.05791, 'y': 22.54310},
                {'x': 114.05790, 'y': 22.54311},
            ],
        }],
    }

    node.web_areas_callback(_message(invalid))

    assert node.areas == previous
    assert any('Web 区域数据校验失败' in message
               for message in node.logger.errors)


if __name__ == '__main__':
    test_invalid_point_shape_is_rejected_atomically_and_next_valid_batch_works()
    test_invalid_polygon_is_rejected_atomically()
    print('✅ Web area payload regression test passed')
