#!/usr/bin/env python3
"""Regression tests for the remote-capture ROS adapter."""

import os
import sys
import tempfile
import time
import unittest
import json
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_coverage.mission.remote_capture_node import RemoteCaptureNode
from mower_coverage.mission.store import MissionCaptureStore


PROFILE = {
    'closure_tolerance': 0.5,
    'simplify_tolerance': 0.01,
}


def sample(x, y, timestamp):
    return {
        'x': x,
        'y': y,
        'timestamp': timestamp,
        'frame_id': 'map',
        'localization_ok': True,
    }


class RemoteCaptureNodeTests(unittest.TestCase):
    def test_movement_permission_does_not_require_active_sampling(self):
        node = object.__new__(RemoteCaptureNode)
        node.store = MissionCaptureStore(PROFILE, frame_id='map')
        node.pose_timeout = 0.5
        node.latest_pose_at = time.monotonic()
        node.health_state = 'GREEN'
        node.publish_all = lambda: None

        self.assertTrue(node._drive_allowed())
        self.assertFalse(node._sampling_allowed())

        node.tick()

        self.assertIsNone(node.store.snapshot()['active'])

    def test_confirmed_object_keeps_movement_for_next_capture(self):
        node = object.__new__(RemoteCaptureNode)
        node.store = MissionCaptureStore(PROFILE, frame_id='map')
        node.pose_timeout = 0.5
        node.latest_pose_at = time.monotonic()
        node.health_state = 'GREEN'

        node.store.start('work_area', 'area-1')
        for point in [(0, 0), (4, 0), (4, 3), (0, 3), (0.1, 0.1)]:
            node.store.record_pose(sample(*point, point[0] + point[1] + 1.0))
        node.store.finish('GREEN')
        node.store.confirm('GREEN')

        self.assertTrue(node._drive_allowed())
        self.assertFalse(node._sampling_allowed())

        node.store.start('no_go_zone', 'pond-1')

        self.assertTrue(node._drive_allowed())
        self.assertTrue(node._sampling_allowed())

    def test_delete_command_removes_only_selected_object_after_reload(self):
        node = object.__new__(RemoteCaptureNode)
        node.store = MissionCaptureStore(PROFILE, frame_id='map')
        node.store.load_document({
            'version': 1,
            'frame_id': 'map',
            'objects': [
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0],
                                 [0.0, 1.0], [0.0, 0.0]],
                },
                {
                    'id': 'pond-1',
                    'type': 'no_go_zone',
                    'status': 'confirmed',
                    'geometry': [[2.0, 2.0], [3.0, 2.0], [3.0, 3.0],
                                 [2.0, 3.0], [2.0, 2.0]],
                },
            ],
            'drafts': [],
            'order': ['area-1'],
        })
        node.last_error = ''
        node.publish_all = lambda: None
        node.get_logger = lambda: SimpleNamespace(
            warn=lambda _message: None)

        with tempfile.TemporaryDirectory() as directory:
            node.mission_file = os.path.join(directory, 'mission.yaml')
            node._persist()
            node.command_callback(SimpleNamespace(data=json.dumps({
                'action': 'delete',
                'id': 'area-1',
            })))

            self.assertEqual(node.last_error, '')
            self.assertEqual([item['id'] for item in node.store.objects],
                             ['pond-1'])
            self.assertEqual(node.store.order, [])

            reloaded = MissionCaptureStore(PROFILE, frame_id='map')
            reloaded_node = object.__new__(RemoteCaptureNode)
            reloaded_node.store = reloaded
            reloaded_node.mission_file = node.mission_file
            reloaded_node.last_error = ''
            reloaded_node._load_mission()

            self.assertEqual([item['id'] for item in reloaded.objects],
                             ['pond-1'])
            self.assertEqual(reloaded.order, [])

    def test_non_closed_finish_keeps_sampling_until_explicit_finish(self):
        node = object.__new__(RemoteCaptureNode)
        node.store = MissionCaptureStore(PROFILE, frame_id='map')
        node.store.start('work_area', 'area-1')
        for point in [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]:
            node.store.record_pose(sample(*point, point[0] + point[1] + 1.0))

        unfinished = node.store.finish('GREEN')

        self.assertEqual(unfinished['state'], 'draft')
        self.assertEqual(len(unfinished['raw_trajectory']), 4)
        self.assertEqual(unfinished['geometry'], [])

        node.pose_timeout = 0.5
        node.latest_pose = SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.2, y=0.2),
                ),
            ),
        )
        node.latest_pose_at = time.monotonic()
        node.latest_pose_stamp = 5.0
        node.capture_frame_id = 'map'
        node.health_state = 'GREEN'
        node._last_sample_stamp = None
        node.last_error = ''
        node.publish_all = lambda: None

        node.tick()

        active = node.store.snapshot()['active']
        self.assertEqual(len(active['raw_trajectory']), 5)
        self.assertEqual(active['state'], 'draft')
        self.assertEqual(active['geometry'], [])
        self.assertTrue(node._drive_allowed())

        ready = node.store.finish('GREEN')
        self.assertEqual(ready['state'], 'ready')

    def test_invalid_closed_geometry_stays_draft_after_next_pose_sample(self):
        node = object.__new__(RemoteCaptureNode)
        node.store = MissionCaptureStore(PROFILE, frame_id='map')
        node.store.start('work_area', 'area-1')
        for point in [
                (0.0, 0.0), (4.0, 4.0), (0.0, 4.0),
                (4.0, 0.0), (0.1, 0.1)]:
            node.store.record_pose(sample(*point, point[0] + point[1] + 1.0))

        unfinished = node.store.finish('GREEN')

        self.assertEqual(unfinished['state'], 'draft')
        self.assertTrue(any('valid polygon' in issue
                            for issue in unfinished['issues']))

        node.pose_timeout = 0.5
        node.latest_pose = SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=0.1, y=0.1),
                ),
            ),
        )
        node.latest_pose_at = time.monotonic()
        node.latest_pose_stamp = 6.0
        node.capture_frame_id = 'map'
        node.health_state = 'GREEN'
        node._last_sample_stamp = None
        node.last_error = ''
        node.publish_all = lambda: None

        node.tick()

        active = node.store.snapshot()['active']
        self.assertEqual(active['state'], 'draft')
        self.assertTrue(any('valid polygon' in issue
                            for issue in active['issues']))
        self.assertEqual(active['geometry'], [])

    def test_manual_geometry_command_updates_active_capture(self):
        node = object.__new__(RemoteCaptureNode)
        node.store = MissionCaptureStore(PROFILE, frame_id='map')
        node.store.start('work_area', 'area-1')
        for point in [
                (0.0, 0.0), (4.0, 4.0), (0.0, 4.0),
                (4.0, 0.0), (0.1, 0.1)]:
            node.store.record_pose(sample(*point, point[0] + point[1] + 1.0))
        node.store.finish('GREEN')
        node.health_state = 'GREEN'
        node.last_error = ''
        node.publish_all = lambda: None
        node.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)

        node.command_callback(SimpleNamespace(data=json.dumps({
            'action': 'manual_geometry',
            'geometry': [
                [0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
                [0.0, 3.0], [0.0, 0.0],
            ],
        })))

        active = node.store.snapshot()['active']
        self.assertEqual(active['state'], 'ready')
        self.assertEqual(active['geometry'], [
            [0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
            [0.0, 3.0], [0.0, 0.0],
        ])
        self.assertEqual(len(active['raw_trajectory']), 5)


if __name__ == '__main__':
    unittest.main()
