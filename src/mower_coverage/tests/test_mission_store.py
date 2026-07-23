#!/usr/bin/env python3
"""Behavior tests for multi-object capture storage."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_coverage.mission.model import validate_mission
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
        'frame_id': 'odom',
        'localization_ok': True,
    }


class MissionCaptureStoreTests(unittest.TestCase):
    def test_delete_work_area_rejects_referenced_corridor_atomically(self):
        store = MissionCaptureStore(PROFILE, frame_id='map')
        store.load_document({
            'version': 1,
            'frame_id': 'map',
            'objects': [
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [],
                },
                {
                    'id': 'area-2',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [],
                },
                {
                    'id': 'corridor-1',
                    'type': 'corridor',
                    'status': 'confirmed',
                    'geometry': [],
                    'from_work_area_id': 'area-1',
                    'to_work_area_id': 'area-2',
                },
            ],
            'drafts': [],
            'order': ['area-1', 'corridor-1', 'area-2'],
        })
        before = store.document()

        with self.assertRaisesRegex(RuntimeError, 'corridor-1'):
            store.delete_object('area-1')

        self.assertEqual(store.document(), before)

    def test_confirmed_objects_accumulate_and_order_excludes_no_go(self):
        store = MissionCaptureStore(PROFILE, frame_id='odom',
                                    allow_local_odom=True)

        store.start('work_area', 'area-1')
        for point in [(0, 0), (4, 0), (4, 3), (0, 3), (0.1, 0.1)]:
            store.record_pose(sample(*point, point[0] + point[1] + 1.0))
        store.finish()
        store.confirm()

        store.start('no_go_zone', 'pond-1')
        for point in [(1, 1), (2, 1), (2, 2), (1, 2), (1.1, 1.1)]:
            store.record_pose(sample(*point, point[0] + point[1] + 10.0))
        store.finish()
        store.confirm()

        self.assertEqual([item['id'] for item in store.objects],
                         ['area-1', 'pond-1'])
        self.assertEqual(store.order, ['area-1'])

    def test_corridor_confirmed_after_areas_is_inserted_between_endpoints(self):
        store = MissionCaptureStore(PROFILE, frame_id='odom',
                                    allow_local_odom=True)

        for object_id, points in [
            ('area-1', [(0, 0), (2, 0), (2, 2), (0, 2), (0.1, 0.1)]),
            ('area-2', [(6, 0), (8, 0), (8, 2), (6, 2), (6.1, 0.1)]),
        ]:
            store.start('work_area', object_id)
            for index, point in enumerate(points):
                store.record_pose(sample(*point, index + 1.0))
            store.finish()
            store.confirm()

        store.start('corridor', 'corridor-1')
        store.record_pose(sample(2, 1, 1.0))
        store.record_pose(sample(6, 1, 2.0))
        store.finish()
        store.set_corridor_metadata(1.0, 'area-1', 'area-2', True)
        store.confirm()

        mission = store.document()
        self.assertEqual(
            mission['order'], ['area-1', 'corridor-1', 'area-2'])
        result = validate_mission(
            mission,
            {'robot_length': 0.4, 'robot_width': 0.3,
             'safety_margin': 0.1},
        )
        self.assertTrue(result.valid, result.issues)

    def test_non_adjacent_corridor_does_not_reorder_existing_work(self):
        store = MissionCaptureStore(PROFILE, frame_id='odom',
                                    allow_local_odom=True)
        store.load_document({
            'version': 1,
            'frame_id': 'odom',
            'objects': [
                {
                    'id': 'area-1', 'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[0, 0], [2, 0], [2, 2], [0, 2], [0, 0]],
                },
                {
                    'id': 'area-3', 'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[3, 0], [5, 0], [5, 2], [3, 2], [3, 0]],
                },
                {
                    'id': 'area-2', 'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[6, 0], [8, 0], [8, 2], [6, 2], [6, 0]],
                },
            ],
            'drafts': [],
            'order': ['area-1', 'area-3', 'area-2'],
        })

        store.start('corridor', 'corridor-1')
        store.record_pose(sample(2, 1, 1.0))
        store.record_pose(sample(6, 1, 2.0))
        store.finish()
        store.set_corridor_metadata(1.0, 'area-1', 'area-2', True)
        store.confirm()

        self.assertEqual(
            store.order, ['area-1', 'area-3', 'area-2', 'corridor-1'])

    def test_save_draft_does_not_enter_execution_order(self):
        store = MissionCaptureStore(PROFILE, frame_id='odom',
                                    allow_local_odom=True)
        store.start('work_area', 'area-draft')
        store.record_pose(sample(0, 0, 1.0))
        draft = store.save_draft()

        self.assertEqual(draft['status'], 'draft')
        self.assertEqual(store.objects, [])
        self.assertEqual(store.drafts[0]['id'], 'area-draft')
        self.assertEqual(store.order, [])

    def test_non_closed_finished_draft_keeps_route_and_closure_feedback(self):
        store = MissionCaptureStore(PROFILE, frame_id='odom',
                                    allow_local_odom=True)
        store.start('work_area', 'area-draft')
        samples = [
            sample(0.0, 0.0, 1.0),
            sample(4.0, 0.0, 2.0),
            sample(4.0, 3.0, 3.0),
            sample(0.0, 3.0, 4.0),
        ]
        for item in samples:
            store.record_pose(item)
        store.finish()

        draft = store.save_draft()

        self.assertEqual(draft['status'], 'draft')
        self.assertEqual(draft['raw_trajectory'], samples)
        self.assertEqual(draft['start_point'], {'x': 0.0, 'y': 0.0})
        self.assertAlmostEqual(draft['closure_distance'], 3.0)
        self.assertEqual(draft['closure_tolerance'], 0.5)

    def test_corridor_metadata_is_confirmed_with_the_object(self):
        store = MissionCaptureStore(PROFILE, frame_id='odom',
                                    allow_local_odom=True)
        store.start('corridor', 'route-1')
        store.record_pose(sample(0, 0, 1.0))
        store.record_pose(sample(5, 0, 2.0))
        store.finish()
        store.set_corridor_metadata(1.2, 'area-1', 'area-2', True)
        corridor = store.confirm()

        self.assertEqual(corridor['width'], 1.2)
        self.assertEqual(corridor['from_work_area_id'], 'area-1')
        self.assertEqual(store.order, ['route-1'])


if __name__ == '__main__':
    unittest.main()
