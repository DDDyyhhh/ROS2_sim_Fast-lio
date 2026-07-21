#!/usr/bin/env python3
"""Behavior tests for multi-object capture storage."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

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
