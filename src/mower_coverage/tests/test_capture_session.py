#!/usr/bin/env python3
"""Behavior tests for the single-object remote capture session."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_coverage.mission.capture import CaptureSession


PROFILE = {
    'closure_tolerance': 0.5,
    'simplify_tolerance': 0.01,
}


def _sample(x, y, timestamp, localization_ok=True):
    return {
        'x': x,
        'y': y,
        'timestamp': timestamp,
        'frame_id': 'map',
        'localization_ok': localization_ok,
    }


def _rectangle_samples(localization_ok=True):
    return [
        _sample(0.0, 0.0, 1.0, localization_ok),
        _sample(4.0, 0.0, 2.0, localization_ok),
        _sample(4.0, 3.0, 3.0, localization_ok),
        _sample(0.0, 3.0, 4.0, localization_ok),
        _sample(0.1, 0.1, 5.0, localization_ok),
    ]


class CaptureSessionTests(unittest.TestCase):
    def test_one_object_capture_preserves_raw_trajectory_until_confirmation(self):
        session = CaptureSession('area-1', 'work_area', PROFILE)
        session.start()
        for sample in _rectangle_samples():
            session.record_pose(sample)

        snapshot = session.finish()

        self.assertEqual(snapshot['state'], 'ready')
        self.assertEqual(
            snapshot['raw_trajectory'], _rectangle_samples())
        self.assertEqual(
            snapshot['geometry'],
            [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
             [0.0, 3.0], [0.0, 0.0]],
        )

        confirmed = session.confirm()

        self.assertEqual(confirmed['status'], 'confirmed')
        self.assertEqual(confirmed['id'], 'area-1')
        self.assertEqual(confirmed['type'], 'work_area')
        self.assertEqual(confirmed['raw_trajectory'], _rectangle_samples())

    def test_unhealthy_localization_can_only_be_saved_as_draft(self):
        session = CaptureSession('pond-1', 'no_go_zone', PROFILE)
        session.start()
        for sample in _rectangle_samples(localization_ok=False):
            session.record_pose(sample)

        snapshot = session.finish()

        self.assertEqual(snapshot['state'], 'draft')
        self.assertEqual(snapshot['geometry'], [])
        self.assertTrue(snapshot['issues'])
        self.assertEqual(session.save_draft()['status'], 'draft')
        with self.assertRaisesRegex(RuntimeError, 'only when ready'):
            session.confirm()

    def test_undo_reopens_a_finished_capture_for_editing(self):
        session = CaptureSession('corridor-1', 'corridor', PROFILE)
        session.start()
        session.record_pose(_sample(0.0, 0.0, 1.0))
        session.record_pose(_sample(5.0, 0.0, 2.0))
        self.assertEqual(session.finish()['state'], 'ready')

        removed = session.undo()

        self.assertEqual(removed['x'], 5.0)
        self.assertEqual(session.state, 'capturing')
        self.assertEqual(session.snapshot()['raw_trajectory'], [
            _sample(0.0, 0.0, 1.0),
        ])

    def test_corridor_confirmation_requires_and_persists_route_metadata(self):
        session = CaptureSession('corridor-1', 'corridor', PROFILE)
        session.start()
        session.record_pose(_sample(0.0, 0.0, 1.0))
        session.record_pose(_sample(5.0, 0.0, 2.0))
        session.finish()

        with self.assertRaisesRegex(RuntimeError, 'metadata'):
            session.confirm()

        with self.assertRaisesRegex(ValueError, 'width'):
            session.set_corridor_metadata(
                0.0, 'area-1', 'area-2', bidirectional=True)

        session.set_corridor_metadata(
            1.0, 'area-1', 'area-2', bidirectional=True)
        confirmed = session.confirm()

        self.assertEqual(confirmed['width'], 1.0)
        self.assertEqual(confirmed['from_work_area_id'], 'area-1')
        self.assertEqual(confirmed['to_work_area_id'], 'area-2')
        self.assertTrue(confirmed['bidirectional'])

    def test_state_and_sample_contracts_fail_closed(self):
        session = CaptureSession('area-1', 'work_area', PROFILE)
        with self.assertRaisesRegex(RuntimeError, 'not capturing'):
            session.record_pose(_sample(0.0, 0.0, 1.0))

        session.start()
        with self.assertRaisesRegex(ValueError, 'finite'):
            session.record_pose(_sample(float('nan'), 0.0, 1.0))
        with self.assertRaisesRegex(ValueError, 'frame_id'):
            session.record_pose({
                'x': 0.0,
                'y': 0.0,
                'timestamp': 1.0,
                'frame_id': 'odom',
                'localization_ok': True,
            })

    def test_local_odom_requires_an_explicit_simulation_fallback(self):
        with self.assertRaisesRegex(ValueError, 'simulation'):
            CaptureSession(
                'area-1', 'work_area', PROFILE, frame_id='odom')

        session = CaptureSession(
            'area-1', 'work_area', PROFILE,
            frame_id='odom', allow_local_odom=True)
        session.start()
        session.record_pose({
            'x': 0.0,
            'y': 0.0,
            'timestamp': 1.0,
            'frame_id': 'odom',
            'localization_ok': True,
        })
        self.assertEqual(len(session.snapshot()['raw_trajectory']), 1)

    def test_confirmation_is_not_available_before_finish_or_after_cancel(self):
        session = CaptureSession('area-1', 'work_area', PROFILE)
        session.start()
        with self.assertRaisesRegex(RuntimeError, 'only when ready'):
            session.confirm()

        session.cancel()
        with self.assertRaisesRegex(RuntimeError, 'only when ready'):
            session.confirm()

    def test_finish_rechecks_current_localization_health(self):
        session = CaptureSession('area-1', 'work_area', PROFILE)
        session.start()
        for sample in _rectangle_samples():
            session.record_pose(sample)

        snapshot = session.finish('YELLOW')

        self.assertEqual(snapshot['state'], 'draft')
        self.assertEqual(snapshot['geometry'], [])
        self.assertIn('finish', ' '.join(snapshot['issues']))
        with self.assertRaisesRegex(RuntimeError, 'only when ready'):
            session.confirm('GREEN')

    def test_confirm_rechecks_current_localization_health(self):
        session = CaptureSession('area-1', 'work_area', PROFILE)
        session.start()
        for sample in _rectangle_samples():
            session.record_pose(sample)
        self.assertEqual(session.finish('GREEN')['state'], 'ready')

        with self.assertRaisesRegex(RuntimeError, 'YELLOW'):
            session.confirm('YELLOW')

        self.assertEqual(session.state, 'draft')
        self.assertEqual(session.snapshot()['geometry'], [])
        session.finish('GREEN')
        self.assertEqual(session.confirm('GREEN')['status'], 'confirmed')


if __name__ == '__main__':
    unittest.main()
