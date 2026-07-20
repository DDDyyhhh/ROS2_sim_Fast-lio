#!/usr/bin/env python3
"""Public behavior tests for captured mission geometry."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_coverage.mission.model import (
    derive_effective_geometry,
    legacy_areas_to_mission,
    validate_mission,
)


class MissionGeometryTests(unittest.TestCase):
    def test_missing_trajectory_returns_invalid_result(self):
        result = derive_effective_geometry(
            None,
            'work_area',
            {'closure_tolerance': 0.5, 'simplify_tolerance': 0.01},
        )

        self.assertEqual(result.status, 'invalid')
        self.assertIn('trajectory', result.issues[0])

    def test_missing_geometry_profile_returns_invalid_result(self):
        result = derive_effective_geometry(
            [
                {'x': 0.0, 'y': 0.0},
                {'x': 4.0, 'y': 0.0},
                {'x': 4.0, 'y': 3.0},
                {'x': 0.0, 'y': 3.0},
                {'x': 0.0, 'y': 0.0},
            ],
            'work_area',
            {},
        )

        self.assertEqual(result.status, 'invalid')
        self.assertIn('geometry profile', result.issues[0])

    def test_near_closed_work_area_becomes_closed_polygon(self):
        trajectory = [
            {'x': 0.0, 'y': 0.0, 'localization_ok': True},
            {'x': 4.0, 'y': 0.0, 'localization_ok': True},
            {'x': 4.0, 'y': 3.0, 'localization_ok': True},
            {'x': 0.0, 'y': 3.0, 'localization_ok': True},
            {'x': 0.2, 'y': 0.1, 'localization_ok': True},
        ]

        result = derive_effective_geometry(
            trajectory,
            'work_area',
            {
                'closure_tolerance': 0.5,
                'simplify_tolerance': 0.01,
            },
        )

        self.assertEqual(result.status, 'ready')
        self.assertEqual(
            result.geometry,
            ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)),
        )
        self.assertEqual(result.raw_trajectory, tuple(trajectory))

    def test_localization_failure_keeps_capture_as_draft(self):
        trajectory = [
            {'x': 0.0, 'y': 0.0, 'localization_ok': True},
            {'x': 4.0, 'y': 0.0, 'localization_ok': False},
            {'x': 4.0, 'y': 3.0, 'localization_ok': True},
            {'x': 0.0, 'y': 3.0, 'localization_ok': True},
            {'x': 0.1, 'y': 0.1, 'localization_ok': True},
        ]

        result = derive_effective_geometry(
            trajectory,
            'work_area',
            {'closure_tolerance': 0.5, 'simplify_tolerance': 0.01},
        )

        self.assertEqual(result.status, 'draft')
        self.assertEqual(result.geometry, ())
        self.assertIn('localization', result.issues[0])

    def test_corridor_keeps_an_open_centerline(self):
        trajectory = [
            {'x': 0.0, 'y': 0.0, 'localization_ok': True},
            {'x': 5.0, 'y': 0.0, 'localization_ok': True},
        ]

        result = derive_effective_geometry(
            trajectory,
            'corridor',
            {'closure_tolerance': 0.5, 'simplify_tolerance': 0.01},
        )

        self.assertEqual(result.status, 'ready')
        self.assertEqual(result.geometry, ((0.0, 0.0), (5.0, 0.0)))

    def test_confirmed_work_area_is_accepted_as_a_mission(self):
        mission = {
            'objects': [{
                'id': 'area-1',
                'type': 'work_area',
                'status': 'confirmed',
                'geometry': [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
                             [0.0, 3.0], [0.0, 0.0]],
            }],
            'order': ['area-1'],
        }

        result = validate_mission(
            mission,
            {
                'robot_length': 0.46,
                'robot_width': 0.40,
                'safety_margin': 0.2,
            },
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.issues, ())

    def test_missing_robot_profile_fails_closed(self):
        mission = {
            'objects': [{
                'id': 'area-1',
                'type': 'work_area',
                'status': 'confirmed',
                'geometry': [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
                             [0.0, 3.0], [0.0, 0.0]],
            }],
            'order': ['area-1'],
        }

        result = validate_mission(mission, {})

        self.assertFalse(result.valid)
        self.assertTrue(any('robot profile' in issue for issue in result.issues))

    def test_draft_object_cannot_be_validated_for_planning(self):
        mission = {
            'objects': [{
                'id': 'area-draft',
                'type': 'work_area',
                'status': 'draft',
                'geometry': [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
                             [0.0, 3.0], [0.0, 0.0]],
            }],
            'order': ['area-draft'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('not confirmed' in issue for issue in result.issues))

    def test_duplicate_object_ids_are_rejected(self):
        mission = {
            'objects': [
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
                                 [0.0, 3.0], [0.0, 0.0]],
                },
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[5.0, 0.0], [9.0, 0.0], [9.0, 3.0],
                                 [5.0, 3.0], [5.0, 0.0]],
                },
            ],
            'order': ['area-1'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('duplicate object id' in issue
                            for issue in result.issues))

    def test_missing_geometry_is_rejected(self):
        mission = {
            'objects': [{
                'id': 'area-1',
                'type': 'work_area',
                'status': 'confirmed',
            }],
            'order': ['area-1'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('invalid geometry' in issue
                            for issue in result.issues))

    def test_order_must_be_a_complete_list_of_executable_objects(self):
        mission = {
            'objects': [{
                'id': 'area-1',
                'type': 'work_area',
                'status': 'confirmed',
                'geometry': [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
                             [0.0, 3.0], [0.0, 0.0]],
            }],
            'order': 'area-1',
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('mission order' in issue
                            for issue in result.issues))

    def test_null_order_is_rejected_instead_of_being_treated_as_empty(self):
        mission = {
            'objects': [{
                'id': 'area-1',
                'type': 'work_area',
                'status': 'confirmed',
                'geometry': [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
                             [0.0, 3.0], [0.0, 0.0]],
            }],
            'order': None,
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('mission order' in issue
                            for issue in result.issues))

    def test_order_entries_must_be_hashable_object_ids(self):
        mission = {
            'objects': [{
                'id': 'area-1',
                'type': 'work_area',
                'status': 'confirmed',
                'geometry': [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
                             [0.0, 3.0], [0.0, 0.0]],
            }],
            'order': [{}],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('mission order' in issue
                            for issue in result.issues))

    def test_overlapping_work_areas_are_rejected(self):
        mission = {
            'objects': [
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0],
                                 [0.0, 3.0], [0.0, 0.0]],
                },
                {
                    'id': 'area-2',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[3.0, 0.0], [7.0, 0.0], [7.0, 3.0],
                                 [3.0, 3.0], [3.0, 0.0]],
                },
            ],
            'order': ['area-1', 'area-2'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('overlap' in issue for issue in result.issues))

    def test_work_area_boundaries_may_touch_without_overlap(self):
        mission = {
            'objects': [
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0],
                                 [0.0, 2.0], [0.0, 0.0]],
                },
                {
                    'id': 'area-2',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[2.0, 0.0], [4.0, 0.0], [4.0, 2.0],
                                 [2.0, 2.0], [2.0, 0.0]],
                },
            ],
            'order': ['area-1', 'area-2'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertTrue(result.valid)

    def test_no_go_zone_is_subtracted_from_work_area_coverage(self):
        mission = {
            'objects': [
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0],
                                 [0.0, 4.0], [0.0, 0.0]],
                },
                {
                    'id': 'pond',
                    'type': 'no_go_zone',
                    'status': 'confirmed',
                    'geometry': [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0],
                                 [1.0, 2.0], [1.0, 1.0]],
                },
            ],
            'order': ['area-1'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.effective_geometry['area-1'].area, 15.0)

    def test_overlapping_no_go_zones_are_allowed_and_merged(self):
        mission = {
            'objects': [
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[0.0, 0.0], [5.0, 0.0], [5.0, 5.0],
                                 [0.0, 5.0], [0.0, 0.0]],
                },
                {
                    'id': 'pond-1',
                    'type': 'no_go_zone',
                    'status': 'confirmed',
                    'geometry': [[1.0, 1.0], [3.0, 1.0], [3.0, 3.0],
                                 [1.0, 3.0], [1.0, 1.0]],
                },
                {
                    'id': 'pond-2',
                    'type': 'no_go_zone',
                    'status': 'confirmed',
                    'geometry': [[2.0, 2.0], [4.0, 2.0], [4.0, 4.0],
                                 [2.0, 4.0], [2.0, 2.0]],
                },
            ],
            'order': ['area-1'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.effective_geometry['area-1'].area, 18.0)

    def test_corridor_narrower_than_robot_clearance_is_rejected(self):
        mission = {
            'objects': [
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0],
                                 [0.0, 2.0], [0.0, 0.0]],
                },
                {
                    'id': 'area-2',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[4.0, 0.0], [6.0, 0.0], [6.0, 2.0],
                                 [4.0, 2.0], [4.0, 0.0]],
                },
                {
                    'id': 'corridor-1',
                    'type': 'corridor',
                    'status': 'confirmed',
                    'geometry': [[2.0, 1.0], [4.0, 1.0]],
                    'width': 0.5,
                    'from_work_area_id': 'area-1',
                    'to_work_area_id': 'area-2',
                },
            ],
            'order': ['area-1', 'corridor-1', 'area-2'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('corridor is too narrow' in issue
                            for issue in result.issues))

    def test_corridor_must_reference_two_work_areas(self):
        mission = {
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
                    'geometry': [[2.0, 1.0], [4.0, 1.0]],
                    'width': 1.0,
                    'from_work_area_id': 'area-1',
                    'to_work_area_id': 'missing-area',
                },
            ],
            'order': ['area-1', 'corridor-1'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('corridor must connect two work areas' in issue
                            for issue in result.issues))

    def test_corridor_envelope_must_not_touch_no_go_zone(self):
        mission = {
            'objects': [
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0],
                                 [0.0, 2.0], [0.0, 0.0]],
                },
                {
                    'id': 'area-2',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[6.0, 0.0], [8.0, 0.0], [8.0, 2.0],
                                 [6.0, 2.0], [6.0, 0.0]],
                },
                {
                    'id': 'pond',
                    'type': 'no_go_zone',
                    'status': 'confirmed',
                    'geometry': [[3.0, 1.4], [5.0, 1.4], [5.0, 1.8],
                                 [3.0, 1.8], [3.0, 1.4]],
                },
                {
                    'id': 'corridor-1',
                    'type': 'corridor',
                    'status': 'confirmed',
                    'geometry': [[2.0, 1.0], [6.0, 1.0]],
                    'width': 1.0,
                    'from_work_area_id': 'area-1',
                    'to_work_area_id': 'area-2',
                },
            ],
            'order': ['area-1', 'corridor-1', 'area-2'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('corridor crosses no-go zone' in issue
                            for issue in result.issues))

    def test_corridor_endpoints_must_touch_their_work_areas(self):
        mission = {
            'objects': [
                {
                    'id': 'area-1',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0],
                                 [0.0, 2.0], [0.0, 0.0]],
                },
                {
                    'id': 'area-2',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[6.0, 0.0], [8.0, 0.0], [8.0, 2.0],
                                 [6.0, 2.0], [6.0, 0.0]],
                },
                {
                    'id': 'corridor-1',
                    'type': 'corridor',
                    'status': 'confirmed',
                    'geometry': [[2.5, 1.0], [5.5, 1.0]],
                    'width': 1.0,
                    'from_work_area_id': 'area-1',
                    'to_work_area_id': 'area-2',
                },
            ],
            'order': ['area-1', 'corridor-1', 'area-2'],
        }

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )

        self.assertFalse(result.valid)
        self.assertTrue(any('corridor endpoints must touch work areas' in issue
                            for issue in result.issues))

    def test_legacy_inner_rings_become_global_no_go_objects(self):
        legacy = {
            'areas': [{
                'name': 'front_lawn',
                'points': [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0],
                           [0.0, 4.0], [0.0, 0.0]],
                'inner_rings': [[
                    [1.0, 1.0], [2.0, 1.0], [2.0, 2.0],
                    [1.0, 2.0], [1.0, 1.0],
                ]],
                'cutting_angle': 0.3,
                'max_speed': 0.8,
            }],
        }

        mission = legacy_areas_to_mission(legacy)

        self.assertEqual(
            [(item['id'], item['type']) for item in mission['objects']],
            [('front_lawn', 'work_area'),
             ('front_lawn:no_go:1', 'no_go_zone')],
        )
        self.assertEqual(mission['order'], ['front_lawn'])
        self.assertEqual(mission['objects'][0]['cutting_angle'], 0.3)
        self.assertEqual(mission['objects'][0]['max_speed'], 0.8)

        result = validate_mission(
            mission,
            {'robot_length': 0.46, 'robot_width': 0.40, 'safety_margin': 0.2},
        )
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.effective_geometry['front_lawn'].area, 15.0)


if __name__ == '__main__':
    unittest.main()
