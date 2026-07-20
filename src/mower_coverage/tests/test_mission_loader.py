#!/usr/bin/env python3
"""Behavior tests for mission YAML compatibility loading."""

import os
import sys
import tempfile
from types import SimpleNamespace
import unittest

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_coverage.mission.loader import (
    load_mission_data,
    load_mission_file,
    mission_to_legacy_areas,
)
from mower_coverage.mission.multi_area_definer import MultiAreaDefiner


class _Logger:
    def error(self, _message):
        pass

    def info(self, _message):
        pass


class MissionLoaderTests(unittest.TestCase):
    def test_legacy_areas_are_normalized_to_the_mission_model(self):
        legacy = {
            'areas': [{
                'name': 'front_lawn',
                'points': [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0],
                           [0.0, 4.0], [0.0, 0.0]],
                'inner_rings': [[
                    [1.0, 1.0], [2.0, 1.0], [2.0, 2.0],
                    [1.0, 2.0], [1.0, 1.0],
                ]],
            }],
        }

        mission = load_mission_data(legacy)

        self.assertEqual(mission['order'], ['front_lawn'])
        self.assertEqual(
            [(item['id'], item['type']) for item in mission['objects']],
            [('front_lawn', 'work_area'),
             ('front_lawn:no_go:1', 'no_go_zone')],
        )

    def test_mission_documents_are_loaded_without_mutating_the_input(self):
        source = {
            'objects': [{
                'id': 'area-1',
                'type': 'work_area',
                'status': 'confirmed',
                'geometry': [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0],
                             [0.0, 2.0], [0.0, 0.0]],
            }],
            'order': ['area-1'],
            'metadata': {'source': 'capture'},
        }

        loaded = load_mission_data(source)
        loaded['objects'][0]['geometry'][0][0] = 99.0

        self.assertEqual(source['objects'][0]['geometry'][0][0], 0.0)
        self.assertEqual(loaded['metadata'], {'source': 'capture'})

    def test_mission_file_is_read_as_utf8_yaml(self):
        mission = {
            'objects': [{
                'id': '草坪一',
                'type': 'work_area',
                'status': 'confirmed',
                'geometry': [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0],
                             [0.0, 2.0], [0.0, 0.0]],
            }],
            'order': ['草坪一'],
        }

        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml',
                                         encoding='utf-8') as stream:
            stream.write('objects:\n')
            stream.write('  - id: 草坪一\n')
            stream.write('    type: work_area\n')
            stream.write('    status: confirmed\n')
            stream.write('    geometry: [[0, 0], [2, 0], [2, 2], [0, 2], [0, 0]]\n')
            stream.write('order: [草坪一]\n')
            stream.flush()

            loaded = load_mission_file(stream.name)

        self.assertEqual(loaded, mission)

    def test_unknown_yaml_shape_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'objects or areas'):
            load_mission_data({'version': 1})

    def test_legacy_adapter_keeps_global_no_go_constraints_for_old_consumers(self):
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
                    'id': 'area-2',
                    'type': 'work_area',
                    'status': 'confirmed',
                    'geometry': [[5.0, 0.0], [9.0, 0.0], [9.0, 4.0],
                                 [5.0, 4.0], [5.0, 0.0]],
                },
                {
                    'id': 'pond',
                    'type': 'no_go_zone',
                    'status': 'confirmed',
                    'geometry': [[3.0, 1.0], [6.0, 1.0], [6.0, 2.0],
                                 [3.0, 2.0], [3.0, 1.0]],
                },
            ],
            'order': ['area-1', 'area-2'],
        }

        legacy = mission_to_legacy_areas(mission)

        self.assertEqual(
            [entry['name'] for entry in legacy['areas']],
            ['area-1', 'area-2'],
        )
        self.assertEqual(
            legacy['areas'][0]['inner_rings'],
            [mission['objects'][2]['geometry']],
        )
        self.assertEqual(
            legacy['areas'][1]['inner_rings'],
            [mission['objects'][2]['geometry']],
        )

    def test_legacy_adapter_rejects_corridors_instead_of_dropping_them(self):
        mission = {
            'objects': [{
                'id': 'corridor-1',
                'type': 'corridor',
                'status': 'confirmed',
                'geometry': [[0.0, 0.0], [4.0, 0.0]],
            }],
            'order': ['corridor-1'],
        }

        with self.assertRaisesRegex(ValueError, 'corridor'):
            mission_to_legacy_areas(mission)

    def test_multi_area_loader_accepts_new_mission_documents(self):
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
        node = object.__new__(MultiAreaDefiner)
        node.area_file = None
        node.areas = {}
        node.next_color_idx = 0
        node.get_logger = lambda: _Logger()
        node.publish_visualization = lambda: None

        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml',
                                         encoding='utf-8') as stream:
            yaml.safe_dump(mission, stream, allow_unicode=True)
            stream.flush()
            node.area_file = stream.name
            response = SimpleNamespace(success=False, message='')
            node.load_cb(None, response)

        self.assertTrue(response.success)
        self.assertEqual(list(node.areas), ['area-1'])
        self.assertEqual(len(node.areas['area-1']['inner_rings']), 1)


if __name__ == '__main__':
    unittest.main()
