"""YAML loading seams for the captured mission model."""

from copy import deepcopy

import yaml

from .model import legacy_areas_to_mission


def load_mission_data(data):
    """Normalize a new mission document or a legacy areas document."""
    if not isinstance(data, dict):
        raise ValueError('mission YAML must be an object')

    has_objects = 'objects' in data
    has_areas = 'areas' in data
    if has_objects and has_areas:
        raise ValueError('mission YAML cannot contain both objects and areas')

    if has_objects:
        if not isinstance(data['objects'], list):
            raise ValueError('mission objects must be a list')
        return deepcopy(data)

    if has_areas:
        return legacy_areas_to_mission(data)

    raise ValueError('mission YAML must contain objects or areas')


def load_mission_file(path):
    """Read and normalize one UTF-8 YAML mission file."""
    try:
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f'invalid mission YAML: {exc}') from exc

    return load_mission_data(data)


def mission_to_legacy_areas(mission):
    """Adapt work/no-go objects for consumers that only understand areas.

    The legacy shape has no corridor representation.  Rejecting a mission
    containing one is safer than silently dropping its transit constraint.
    Global no-go objects are attached to every work area because that is the
    only legacy representation that makes them constrain all coverage areas.
    """
    if not isinstance(mission, dict):
        raise ValueError('mission must be an object')
    objects = mission.get('objects')
    if not isinstance(objects, list):
        raise ValueError('mission objects must be a list')

    work_areas = []
    no_go_zones = []
    seen_ids = set()
    for item in objects:
        if not isinstance(item, dict):
            raise ValueError('mission object must be an object')
        object_id = item.get('id')
        object_type = item.get('type')
        if not isinstance(object_id, str) or not object_id:
            raise ValueError('mission object id is required')
        if object_id in seen_ids:
            raise ValueError(f'duplicate object id: {object_id}')
        seen_ids.add(object_id)
        if item.get('status') != 'confirmed':
            raise ValueError(f'legacy consumer cannot load draft: {object_id}')
        if object_type == 'corridor':
            raise ValueError(
                f'legacy consumer cannot represent corridor: {object_id}')
        if object_type == 'work_area':
            work_areas.append(item)
        elif object_type == 'no_go_zone':
            no_go_zones.append(item)
        else:
            raise ValueError(f'unknown object type: {object_type}')

    order = mission.get('order', [])
    if not isinstance(order, list):
        raise ValueError('mission order must be a list')
    work_by_id = {item['id']: item for item in work_areas}
    ordered_ids = []
    for object_id in order:
        if object_id not in work_by_id:
            raise ValueError(
                f'legacy order contains non-work object: {object_id}')
        ordered_ids.append(object_id)
    if (
        len(ordered_ids) != len(set(ordered_ids))
        or set(ordered_ids) != set(work_by_id)
    ):
        raise ValueError(
            'legacy order must contain every work area exactly once')

    legacy_areas = []
    for object_id in ordered_ids:
        item = work_by_id[object_id]
        legacy_areas.append({
            'name': object_id,
            'points': deepcopy(item.get('geometry', [])),
            'inner_rings': [
                deepcopy(no_go['geometry']) for no_go in no_go_zones
            ],
            'color': deepcopy(item.get('color', [0.0, 1.0, 0.0])),
            'cutting_angle': item.get('cutting_angle', 0.0),
            'max_speed': item.get('max_speed', 1.0),
        })

    if not work_areas:
        legacy_areas.extend({
            'name': item['id'],
            'points': deepcopy(item.get('geometry', [])),
            'inner_rings': [],
            'color': [1.0, 0.0, 0.0],
            'cutting_angle': 0.0,
            'max_speed': 0.0,
        } for item in no_go_zones)

    return {'areas': legacy_areas}
