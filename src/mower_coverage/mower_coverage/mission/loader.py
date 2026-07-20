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
