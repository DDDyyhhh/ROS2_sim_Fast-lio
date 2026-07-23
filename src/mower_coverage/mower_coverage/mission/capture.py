"""State machine for one remote-captured mission object."""

from copy import deepcopy
from math import hypot, isfinite

from shapely.geometry import Polygon
from shapely.validation import explain_validity

from .model import GeometryResult, derive_effective_geometry


_OBJECT_TYPES = {'work_area', 'no_go_zone', 'corridor'}
_CAPTURE_EDITABLE_STATES = {'capturing', 'draft', 'ready'}
_HEALTH_STATES = {'GREEN', 'YELLOW', 'RED'}


class CaptureSession:
    """Capture, review and confirm exactly one mission object."""

    def __init__(self, object_id, object_type, geometry_profile,
                 frame_id='map', allow_local_odom=False):
        if not isinstance(object_id, str) or not object_id:
            raise ValueError('object id is required')
        if object_type not in _OBJECT_TYPES:
            raise ValueError(f'unknown object type: {object_type}')
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError('frame_id is required')
        if frame_id != 'map' and not (
                allow_local_odom and frame_id == 'odom'):
            raise ValueError(
                'local odom capture requires explicit simulation fallback')

        self.object_id = object_id
        self.object_type = object_type
        self.geometry_profile = deepcopy(geometry_profile)
        self.frame_id = frame_id
        self.allow_local_odom = allow_local_odom
        self.state = 'idle'
        self._raw_trajectory = []
        self._result = None
        self._corridor_metadata = {}

    def start(self):
        """Start collecting samples for this one object."""
        if self.state != 'idle':
            raise RuntimeError('capture session is not idle')
        self.state = 'capturing'
        return self.snapshot()

    def record_pose(self, sample):
        """Append one validated pose sample to the active capture."""
        if self.state not in {'capturing', 'draft'}:
            raise RuntimeError('capture session is not capturing')

        was_draft = self.state == 'draft'
        normalized = self._normalize_sample(sample)
        self._raw_trajectory.append(normalized)
        if not was_draft:
            self._result = None
            self.state = 'capturing'
        return self.snapshot()

    def finish(self, current_health_state=None):
        """Derive geometry; unhealthy or invalid captures remain drafts."""
        if self.state not in {'capturing', 'draft'}:
            raise RuntimeError('capture session is not capturing')

        self._validate_health_state(current_health_state)
        if (current_health_state is not None
                and current_health_state != 'GREEN'):
            self._mark_unhealthy(current_health_state, 'finish')
            return self.snapshot()

        self._result = derive_effective_geometry(
            self._raw_trajectory,
            self.object_type,
            self.geometry_profile,
        )
        self.state = (
            'ready' if self._result.status == 'ready' else 'draft')
        return self.snapshot()

    def set_manual_geometry(self, geometry):
        """Replace effective closed geometry without changing raw evidence."""
        if self.state not in {'draft', 'ready'}:
            raise RuntimeError('manual geometry requires a finished capture')
        if self.object_type == 'corridor':
            raise RuntimeError('manual geometry is only for closed objects')
        if any(
            isinstance(sample, dict)
            and sample.get('localization_ok') is False
            for sample in self._raw_trajectory
        ):
            raise ValueError(
                'manual geometry cannot override unhealthy localization')

        try:
            points = tuple(
                (float(point[0]), float(point[1]))
                for point in geometry
            )
        except (IndexError, TypeError, ValueError):
            raise ValueError('manual geometry points are invalid')
        if len(points) < 3 or any(
            not isfinite(value) for point in points for value in point
        ):
            raise ValueError('manual geometry needs three finite points')
        if points[0] != points[-1]:
            points += (points[0],)

        polygon = Polygon(points)
        if not polygon.is_valid or polygon.area <= 0.0:
            validity = explain_validity(polygon)
            issue = validity if validity and validity != 'Valid Geometry' else 'empty area'
            raise ValueError(f'manual geometry is invalid: {issue}')

        effective_geometry = tuple(
            (float(x), float(y)) for x, y in polygon.exterior.coords
        )
        self._result = GeometryResult(
            'ready',
            effective_geometry,
            (),
            tuple(self._raw_trajectory),
        )
        self.state = 'ready'
        return self.snapshot()

    def undo(self):
        """Remove the most recent sample and reopen the capture for editing."""
        if self.state not in _CAPTURE_EDITABLE_STATES:
            raise RuntimeError('capture session cannot be edited')
        if not self._raw_trajectory:
            raise RuntimeError('capture has no samples to undo')

        removed = self._raw_trajectory.pop()
        self._result = None
        self.state = 'capturing'
        return deepcopy(removed)

    def cancel(self):
        """Cancel the session without deleting its in-memory draft."""
        if self.state == 'confirmed':
            raise RuntimeError('confirmed capture cannot be cancelled')
        self.state = 'cancelled'
        return self.snapshot()

    def save_draft(self):
        """Return a persistable draft; this never promotes the object."""
        if self.state not in _CAPTURE_EDITABLE_STATES:
            raise RuntimeError('capture session has no editable draft')
        return self._mission_object('draft')

    def confirm(self, current_health_state=None):
        """Promote a geometrically ready capture to a confirmed object."""
        if self.state != 'ready':
            raise RuntimeError('capture can be confirmed only when ready')
        self._validate_health_state(current_health_state)
        if (current_health_state is not None
                and current_health_state != 'GREEN'):
            self._mark_unhealthy(current_health_state, 'confirm')
            raise RuntimeError(
                f'capture cannot be confirmed while localization is '
                f'{current_health_state}')
        if self.object_type == 'corridor' and not self._corridor_metadata:
            raise RuntimeError('corridor metadata is required before confirmation')
        self.state = 'confirmed'
        return self._mission_object('confirmed')

    def set_corridor_metadata(self, width, from_work_area_id,
                              to_work_area_id, bidirectional):
        """Set the fields the legacy geometry cannot infer from a centerline."""
        if self.object_type != 'corridor':
            raise RuntimeError('only corridors have route metadata')
        if self.state not in _CAPTURE_EDITABLE_STATES:
            raise RuntimeError('capture session cannot be edited')
        if (isinstance(width, bool)
                or not isinstance(width, (int, float))
                or not isfinite(width)
                or width <= 0.0):
            raise ValueError('corridor width must be finite and positive')
        if (not isinstance(from_work_area_id, str)
                or not from_work_area_id
                or not isinstance(to_work_area_id, str)
                or not to_work_area_id
                or from_work_area_id == to_work_area_id):
            raise ValueError('corridor endpoints must be two work areas')
        if not isinstance(bidirectional, bool):
            raise ValueError('corridor bidirectional must be boolean')

        self._corridor_metadata = {
            'width': float(width),
            'from_work_area_id': from_work_area_id,
            'to_work_area_id': to_work_area_id,
            'bidirectional': bidirectional,
        }
        return self.snapshot()

    def snapshot(self):
        """Return the current UI/persistence-neutral session snapshot."""
        geometry = () if self._result is None else self._result.geometry
        issues = () if self._result is None else self._result.issues
        return {
            'state': self.state,
            'id': self.object_id,
            'type': self.object_type,
            'frame_id': self.frame_id,
            'raw_trajectory': deepcopy(self._raw_trajectory),
            'geometry': [list(point) for point in geometry],
            'issues': list(issues),
            'corridor_metadata': deepcopy(self._corridor_metadata),
            **self._closure_feedback(),
        }

    def _mission_object(self, status):
        snapshot = self.snapshot()
        result = {
            'id': self.object_id,
            'type': self.object_type,
            'status': status,
            'frame_id': self.frame_id,
            'geometry': snapshot['geometry'],
            'raw_trajectory': snapshot['raw_trajectory'],
            'source': 'remote_capture',
            'issues': snapshot['issues'],
            'start_point': snapshot['start_point'],
            'end_point': snapshot['end_point'],
            'closure_distance': snapshot['closure_distance'],
            'closure_tolerance': snapshot['closure_tolerance'],
        }
        if self.object_type == 'corridor':
            result.update(deepcopy(self._corridor_metadata))
        return result

    def _normalize_sample(self, sample):
        if not isinstance(sample, dict):
            raise ValueError('pose sample must be an object')

        for axis in ('x', 'y'):
            value = sample.get(axis)
            if (isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not isfinite(value)):
                raise ValueError(f'pose {axis} must be finite')

        timestamp = sample.get('timestamp')
        if (isinstance(timestamp, bool)
                or not isinstance(timestamp, (int, float))
                or not isfinite(timestamp)):
            raise ValueError('pose timestamp must be finite')

        if sample.get('frame_id') != self.frame_id:
            raise ValueError(
                f'pose frame_id must be {self.frame_id}')

        normalized = deepcopy(sample)
        health_state = sample.get('localization_state')
        if health_state is not None:
            if health_state not in _HEALTH_STATES:
                raise ValueError('unknown localization state')
            expected_ok = health_state == 'GREEN'
            if ('localization_ok' in sample
                    and sample['localization_ok'] is not expected_ok):
                raise ValueError('localization state conflicts with localization_ok')
            normalized['localization_ok'] = expected_ok
        elif not isinstance(sample.get('localization_ok'), bool):
            raise ValueError('localization_ok must be boolean')

        return normalized

    def _validate_health_state(self, health_state):
        if health_state is not None and health_state not in _HEALTH_STATES:
            raise ValueError('unknown localization state')

    def _mark_unhealthy(self, health_state, phase):
        self._result = GeometryResult(
            'draft',
            (),
            (f'localization is {health_state} at {phase}',),
            tuple(self._raw_trajectory),
        )
        self.state = 'draft'

    def _closure_feedback(self):
        tolerance = self.geometry_profile.get('closure_tolerance')
        try:
            tolerance = float(tolerance)
        except (TypeError, ValueError):
            tolerance = None
        if tolerance is not None and (
                not isfinite(tolerance) or tolerance < 0.0):
            tolerance = None

        if not self._raw_trajectory:
            return {
                'start_point': None,
                'end_point': None,
                'closure_distance': None,
                'closure_tolerance': tolerance,
            }

        start = self._raw_trajectory[0]
        end = self._raw_trajectory[-1]
        start_point = {'x': float(start['x']), 'y': float(start['y'])}
        end_point = {'x': float(end['x']), 'y': float(end['y'])}
        return {
            'start_point': start_point,
            'end_point': end_point,
            'closure_distance': hypot(
                end_point['x'] - start_point['x'],
                end_point['y'] - start_point['y'],
            ),
            'closure_tolerance': tolerance,
        }
