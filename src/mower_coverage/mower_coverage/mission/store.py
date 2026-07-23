"""Mission collection store for the simulation capture workflow."""

from copy import deepcopy

from .capture import CaptureSession


class MissionCaptureStore:
    """Keep one active capture and the confirmed/draft mission objects."""

    def __init__(self, geometry_profile, frame_id='map', allow_local_odom=False):
        self.geometry_profile = deepcopy(geometry_profile)
        self.frame_id = frame_id
        self.allow_local_odom = allow_local_odom
        self.session = None
        self.objects = []
        self.drafts = []
        self.order = []

    def start(self, object_type, object_id=None):
        if self.session is not None:
            raise RuntimeError('another capture is active')
        if object_type not in {'work_area', 'no_go_zone', 'corridor'}:
            raise ValueError('unknown object type')
        object_id = object_id or self._next_id(object_type)
        if not isinstance(object_id, str) or not object_id:
            raise ValueError('object id is required')
        if object_id in self._known_ids():
            raise ValueError(f'duplicate object id: {object_id}')

        self.session = CaptureSession(
            object_id,
            object_type,
            self.geometry_profile,
            frame_id=self.frame_id,
            allow_local_odom=self.allow_local_odom,
        )
        return self.session.start()

    def record_pose(self, sample):
        self._require_session()
        return self.session.record_pose(sample)

    def finish(self, current_health_state=None):
        self._require_session()
        return self.session.finish(current_health_state)

    def set_manual_geometry(self, geometry):
        self._require_session()
        return self.session.set_manual_geometry(geometry)

    def undo(self):
        self._require_session()
        return self.session.undo()

    def set_corridor_metadata(self, width, from_work_area_id,
                              to_work_area_id, bidirectional):
        self._require_session()
        return self.session.set_corridor_metadata(
            width,
            from_work_area_id,
            to_work_area_id,
            bidirectional,
        )

    def confirm(self, current_health_state=None):
        self._require_session()
        item = self.session.confirm(current_health_state)
        self.objects.append(deepcopy(item))
        if item['type'] in {'work_area', 'corridor'}:
            if item['type'] == 'corridor':
                from_id = item.get('from_work_area_id')
                to_id = item.get('to_work_area_id')
                try:
                    from_index = self.order.index(from_id)
                    to_index = self.order.index(to_id)
                except ValueError:
                    self.order.append(item['id'])
                else:
                    if abs(from_index - to_index) == 1:
                        self.order.insert(max(from_index, to_index), item['id'])
                    else:
                        self.order.append(item['id'])
            else:
                self.order.append(item['id'])
        self.session = None
        return deepcopy(item)

    def save_draft(self):
        self._require_session()
        item = self.session.save_draft()
        self._replace_draft(item)
        self.session = None
        return deepcopy(item)

    def cancel(self):
        self._require_session()
        result = self.session.cancel()
        self.session = None
        return result

    def delete_object(self, object_id):
        """Delete one persisted object without breaking mission references."""
        if self.session is not None:
            raise RuntimeError('cannot delete while capture is active')
        if not isinstance(object_id, str) or not object_id:
            raise ValueError('object id is required')

        matches = [
            item for item in self.objects + self.drafts
            if isinstance(item, dict) and item.get('id') == object_id
        ]
        if not matches:
            raise ValueError(f'unknown mission object id: {object_id}')

        item = matches[0]
        if item.get('type') == 'work_area':
            dependents = [
                corridor.get('id') for corridor in self.objects + self.drafts
                if isinstance(corridor, dict)
                and corridor.get('type') == 'corridor'
                and object_id in {
                    corridor.get('from_work_area_id'),
                    corridor.get('to_work_area_id'),
                }
            ]
            if dependents:
                names = ', '.join(str(value) for value in dependents)
                raise RuntimeError(
                    f'cannot delete work area {object_id}; '
                    f'delete dependent corridor(s) first: {names}')

        self.objects = [
            value for value in self.objects
            if not isinstance(value, dict) or value.get('id') != object_id
        ]
        self.drafts = [
            value for value in self.drafts
            if not isinstance(value, dict) or value.get('id') != object_id
        ]
        self.order = [value for value in self.order if value != object_id]
        return deepcopy(item)

    def clear(self):
        self.session = None
        self.objects = []
        self.drafts = []
        self.order = []

    def document(self):
        return {
            'version': 1,
            'frame_id': self.frame_id,
            'objects': deepcopy(self.objects),
            'drafts': deepcopy(self.drafts),
            'order': list(self.order),
        }

    def snapshot(self):
        return {
            'active': (
                None if self.session is None else self.session.snapshot()
            ),
            'objects': deepcopy(self.objects),
            'drafts': deepcopy(self.drafts),
            'order': list(self.order),
        }

    def load_document(self, document):
        if not isinstance(document, dict):
            raise ValueError('mission document must be an object')
        objects = document.get('objects', [])
        drafts = document.get('drafts', [])
        order = document.get('order', [])
        if not isinstance(objects, list) or not isinstance(drafts, list):
            raise ValueError('mission objects and drafts must be lists')
        if not isinstance(order, list) or any(
                not isinstance(item, str) for item in order):
            raise ValueError('mission order must be a list of strings')
        known_ids = [item.get('id') for item in objects + drafts
                     if isinstance(item, dict)]
        if any(not isinstance(item, str) or not item for item in known_ids):
            raise ValueError('mission object id is required')
        if len(known_ids) != len(set(known_ids)):
            raise ValueError('duplicate mission object id')
        self.objects = deepcopy(objects)
        self.drafts = deepcopy(drafts)
        self.order = list(order)
        self.session = None

    def _require_session(self):
        if self.session is None:
            raise RuntimeError('no active capture')

    def _known_ids(self):
        return {
            item['id'] for item in self.objects + self.drafts
            if isinstance(item, dict) and isinstance(item.get('id'), str)
        }

    def _next_id(self, object_type):
        prefix = {
            'work_area': 'work-area',
            'no_go_zone': 'no-go',
            'corridor': 'corridor',
        }[object_type]
        index = 1
        while f'{prefix}-{index}' in self._known_ids():
            index += 1
        return f'{prefix}-{index}'

    def _replace_draft(self, item):
        self.drafts = [
            draft for draft in self.drafts if draft.get('id') != item['id']
        ]
        self.drafts.append(deepcopy(item))
