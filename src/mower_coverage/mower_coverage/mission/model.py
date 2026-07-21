"""Pure mission geometry seams for captured mowing objects."""

from dataclasses import dataclass
from math import hypot, isfinite

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union


@dataclass(frozen=True)
class GeometryResult:
    """Result returned by the public geometry derivation seam."""

    status: str
    geometry: tuple
    issues: tuple = ()
    raw_trajectory: tuple = ()


@dataclass(frozen=True)
class ValidationResult:
    """Public mission validation result."""

    valid: bool
    issues: tuple = ()
    effective_geometry: object = ()


def derive_effective_geometry(raw_trajectory, object_type, profile):
    """Derive a validated geometry from one captured trajectory."""
    if object_type not in {'work_area', 'no_go_zone', 'corridor'}:
        return GeometryResult('invalid', (), ('unknown object type',))

    try:
        samples = tuple(raw_trajectory)
    except TypeError:
        return GeometryResult('invalid', (), ('trajectory is missing',))
    if any(
        isinstance(sample, dict) and sample.get('localization_ok') is False
        for sample in samples
    ):
        return GeometryResult(
            'draft', (), ('localization is unhealthy',), samples)

    try:
        points = tuple(
            (float(sample['x']), float(sample['y']))
            for sample in samples
        )
    except (KeyError, TypeError, ValueError):
        return GeometryResult(
            'invalid', (), ('trajectory points are invalid',), samples)

    if any(not isfinite(value) for point in points for value in point):
        return GeometryResult(
            'invalid', (), ('trajectory contains non-finite values',), samples)

    if object_type == 'corridor':
        if len(points) < 2:
            return GeometryResult(
                'invalid', (), ('corridor needs two points',), samples)
        return GeometryResult('ready', points, (), samples)

    if len(points) < 3:
        return GeometryResult(
            'invalid', (), ('closed geometry needs three points',), samples)

    try:
        closure_tolerance = float(profile['closure_tolerance'])
        simplify_tolerance = float(profile['simplify_tolerance'])
    except (KeyError, TypeError, ValueError):
        return GeometryResult(
            'invalid', (), ('geometry profile is incomplete',), samples)
    if (
        not isfinite(closure_tolerance)
        or not isfinite(simplify_tolerance)
        or closure_tolerance < 0.0
        or simplify_tolerance < 0.0
    ):
        return GeometryResult(
            'invalid', (), ('geometry profile is invalid',), samples)
    if hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1]) > closure_tolerance:
        return GeometryResult(
            'invalid', (), ('trajectory is not close to its start',), samples)

    # The robot keeps publishing odometry after the user releases the
    # direction control.  Normalize a near-closed endpoint and collapse those
    # stationary samples for effective geometry; raw evidence remains intact.
    closed_points = list(points)
    while (len(closed_points) > 1
           and hypot(closed_points[-1][0] - points[0][0],
                    closed_points[-1][1] - points[0][1]) <= closure_tolerance):
        closed_points.pop()
    closed_points.append(points[0])
    filtered_points = [closed_points[0]]
    for point in closed_points[1:]:
        if hypot(point[0] - filtered_points[-1][0],
                 point[1] - filtered_points[-1][1]) > simplify_tolerance:
            filtered_points.append(point)
    if filtered_points[-1] != filtered_points[0]:
        filtered_points.append(filtered_points[0])

    polygon = Polygon(filtered_points)
    polygon = polygon.simplify(simplify_tolerance, preserve_topology=True)
    if not polygon.is_valid or polygon.area <= 0.0:
        return GeometryResult(
            'invalid', (), ('trajectory does not form a valid polygon',), samples)

    geometry = tuple((float(x), float(y)) for x, y in polygon.exterior.coords)
    return GeometryResult('ready', geometry, (), samples)


def validate_mission(mission, profile):
    """Validate a confirmed mission without involving ROS or execution."""
    if not isinstance(mission, dict):
        return ValidationResult(False, ('mission must be an object',))

    objects = mission.get('objects')
    if not isinstance(objects, list) or not objects:
        return ValidationResult(False, ('mission needs objects',))

    try:
        robot_length = float(profile['robot_length'])
        robot_width = float(profile['robot_width'])
        safety_margin = float(profile['safety_margin'])
    except (KeyError, TypeError, ValueError):
        return ValidationResult(False, ('robot profile is incomplete',))
    if (
        not isfinite(robot_length)
        or not isfinite(robot_width)
        or not isfinite(safety_margin)
        or robot_length <= 0.0
        or robot_width <= 0.0
        or safety_margin < 0.0
    ):
        return ValidationResult(False, ('robot profile is invalid',))

    allowed_types = {'work_area', 'no_go_zone', 'corridor'}
    seen_ids = set()
    issues = []
    work_areas = []
    no_go_zones = []
    corridors = []
    for item in objects:
        if not isinstance(item, dict):
            issues.append('mission object must be an object')
            continue
        object_id = item.get('id')
        object_type = item.get('type')
        if not isinstance(object_id, str) or not object_id:
            issues.append('mission object id is required')
        elif object_id in seen_ids:
            issues.append(f'duplicate object id: {object_id}')
        else:
            seen_ids.add(object_id)
        if object_type not in allowed_types:
            issues.append(f'unknown object type: {object_type}')
            continue
        if item.get('status') != 'confirmed':
            issues.append(f'object is not confirmed: {object_id}')
            continue

        geometry = item.get('geometry')
        try:
            shape = (
                LineString(geometry)
                if object_type == 'corridor'
                else Polygon(geometry)
            )
        except (TypeError, ValueError):
            issues.append(f'invalid geometry: {object_id}')
            continue
        if not shape.is_valid or shape.is_empty:
            issues.append(f'invalid geometry: {object_id}')
        elif object_type != 'corridor' and shape.area <= 0.0:
            issues.append(f'empty area geometry: {object_id}')
        elif object_type == 'work_area':
            work_areas.append((object_id, shape))
        elif object_type == 'no_go_zone':
            no_go_zones.append(shape)
        elif object_type == 'corridor':
            corridors.append((object_id, shape, item))

    for index, (left_id, left) in enumerate(work_areas):
        for right_id, right in work_areas[index + 1:]:
            if left.intersection(right).area > 1e-9:
                issues.append(f'work areas overlap: {left_id}, {right_id}')

    effective_geometry = {}
    if no_go_zones:
        blocked = unary_union(no_go_zones)
        for object_id, area in work_areas:
            coverage = area.difference(blocked)
            if coverage.is_empty:
                issues.append(f'no usable coverage remains: {object_id}')
            else:
                effective_geometry[object_id] = coverage
        for object_id, centerline, item in corridors:
            try:
                width = float(item['width'])
            except (KeyError, TypeError, ValueError):
                continue
            envelope = centerline.buffer(width / 2.0, cap_style=2)
            if envelope.intersects(blocked):
                issues.append(f'corridor crosses no-go zone: {object_id}')
    else:
        effective_geometry = {
            object_id: area for object_id, area in work_areas
        }

    minimum_corridor_width = (
        max(robot_length, robot_width) + 2.0 * safety_margin
    )
    for object_id, _centerline, item in corridors:
        try:
            width = float(item['width'])
        except (KeyError, TypeError, ValueError):
            issues.append(f'corridor width is invalid: {object_id}')
            continue
        if not isfinite(width) or width < minimum_corridor_width:
            issues.append(
                f'corridor is too narrow: {object_id} '
                f'(minimum {minimum_corridor_width:.2f}m)')

    work_area_ids = {object_id for object_id, _area in work_areas}
    work_area_shapes = dict(work_areas)
    for object_id, _centerline, item in corridors:
        from_id = item.get('from_work_area_id')
        to_id = item.get('to_work_area_id')
        if (
            from_id == to_id
            or from_id not in work_area_ids
            or to_id not in work_area_ids
        ):
            issues.append(f'corridor must connect two work areas: {object_id}')
            continue
        start = Point(_centerline.coords[0])
        end = Point(_centerline.coords[-1])
        if not (
            work_area_shapes[from_id].covers(start)
            and work_area_shapes[to_id].covers(end)
        ):
            issues.append(
                f'corridor endpoints must touch work areas: {object_id}')

    executable_ids = {
        item.get('id') for item in objects
        if isinstance(item, dict)
        and item.get('type') in {'work_area', 'corridor'}
        and isinstance(item.get('id'), str)
    }
    order = mission.get('order', [])
    if not isinstance(order, list):
        issues.append('mission order must be a list')
    elif not order and executable_ids:
        issues.append(
            'mission order must contain every executable object exactly once')
    elif (
        any(not isinstance(item, str) for item in order)
        or len(order) != len(set(order))
        or set(order) != executable_ids
    ):
        issues.append(
            'mission order must contain every executable object exactly once')

    return ValidationResult(not issues, tuple(issues), effective_geometry)


def legacy_areas_to_mission(legacy_yaml):
    """Convert the historical ``areas`` YAML shape to the mission model."""
    if not isinstance(legacy_yaml, dict) or not isinstance(legacy_yaml.get('areas'), list):
        raise ValueError('legacy YAML must contain an areas list')

    objects = []
    order = []
    seen_ids = set()

    def add_object(item):
        if item['id'] in seen_ids:
            raise ValueError(f'duplicate legacy object id: {item["id"]}')
        seen_ids.add(item['id'])
        objects.append(item)

    for entry in legacy_yaml['areas']:
        if not isinstance(entry, dict):
            raise ValueError('legacy area must be an object')
        name = entry.get('name')
        points = entry.get('points', [])
        if not isinstance(name, str) or not name or len(points) < 3:
            raise ValueError('legacy area needs a name and three points')

        max_speed = float(entry.get('max_speed', 1.0))
        if max_speed <= 0.0:
            add_object({
                'id': name,
                'type': 'no_go_zone',
                'status': 'confirmed',
                'geometry': points,
                'source': 'legacy',
                'color': entry.get('color', [1.0, 0.0, 0.0]),
            })
        else:
            add_object({
                'id': name,
                'type': 'work_area',
                'status': 'confirmed',
                'geometry': points,
                'source': 'legacy',
                'cutting_angle': float(entry.get('cutting_angle', 0.0)),
                'max_speed': max_speed,
                'color': entry.get('color', [0.0, 1.0, 0.0]),
            })
            order.append(name)

        for ring_index, ring in enumerate(entry.get('inner_rings', []), start=1):
            add_object({
                'id': f'{name}:no_go:{ring_index}',
                'type': 'no_go_zone',
                'status': 'confirmed',
                'geometry': ring,
                'source': 'legacy',
                'color': [1.0, 0.0, 0.0],
            })

    return {'objects': objects, 'order': order}
