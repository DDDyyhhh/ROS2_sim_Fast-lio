#!/usr/bin/env python3
"""
Convert hill LiDAR point clouds to scans without an absolute-height cut.

The generic ``pointcloud_to_laserscan`` height filter assumes a horizontal
ground plane.  That assumption is unsafe on the hill world: a valid ground
return can be above 1 m in the LiDAR frame, while a real obstacle can be much
lower.  This node estimates a local ground surface from the scan itself and
keeps points that are not explained by that surface.
"""

import math

import numpy as np


def _cell_levels(points: np.ndarray, cell_size: float):
    """Return cell centres, robust low ground levels, and cell coordinates."""
    cell_coords = np.floor(points[:, :2] / cell_size).astype(np.int32)
    unique_cells, inverse = np.unique(
        cell_coords, axis=0, return_inverse=True)
    # Sort by cell and then by Z in compiled NumPy code.  This lets us pick a
    # lower-quartile element per cell without a Python quantile call for every
    # occupied cell (the cloud contains tens of thousands of points/frame).
    order = np.lexsort((points[:, 2], inverse))
    sorted_inverse = inverse[order]
    sorted_z = points[order, 2]
    starts = np.r_[0, np.flatnonzero(np.diff(sorted_inverse)) + 1]
    ends = np.r_[starts[1:], len(sorted_z)]

    counts = (ends - starts).astype(np.int32)
    quartile_offsets = ((counts - 1) * 0.25).astype(np.int32)
    levels = sorted_z[starts + quartile_offsets]
    spreads = sorted_z[ends - 1] - sorted_z[starts]

    centres = (unique_cells.astype(np.float64) + 0.5) * cell_size
    representatives = points[order[starts + quartile_offsets], :2]
    return centres, levels, counts, unique_cells, representatives, spreads


def _fit_ground_plane(cell_centres: np.ndarray, levels: np.ndarray):
    """Fit a deterministic, trimmed plane ``z = ax + by + c``."""
    if len(cell_centres) < 3:
        return None

    inliers = np.ones(len(levels), dtype=bool)
    coefficients = None
    for _ in range(4):
        design = np.c_[cell_centres[inliers], np.ones(inliers.sum())]
        try:
            coefficients = np.linalg.lstsq(
                design, levels[inliers], rcond=None)[0]
        except np.linalg.LinAlgError:
            return None

        residual = levels - (
            coefficients[0] * cell_centres[:, 0]
            + coefficients[1] * cell_centres[:, 1]
            + coefficients[2]
        )
        median = np.median(residual)
        deviation = np.abs(residual - median)
        # Keep enough points for a rough terrain while trimming isolated
        # elevated obstacle cells from the global model.
        limit = max(0.08, 3.0 * np.median(deviation))
        next_inliers = deviation <= limit
        if next_inliers.sum() < 3 or np.array_equal(next_inliers, inliers):
            break
        inliers = next_inliers

    return coefficients


def _cell_key_layout(cell_coords: np.ndarray, radius_cells: int):
    """Return a collision-free integer layout for neighbouring cell lookup."""
    origin = cell_coords.min(axis=0)
    span_y = int(cell_coords[:, 1].max() - origin[1] + 1)
    stride = span_y + 2 * radius_cells + 1
    return origin, stride


def _cell_keys(
    cell_coords: np.ndarray,
    origin: np.ndarray,
    stride: int,
    radius_cells: int,
):
    """Encode integer cell coordinates without allocating a dense grid."""
    return (
        (cell_coords[:, 0] - origin[0] + radius_cells) * stride
        + cell_coords[:, 1] - origin[1] + radius_cells
    )


def _expand_ground_support(
    cell_coords: np.ndarray,
    seed_mask: np.ndarray,
    origin: np.ndarray,
    stride: int,
    radius_cells: int,
):
    """Allow sparse returns near a supported cell to use the ground model."""
    if not seed_mask.any():
        return np.zeros(len(cell_coords), dtype=bool)

    cell_keys = _cell_keys(cell_coords, origin, stride, radius_cells)
    seed_keys = cell_keys[seed_mask]
    supported = np.zeros(len(cell_coords), dtype=bool)
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            supported |= np.isin(
                cell_keys,
                seed_keys + dx * stride + dy,
            )
    return supported


def extract_obstacle_points(
    points,
    *,
    min_range: float = 0.35,
    max_range: float = 30.0,
    ground_fit_range: float = 8.0,
    ground_cell_size: float = 0.25,
    ground_clearance: float = 0.03,
    min_ground_points: int = 3,
    min_ground_fit_points: int = 64,
    min_ground_cells: int = 32,
    ground_support_radius: float = 1.0,
    min_ground_cell_span: float = 0.02,
):
    """
    Return points that are not explained by a locally supported ground.

    This function deliberately fails safe.  Invalid points, points without a
    usable ground model, and points outside the fitting range are retained
    until the range limits are applied by the scan publisher.  Only points
    above the robust local ground-plane estimate are classified as obstacles.
    """
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("points must be an N×3 array")
    xyz = xyz[:, :3]

    ranges = np.hypot(xyz[:, 0], xyz[:, 1])
    valid = np.isfinite(xyz).all(axis=1)
    in_range = valid & (ranges >= min_range) & (ranges <= max_range)
    fit_mask = in_range & (ranges <= ground_fit_range)
    if fit_mask.sum() < min_ground_fit_points:
        return xyz[in_range]

    fit_points = xyz[fit_mask]
    (
        _, levels, counts, unique_cells, ground_xy, cell_spans
    ) = _cell_levels(fit_points, ground_cell_size)
    supported = counts >= min_ground_points
    supported &= cell_spans >= min_ground_cell_span
    if (
        len(unique_cells) < min_ground_cells
        or supported.sum() < max(3, min_ground_cells // 2)
    ):
        return xyz[in_range]

    supported_span = np.ptp(unique_cells[supported], axis=0)
    if supported_span.min() * ground_cell_size < ground_support_radius:
        return xyz[in_range]

    plane = _fit_ground_plane(
        ground_xy[supported], levels[supported])
    if plane is None:
        return xyz[in_range]

    finite_cells = np.floor(
        xyz[valid, :2] / ground_cell_size).astype(np.int32)
    radius_cells = max(
        0, int(math.ceil(ground_support_radius / ground_cell_size)))
    origin, stride = _cell_key_layout(finite_cells, radius_cells)
    unique_keys = _cell_keys(
        unique_cells, origin, stride, radius_cells)
    model_supported = _expand_ground_support(
        unique_cells,
        supported,
        origin,
        stride,
        radius_cells,
    )

    predicted = np.full(len(xyz), np.nan, dtype=np.float64)
    predicted[valid] = (
        plane[0] * xyz[valid, 0]
        + plane[1] * xyz[valid, 1]
        + plane[2]
    )
    point_keys = np.zeros(len(xyz), dtype=np.int64)
    point_keys[valid] = _cell_keys(
        finite_cells, origin, stride, radius_cells)
    locally_supported = np.zeros(len(xyz), dtype=bool)
    locally_supported[valid] = np.isin(
        point_keys[valid], unique_keys[model_supported])
    near_ground = np.abs(xyz[:, 2] - predicted) <= ground_clearance
    # Only points in a locally supported cell and inside the narrow ground
    # band may be filtered.  Unsupported/ambiguous points remain obstacles.
    obstacle = (~locally_supported) | (~near_ground)
    return xyz[in_range & obstacle]


def build_scan_ranges(
    obstacles,
    *,
    count: int,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    emergency: bool = False,
):
    """Project obstacle points to ranges, or request a conservative stop."""
    ranges = np.full(count, np.inf, dtype=np.float32)
    if emergency:
        # MultiAreaExecutor treats range_min as a valid hit.  Filling the
        # scan makes a conversion failure stop the safe executor immediately.
        ranges.fill(range_min)
        return ranges

    obstacle_points = np.asarray(obstacles, dtype=np.float64).reshape(-1, 3)
    if len(obstacle_points) == 0:
        return ranges

    distances = np.hypot(obstacle_points[:, 0], obstacle_points[:, 1])
    angles = np.arctan2(obstacle_points[:, 1], obstacle_points[:, 0])
    indices = np.floor(
        (angles - angle_min) / angle_increment
    ).astype(np.int32)
    usable = (
        (indices >= 0) & (indices < count)
        & (distances >= range_min)
        & (distances <= range_max)
    )
    np.minimum.at(
        ranges, indices[usable], distances[usable].astype(np.float32))
    return ranges


class HillGroundObstacleScan:
    """ROS 2 adapter around :func:`extract_obstacle_points`."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2
        from sensor_msgs_py import point_cloud2

        class _Node(Node):
            def __init__(self):
                super().__init__("hill_ground_obstacle_scan")
                self._LaserScan = LaserScan
                self._point_cloud2 = point_cloud2
                self.declare_parameter("input_topic", "/velodyne_points")
                self.declare_parameter("output_topic", "/scan")
                self.declare_parameter("angle_min", -math.pi)
                self.declare_parameter("angle_max", math.pi)
                self.declare_parameter("angle_increment", 0.0087)
                self.declare_parameter("scan_time", 0.1)
                self.declare_parameter("range_min", 0.2)
                self.declare_parameter("range_max", 30.0)
                self.declare_parameter("ground_fit_range", 8.0)
                self.declare_parameter("ground_cell_size", 0.25)
                self.declare_parameter("ground_clearance", 0.03)
                self.declare_parameter("min_ground_points", 3)
                self.declare_parameter("min_ground_fit_points", 64)
                self.declare_parameter("min_ground_cells", 32)
                self.declare_parameter("ground_support_radius", 1.0)
                self.declare_parameter("min_ground_cell_span", 0.02)
                self.declare_parameter("self_filter_x", 0.25)
                self.declare_parameter("self_filter_y", 0.20)

                self._angle_min = float(self.get_parameter("angle_min").value)
                self._angle_max = float(self.get_parameter("angle_max").value)
                self._angle_increment = float(
                    self.get_parameter("angle_increment").value)
                self._scan_time = float(self.get_parameter("scan_time").value)
                self._range_min = float(self.get_parameter("range_min").value)
                self._range_max = float(self.get_parameter("range_max").value)
                self._ground_fit_range = float(
                    self.get_parameter("ground_fit_range").value)
                self._ground_cell_size = float(
                    self.get_parameter("ground_cell_size").value)
                self._ground_clearance = float(
                    self.get_parameter("ground_clearance").value)
                self._min_ground_points = int(
                    self.get_parameter("min_ground_points").value)
                self._min_ground_fit_points = int(
                    self.get_parameter("min_ground_fit_points").value)
                self._min_ground_cells = int(
                    self.get_parameter("min_ground_cells").value)
                self._ground_support_radius = float(
                    self.get_parameter("ground_support_radius").value)
                self._min_ground_cell_span = float(
                    self.get_parameter("min_ground_cell_span").value)
                self._self_filter_x = float(
                    self.get_parameter("self_filter_x").value)
                self._self_filter_y = float(
                    self.get_parameter("self_filter_y").value)

                input_topic = str(self.get_parameter("input_topic").value)
                output_topic = str(self.get_parameter("output_topic").value)
                self._scan_pub = self.create_publisher(
                    LaserScan, output_topic, qos_profile_sensor_data)
                self._cloud_sub = self.create_subscription(
                    PointCloud2, input_topic, self._cloud_callback,
                    qos_profile_sensor_data)
                self.get_logger().info(
                    "坡面安全点云过滤已启动：保留低矮障碍，"
                    "仅过滤局部地面点")

            def _cloud_callback(self, msg):
                try:
                    # Humble's read_points_numpy requires every field in the
                    # message to share one datatype.  The simulated cloud
                    # also contains float intensity and integer ring fields,
                    # so read only the selected XYZ fields through the
                    # datatype-agnostic iterator.
                    structured_points = self._point_cloud2.read_points(
                        msg, field_names=["x", "y", "z"], skip_nans=True)
                    points = np.column_stack((
                        structured_points["x"],
                        structured_points["y"],
                        structured_points["z"],
                    )).astype(np.float64, copy=False)
                    if not np.isfinite(points).all(axis=1).any():
                        raise ValueError("点云没有有效 XYZ 点")
                    self_filter = (
                        (np.abs(points[:, 0]) < self._self_filter_x)
                        & (np.abs(points[:, 1]) < self._self_filter_y)
                    )
                    points = points[~self_filter]
                    obstacles = extract_obstacle_points(
                        points,
                        min_range=self._range_min,
                        max_range=self._range_max,
                        ground_fit_range=self._ground_fit_range,
                        ground_cell_size=self._ground_cell_size,
                        ground_clearance=self._ground_clearance,
                        min_ground_points=self._min_ground_points,
                        min_ground_fit_points=self._min_ground_fit_points,
                        min_ground_cells=self._min_ground_cells,
                        ground_support_radius=self._ground_support_radius,
                        min_ground_cell_span=self._min_ground_cell_span,
                    )
                except (
                    AssertionError, RuntimeError, TypeError, ValueError
                ) as exc:
                    self.get_logger().warning(
                        f"点云转换失败，发布停车扫描: {exc}")
                    self._publish_scan(
                        msg.header, np.empty((0, 3)), emergency=True)
                    return

                self._publish_scan(msg.header, obstacles)

            def _publish_scan(self, header, obstacles, emergency=False):
                scan = self._LaserScan()
                scan.header = header
                scan.angle_min = self._angle_min
                scan.angle_max = self._angle_max
                scan.angle_increment = self._angle_increment
                scan.time_increment = 0.0
                scan.scan_time = self._scan_time
                scan.range_min = self._range_min
                scan.range_max = self._range_max
                count = int(math.ceil(
                    (self._angle_max - self._angle_min)
                    / self._angle_increment
                )) + 1
                ranges = build_scan_ranges(
                    obstacles,
                    count=count,
                    angle_min=self._angle_min,
                    angle_increment=self._angle_increment,
                    range_min=self._range_min,
                    range_max=self._range_max,
                    emergency=emergency,
                )
                scan.ranges = ranges.tolist()
                self._scan_pub.publish(scan)

        self.node = _Node()
        self._rclpy = rclpy

    def spin(self):
        self._rclpy.spin(self.node)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    adapter = HillGroundObstacleScan()
    try:
        adapter.spin()
    finally:
        adapter.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
