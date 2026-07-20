"""Dependency-free localization health assessment."""

from dataclasses import dataclass


@dataclass(frozen=True)
class HealthAssessment:
    """Decision exposed to capture and execution callers."""

    state: str
    capture_allowed: bool
    execution_allowed: bool
    reasons: tuple = ()


_REQUIRED_SIGNALS = (
    'fused_pose_valid',
    'fused_pose_fresh',
    'fused_pose_in_map',
    'local_odom_valid',
    'local_odom_fresh',
    'mid360_valid',
    'mid360_fresh',
    'imu_valid',
    'imu_fresh',
    'pointcloud_valid',
    'pointcloud_fresh',
    'pointcloud_time_monotonic',
    'rtk_fixed',
    'rtk_fresh',
)

_CRITICAL_FAILURES = {
    'fused_pose_valid': 'fused pose is invalid',
    'fused_pose_fresh': 'fused pose is stale',
    'fused_pose_in_map': 'fused pose is not in map frame',
    'local_odom_valid': 'local odometry is invalid',
    'local_odom_fresh': 'local odometry is stale',
    'mid360_valid': 'Mid-360/FAST-LIO is invalid',
    'mid360_fresh': 'Mid-360/FAST-LIO is stale',
    'imu_valid': 'IMU is invalid',
    'imu_fresh': 'IMU is stale',
    'pointcloud_valid': 'point cloud is invalid',
    'pointcloud_fresh': 'point cloud is stale',
    'pointcloud_time_monotonic': 'point cloud time is not monotonic',
}


def assess_localization(observation):
    """Classify one complete localization observation.

    ``GREEN`` requires the fused/local chain and sensor freshness checks to
    pass, plus a fresh RTK fixed solution.  A healthy local chain without that
    RTK condition is ``YELLOW``; this release still blocks capture and motion.
    Any malformed or missing input is ``RED``.
    """
    if not isinstance(observation, dict):
        return HealthAssessment(
            'RED', False, False, ('health observation must be an object',))

    reasons = []
    values = {}
    for signal in _REQUIRED_SIGNALS:
        if signal not in observation:
            reasons.append(f'missing health signal: {signal}')
            continue
        value = observation[signal]
        if not isinstance(value, bool):
            reasons.append(f'health signal must be boolean: {signal}')
            continue
        values[signal] = value
        if signal in _CRITICAL_FAILURES and not value:
            reasons.append(_CRITICAL_FAILURES[signal])

    if reasons:
        return HealthAssessment('RED', False, False, tuple(reasons))

    if not values['rtk_fixed'] or not values['rtk_fresh']:
        return HealthAssessment(
            'YELLOW', False, False, ('RTK is not fixed and fresh',))

    return HealthAssessment('GREEN', True, True)
