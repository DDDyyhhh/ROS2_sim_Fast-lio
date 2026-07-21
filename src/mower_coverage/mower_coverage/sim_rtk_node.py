#!/usr/bin/env python3
"""Publish an explicitly simulated RTK antenna and health contract."""

import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


class SimulationRtkNode(Node):
    """Mirror Gazebo's antenna fix onto the canonical RTK topics."""

    def __init__(self):
        super().__init__('simulation_rtk')
        self.declare_parameter('input_fix_topic', '/gps/fix')
        self.declare_parameter('fix_topic', '/rtk/gps/fix')
        self.declare_parameter('status_topic', '/rtk/status')
        self.declare_parameter('health_topic', '/localization/health')
        self.declare_parameter('stale_after', 1.0)
        self.declare_parameter('simulated_satellites', 18)
        self.declare_parameter('simulated_hdop', 0.6)

        input_topic = str(self.get_parameter('input_fix_topic').value)
        fix_topic = str(self.get_parameter('fix_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        health_topic = str(self.get_parameter('health_topic').value)
        self.stale_after = float(self.get_parameter('stale_after').value)
        self.simulated_satellites = int(
            self.get_parameter('simulated_satellites').value)
        self.simulated_hdop = float(self.get_parameter('simulated_hdop').value)
        if self.stale_after <= 0.0:
            raise ValueError('RTK stale_after must be positive')

        self.fix_pub = self.create_publisher(NavSatFix, fix_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.health_pub = self.create_publisher(String, health_topic, 10)
        self.fix_sub = self.create_subscription(
            NavSatFix, input_topic, self.fix_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.2, self.publish_status)

        self.latest_fix = None
        self.latest_fix_at = None
        self.latest_odom_at = None
        self.latest_frame_id = 'gps_link'
        self.get_logger().info(
            f'simulated RTK: {input_topic} -> {fix_topic}; '
            f'status={status_topic}, health={health_topic}')

    def fix_callback(self, msg):
        now = time.monotonic()
        if self._valid_fix(msg):
            self.latest_fix = msg
            self.latest_fix_at = now
            self.latest_frame_id = msg.header.frame_id or 'gps_link'
        else:
            self.latest_fix = None
            self.latest_fix_at = None
        self.fix_pub.publish(msg)

    def odom_callback(self, msg):
        pose = msg.pose.pose.position
        if all(math.isfinite(float(value)) for value in (
                pose.x, pose.y, pose.z)):
            self.latest_odom_at = time.monotonic()
        else:
            self.latest_odom_at = None

    def publish_status(self):
        now = time.monotonic()
        fix_age = self._age(self.latest_fix_at, now)
        odom_age = self._age(self.latest_odom_at, now)
        fix_fresh = (
            self.latest_fix is not None
            and fix_age is not None
            and fix_age <= self.stale_after
        )
        odom_fresh = (
            odom_age is not None and odom_age <= self.stale_after
        )
        trusted = fix_fresh and odom_fresh
        solution = 'RTK_FIXED' if fix_fresh else 'NO_FIX'
        reasons = []
        if not fix_fresh:
            reasons.append('simulated RTK fix is missing or stale')
        if not odom_fresh:
            reasons.append('simulation odometry is missing or stale')
        status = {
            'source': 'simulation',
            'simulated': True,
            'state': solution,
            'solution': solution,
            'quality': 4 if fix_fresh else 0,
            'satellites': self.simulated_satellites if fix_fresh else 0,
            'hdop': self.simulated_hdop if fix_fresh else None,
            'ntrip': 'SIMULATED',
            'corrections_fresh': fix_fresh,
            'global_position_trusted': trusted,
            'serial': 'SIMULATED',
            'frame_id': self.latest_frame_id,
            'last_fix_age_s': fix_age,
            'last_odom_age_s': odom_age,
            'last_error': None if not reasons else '; '.join(reasons),
        }
        status_msg = String()
        status_msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(status_msg)

        health = {
            'source': 'simulation',
            'mode': 'simulation_local_odom',
            'state': 'GREEN' if trusted else 'RED',
            'capture_allowed': trusted,
            'execution_allowed': False,
            'reasons': reasons,
        }
        health_msg = String()
        health_msg.data = json.dumps(health, ensure_ascii=False)
        self.health_pub.publish(health_msg)

    @staticmethod
    def _age(observed_at, now):
        if observed_at is None:
            return None
        return max(0.0, now - observed_at)

    @staticmethod
    def _valid_fix(msg):
        status = getattr(msg.status, 'status', NavSatStatus.STATUS_NO_FIX)
        return (
            status >= NavSatStatus.STATUS_FIX
            and math.isfinite(float(msg.latitude))
            and math.isfinite(float(msg.longitude))
            and abs(float(msg.latitude)) > 1e-9
            and abs(float(msg.longitude)) > 1e-9
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimulationRtkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass


if __name__ == '__main__':
    main()
