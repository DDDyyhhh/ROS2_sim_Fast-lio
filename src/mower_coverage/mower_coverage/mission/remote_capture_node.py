#!/usr/bin/env python3
"""ROS adapter for the simulation remote-capture workflow."""

import json
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, String

from mower_coverage.state_paths import ensure_parent, state_file

from .store import MissionCaptureStore


class RemoteCaptureNode(Node):
    """Record one simulated robot trajectory at a time and persist missions."""

    def __init__(self):
        super().__init__('remote_capture')
        self.declare_parameter('pose_topic', '/odom')
        self.declare_parameter('capture_frame_id', 'odom')
        self.declare_parameter('allow_local_odom', True)
        self.declare_parameter('pose_timeout', 0.5)
        self.declare_parameter('sample_period', 0.1)
        self.declare_parameter(
            'mission_file', state_file('remote_capture_mission.yaml'))
        self.declare_parameter('initial_health_state', 'RED')
        self.declare_parameter('health_topic', '/localization/health')

        self.capture_frame_id = str(
            self.get_parameter('capture_frame_id').value)
        self.allow_local_odom = bool(
            self.get_parameter('allow_local_odom').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.mission_file = os.path.expanduser(
            str(self.get_parameter('mission_file').value))
        profile = {
            'closure_tolerance': 0.5,
            'simplify_tolerance': 0.01,
        }
        self.store = MissionCaptureStore(
            profile,
            frame_id=self.capture_frame_id,
            allow_local_odom=self.allow_local_odom,
        )

        self.latest_pose = None
        self.latest_pose_at = None
        self.latest_pose_stamp = None
        self.health_state = str(
            self.get_parameter('initial_health_state').value)
        self.health_reasons = []
        self.last_error = ''
        self._last_sample_stamp = None

        self.state_pub = self.create_publisher(
            String, '/mission/capture/state', 10)
        self.raw_path_pub = self.create_publisher(
            Path, '/mission/capture/raw_path', 10)
        self.mission_pub = self.create_publisher(
            String, '/mission/capture/mission', 10)
        self.drive_allowed_pub = self.create_publisher(
            Bool, '/mission/capture/drive_allowed', 10)

        pose_topic = self.get_parameter('pose_topic').value
        health_topic = self.get_parameter('health_topic').value
        self.pose_sub = self.create_subscription(
            Odometry, pose_topic, self.pose_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/mission/capture/command', self.command_callback, 10)
        self.health_sub = self.create_subscription(
            String, health_topic, self.health_callback, 10)
        self.timer = self.create_timer(
            float(self.get_parameter('sample_period').value),
            self.tick,
        )

        self._load_mission()
        self.publish_all()
        self.get_logger().info(
            f'remote capture ready: pose={pose_topic}, '
            f'frame={self.capture_frame_id}')

    def pose_callback(self, msg):
        pose = msg.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.latest_pose = None
            self.latest_pose_at = None
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = time.monotonic()
        self.latest_pose = msg
        self.latest_pose_at = time.monotonic()
        self.latest_pose_stamp = stamp

    def health_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.health_state = 'RED'
            self.health_reasons = ['invalid localization health message']
            return
        state = data.get('state')
        if state not in {'GREEN', 'YELLOW', 'RED'}:
            self.health_state = 'RED'
            self.health_reasons = ['invalid localization health state']
            return
        self.health_state = state
        reasons = data.get('reasons', [])
        self.health_reasons = [str(reason) for reason in reasons]

    def command_callback(self, msg):
        self.last_error = ''
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('command must be an object')
            action = data.get('action')
            if action == 'start':
                self.store.start(data.get('type'), data.get('id'))
            elif action == 'finish':
                self.store.finish(self.health_state)
            elif action == 'undo':
                self.store.undo()
            elif action == 'save_draft':
                self.store.save_draft()
                self._persist()
            elif action == 'cancel':
                self.store.cancel()
            elif action == 'confirm':
                corridor = data.get('corridor')
                active = self.store.session
                if active is not None and active.object_type == 'corridor':
                    if not isinstance(corridor, dict):
                        raise ValueError('corridor metadata is required')
                    self.store.set_corridor_metadata(
                        corridor.get('width'),
                        corridor.get('from_work_area_id'),
                        corridor.get('to_work_area_id'),
                        corridor.get('bidirectional'),
                    )
                self.store.confirm(self.health_state)
                self._persist()
            elif action == 'clear':
                self.store.clear()
                self._persist()
            else:
                raise ValueError(f'unknown capture action: {action}')
        except (TypeError, ValueError, RuntimeError) as error:
            self.last_error = str(error)
            self.get_logger().warn(f'capture command rejected: {error}')
        self.publish_all()

    def tick(self):
        if (self.store.session is not None
                and self.store.session.state == 'capturing'):
            self._record_latest_pose()
        self.publish_all()

    def _record_latest_pose(self):
        if self.latest_pose is None or self.latest_pose_at is None:
            return
        if self.latest_pose_stamp == self._last_sample_stamp:
            return

        pose = self.latest_pose.pose.pose
        fresh = time.monotonic() - self.latest_pose_at <= self.pose_timeout
        localization_ok = fresh and self.health_state == 'GREEN'
        sample = {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'timestamp': float(self.latest_pose_stamp),
            'frame_id': self.capture_frame_id,
            'localization_ok': localization_ok,
            'localization_state': self.health_state,
        }
        try:
            self.store.record_pose(sample)
            self._last_sample_stamp = self.latest_pose_stamp
        except (TypeError, ValueError, RuntimeError) as error:
            self.last_error = str(error)

    def _load_mission(self):
        if not os.path.exists(self.mission_file):
            return
        try:
            with open(self.mission_file, 'r', encoding='utf-8') as stream:
                self.store.load_document(yaml.safe_load(stream))
        except (OSError, ValueError, yaml.YAMLError) as error:
            self.last_error = f'failed to load mission: {error}'

    def _persist(self):
        try:
            ensure_parent(self.mission_file)
            with open(self.mission_file, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(
                    self.store.document(),
                    stream,
                    allow_unicode=True,
                    sort_keys=False,
                )
        except (OSError, yaml.YAMLError) as error:
            self.last_error = f'failed to save mission: {error}'

    def _active_state(self):
        active = self.store.session
        if active is None:
            return 'idle'
        return active.state

    def _is_pose_fresh(self):
        return (
            self.latest_pose_at is not None
            and time.monotonic() - self.latest_pose_at <= self.pose_timeout
        )

    def _drive_allowed(self):
        return (
            self.store.session is not None
            and self.store.session.state == 'capturing'
            and self.health_state == 'GREEN'
            and self._is_pose_fresh()
        )

    def publish_all(self):
        active_snapshot = self.store.snapshot()
        state = {
            **active_snapshot,
            'state': self._active_state(),
            'health_state': self.health_state,
            'health_reasons': list(self.health_reasons),
            'pose_fresh': self._is_pose_fresh(),
            'drive_allowed': self._drive_allowed(),
            'last_error': self.last_error,
        }
        state_msg = String()
        state_msg.data = json.dumps(state, ensure_ascii=False)
        self.state_pub.publish(state_msg)

        mission_msg = String()
        mission_msg.data = json.dumps(
            self.store.document(), ensure_ascii=False)
        self.mission_pub.publish(mission_msg)

        allowed_msg = Bool()
        allowed_msg.data = self._drive_allowed()
        self.drive_allowed_pub.publish(allowed_msg)

        self._publish_raw_path()

    def _publish_raw_path(self):
        path = Path()
        path.header.frame_id = self.capture_frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        active = self.store.snapshot().get('active')
        samples = [] if active is None else active.get('raw_trajectory', [])
        for sample in samples:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(sample['x'])
            pose.pose.position.y = float(sample['y'])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.raw_path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = RemoteCaptureNode()
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
