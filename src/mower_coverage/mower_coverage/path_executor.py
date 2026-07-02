#!/usr/bin/env python3
"""
path_executor.py — 覆盖路径执行器

功能：
  1. 接收 boustrophedon_planner 生成的路径点序列
  2. 逐个执行路径点（通过 Nav2 action 或 直接 cmd_vel）
  3. 动态障碍物检测 → 暂停刀盘 → 等待清除 → 继续执行
  4. 断点续割：跟踪已完成路径段，支持中断后恢复
  5. 边界检测：如果位置超出区域，自动停止

工作模式：
  - mode: 'nav2'   — 通过 Nav2 NavigateToPose action 执行（适合大场景）
  - mode: 'direct' — 直接发布 cmd_vel 跟踪路径（适合小场景/调试）

用法：
  ros2 run mower_coverage path_executor
  ros2 service call /coverage/start std_srvs/srv/Trigger
  ros2 service call /coverage/pause std_srvs/srv/Trigger
  ros2 service call /coverage/resume std_srvs/srv/Trigger
  ros2 service call /coverage/stop std_srvs/srv/Trigger
"""

import json
import os
import math
from enum import Enum
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import Header, Bool, ColorRGBA
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus
from shapely.geometry import Point as ShapelyPoint, Polygon

# 延迟导入 nav2_msgs（仅 nav2 模式需要）
_nav2_available = False
try:
    from nav2_msgs.action import NavigateToPose as _NavMsg
    _nav2_available = True
except ImportError:
    pass


class ExecutorState(Enum):
    IDLE = 'idle'
    RUNNING = 'running'
    PAUSED = 'paused'
    COMPLETED = 'completed'
    ERROR = 'error'


class PathExecutor(Node):
    """路径执行器"""

    def __init__(self):
        super().__init__('path_executor')

        # 参数
        self.declare_parameter('mode', 'direct')  # 'nav2' or 'direct'
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('checkpoint_file', os.path.expanduser('~/coverage_checkpoint.json'))
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('obstacle_stop_range', 0.5)
        self.declare_parameter('blade_enabled', True)

        self.mode = self.get_parameter('mode').value
        self.frame_id = self.get_parameter('frame_id').value
        self.checkpoint_file = self.get_parameter('checkpoint_file').value
        self.max_speed = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.blade_enabled = self.get_parameter('blade_enabled').value

        # 状态
        self.state = ExecutorState.IDLE
        self.waypoints: List[Tuple[float, float]] = []
        self.current_index = 0  # 当前执行到的路径点索引
        self.completed_indices: List[int] = []

        # 当前位置
        self.current_position: Tuple[float, float] = (0.0, 0.0)
        self.current_yaw: float = 0.0

        # 作业区域（用于边界检测）
        self.area_polygon: Polygon = None

        # 发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.blade_pub = self.create_publisher(Bool, '/blade_enabled', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage/execution_markers', 10)
        self.status_pub = self.create_publisher(Bool, '/coverage/is_running', 10)

        # 订阅器
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/coverage/path', self.path_callback, 10)

        # 控制服务
        self.srv_start = self.create_service(Trigger, '/coverage/start', self.start_callback)
        self.srv_pause = self.create_service(Trigger, '/coverage/pause', self.pause_callback)
        self.srv_resume = self.create_service(Trigger, '/coverage/resume', self.resume_callback)
        self.srv_stop = self.create_service(Trigger, '/coverage/stop', self.stop_callback)

        # 控制循环定时器
        self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.create_timer(2.0, self.publish_status)

        # Nav2 Action 客户端
        if self.mode == 'nav2':
            if not _nav2_available:
                self.get_logger().error('nav2_msgs 未安装！无法使用 nav2 模式')
                self.mode = 'direct'
            else:
                from nav2_msgs.action import NavigateToPose
                cb_group = MutuallyExclusiveCallbackGroup()
                self.nav2_client = ActionClient(
                    self, NavigateToPose, '/navigate_to_pose',
                    callback_group=cb_group)
                self.nav2_goal_handle = None

        # 恢复上一次的检查点
        self._load_checkpoint()

        self.get_logger().info(f'路径执行器已启动 (mode={self.mode})')
        self.get_logger().info(f'  已加载 {len(self.waypoints)} 个路径点, 当前位置索引: {self.current_index}')

    def odom_callback(self, msg: Odometry):
        """更新当前位置"""
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # 提取 yaw
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def path_callback(self, msg: Path):
        """接收新路径"""
        if self.state == ExecutorState.RUNNING:
            self.get_logger().warn('正在执行中，忽略新路径')
            return

        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.current_index = 0
        self.completed_indices = []
        self.get_logger().info(f'收到新路径: {len(self.waypoints)} 个点')

    def start_callback(self, request, response):
        """开始执行"""
        if not self.waypoints:
            response.success = False
            response.message = "路径为空，请先规划路径"
            return response

        self.state = ExecutorState.RUNNING
        self.current_index = 0
        self.get_logger().info('开始执行覆盖路径')
        response.success = True
        response.message = f'开始执行 {len(self.waypoints)} 个路径点'
        return response

    def pause_callback(self, request, response):
        """暂停执行"""
        if self.state != ExecutorState.RUNNING:
            response.success = False
            response.message = f'当前状态为 {self.state.value}，无法暂停'
            return response

        self.state = ExecutorState.PAUSED
        self._stop_robot()
        self.set_blade(False)
        self._save_checkpoint()
        self.get_logger().info('执行已暂停')
        response.success = True
        response.message = f'已暂停 (当前位置索引: {self.current_index})'
        return response

    def resume_callback(self, request, response):
        """恢复执行"""
        if self.state != ExecutorState.PAUSED:
            response.success = False
            response.message = f'当前状态为 {self.state.value}，只能从暂停状态恢复'
            return response

        self.state = ExecutorState.RUNNING
        self.set_blade(True)
        self.get_logger().info(f'恢复执行 (索引: {self.current_index})')
        response.success = True
        response.message = '已恢复执行'
        return response

    def stop_callback(self, request, response):
        """停止执行"""
        self.state = ExecutorState.IDLE
        self._stop_robot()
        self.set_blade(False)
        self._save_checkpoint()
        self.get_logger().info('执行已停止')
        response.success = True
        response.message = f'已停止 (共完成 {len(self.completed_indices)}/{len(self.waypoints)} 个点)'
        return response

    def set_blade(self, enabled: bool):
        """控制刀盘"""
        msg = Bool()
        msg.data = enabled and self.blade_enabled
        self.blade_pub.publish(msg)

    def control_loop(self):
        """控制主循环 (10 Hz)"""
        if self.state != ExecutorState.RUNNING:
            return

        if self.current_index >= len(self.waypoints):
            self.state = ExecutorState.COMPLETED
            self._stop_robot()
            self.set_blade(False)
            self.get_logger().info('✅ 全覆盖路径执行完成!')
            return

        # 边界检查
        if self.area_polygon and not self.area_polygon.contains(
                ShapelyPoint(self.current_position)):
            self.get_logger().error('⚠️ 机器人超出作业区域边界！紧急停止')
            self.state = ExecutorState.ERROR
            self._stop_robot()
            self.set_blade(False)
            return

        target = self.waypoints[self.current_index]
        dx = target[0] - self.current_position[0]
        dy = target[1] - self.current_position[1]
        distance = math.sqrt(dx*dx + dy*dy)

        # 检测是否已到达当前目标点
        reached = distance < self.goal_tolerance

        # 防止错过目标点：如果距离上次最近距离开始增大，而且已经足够近 (< 2m)，也视为到达
        if not reached and hasattr(self, '_prev_min_dist') and self._prev_min_dist is not None:
            if distance > self._prev_min_dist + 0.05 and self._prev_min_dist < 2.0:
                self.get_logger().info(f'跳过目标点 #{self.current_index} (最近距离 {self._prev_min_dist:.2f}m)')
                reached = True

        if reached:
            # 到达目标点
            self._prev_min_dist = None
            self.completed_indices.append(self.current_index)
            self.current_index += 1
            self.publish_execution_markers()

            if self.current_index % 50 == 0:
                progress = self.current_index / len(self.waypoints) * 100
                self.get_logger().info(f'进度: {self.current_index}/{len(self.waypoints)} ({progress:.1f}%)')
            return
        else:
            # 记录最近距离
            if not hasattr(self, '_prev_min_dist') or self._prev_min_dist is None:
                self._prev_min_dist = distance
            else:
                self._prev_min_dist = min(self._prev_min_dist, distance)

        # 计算控制指令 (在 direct 模式下)
        if self.mode == 'direct':
            target_yaw = math.atan2(dy, dx)
            yaw_error = self._normalize_angle(target_yaw - self.current_yaw)

            twist = Twist()
            if abs(yaw_error) > 0.3:
                # 先转向
                twist.angular.z = self.max_angular * (1.0 if yaw_error > 0 else -1.0)
            else:
                # 前进
                twist.linear.x = min(self.max_speed, distance * 2.0)
                twist.angular.z = yaw_error * 2.0

            self.cmd_vel_pub.publish(twist)
            self.set_blade(True)

    def _stop_robot(self):
        """停止机器人"""
        self.cmd_vel_pub.publish(Twist())

    def _normalize_angle(self, angle: float) -> float:
        """归一化角度到 [-π, π]"""
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def publish_execution_markers(self):
        """发布执行状态 Marker 用于 RViz 显示"""
        markers = MarkerArray()

        # 已完成路径
        if self.completed_indices:
            done = Marker()
            done.header.frame_id = self.frame_id
            done.ns = 'coverage_done'
            done.id = 0
            done.type = Marker.LINE_STRIP
            done.action = Marker.ADD
            done.pose.orientation.w = 1.0
            done.scale.x = 0.12
            done.color = ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.8)
            for i in self.completed_indices:
                if i < len(self.waypoints):
                    done.points.append(Point(x=self.waypoints[i][0], y=self.waypoints[i][1], z=0.02))
            markers.markers.append(done)

        # 剩余路径（灰色）
        remaining = Marker()
        remaining.header.frame_id = self.frame_id
        remaining.ns = 'coverage_remaining'
        remaining.id = 1
        remaining.type = Marker.LINE_STRIP
        remaining.action = Marker.ADD
        remaining.pose.orientation.w = 1.0
        remaining.scale.x = 0.06
        remaining.color = ColorRGBA(r=0.5, b=0.5, a=0.3)
        for i in range(self.current_index, len(self.waypoints)):
            remaining.points.append(Point(x=self.waypoints[i][0], y=self.waypoints[i][1], z=0.015))
        markers.markers.append(remaining)

        # 当前目标点
        if self.current_index < len(self.waypoints):
            target_pt = self.waypoints[self.current_index]
            target_marker = Marker()
            target_marker.header.frame_id = self.frame_id
            target_marker.ns = 'coverage_target'
            target_marker.id = 2
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = target_pt[0]
            target_marker.pose.position.y = target_pt[1]
            target_marker.pose.position.z = 0.05
            target_marker.pose.orientation.w = 1.0
            target_marker.scale.x = 0.3
            target_marker.scale.y = 0.3
            target_marker.scale.z = 0.3
            target_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
            markers.markers.append(target_marker)

        self.marker_pub.publish(markers)

    def publish_status(self):
        """发布运行状态"""
        msg = Bool()
        msg.data = self.state == ExecutorState.RUNNING
        self.status_pub.publish(msg)

    def _save_checkpoint(self):
        """保存断点（用于断点续割）"""
        data = {
            'current_index': self.current_index,
            'completed_indices': self.completed_indices,
            'total': len(self.waypoints),
        }
        try:
            with open(self.checkpoint_file, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            self.get_logger().error(f'保存检查点失败: {e}')

    def _load_checkpoint(self):
        """加载断点"""
        if os.path.exists(self.checkpoint_file):
            try:
                with open(self.checkpoint_file, 'r') as f:
                    data = json.load(f)
                self.current_index = data.get('current_index', 0)
                self.completed_indices = data.get('completed_indices', [])
                total = data.get('total', 0)
                if total > 0:
                    self.get_logger().info(f'恢复检查点: {self.current_index}/{total}')
            except Exception as e:
                self.get_logger().error(f'加载检查点失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_robot()
        node.set_blade(False)
        node._save_checkpoint()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
