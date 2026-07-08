#!/usr/bin/env python3
"""
multi_area_executor.py — 多区域路径执行器（带 LiDAR 避障）

功能：
  - 获取规划器发布的路径 (/coverage/multi_path)
  - 沿路径逐个执行（发布 /cmd_vel）
  - 覆盖区域可视化 + 断点续割
  - LiDAR 避障（safe 模式）：检测前方障碍物自动停车避让

执行模式：
  - direct:  直接跟随路径，无避障（原行为）
  - safe:    带 LiDAR 避障（默认，推荐）

用法：
  ros2 service call /multi_area/plan std_srvs/srv/Trigger
  ros2 service call /multi_area/plan_and_start std_srvs/srv/Trigger
"""

import json
import math
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header, String
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


class MultiAreaExecutor(Node):
    """多区域路径执行器（带 LiDAR 避障）"""

    def __init__(self):
        super().__init__('multi_area_executor')

        # 参数
        param_defaults = [
            ('frame_id', 'map'),
            ('goal_tolerance', 0.3),
            ('checkpoint_file', os.path.expanduser('~/hill_coverage_checkpoint.json')),
            ('max_linear_speed', 1.0),
            ('max_angular_speed', 1.0),
            ('execution_mode', 'safe'),          # direct | safe
            ('obstacle_stop_range', 0.8),         # 前方 0.8m 内有障碍物 → 停车
            ('obstacle_scan_angle', 60.0),        # 前方 ±30° 扫描范围（度）
        ]
        for name, default in param_defaults:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

        self.frame_id = self.get_parameter('frame_id').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.checkpoint_file = self.get_parameter('checkpoint_file').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.execution_mode = self.get_parameter('execution_mode').value
        self.obstacle_stop_range = self.get_parameter('obstacle_stop_range').value
        self.obstacle_scan_angle = self.get_parameter('obstacle_scan_angle').value

        # 状态
        self.waypoints = []         # [(x, y, speed), ...]
        self.current_index = 0
        self.total_waypoints = 0
        self.executing = False
        self._paused = False        # True=暂停可恢复, False=停止不可恢复
        self.completed_areas = []
        self.robot_pose = (0.0, 0.0)
        self.robot_yaw = 0.0       # 机器人朝向（弧度）
        self.area_names = []
        self.path_received = False

        # LiDAR 避障状态
        self.latest_scan = None
        self.obstacle_detected = False
        self.obstacle_angle = 0.0       # 障碍物方向（弧度）
        self.obstacle_distance = 999.0  # 障碍物距离（m）
        self.obstacle_brake_count = 0   # 连续刹车计数（用于恢复决策）

        # 里程计
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # 速度发布
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 覆盖可视化
        self.marker_pub = self.create_publisher(
            MarkerArray, '/coverage/execution_markers', 10)

        # ★ 覆盖率统计信息发布 → 给 Web 前端
        self.stat_pub = self.create_publisher(
            String, '/coverage/statistics', 10)

        # 路径订阅（接收 planner 发布的多区域路径）
        self.path_sub = self.create_subscription(
            Path, '/coverage/multi_path', self.path_callback, 10)

        # LiDAR 扫描订阅（避障用）
        if self.execution_mode == 'safe':
            self.scan_sub = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, 10)
            self.get_logger().info(
                f'🔒 避障模式: 前方 {self.obstacle_stop_range}m 内障碍物自动停车')

        # 服务
        self.srv_plan_and_start = self.create_service(
            Trigger, '/multi_area/plan_and_start', self.plan_and_start_cb)
        self.srv_stop = self.create_service(
            Trigger, '/multi_area/stop', self.stop_cb)
        self.srv_pause = self.create_service(
            Trigger, '/multi_area/pause', self.pause_cb)
        self.srv_resume = self.create_service(
            Trigger, '/multi_area/resume', self.resume_cb)
        self.srv_status = self.create_service(
            Trigger, '/multi_area/status', self.status_cb)
        self.srv_clear = self.create_service(
            Trigger, '/multi_area/clear', self.clear_cb)

        # 控制定时器（10Hz）
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # 可视化定时器（1Hz）
        self.viz_timer = self.create_timer(1.0, self.publish_coverage_viz)

        # 已覆盖位置
        self.covered_positions = []

        mode_label = 'safe（带 LiDAR 避障）' if self.execution_mode == 'safe' else 'direct（无避障）'
        self.get_logger().info(f'多区域执行器已启动 — 模式: {mode_label}')
        self.get_logger().info(
            '  使用: ros2 service call /multi_area/plan 后调用 /multi_area/plan_and_start')

    # ==============================================================
    # 回调
    # ==============================================================
    def odom_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x,
                           msg.pose.pose.position.y)
        # 从四元数提取偏航角
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        """处理 LiDAR 扫描数据 — 检测前方障碍物"""
        self.latest_scan = msg
        self.obstacle_detected = False
        self.obstacle_angle = 0.0
        self.obstacle_distance = 999.0

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        scan_angle_rad = math.radians(self.obstacle_scan_angle / 2.0)

        for i, range_val in enumerate(msg.ranges):
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
            if math.isnan(range_val) or math.isinf(range_val):
                continue

            angle = angle_min + i * angle_increment
            # 只检查前方扇区内的点
            if abs(angle) < scan_angle_rad:
                if range_val < self.obstacle_stop_range:
                    self.obstacle_detected = True
                    if range_val < self.obstacle_distance:
                        self.obstacle_distance = range_val
                        self.obstacle_angle = angle

    def path_callback(self, msg):
        """从规划器接收路径（任何时候收到都更新）

        注意：规划器以 1Hz 重复发布路径（供晚加入的可视化订阅者），
        因此执行中不重置 current_index，避免进度归零。
        """
        wps = []
        for ps in msg.poses:
            wps.append((ps.pose.position.x, ps.pose.position.y, 1.0))
        if wps:
            path_len = len(wps)
            # 如果正在执行且路径一样 → 不重置进度（防 1Hz 重发导致归零）
            if self.executing and path_len == self.total_waypoints:
                self.waypoints = wps  # 仍然更新路径点（可能有微小优化）
                return
            self.waypoints = wps
            self.total_waypoints = path_len
            self.current_index = 0
            self.path_received = True
            self.get_logger().info(f'已接收路径: {path_len} 个路径点')

    def plan_and_start_cb(self, req, resp):
        """开始执行已有路径（需先调用 /multi_area/plan）"""
        if self.executing:
            resp.success = False
            resp.message = '正在执行中，请先 stop'
            return resp

        if not self.waypoints:
            resp.success = False
            resp.message = '没有路径数据。请先调用 /multi_area/plan 规划'
            return resp

        self.executing = True
        self.current_index = 0
        self._paused = True          # ← 开始后可暂停
        self.obstacle_brake_count = 0
        resp.success = True
        resp.message = f'开始执行: {len(self.waypoints)} 个路径点'
        self.get_logger().info(resp.message)
        return resp

    def pause_cb(self, req, resp):
        self.executing = False
        self._paused = True          # ← 暂停：可恢复
        self._stop_robot()
        resp.success = True
        resp.message = f'已暂停 (目标点 #{self.current_index})'
        return resp

    def stop_cb(self, req, resp):
        self.executing = False
        self._paused = False         # ← 停止：不可恢复
        self.current_index = 0
        self.covered_positions = []
        self._stop_robot()
        self.save_checkpoint()
        resp.success = True
        resp.message = '已停止，请重新执行或规划'
        return resp

    def resume_cb(self, req, resp):
        if self.executing:
            resp.success = False
            resp.message = '已在执行中'
            return resp
        if not self._paused:         # ← 只有暂停后才能恢复
            resp.success = False
            resp.message = '已停止，请重新执行'
            return resp
        if not self.waypoints:
            resp.success = False
            resp.message = '没有路径数据'
            return resp
        self.executing = True
        resp.success = True
        resp.message = f'已恢复 (目标点 #{self.current_index})'
        return resp

    def status_cb(self, req, resp):
        info = f'执行中: {self.executing}'
        info += f', 进度: {self.current_index}/{len(self.waypoints)}'
        info += f', 已覆盖: {len(self.covered_positions)}'
        info += f', 位置: ({self.robot_pose[0]:.1f}, {self.robot_pose[1]:.1f})'
        if self.obstacle_detected:
            info += f', ⚠️ 障碍物: {self.obstacle_distance:.2f}m'
        resp.success = True
        resp.message = info
        return resp

    def clear_cb(self, req, resp):
        """清除所有路径数据和执行状态（停止 + 清空路径）"""
        self.executing = False
        self._paused = False         # ← 清除：不可恢复
        self._stop_robot()
        self.waypoints = []
        self.current_index = 0
        self.total_waypoints = 0
        self.covered_positions = []
        self.path_received = False
        self.completed_areas = []
        self.area_names = []
        self.clear_checkpoint()
        resp.success = True
        resp.message = '已清除所有路径和状态'
        self.get_logger().info('🧹 已清除所有路径和状态')
        return resp

    # ==============================================================
    # 控制循环
    # ==============================================================
    def control_loop(self):
        """10Hz 控制器 — 先转后直策略 + LiDAR 避障"""
        if not self.executing or not self.waypoints:
            return

        if self.current_index >= len(self.waypoints):
            self.executing = False
            self._stop_robot()
            self.save_checkpoint()
            self.get_logger().info('全部路径执行完成！')
            return

        # ==========================================================
        # LiDAR 避障检查（safe 模式）
        # ==========================================================
        if self.execution_mode == 'safe' and self.obstacle_detected:
            self.obstacle_brake_count += 1
            self._stop_robot()
            twist = Twist()

            if self.obstacle_brake_count < 5:
                # 前 5 帧（0.5秒）：完全停住，观察
                self.cmd_pub.publish(twist)
                return

            # 超过 0.5 秒：尝试绕行
            # 障碍物在右边 → 左转；障碍物在左边 → 右转
            avoid_turn = -1.0 if self.obstacle_angle > 0 else 1.0
            twist.angular.z = avoid_turn * self.max_angular * 0.6

            if self.obstacle_brake_count > 30:
                # 3秒还没摆脱障碍物 → 向后倒车
                twist.linear.x = -0.3

            self.cmd_pub.publish(twist)
            return
        else:
            self.obstacle_brake_count = 0

        # ==========================================================
        # 正常路径跟踪
        # ==========================================================
        target = self.waypoints[self.current_index]
        tx, ty, target_speed = target[0], target[1], target[2] if len(target) == 3 else self.max_linear

        dx = tx - self.robot_pose[0]
        dy = ty - self.robot_pose[1]
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.goal_tolerance:
            self.current_index += 1
            self.covered_positions.append((tx, ty))
            if self.current_index % 50 == 0:
                self.get_logger().info(
                    f'  进度: {self.current_index}/{len(self.waypoints)}')
            return

        # 计算相对于机器人朝向的角度误差
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.robot_yaw
        # 归一化到 [-π, π]
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        twist = Twist()

        TURN_THRESHOLD = 0.35  # ~20度，超过此值则原地旋转

        if abs(angle_error) > TURN_THRESHOLD:
            # 先转后直 — 原地旋转到目标方向
            twist.angular.z = max(-self.max_angular,
                                  min(self.max_angular, angle_error * 3.0))
            twist.linear.x = 0.0
        else:
            # 对得准 → 向前行驶 + 轻微方向修正
            twist.linear.x = min(target_speed, distance)
            twist.angular.z = max(-self.max_angular * 0.4,
                                  min(self.max_angular * 0.4,
                                      angle_error * 1.5))

        self.cmd_pub.publish(twist)

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ==============================================================
    # 路径 & Checkpoint
    # ==============================================================
    def set_waypoints(self, waypoints, area_names=None):
        self.waypoints = waypoints
        if area_names:
            self.area_names = area_names
        self.current_index = 0
        self.get_logger().info(f'已加载 {len(waypoints)} 个路径点')

    def save_checkpoint(self):
        data = {
            'completed_areas': self.completed_areas,
            'last_index': self.current_index,
            'position': list(self.robot_pose),
        }
        try:
            with open(self.checkpoint_file, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            self.get_logger().error(f'Checkpoint 保存失败: {e}')

    def load_checkpoint(self):
        if not os.path.exists(self.checkpoint_file):
            return None
        try:
            with open(self.checkpoint_file, 'r') as f:
                return json.load(f)
        except Exception:
            return None

    def clear_checkpoint(self):
        if os.path.exists(self.checkpoint_file):
            os.remove(self.checkpoint_file)

    # ==============================================================
    # 可视化
    # ==============================================================
    def publish_coverage_viz(self):
        markers = MarkerArray()

        # 已覆盖点（灰色）
        covered = Marker()
        covered.header.frame_id = self.frame_id
        covered.header.stamp = self.get_clock().now().to_msg()
        covered.ns = 'covered_area'
        covered.id = 0
        covered.type = Marker.POINTS
        covered.action = Marker.ADD
        covered.pose.orientation.w = 1.0
        covered.scale.x = 0.15
        covered.scale.y = 0.15
        covered.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5)

        for x, y in self.covered_positions:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.01
            covered.points.append(p)
            if len(covered.points) >= 1000:
                break
        markers.markers.append(covered)

        # 当前目标点（红色球）
        if self.executing and self.current_index < len(self.waypoints):
            target = self.waypoints[self.current_index]
            goal = Marker()
            goal.header.frame_id = self.frame_id
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.ns = 'current_goal'
            goal.id = 0
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose.position.x = float(target[0])
            goal.pose.position.y = float(target[1])
            goal.pose.position.z = 0.0
            goal.scale.x = 0.3
            goal.scale.y = 0.3
            goal.scale.z = 0.3
            goal.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            markers.markers.append(goal)

        self.marker_pub.publish(markers)

        # ★ 发布覆盖率统计信息（供 Web 前端显示）
        self.publish_statistics()

    def publish_statistics(self):
        """发布覆盖率统计到 /coverage/statistics 话题"""
        total = max(self.total_waypoints, 1)
        covered = min(self.current_index, total)
        pct = (covered / total) * 100.0

        stat = {
            'coverage_percent': round(pct, 1),
            'executing': self.executing,
            'covered': covered,
            'total': total,
            'remaining': total - covered,
            'mode': 'executing' if self.executing else 'idle',
        }

        msg = String()
        msg.data = json.dumps(stat)
        self.stat_pub.publish(msg)


def main():
    rclpy.init()
    node = MultiAreaExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
