#!/usr/bin/env python3
"""Simulation-only teleoperation with a dead-man and timeout gate."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from .teleop import TeleopCommand, TeleopGate


class SimulationTeleopNode(Node):
    """Forward fresh Web teleop input to the simulation command topic."""

    def __init__(self):
        super().__init__('simulation_teleop')
        self.declare_parameter('input_topic', '/teleop/cmd_vel')
        self.declare_parameter('output_topic', '/simulation/teleop_cmd_vel')
        self.declare_parameter('allowed_topic', '/mission/capture/drive_allowed')
        self.declare_parameter('command_timeout', 0.4)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.2)

        self.gate = TeleopGate(
            timeout=float(self.get_parameter('command_timeout').value),
            max_linear=float(self.get_parameter('max_linear_speed').value),
            max_angular=float(self.get_parameter('max_angular_speed').value),
        )
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        allowed_topic = self.get_parameter('allowed_topic').value

        self.command_sub = self.create_subscription(
            Twist, input_topic, self.command_callback, 10)
        self.allowed_sub = self.create_subscription(
            Bool, allowed_topic, self.allowed_callback, 10)
        self.command_pub = self.create_publisher(Twist, output_topic, 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info(
            f'simulation teleop: {input_topic} -> {output_topic}; '
            f'timeout={self.gate.timeout:.2f}s')

    def command_callback(self, msg):
        self.gate.accept(
            TeleopCommand(msg.linear.x, msg.angular.z),
            time.monotonic(),
        )

    def allowed_callback(self, msg):
        self.gate.set_allowed(msg.data)

    def publish_command(self):
        command = self.gate.output(time.monotonic())
        msg = Twist()
        msg.linear.x = command.linear_x
        msg.angular.z = command.angular_z
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.gate.set_allowed(False)
            node.publish_command()
            node.destroy_node()
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass


if __name__ == '__main__':
    main()
