"""Simulation-only command arbiter for teleop and planned execution."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .command_mux import CommandMux, MuxCommand


class SimulationCommandMuxNode(Node):
    """Publish one private simulation command with an explicit owner."""

    def __init__(self):
        super().__init__('simulation_cmd_mux')
        self.declare_parameter('teleop_topic', '/simulation/teleop_cmd_vel')
        self.declare_parameter('plan_topic', '/simulation/plan_cmd_vel')
        self.declare_parameter('output_topic', '/simulation/cmd_vel')
        self.declare_parameter('statistics_topic', '/coverage/statistics')
        self.declare_parameter('command_timeout', 0.4)

        teleop_topic = str(self.get_parameter('teleop_topic').value)
        plan_topic = str(self.get_parameter('plan_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        statistics_topic = str(self.get_parameter('statistics_topic').value)
        for name, topic in (
            ('teleop_topic', teleop_topic),
            ('plan_topic', plan_topic),
            ('output_topic', output_topic),
        ):
            if not topic.startswith('/simulation/'):
                raise ValueError(
                    f'{name} must remain under /simulation/: {topic}')
        timeout = float(self.get_parameter('command_timeout').value)
        self.mux = CommandMux(timeout=timeout)

        self.teleop_sub = self.create_subscription(
            Twist, teleop_topic, self.teleop_callback, 10)
        self.plan_sub = self.create_subscription(
            Twist, plan_topic, self.plan_callback, 10)
        self.statistics_sub = self.create_subscription(
            String, statistics_topic, self.statistics_callback, 10)
        self.command_pub = self.create_publisher(Twist, output_topic, 10)
        self.timer = self.create_timer(0.05, self.publish_command)
        self.get_logger().info(
            f'simulation command mux: {teleop_topic} + {plan_topic} '
            f'-> {output_topic}; timeout={timeout:.2f}s')

    def teleop_callback(self, msg):
        self.mux.accept_teleop(
            MuxCommand(msg.linear.x, msg.angular.z), time.monotonic())

    def plan_callback(self, msg):
        self.mux.accept_plan(
            MuxCommand(msg.linear.x, msg.angular.z), time.monotonic())

    def statistics_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.mux.set_plan_active(data.get('executing') is True)
        except (TypeError, json.JSONDecodeError):
            self.mux.set_plan_active(False)

    def publish_command(self):
        command = self.mux.output(time.monotonic())
        msg = Twist()
        msg.linear.x = command.linear_x
        msg.angular.z = command.angular_z
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationCommandMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.mux.set_plan_active(False)
            node.mux.accept_teleop(MuxCommand(), time.monotonic())
            node.publish_command()
            node.destroy_node()
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass


if __name__ == '__main__':
    main()
