"""Lowest-priority default command source for twist_mux.

Publishes a zero Twist on /cmd_vel_idle_stop continuously so that when every
higher-priority source (e.g. /cmd_vel_nav) times out, twist_mux outputs zero
instead of leaving the last non-zero command active in the diff drive.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class IdleStop(Node):
    def __init__(self):
        super().__init__('idle_stop')
        rate = self.declare_parameter('rate', 20.0).value
        self.pub = self.create_publisher(Twist, 'cmd_vel_idle_stop', 10)
        self.create_timer(1.0 / rate, lambda: self.pub.publish(Twist()))
        self.get_logger().info(f'Publishing zero Twist on /cmd_vel_idle_stop at {rate:.0f} Hz')


def main():
    rclpy.init()
    node = IdleStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
