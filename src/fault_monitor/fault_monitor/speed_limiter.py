"""Degradation speed limiter in front of twist_mux.

/cmd_vel_nav -> (clamp |linear.x| to /health/speed_limit) -> /cmd_vel_nav_limited -> twist_mux

/health/speed_limit (std_msgs/Float64, transient local) is set by health_monitor:
a value <= 0 means "no limit". angular.z is scaled by the same factor as linear.x so the
robot keeps the same path curvature (slower, but on the same track). Messages are relayed one-to-one, so when the nav
source goes silent this node goes silent too and twist_mux's timeout still applies.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class SpeedLimiter(Node):
    def __init__(self):
        super().__init__('speed_limiter')
        self.limit = 0.0
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav_limited', 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.on_cmd, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64, 'health/speed_limit', self.on_limit, latched)

    def on_limit(self, msg):
        if msg.data != self.limit:
            self.get_logger().warn(
                f'linear speed limit: {msg.data:.2f} m/s' if msg.data > 0 else 'linear speed limit released')
        self.limit = msg.data

    def on_cmd(self, msg):
        if self.limit > 0.0 and abs(msg.linear.x) > self.limit:
            scale = self.limit / abs(msg.linear.x)
            msg.linear.x *= scale
            msg.angular.z *= scale
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SpeedLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
