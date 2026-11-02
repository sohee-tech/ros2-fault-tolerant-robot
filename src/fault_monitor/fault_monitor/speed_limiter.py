"""Degradation speed limiter + Safe Resume permit in front of twist_mux.

/cmd_vel_nav -> (permit check) -> (clamp |linear.x| to /health/speed_limit) -> /cmd_vel_nav_limited -> twist_mux

/health/speed_limit (std_msgs/Float64, transient local) is set by health_monitor:
a value <= 0 means "no limit". angular.z is scaled by the same factor as linear.x so the
robot keeps the same path curvature (slower, but on the same track). Messages are relayed one-to-one, so when the nav
source goes silent this node goes silent too and twist_mux's timeout still applies.

require_permit:=true (Nav2 system): a command only passes while the last /safe_resume/approved was true
and is younger than permit_timeout; otherwise it is replaced by zero. No or old permit (gate not started,
crashed, hung) = denied. If this relay itself dies, twist_mux times the channel out and idle_stop outputs 0.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64


class SpeedLimiter(Node):
    def __init__(self):
        super().__init__('speed_limiter')
        self.limit = 0.0
        self.require_permit = self.declare_parameter('require_permit', False).value
        self.permit_timeout = self.declare_parameter('permit_timeout', 0.5).value
        self.permit_value = False
        self.t_permit = None
        self.open = False
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav_limited', 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.on_cmd, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64, 'health/speed_limit', self.on_limit, latched)
        if self.require_permit:
            self.create_subscription(Bool, 'safe_resume/approved', self.on_permit, 10)
            self.create_timer(0.05, self.update_permit)

    def on_limit(self, msg):
        if msg.data != self.limit:
            self.get_logger().warn(
                f'linear speed limit: {msg.data:.2f} m/s' if msg.data > 0 else 'linear speed limit released')
        self.limit = msg.data

    def permitted(self):
        if self.t_permit is None or not self.permit_value:
            return False
        return self.get_clock().now().nanoseconds * 1e-9 - self.t_permit <= self.permit_timeout

    def on_permit(self, msg):
        self.permit_value = msg.data
        self.t_permit = self.get_clock().now().nanoseconds * 1e-9
        self.update_permit()

    def update_permit(self):
        ok = self.permitted()
        if ok == self.open:
            return
        self.open = ok
        if ok:
            self.get_logger().info('Safe Resume permit granted: Nav2 commands pass')
        else:
            self.pub.publish(Twist())   # stop now, do not wait for the next Nav2 command
            self.get_logger().warn('Safe Resume permit denied: Nav2 commands blocked')

    def on_cmd(self, msg):
        if self.require_permit and not self.permitted():
            self.pub.publish(Twist())
            return
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
