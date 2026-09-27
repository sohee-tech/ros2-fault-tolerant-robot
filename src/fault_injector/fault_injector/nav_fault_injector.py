"""Crash-loop switch for the navigation command source.

The crash-loop flag must survive restarts of nav_command_source, so it lives in
this long-running node and is published latched on /nav_fault/crash_loop.

Services (std_srvs/Trigger):
  /nav_fault/crash_loop_enable   every (re)started nav_command_source exits shortly after start
  /nav_fault/crash_loop_disable  the next restart stays alive
(A single crash is requested on nav_command_source itself: /nav_fault/crash.)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class NavFaultInjector(Node):
    def __init__(self):
        super().__init__('nav_fault_injector')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Bool, 'nav_fault/crash_loop', latched)
        self.create_service(Trigger, 'nav_fault/crash_loop_enable', lambda req, res: self.set(True, res))
        self.create_service(Trigger, 'nav_fault/crash_loop_disable', lambda req, res: self.set(False, res))
        self.pub.publish(Bool(data=False))

    def set(self, on, res):
        self.pub.publish(Bool(data=on))
        res.success = True
        res.message = f'nav crash loop {"ENABLED" if on else "DISABLED"}'
        self.get_logger().warn(res.message)
        return res


def main():
    rclpy.init()
    node = NavFaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
