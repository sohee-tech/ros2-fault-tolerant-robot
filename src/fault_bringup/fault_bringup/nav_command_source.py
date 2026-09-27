"""Navigation command source (the normal driving command generator).

Publishes:
  /cmd_vel_nav_raw  geometry_msgs/Twist  forward `speed` at `rate` Hz while driving is requested
  /nav/heartbeat    std_msgs/UInt64      at `rate` Hz, always; data = this process' PID
                                         (lets the monitor detect restarts)
Subscribes:
  /nav_source/active_request  std_msgs/Bool (transient local) - drive on/off request from the
                              operator/test. Latched, so a respawned process resumes the mission.
  /nav_fault/crash_loop       std_msgs/Bool (transient local) - from nav_fault_injector; when true
                              this process exits `crash_loop_uptime` s after starting.
Service:
  /nav_fault/crash  std_srvs/Trigger - the process really terminates (os._exit) right after
                    the response is sent. launch respawns it.
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt64
from std_srvs.srv import Trigger


class NavCommandSource(Node):
    def __init__(self):
        super().__init__('nav_command_source')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.speed = p('speed', 0.20)
        rate = p('rate', 10.0)
        self.crash_loop_uptime = p('crash_loop_uptime', 0.5)
        self.pid = os.getpid()
        self.active = False
        self.exit_timer = None

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_nav_raw', 10)
        self.pub_hb = self.create_publisher(UInt64, 'nav/heartbeat', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'nav_source/active_request', self.on_active, latched)
        self.create_subscription(Bool, 'nav_fault/crash_loop', self.on_crash_loop, latched)
        self.create_service(Trigger, 'nav_fault/crash', self.on_crash)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'started, pid={self.pid}, speed={self.speed:.2f} m/s')

    def tick(self):
        self.pub_hb.publish(UInt64(data=self.pid))
        if self.active:
            cmd = Twist()
            cmd.linear.x = self.speed
            self.pub_cmd.publish(cmd)

    def on_active(self, msg):
        if msg.data != self.active:
            self.get_logger().info(f'driving {"requested" if msg.data else "stopped"}')
        self.active = msg.data

    def on_crash(self, _req, res):
        self.schedule_exit(0.1, 'crash requested via /nav_fault/crash')
        res.success = True
        res.message = f'nav_command_source pid={self.pid} will terminate now'
        return res

    def on_crash_loop(self, msg):
        if msg.data:
            self.schedule_exit(self.crash_loop_uptime, 'crash loop active')
        elif self.exit_timer is not None:
            self.exit_timer.cancel()
            self.exit_timer = None
            self.get_logger().info('crash loop disabled, staying alive')

    def schedule_exit(self, delay, reason):
        if self.exit_timer is not None:
            self.exit_timer.cancel()
        self.get_logger().error(f'{reason}: pid={self.pid} exits in {delay:.1f} s')
        self.exit_timer = self.create_timer(delay, self.crash)

    def crash(self):
        # Real process failure: no cleanup, non-zero exit code (launch respawns it)
        os._exit(1)


def main():
    rclpy.init()
    node = NavCommandSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
