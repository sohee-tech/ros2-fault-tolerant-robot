"""Control latency injector.

/cmd_vel_nav_raw -> (hold for injected delay) -> /cmd_vel_nav

Every command is stamped with a monotonic receive time and released once
`receive + delay` has passed. The latency actually experienced by each forwarded
command (publish time - receive time) is published on /control/latency_sec.

Stale-command policy: when the delay is disabled, every queued (old) command is
dropped, so nothing bursts out; the next fresh command passes immediately. The
queue is also bounded (`max_queue`, oldest dropped first).

Services (std_srvs/Trigger):
  /control_delay/degraded  delay = degraded_delay_sec (default 0.30 s)
  /control_delay/critical  delay = critical_delay_sec (default 0.70 s)
  /control_delay/disable   delay = 0, queue flushed
Topics:
  /control/latency_sec         std_msgs/Float64  measured latency per forwarded command
  /control/delay_active        std_msgs/Bool     (transient local)
  /control/delay_mode          std_msgs/String   NONE | DEGRADED | CRITICAL (transient local)
  /control/injected_delay_sec  std_msgs/Float64  target delay (transient local)
"""
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger


class ControlDelayInjector(Node):
    def __init__(self):
        super().__init__('control_delay_injector')
        p = lambda name, default: self.declare_parameter(name, default).value
        in_topic = p('input_topic', 'cmd_vel_nav_raw')
        out_topic = p('output_topic', 'cmd_vel_nav')
        self.delays = {'NONE': 0.0,
                       'DEGRADED': p('degraded_delay_sec', 0.30),
                       'CRITICAL': p('critical_delay_sec', 0.70)}
        self.queue = deque(maxlen=p('max_queue', 50))   # (receive_monotonic, Twist)
        release_rate = p('release_rate', 200.0)

        self.mode = 'NONE'
        self.delay = 0.0

        self.pub_cmd = self.create_publisher(Twist, out_topic, 10)
        self.pub_latency = self.create_publisher(Float64, 'control/latency_sec', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_active = self.create_publisher(Bool, 'control/delay_active', latched)
        self.pub_mode = self.create_publisher(String, 'control/delay_mode', latched)
        self.pub_target = self.create_publisher(Float64, 'control/injected_delay_sec', latched)
        self.create_subscription(Twist, in_topic, self.on_cmd, 10)

        for mode in ('DEGRADED', 'CRITICAL'):
            self.create_service(Trigger, f'control_delay/{mode.lower()}',
                                lambda req, res, m=mode: self.on_mode(m, res))
        self.create_service(Trigger, 'control_delay/disable', lambda req, res: self.on_mode('NONE', res))

        self.create_timer(1.0 / release_rate, self.release)
        self.publish_mode()
        self.get_logger().info(
            f'Relaying /{in_topic} -> /{out_topic} (degraded {self.delays["DEGRADED"]:.2f} s, '
            f'critical {self.delays["CRITICAL"]:.2f} s)')

    def on_cmd(self, msg):
        now = time.monotonic()
        if self.delay <= 0.0:
            self.forward(msg, now)
        else:
            self.queue.append((now, msg))

    def release(self):
        now = time.monotonic()
        while self.queue and now - self.queue[0][0] >= self.delay:
            recv, msg = self.queue.popleft()
            self.forward(msg, recv)

    def forward(self, msg, recv):
        self.pub_cmd.publish(msg)
        self.pub_latency.publish(Float64(data=time.monotonic() - recv))

    def on_mode(self, mode, res):
        dropped = 0
        if mode == 'NONE':
            dropped = len(self.queue)
            self.queue.clear()   # never release stale commands after recovery
        self.mode = mode
        self.delay = self.delays[mode]
        self.publish_mode()
        text = (f'control delay {mode}: {self.delay * 1000:.0f} ms'
                + (f' (dropped {dropped} stale queued commands)' if mode == 'NONE' else ''))
        self.get_logger().warn(text)
        res.success = True
        res.message = text
        return res

    def publish_mode(self):
        self.pub_active.publish(Bool(data=self.delay > 0.0))
        self.pub_mode.publish(String(data=self.mode))
        self.pub_target.publish(Float64(data=self.delay))


def main():
    rclpy.init()
    node = ControlDelayInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
