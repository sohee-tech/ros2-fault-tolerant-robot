"""Odometry velocity-spike injector.

Relays /odom_raw -> /odom. When a fault is active, twist.linear.x of the relayed
message is overwritten with `spike_vx` (default 2.5 m/s; the real speed is ~0.2 m/s).

Services:
  /odom_fault/enable   std_srvs/Trigger   continuous spike on every sample
  /odom_fault/disable  std_srvs/Trigger
  /odom_fault/set      std_srvs/SetBool
  /odom_fault/spike    std_srvs/Trigger   corrupt exactly ONE next sample
Topic:
  /odom_fault/active   std_msgs/Bool (transient local) - continuous fault flag
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class OdomFaultInjector(Node):
    def __init__(self):
        super().__init__('odom_fault_injector')
        in_topic = self.declare_parameter('input_topic', 'odom_raw').value
        out_topic = self.declare_parameter('output_topic', 'odom').value
        self.spike_vx = self.declare_parameter('spike_vx', 2.5).value

        self.fault_active = False
        self.pending_spikes = 0
        self.corrupted = 0

        self.pub_odom = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self.on_odom, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_active = self.create_publisher(Bool, 'odom_fault/active', latched)

        self.create_service(SetBool, 'odom_fault/set', self.on_set)
        self.create_service(Trigger, 'odom_fault/enable', lambda req, res: self.on_trigger(True, res))
        self.create_service(Trigger, 'odom_fault/disable', lambda req, res: self.on_trigger(False, res))
        self.create_service(Trigger, 'odom_fault/spike', self.on_spike)

        self.pub_active.publish(Bool(data=False))
        self.get_logger().info(f'Relaying /{in_topic} -> /{out_topic} (spike_vx={self.spike_vx:.2f} m/s)')

    def on_odom(self, msg):
        if self.fault_active or self.pending_spikes > 0:
            real = msg.twist.twist.linear.x
            msg.twist.twist.linear.x = self.spike_vx
            self.corrupted += 1
            if self.pending_spikes > 0:
                self.pending_spikes -= 1
                self.get_logger().warn(
                    f'Single odom spike injected: vx {real:.3f} -> {self.spike_vx:.2f} m/s')
        self.pub_odom.publish(msg)

    def set_fault(self, active):
        if active == self.fault_active:
            return f'odom fault already {"ENABLED" if active else "DISABLED"}'
        self.fault_active = active
        self.pub_active.publish(Bool(data=active))
        if active:
            self.corrupted = 0
            text = f'Odom fault ENABLED: twist.linear.x -> {self.spike_vx:.2f} m/s on every sample'
        else:
            text = f'Odom fault DISABLED: relay restored ({self.corrupted} samples corrupted)'
        self.get_logger().warn(text)
        return text

    def on_set(self, req, res):
        res.message = self.set_fault(req.data)
        res.success = True
        return res

    def on_trigger(self, active, res):
        res.message = self.set_fault(active)
        res.success = True
        return res

    def on_spike(self, _req, res):
        self.pending_spikes += 1
        res.message = f'single spike armed ({self.spike_vx:.2f} m/s on next sample)'
        res.success = True
        return res


def main():
    rclpy.init()
    node = OdomFaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
