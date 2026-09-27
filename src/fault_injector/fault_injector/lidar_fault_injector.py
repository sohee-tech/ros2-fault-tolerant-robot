"""LiDAR dropout injector.

Relays /scan_raw -> /scan. While the fault is active, /scan is not published at all.

Services:
  /lidar_fault/set      std_srvs/SetBool   (data=true: enable, false: disable)
  /lidar_fault/enable   std_srvs/Trigger
  /lidar_fault/disable  std_srvs/Trigger
Topic:
  /lidar_fault/active   std_msgs/Bool (transient local) - current fault flag
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class LidarFaultInjector(Node):
    def __init__(self):
        super().__init__('lidar_fault_injector')
        self.declare_parameter('input_topic', 'scan_raw')
        self.declare_parameter('output_topic', 'scan')
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.fault_active = False
        self.relayed = 0
        self.dropped = 0

        self.pub_scan = self.create_publisher(LaserScan, out_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, in_topic, self.on_scan, qos_profile_sensor_data)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_active = self.create_publisher(Bool, 'lidar_fault/active', latched)

        self.create_service(SetBool, 'lidar_fault/set', self.on_set)
        self.create_service(Trigger, 'lidar_fault/enable', lambda req, res: self.on_trigger(True, res))
        self.create_service(Trigger, 'lidar_fault/disable', lambda req, res: self.on_trigger(False, res))

        self.publish_active()
        self.get_logger().info(f'Relaying /{in_topic} -> /{out_topic} (fault inactive)')

    def on_scan(self, msg):
        if self.fault_active:
            self.dropped += 1
            return
        self.relayed += 1
        self.pub_scan.publish(msg)

    def set_fault(self, active):
        if active == self.fault_active:
            return f'fault already {"ENABLED" if active else "DISABLED"}'
        self.fault_active = active
        self.publish_active()
        if active:
            self.dropped = 0
            text = 'LiDAR fault ENABLED: /scan publishing stopped'
        else:
            text = f'LiDAR fault DISABLED: /scan relay resumed (dropped {self.dropped} scans)'
        self.get_logger().warn(text)
        return text

    def publish_active(self):
        self.pub_active.publish(Bool(data=self.fault_active))

    def on_set(self, req, res):
        res.message = self.set_fault(req.data)
        res.success = True
        return res

    def on_trigger(self, active, res):
        res.message = self.set_fault(active)
        res.success = True
        return res


def main():
    rclpy.init()
    node = LidarFaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
