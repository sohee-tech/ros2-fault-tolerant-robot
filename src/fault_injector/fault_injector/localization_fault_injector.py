"""Localization fault injector (test tool), same relay pattern as lidar_fault_injector.

Relays /amcl_pose_raw (AMCL output, remapped in nav2_warehouse.launch.py) -> /amcl_pose.

Services (std_srvs/Trigger):
  /localization_fault/drop     /amcl_pose is no longer published
  /localization_fault/invalid  covariance is replaced by a NaN / huge value
  /localization_fault/disable  normal relay; the newest pose is sent at once
Topic:
  /localization_fault/mode     std_msgs/String (transient local): NONE | DROP | INVALID
"""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class LocalizationFaultInjector(Node):
    def __init__(self):
        super().__init__('localization_fault_injector')
        self.mode = 'NONE'
        self.last = None
        pose_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', pose_qos)
        self.pub_mode = self.create_publisher(String, 'localization_fault/mode', latched)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose_raw', self.on_pose, pose_qos)
        for service, mode in (('drop', 'DROP'), ('invalid', 'INVALID'), ('disable', 'NONE')):
            self.create_service(Trigger, f'localization_fault/{service}',
                                lambda _q, r, m=mode: self.on_mode(m, r))
        self.pub_mode.publish(String(data=self.mode))

    def corrupt(self, msg):
        cov = list(msg.pose.covariance)
        cov[0], cov[7], cov[35] = float('nan'), 25.0, 25.0
        msg.pose.covariance = cov
        return msg

    def on_pose(self, msg):
        self.last = msg
        if self.mode == 'NONE':
            self.pub.publish(msg)
        elif self.mode == 'INVALID':
            self.pub.publish(self.corrupt(msg))

    def on_mode(self, mode, res):
        self.mode = mode
        self.pub_mode.publish(String(data=mode))
        if mode == 'NONE' and self.last is not None:
            self.pub.publish(self.last)
        res.success, res.message = True, f'localization fault mode {mode}'
        self.get_logger().warn(res.message)
        return res


def main():
    rclpy.init()
    node = LocalizationFaultInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
