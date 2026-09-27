"""LiDAR health monitor.

Tracks the age of the last /scan message (receive time, node clock = sim time)
and drives the state machine NORMAL -> WARNING -> DEGRADED -> CRITICAL.

- Escalation: a higher level must be observed for `hold_ticks` consecutive ticks.
- Recovery: only after scans are healthy (age <= warn threshold) continuously
  for `recovery_time` seconds does the state return to NORMAL.
- CRITICAL: zero Twist is published on /cmd_vel_safety every tick (safe stop).
  twist_mux gives it priority over /cmd_vel_nav. Nothing is published otherwise.
"""
import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String

NORMAL, WARNING, DEGRADED, CRITICAL = range(4)
NAMES = ['NORMAL', 'WARNING', 'DEGRADED', 'CRITICAL']


class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        self.warn_age = self.declare_parameter('warning_age', 0.3).value
        self.degraded_age = self.declare_parameter('degraded_age', 0.7).value
        self.critical_age = self.declare_parameter('critical_age', 1.0).value
        self.hold_ticks = self.declare_parameter('hold_ticks', 2).value
        self.recovery_time = self.declare_parameter('recovery_time', 2.0).value
        rate = self.declare_parameter('rate', 20.0).value
        log_dir = os.path.expanduser(
            self.declare_parameter('log_dir', '~/capstone_fault_ws/logs').value)

        self.state = NORMAL
        self.last_scan = None
        self.pending_ticks = 0
        self.healthy_since = None
        self.fault_enabled = False
        self.odom_vx = float('nan')
        self.nav_vx = float('nan')
        self.out_vx = float('nan')

        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.on_nav, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_out, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'lidar_fault/active', self.on_fault, latched)

        self.pub_safety = self.create_publisher(Twist, 'cmd_vel_safety', 10)
        self.pub_state = self.create_publisher(String, 'health/state', latched)
        self.pub_age = self.create_publisher(Float64, 'health/scan_age', 10)

        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(
            log_dir, f'fault1_lidar_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['timestamp', 'sim_time', 'state', 'scan_age',
                           'fault_enabled', 'odom_linear_x', 'safety_stop_active',
                           'cmd_vel_nav_x', 'cmd_vel_out_x'])

        self.create_timer(1.0 / rate, self.tick)
        self.publish_state()
        self.get_logger().info(f'Health monitor started, CSV: {self.csv_path}')

    # --- callbacks -------------------------------------------------------
    def on_scan(self, _msg):
        self.last_scan = self.now()

    def on_odom(self, msg):
        self.odom_vx = msg.twist.twist.linear.x

    def on_nav(self, msg):
        self.nav_vx = msg.linear.x

    def on_out(self, msg):
        self.out_vx = msg.linear.x

    def on_fault(self, msg):
        self.fault_enabled = msg.data

    # --- state machine ---------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def level_for(self, age):
        if age > self.critical_age:
            return CRITICAL
        if age > self.degraded_age:
            return DEGRADED
        if age > self.warn_age:
            return WARNING
        return NORMAL

    def tick(self):
        if self.last_scan is None:
            return  # no scan yet (sim starting)
        now = self.now()
        age = now - self.last_scan
        raw = self.level_for(age)

        if raw > self.state:
            self.healthy_since = None
            self.pending_ticks += 1
            if self.pending_ticks >= self.hold_ticks:
                self.change_state(raw, age)
        else:
            self.pending_ticks = 0
            if self.state != NORMAL:
                if raw == NORMAL:
                    if self.healthy_since is None:
                        self.healthy_since = now
                        self.get_logger().info(
                            f'[{NAMES[self.state]}] LiDAR scan resumed, recovering '
                            f'(need {self.recovery_time:.1f} s healthy)')
                    elif now - self.healthy_since >= self.recovery_time:
                        self.change_state(NORMAL, age)
                else:
                    self.healthy_since = None

        safe_stop = self.state == CRITICAL
        if safe_stop:
            self.pub_safety.publish(Twist())

        self.pub_age.publish(Float64(data=age))
        self.csv.writerow([f'{time.time():.3f}', f'{now:.3f}', NAMES[self.state], f'{age:.3f}',
                           int(self.fault_enabled), f'{self.odom_vx:.4f}', int(safe_stop),
                           f'{self.nav_vx:.3f}', f'{self.out_vx:.3f}'])
        self.csv_file.flush()

    def change_state(self, new, age):
        self.state = new
        self.publish_state()  # latched; published on change only
        self.pending_ticks = 0
        self.healthy_since = None
        log = self.get_logger()
        if new == NORMAL:
            log.info(f'[NORMAL] scan_age={age:.2f}')
        elif new == WARNING:
            log.warn(f'[WARNING] LiDAR delayed: {age:.2f} s')
        elif new == DEGRADED:
            log.warn(f'[DEGRADED] LiDAR unavailable: {age:.2f} s')
        else:
            log.error(f'[CRITICAL] LiDAR dropout: {age:.2f} s -> SAFE STOP')

    def publish_state(self):
        self.pub_state.publish(String(data=NAMES[self.state]))

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
