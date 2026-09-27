"""System health monitor (LiDAR + odometry + control latency + navigation liveness).

LiDAR (Fault 1): age of the last /scan message (receive time, node clock = sim time)
  drives NORMAL -> WARNING -> DEGRADED -> CRITICAL.
  - Escalation: a higher level must be observed for `hold_ticks` consecutive ticks.
  - Recovery: scans healthy (age <= warning_age) for `recovery_time` s -> NORMAL.

Odometry (Fault 2): every /odom sample gets a plausibility check.
  - over_speed:   |vx| > odom_max_speed (physical bound)
  - accel:        |vx - last valid vx| / dt > odom_max_accel (physical bound)
  - cmd_mismatch: |vx - final /cmd_vel x| > odom_cmd_error for odom_cmd_error_samples
                  consecutive samples (ignores normal start/stop transients)
  An implausible sample is rejected (not forwarded to /odom_validated).
  `odom_degrade_samples` consecutive rejects -> odom DEGRADED; valid samples for
  `odom_recovery_time` s -> NORMAL.

Control latency (Fault 3): /control/latency_sec (measured by control_delay_injector
  per forwarded nav command) drives NORMAL -> WARNING -> DEGRADED -> CRITICAL.
  - Escalation: one level per sample; WARNING/DEGRADED need `ctrl_samples`,
    CRITICAL needs `ctrl_critical_samples` consecutive samples at/above the threshold.
  - Recovery: latency < ctrl_warning_latency for `ctrl_recovery_time` s -> NORMAL.

Navigation liveness (Fault 4): age of the last /nav/heartbeat (UInt64 = PID of
  nav_command_source) drives NORMAL -> WARNING -> DEGRADED -> CRITICAL, same rules as
  LiDAR (`nav_hold_ticks`, `nav_recovery_time`). A PID change counts as a restart.

System state = worst of LiDAR, odometry, control latency and navigation liveness state.
  - >= DEGRADED: /health/speed_limit = degraded_speed_limit (speed_limiter clamps nav)
  - CRITICAL:    zero Twist on /cmd_vel_safety every tick (twist_mux priority -> stop)
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
from std_msgs.msg import Bool, Float64, String, UInt64

NORMAL, WARNING, DEGRADED, CRITICAL = range(4)
NAMES = ['NORMAL', 'WARNING', 'DEGRADED', 'CRITICAL']
NAN = float('nan')


class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        p = lambda name, default: self.declare_parameter(name, default).value
        # LiDAR
        self.warn_age = p('warning_age', 0.3)
        self.degraded_age = p('degraded_age', 0.7)
        self.critical_age = p('critical_age', 1.0)
        self.hold_ticks = p('hold_ticks', 2)
        self.recovery_time = p('recovery_time', 2.0)
        # Odometry
        self.odom_max_speed = p('odom_max_speed', 0.5)
        self.odom_max_accel = p('odom_max_accel', 5.0)
        self.odom_cmd_error = p('odom_cmd_error', 0.3)
        self.odom_cmd_error_samples = p('odom_cmd_error_samples', 5)
        self.odom_degrade_samples = p('odom_degrade_samples', 5)
        self.odom_recovery_time = p('odom_recovery_time', 2.0)
        # Control latency
        self.ctrl_thresholds = [0.0, p('ctrl_warning_latency', 0.10),
                                p('ctrl_degraded_latency', 0.25), p('ctrl_critical_latency', 0.50)]
        self.ctrl_samples = p('ctrl_samples', 3)
        self.ctrl_critical_samples = p('ctrl_critical_samples', 2)
        self.ctrl_recovery_time = p('ctrl_recovery_time', 2.0)
        # Navigation liveness
        self.nav_thresholds = [0.0, p('nav_warning_age', 0.3), p('nav_degraded_age', 0.7),
                               p('nav_critical_age', 1.2)]
        self.nav_hold_ticks = p('nav_hold_ticks', 2)
        self.nav_recovery_time = p('nav_recovery_time', 2.0)
        # Response
        self.degraded_speed_limit = p('degraded_speed_limit', 0.10)
        rate = p('rate', 20.0)
        log_dir = os.path.expanduser(p('log_dir', '~/capstone_fault_ws/logs'))

        # LiDAR state
        self.lidar_state = NORMAL
        self.last_scan = None
        self.pending_ticks = 0
        self.healthy_since = None
        self.lidar_fault = False
        # Odometry state
        self.odom_state = NORMAL
        self.last_valid = None            # (stamp, vx) of last accepted sample
        self.mismatch_count = 0
        self.invalid_streak = 0
        self.odom_valid_since = None
        self.odom_invalid_total = 0
        self.tick_invalid = False         # any invalid sample since last CSV row
        self.tick_errors = set()
        self.odom_fault = False
        # Control latency state
        self.control_state = NORMAL
        self.ctrl_counts = [0, 0, 0, 0]   # consecutive samples >= threshold of each level
        self.ctrl_healthy_since = None
        self.latency = NAN
        self.delay_mode = 'NONE'
        self.injected_delay = 0.0
        # Navigation liveness state
        self.nav_state = NORMAL
        self.last_hb = None
        self.nav_pending = 0
        self.nav_healthy_since = None
        self.nav_pid = 0
        self.nav_restart_count = 0
        self.nav_age = NAN
        # System
        self.state = NORMAL
        self.speed_limit = 0.0
        self.odom_vx = NAN
        self.odom_raw_vx = NAN
        self.nav_vx = NAN
        self.nav_raw_vx = NAN
        self.out_vx = NAN

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Odometry, 'odom_raw', self.on_odom_raw, 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.on_nav, 10)
        self.create_subscription(Twist, 'cmd_vel_nav_raw', self.on_nav_raw, 10)
        self.create_subscription(Float64, 'control/latency_sec', self.on_latency, 10)
        self.create_subscription(UInt64, 'nav/heartbeat', self.on_heartbeat, 10)
        self.create_subscription(String, 'control/delay_mode',
                                 lambda m: setattr(self, 'delay_mode', m.data), latched)
        self.create_subscription(Float64, 'control/injected_delay_sec',
                                 lambda m: setattr(self, 'injected_delay', m.data), latched)
        self.create_subscription(Twist, 'cmd_vel', self.on_out, 10)
        self.create_subscription(Bool, 'lidar_fault/active', self.on_lidar_fault, latched)
        self.create_subscription(Bool, 'odom_fault/active', self.on_odom_fault, latched)

        self.pub_safety = self.create_publisher(Twist, 'cmd_vel_safety', 10)
        self.pub_state = self.create_publisher(String, 'health/state', latched)
        self.pub_lidar_state = self.create_publisher(String, 'health/lidar_state', latched)
        self.pub_odom_state = self.create_publisher(String, 'health/odom_state', latched)
        self.pub_control_state = self.create_publisher(String, 'health/control_state', latched)
        self.pub_nav_state = self.create_publisher(String, 'health/nav_state', latched)
        self.pub_nav_restarts = self.create_publisher(UInt64, 'health/nav_restart_count', latched)
        self.pub_limit = self.create_publisher(Float64, 'health/speed_limit', latched)
        self.pub_age = self.create_publisher(Float64, 'health/scan_age', 10)
        self.pub_odom_valid = self.create_publisher(Bool, 'health/odom_valid', 10)
        self.pub_odom_validated = self.create_publisher(Odometry, 'odom_validated', 10)

        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(
            log_dir, f'health_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow([
            'timestamp', 'sim_time', 'system_state', 'lidar_state', 'odom_state', 'control_state',
            'nav_state',
            'scan_age', 'lidar_fault_enabled', 'odom_fault_enabled',
            'odom_raw_vx', 'odom_output_vx', 'odom_valid', 'odom_error', 'odom_invalid_total',
            'degraded_speed_limit', 'safety_stop_active', 'cmd_vel_nav_x', 'cmd_vel_out_x',
            'control_latency_sec', 'delay_mode', 'injected_delay_sec', 'cmd_vel_nav_raw_x',
            'nav_heartbeat_age', 'nav_alive', 'nav_restart_count', 'nav_process_pid'])

        self.create_timer(1.0 / rate, self.tick)
        self.pub_state.publish(String(data=NAMES[self.state]))
        self.pub_lidar_state.publish(String(data=NAMES[self.lidar_state]))
        self.pub_odom_state.publish(String(data=NAMES[self.odom_state]))
        self.pub_control_state.publish(String(data=NAMES[self.control_state]))
        self.pub_nav_state.publish(String(data=NAMES[self.nav_state]))
        self.pub_nav_restarts.publish(UInt64(data=0))
        self.pub_limit.publish(Float64(data=self.speed_limit))
        self.get_logger().info(f'Health monitor started, CSV: {self.csv_path}')

    # --- simple callbacks ------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, _msg):
        self.last_scan = self.now()

    def on_odom_raw(self, msg):
        self.odom_raw_vx = msg.twist.twist.linear.x

    def on_nav(self, msg):
        self.nav_vx = msg.linear.x

    def on_nav_raw(self, msg):
        self.nav_raw_vx = msg.linear.x

    def on_out(self, msg):
        self.out_vx = msg.linear.x

    def on_lidar_fault(self, msg):
        self.lidar_fault = msg.data

    def on_odom_fault(self, msg):
        self.odom_fault = msg.data

    # --- odometry plausibility -------------------------------------------
    def check_odom(self, stamp, vx):
        errors = []
        if abs(vx) > self.odom_max_speed:
            errors.append('over_speed')
        if self.last_valid is not None:
            dt = stamp - self.last_valid[0]
            if dt > 1e-6 and abs(vx - self.last_valid[1]) / dt > self.odom_max_accel:
                errors.append('accel')
        if self.out_vx == self.out_vx:  # a command has been seen (not NaN)
            if abs(vx - self.out_vx) > self.odom_cmd_error:
                self.mismatch_count += 1
            else:
                self.mismatch_count = 0
            if self.mismatch_count >= self.odom_cmd_error_samples:
                errors.append('cmd_mismatch')
        return errors

    def on_odom(self, msg):
        vx = msg.twist.twist.linear.x
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.now()
        self.odom_vx = vx
        errors = self.check_odom(stamp, vx)
        valid = not errors
        self.pub_odom_valid.publish(Bool(data=valid))

        if valid:
            self.last_valid = (stamp, vx)
            self.pub_odom_validated.publish(msg)
            if self.invalid_streak:
                self.get_logger().info(
                    f'[ODOM] valid again after {self.invalid_streak} rejected sample(s)')
            self.invalid_streak = 0
            if self.odom_state != NORMAL:
                if self.odom_valid_since is None:
                    self.odom_valid_since = now
                    self.get_logger().info(
                        f'[{NAMES[self.odom_state]}] odometry plausible again, recovering '
                        f'(need {self.odom_recovery_time:.1f} s healthy)')
                elif now - self.odom_valid_since >= self.odom_recovery_time:
                    self.set_odom_state(NORMAL, f'odometry healthy for {self.odom_recovery_time:.1f} s')
            return

        self.odom_invalid_total += 1
        self.invalid_streak += 1
        self.odom_valid_since = None
        self.tick_invalid = True
        self.tick_errors.update(errors)
        ref = self.last_valid[1] if self.last_valid else NAN
        if self.invalid_streak == 1:
            self.get_logger().warn(
                f'[ODOM] sample rejected: vx={vx:.3f} (last valid {ref:.3f}, '
                f'cmd {self.out_vx:.3f}) errors={",".join(errors)}')
        if self.odom_state == NORMAL and self.invalid_streak >= self.odom_degrade_samples:
            self.set_odom_state(DEGRADED, f'{self.invalid_streak} consecutive implausible samples '
                                          f'(vx={vx:.2f}, {",".join(errors)})')

    def set_odom_state(self, new, reason):
        self.odom_state = new
        self.odom_valid_since = None
        self.pub_odom_state.publish(String(data=NAMES[new]))
        # rclpy fixes the severity per call site, so use separate calls
        if new == NORMAL:
            self.get_logger().info(f'[ODOM NORMAL] {reason}')
        else:
            self.get_logger().warn(f'[ODOM {NAMES[new]}] {reason}')
        self.update_system()

    # --- control latency state machine -----------------------------------
    def on_latency(self, msg):
        lat = msg.data
        now = self.now()
        self.latency = lat
        for lvl in (WARNING, DEGRADED, CRITICAL):
            self.ctrl_counts[lvl] = self.ctrl_counts[lvl] + 1 if lat >= self.ctrl_thresholds[lvl] else 0

        nxt = self.control_state + 1
        if nxt <= CRITICAL:
            need = self.ctrl_critical_samples if nxt == CRITICAL else self.ctrl_samples
            if self.ctrl_counts[nxt] >= need:
                self.set_control_state(nxt, lat)
                return
        if self.control_state == NORMAL:
            return
        if lat >= self.ctrl_thresholds[WARNING]:
            self.ctrl_healthy_since = None
        elif self.ctrl_healthy_since is None:
            self.ctrl_healthy_since = now
            self.get_logger().info(
                f'[{NAMES[self.control_state]}] control latency normal again ({lat * 1000:.0f} ms), '
                f'recovering (need {self.ctrl_recovery_time:.1f} s healthy)')
        elif now - self.ctrl_healthy_since >= self.ctrl_recovery_time:
            self.set_control_state(NORMAL, lat)

    def set_control_state(self, new, lat):
        self.control_state = new
        self.ctrl_healthy_since = None
        self.pub_control_state.publish(String(data=NAMES[new]))
        msg = f'control latency {lat * 1000:.0f} ms'
        if new == NORMAL:
            self.get_logger().info(f'[CONTROL NORMAL] {msg}')
        elif new == CRITICAL:
            self.get_logger().error(f'[CONTROL CRITICAL] {msg} -> SAFE STOP')
        else:
            self.get_logger().warn(f'[CONTROL {NAMES[new]}] {msg}')
        self.update_system()

    # --- navigation liveness state machine --------------------------------
    def on_heartbeat(self, msg):
        if self.nav_pid and msg.data != self.nav_pid:
            self.nav_restart_count += 1
            self.pub_nav_restarts.publish(UInt64(data=self.nav_restart_count))
            self.get_logger().warn(
                f'[NAV] nav_command_source restarted: pid {self.nav_pid} -> {msg.data} '
                f'(restart #{self.nav_restart_count})')
        self.nav_pid = msg.data
        self.last_hb = self.now()

    def nav_level(self, age):
        for lvl in (CRITICAL, DEGRADED, WARNING):
            if age >= self.nav_thresholds[lvl]:
                return lvl
        return NORMAL

    def update_nav(self, now, age):
        raw = self.nav_level(age)
        if raw > self.nav_state:
            self.nav_healthy_since = None
            self.nav_pending += 1
            if self.nav_pending >= self.nav_hold_ticks:
                self.set_nav_state(raw, age)
            return
        self.nav_pending = 0
        if self.nav_state == NORMAL:
            return
        if raw != NORMAL:
            self.nav_healthy_since = None
        elif self.nav_healthy_since is None:
            self.nav_healthy_since = now
            self.get_logger().info(
                f'[NAV {NAMES[self.nav_state]}] heartbeat resumed (pid {self.nav_pid}), recovering '
                f'(need {self.nav_recovery_time:.1f} s healthy)')
        elif now - self.nav_healthy_since >= self.nav_recovery_time:
            self.set_nav_state(NORMAL, age)

    def set_nav_state(self, new, age):
        self.nav_state = new
        self.nav_pending = 0
        self.nav_healthy_since = None
        self.pub_nav_state.publish(String(data=NAMES[new]))
        if new == NORMAL:
            self.get_logger().info(f'[NAV NORMAL] heartbeat healthy for {self.nav_recovery_time:.1f} s '
                                   f'(pid {self.nav_pid})')
        elif new == CRITICAL:
            self.get_logger().error(f'[NAV CRITICAL] navigation command source unresponsive: '
                                    f'heartbeat age {age:.2f} s -> SAFE STOP')
        else:
            self.get_logger().warn(f'[NAV {NAMES[new]}] heartbeat age {age:.2f} s')
        self.update_system()

    # --- LiDAR state machine ---------------------------------------------
    def lidar_level(self, age):
        if age > self.critical_age:
            return CRITICAL
        if age > self.degraded_age:
            return DEGRADED
        if age > self.warn_age:
            return WARNING
        return NORMAL

    def update_lidar(self, now, age):
        raw = self.lidar_level(age)
        if raw > self.lidar_state:
            self.healthy_since = None
            self.pending_ticks += 1
            if self.pending_ticks >= self.hold_ticks:
                self.set_lidar_state(raw, age)
            return
        self.pending_ticks = 0
        if self.lidar_state == NORMAL:
            return
        if raw != NORMAL:
            self.healthy_since = None
        elif self.healthy_since is None:
            self.healthy_since = now
            self.get_logger().info(
                f'[{NAMES[self.lidar_state]}] LiDAR scan resumed, recovering '
                f'(need {self.recovery_time:.1f} s healthy)')
        elif now - self.healthy_since >= self.recovery_time:
            self.set_lidar_state(NORMAL, age)

    def set_lidar_state(self, new, age):
        self.lidar_state = new
        self.pending_ticks = 0
        self.healthy_since = None
        self.pub_lidar_state.publish(String(data=NAMES[new]))
        log = self.get_logger()
        if new == NORMAL:
            log.info(f'[NORMAL] scan_age={age:.2f}')
        elif new == WARNING:
            log.warn(f'[WARNING] LiDAR delayed: {age:.2f} s')
        elif new == DEGRADED:
            log.warn(f'[DEGRADED] LiDAR unavailable: {age:.2f} s')
        else:
            log.error(f'[CRITICAL] LiDAR dropout: {age:.2f} s -> SAFE STOP')
        self.update_system()

    # --- system ------------------------------------------------------------
    def update_system(self):
        new = max(self.lidar_state, self.odom_state, self.control_state, self.nav_state)
        if new != self.state:
            self.get_logger().info(
                f'system state {NAMES[self.state]} -> {NAMES[new]} '
                f'(lidar {NAMES[self.lidar_state]}, odom {NAMES[self.odom_state]}, '
                f'control {NAMES[self.control_state]}, nav {NAMES[self.nav_state]})')
            self.state = new
            self.pub_state.publish(String(data=NAMES[new]))  # latched; on change only
        limit = self.degraded_speed_limit if self.state >= DEGRADED else 0.0
        if limit != self.speed_limit:
            self.speed_limit = limit
            self.pub_limit.publish(Float64(data=limit))

    def tick(self):
        if self.last_scan is None:
            return  # no scan yet (sim starting)
        now = self.now()
        age = now - self.last_scan
        self.update_lidar(now, age)
        if self.last_hb is not None:
            self.nav_age = now - self.last_hb
            self.update_nav(now, self.nav_age)

        safe_stop = self.state == CRITICAL
        if safe_stop:
            self.pub_safety.publish(Twist())

        self.pub_age.publish(Float64(data=age))
        self.csv.writerow([
            f'{time.time():.3f}', f'{now:.3f}', NAMES[self.state],
            NAMES[self.lidar_state], NAMES[self.odom_state], NAMES[self.control_state],
            NAMES[self.nav_state],
            f'{age:.3f}', int(self.lidar_fault), int(self.odom_fault),
            f'{self.odom_raw_vx:.4f}', f'{self.odom_vx:.4f}', int(not self.tick_invalid),
            '|'.join(sorted(self.tick_errors)) or 'none', self.odom_invalid_total,
            f'{self.speed_limit:.2f}', int(safe_stop), f'{self.nav_vx:.3f}', f'{self.out_vx:.3f}',
            f'{self.latency:.3f}', self.delay_mode, f'{self.injected_delay:.2f}', f'{self.nav_raw_vx:.3f}',
            f'{self.nav_age:.3f}', int(self.nav_age < self.nav_thresholds[WARNING]),
            self.nav_restart_count, self.nav_pid])
        self.csv_file.flush()
        self.tick_invalid = False
        self.tick_errors.clear()

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
