"""Safe Resume Gate: decides whether the robot may move after a fault (fail-closed).

health_monitor returning to NORMAL is necessary but not sufficient: the gate re-checks every
condition itself and only grants the permit after all of them held for `stable_time` seconds.

States:
  WAITING    waiting for normalization (a required status never seen yet, or system CRITICAL).
             After `waiting_timeout` s -> BLOCKED.
  VERIFYING  every condition passes; waiting for them to hold for `stable_time` s.
  APPROVED   permit granted (the only state in which /safe_resume/approved is true).
  BLOCKED    a safety condition failed; see /safe_resume/reason.

Two levels of the same checks:
  strict (not approved): subsystems must be NORMAL; clearance is computed for `resume_speed`.
  hold   (approved):     WARNING / DEGRADED stay with the existing speed limit, so they do not revoke;
                         CRITICAL, stale or invalid data and a failed condition revoke immediately.
                         Clearance is computed for the current speed.

How the permit stops the robot: speed_limiter forwards Nav2 commands only while a true
/safe_resume/approved is fresher than its permit_timeout (published here at `rate` Hz), otherwise it
outputs zero. The gate never publishes velocity: /cmd_vel_safety stays health_monitor's only stop
channel, and if the gate or the relay dies the Nav2 channel closes by itself.

Stopped robot: AMCL publishes /amcl_pose only after motion, so `request_nomotion_update` is called
when the pose gets old (a dead AMCL then simply stays stale). Latency samples come from the
control_delay_injector probe (probe_rate), since a stopped robot has no commands to measure.

Gate hang: after a stall nothing it believes is current (even its /clock is old), so a gap of more than
`max_tick_gap` wall-clock seconds between ticks revokes the permit, and `stable_time` must also pass on the
wall clock (the sim clock jumps forward when the buffered /clock messages arrive).

Topics: /safe_resume/state, /safe_resume/reason (String, transient local), /safe_resume/approved (Bool),
        /safe_resume/check/{system,lidar,odom,nav2,localization,obstacle,control} (Bool),
        /safe_resume/status (String JSON), /safe_resume/event (String)
"""
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Empty

WAITING, VERIFYING, APPROVED, BLOCKED = 'WAITING', 'VERIFYING', 'APPROVED', 'BLOCKED'
CHECKS = ('system', 'lidar', 'odom', 'nav2', 'localization', 'obstacle', 'control')


def required_clearance(speed, reaction_time, latency, max_decel, margin, robot_radius):
    """Distance from the robot centre that must be free: reaction + braking + margin + body."""
    speed = abs(speed)
    return speed * (reaction_time + latency) + speed ** 2 / (2.0 * max_decel) + margin + robot_radius


def forward_clearance(ranges, angle_min, angle_inc, range_min, half_width, laser_x, cone=math.pi / 4):
    """Free distance ahead in the robot's driving corridor.

    Returns (distance from the robot centre, fraction of NaN rays in the forward cone). +inf means no
    return within range (REP 117); a return below range_min is an obstacle closer than the sensor can
    measure, so it is treated as range_min, never as free space.
    """
    r = np.asarray(ranges, dtype=float)
    a = (angle_min + angle_inc * np.arange(r.size) + math.pi) % (2 * math.pi) - math.pi
    in_cone = np.abs(a) <= cone
    nan_ratio = float(np.isnan(r[in_cone]).mean()) if in_cone.any() else 1.0
    known = ~np.isnan(r) & ~np.isposinf(r)
    d = np.maximum(r[known], range_min)
    x = laser_x + d * np.cos(a[known])
    y = d * np.sin(a[known])
    hit = x[(x > 0.0) & (np.abs(y) <= half_width)]
    return (float(hit.min()) if hit.size else math.inf), nan_ratio


def scan_quality(ranges, range_min, range_max):
    """(usable fraction, number of real returns): NaN is unusable, +inf is a valid "nothing there"."""
    r = np.asarray(ranges, dtype=float)
    if r.size == 0:
        return 0.0, 0
    returns = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    usable = returns | np.isposinf(r)
    return float(usable.mean()), int(returns.sum())


def pose_problem(msg, frame, max_xy_cov, max_yaw_cov):
    """Why a PoseWithCovarianceStamped cannot be trusted ('' = fine)."""
    p, c = msg.pose.pose.position, msg.pose.covariance
    if msg.header.frame_id != frame:
        return f'frame {msg.header.frame_id!r} != {frame!r}'
    values = (p.x, p.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w, c[0], c[7], c[35])
    if not all(math.isfinite(v) for v in values):
        return 'pose or covariance is not finite'
    if c[0] < 0.0 or c[7] < 0.0 or c[35] < 0.0:
        return 'negative covariance'
    if max(c[0], c[7]) > max_xy_cov:
        return f'xy covariance {max(c[0], c[7]):.3f} > {max_xy_cov:.3f} m^2'
    if c[35] > max_yaw_cov:
        return f'yaw covariance {c[35]:.3f} > {max_yaw_cov:.3f} rad^2'
    return ''


class SafeResumeGate(Node):
    def __init__(self):
        super().__init__('safe_resume_gate')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.stable_time = p('stable_time', 3.0)
        self.max_tick_gap = p('max_tick_gap', 0.5)
        self.waiting_timeout = p('waiting_timeout', 8.0)
        self.health_max_age = p('health_max_age', 0.5)
        self.scan_max_age = p('scan_max_age', 0.5)
        self.scan_min_rate = p('scan_min_rate', 3.0)
        self.scan_min_usable = p('scan_min_usable', 0.9)
        self.scan_min_returns = p('scan_min_returns', 20)
        self.odom_max_age = p('odom_max_age', 0.5)
        self.odom_min_rate = p('odom_min_rate', 10.0)
        self.odom_stable_window = p('odom_stable_window', 2.0)
        self.nav2_max_age = p('nav2_max_age', 1.0)
        self.amcl_max_age = p('amcl_max_age', 4.0)
        self.amcl_ping_period = p('amcl_ping_period', 2.0)
        self.amcl_frame = p('amcl_frame', 'map')
        self.amcl_max_xy_cov = p('amcl_max_xy_cov', 0.25)
        self.amcl_max_yaw_cov = p('amcl_max_yaw_cov', 0.25)
        self.amcl_hold_max_xy_cov = p('amcl_hold_max_xy_cov', 1.0)
        self.amcl_hold_max_yaw_cov = p('amcl_hold_max_yaw_cov', 0.40)
        self.control_max_age = p('control_max_age', 1.5)
        self.control_max_latency = p('control_max_latency', 0.10)
        self.control_hold_max_latency = p('control_hold_max_latency', 0.50)
        self.resume_speed = p('resume_speed', 0.20)
        self.reaction_time = p('reaction_time', 0.25)
        self.max_decel = p('max_decel', 0.5)
        self.safety_margin = p('safety_margin', 0.10)
        self.robot_radius = p('robot_radius', 0.105)
        self.corridor_half_width = p('corridor_half_width', 0.155)
        self.laser_x_offset = p('laser_x_offset', -0.032)
        self.max_nan_ratio = p('max_nan_ratio', 0.1)
        rate = p('rate', 20.0)

        self.state = WAITING
        self.t_state = None
        self.t_verify = None
        self.wall_verify = None
        self.last_verify = None             # sim seconds of the verification that led to the last approval
        self.wall_tick = None
        self.t_ping = -1e9
        self.reason = ''
        self.results = {}
        self.clear = (math.inf, math.inf)   # measured, required
        self.t_status = -1e9

        self.health = {k: None for k in ('state', 'lidar_state', 'odom_state', 'control_state', 'nav2_state')}
        self.recovery_state = None
        self.nav2_nodes = {}
        self.t = {}                         # topic key -> receive time (node clock)
        self.scan = None
        self.scan_times = deque(maxlen=64)
        self.odom_times = deque(maxlen=128)
        self.vx = 0.0
        self.pose = None
        self.latency = None
        self.stack_ok = False
        self.odom_last_invalid = -1e9

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        for key in self.health:
            self.create_subscription(String, f'health/{key}',
                                     lambda m, k=key: self.on_health(k, m.data), latched)
        self.create_subscription(Float64, 'health/scan_age', lambda _m: self.stamp('health'), 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom_validated', self.on_odom_validated, 10)
        self.create_subscription(Bool, 'health/odom_valid', self.on_odom_valid, 10)
        self.create_subscription(Bool, 'nav2/stack_ok', self.on_stack_ok, 10)
        self.create_subscription(String, 'nav2_recovery/state',
                                 lambda m: setattr(self, 'recovery_state', m.data), latched)
        self.create_subscription(String, 'nav2_recovery/node_states', self.on_nav2_nodes, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose, amcl_qos)
        self.create_subscription(Float64, 'control/latency_sec', self.on_latency, 10)

        self.ping = self.create_client(Empty, 'request_nomotion_update')
        self.pub_approved = self.create_publisher(Bool, 'safe_resume/approved', 10)
        self.pub_state = self.create_publisher(String, 'safe_resume/state', latched)
        self.pub_reason = self.create_publisher(String, 'safe_resume/reason', latched)
        self.pub_status = self.create_publisher(String, 'safe_resume/status', 10)
        self.pub_event = self.create_publisher(String, 'safe_resume/event', 10)
        self.pub_check = {c: self.create_publisher(Bool, f'safe_resume/check/{c}', 10) for c in CHECKS}

        self.pub_state.publish(String(data=self.state))
        self.pub_reason.publish(String(data='waiting for required status'))
        self.pub_approved.publish(Bool(data=False))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'Safe Resume Gate started: {self.state}, permit denied')

    # --- inputs ---
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp(self, key):
        self.t[key] = self.now()

    def age(self, key, now):
        return now - self.t[key] if key in self.t else math.inf

    def on_health(self, key, value):
        self.health[key] = value
        if key == 'state' and value == 'CRITICAL':
            self.tick()            # revoke now, not at the next timer tick

    def on_scan(self, msg):
        self.scan = msg
        self.stamp('scan')
        self.scan_times.append(self.t['scan'])

    def on_odom_validated(self, msg):
        self.vx = msg.twist.twist.linear.x
        self.stamp('odom_validated')
        self.odom_times.append(self.t['odom_validated'])

    def on_odom_valid(self, msg):
        self.stamp('odom_valid')
        if not msg.data:
            self.odom_last_invalid = self.t['odom_valid']

    def on_stack_ok(self, msg):
        self.stack_ok = msg.data
        self.stamp('stack_ok')

    def on_nav2_nodes(self, msg):
        try:
            self.nav2_nodes = json.loads(msg.data).get('nodes', {})
        except ValueError:
            self.nav2_nodes = {}

    def on_pose(self, msg):
        self.pose = msg
        self.stamp('pose')

    def on_latency(self, msg):
        self.latency = msg.data
        self.stamp('latency')

    @staticmethod
    def rate_of(times, now, window=2.0):
        return sum(1 for t in times if now - t <= window) / window

    # --- checks: each returns '' when the condition holds, else why it does not ---
    def check_system(self, now, strict):
        state = self.health['state']
        if state is None or self.age('health', now) > self.health_max_age:
            return 'health_monitor status unavailable'
        if state == 'CRITICAL' or (strict and state != 'NORMAL'):
            return f'system {state}'
        return ''

    def check_lidar(self, now, strict):
        age = self.age('scan', now)
        if age > self.scan_max_age:
            return 'no scan received' if self.scan is None else f'scan age {age:.2f} s > {self.scan_max_age:.2f} s'
        s = self.scan
        stamp_age = now - (s.header.stamp.sec + s.header.stamp.nanosec * 1e-9)
        if stamp_age > self.scan_max_age:
            return f'scan stamp is {stamp_age:.2f} s old'
        usable, returns = scan_quality(s.ranges, s.range_min, s.range_max)
        if usable < self.scan_min_usable or returns < self.scan_min_returns:
            return f'invalid scan (usable {usable:.2f}, returns {returns})'
        lidar = self.health['lidar_state']
        if lidar is None or lidar == 'CRITICAL' or (strict and lidar != 'NORMAL'):
            return f'health lidar {lidar}'
        rate = self.rate_of(self.scan_times, now)
        if strict and rate < self.scan_min_rate:
            return f'scan rate {rate:.1f} Hz < {self.scan_min_rate:.1f} Hz'
        return ''

    def check_odom(self, now, strict):
        if self.age('odom_valid', now) > self.odom_max_age:
            return 'odometry check not running (no /health/odom_valid)'
        if not strict:
            return ''       # implausible odometry while driving: existing DEGRADED speed limit
        odom = self.health['odom_state']
        if odom != 'NORMAL':
            return f'health odom {odom}'
        since = now - self.odom_last_invalid
        if since < self.odom_stable_window:
            return f'implausible sample {since:.1f} s ago'
        age = self.age('odom_validated', now)
        if age > self.odom_max_age:
            return f'/odom_validated age {age:.2f} s'
        rate = self.rate_of(self.odom_times, now)
        if rate < self.odom_min_rate:
            return f'validated odom rate {rate:.1f} Hz < {self.odom_min_rate:.1f} Hz'
        return ''

    def check_nav2(self, now, strict):
        age = self.age('stack_ok', now)
        if age > self.nav2_max_age:
            return 'nav2_recovery_manager status unavailable'
        rec = self.recovery_state
        if not self.stack_ok:
            bad = [n for n, v in self.nav2_nodes.items() if v.get('state') != 'ACTIVE']
            return f'not ACTIVE: {", ".join(bad) or "unknown"} (recovery {rec})'
        if rec is None or rec not in (('IDLE',) if strict else ('IDLE', 'RECOVERED')):
            return f'recovery {rec}'
        if strict and self.health['nav2_state'] != 'NORMAL':
            return f'health nav2 {self.health["nav2_state"]}'
        return ''

    def check_localization(self, now, strict):
        if self.pose is None:
            return 'no /amcl_pose received'
        h = self.pose.header.stamp
        age = max(self.age('pose', now), now - (h.sec + h.nanosec * 1e-9))
        if age > self.amcl_max_age:
            return f'/amcl_pose age {age:.1f} s > {self.amcl_max_age:.1f} s'
        if strict:
            return pose_problem(self.pose, self.amcl_frame, self.amcl_max_xy_cov, self.amcl_max_yaw_cov)
        return pose_problem(self.pose, self.amcl_frame, self.amcl_hold_max_xy_cov, self.amcl_hold_max_yaw_cov)

    def check_obstacle(self, now, strict):
        self.clear = (math.inf, math.inf)
        if self.scan is None or self.age('scan', now) > self.scan_max_age:
            return 'no fresh scan: clearance unknown'
        s = self.scan
        dist, nan_ratio = forward_clearance(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                            self.corridor_half_width, self.laser_x_offset)
        if nan_ratio > self.max_nan_ratio:
            return f'{nan_ratio * 100:.0f} % invalid rays ahead: clearance unknown'
        speed = max(self.resume_speed, abs(self.vx)) if strict else abs(self.vx)
        latency = self.latency if self.latency is not None and math.isfinite(self.latency) else 0.0
        need = required_clearance(speed, self.reaction_time, latency, self.max_decel,
                                  self.safety_margin, self.robot_radius)
        self.clear = (dist, need)
        return f'obstacle {dist:.2f} m ahead < required {need:.2f} m' if dist < need else ''

    def check_control(self, now, strict):
        age = self.age('latency', now)
        if age > self.control_max_age:
            return 'no latency sample' if self.latency is None else f'latency sample age {age:.1f} s'
        limit = self.control_max_latency if strict else self.control_hold_max_latency
        if not math.isfinite(self.latency) or self.latency > limit:
            return f'latency {self.latency * 1000:.0f} ms > {limit * 1000:.0f} ms'
        control = self.health['control_state']
        if control is None or control == 'CRITICAL' or (strict and control != 'NORMAL'):
            return f'health control {control}'
        return ''

    # --- state machine ---
    def transition(self, new, now):
        if new == self.state:
            return
        self.pub_event.publish(String(data=self.describe(new)))
        log = self.get_logger()
        if new == APPROVED:
            log.info(self.describe(new))
        else:
            log.warn(self.describe(new))
        self.state = new
        self.t_state = now
        self.pub_state.publish(String(data=new))

    def describe(self, new):
        text = f'{self.state} -> {new}'
        return f'{text}: {self.reason}' if self.reason else text

    def tick(self):
        now = self.now()
        if now <= 0.0:
            return                  # no /clock yet
        if self.t_state is None:
            self.t_state = now
        wall = time.monotonic()
        gap = wall - self.wall_tick if self.wall_tick is not None else 0.0
        self.wall_tick = wall
        if gap > self.max_tick_gap:
            self.t_verify = None
            self.reason = f'gate: evaluation stalled for {gap:.1f} s'
            self.pub_reason.publish(String(data=self.reason))
            if self.state in (APPROVED, VERIFYING):
                self.transition(BLOCKED, now)
            self.pub_approved.publish(Bool(data=False))
            return
        strict = self.state != APPROVED
        failures = {}
        self.results = {}
        for name in CHECKS:
            why = getattr(self, f'check_{name}')(now, strict)
            self.results[name] = why
            if why:
                failures[name] = why
        reason = '; '.join(f'{k}: {v}' for k, v in failures.items())
        if reason != self.reason:
            self.reason = reason
            self.pub_reason.publish(String(data=reason))
        self.step(now, bool(failures))

        self.pub_approved.publish(Bool(data=self.state == APPROVED))
        self.refresh_pose(now)
        if now - self.t_status >= 0.2:
            self.t_status = now
            self.publish_status(now)

    def step(self, now, failed):
        critical = self.health['state'] in (None, 'CRITICAL')
        unknown = any(k not in self.t for k in ('health', 'scan', 'odom_valid', 'stack_ok', 'pose', 'latency'))
        if self.state == APPROVED:
            if failed:
                self.transition(WAITING if critical else BLOCKED, now)
            return
        if not failed:
            if self.state != VERIFYING:
                self.t_verify = now
                self.wall_verify = time.monotonic()
                self.transition(VERIFYING, now)
            elif (now - self.t_verify >= self.stable_time
                  and time.monotonic() - self.wall_verify >= self.stable_time):
                self.last_verify = now - self.t_verify
                self.transition(APPROVED, now)
            return
        self.t_verify = None
        if self.state == VERIFYING:
            self.transition(BLOCKED, now)
        elif self.state == WAITING and (self.recovery_state == 'FAILED' or not (critical or unknown)
                                        or now - self.t_state >= self.waiting_timeout):
            self.transition(BLOCKED, now)

    def refresh_pose(self, now):
        if (self.age('pose', now) >= self.amcl_ping_period and now - self.t_ping >= self.amcl_ping_period
                and self.ping.service_is_ready()):
            self.t_ping = now
            self.ping.call_async(Empty.Request())

    def publish_status(self, now):
        for name, why in self.results.items():
            self.pub_check[name].publish(Bool(data=not why))
        dist, need = self.clear
        self.pub_status.publish(String(data=json.dumps({
            'state': self.state, 'approved': self.state == APPROVED, 'reason': self.reason,
            'checks': {k: {'ok': not v, 'detail': v} for k, v in self.results.items()},
            'verify_elapsed_s': round(now - self.t_verify, 2) if self.state == VERIFYING else None,
            'stable_time_s': self.stable_time,
            'last_verify_s': round(self.last_verify, 3) if self.last_verify is not None else None,
            'obstacle_distance_m': round(dist, 3) if math.isfinite(dist) else None,
            'required_clearance_m': round(need, 3) if math.isfinite(need) else None,
        })))


def main():
    rclpy.init()
    node = SafeResumeGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
