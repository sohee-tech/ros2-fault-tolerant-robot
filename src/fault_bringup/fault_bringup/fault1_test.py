"""Automated Fault 1 (LiDAR dropout) test with twist_mux arbitration.

Requires fault1_sim.launch.py running.
The nav command (0.2 m/s @10 Hz) comes from nav_command_source; the test only switches it
on/off through the latched /nav_source/active_request topic.
A-C: nav command loss -> idle_stop (prio 1) must stop the robot; nav restart -> drive again.
D:   forward command (linear.x = NAV_SPEED) on /cmd_vel_nav_raw (nav source) at 10 Hz during the whole
     LiDAR fault; the safety stop must win via twist_mux, and the robot must restart on
     its own after recovery.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

NAV_SPEED = 0.2


class Fault1Test(Node):
    def __init__(self, name='fault1_test'):
        super().__init__(name, parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.scan_times = []          # sim time of each /scan receive
        self.states = []              # (sim_time, state) on change
        self.nav = []                 # (sim_time, linear.x) seen on /cmd_vel_nav_raw (nav source)
        self.out = []                 # (sim_time, linear.x) seen on /cmd_vel (mux output)
        self.odom = None
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Twist, 'cmd_vel_nav_raw', lambda m: self.nav.append((self.now(), m.linear.x)), 10)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self.out.append((self.now(), m.linear.x)), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'health/state', self.on_state, latched)
        self.pub_nav_request = self.create_publisher(Bool, 'nav_source/active_request', latched)
        self.set_nav(True)
        self.cli_enable = self.create_client(Trigger, 'lidar_fault/enable')
        self.cli_disable = self.create_client(Trigger, 'lidar_fault/disable')
        self.results = []
        self.snapshots = []

    # --- callbacks
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, _):
        self.scan_times.append(self.now())

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, msg.twist.twist.linear.x)

    def on_state(self, msg):
        if not self.states or self.states[-1][1] != msg.data:
            self.states.append((self.now(), msg.data))
            print(f'  state -> {msg.data} (t={self.now():.2f})', flush=True)

    # --- helpers
    def state(self):
        return self.states[-1][1] if self.states else None

    def spin_for(self, sec):
        self.wait_until(lambda: False, sec)

    def set_nav(self, active):
        """Request driving on/off from nav_command_source (latched: survives its restarts)."""
        self.pub_nav_request.publish(Bool(data=active))

    def wait_until(self, cond, timeout):
        """Spin with wall timeout until cond() is true."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if cond():
                return True
        return False

    def check(self, name, ok, detail=''):
        self.results.append((name, ok, detail))
        print(f'[{"PASS" if ok else "FAIL"}] {name} {detail}', flush=True)
        return ok

    def call(self, client):
        client.wait_for_service(timeout_sec=10.0)
        fut = client.call_async(Trigger.Request())
        self.wait_until(fut.done, 5.0)
        return fut.result()

    def window(self, series, t0, t1):
        return [v for t, v in series if t0 <= t <= t1]

    def snapshot(self, label, dur=1.0):
        """Measure nav/out/odom over `dur` seconds (sim time)."""
        t0 = self.now()
        x0, y0, _ = self.odom
        self.spin_for(dur)
        t1 = self.now()
        x1, y1, vx = self.odom
        nav = self.window(self.nav, t0, t1)
        out = self.window(self.out, t0, t1)
        snap = dict(label=label, nav=nav, out=out, vx=vx,
                    moved=math.hypot(x1 - x0, y1 - y0), dt=t1 - t0)
        self.snapshots.append(snap)
        fmt = lambda v: f'{min(v):.2f}..{max(v):.2f} (n={len(v)})' if v else 'none'
        print(f'  {label:22s} cmd_vel_nav={fmt(nav)}  cmd_vel={fmt(out)}  '
              f'odom_vx={vx:.3f}  moved={snap["moved"] * 100:.1f} cm/{snap["dt"]:.1f}s', flush=True)
        return snap

    # --- test
    def run(self):
        print('== 1. normal scan', flush=True)
        ok = self.wait_until(lambda: len(self.scan_times) >= 10 and self.odom is not None, 30.0)
        if not self.check('scan received', ok, f'({len(self.scan_times)} msgs)'):
            return False
        self.wait_until(lambda: self.state() is not None, 5.0)
        self.check('state NORMAL at start', self.state() == 'NORMAL')

        print(f'== 2. (A) nav command {NAV_SPEED} m/s @10 Hz', flush=True)
        self.spin_for(1.5)
        s = self.snapshot('before fault')
        self.check('pre-fault: cmd_vel_nav forward', s['nav'] and min(s['nav']) == NAV_SPEED)
        self.check('pre-fault: mux cmd_vel forward', s['out'] and min(s['out']) == NAV_SPEED)
        self.check('pre-fault: robot moving', s['vx'] > NAV_SPEED * 0.7, f'(odom vx={s["vx"]:.3f})')

        print('== 2b. nav command stopped (command loss)', flush=True)
        self.set_nav(False)
        t_loss = self.now()
        self.spin_for(1.5)   # > nav timeout 0.5 s + deceleration
        s = self.snapshot('nav lost')
        self.check('nav loss: no cmd_vel_nav received', not self.window(self.nav, t_loss + 0.2, self.now()))
        self.check('nav loss: mux cmd_vel zero (idle_stop)',
                   len(s['out']) >= 10 and max(abs(v) for v in s['out']) == 0.0, f'(n={len(s["out"])})')
        self.check('nav loss: robot stopped (odom)', abs(s['vx']) < 0.01 and s['moved'] < 0.01,
                   f'(vx={s["vx"]:.4f}, moved {s["moved"] * 1000:.1f} mm)')

        print('== 2c. nav command resumed', flush=True)
        self.set_nav(True)
        self.spin_for(1.5)
        s = self.snapshot('nav recovered')
        self.check('nav recovery: mux cmd_vel forward', s['out'] and min(s['out']) == NAV_SPEED)
        self.check('nav recovery: robot moving (odom)', s['vx'] > NAV_SPEED * 0.7 and s['moved'] > 0.1,
                   f'(odom vx={s["vx"]:.3f})')

        print('== 3. fault enable', flush=True)
        self.states = self.states[-1:]
        res = self.call(self.cli_enable)
        t_fault = self.now()
        print(f'  enable: {res.message}', flush=True)
        ok = self.wait_until(lambda: self.state() == 'CRITICAL', 10.0)
        self.check('reached CRITICAL', ok)
        seq = [st for _, st in self.states]
        self.check('state order NORMAL->WARNING->DEGRADED->CRITICAL',
                   seq == ['NORMAL', 'WARNING', 'DEGRADED', 'CRITICAL'], f'({seq})')
        for t, st in self.states[1:]:
            print(f'  {st:9s} at fault+{t - t_fault:.2f} s', flush=True)

        print('== 4. CRITICAL (D): nav still forward, mux must output zero', flush=True)
        self.spin_for(1.0)   # let the robot decelerate
        s = self.snapshot('CRITICAL', 2.0)
        self.check('CRITICAL: cmd_vel_nav still non-zero',
                   len(s['nav']) >= 10 and min(s['nav']) == NAV_SPEED, f'(n={len(s["nav"])})')
        self.check('CRITICAL: mux cmd_vel all zero',
                   len(s['out']) >= 10 and max(abs(v) for v in s['out']) == 0.0,
                   f'(n={len(s["out"])})')
        self.check('CRITICAL: robot stopped (odom)', abs(s['vx']) < 0.01 and s['moved'] < 0.01,
                   f'(vx={s["vx"]:.4f}, moved {s["moved"] * 1000:.1f} mm)')
        late = [t for t in self.scan_times if t > t_fault + 0.25]
        self.check('/scan silent during fault', len(late) == 0, f'({len(late)} scans)')

        print('== 5. fault disable', flush=True)
        res = self.call(self.cli_disable)
        t_off = self.now()
        print(f'  disable: {res.message}', flush=True)
        ok = self.wait_until(lambda: self.scan_times and self.scan_times[-1] > t_off, 5.0)
        self.check('/scan resumed', ok)
        t_resume = next(t for t in self.scan_times if t > t_off) if ok else t_off
        early = self.wait_until(lambda: self.state() == 'NORMAL', 1.0)
        self.check('no immediate NORMAL (hysteresis)', not early, f'(state {self.state()})')
        ok = self.wait_until(lambda: self.state() == 'NORMAL', 10.0)
        t_norm = self.states[-1][0]
        self.check('returned to NORMAL', ok)
        self.check('recovery hold >= 2.0 s', ok and t_norm - t_resume >= 2.0,
                   f'({t_norm - t_resume:.2f} s after first resumed scan)')
        stopped_until_normal = self.window(self.out, t_resume, t_norm - 0.05)
        self.check('robot held stopped during recovery',
                   stopped_until_normal and max(abs(v) for v in stopped_until_normal) == 0.0)

        print('== 6. auto restart', flush=True)
        ok = self.wait_until(lambda: self.out and self.out[-1][1] == NAV_SPEED, 2.0)
        t_restart = self.out[-1][0]
        self.check('mux selects cmd_vel_nav again', ok, f'({t_restart - t_norm:.2f} s after NORMAL)')
        self.spin_for(1.0)
        s = self.snapshot('after recovery')
        self.check('robot restarted (odom)', s['vx'] > NAV_SPEED * 0.7 and s['moved'] > 0.1,
                   f'(odom vx={s["vx"]:.3f})')

        # stop the robot at the end of the test
        self.set_nav(False)
        self.spin_for(0.3)
        return all(r[1] for r in self.results)


def main():
    rclpy.init()
    node = Fault1Test()
    try:
        passed = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print('\n==== FAULT 1 + TWIST_MUX TEST:', 'PASSED' if passed else 'FAILED', '====')
    sys.exit(0 if passed else 1)
