"""Automated Fault 2 (odometry velocity spike) test. Requires fault_sim.launch.py running.

The nav command (0.2 m/s @10 Hz on /cmd_vel_nav_raw) stays on for the whole test.
A. normal: cmd_vel 0.2, odom 0.2, NORMAL
B. one spiked odom sample: rejected, system does not escalate
C. continuous spike: DEGRADED, speed limited to 0.1 (cmd_vel and real motion)
D. fault disabled: DEGRADED held ~2 s, then NORMAL and 0.2 m/s again
Real speed is taken from /odom_raw (Gazebo) and from the /odom pose, which the
injector does not modify.
"""
import sys

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from fault_bringup.fault1_test import Fault1Test, NAV_SPEED

LIMIT = 0.10
TOL = 0.02


class Fault2Test(Fault1Test):
    def __init__(self):
        super().__init__('fault2_test')
        self.odom_out = []      # (t, vx) on /odom (after injector)
        self.odom_raw = []      # (t, vx) on /odom_raw (Gazebo ground truth)
        self.valid = []         # (t, bool) on /health/odom_valid
        self.odom_states = []
        self.limit = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, 'odom', lambda m: self.odom_out.append(
            (self.now(), m.twist.twist.linear.x)), 10)
        self.create_subscription(Odometry, 'odom_raw', lambda m: self.odom_raw.append(
            (self.now(), m.twist.twist.linear.x)), 10)
        self.create_subscription(Bool, 'health/odom_valid', lambda m: self.valid.append(
            (self.now(), m.data)), 10)
        self.create_subscription(String, 'health/odom_state', lambda m: self.odom_states.append(
            (self.now(), m.data)), latched)
        self.create_subscription(Float64, 'health/speed_limit',
                                 lambda m: setattr(self, 'limit', m.data), latched)
        self.cli_odom_enable = self.create_client(Trigger, 'odom_fault/enable')
        self.cli_odom_disable = self.create_client(Trigger, 'odom_fault/disable')
        self.cli_odom_spike = self.create_client(Trigger, 'odom_fault/spike')

    def raw_snapshot(self, label, dur):
        t0 = self.now()
        s = self.snapshot(label, dur)
        t1 = self.now()
        raw = self.window(self.odom_raw, t0, t1)
        out = self.window(self.odom_out, t0, t1)
        s['raw_mean'] = sum(raw) / len(raw) if raw else float('nan')
        s['odom_out_max'] = max(out) if out else float('nan')
        s['speed'] = s['moved'] / s['dt']
        print(f'  {"":22s} odom_raw mean={s["raw_mean"]:.3f}  /odom max={s["odom_out_max"]:.3f}  '
              f'pose speed={s["speed"]:.3f} m/s  state={self.state()}  limit={self.limit}', flush=True)
        return s

    def run(self):
        print('== A. normal', flush=True)
        ok = self.wait_until(lambda: len(self.scan_times) >= 10 and self.odom is not None, 30.0)
        if not self.check('sim ready (scan + odom)', ok):
            return False
        self.spin_for(1.5)
        s = self.raw_snapshot('normal', 1.0)
        self.check('A: state NORMAL', self.state() == 'NORMAL')
        self.check('A: cmd_vel = 0.20', s['out'] and min(s['out']) == NAV_SPEED)
        self.check('A: odom ~0.20', abs(s['raw_mean'] - NAV_SPEED) < TOL and abs(s['speed'] - NAV_SPEED) < TOL,
                   f'(odom_raw {s["raw_mean"]:.3f}, pose {s["speed"]:.3f})')
        self.check('A: no speed limit', not self.limit)

        print('== B. single odom spike', flush=True)
        n_states = len(self.states)
        t0 = self.now()
        print(f'  spike: {self.call(self.cli_odom_spike).message}', flush=True)
        self.spin_for(1.5)
        t1 = self.now()
        spikes = [v for v in self.window(self.odom_out, t0, t1) if v > 1.0]
        rejected = [v for v in self.window(self.valid, t0, t1) if not v]
        self.check('B: spike present on /odom', len(spikes) == 1, f'(/odom vx={spikes})')
        self.check('B: spiked sample rejected (odom_valid=false)', len(rejected) == 1,
                   f'({len(rejected)} invalid of {len(self.window(self.valid, t0, t1))} samples)')
        self.check('B: system state unchanged (NORMAL)',
                   len(self.states) == n_states and self.state() == 'NORMAL', f'({self.state()})')
        s = self.raw_snapshot('after single spike', 1.0)
        self.check('B: still 0.20 m/s, no limit', min(s['out']) == NAV_SPEED and not self.limit)

        print('== C. continuous odom anomaly', flush=True)
        n_states = len(self.states)
        print(f'  enable: {self.call(self.cli_odom_enable).message}', flush=True)
        t_fault = self.now()
        ok = self.wait_until(lambda: self.state() == 'DEGRADED', 3.0)
        self.check('C: system DEGRADED', ok,
                   f'(after {self.states[-1][0] - t_fault:.2f} s)' if ok else f'({self.state()})')
        self.spin_for(1.0)   # decelerate to the limit
        s = self.raw_snapshot('DEGRADED', 2.0)
        self.check('C: nav command still 0.20', len(s['nav']) >= 15 and min(s['nav']) == NAV_SPEED,
                   f'(n={len(s["nav"])})')
        self.check('C: final cmd_vel limited to 0.10',
                   s['out'] and all(abs(v - LIMIT) < 1e-6 for v in s['out']), f'(n={len(s["out"])})')
        self.check('C: real speed ~0.10 (odom_raw + pose)',
                   abs(s['raw_mean'] - LIMIT) < TOL and abs(s['speed'] - LIMIT) < TOL,
                   f'(odom_raw {s["raw_mean"]:.3f}, pose {s["speed"]:.3f})')
        self.check('C: /odom still corrupted (fault real)', s['odom_out_max'] > 2.0,
                   f'(/odom max {s["odom_out_max"]:.2f})')

        print('== D. fault disable -> recovery', flush=True)
        print(f'  disable: {self.call(self.cli_odom_disable).message}', flush=True)
        t_off = self.now()
        early = self.wait_until(lambda: self.state() == 'NORMAL', 1.0)
        self.check('D: DEGRADED held (hysteresis)', not early, f'(state {self.state()} 1 s after disable)')
        ok = self.wait_until(lambda: self.state() == 'NORMAL', 5.0)
        t_norm = self.states[-1][0]
        self.check('D: returned to NORMAL', ok)
        self.check('D: recovery hold ~2 s', ok and 2.0 <= t_norm - t_off <= 2.6,
                   f'({t_norm - t_off:.2f} s after disable)')
        self.spin_for(1.0)
        s = self.raw_snapshot('after recovery', 1.0)
        self.check('D: cmd_vel back to 0.20', s['out'] and min(s['out']) == NAV_SPEED)
        self.check('D: real speed back to ~0.20', abs(s['raw_mean'] - NAV_SPEED) < TOL
                   and abs(s['speed'] - NAV_SPEED) < TOL,
                   f'(odom_raw {s["raw_mean"]:.3f}, pose {s["speed"]:.3f})')
        self.check('D: speed limit released', not self.limit)

        seq = [st for _, st in self.states[n_states - 1:]]
        self.check('state sequence NORMAL->DEGRADED->NORMAL (never CRITICAL)',
                   seq == ['NORMAL', 'DEGRADED', 'NORMAL'], f'({seq})')

        self.set_nav(False)
        self.spin_for(0.3)
        return all(r[1] for r in self.results)


def main():
    rclpy.init()
    node = Fault2Test()
    try:
        passed = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print('\n==== FAULT 2 ODOM ANOMALY TEST:', 'PASSED' if passed else 'FAILED', '====')
    sys.exit(0 if passed else 1)
