"""Automated Fault 3 (control latency) test. Requires fault_sim.launch.py running.

The nav source (nav_command_source, 0.2 m/s @10 Hz on /cmd_vel_nav_raw) stays on for the whole test.
A. normal: latency < 0.1 s, cmd_vel 0.2, odom 0.2, NORMAL
B. 300 ms delay: control DEGRADED, cmd_vel 0.1, real speed 0.1
C. 700 ms delay: control CRITICAL, safety channel -> cmd_vel 0, robot stopped
D. delay disabled: no stale burst, CRITICAL held ~2 s, then NORMAL and 0.2 m/s again
E. nav source lost (injector alive) -> idle_stop stops the robot, restart on return
F. control_delay_injector process killed -> idle_stop stops the robot
"""
import os
import signal
import subprocess
import sys

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from fault_bringup.fault1_test import NAV_SPEED
from fault_bringup.fault2_test import Fault2Test, LIMIT, TOL


class Fault3Test(Fault2Test):
    def __init__(self):
        super().__init__()
        self.latency = []        # (t, sec) on /control/latency_sec
        self.nav_out = []        # (t, x) on /cmd_vel_nav (injector output)
        self.safety = []         # (t, x) on /cmd_vel_safety
        self.control_states = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64, 'control/latency_sec', lambda m: self.latency.append(
            (self.now(), m.data)), 10)
        self.create_subscription(Twist, 'cmd_vel_nav', lambda m: self.nav_out.append(
            (self.now(), m.linear.x)), 10)
        self.create_subscription(Twist, 'cmd_vel_safety', lambda m: self.safety.append(
            (self.now(), m.linear.x)), 10)
        self.create_subscription(String, 'health/control_state', self.on_control_state, latched)
        self.cli_delay = {m: self.create_client(Trigger, f'control_delay/{m}')
                          for m in ('degraded', 'critical', 'disable')}

    def on_control_state(self, msg):
        if not self.control_states or self.control_states[-1][1] != msg.data:
            self.control_states.append((self.now(), msg.data))
            print(f'  control -> {msg.data} (t={self.now():.2f})', flush=True)

    def control_state(self):
        return self.control_states[-1][1] if self.control_states else None

    def lat_stats(self, t0, t1):
        v = self.window(self.latency, t0, t1)
        return (sum(v) / len(v), min(v), max(v), len(v)) if v else (float('nan'),) * 3 + (0,)

    def measured(self, label, dur):
        t0 = self.now()
        s = self.raw_snapshot(label, dur)
        s['lat'] = self.lat_stats(t0, self.now())
        s['safety_n'] = len(self.window(self.safety, t0, self.now()))
        mean, lo, hi, n = s['lat']
        print(f'  {"":22s} latency mean={mean * 1000:.0f} ms [{lo * 1000:.0f}..{hi * 1000:.0f}] '
              f'(n={n})  control={self.control_state()}  safety msgs={s["safety_n"]}', flush=True)
        return s

    def run(self):
        print('== A. normal', flush=True)
        ok = self.wait_until(lambda: len(self.scan_times) >= 10 and self.odom is not None, 30.0)
        if not self.check('sim ready (scan + odom)', ok):
            return False
        self.spin_for(1.5)
        s = self.measured('normal', 1.0)
        self.check('A: system NORMAL', self.state() == 'NORMAL' and self.control_state() == 'NORMAL')
        self.check('A: source 0.20', s['nav'] and min(s['nav']) == NAV_SPEED)
        self.check('A: latency < 0.10 s', s['lat'][3] >= 5 and s['lat'][2] < 0.10,
                   f'(max {s["lat"][2] * 1000:.1f} ms)')
        self.check('A: cmd_vel 0.20', s['out'] and min(s['out']) == NAV_SPEED)
        self.check('A: odom ~0.20', abs(s['raw_mean'] - NAV_SPEED) < TOL and abs(s['speed'] - NAV_SPEED) < TOL,
                   f'(odom_raw {s["raw_mean"]:.3f}, pose {s["speed"]:.3f})')

        print('== B. DEGRADED delay (300 ms)', flush=True)
        n_ctrl = len(self.control_states)
        print(f'  {self.call(self.cli_delay["degraded"]).message}', flush=True)
        t_b = self.now()
        ok = self.wait_until(lambda: self.control_state() == 'DEGRADED', 3.0)
        self.check('B: control DEGRADED', ok, f'(after {self.now() - t_b:.2f} s)')
        self.spin_for(1.0)
        s = self.measured('DEGRADED', 2.0)
        self.check('B: measured latency ~0.30 s', abs(s['lat'][0] - 0.30) < 0.03,
                   f'(mean {s["lat"][0] * 1000:.0f} ms)')
        self.check('B: system DEGRADED', self.state() == 'DEGRADED')
        self.check('B: source still 0.20', len(s['nav']) >= 15 and min(s['nav']) == NAV_SPEED)
        self.check('B: speed limit 0.10 applied', self.limit == LIMIT)
        self.check('B: cmd_vel ~0.10', s['out'] and all(abs(v - LIMIT) < 1e-6 for v in s['out']),
                   f'(n={len(s["out"])})')
        self.check('B: real speed ~0.10', abs(s['raw_mean'] - LIMIT) < TOL and abs(s['speed'] - LIMIT) < TOL,
                   f'(odom_raw {s["raw_mean"]:.3f}, pose {s["speed"]:.3f})')

        print('== C. CRITICAL delay (700 ms)', flush=True)
        print(f'  {self.call(self.cli_delay["critical"]).message}', flush=True)
        t_c = self.now()
        ok = self.wait_until(lambda: self.control_state() == 'CRITICAL', 4.0)
        self.check('C: control CRITICAL', ok, f'(after {self.now() - t_c:.2f} s)')
        self.spin_for(1.0)
        s = self.measured('CRITICAL', 2.0)
        self.check('C: measured latency ~0.70 s', abs(s['lat'][0] - 0.70) < 0.03,
                   f'(mean {s["lat"][0] * 1000:.0f} ms)')
        self.check('C: system CRITICAL', self.state() == 'CRITICAL')
        self.check('C: source still 0.20', len(s['nav']) >= 15 and min(s['nav']) == NAV_SPEED)
        self.check('C: safety channel active', s['safety_n'] >= 20, f'({s["safety_n"]} msgs)')
        self.check('C: cmd_vel = 0', s['out'] and max(abs(v) for v in s['out']) == 0.0,
                   f'(n={len(s["out"])})')
        self.check('C: robot stopped (odom)', abs(s['raw_mean']) < 0.005 and s['moved'] < 0.005,
                   f'(odom_raw {s["raw_mean"]:.4f}, moved {s["moved"] * 1000:.1f} mm)')
        seq = [st for _, st in self.control_states[n_ctrl - 1:]]
        self.check('control sequence NORMAL->WARNING->DEGRADED->CRITICAL',
                   seq == ['NORMAL', 'WARNING', 'DEGRADED', 'CRITICAL'], f'({seq})')

        print('== D. delay disabled -> recovery', flush=True)
        t_req = self.now()
        print(f'  {self.call(self.cli_delay["disable"]).message}', flush=True)
        t_off = self.now()
        early = self.wait_until(lambda: self.state() == 'NORMAL', 1.0)
        after = self.window(self.latency, t_off + 0.05, self.now())
        burst = self.window(self.nav_out, t_off, t_off + 0.5)
        self.check('D: no stale command after disable (all latency < 0.10 s)',
                   after and max(after) < 0.10, f'(max {max(after) * 1000:.1f} ms, n={len(after)})')
        self.check('D: no burst (<= 7 commands in first 0.5 s)', len(burst) <= 7, f'({len(burst)} msgs)')
        self.check('D: CRITICAL held (hysteresis)', not early and self.state() == 'CRITICAL',
                   f'(state {self.state()} 1 s after disable)')
        # system and control state arrive on separate topics; wait for both
        ok = self.wait_until(lambda: self.state() == 'NORMAL' and self.control_state() == 'NORMAL', 5.0)
        t_norm = self.states[-1][0]
        t_healthy = next((t for t, v in self.latency if t >= t_req and v < 0.10), t_off)
        self.check('D: returned to NORMAL', ok and self.control_state() == 'NORMAL')
        # 0.05 s margin: monitor and test receive the same latency sample at slightly different times
        self.check('D: recovery hold ~2 s', ok and 1.95 <= t_norm - t_healthy <= 2.6,
                   f'({t_norm - t_healthy:.2f} s after first healthy command)')
        self.spin_for(1.0)
        s = self.measured('after recovery', 1.0)
        self.check('D: limit + safety released', not self.limit and s['safety_n'] == 0)
        self.check('D: cmd_vel 0.20', s['out'] and min(s['out']) == NAV_SPEED)
        self.check('D: odom ~0.20 (auto restart)', abs(s['raw_mean'] - NAV_SPEED) < TOL
                   and abs(s['speed'] - NAV_SPEED) < TOL,
                   f'(odom_raw {s["raw_mean"]:.3f}, pose {s["speed"]:.3f})')

        print('== E. nav source lost (injector alive)', flush=True)
        self.set_nav(False)
        self.spin_for(1.5)
        s = self.measured('nav source lost', 1.0)
        self.check('E: cmd_vel 0 via idle_stop', len(s['out']) >= 10 and max(abs(v) for v in s['out']) == 0.0)
        self.check('E: robot stopped', abs(s['raw_mean']) < 0.005 and s['moved'] < 0.005,
                   f'(moved {s["moved"] * 1000:.1f} mm)')
        self.set_nav(True)
        self.spin_for(1.5)
        s = self.measured('nav source back', 1.0)
        self.check('E: restart at 0.20', abs(s['raw_mean'] - NAV_SPEED) < TOL)

        print('== F. control_delay_injector killed', flush=True)
        pgid = os.environ.get('SIM_PGID')
        pids = subprocess.run(['pgrep', '-g', pgid, '-f', 'lib/fault_injector/control_delay_injector'],
                              capture_output=True, text=True).stdout.split() if pgid else []
        if not self.check('F: injector pid found in sim process group', len(pids) == 1, f'({pids})'):
            return False
        os.kill(int(pids[0]), signal.SIGKILL)
        t_kill = self.now()
        self.spin_for(1.5)
        s = self.measured('injector killed', 1.0)
        self.check('F: no /cmd_vel_nav after kill', not self.window(self.nav_out, t_kill + 0.2, self.now()))
        self.check('F: cmd_vel 0 via idle_stop', len(s['out']) >= 10 and max(abs(v) for v in s['out']) == 0.0)
        self.check('F: robot stopped', abs(s['raw_mean']) < 0.005 and s['moved'] < 0.005,
                   f'(moved {s["moved"] * 1000:.1f} mm)')

        self.set_nav(False)
        self.spin_for(0.3)
        return all(r[1] for r in self.results)


def main():
    rclpy.init()
    node = Fault3Test()
    try:
        passed = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print('\n==== FAULT 3 CONTROL LATENCY TEST:', 'PASSED' if passed else 'FAILED', '====')
    sys.exit(0 if passed else 1)
