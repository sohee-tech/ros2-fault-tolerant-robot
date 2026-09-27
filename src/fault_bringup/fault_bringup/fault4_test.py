"""Automated Fault 4 (Navigation Command Source Failure) test. Requires fault_sim.launch.py.

nav_command_source is a real process started by launch with respawn=True. The test only
requests driving (latched /nav_source/active_request) and triggers faults.
A. NORMAL             process alive, heartbeat, nav 0.2, cmd_vel 0.2, odom 0.2
B. PROCESS CRASH      /nav_fault/crash -> PID really gone, heartbeat + nav command stop
C. SAFE STOP          nav WARNING -> DEGRADED -> CRITICAL, cmd_vel 0, odom 0
D. RESPAWN            new PID in the sim process group, heartbeat + nav command resume
E. RECOVERY HOLD      not NORMAL right after respawn; NORMAL after ~2 s healthy heartbeat
F. AUTO RESUME        cmd_vel 0.2, odom 0.2
G. CRASH LOOP         repeated restarts keep the system CRITICAL and the robot stopped;
                      after disabling, the next process stays up and driving resumes
"""
import os
import subprocess
import sys

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger

from fault_bringup.fault1_test import NAV_SPEED
from fault_bringup.fault2_test import Fault2Test, TOL


def pid_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def nav_pids_in_sim():
    """PIDs of nav_command_source inside THIS simulation's process group."""
    pgid = os.environ.get('SIM_PGID')
    if not pgid:
        return []
    out = subprocess.run(['pgrep', '-g', pgid, '-f', 'lib/fault_bringup/nav_command_source'],
                         capture_output=True, text=True).stdout
    return [int(p) for p in out.split()]


class Fault4Test(Fault2Test):
    def __init__(self):
        super().__init__()
        self.hb = []              # (t, pid) on /nav/heartbeat
        self.safety = []          # (t, x) on /cmd_vel_safety
        self.nav_states = []
        self.restarts = []        # (t, count) on /health/nav_restart_count (monitor saw a new PID)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(UInt64, 'nav/heartbeat', lambda m: self.hb.append((self.now(), m.data)), 10)
        self.create_subscription(Twist, 'cmd_vel_safety', lambda m: self.safety.append((self.now(), m.linear.x)), 10)
        self.create_subscription(String, 'health/nav_state', self.on_nav_state, latched)
        self.create_subscription(UInt64, 'health/nav_restart_count',
                                 lambda m: self.restarts.append((self.now(), m.data)), latched)
        self.cli_crash = self.create_client(Trigger, 'nav_fault/crash')
        self.cli_loop_on = self.create_client(Trigger, 'nav_fault/crash_loop_enable')
        self.cli_loop_off = self.create_client(Trigger, 'nav_fault/crash_loop_disable')

    def on_nav_state(self, msg):
        if not self.nav_states or self.nav_states[-1][1] != msg.data:
            self.nav_states.append((self.now(), msg.data))
            print(f'  nav -> {msg.data} (t={self.now():.2f})', flush=True)

    def nav_state(self):
        return self.nav_states[-1][1] if self.nav_states else None

    def hb_pid(self):
        return self.hb[-1][1] if self.hb else None

    def stopped(self, label, s):
        zero = s['out'] and max(abs(v) for v in s['out']) == 0.0
        self.check(f'{label}: cmd_vel = 0', zero, f'(n={len(s["out"])})')
        self.check(f'{label}: robot stopped (odom)', abs(s['raw_mean']) < 0.005 and s['moved'] < 0.005,
                   f'(odom_raw {s["raw_mean"]:.4f}, moved {s["moved"] * 1000:.1f} mm)')

    def driving(self, label, s):
        self.check(f'{label}: cmd_vel 0.20', s['out'] and min(s['out']) == NAV_SPEED)
        self.check(f'{label}: odom ~0.20', abs(s['raw_mean'] - NAV_SPEED) < TOL
                   and abs(s['speed'] - NAV_SPEED) < TOL,
                   f'(odom_raw {s["raw_mean"]:.3f}, pose {s["speed"]:.3f})')

    def run(self):
        print('== A. NORMAL', flush=True)
        ok = self.wait_until(lambda: len(self.scan_times) >= 10 and self.odom is not None
                             and self.hb and self.nav_state() is not None, 30.0)
        if not self.check('sim ready (scan, odom, heartbeat)', ok):
            return False
        self.spin_for(1.5)
        t0 = self.now()
        s = self.raw_snapshot('normal', 1.0)
        pid_before = self.hb_pid()
        procs = nav_pids_in_sim()
        n_hb = len(self.window(self.hb, t0, self.now()))
        self.check('A: nav process alive (heartbeat PID == process in sim group)',
                   procs == [pid_before], f'(heartbeat pid {pid_before}, pgrep {procs})')
        self.check('A: heartbeat ~10 Hz', n_hb >= 8, f'({n_hb} in 1 s)')
        self.check('A: nav state NORMAL', self.nav_state() == 'NORMAL' and self.state() == 'NORMAL')
        self.check('A: cmd_vel_nav_raw 0.20', s['nav'] and min(s['nav']) == NAV_SPEED)
        self.driving('A', s)

        print('== B. PROCESS CRASH', flush=True)
        n_nav = len(self.nav_states)
        res = self.call(self.cli_crash)
        print(f'  crash: {res.message}', flush=True)
        ok = self.wait_until(lambda: not pid_alive(pid_before), 3.0)
        t_exit = self.now()
        self.check('B: nav process really terminated', ok, f'(pid {pid_before} gone)')
        last_hb = max(t for t, pid in self.hb if pid == pid_before)
        self.spin_for(0.8)
        self.check('B: heartbeat stopped', not self.window(self.hb, t_exit + 0.05, self.now()))
        self.check('B: cmd_vel_nav_raw stopped', not self.window(self.nav, t_exit + 0.05, self.now()))

        print('== C. SAFE STOP', flush=True)
        # nav and system state arrive on separate topics; wait for both
        ok = self.wait_until(lambda: self.nav_state() == 'CRITICAL' and self.state() == 'CRITICAL', 5.0)
        self.check('C: nav CRITICAL', ok)
        seq = [st for _, st in self.nav_states[n_nav - 1:]]
        self.check('C: nav WARNING -> DEGRADED -> CRITICAL',
                   seq == ['NORMAL', 'WARNING', 'DEGRADED', 'CRITICAL'], f'({seq})')
        for t, st in self.nav_states[n_nav:]:
            print(f'  {st:9s} at heartbeat age {t - last_hb:.2f} s', flush=True)
        self.check('C: system CRITICAL', self.state() == 'CRITICAL')
        s = self.raw_snapshot('crashed', 1.0)
        self.stopped('C', s)
        self.check('C: safety channel active', len(self.window(self.safety, self.now() - 1.0, self.now())) >= 15)

        print('== D. RESPAWN', flush=True)
        ok = self.wait_until(lambda: self.hb_pid() not in (None, pid_before), 8.0)
        t_first_hb = next((t for t, pid in self.hb if pid != pid_before and t > t_exit), None)
        pid_after = self.hb_pid()
        self.check('D: new heartbeat PID', ok and pid_after != pid_before,
                   f'(before {pid_before}, after {pid_after}, respawn {t_first_hb - t_exit:.2f} s after exit)'
                   if ok else '')
        self.check('D: new PID is a live process in sim group', nav_pids_in_sim() == [pid_after],
                   f'(pgrep {nav_pids_in_sim()})')
        ok = self.wait_until(lambda: self.window(self.nav, t_first_hb, self.now()), 2.0) if t_first_hb else False
        self.check('D: cmd_vel_nav_raw resumed', ok)

        print('== E. RECOVERY HOLD', flush=True)
        self.wait_until(lambda: False, max(0.0, 1.0 - (self.now() - t_first_hb)))
        s = self.raw_snapshot('respawned (holding)', 0.5)
        self.check('E: not NORMAL right after respawn', self.nav_state() == 'CRITICAL',
                   f'(nav {self.nav_state()} {self.now() - t_first_hb:.2f} s after first heartbeat)')
        self.stopped('E (hold)', s)
        ok = self.wait_until(lambda: self.nav_state() == 'NORMAL' and self.state() == 'NORMAL', 5.0)
        t_norm = self.nav_states[-1][0]
        self.check('E: NORMAL after healthy hold', ok)
        # Reference = when the MONITOR first saw the new PID (DDS discovery of the new process can
        # reach the monitor later than this test node).
        t_seen = next((t for t, n in self.restarts if n == 1), None)
        self.check('E: monitor counted exactly 1 restart', t_seen is not None and self.restarts[-1][1] == 1,
                   f'(restart_count {self.restarts[-1][1] if self.restarts else None})')
        self.check('E: hold ~2 s', ok and t_seen is not None and 1.95 <= t_norm - t_seen <= 2.6,
                   f'({t_norm - t_seen:.2f} s after monitor saw new heartbeat, '
                   f'{t_norm - t_first_hb:.2f} s after test saw it)' if t_seen else '')

        print('== F. AUTO RESUME', flush=True)
        self.spin_for(1.0)
        s = self.raw_snapshot('resumed', 1.0)
        self.driving('F', s)
        self.check('F: safety released', not self.window(self.safety, self.now() - 1.0, self.now()))

        print('== G. CRASH LOOP', flush=True)
        n_pids = len({pid for _, pid in self.hb})
        n_nav = len(self.nav_states)
        print(f'  {self.call(self.cli_loop_on).message}', flush=True)
        t_loop = self.now()
        ok = self.wait_until(lambda: self.nav_state() == 'CRITICAL', 6.0)
        self.check('G: nav CRITICAL under crash loop', ok)
        t_crit = self.now()
        x0, y0, _ = self.odom
        self.spin_for(10.0)
        x1, y1, _ = self.odom
        pids_seen = len({pid for _, pid in self.hb}) - n_pids
        self.check('G: repeated restarts', pids_seen >= 2, f'({pids_seen} new PIDs in 10 s)')
        after = [st for t, st in self.nav_states if t > t_crit]
        self.check('G: stays CRITICAL (no NORMAL)', 'NORMAL' not in after and self.state() == 'CRITICAL',
                   f'(nav states after CRITICAL: {after})')
        out = self.window(self.out, t_crit + 0.5, self.now())
        moved = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        self.check('G: robot kept stopped', out and max(abs(v) for v in out) == 0.0 and moved < 0.01,
                   f'(moved {moved * 1000:.1f} mm in 10 s)')
        print(f'  {self.call(self.cli_loop_off).message}', flush=True)
        ok = self.wait_until(lambda: self.nav_state() == 'NORMAL' and self.state() == 'NORMAL', 12.0)
        self.check('G: NORMAL after crash loop disabled', ok)
        pid_final = self.hb_pid()
        self.spin_for(1.0)
        s = self.raw_snapshot('after crash loop', 1.0)
        self.check('G: final process stays up', nav_pids_in_sim() == [pid_final] and self.hb_pid() == pid_final,
                   f'(pid {pid_final})')
        self.driving('G', s)
        print(f'  PIDs: before crash {pid_before}, after respawn {pid_after}, final {pid_final}', flush=True)

        self.set_nav(False)
        self.spin_for(0.3)
        return all(r[1] for r in self.results)


def main():
    rclpy.init()
    node = Fault4Test()
    try:
        passed = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print('\n==== FAULT 4 NAV COMMAND SOURCE FAILURE TEST:', 'PASSED' if passed else 'FAILED', '====')
    sys.exit(0 if passed else 1)
