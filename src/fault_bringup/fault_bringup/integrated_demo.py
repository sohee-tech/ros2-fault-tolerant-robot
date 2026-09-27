"""Integrated demo: Fault 1-4 in sequence while the robot drives, with checks and a summary.

Run with fault_sim.launch.py demo:=true (robot circles between four pillars) and
dashboard:=true. The current phase is published on /demo/phase (latched) and narrative
events on /demo/event so the dashboard shows them.

A. NORMAL driving
B. Fault 1  LiDAR dropout        -> CRITICAL stop -> recovery
C. Fault 2  odometry spike        -> single spike rejected; continuous -> DEGRADED 0.10 -> recovery
D. Fault 3  control latency       -> 300 ms DEGRADED 0.10 -> 700 ms CRITICAL stop -> recovery
E. Fault 4  nav process crash     -> stop -> respawn (new PID) -> hold -> NORMAL

Writes results/final_summary.json and results/final_summary.csv (sim-time based metrics).
"""
import csv
import json
import math
import os
import subprocess
import sys
from datetime import datetime

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from fault_bringup.fault1_test import NAV_SPEED
from fault_bringup.fault2_test import LIMIT
from fault_bringup.fault4_test import Fault4Test, nav_pids_in_sim, pid_alive

RESULTS = os.path.expanduser('~/capstone_fault_ws/results')


def uptime():
    """Seconds since boot (/proc/uptime, 10 ms resolution)."""
    with open('/proc/uptime') as fp:
        return float(fp.read().split()[0])


def proc_start_uptime(pid):
    """Start time of a process in seconds since boot (/proc/<pid>/stat field 22)."""
    with open(f'/proc/{pid}/stat') as fp:
        start_ticks = int(fp.read().rsplit(')', 1)[1].split()[19])
    return start_ticks / os.sysconf('SC_CLK_TCK')
TOL = 0.03   # speed tolerance on the circular demo path


class IntegratedDemo(Fault4Test):
    def __init__(self):
        super().__init__()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_phase = self.create_publisher(String, 'demo/phase', latched)
        self.pub_event = self.create_publisher(String, 'demo/event', 10)
        self.latency = []
        self.control_states = []
        self.poses = []   # (t, x, y) from /odom_raw
        self.create_subscription(Float64, 'control/latency_sec',
                                 lambda m: self.latency.append((self.now(), m.data)), 10)
        self.create_subscription(String, 'health/control_state', lambda m: self.control_states.append(
            (self.now(), m.data)), latched)
        self.create_subscription(Odometry, 'odom_raw', lambda m: self.poses.append(
            (self.now(), m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.cli = {name: self.create_client(Trigger, name) for name in (
            'lidar_fault/enable', 'lidar_fault/disable', 'odom_fault/spike', 'odom_fault/enable',
            'odom_fault/disable', 'control_delay/degraded', 'control_delay/critical',
            'control_delay/disable')}
        self.summary = {}

    # --- helpers ------------------------------------------------------------
    def phase(self, text):
        print(f'\n######## {text}', flush=True)
        self.pub_phase.publish(String(data=text))

    def say(self, text):
        print(f'  > {text}', flush=True)
        self.pub_event.publish(String(data=text))

    def svc(self, name):
        res = self.call(self.cli[name])
        self.say(f'/{name}: {res.message if res else "no response"}')
        return self.now()

    def path_length(self, t0, t1):
        pts = [(x, y) for t, x, y in self.poses if t0 <= t <= t1]
        return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))

    def state_times(self, series, t0):
        """{state: seconds after t0} for the first entry of each state after t0."""
        out = {}
        for t, st in series:
            if t >= t0 and st not in out:
                out[st] = round(t - t0, 2)
        return out

    def wait_state(self, target, timeout, getter=None):
        getter = getter or self.state
        ok = self.wait_until(lambda: getter() == target, timeout)
        return ok, self.now()

    def wait_normal(self, timeout):
        """System, and every subsystem topic we track, back to NORMAL."""
        return self.wait_until(lambda: self.state() == 'NORMAL' and self.nav_state() == 'NORMAL'
                               and (not self.control_states or self.control_states[-1][1] == 'NORMAL'),
                               timeout)

    def cruise(self, label, seconds=2.0):
        self.spin_for(max(0.0, seconds - 1.0))
        s = self.raw_snapshot(label, 1.0)
        ok = abs(s['raw_mean'] - NAV_SPEED) < TOL and s['out'] and min(s['out']) == NAV_SPEED
        self.check(f'{label}: NORMAL driving 0.20 m/s', ok and self.state() == 'NORMAL',
                   f'(cmd {min(s["out"]) if s["out"] else None}, odom {s["raw_mean"]:.3f})')
        return s

    def stop_check(self, label, dur=1.5):
        s = self.raw_snapshot(label, dur)
        stopped = s['out'] and max(abs(v) for v in s['out']) == 0.0 and abs(s['raw_mean']) < 0.005 \
            and s['moved'] < 0.005
        self.check(f'{label}: stopped (cmd 0, odom 0)', stopped,
                   f'(odom {s["raw_mean"]:.4f}, moved {s["moved"] * 1000:.1f} mm in {dur:.1f} s)')
        return s

    # --- demo ----------------------------------------------------------------
    def run(self):
        self.phase('Starting up')
        ok = self.wait_until(lambda: len(self.scan_times) >= 10 and self.odom is not None and self.hb
                             and self.nav_state() is not None and self.state() == 'NORMAL', 30.0)
        if not self.check('system ready', ok):
            return False

        self.phase('A. NORMAL driving')
        self.say('driving 0.20 m/s on a 0.4 m circle')
        self.spin_for(2.0)
        s = self.cruise('A normal', 2.0)
        self.summary['normal'] = {'nav_cmd': NAV_SPEED, 'final_cmd': min(s['out']),
                                  'odom_mps': round(s['raw_mean'], 3)}

        # ---------------- Fault 1 ----------------
        self.phase('B. Fault 1 - LiDAR dropout')
        n_states = len(self.states)
        t0 = self.svc('lidar_fault/enable')
        ok, _ = self.wait_state('CRITICAL', 5.0)
        self.check('F1: reached CRITICAL', ok)
        times = self.state_times(self.states[n_states:], t0)
        seq = [st for _, st in self.states[n_states:]]
        self.check('F1: WARNING -> DEGRADED -> CRITICAL', seq == ['WARNING', 'DEGRADED', 'CRITICAL'], f'({seq})')
        self.spin_for(0.5)
        s = self.stop_check('F1 CRITICAL')
        stop_dist = self.path_length(t0, self.now())
        t_off = self.svc('lidar_fault/disable')
        self.wait_until(lambda: self.scan_times[-1] > t_off, 3.0)
        t_scan = next(t for t in self.scan_times if t > t_off)
        ok = self.wait_normal(6.0)
        t_norm = self.now()
        self.check('F1: recovered to NORMAL', ok)
        s2 = self.cruise('F1 resumed')
        self.summary['fault1_lidar'] = {
            'injection': 'LiDAR /scan relay stopped',
            'to_warning_s': times.get('WARNING'), 'to_degraded_s': times.get('DEGRADED'),
            'to_critical_s': times.get('CRITICAL'),
            'stop_final_cmd': max(abs(v) for v in s['out']), 'stop_odom_mps': round(s['raw_mean'], 4),
            'stopped_window_moved_mm': round(s['moved'] * 1000, 1),
            'distance_after_injection_m': round(stop_dist, 3),
            'recovery_hold_s': round(t_norm - t_scan, 2), 'resumed_odom_mps': round(s2['raw_mean'], 3)}

        # ---------------- Fault 2 ----------------
        self.phase('C. Fault 2 - Odometry anomaly')
        n_states = len(self.states)
        t_spike = self.svc('odom_fault/spike')
        self.spin_for(1.5)
        spikes = [v for v in self.window(self.odom_out, t_spike, self.now()) if v > 1.0]
        rejected = [v for v in self.window(self.valid, t_spike, self.now()) if not v]
        spike_changed_state = len(self.states) != n_states
        self.check('F2: single spike rejected, state unchanged',
                   len(spikes) == 1 and len(rejected) == 1 and not spike_changed_state,
                   f'(spikes {spikes}, rejected {len(rejected)})')
        t0 = self.svc('odom_fault/enable')
        ok, t_deg = self.wait_state('DEGRADED', 3.0)
        self.check('F2: DEGRADED', ok)
        self.spin_for(1.0)
        s = self.raw_snapshot('F2 DEGRADED', 2.0)
        self.check('F2: speed limited to 0.10',
                   s['out'] and all(abs(v - LIMIT) < 1e-6 for v in s['out']) and abs(s['raw_mean'] - LIMIT) < TOL,
                   f'(cmd {s["out"][-1] if s["out"] else None}, odom {s["raw_mean"]:.3f}, /odom max {s["odom_out_max"]:.2f})')
        t_off = self.svc('odom_fault/disable')
        ok = self.wait_normal(6.0)
        t_norm = self.now()
        self.check('F2: recovered to NORMAL', ok)
        s2 = self.cruise('F2 resumed')
        self.summary['fault2_odometry'] = {
            'injected_vx_mps': round(max(spikes), 2) if spikes else None,
            'real_vx_mps': NAV_SPEED,
            'single_spike_rejected_samples': len(rejected), 'single_spike_state_change': spike_changed_state,
            'to_degraded_s': round(t_deg - t0, 2),
            'speed_before_mps': self.summary['normal']['odom_mps'],
            'degraded_final_cmd': s['out'][-1] if s['out'] else None, 'degraded_odom_mps': round(s['raw_mean'], 3),
            'corrupted_odom_max_mps': round(s['odom_out_max'], 2),
            'recovery_s': round(t_norm - t_off, 2), 'resumed_odom_mps': round(s2['raw_mean'], 3)}

        # ---------------- Fault 3 ----------------
        self.phase('D. Fault 3 - Control latency')
        n_ctrl = len(self.control_states)
        t0 = self.svc('control_delay/degraded')
        ok = self.wait_until(lambda: self.control_states[-1][1] == 'DEGRADED', 3.0)
        t_deg = self.now()
        self.check('F3: 300 ms -> DEGRADED', ok)
        self.spin_for(1.0)
        t_a = self.now()
        s = self.raw_snapshot('F3 300 ms', 1.5)
        lat300 = self.window(self.latency, t_a, self.now())
        self.check('F3: speed limited to 0.10', s['out'] and all(abs(v - LIMIT) < 1e-6 for v in s['out'])
                   and abs(s['raw_mean'] - LIMIT) < TOL, f'(odom {s["raw_mean"]:.3f})')
        t1 = self.svc('control_delay/critical')
        ok = self.wait_until(lambda: self.control_states[-1][1] == 'CRITICAL', 4.0)
        t_crit = self.now()
        self.check('F3: 700 ms -> CRITICAL', ok)
        self.spin_for(0.5)
        t_b = self.now()
        s3 = self.stop_check('F3 CRITICAL')
        lat700 = self.window(self.latency, t_b, self.now())
        seq = [st for _, st in self.control_states[n_ctrl:]]
        self.check('F3: WARNING -> DEGRADED -> CRITICAL', seq == ['WARNING', 'DEGRADED', 'CRITICAL'], f'({seq})')
        t_req = self.now()
        t_off = self.svc('control_delay/disable')
        ok = self.wait_normal(6.0)
        t_norm = self.now()
        t_healthy = next((t for t, v in self.latency if t >= t_req and v < 0.10), t_off)
        stale = [v for v in self.window(self.latency, t_off + 0.05, t_norm) if v >= 0.10]
        self.check('F3: recovered, no stale burst', ok and not stale, f'({len(stale)} stale)')
        s2 = self.cruise('F3 resumed')
        mean = lambda xs: round(sum(xs) / len(xs), 3) if xs else None
        self.summary['fault3_latency'] = {
            'injected_degraded_s': 0.30, 'measured_degraded_s': mean(lat300),
            'injected_critical_s': 0.70, 'measured_critical_s': mean(lat700),
            'to_degraded_s': round(t_deg - t0, 2), 'to_critical_s': round(t_crit - t1, 2),
            'speed_normal_mps': self.summary['fault2_odometry']['resumed_odom_mps'],
            'speed_degraded_mps': round(s['raw_mean'], 3), 'speed_critical_mps': round(s3['raw_mean'], 4),
            'critical_final_cmd': max(abs(v) for v in s3['out']),
            'stale_commands_after_disable': len(stale),
            'recovery_hold_s': round(t_norm - t_healthy, 2), 'resumed_odom_mps': round(s2['raw_mean'], 3)}

        # ---------------- Fault 4 ----------------
        self.phase('E. Fault 4 - Navigation process crash')
        n_nav = len(self.nav_states)
        n_restart = self.restarts[-1][1] if self.restarts else 0
        pid_before = self.hb_pid()
        t0 = self.now()
        res = self.call(self.cli_crash)
        self.say(f'/nav_fault/crash: {res.message}')
        ok = self.wait_until(lambda: not pid_alive(pid_before), 3.0)
        t_exit = self.now()
        t_exit_up = uptime()
        self.check('F4: process terminated', ok, f'(pid {pid_before})')
        last_hb = max(t for t, pid in self.hb if pid == pid_before)
        ok, _ = self.wait_state('CRITICAL', 5.0, self.nav_state)
        self.check('F4: nav CRITICAL', ok)
        times = self.state_times(self.nav_states[n_nav:], last_hb)
        s = self.stop_check('F4 stopped', 1.0)
        ok = self.wait_until(lambda: self.hb_pid() not in (None, pid_before), 8.0)
        # actual process restart by launch (wall clock, from the new PID's /proc start time)
        new = [p for p in nav_pids_in_sim() if p != pid_before]
        respawn_proc = round(proc_start_uptime(new[0]) - t_exit_up, 2) if new else None
        pid_after = self.hb_pid()
        t_first = next((t for t, pid in self.hb if pid == pid_after), self.now())
        self.check('F4: respawned with new PID', ok and pid_after != pid_before,
                   f'({pid_before} -> {pid_after})')
        self.say(f'nav_command_source respawned: pid {pid_before} -> {pid_after}')
        ok = self.wait_normal(6.0)
        t_norm = self.now()
        t_seen = next((t for t, n in self.restarts if n == n_restart + 1), t_first)
        self.check('F4: recovered to NORMAL after hold', ok and t_norm - t_seen >= 1.9,
                   f'({t_norm - t_seen:.2f} s after monitor saw new PID)')
        stop_dist = self.path_length(t0, t_norm)
        s2 = self.cruise('F4 resumed')
        self.summary['fault4_navigation'] = {
            'crash_pid': pid_before, 'respawn_pid': pid_after,
            'hb_age_at_warning_s': times.get('WARNING'), 'hb_age_at_degraded_s': times.get('DEGRADED'),
            'hb_age_at_critical_s': times.get('CRITICAL'),
            'respawn_process_s': respawn_proc,
            'restart_detected_by_monitor_s': round(t_seen - t_exit, 2),
            'recovery_hold_s': round(t_norm - t_seen, 2),
            'distance_crash_to_normal_m': round(stop_dist, 3),
            'stopped_window_moved_mm': round(s['moved'] * 1000, 1),
            'resumed_odom_mps': round(s2['raw_mean'], 3)}

        self.phase('Demo complete - all faults recovered')
        self.say('demo complete')
        self.spin_for(1.0)
        self.set_nav(False)
        self.spin_for(0.5)
        return all(r[1] for r in self.results)

    def write_summary(self, passed, started):
        os.makedirs(RESULTS, exist_ok=True)
        try:
            commit = subprocess.run(['git', '-C', os.path.expanduser('~/capstone_fault_ws'), 'rev-parse',
                                     '--short', 'HEAD'], capture_output=True, text=True).stdout.strip()
        except OSError:
            commit = ''
        doc = {'generated': datetime.now().isoformat(timespec='seconds'), 'started': started,
               'git_commit': commit, 'demo_passed': passed,
               'checks_passed': sum(r[1] for r in self.results), 'checks_total': len(self.results),
               'failed_checks': [r[0] for r in self.results if not r[1]],
               'time_base': 'sim time (s); speeds from Gazebo /odom_raw (ground truth)',
               'results': self.summary}
        with open(os.path.join(RESULTS, 'final_summary.json'), 'w') as fp:
            json.dump(doc, fp, indent=2)
        with open(os.path.join(RESULTS, 'final_summary.csv'), 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerow(['section', 'metric', 'value'])
            for section, metrics in self.summary.items():
                for k, v in metrics.items():
                    w.writerow([section, k, v])
            w.writerow(['demo', 'checks', f'{doc["checks_passed"]}/{doc["checks_total"]}'])
            w.writerow(['demo', 'passed', passed])
        print(f'summary written to {RESULTS}/final_summary.json and .csv', flush=True)


def main():
    rclpy.init()
    node = IntegratedDemo()
    started = datetime.now().isoformat(timespec='seconds')
    passed = False
    try:
        passed = node.run()
    finally:
        node.write_summary(passed, started)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'\n==== INTEGRATED DEMO: {"PASSED" if passed else "FAILED"} '
          f'({sum(r[1] for r in node.results)}/{len(node.results)} checks) ====')
    sys.exit(0 if passed else 1)
