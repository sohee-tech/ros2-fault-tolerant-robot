"""Automated Nav2 lifecycle fault-recovery test (warehouse, 5-waypoint mission).

Requires nav2_warehouse.launch.py mission_config:=nav2_mission_recovery.yaml (runner idle).
  A. NORMAL            all Nav2 servers ACTIVE, mission running
  B. leg 1             controller_server DEACTIVATE -> auto recovery -> mission continues
  C. leg 2             planner_server DEACTIVATE    -> auto recovery -> mission continues
  D. leg 3             controller_server PROCESS CRASH (SIGKILL) -> respawn (new PID) -> recovery
  E. leg 4             recovery blocked: attempts fail -> FAILED, CRITICAL, robot stays stopped
  F. leg 4             unblock -> ACTIVE -> healthy hold -> NORMAL -> mission resumes
  G. leg 5             Fault 2 odom spike: Nav2 velocity feedback is /odom_validated (no 2.5 m/s),
                       DEGRADED 0.10 -> recovery -> delivery zone reached
Lifecycle states are read independently here (GetState) - not only from the recovery manager.
Writes results/nav2_recovery_summary.json / .csv.
"""
import csv
import json
import math
import os
import sys
from datetime import datetime

import numpy as np
import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger

from fault_bringup.fault4_test import pid_alive
from fault_bringup.nav2_demo import Nav2Demo, RESULTS, ROBOT_RADIUS

LABELS = {1: 'UNCONFIGURED', 2: 'INACTIVE', 3: 'ACTIVE', 4: 'FINALIZED'}


class Nav2RecoveryTest(Nav2Demo):
    def __init__(self):
        super().__init__()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.nav2 = {}                 # latest /nav2_recovery/node_states
        self.rec_states = []           # (t, recovery manager state)
        self.nav2_health = []          # (t, health/nav2_state)
        self.attempts = 0
        self.odom_out = []             # (t, vx) on /odom
        self.odom_valid = []           # (t, vx) on /odom_validated
        self.events = []
        self.create_subscription(String, 'nav2_recovery/node_states',
                                 lambda m: setattr(self, 'nav2', json.loads(m.data)), 10)
        self.create_subscription(String, 'nav2_recovery/state', lambda m: self.rec_states.append(
            (self.now(), m.data)), latched)
        self.create_subscription(String, 'health/nav2_state', lambda m: self.nav2_health.append(
            (self.now(), m.data)), latched)
        self.create_subscription(UInt64, 'nav2_recovery/attempt_count',
                                 lambda m: setattr(self, 'attempts', max(self.attempts, m.data)), latched)
        self.create_subscription(Odometry, 'odom', lambda m: self.odom_out.append(
            (self.now(), m.twist.twist.linear.x)), 10)
        self.create_subscription(Odometry, 'odom_validated', lambda m: self.odom_valid.append(
            (self.now(), m.twist.twist.linear.x)), 10)
        for name in ('controller_deactivate', 'planner_deactivate', 'controller_crash',
                     'recovery_block_enable', 'recovery_block_disable'):
            self.cli[f'nav2_fault/{name}'] = self.create_client(Trigger, f'nav2_fault/{name}')
        self.getstate = {n: self.create_client(GetState, f'/{n}/get_state')
                         for n in ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server')}

    # --- helpers ---
    def lifecycle(self, node):
        """Lifecycle state read directly from the node (None = service gone / no answer)."""
        cli = self.getstate[node]
        if not cli.service_is_ready():
            return None
        fut = cli.call_async(GetState.Request())
        self.wait_until(fut.done, 1.0)
        return LABELS.get(fut.result().current_state.id) if fut.done() and fut.result() else None

    def stack_active(self):
        return self.nav2.get('nodes') and all(v['state'] == 'ACTIVE' for v in self.nav2['nodes'].values())

    def nav2_state(self):
        return self.nav2_health[-1][1] if self.nav2_health else None

    def first_after(self, series, t0, value):
        return next((t for t, v in series if t >= t0 and v == value), None)

    def pid(self, node='controller_server'):
        return (self.nav2.get('nodes', {}).get(node) or {}).get('pid')

    def path_len(self, t0, t1):
        pts = [(x, y) for t, x, y, _ in self.poses if t0 <= t <= t1]
        return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))

    def stopped_window(self, label, dur):
        t_a = self.now()
        self.spin_for(dur)
        out = self.window(self.out, t_a, self.now())
        _, d = self.speed(t_a, self.now())
        ok = out and max(map(abs, out)) == 0.0 and d < 0.01
        self.check(f'{label}: safe stop (cmd_vel 0, robot still)', ok,
                   f'(n={len(out)}, moved {d * 1000:.1f} mm in {dur:.1f} s)')
        return d

    def recover_and_resume(self, label, t0, ev):
        """Wait for ACTIVE -> NORMAL -> robot moving; fill timing fields of ev."""
        ok = self.wait_until(lambda: self.stack_active(), 40.0)
        t_active = self.now()
        self.check(f'{label}: all Nav2 servers ACTIVE again', ok,
                   f'({t_active - t0:.2f} s after injection, attempts {self.nav2.get("attempts")})')
        ok = self.wait_until(lambda: self.state() == 'NORMAL' and self.nav2_state() == 'NORMAL', 10.0)
        t_norm = self.now()
        self.check(f'{label}: NORMAL after healthy hold', ok and t_norm - t_active >= 1.8,
                   f'({t_norm - t_active:.2f} s after ACTIVE)')
        ok = self.wait_moving(20.0)
        t_move = self.now()
        self.check(f'{label}: mission resumed (robot moving)', ok and self.status.get('state') in ('RUNNING',),
                   f'({t_move - t_norm:.2f} s after NORMAL, mission {self.status.get("state")}, '
                   f'retries {self.status.get("total_retries")}, fault re-sends {self.status.get("fault_resends")})')
        ev.update({'lifecycle_active_s': round(t_active - t0, 2), 'normal_s': round(t_norm - t0, 2),
                   'mission_resume_s': round(t_move - t0, 2),
                   'stopped_distance_m': round(self.path_len(t0, t_move), 3)})

    def lifecycle_fault(self, label, node, service, crash=False):
        """B / C / D: inject, check the real lifecycle change, detection, stop, recovery, resume."""
        self.wait_moving()
        self.spin_for(2.0)
        pid_before = self.pid()
        n_rec = len(self.rec_states)
        t0 = self.svc(service)
        ev = {'fault': label, 'faulted_node': node, 'failure_type': 'crash' if crash else 'deactivate'}
        if crash:
            ok = self.wait_until(lambda: not pid_alive(pid_before), 3.0)
            t_exit = self.now()
            self.check(f'{label}: old PID {pid_before} terminated', ok)
            observed = self.lifecycle(node)
            self.check(f'{label}: {node} gone from the ROS graph / not ACTIVE', observed != 'ACTIVE',
                       f'(GetState -> {observed or "no service"})')
            ev.update({'old_pid': pid_before})
        else:
            observed = self.lifecycle(node)
            self.check(f'{label}: {node} really left ACTIVE', observed in ('INACTIVE', 'UNCONFIGURED'),
                       f'(GetState -> {observed})')
        ev['observed_state'] = observed or 'MISSING'
        ok = self.wait_until(lambda: self.state() == 'CRITICAL', 6.0)
        t_warn = self.first_after(self.nav2_health, t0, 'WARNING')
        t_crit = self.now()
        self.check(f'{label}: Nav2 health -> CRITICAL', ok,
                   f'(WARNING {t_warn - t0 if t_warn else float("nan"):.2f} s, CRITICAL {t_crit - t0:.2f} s)')
        ev.update({'detection_s': round(t_warn - t0, 2) if t_warn else None, 'safe_stop_s': round(t_crit - t0, 2)})
        self.spin_for(0.5)
        self.stopped_window(label, 1.0)
        if crash:
            ok = self.wait_until(lambda: self.pid() not in (None, pid_before), 15.0)
            pid_after = self.pid()
            self.check(f'{label}: respawned with a new PID', ok and pid_after != pid_before,
                       f'({pid_before} -> {pid_after}, {self.now() - t_exit:.2f} s after exit)')
            ev.update({'new_pid': pid_after, 'respawn_s': round(self.now() - t0, 2)})
        self.recover_and_resume(label, t0, ev)
        ev['recovery_states'] = [s for _, s in self.rec_states[n_rec:]]
        ev['recovery_attempts'] = ev['recovery_states'].count('VERIFY')   # one VERIFY per STARTUP attempt
        self.events.append(ev)

    # --- scenario ---
    def run(self):
        self.phase('Nav2 recovery test: startup')
        ok = self.wait_until(lambda: self.status.get('mode') == 'NAV2' and self.state() == 'NORMAL'
                             and self.stack_active() and self.cli['mission/start'].service_is_ready(), 60.0)
        if not self.check('A: Nav2 stack ACTIVE, runner ready', ok,
                          f'({ {k: v["state"] for k, v in self.nav2.get("nodes", {}).items()} })'):
            return False
        subs = [t for t, _ in self.get_subscriber_names_and_types_by_node('controller_server', '/')]
        self.check('A: controller_server velocity feedback = /odom_validated',
                   '/odom_validated' in subs and '/odom' not in subs, f'(odom topics {[t for t in subs if "odom" in t]})')
        t_start = self.svc('mission/start')
        ok = self.wait_moving()
        self.check('A: mission running', ok and self.status.get('state') == 'RUNNING')

        self.phase('B. controller_server DEACTIVATE (leg 1)')
        self.lifecycle_fault('B controller deactivate', 'controller_server', 'nav2_fault/controller_deactivate')

        self.phase('Mission running')
        self.check('waypoint 1 reached', self.wait_until(lambda: self.reached() >= 1, 120.0))
        self.phase('C. planner_server DEACTIVATE (leg 2)')
        self.lifecycle_fault('C planner deactivate', 'planner_server', 'nav2_fault/planner_deactivate')

        self.phase('Mission running')
        self.check('waypoint 2 reached', self.wait_until(lambda: self.reached() >= 2, 120.0))
        self.phase('D. controller_server PROCESS CRASH (leg 3)')
        self.lifecycle_fault('D controller crash', 'controller_server', 'nav2_fault/controller_crash', crash=True)

        self.phase('Mission running')
        self.check('waypoint 3 reached', self.wait_until(lambda: self.reached() >= 3, 120.0))
        self.phase('E. recovery BLOCKED (leg 4)')
        self.wait_moving()
        self.spin_for(2.0)
        self.svc('nav2_fault/recovery_block_enable')
        t0 = self.svc('nav2_fault/controller_deactivate')
        ok = self.wait_until(lambda: self.rec_states and self.rec_states[-1][1] == 'FAILED', 60.0)
        t_failed = self.now()
        self.check('E: recovery FAILED after max attempts', ok,
                   f'({t_failed - t0:.1f} s, attempts {self.nav2.get("attempts")})')
        self.check('E: system CRITICAL', self.state() == 'CRITICAL')
        moved = self.stopped_window('E blocked', 5.0)
        self.check('E: still CRITICAL / FAILED after 5 s', self.state() == 'CRITICAL'
                   and self.rec_states[-1][1] == 'FAILED')
        ev = {'fault': 'E recovery blocked', 'faulted_node': 'controller_server', 'failure_type': 'deactivate + blocked',
              'recovery_attempts': self.nav2.get('attempts'), 'failed_after_s': round(t_failed - t0, 2),
              'moved_while_failed_mm': round(moved * 1000, 1)}

        self.phase('F. recovery UNBLOCKED (leg 4)')
        t1 = self.svc('nav2_fault/recovery_block_disable')
        self.recover_and_resume('F unblock', t1, ev)
        ev['mission_resume_s'] = round(t1 + ev['mission_resume_s'] - t0, 2)     # relative to the injection
        ev['stopped_distance_m'] = round(self.path_len(t0, t0 + ev['mission_resume_s']), 3)
        ev['unblock_to_resume_s'] = round(t0 + ev['mission_resume_s'] - t1, 2)
        self.events.append(ev)

        self.phase('Mission running')
        self.check('waypoint 4 reached', self.wait_until(lambda: self.reached() >= 4, 150.0))
        self.phase('G. Fault 2 odom spike with validated odometry (leg 5)')
        self.wait_moving()
        self.spin_for(1.5)
        t0 = self.svc('odom_fault/enable')
        ok = self.wait_until(lambda: self.state() == 'DEGRADED', 5.0)
        self.check('G: system DEGRADED', ok, f'({self.now() - t0:.2f} s)')
        self.spin_for(3.0)
        corrupt = [v for v in self.window(self.odom_out, t0, self.now())]
        valid = [v for v in self.window(self.odom_valid, t0, self.now())]
        out = self.window(self.out, self.now() - 2.0, self.now())
        self.check('G: /odom carries the injected 2.5 m/s', corrupt and max(corrupt) > 2.0, f'(max {max(corrupt):.2f})')
        self.check('G: /odom_validated (Nav2 feedback) has no invalid sample',
                   max(valid, default=0.0) < 0.5, f'({len(valid)} samples, max {max(valid, default=0.0):.3f})')
        self.check('G: final cmd_vel <= 0.10', out and max(out) <= 0.1 + 1e-6, f'(max {max(out, default=0):.3f})')
        t_off = self.svc('odom_fault/disable')
        ok = self.wait_until(lambda: self.state() == 'NORMAL', 6.0)
        self.check('G: recovered to NORMAL', ok, f'({self.now() - t_off:.2f} s)')
        self.events.append({'fault': 'G odom spike', 'faulted_node': '-', 'failure_type': 'odom 2.5 m/s',
                            'odom_max_mps': round(max(corrupt), 2), 'odom_validated_max_mps': round(max(valid, default=0), 3),
                            'final_cmd_max_mps': round(max(out, default=0), 3)})

        self.phase('Mission running - to delivery zone')
        ok = self.wait_until(lambda: self.status.get('state') in ('COMPLETE', 'FAILED'), 200.0)
        t_end = self.now()
        done = ok and self.status.get('state') == 'COMPLETE'
        self.check('mission COMPLETE (delivery zone reached)', done, f'({self.status.get("state")})')
        _, gx, gy, _ = self.poses[-1]
        err = math.dist((gx, gy), self.status.get('goal', [0, 0])[:2])
        self.check('goal position error < 0.25 m', err < 0.25, f'({err:.3f} m)')
        pts = np.array([(x, y) for t, x, y, _ in self.poses if t_start <= t <= t_end][::3])
        clear = np.array([np.min(np.hypot(*(self.obstacles - p).T)) for p in pts]) - 0.025
        self.check('no collision (map clearance > robot radius)', clear.min() > ROBOT_RADIUS,
                   f'(min clearance {clear.min():.3f} m)')
        self.summary = {'mission_success': done, 'mission_time_s': round(t_end - t_start, 1),
                        'waypoints_reached': self.reached(), 'waypoints_total': self.status.get('total'),
                        'waypoint_results': self.status.get('results', []),
                        'retries': self.status.get('total_retries'), 'fault_resends': self.status.get('fault_resends'),
                        'goal_position_error_m': round(err, 3), 'min_clearance_m': round(float(clear.min()), 3),
                        'events': self.events}
        self.phase('Nav2 recovery test complete' if done else 'Nav2 recovery test FAILED')
        return all(r[1] for r in self.results)

    def write_summary(self, passed):
        os.makedirs(RESULTS, exist_ok=True)
        doc = {'generated': datetime.now().isoformat(timespec='seconds'), 'world': 'amr_warehouse',
               'test_passed': passed, 'checks_passed': sum(r[1] for r in self.results),
               'checks_total': len(self.results), 'failed_checks': [r[0] for r in self.results if not r[1]],
               'time_base': 'sim time [s] relative to the fault injection; distances from Gazebo /odom_raw',
               'results': getattr(self, 'summary', {'events': self.events})}
        with open(os.path.join(RESULTS, 'nav2_recovery_summary.json'), 'w') as fp:
            json.dump(doc, fp, indent=2)
        keys = ['fault', 'faulted_node', 'failure_type', 'observed_state', 'detection_s', 'safe_stop_s',
                'recovery_attempts', 'old_pid', 'new_pid', 'respawn_s', 'lifecycle_active_s', 'normal_s',
                'mission_resume_s', 'stopped_distance_m', 'failed_after_s', 'moved_while_failed_mm',
                'odom_max_mps', 'odom_validated_max_mps', 'final_cmd_max_mps']
        with open(os.path.join(RESULTS, 'nav2_recovery_summary.csv'), 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerow(keys)
            for ev in doc['results'].get('events', []):
                w.writerow([ev.get(k, '') for k in keys])
            s = doc['results']
            w.writerow([f'mission_success={s.get("mission_success")}', f'time={s.get("mission_time_s")}s',
                        f'waypoints={s.get("waypoints_reached")}/{s.get("waypoints_total")}',
                        f'retries={s.get("retries")}', f'fault_resends={s.get("fault_resends")}',
                        f'checks={doc["checks_passed"]}/{doc["checks_total"]}'] + [''] * (len(keys) - 6))
        print(f'summary written to {RESULTS}/nav2_recovery_summary.json and .csv', flush=True)


def main():
    rclpy.init()
    node = Nav2RecoveryTest()
    passed = False
    try:
        passed = node.run()
    finally:
        node.write_summary(passed)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'\n==== NAV2 RECOVERY TEST: {"PASSED" if passed else "FAILED"} '
          f'({sum(r[1] for r in node.results)}/{len(node.results)} checks) ====')
    sys.exit(0 if passed else 1)
