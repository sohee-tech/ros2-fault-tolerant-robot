"""Automated Nav2 warehouse demo with Fault 1/2/3 injected during the waypoint mission.

Requires nav2_warehouse.launch.py (mission runner idle). Scenario:
  mission start -> waypoint 1 reached
  leg 2: LiDAR dropout   -> CRITICAL safe stop -> recovery -> mission resumes
  leg 3: odom anomaly    -> DEGRADED, Nav2 command limited to 0.10 -> recovery -> 0.20 again
  leg 4: control latency -> 300 ms DEGRADED -> 700 ms CRITICAL stop -> recovery -> resume
  mission complete at the delivery zone
Ground truth for speed / position / collisions: Gazebo /odom_raw (world frame == map frame).
Collision check: clearance between the robot centre and the nearest occupied map cell.
Writes results/nav2_summary.json and results/nav2_summary.csv.
"""
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

RESULTS = os.path.expanduser('~/capstone_fault_ws/results')
ROBOT_RADIUS = 0.105     # TurtleBot3 Burger footprint radius [m]


def load_map(share):
    """Occupied cell centres (world coords) of maps/amr_warehouse.{yaml,pgm}."""
    meta = yaml.safe_load(open(os.path.join(share, 'maps', 'amr_warehouse.yaml')))
    with open(os.path.join(share, 'maps', meta['image']), 'rb') as fp:
        assert fp.readline().strip() == b'P5'
        line = fp.readline()
        while line.startswith(b'#'):
            line = fp.readline()
        w, h = map(int, line.split())
        fp.readline()
        img = np.frombuffer(fp.read(), dtype=np.uint8).reshape(h, w)
    rows, cols = np.nonzero(img == 0)
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    return np.stack([ox + (cols + 0.5) * res, oy + (h - rows - 0.5) * res], axis=1)


class Nav2Demo(Node):
    def __init__(self):
        super().__init__('nav2_demo', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.share = get_package_share_directory('fault_bringup')
        self.obstacles = load_map(self.share)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = {}
        self.states = []          # (t, system state)
        self.poses = []           # (t, x, y, vx) from /odom_raw
        self.out = []             # (t, x) final /cmd_vel
        self.nav_raw = []         # (t, x) Nav2 command on /cmd_vel_nav_raw
        self.safety = []
        self.plans = 0
        self.limit = 0.0
        self.results = []
        self.faults = {}
        self.pub_phase = self.create_publisher(String, 'demo/phase', latched)
        self.pub_event = self.create_publisher(String, 'demo/event', 10)
        self.create_subscription(String, 'mission/status', lambda m: setattr(self, 'status', json.loads(m.data)),
                                 latched)
        self.create_subscription(String, 'health/state', self.on_state, latched)
        self.create_subscription(Float64, 'health/speed_limit', lambda m: setattr(self, 'limit', m.data), latched)
        self.create_subscription(Odometry, 'odom_raw', lambda m: self.poses.append(
            (self.now(), m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x)), 10)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self.out.append((self.now(), m.linear.x)), 10)
        self.create_subscription(Twist, 'cmd_vel_nav_raw', lambda m: self.nav_raw.append(
            (self.now(), m.linear.x)), 10)
        self.create_subscription(Twist, 'cmd_vel_safety', lambda m: self.safety.append(self.now()), 10)
        self.create_subscription(Path, 'plan', lambda m: setattr(self, 'plans', self.plans + 1), 10)
        self.cli = {n: self.create_client(Trigger, n) for n in (
            'mission/start', 'lidar_fault/enable', 'lidar_fault/disable', 'odom_fault/enable',
            'odom_fault/disable', 'control_delay/degraded', 'control_delay/critical', 'control_delay/disable')}

    # --- helpers ---
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        if not self.states or self.states[-1][1] != msg.data:
            self.states.append((self.now(), msg.data))
            print(f'  system -> {msg.data} (t={self.now():.1f})', flush=True)

    def state(self):
        return self.states[-1][1] if self.states else None

    def reached(self):
        return self.status.get('reached', 0)

    def wait_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if cond():
                return True
        return False

    def spin_for(self, sec):
        self.wait_until(lambda: False, sec)

    def check(self, name, ok, detail=''):
        self.results.append((name, bool(ok), detail))
        print(f'[{"PASS" if ok else "FAIL"}] {name} {detail}', flush=True)
        return ok

    def phase(self, text):
        print(f'\n######## {text}', flush=True)
        self.pub_phase.publish(String(data=text))

    def svc(self, name):
        cli = self.cli[name]
        cli.wait_for_service(timeout_sec=10.0)
        fut = cli.call_async(Trigger.Request())
        self.wait_until(fut.done, 5.0)
        msg = fut.result().message if fut.result() else 'no response'
        print(f'  > /{name}: {msg}', flush=True)
        self.pub_event.publish(String(data=f'/{name}: {msg}'))
        return self.now()

    def window(self, series, t0, t1, idx=1):
        return [s[idx] for s in series if t0 <= s[0] <= t1]

    def speed(self, t0, t1):
        pts = [(t, x, y) for t, x, y, _ in self.poses if t0 <= t <= t1]
        if len(pts) < 2:
            return 0.0, 0.0
        d = sum(math.dist(a[1:], b[1:]) for a, b in zip(pts, pts[1:]))
        return d / (pts[-1][0] - pts[0][0]), d

    def moving(self, v=0.05):
        return self.poses and abs(self.poses[-1][3]) > v

    def wait_moving(self, timeout=15.0):
        return self.wait_until(lambda: self.moving(0.08), timeout)

    def fault_stop_cycle(self, key, enable, disable, label, nav2_keeps_commanding):
        """Inject a CRITICAL fault while driving, verify stop, clear it, verify resume.

        nav2_keeps_commanding: with a control delay Nav2 keeps sending velocity and only the safety
        channel stops the robot (checked). Without LiDAR, Nav2 itself also commands ~0 because its
        local costmap stops updating, so that is only recorded.
        """
        t0 = self.svc(enable)
        ok = self.wait_until(lambda: self.state() == 'CRITICAL', 5.0)
        t_crit = self.now()
        self.check(f'{label}: system CRITICAL', ok, f'({t_crit - t0:.2f} s after injection)')
        self.spin_for(1.0)
        t_a = self.now()
        self.spin_for(2.0)
        out = self.window(self.out, t_a, self.now())
        v, d = self.speed(t_a, self.now())
        paused = self.status.get('state') == 'PAUSED'
        self.check(f'{label}: safe stop (cmd_vel 0, robot still)', out and max(map(abs, out)) == 0.0 and d < 0.01,
                   f'(moved {d * 1000:.1f} mm in 2 s, mission {self.status.get("state")})')
        nav2_cmd = max(map(abs, self.window(self.nav_raw, t_a, self.now())), default=0.0)
        self.check(f'{label}: mission PAUSED (not cancelled)', paused, f'(Nav2 command max {nav2_cmd:.2f} m/s)')
        if nav2_keeps_commanding:
            self.check(f'{label}: safety overrides a non-zero Nav2 command', nav2_cmd > 0.01,
                       f'(Nav2 {nav2_cmd:.2f} m/s -> final 0)')
        t_off = self.svc(disable)
        ok = self.wait_until(lambda: self.state() == 'NORMAL', 8.0)
        t_norm = self.now()
        self.check(f'{label}: recovered to NORMAL', ok, f'({t_norm - t_off:.2f} s after clearing)')
        ok = self.wait_moving(10.0)
        t_move = self.now()
        ok = ok and self.wait_until(lambda: self.status.get('state') == 'RUNNING', 2.0)
        self.check(f'{label}: mission resumed (robot moving)', ok,
                   f'({t_move - t_norm:.2f} s after NORMAL, mission {self.status.get("state")})')
        stopped = t_move - t_crit
        self.faults[key] = {'to_critical_s': round(t_crit - t0, 2), 'stopped_window_moved_mm': round(d * 1000, 1),
                            'nav2_cmd_max_during_stop_mps': round(nav2_cmd, 3),
                            'recovery_to_normal_s': round(t_norm - t_off, 2),
                            'resume_after_normal_s': round(t_move - t_norm, 2),
                            'time_stopped_s': round(stopped, 2)}

    # --- scenario ---
    def run(self):
        self.phase('Nav2 warehouse mission: startup')
        ok = self.wait_until(lambda: self.status.get('mode') == 'NAV2' and self.state() == 'NORMAL'
                             and len(self.poses) > 10 and self.cli['mission/start'].service_is_ready(), 60.0)
        if not self.check('Nav2 + mission runner ready', ok):
            return False
        total = self.status['total']
        print(f'  mission: {total} waypoints', flush=True)
        self.spin_for(2.0)
        t_start = self.svc('mission/start')
        self.phase('Mission running')

        ok = self.wait_until(lambda: self.reached() >= 1, 90.0)
        self.check('waypoint 1 reached', ok, f'({self.status.get("results", [{}])[0].get("name") if ok else ""})')
        self.wait_moving()
        self.spin_for(2.0)

        self.phase('Fault 1 during Nav2 - LiDAR dropout')
        self.fault_stop_cycle('lidar', 'lidar_fault/enable', 'lidar_fault/disable', 'Nav2+F1',
                              nav2_keeps_commanding=False)

        self.phase('Mission running')
        ok = self.wait_until(lambda: self.reached() >= 2, 120.0)
        self.check('waypoint 2 reached', ok)
        self.wait_moving()
        self.spin_for(1.5)

        self.phase('Fault 2 during Nav2 - odometry anomaly')
        t0 = self.svc('odom_fault/enable')
        ok = self.wait_until(lambda: self.state() == 'DEGRADED', 5.0)
        t_deg = self.now()
        self.check('Nav2+F2: system DEGRADED', ok, f'({t_deg - t0:.2f} s)')
        self.spin_for(1.0)
        t_a = self.now()
        self.spin_for(3.0)
        out = self.window(self.out, t_a, self.now())
        raw = self.window(self.nav_raw, t_a, self.now())
        v_deg, _ = self.speed(t_a, self.now())
        vmax = max((abs(p[3]) for p in self.poses if p[0] >= t_a), default=0.0)
        self.check('Nav2+F2: final cmd_vel limited to <= 0.10', out and max(out) <= 0.1 + 1e-6 and self.limit == 0.1,
                   f'(Nav2 raw max {max(raw, default=0):.2f}, final max {max(out, default=0):.2f})')
        self.check('Nav2+F2: real speed <= 0.10', vmax <= 0.11, f'(max {vmax:.3f}, mean {v_deg:.3f})')
        t_off = self.svc('odom_fault/disable')
        ok = self.wait_until(lambda: self.state() == 'NORMAL', 6.0)
        t_norm = self.now()
        self.check('Nav2+F2: recovered to NORMAL', ok, f'({t_norm - t_off:.2f} s)')
        ok = self.wait_until(lambda: self.moving(0.15), 12.0)
        self.check('Nav2+F2: speed back above 0.15 m/s', ok, f'({self.now() - t_norm:.2f} s after NORMAL)')
        self.faults['odom'] = {'to_degraded_s': round(t_deg - t0, 2),
                               'nav2_raw_max_mps': round(max(raw, default=0), 3),
                               'final_cmd_max_mps': round(max(out, default=0), 3),
                               'degraded_real_max_mps': round(vmax, 3), 'recovery_to_normal_s': round(t_norm - t_off, 2)}

        self.phase('Mission running')
        ok = self.wait_until(lambda: self.reached() >= 3, 120.0)
        self.check('waypoint 3 reached', ok)
        self.wait_moving()
        self.spin_for(1.5)

        self.phase('Fault 3 during Nav2 - control latency')
        t0 = self.svc('control_delay/degraded')
        ok = self.wait_until(lambda: self.state() == 'DEGRADED', 5.0)
        self.check('Nav2+F3: 300 ms -> DEGRADED', ok, f'({self.now() - t0:.2f} s)')
        self.spin_for(2.0)
        out = self.window(self.out, self.now() - 1.5, self.now())
        self.check('Nav2+F3: final cmd_vel limited to <= 0.10', out and max(out) <= 0.1 + 1e-6)
        self.fault_stop_cycle('latency', 'control_delay/critical', 'control_delay/disable', 'Nav2+F3 700 ms',
                              nav2_keeps_commanding=True)

        self.phase('Mission running - to delivery zone')
        ok = self.wait_until(lambda: self.status.get('state') in ('COMPLETE', 'FAILED'), 180.0)
        t_end = self.now()
        done = ok and self.status.get('state') == 'COMPLETE'
        self.check('mission COMPLETE (delivery zone reached)', done, f'({self.status.get("state")})')
        goal = self.status.get('goal', [0, 0])
        _, gx, gy, _ = self.poses[-1]
        err = math.dist((gx, gy), goal[:2])
        self.check('goal position error < 0.25 m (ground truth)', err < 0.25, f'({err:.3f} m)')

        # collisions: robot centre closer than its radius to an occupied map cell
        pts = np.array([(x, y) for t, x, y, _ in self.poses if t_start <= t <= t_end][::3])
        clear = np.array([np.min(np.hypot(*(self.obstacles - p).T)) for p in pts]) - 0.025   # half cell
        touching = clear < ROBOT_RADIUS
        episodes = int(np.sum(touching[1:] & ~touching[:-1]) + (touching[0] if len(touching) else 0))
        self.check('no collision (map clearance > robot radius)', episodes == 0,
                   f'(min clearance {clear.min():.3f} m, robot radius {ROBOT_RADIUS})')
        stopped = sum(b[0] - a[0] for a, b in zip(self.poses, self.poses[1:])
                      if t_start <= a[0] <= t_end and abs(a[3]) < 0.01)
        _, dist = self.speed(t_start, t_end)
        self.summary = {
            'mission_success': done, 'mission_time_s': round(t_end - t_start, 1),
            'waypoints_total': total, 'waypoints_reached': self.reached(),
            'waypoint_results': self.status.get('results', []),
            'retries': self.status.get('total_retries'), 'fault_pauses': self.status.get('pauses'),
            'global_plans_published': self.plans,
            'fault_recoveries': sum(1 for a, b in zip(self.states, self.states[1:])
                                    if b[1] == 'NORMAL' and a[1] != 'NORMAL' and a[0] >= t_start),
            'time_stopped_s': round(stopped, 1), 'distance_travelled_m': round(dist, 2),
            'goal': goal[:2], 'final_position': [round(gx, 3), round(gy, 3)],
            'goal_position_error_m': round(err, 3),
            'collision_count': episodes, 'min_clearance_m': round(float(clear.min()), 3),
            'faults': self.faults}
        self.phase('Nav2 mission complete' if done else 'Nav2 mission FAILED')
        return all(r[1] for r in self.results)

    def write_summary(self, passed):
        os.makedirs(RESULTS, exist_ok=True)
        doc = {'generated': datetime.now().isoformat(timespec='seconds'), 'world': 'amr_warehouse',
               'demo_passed': passed, 'checks_passed': sum(r[1] for r in self.results),
               'checks_total': len(self.results), 'failed_checks': [r[0] for r in self.results if not r[1]],
               'time_base': 'sim time; positions/speeds from Gazebo /odom_raw (ground truth)',
               'results': getattr(self, 'summary', {})}
        with open(os.path.join(RESULTS, 'nav2_summary.json'), 'w') as fp:
            json.dump(doc, fp, indent=2)
        with open(os.path.join(RESULTS, 'nav2_summary.csv'), 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerow(['section', 'metric', 'value'])
            for k, v in doc['results'].items():
                if k == 'faults':
                    for fk, fv in v.items():
                        for mk, mv in fv.items():
                            w.writerow([f'fault_{fk}', mk, mv])
                elif k == 'waypoint_results':
                    for r in v:
                        w.writerow(['waypoint', r['name'], f'{r["result"]} in {r["time_s"]} s (retries {r["retries"]})'])
                else:
                    w.writerow(['mission', k, v])
            w.writerow(['demo', 'checks', f'{doc["checks_passed"]}/{doc["checks_total"]}'])
            w.writerow(['demo', 'passed', passed])
        print(f'summary written to {RESULTS}/nav2_summary.json and .csv', flush=True)


def main():
    rclpy.init()
    node = Nav2Demo()
    passed = False
    try:
        passed = node.run()
    finally:
        node.write_summary(passed)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'\n==== NAV2 WAREHOUSE DEMO: {"PASSED" if passed else "FAILED"} '
          f'({sum(r[1] for r in node.results)}/{len(node.results)} checks) ====')
    sys.exit(0 if passed else 1)
