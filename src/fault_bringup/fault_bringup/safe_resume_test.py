"""Automated Safe Resume Gate test (warehouse, 5-waypoint mission). Run with run_safe_resume_test.sh.

Requires nav2_warehouse.launch.py safe_resume_gate:=false: the test starts the gate process itself.
  T0 startup order     no gate yet -> Nav2 commands never reach /cmd_vel; gate starts -> verify -> mission
  T1 normal recovery   LiDAR dropout -> stop -> restored -> VERIFYING -> APPROVED -> mission resumes
  T2 sensor not back   LiDAR stays off -> BLOCKED, no motion
  T3 localization      pose dropped / invalid / real AMCL high covariance -> denied, restored -> resumes
  T4 obstacle          real Gazebo box ahead, system NORMAL -> denied, removed -> resumes
  T5 Nav2 recovery     recovery FAILED -> denied, safe stop stays; unblocked -> resumes
  T6 fault after OK    Fault 3 / Fault 4 while driving -> permit revoked at once, safe stop, resume
  T7 gate hung         SIGSTOP on the gate -> permit expires, robot stops; SIGCONT -> re-verified, resumes
  T8 odom / control    fault remains while stopped -> denied with that reason, cleared -> resumes
While the gate denies, the test itself publishes a 0.2 m/s command on /cmd_vel_nav_raw (Nav2's output topic)
and requires final /cmd_vel = 0 and no motion of the Gazebo ground truth (/odom_raw).
Writes results/safe_resume_summary.json / .csv.
"""
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
from datetime import datetime

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from fault_bringup.nav2_demo import RESULTS, ROBOT_RADIUS
from fault_bringup.nav2_recovery_test import Nav2RecoveryTest

SNAPSHOT = os.path.expanduser('~/capstone_fault_ws/logs/dashboard_snapshot.json')
BOX = """<sdf version="1.6"><model name="{name}"><static>true</static><link name="l">
<collision name="c"><geometry><box><size>0.1 0.3 0.5</size></box></geometry></collision>
<visual name="v"><geometry><box><size>0.1 0.3 0.5</size></box></geometry></visual>
</link></model></sdf>"""


class SafeResumeTest(Nav2RecoveryTest):
    def __init__(self):
        super().__init__()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.gate_states = []          # (t, state)
        self.reason = ''
        self.approved = []             # (t, value) on change
        self.resume = {}
        self.yaw = 0.0
        self.amcl = None
        self.table = []                # one row per test
        self.gate = None
        self.create_subscription(String, 'safe_resume/state', lambda m: self.gate_states.append(
            (self.now(), m.data)), latched)
        self.create_subscription(String, 'safe_resume/reason', lambda m: setattr(self, 'reason', m.data), latched)
        self.create_subscription(String, 'safe_resume/status', lambda m: setattr(self, 'resume', json.loads(m.data)), 10)
        self.create_subscription(Bool, 'safe_resume/approved', self.on_approved, 10)
        self.create_subscription(Odometry, 'odom_raw', self.on_truth, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: setattr(self, 'amcl', m), QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_nav = self.create_publisher(Twist, 'cmd_vel_nav_raw', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.spawn = self.create_client(SpawnEntity, 'spawn_entity')
        self.delete = self.create_client(DeleteEntity, 'delete_entity')
        for name in ('localization_fault/drop', 'localization_fault/invalid', 'localization_fault/disable',
                     'nav_fault/crash'):
            self.cli[name] = self.create_client(Trigger, name)
        self.fake_on = False
        self.create_timer(0.1, self.fake_tick)

    # --- helpers ---
    def on_approved(self, msg):
        if not self.approved or self.approved[-1][1] != msg.data:
            self.approved.append((self.now(), msg.data))

    def on_truth(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    def fake_tick(self):
        if self.fake_on:
            cmd = Twist()
            cmd.linear.x = 0.2
            self.pub_nav.publish(cmd)

    def gate_state(self):
        return self.gate_states[-1][1] if self.gate_states else None

    def is_approved(self):
        return bool(self.approved) and self.approved[-1][1]

    def mission(self):
        return self.status.get('state')

    def pose_now(self):
        return self.poses[-1][1:3]

    def blocked_window(self, label, dur, require_reason=None):
        """While denied and with a non-zero Nav2-channel command: final /cmd_vel 0, robot still, PAUSED."""
        self.fake_on = True
        self.spin_for(0.6)     # commands issued before the window are irrelevant; let the robot settle
        t_a, n0 = self.now(), len(self.approved)
        self.spin_for(dur)
        self.fake_on = False
        t_b = self.now()
        out = self.window(self.out, t_a, t_b)
        raw = self.window(self.nav_raw, t_a, t_b)
        _, moved = self.speed(t_a, t_b)
        never = not self.is_approved() and len(self.approved) == n0
        self.check(f'{label}: permit stays denied ({self.gate_state()})', never, f'(reason: {self.reason})')
        self.check(f'{label}: Nav2-channel command present', max(map(abs, raw), default=0.0) > 0.1,
                   f'(max {max(map(abs, raw), default=0.0):.2f} m/s, n={len(raw)})')
        self.check(f'{label}: final /cmd_vel = 0', len(out) > 5 and max(map(abs, out)) == 0.0,
                   f'(max {max(map(abs, out), default=-1.0):.3f}, n={len(out)})')
        self.check(f'{label}: robot did not move (ground truth)', moved < 0.01, f'({moved * 1000:.1f} mm in {t_b - t_a:.1f} s)')
        self.check(f'{label}: mission PAUSED', self.mission() == 'PAUSED', f'({self.mission()})')
        if require_reason:
            self.check(f'{label}: reason names {require_reason}', require_reason in self.reason, f'({self.reason})')
        self.spin_for(0.6)     # the last fake command must be gone before anything can be approved
        return moved

    def expect_resume(self, label, t_clear, t_normal_required=True):
        """Conditions were restored at t_clear: VERIFYING -> APPROVED, then the mission really moves."""
        n = len(self.gate_states)
        ok = self.wait_until(self.is_approved, 45.0)
        t_app = self.now()
        hist = self.gate_states[max(0, n - 3):]
        # only a verification that started after the restoration counts
        t_ver = next((t for t, s in reversed(self.gate_states) if s == 'VERIFYING' and t >= t_clear), None)
        self.check(f'{label}: APPROVED after restoring', ok, f'({t_app - t_clear:.2f} s, {[s for _, s in hist]})')
        # the gate's own measurement: stamps taken here include message delivery delays
        self.wait_until(lambda: self.resume.get('approved'), 2.0)
        stable, held = self.resume.get('stable_time_s', 3.0), self.resume.get('last_verify_s')
        self.check(f'{label}: held VERIFYING for the stable time, started after restoring',
                   t_ver is not None and held is not None and held >= stable,
                   f'({held} s, need {stable:.1f} s)')
        if t_normal_required:
            self.check(f'{label}: system NORMAL when approved', self.state() == 'NORMAL', f'({self.state()})')
        ok = self.wait_until(lambda: self.moving(0.05) and self.mission() == 'RUNNING', 30.0)
        t_move = self.now()
        self.check(f'{label}: mission resumed, robot moving', ok,
                   f'({t_move - t_app:.2f} s after APPROVED, mission {self.mission()}, wp {self.status.get("index")})')
        return {'approved_after_clear_s': round(t_app - t_clear, 2), 'moving_after_approved_s': round(t_move - t_app, 2)}

    def row(self, test, **kw):
        self.table.append({'test': test, **kw})

    def driving(self):
        self.wait_until(lambda: self.moving(0.08) and self.is_approved() and self.mission() == 'RUNNING', 40.0)
        self.spin_for(2.0)

    def dashboard(self, label, expect):
        pids = subprocess.run(['pgrep', '-g', os.environ.get('SIM_PGID', '0'), '-f', 'fault_dashboard'],
                              capture_output=True, text=True).stdout.split()
        if not pids:
            self.check(f'{label}: dashboard running', False)
            return
        try:
            os.remove(SNAPSHOT)
        except OSError:
            pass
        os.kill(int(pids[0]), signal.SIGUSR1)
        self.wait_until(lambda: os.path.exists(SNAPSHOT), 3.0)
        self.spin_for(0.3)
        try:
            shown = json.load(open(SNAPSHOT))['safe_resume']
        except (OSError, ValueError, KeyError):
            shown = None
        text = f'{shown["state"]["text"]} | {shown["reason"]}' if shown else ''
        self.check(f'{label}: dashboard shows {expect}', all(e in text for e in expect), f'({text[:110]})')

    def stop_with_lidar(self, label):
        self.svc('lidar_fault/enable')
        ok = self.wait_until(lambda: self.state() == 'CRITICAL' and not self.is_approved(), 6.0)
        self.check(f'{label}: LiDAR dropout -> CRITICAL, permit revoked', ok)
        self.spin_for(1.0)

    def call(self, client, req):
        client.wait_for_service(timeout_sec=10.0)
        fut = client.call_async(req)
        self.wait_until(fut.done, 10.0)
        return fut.result()

    def start_gate(self):
        exe = os.path.join(get_package_prefix('fault_monitor'), 'lib', 'fault_monitor', 'safe_resume_gate')
        cfg = os.path.join(self.share, 'config', 'safe_resume.yaml')
        self.gate = subprocess.Popen([exe, '--ros-args', '--params-file', cfg])

    # --- tests ---
    def t0_startup(self):
        self.phase('T0 startup: no gate running')
        ok = self.wait_until(lambda: self.status.get('mode') == 'NAV2' and self.state() == 'NORMAL'
                             and self.stack_active() and self.amcl is not None
                             and self.cli['mission/start'].service_is_ready(), 90.0)
        if not self.check('T0: Nav2 ACTIVE, system NORMAL, runner ready', ok):
            return False
        self.t_start = self.svc('mission/start')
        self.check('T0: started mission waits PAUSED', self.wait_until(lambda: self.mission() == 'PAUSED', 5.0),
                   f'({self.mission()})')
        self.blocked_window('T0 no gate', 4.0)
        self.phase('T0 startup: gate process started')
        t0 = self.now()
        self.start_gate()
        self.check('T0: gate first state is not APPROVED',
                   self.wait_until(lambda: self.gate_states, 15.0) and self.gate_states[0][1] != 'APPROVED',
                   f'({self.gate_states[0][1] if self.gate_states else None})')
        self.row('T0 startup order', **self.expect_resume('T0', t0))
        return True

    def t1_recovery(self):
        self.phase('TEST 1 LiDAR dropout -> restored -> resume')
        self.driving()
        idx = self.status.get('index')
        self.stop_with_lidar('T1')
        d = self.blocked_window('T1 dropout', 2.0, 'lidar')
        t_off = self.svc('lidar_fault/disable')
        self.check('T1: no approval before system NORMAL',
                   self.wait_until(lambda: self.state() == 'NORMAL', 8.0) and not self.is_approved(),
                   f'({self.gate_state()})')
        res = self.expect_resume('T1', t_off)
        self.check('T1: same waypoint continued', self.status.get('index') == idx, f'({idx}->{self.status.get("index")})')
        self.row('T1 normal recovery', moved_blocked_mm=round(d * 1000, 1), **res)

    def t2_sensor_lost(self):
        self.phase('TEST 2 LiDAR never comes back')
        self.driving()
        self.stop_with_lidar('T2')
        ok = self.wait_until(lambda: self.gate_state() == 'BLOCKED', 12.0)
        self.check('T2: BLOCKED while LiDAR is missing', ok, f'({self.gate_state()})')
        self.dashboard('T2', ['BLOCKED', 'MOTION DENIED', 'lidar'])
        d = self.blocked_window('T2 no LiDAR', 15.0, 'lidar')
        self.check('T2: still BLOCKED', self.gate_state() == 'BLOCKED')
        t_off = self.svc('lidar_fault/disable')
        self.row('T2 sensor not recovered', moved_blocked_mm=round(d * 1000, 1), blocked_s=15.0,
                 **self.expect_resume('T2 restore', t_off))
        self.dashboard('T2 restore', ['APPROVED', 'MOTION ALLOWED'])

    def t3_localization(self):
        self.phase('TEST 3a /amcl_pose dropped (system stays NORMAL)')
        self.driving()
        self.svc('localization_fault/drop')
        ok = self.wait_until(lambda: self.gate_state() == 'BLOCKED', 10.0)
        self.check('T3a: permit revoked by stale pose while system NORMAL', ok and self.state() == 'NORMAL',
                   f'({self.gate_state()}, system {self.state()}, {self.reason})')
        d1 = self.blocked_window('T3a pose dropped', 5.0, 'localization')
        t_off = self.svc('localization_fault/disable')
        res = self.expect_resume('T3a', t_off)
        self.row('T3a pose dropped', moved_blocked_mm=round(d1 * 1000, 1), **res)

        self.phase('TEST 3b /amcl_pose invalid')
        self.driving()
        self.stop_with_lidar('T3b')
        self.svc('localization_fault/invalid')
        t_off = self.svc('lidar_fault/disable')
        self.wait_until(lambda: self.state() == 'NORMAL' and 'localization' in self.reason, 10.0)
        d2 = self.blocked_window('T3b invalid covariance', 5.0, 'localization')
        t_off = self.svc('localization_fault/disable')
        self.row('T3b invalid pose', moved_blocked_mm=round(d2 * 1000, 1), **self.expect_resume('T3b', t_off))

        self.phase('TEST 3c real AMCL with high covariance (/initialpose)')
        self.driving()
        self.stop_with_lidar('T3c')
        t_off = self.svc('lidar_fault/disable')
        self.wait_until(lambda: self.state() == 'NORMAL', 8.0)
        self.publish_initial(1.0, 0.5)
        ok = self.wait_until(lambda: 'covariance' in self.reason, 6.0)
        self.check('T3c: real AMCL covariance rejected', ok, f'({self.reason})')
        self.check('T3c: permit denied, final cmd_vel 0', not self.is_approved()
                   and max(map(abs, self.window(self.out, self.now() - 0.5, self.now())), default=0.0) == 0.0)
        self.publish_initial(0.01, 0.01)
        t_off = self.now()
        self.row('T3c high covariance', **self.expect_resume('T3c', t_off))

        self.phase('TEST 3d covariance explodes while driving (stay-approved limit)')
        self.driving()
        self.publish_initial(4.0, 0.5)
        ok = self.wait_until(lambda: not self.is_approved(), 5.0)
        self.check('T3d: permit revoked while driving', ok and 'covariance' in self.reason, f'({self.reason})')
        self.spin_for(1.5)
        out = self.window(self.out, self.now() - 0.5, self.now())
        self.check('T3d: final /cmd_vel = 0', out and max(map(abs, out)) == 0.0)
        self.publish_initial(0.01, 0.01)
        self.row('T3d covariance while driving', **self.expect_resume('T3d', self.now()))

    def publish_initial(self, cov_xy, cov_yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        x, y = self.pose_now()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov_xy
        cov[35] = cov_yaw
        msg.pose.covariance = cov
        self.pub_init.publish(msg)

    def t4_obstacle(self):
        self.phase('TEST 4 obstacle ahead of the stopped robot')
        self.driving()
        self.stop_with_lidar('T4')
        x, y = self.pose_now()
        d = 0.27     # box centre ahead of the robot centre; front face at 0.22 m
        req = SpawnEntity.Request()
        req.name = 'safe_resume_box'
        req.xml = BOX.format(name=req.name)
        req.initial_pose.position.x = x + d * math.cos(self.yaw)
        req.initial_pose.position.y = y + d * math.sin(self.yaw)
        req.initial_pose.position.z = 0.25
        req.initial_pose.orientation.z = math.sin(self.yaw / 2)
        req.initial_pose.orientation.w = math.cos(self.yaw / 2)
        res = self.call(self.spawn, req)
        self.check('T4: obstacle spawned in Gazebo', res is not None and res.success)
        self.svc('lidar_fault/disable')
        ok = self.wait_until(lambda: self.state() == 'NORMAL' and 'obstacle:' in self.reason
                             and 'lidar' not in self.reason, 12.0)
        self.check('T4: system NORMAL but obstacle denies', ok, f'(system {self.state()}, {self.reason})')
        dist, need = self.resume.get('obstacle_distance_m'), self.resume.get('required_clearance_m')
        self.check('T4: measured distance < required', dist is not None and need and dist < need,
                   f'({dist} m < {need} m)')
        self.dashboard('T4', ['BLOCKED', 'obstacle'])
        moved = self.blocked_window('T4 obstacle', 12.0, 'obstacle')
        self.check('T4: system NORMAL during the whole denial (only the gate holds the robot)',
                   self.state() == 'NORMAL')
        out = self.call(self.delete, DeleteEntity.Request(name='safe_resume_box'))
        self.check('T4: obstacle removed', out is not None and out.success)
        t_off = self.now()
        self.row('T4 obstacle', moved_blocked_mm=round(moved * 1000, 1), distance_m=dist, required_m=need,
                 **self.expect_resume('T4', t_off))

    def t5_nav2_failed(self):
        self.phase('TEST 5 Nav2 recovery failure')
        self.driving()
        self.svc('nav2_fault/recovery_block_enable')
        t0 = self.svc('nav2_fault/controller_crash')
        ok = self.wait_until(lambda: self.rec_states and self.rec_states[-1][1] == 'FAILED', 90.0)
        self.check('T5: recovery FAILED after max attempts', ok, f'({self.now() - t0:.1f} s)')
        self.wait_until(lambda: self.gate_state() == 'BLOCKED', 5.0)
        self.check('T5: BLOCKED, reason nav2', self.gate_state() == 'BLOCKED' and 'nav2' in self.reason,
                   f'({self.reason})')
        self.check('T5: system CRITICAL (existing safe stop stays)', self.state() == 'CRITICAL')
        n = len(self.safety)
        moved = self.blocked_window('T5 recovery failed', 8.0, 'nav2')
        self.check('T5: /cmd_vel_safety still active', len(self.safety) > n + 50, f'({len(self.safety) - n} msgs)')
        t_off = self.svc('nav2_fault/recovery_block_disable')
        self.row('T5 Nav2 recovery failed', moved_blocked_mm=round(moved * 1000, 1), **self.expect_resume('T5', t_off))

    def revoke_cycle(self, label, inject, restore):
        self.driving()
        t0 = self.svc(inject)
        ok = self.wait_until(lambda: self.state() == 'CRITICAL', 8.0)
        t_crit = self.now()
        ok2 = self.wait_until(lambda: not self.is_approved(), 2.0)
        delay = self.approved[-1][0] - t_crit if self.approved else math.nan
        self.check(f'{label}: CRITICAL and permit revoked', ok and ok2,
                   f'({t_crit - t0:.2f} s after injection, revoke at {delay:+.3f} s from CRITICAL)')
        self.check(f'{label}: revoke within 0.2 s of CRITICAL', delay < 0.2)
        n = len(self.safety)
        moved = self.blocked_window(label, 2.0)
        self.check(f'{label}: existing safe stop active', len(self.safety) > n + 20, f'({len(self.safety) - n} msgs)')
        t_off = self.svc(restore) if restore else t_crit
        self.row(label, revoke_after_critical_s=round(delay, 3), moved_blocked_mm=round(moved * 1000, 1),
                 **self.expect_resume(label, t_off))

    def t6_refault(self):
        self.phase('TEST 6 new fault after approval')
        self.revoke_cycle('T6a Fault 3 latency', 'control_delay/critical', 'control_delay/disable')
        self.revoke_cycle('T6b Fault 4 nav crash', 'nav_fault/crash', None)
        self.revoke_cycle('T6c Fault 1 LiDAR', 'lidar_fault/enable', 'lidar_fault/disable')

    def t7_gate_hung(self):
        self.phase('T7 gate process hung')
        self.driving()
        self.gate.send_signal(signal.SIGSTOP)
        self.wait_until(lambda: self.mission() == 'PAUSED', 4.0)
        self.spin_for(1.5)
        self.fake_on = True
        self.spin_for(0.6)
        t_a = self.now()
        self.spin_for(3.0)
        self.fake_on = False
        out = self.window(self.out, t_a, self.now())
        raw = self.window(self.nav_raw, t_a, self.now())
        _, moved = self.speed(t_a, self.now())
        self.check('T7: Nav2-channel command present', max(map(abs, raw), default=0.0) > 0.1,
                   f'(max {max(map(abs, raw), default=0.0):.2f} m/s)')
        self.check('T7: gate hung -> final /cmd_vel 0, robot still, PAUSED',
                   out and max(map(abs, out)) == 0.0 and moved < 0.01 and self.mission() == 'PAUSED',
                   f'(moved {moved * 1000:.1f} mm, system {self.state()}, mission {self.mission()})')
        self.spin_for(0.6)
        t_cont = self.now()
        self.approved.append((t_cont, False))
        self.gate.send_signal(signal.SIGCONT)
        ok = self.wait_until(lambda: any(t >= t_cont and s == 'BLOCKED' for t, s in self.gate_states), 3.0)
        self.check('T7: stall detected, old approval not reused', ok, f'({self.reason})')
        self.row('T7 gate hung', moved_blocked_mm=round(moved * 1000, 1),
                 **self.expect_resume('T7', t_cont, t_normal_required=False))

    def t8_odom_control(self):
        self.phase('T8 odometry fault stays while stopped')
        self.driving()
        self.stop_with_lidar('T8a')
        self.svc('odom_fault/enable')
        self.svc('lidar_fault/disable')
        ok = self.wait_until(lambda: 'odom' in self.reason and 'lidar' not in self.reason, 10.0)
        self.check('T8a: denied by odometry', ok, f'({self.reason})')
        moved = self.blocked_window('T8a odometry', 3.0, 'odom')
        t_off = self.svc('odom_fault/disable')
        self.row('T8a odometry', moved_blocked_mm=round(moved * 1000, 1), **self.expect_resume('T8a', t_off))

        self.phase('T8 control delay stays while stopped')
        self.driving()
        self.stop_with_lidar('T8b')
        self.svc('control_delay/degraded')
        self.svc('lidar_fault/disable')
        ok = self.wait_until(lambda: 'control' in self.reason and 'lidar' not in self.reason, 10.0)
        self.check('T8b: denied by control latency', ok, f'({self.reason})')
        moved = self.blocked_window('T8b latency', 3.0, 'control')
        t_off = self.svc('control_delay/disable')
        self.row('T8b control latency', moved_blocked_mm=round(moved * 1000, 1), **self.expect_resume('T8b', t_off))

    def run(self):
        if not self.t0_startup():
            return False
        self.t1_recovery()
        self.t2_sensor_lost()
        self.t3_localization()
        self.t4_obstacle()
        self.t5_nav2_failed()
        self.t6_refault()
        self.t7_gate_hung()
        self.t8_odom_control()

        self.phase('Mission running - to delivery zone')
        ok = self.wait_until(lambda: self.mission() in ('COMPLETE', 'FAILED'), 300.0)
        t_end = self.now()
        done = ok and self.mission() == 'COMPLETE'
        self.check('mission COMPLETE after all faults', done, f'({self.mission()}, reached {self.reached()}/'
                   f'{self.status.get("total")}, pauses {self.status.get("pauses")})')
        _, gx, gy, _ = self.poses[-1]
        err = math.dist((gx, gy), self.status.get('goal', [0, 0])[:2])
        self.check('goal position error < 0.25 m (ground truth)', err < 0.25, f'({err:.3f} m)')
        if self.amcl:
            p = self.amcl.pose.pose.position
            loc = math.dist((p.x, p.y), (gx, gy))
            self.check('AMCL still matches ground truth at the end', loc < 0.25, f'({loc:.3f} m)')
        pts = np.array([(x, y) for t, x, y, _ in self.poses if self.t_start <= t <= t_end][::3])
        clear = np.array([np.min(np.hypot(*(self.obstacles - q).T)) for q in pts]) - 0.025
        self.check('no collision (map clearance > robot radius)', clear.min() > ROBOT_RADIUS,
                   f'(min clearance {clear.min():.3f} m)')
        self.summary = {'mission_success': done, 'mission_time_s': round(t_end - self.t_start, 1),
                        'waypoints_reached': self.reached(), 'pauses': self.status.get('pauses'),
                        'retries': self.status.get('total_retries'), 'fault_resends': self.status.get('fault_resends'),
                        'goal_position_error_m': round(err, 3), 'min_clearance_m': round(float(clear.min()), 3),
                        'tests': self.table}
        return all(r[1] for r in self.results)

    def write_summary(self, passed):
        os.makedirs(RESULTS, exist_ok=True)
        groups = {}
        for name, ok, _ in self.results:
            m = re.match(r'T(\d)', name)
            key = f'T{m.group(1)}' if m else 'mission'
            groups.setdefault(key, [0, 0])
            groups[key][0] += bool(ok)
            groups[key][1] += 1
        doc = {'generated': datetime.now().isoformat(timespec='seconds'), 'world': 'amr_warehouse',
               'test_passed': passed, 'checks_passed': sum(r[1] for r in self.results),
               'checks_total': len(self.results), 'failed_checks': [r[0] for r in self.results if not r[1]],
               'groups': {k: f'{a}/{b}' for k, (a, b) in groups.items()},
               'time_base': 'sim time; motion from Gazebo /odom_raw',
               'results': getattr(self, 'summary', {'tests': self.table})}
        with open(os.path.join(RESULTS, 'safe_resume_summary.json'), 'w') as fp:
            json.dump(doc, fp, indent=2)
        keys = ['test', 'moved_blocked_mm', 'blocked_s', 'distance_m', 'required_m', 'revoke_after_critical_s',
                'approved_after_clear_s', 'moving_after_approved_s']
        with open(os.path.join(RESULTS, 'safe_resume_summary.csv'), 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerow(keys)
            for row in self.table:
                w.writerow([row.get(k, '') for k in keys])
        print('per test:', doc['groups'], flush=True)

    def shutdown_gate(self):
        if self.gate is not None and self.gate.poll() is None:
            self.gate.send_signal(signal.SIGCONT)
            self.gate.send_signal(signal.SIGINT)
            try:
                self.gate.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.gate.kill()


def main():
    rclpy.init()
    node = SafeResumeTest()
    passed = False
    try:
        passed = node.run()
    finally:
        node.shutdown_gate()
        node.write_summary(passed)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'\n==== SAFE RESUME TEST: {"PASSED" if passed else "FAILED"} '
          f'({sum(r[1] for r in node.results)}/{len(node.results)} checks) ====')
    sys.exit(0 if passed else 1)
