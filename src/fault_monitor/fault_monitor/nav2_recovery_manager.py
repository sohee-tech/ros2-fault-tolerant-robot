"""Nav2 lifecycle recovery manager.

Watches the lifecycle state of the Nav2 navigation servers (GetState, 5 Hz) and the PID of each
server process, and restores the stack when something is not ACTIVE.

Humble behaviour this builds on: when a server is deactivated or its process dies, its bond to
lifecycle_manager_navigation breaks and that manager resets (cleans up) the whole navigation stack.
Its own automatic restart is disabled (attempt_respawn_reconnection: false), so recovery is done
here, with an attempt limit:

  IDLE -> DETECTED -> RECOVERING: wait until the stack has settled (every server's lifecycle
          service is back - a crashed server must be respawned - and no server is still ACTIVE),
          at most `settle_timeout` s; if it did not settle, send RESET first.
       -> STARTUP via /lifecycle_manager_navigation/manage_nodes (configure + activate in order)
       -> verify all ACTIVE after `verify_time` -> RECOVERED -> (healthy `healthy_hold` s) -> IDLE
  A failed attempt is retried after `retry_delay`; after `max_recovery_attempts` -> FAILED.
  Attempts are only reset after `healthy_hold` s of a healthy stack. FAILED is left when
  /nav2_fault/recovery_block turns false or /nav2_recovery/retry is called.

Topics: /nav2/stack_ok (Bool, 5 Hz; health_monitor input), /nav2_recovery/node_states (String JSON),
        /nav2_recovery/state, /nav2_recovery/failed_node (String, transient local),
        /nav2_recovery/attempt_count (UInt64, transient local), /nav2_recovery/event (String)
"""
import json
import os
import subprocess

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String, UInt64
from std_srvs.srv import Trigger

EXE = {'controller_server': 'nav2_controller/controller_server', 'planner_server': 'nav2_planner/planner_server',
       'bt_navigator': 'nav2_bt_navigator/bt_navigator', 'behavior_server': 'nav2_behaviors/behavior_server',
       'waypoint_follower': 'nav2_waypoint_follower/waypoint_follower'}
LABELS = {1: 'UNCONFIGURED', 2: 'INACTIVE', 3: 'ACTIVE', 4: 'FINALIZED'}


class Nav2RecoveryManager(Node):
    def __init__(self):
        super().__init__('nav2_recovery_manager', parameter_overrides=[Parameter('use_sim_time', value=True)])
        p = lambda n, d: self.declare_parameter(n, d).value
        self.nodes = p('nodes', ['controller_server', 'planner_server', 'bt_navigator', 'behavior_server'])
        manager = p('lifecycle_manager', 'lifecycle_manager_navigation')
        self.max_attempts = p('max_recovery_attempts', 3)
        self.settle_timeout = p('settle_timeout', 8.0)
        self.service_wait_timeout = p('service_wait_timeout', 20.0)
        self.verify_time = p('verify_time', 1.5)
        self.retry_delay = p('retry_delay', 2.0)
        self.healthy_hold = p('healthy_hold', 2.0)

        self.get = {n: self.create_client(GetState, f'/{n}/get_state') for n in self.nodes}
        self.manage = self.create_client(ManageLifecycleNodes, f'/{manager}/manage_nodes')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_ok = self.create_publisher(Bool, 'nav2/stack_ok', 10)
        self.pub_nodes = self.create_publisher(String, 'nav2_recovery/node_states', 10)
        self.pub_state = self.create_publisher(String, 'nav2_recovery/state', latched)
        self.pub_failed = self.create_publisher(String, 'nav2_recovery/failed_node', latched)
        self.pub_attempts = self.create_publisher(UInt64, 'nav2_recovery/attempt_count', latched)
        self.pub_event = self.create_publisher(String, 'nav2_recovery/event', 10)
        self.create_subscription(Bool, 'nav2_fault/recovery_block', self.on_block, latched)
        self.create_service(Trigger, 'nav2_recovery/retry', self.on_retry)

        self.states = {n: 'UNKNOWN' for n in self.nodes}
        self.pending = {n: None for n in self.nodes}          # (future, sent time)
        self.pids = {n: None for n in self.nodes}
        self.restarts = {n: 0 for n in self.nodes}
        self.pid_history = []                                  # (node, old, new)
        self.state = None
        self.failed_node = ''
        self.attempts = 0
        self.t_state = self.now()
        self.busy = False                                      # a manage_nodes call is in flight
        self.startup_ok = None
        self.seen_active = False
        self.set_state('IDLE', 'waiting for the Nav2 stack to become ACTIVE')
        self.pub_attempts.publish(UInt64(data=0))
        self.pub_failed.publish(String(data=''))
        self.create_timer(0.2, self.poll)
        self.create_timer(1.0, self.poll_pids)
        self.create_timer(0.1, self.step)

    # --- helpers ---
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, text):
        self.get_logger().info(text)
        self.pub_event.publish(String(data=text))

    def set_state(self, state, why=''):
        if state != self.state:
            self.state = state
            self.t_state = self.now()
            self.pub_state.publish(String(data=state))
            self.event(f'[{state}] {why}' if why else f'[{state}]')

    def all_active(self):
        return all(s == 'ACTIVE' for s in self.states.values())

    def abnormal(self):
        return [n for n, s in self.states.items() if s != 'ACTIVE']

    # --- observation ---
    def poll(self):
        t = self.now()
        for n in self.nodes:
            pend = self.pending[n]
            if pend is not None:
                fut, sent = pend
                if fut.done():
                    r = fut.result()
                    self.states[n] = LABELS.get(r.current_state.id, r.current_state.label.upper()) if r else 'ERROR'
                    self.pending[n] = None
                elif t - sent > 1.0:
                    self.states[n] = 'UNRESPONSIVE'
                    self.pending[n] = None
                continue
            if not self.get[n].service_is_ready():
                self.states[n] = 'MISSING'
                continue
            self.pending[n] = (self.get[n].call_async(GetState.Request()), t)
        ok = self.all_active()
        self.seen_active |= ok
        self.pub_ok.publish(Bool(data=ok))
        self.pub_nodes.publish(String(data=json.dumps({
            'state': self.state, 'attempts': self.attempts, 'failed_node': self.failed_node,
            'nodes': {n: {'state': self.states[n], 'pid': self.pids[n], 'restarts': self.restarts[n]}
                      for n in self.nodes},
            'pid_history': self.pid_history[-5:]})))

    def poll_pids(self):
        pg = str(os.getpgid(0))
        for n in self.nodes:
            out = subprocess.run(['pgrep', '-g', pg, '-f', f'^/opt/ros/humble/lib/{EXE[n]}( |$)'],
                                 capture_output=True, text=True).stdout.split()
            pid = int(out[0]) if out else None
            if pid and self.pids[n] and pid != self.pids[n]:
                self.restarts[n] += 1
                self.pid_history.append((n, self.pids[n], pid))
                self.event(f'{n} process restarted: pid {self.pids[n]} -> {pid}')
            if pid:
                self.pids[n] = pid

    # --- recovery state machine ---
    def step(self):
        if not self.seen_active or self.busy:
            return
        t, dt = self.now(), self.now() - self.t_state
        bad = self.abnormal()
        if self.state == 'IDLE':
            if bad:
                self.failed_node = bad[0]
                self.pub_failed.publish(String(data=self.failed_node))
                self.set_state('DETECTED', f'{", ".join(f"{n}={self.states[n]}" for n in bad)}')
        elif self.state == 'DETECTED':
            self.set_state('RECOVERING', f'attempt {self.attempts + 1}/{self.max_attempts}: waiting for the '
                                         'stack to settle')
        elif self.state == 'RECOVERING':
            missing = [n for n in self.nodes if self.states[n] in ('MISSING', 'UNRESPONSIVE', 'UNKNOWN')]
            active = [n for n in self.nodes if self.states[n] == 'ACTIVE']
            if missing and dt < self.service_wait_timeout:
                return                                    # e.g. crashed server not respawned yet
            if active and len(active) < len(self.nodes) and dt < self.settle_timeout:
                return                                    # lifecycle manager still resetting
            self.attempts += 1
            self.pub_attempts.publish(UInt64(data=self.attempts))
            self.startup(reset_first=bool(active))
        elif self.state == 'VERIFY':
            if dt >= self.verify_time:
                if self.all_active() and self.startup_ok:
                    self.set_state('RECOVERED', f'all Nav2 servers ACTIVE after {self.attempts} attempt(s)')
                else:
                    self.attempt_failed(f'startup {"ok" if self.startup_ok else "failed"}, '
                                        f'still abnormal: {", ".join(bad) or "-"}')
        elif self.state == 'WAIT_RETRY':
            if dt >= self.retry_delay:
                self.set_state('RECOVERING', f'attempt {self.attempts + 1}/{self.max_attempts}')
        elif self.state == 'RECOVERED':
            if bad:
                self.failed_node = bad[0]
                self.pub_failed.publish(String(data=self.failed_node))
                if self.attempts >= self.max_attempts:
                    self.set_state('FAILED', f'{self.failed_node} failed again, {self.attempts} attempts used')
                else:
                    self.set_state('DETECTED', f'{self.failed_node} failed again during healthy hold')
            elif dt >= self.healthy_hold:
                self.attempts = 0
                self.pub_attempts.publish(UInt64(data=0))
                self.failed_node = ''
                self.pub_failed.publish(String(data=''))
                self.set_state('IDLE', f'healthy for {self.healthy_hold:.1f} s')
        elif self.state == 'FAILED':
            if self.all_active():
                self.set_state('RECOVERED', 'stack became ACTIVE')

    def attempt_failed(self, why):
        if self.attempts >= self.max_attempts:
            self.set_state('FAILED', f'{why}; max {self.max_attempts} attempts reached -> SAFE STOP stays')
        else:
            self.set_state('WAIT_RETRY', why)

    def startup(self, reset_first):
        if not self.manage.service_is_ready():
            self.attempt_failed('manage_nodes service not available')
            return
        self.busy = True
        self.startup_ok = None
        self.event(f'attempt {self.attempts}/{self.max_attempts}: '
                   f'{"RESET + " if reset_first else ""}STARTUP via manage_nodes')

        def send(cmd, then):
            req = ManageLifecycleNodes.Request()
            req.command = cmd
            self.manage.call_async(req).add_done_callback(then)

        def after_startup(f):
            self.startup_ok = bool(f.result() and f.result().success)
            self.busy = False
            self.set_state('VERIFY', f'STARTUP {"succeeded" if self.startup_ok else "FAILED"}')

        if reset_first:
            send(ManageLifecycleNodes.Request.RESET, lambda f: send(ManageLifecycleNodes.Request.STARTUP, after_startup))
        else:
            send(ManageLifecycleNodes.Request.STARTUP, after_startup)

    # --- external triggers ---
    def on_block(self, msg):
        if not msg.data and self.state == 'FAILED':
            self.attempts = 0
            self.pub_attempts.publish(UInt64(data=0))
            self.set_state('RECOVERING', 'recovery block released: retrying')

    def on_retry(self, _req, res):
        if self.state == 'FAILED':
            self.attempts = 0
            self.set_state('RECOVERING', 'manual retry')
            res.success, res.message = True, 'recovery restarted'
        else:
            res.success, res.message = False, f'not FAILED (state {self.state})'
        return res


def main():
    rclpy.init()
    node = Nav2RecoveryManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
