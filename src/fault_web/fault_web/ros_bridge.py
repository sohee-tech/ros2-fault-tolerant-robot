"""ROS 2 side of the monitor: subscriptions only, no publisher, service or action client.

Callbacks only store the latest value, in this separate process. High-rate topics use best-effort QoS, so a hung
or dead monitor never makes a publisher of the control path wait for acknowledgements. Status topics that are
latched by their publishers are read with transient local QoS, so a restarted server gets the current state.
Timestamps are wall clock: the monitor must notice silence even if the simulation clock stops.
"""
import json
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String, UInt64

LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
FAST = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE,
                  reliability=ReliabilityPolicy.BEST_EFFORT)
HEALTH_KEYS = ('state', 'lidar_state', 'odom_state', 'control_state', 'nav_state', 'nav2_state')


class StatusNode(Node):
    def __init__(self):
        super().__init__('web_monitor_bridge')
        self.lock = threading.Lock()
        self.v = {'health': {}}
        self.t = {}
        for key in HEALTH_KEYS:
            self.create_subscription(String, f'health/{key}', lambda m, k=key: self.on_health(k, m.data), LATCHED)
        self.sub(Float64, 'health/scan_age', 'scan_age', FAST, lambda m: m.data)
        self.sub(Float64, 'health/speed_limit', 'speed_limit', LATCHED, lambda m: m.data)
        self.sub(UInt64, 'health/nav_restart_count', 'nav_restarts', LATCHED, lambda m: m.data)
        self.sub(Float64, 'control/latency_sec', 'latency', FAST, lambda m: m.data)
        self.sub(Twist, 'cmd_vel_safety', 'safety', FAST, lambda _m: True)
        self.sub(Twist, 'cmd_vel', 'cmd_vx', FAST, lambda m: m.linear.x)
        self.sub(String, 'nav2_recovery/state', 'recovery_state', LATCHED, lambda m: m.data)
        self.sub(String, 'nav2_recovery/failed_node', 'failed_node', LATCHED, lambda m: m.data)
        self.sub(UInt64, 'nav2_recovery/attempt_count', 'recovery_attempts', LATCHED, lambda m: m.data)
        self.create_subscription(String, 'nav2_recovery/node_states', self.on_nodes, FAST)
        self.create_subscription(String, 'safe_resume/status', self.on_gate, FAST)
        self.create_subscription(String, 'mission/status', self.on_mission, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose, LATCHED)

    def sub(self, msg_type, topic, key, qos, read):
        def callback(msg):
            with self.lock:
                self.v[key] = read(msg)
                self.t[key] = time.time()
        self.create_subscription(msg_type, topic, callback, qos)

    def on_health(self, key, value):
        with self.lock:
            self.v['health'][key] = value

    def on_nodes(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.v['nav2_nodes'] = data.get('nodes', {})
            self.v['recovery_attempts'] = data.get('attempts', self.v.get('recovery_attempts'))
            self.t['nav2_nodes'] = time.time()

    def on_gate(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.v['gate_status'] = data
            self.t['gate_status'] = time.time()

    def on_mission(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.v['mission'] = data
            self.t['mission'] = time.time()

    def on_pose(self, msg):
        c, p = msg.pose.covariance, msg.pose.pose.position
        with self.lock:
            self.v['pose'] = {'x': p.x, 'y': p.y, 'cov_xx': c[0], 'cov_yy': c[7], 'frame': msg.header.frame_id,
                              't': time.time()}

    def snapshot(self):
        now = time.time()
        with self.lock:
            s = dict(self.v)
            s['health'] = dict(self.v['health'])
            age = lambda k: now - self.t[k] if k in self.t else None
            s.update(health_age=age('scan_age'), safety_age=age('safety'), nav2_age=age('nav2_nodes'),
                     gate_status_age=age('gate_status'), mission_age=age('mission'))
        return s


def start():
    """Spin in a daemon thread; returns (node, stop function)."""
    rclpy.init()
    node = StatusNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin, daemon=True).start()

    def stop():
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return node, stop
