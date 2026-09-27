"""Fault dashboard: live health state, robot motion and fault-injection buttons (tkinter).

Everything shown comes from existing topics; every button calls an existing service.
SIGUSR1 writes what is currently displayed (texts + colors) to
~/capstone_fault_ws/logs/dashboard_snapshot.json (used for automated checks).
No fault logic lives here. ROS spins in a background thread; the GUI polls a shared
snapshot every 100 ms. The event log only gets a line when something changes.
"""
import json
import os
import signal
import threading
import time
from collections import deque
from datetime import datetime
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String, UInt64
from std_srvs.srv import Trigger

COLORS = {'NORMAL': '#2e9e44', 'WARNING': '#e3c100', 'DEGRADED': '#f07b12', 'CRITICAL': '#d62b2b',
          None: '#7a7a7a'}
TEXT_ON = {'NORMAL': 'white', 'WARNING': 'black', 'DEGRADED': 'black', 'CRITICAL': 'white', None: 'white'}
STALE = 0.5   # s without a message -> value shown as "--"

SUBSYSTEMS = [('lidar', 'LiDAR', 'health/lidar_state'),
              ('odom', 'Odometry', 'health/odom_state'),
              ('control', 'Control Latency', 'health/control_state'),
              ('nav', 'Navigation', 'health/nav_state')]

BUTTONS = [
    ('LiDAR (Fault 1)', 'lidar', [('Enable dropout', 'lidar_fault/enable'),
                                  ('Disable', 'lidar_fault/disable')]),
    ('Odometry (Fault 2)', 'odom', [('Single spike', 'odom_fault/spike'),
                                    ('Continuous', 'odom_fault/enable'),
                                    ('Disable', 'odom_fault/disable')]),
    ('Control delay (Fault 3)', 'control', [('300 ms', 'control_delay/degraded'),
                                            ('700 ms', 'control_delay/critical'),
                                            ('Disable', 'control_delay/disable')]),
    ('Navigation (Fault 4)', 'nav', [('Crash process', 'nav_fault/crash'),
                                     ('Crash loop ON', 'nav_fault/crash_loop_enable'),
                                     ('Crash loop OFF', 'nav_fault/crash_loop_disable')]),
]


class DashboardNode(Node):
    """Collects the latest values; all callbacks only write into self.v / self.events."""

    def __init__(self):
        super().__init__('fault_dashboard')
        self.lock = threading.Lock()
        self.events = deque(maxlen=200)   # (sequence number, text)
        self.event_seq = 0
        self.v = {'state': None, 'lidar': None, 'odom': None, 'control': None, 'nav': None,
                  'scan_age': None, 'odom_vx': None, 'odom_raw_vx': None, 'odom_valid': None,
                  'latency': None, 'restarts': 0, 'pid': None, 'limit': 0.0,
                  'lidar_fault': False, 'odom_fault': False, 'delay_mode': 'NONE', 'crash_loop': False,
                  'phase': '', 'mission': None, 'drive_requested': False}
        self.t = {}   # topic -> monotonic time of last message
        self.last = {'nav_x': None, 'out_x': None}
        self.odom_invalid = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'health/state', lambda m: self.set_state('state', 'SYSTEM', m.data), latched)
        for key, label, topic in SUBSYSTEMS:
            self.create_subscription(String, topic, lambda m, k=key, lb=label: self.set_state(k, lb, m.data), latched)
        self.create_subscription(Float64, 'health/scan_age', lambda m: self.put('scan_age', m.data), 10)
        self.create_subscription(LaserScan, 'scan', lambda m: self.stamp('scan'), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', lambda m: self.put('odom_vx', m.twist.twist.linear.x), 10)
        self.create_subscription(Odometry, 'odom_raw', lambda m: self.put('odom_raw_vx', m.twist.twist.linear.x), 10)
        self.create_subscription(Bool, 'health/odom_valid', self.on_odom_valid, 10)
        self.create_subscription(Float64, 'control/latency_sec', lambda m: self.put('latency', m.data), 10)
        self.create_subscription(UInt64, 'nav/heartbeat', self.on_heartbeat, 10)
        self.create_subscription(UInt64, 'health/nav_restart_count', lambda m: self.put('restarts', m.data), latched)
        self.create_subscription(Float64, 'health/speed_limit', self.on_limit, latched)
        self.create_subscription(Twist, 'cmd_vel_nav_raw', lambda m: self.put('nav_x', m.linear.x), 10)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self.put('out_x', m.linear.x), 10)
        self.create_subscription(Twist, 'cmd_vel_safety', lambda m: self.stamp('safety'), 10)
        self.create_subscription(Bool, 'lidar_fault/active', lambda m: self.flag('lidar_fault', m.data, 'LiDAR dropout fault'), latched)
        self.create_subscription(Bool, 'odom_fault/active', lambda m: self.flag('odom_fault', m.data, 'Odometry spike fault'), latched)
        self.create_subscription(String, 'control/delay_mode', self.on_delay_mode, latched)
        self.create_subscription(Bool, 'nav_fault/crash_loop', lambda m: self.flag('crash_loop', m.data, 'Nav crash loop'), latched)
        self.create_subscription(String, 'demo/phase', self.on_phase, latched)
        self.create_subscription(String, 'demo/event', lambda m: self.event(m.data), 10)
        self.create_subscription(String, 'mission/status', self.on_mission, latched)
        self.create_subscription(String, 'mission/event', lambda m: self.event(f'[mission] {m.data}'), 10)
        self.create_subscription(Bool, 'nav_source/active_request',
                                 lambda m: self.v.__setitem__('drive_requested', m.data), latched)
        self.clients_ = {}
        self.pub_drive = None

    # --- helpers (called from the ROS thread) ---
    def event(self, text):
        with self.lock:
            self.event_seq += 1
            self.events.append((self.event_seq, f'{datetime.now().strftime("%H:%M:%S")}  {text}'))

    def stamp(self, key):
        self.t[key] = time.monotonic()

    def put(self, key, value):
        self.v[key] = value
        self.stamp(key)

    def set_state(self, key, label, value):
        if self.v[key] != value:
            if self.v[key] is not None:
                self.event(f'{label} -> {value}' if key != 'state' else f'SYSTEM -> {value}')
            self.v[key] = value

    def flag(self, key, value, label):
        if self.v[key] != value:
            self.v[key] = value
            self.event(f'{label} {"ENABLED" if value else "cleared"}')

    def on_delay_mode(self, msg):
        if self.v['delay_mode'] != msg.data:
            self.event('Control delay ' + ('cleared' if msg.data == 'NONE' else f'set: {msg.data}'))
            self.v['delay_mode'] = msg.data

    def on_heartbeat(self, msg):
        if self.v['pid'] is not None and self.v['pid'] != msg.data:
            self.event(f'Navigation process restarted: pid {self.v["pid"]} -> {msg.data}')
        self.put('pid', msg.data)

    def on_odom_valid(self, msg):
        self.put('odom_valid', msg.data)
        if not msg.data:
            self.odom_invalid += 1
            self.t['odom_invalid'] = time.monotonic()

    def on_limit(self, msg):
        if msg.data != self.v['limit']:
            self.event(f'Speed limit {msg.data:.2f} m/s' if msg.data > 0 else 'Speed limit released')
        self.v['limit'] = msg.data

    def on_phase(self, msg):
        if msg.data != self.v['phase']:
            self.v['phase'] = msg.data
            if msg.data:
                self.event(f'=== {msg.data} ===')

    def on_mission(self, msg):
        try:
            self.v['mission'] = json.loads(msg.data)
        except ValueError:
            pass

    def age(self, key):
        return time.monotonic() - self.t[key] if key in self.t else None

    def fresh(self, key):
        a = self.age(key)
        return self.v.get(key) if a is not None and a < STALE else None

    def call(self, service):
        if service not in self.clients_:
            self.clients_[service] = self.create_client(Trigger, service)
        cli = self.clients_[service]
        if not cli.service_is_ready():
            self.event(f'[button] /{service} not available')
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: self.event(
            f'[button] /{service}: {f.result().message if f.result() else "no response"}'))

    def drive(self, active):
        if self.pub_drive is None:   # created on first use so it never competes with a test's request
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_drive = self.create_publisher(Bool, 'nav_source/active_request', latched)
        self.pub_drive.publish(Bool(data=active))
        self.event(f'[button] driving {"requested" if active else "stopped"}')


class Dashboard:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Fault-Tolerant TurtleBot3 — Health Dashboard')
        self.root.configure(bg='#1e1e1e')
        big = tkfont.Font(family='DejaVu Sans', size=30, weight='bold')
        mid = tkfont.Font(family='DejaVu Sans', size=15, weight='bold')
        small = tkfont.Font(family='DejaVu Sans', size=11)
        mono = tkfont.Font(family='DejaVu Sans Mono', size=11)
        num = tkfont.Font(family='DejaVu Sans Mono', size=34, weight='bold')
        bg, fg = '#1e1e1e', '#eeeeee'

        self.phase = tk.Label(self.root, text='', font=mid, bg=bg, fg='#8fd3ff')
        self.phase.pack(fill='x', padx=10, pady=(8, 0))
        self.banner = tk.Label(self.root, text='SYSTEM: --', font=big, bg=COLORS[None], fg='white', pady=10)
        self.banner.pack(fill='x', padx=10, pady=8)

        tiles = tk.Frame(self.root, bg=bg)
        tiles.pack(fill='x', padx=10)
        self.tiles = {}
        for i, (key, label, _) in enumerate(SUBSYSTEMS):
            f = tk.Frame(tiles, bg=COLORS[None], padx=8, pady=6)
            f.grid(row=0, column=i, sticky='nsew', padx=4)
            tiles.columnconfigure(i, weight=1)
            name = tk.Label(f, text=label, font=small, bg=COLORS[None], fg='white')
            state = tk.Label(f, text='--', font=mid, bg=COLORS[None], fg='white')
            detail = tk.Label(f, text='', font=mono, bg=COLORS[None], fg='white', justify='left')
            for w in (name, state, detail):
                w.pack(anchor='w')
            self.tiles[key] = (f, name, state, detail)

        motion = tk.Frame(self.root, bg='#2a2a2a', padx=10, pady=6)
        motion.pack(fill='x', padx=10, pady=8)
        self.motion = {}
        for i, (key, label) in enumerate([('nav', 'NAV COMMAND'), ('out', 'FINAL /cmd_vel'),
                                          ('odom', 'ODOM (actual)')]):
            tk.Label(motion, text=label, font=small, bg='#2a2a2a', fg='#bbbbbb').grid(row=0, column=i, padx=20)
            lbl = tk.Label(motion, text='--', font=num, bg='#2a2a2a', fg=fg)
            lbl.grid(row=1, column=i, padx=20)
            motion.columnconfigure(i, weight=1)
            self.motion[key] = lbl
        self.motion_info = tk.Label(motion, text='', font=mid, bg='#2a2a2a', fg=fg)
        self.motion_info.grid(row=2, column=0, columnspan=3, pady=(4, 0))

        mission = tk.Frame(self.root, bg='#2a2a2a', padx=10, pady=4)
        mission.pack(fill='x', padx=10, pady=(0, 8))
        self.nav_mode = tk.Label(mission, text='NAV MODE: --', font=mid, bg='#2a2a2a', fg='#8fd3ff', width=22,
                                 anchor='w')
        self.nav_mode.grid(row=0, column=0, rowspan=2, sticky='w')
        self.mission_line = tk.Label(mission, text='', font=mid, bg='#2a2a2a', fg=fg, anchor='w')
        self.mission_line.grid(row=0, column=1, sticky='w')
        self.mission_detail = tk.Label(mission, text='', font=mono, bg='#2a2a2a', fg='#dddddd', anchor='w')
        self.mission_detail.grid(row=1, column=1, sticky='w')

        controls = tk.Frame(self.root, bg=bg)
        controls.pack(fill='x', padx=10)
        self.fault_labels = {}
        for i, (title, key, buttons) in enumerate(BUTTONS):
            box = tk.LabelFrame(controls, text=title, font=small, bg=bg, fg=fg, padx=6, pady=4)
            box.grid(row=0, column=i, sticky='nsew', padx=4)
            controls.columnconfigure(i, weight=1)
            for text, srv in buttons:
                tk.Button(box, text=text, font=small, width=14,
                          command=lambda s=srv: self.node.call(s)).pack(pady=2)
            lbl = tk.Label(box, text='', font=small, bg=bg, fg=fg)
            lbl.pack()
            self.fault_labels[key] = lbl
        drive = tk.LabelFrame(controls, text='Driving', font=small, bg=bg, fg=fg, padx=6, pady=4)
        drive.grid(row=0, column=len(BUTTONS), sticky='nsew', padx=4)
        tk.Button(drive, text='Start', font=small, width=10, command=lambda: self.node.drive(True)).pack(pady=2)
        tk.Button(drive, text='Stop', font=small, width=10, command=lambda: self.node.drive(False)).pack(pady=2)

        tk.Label(self.root, text='Event log', font=small, bg=bg, fg='#bbbbbb').pack(anchor='w', padx=12, pady=(8, 0))
        self.log = tk.Text(self.root, height=14, font=mono, bg='#111111', fg='#dddddd', state='disabled')
        self.log.pack(fill='both', expand=True, padx=10, pady=(0, 10))
        self.shown_seq = 0
        self.snapshot_requested = False
        self.root.geometry('1100x1040+1420+40')   # right side of the screen
        self.root.attributes('-topmost', True)   # stay visible above the Gazebo window
        self.root.after(100, self.refresh)

    @staticmethod
    def fmt(v, spec='.2f', unit=''):
        return '--' if v is None else f'{v:{spec}}{unit}'

    def color(self, widget_list, state):
        for w in widget_list:
            w.configure(bg=COLORS.get(state, COLORS[None]))
            if isinstance(w, tk.Label):   # frames have no foreground
                w.configure(fg=TEXT_ON.get(state, 'white'))

    def refresh(self):
        try:
            self.update_view()
        except Exception as e:   # keep the refresh loop alive; report once per message
            msg = f'{type(e).__name__}: {e}'
            if msg != getattr(self, 'last_error', None):
                self.last_error = msg
                self.node.get_logger().error(f'dashboard refresh failed: {msg}')
        self.root.after(100, self.refresh)

    def snapshot(self):
        widget = lambda w: {'text': w.cget('text'), 'bg': w.cget('bg'), 'fg': w.cget('fg')}
        data = {
            'time': datetime.now().isoformat(timespec='seconds'),
            'phase': self.phase.cget('text'),
            'banner': widget(self.banner),
            'tiles': {k: {'state': widget(st), 'detail': detail.cget('text')}
                      for k, (f, name, st, detail) in self.tiles.items()},
            'motion': {k: widget(w) for k, w in self.motion.items()},
            'motion_info': self.motion_info.cget('text'),
            'fault_labels': {k: w.cget('text') for k, w in self.fault_labels.items()},
            'nav_mode': self.nav_mode.cget('text'),
            'mission': [self.mission_line.cget('text'), self.mission_detail.cget('text')],
            'event_log': self.log.get('1.0', 'end').strip().splitlines()[-20:],
        }
        path = os.path.expanduser('~/capstone_fault_ws/logs/dashboard_snapshot.json')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as fp:
            json.dump(data, fp, indent=1, ensure_ascii=False)

    def update_view(self):
        n, v = self.node, self.node.v
        state = v['state']
        suffix = {'DEGRADED': f' — SPEED LIMIT {v["limit"]:.2f} m/s', 'CRITICAL': ' — SAFE STOP'}.get(state, '')
        self.banner.configure(text=f'SYSTEM: {state or "--"}{suffix}')
        self.color([self.banner], state)
        self.phase.configure(text=v['phase'])

        valid = n.fresh('odom_valid')
        invalid_recent = n.age('odom_invalid') is not None and n.age('odom_invalid') < 1.0
        hb_age = n.age('pid')
        details = {
            'lidar': f'scan age  {self.fmt(v["scan_age"], ".2f", " s")}\nfault     {"ON" if v["lidar_fault"] else "off"}',
            'odom': f'odom vx   {self.fmt(n.fresh("odom_vx"), ".2f", " m/s")}\n'
                    f'valid     {"NO (rejected)" if invalid_recent else ("yes" if valid else "--")}',
            'control': f'latency   {self.fmt(n.fresh("latency") * 1000 if n.fresh("latency") is not None else None, ".0f", " ms")}\n'
                       f'delay     {v["delay_mode"]}',
            'nav': f'hb age    {self.fmt(hb_age, ".2f", " s")}\n'
                   f'restarts  {v["restarts"]}   pid {v["pid"] or "--"}',
        }
        for key, (f, name, st, detail) in self.tiles.items():
            s = v[key]
            st.configure(text=s or '--')
            detail.configure(text=details[key])
            self.color([f, name, st, detail], s)

        nav_x, out_x, odom_x = n.fresh('nav_x'), n.fresh('out_x'), n.fresh('odom_raw_vx')
        self.motion['nav'].configure(text=self.fmt(nav_x))
        self.motion['out'].configure(text=self.fmt(out_x),
                                     fg='#ff5555' if out_x == 0.0 else ('#ffb347' if out_x is not None and nav_x
                                                                       and out_x < nav_x - 1e-3 else '#eeeeee'))
        self.motion['odom'].configure(text=self.fmt(odom_x))
        safety = n.age('safety') is not None and n.age('safety') < 0.3
        self.motion_info.configure(
            text=f'speed limit: {"%.2f m/s" % v["limit"] if v["limit"] > 0 else "none"}     '
                 f'safety stop: {"ACTIVE" if safety else "off"}     (units m/s)',
            fg='#ff5555' if safety else '#eeeeee')

        m = v['mission']
        if m:
            self.nav_mode.configure(text='NAV MODE: NAV2')
            colors = {'RUNNING': '#7CFC9A', 'PAUSED': '#ff5555', 'RETRY_WAIT': '#ffb347', 'COMPLETE': '#8fd3ff',
                      'FAILED': '#ff5555'}
            dist = m.get('distance_remaining')
            self.mission_line.configure(
                text=f'Mission {m["state"]}   waypoint {m["index"]}/{m["total"]}  {m["name"]}',
                fg=colors.get(m['state'], '#eeeeee'))
            self.mission_detail.configure(
                text=f'goal ({m["goal"][0]:.2f}, {m["goal"][1]:.2f})   distance '
                     f'{"--" if dist is None else "%.2f m" % dist}   active {"yes" if m["active"] else "no"}   '
                     f'reached {m["reached"]}/{m["total"]}   retries {m["total_retries"]}   '
                     f'fault pauses {m["pauses"]}')
        else:
            self.nav_mode.configure(text='NAV MODE: MANUAL/TEST')
            self.mission_line.configure(text='nav_command_source ' + ('driving' if v['drive_requested'] else 'idle'),
                                        fg='#eeeeee')
            self.mission_detail.configure(text='no Nav2 mission (constant-velocity test source)')

        self.fault_labels['lidar'].configure(text='fault: ' + ('ON' if v['lidar_fault'] else 'off'))
        self.fault_labels['odom'].configure(text='fault: ' + ('ON' if v['odom_fault'] else 'off'))
        self.fault_labels['control'].configure(text='delay: ' + v['delay_mode'])
        self.fault_labels['nav'].configure(text='crash loop: ' + ('ON' if v['crash_loop'] else 'off'))

        with n.lock:
            new = [text for seq, text in n.events if seq > self.shown_seq]
            self.shown_seq = n.event_seq
        if new:
            self.log.configure(state='normal')
            for line in new:
                self.log.insert('end', line + '\n')
            self.log.see('end')
            self.log.configure(state='disabled')
        if self.snapshot_requested:
            self.snapshot_requested = False
            self.snapshot()


def main():
    rclpy.init()
    node = DashboardNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    ui = Dashboard(node)
    # Tk swallows exceptions raised in callbacks, so stop the main loop explicitly on SIGINT/SIGTERM
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: ui.root.quit())
    signal.signal(signal.SIGUSR1, lambda *_: setattr(ui, 'snapshot_requested', True))
    try:
        ui.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()          # stop spinning before tearing the node down
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
