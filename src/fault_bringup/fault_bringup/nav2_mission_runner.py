"""Nav2 waypoint mission runner with fault-aware retry.

Sends NavigateToPose goals for the configured waypoints one after another.
- If the system goes CRITICAL (safe stop by the fault system) while a goal is active, the goal is
  left running (Nav2 continues once twist_mux releases the safety stop); status shows PAUSED.
- If Nav2 aborts / cancels a goal, the runner waits until /health/state is NORMAL again (plus
  `retry_delay`) and re-sends the same waypoint, at most `max_retries` times per waypoint; then the
  mission FAILS.
- A REJECTED goal means Nav2 is not active yet (e.g. right after launch): it is re-sent every
  2 s for up to `nav2_wait_timeout` s without using a retry.

Topics:  /mission/status  std_msgs/String (JSON, transient local)  mode, state, index, total, name,
                          goal, distance_remaining, retries, reached, results
         /mission/event   std_msgs/String  one line per event (dashboard log)
         /mission/complete std_msgs/Bool (transient local)  true once all waypoints are reached
Service: /mission/start   std_srvs/Trigger  (or parameter autostart:=true)
"""
import json
import math
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}


def load_waypoints(layout_file, names):
    with open(layout_file) as fp:
        layout = yaml.safe_load(fp)
    table = {w['name']: w for w in layout.get('waypoints', [])}
    table[layout['goal']['name']] = layout['goal']
    return [dict(table[n]) for n in names]


class MissionRunner(Node):
    def __init__(self):
        super().__init__('nav2_mission_runner', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        p = lambda name, default: self.declare_parameter(name, default).value
        layout_file = p('layout_file', '')
        names = p('waypoints', ['intersection', 'east_cross_aisle', 'ne_narrow_aisle', 'delivery_zone'])
        self.max_retries = p('max_retries', 2)
        self.retry_delay = p('retry_delay', 1.0)
        self.nav2_wait_timeout = p('nav2_wait_timeout', 60.0)
        autostart = p('autostart', False)
        self.waypoints = load_waypoints(layout_file, names)

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_status = self.create_publisher(String, 'mission/status', latched)
        self.pub_event = self.create_publisher(String, 'mission/event', 10)
        self.pub_complete = self.create_publisher(Bool, 'mission/complete', latched)
        self.create_subscription(String, 'health/state', self.on_health, latched)
        self.create_service(Trigger, 'mission/start', self.on_start)

        self.system_state = 'NORMAL'
        self.state = 'IDLE'            # IDLE RUNNING PAUSED RETRY_WAIT WAIT_NAV2 COMPLETE FAILED
        self.index = 0
        self.retries = 0
        self.total_retries = 0
        self.pauses = 0
        self.goal_handle = None
        self.distance = None
        self.results = []              # per waypoint: name, result, retries, t_start, t_end
        self.t_goal = None
        self.t_mission = None
        self.t_retry = 0.0             # earliest time for the next (re)send
        self.t_rejected = None         # first rejection of the current waypoint
        self.start_requested = autostart
        self.pub_complete.publish(Bool(data=False))
        self.create_timer(0.5, self.tick)
        self.get_logger().info('waypoints: ' + ' -> '.join(
            f'{w["name"]}({w["x"]:.2f},{w["y"]:.2f})' for w in self.waypoints))

    # --- helpers ---
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, text):
        self.get_logger().info(text)
        self.pub_event.publish(String(data=text))

    def publish_status(self):
        w = self.waypoints[min(self.index, len(self.waypoints) - 1)]
        self.pub_status.publish(String(data=json.dumps({
            'mode': 'NAV2', 'state': self.state, 'active': self.goal_handle is not None,
            'index': self.index + 1 if self.state not in ('IDLE', 'COMPLETE') else self.index,
            'total': len(self.waypoints), 'name': w['name'],
            'goal': [w['x'], w['y'], w.get('yaw', 0.0)],
            'distance_remaining': self.distance, 'retries': self.retries,
            'total_retries': self.total_retries, 'pauses': self.pauses,
            'reached': sum(r['result'] == 'SUCCEEDED' for r in self.results),
            'mission_time': round(self.now() - self.t_mission, 2) if self.t_mission else None,
            'results': self.results})))

    # --- callbacks ---
    def on_start(self, _req, res):
        if self.state in ('RUNNING', 'PAUSED', 'RETRY_WAIT', 'WAIT_NAV2'):
            res.success, res.message = False, 'mission already running'
        else:
            self.start_requested = True
            res.success, res.message = True, f'mission start requested ({len(self.waypoints)} waypoints)'
        return res

    def on_health(self, msg):
        prev, self.system_state = self.system_state, msg.data
        if self.state == 'RUNNING' and msg.data == 'CRITICAL':
            self.state = 'PAUSED'
            self.pauses += 1
            self.event(f'mission PAUSED at waypoint {self.index + 1} (system CRITICAL -> safe stop)')
        elif self.state == 'PAUSED' and msg.data == 'NORMAL' and prev != 'NORMAL':
            self.state = 'RUNNING'
            self.event(f'mission RESUMED at waypoint {self.index + 1} (system NORMAL)')
        else:
            return
        self.publish_status()   # state changed: do not wait for the next tick

    def tick(self):
        if self.start_requested and self.state in ('IDLE', 'COMPLETE', 'FAILED'):
            if not self.client.wait_for_server(timeout_sec=0.0):
                self.publish_status()
                return
            self.start_requested = False
            self.index, self.results, self.total_retries, self.pauses = 0, [], 0, 0
            self.t_mission = self.now()
            self.pub_complete.publish(Bool(data=False))
            self.event(f'mission START: {len(self.waypoints)} waypoints')
            self.send_goal()
        elif self.state == 'RETRY_WAIT' and self.system_state == 'NORMAL':
            if self.t_retry == 0.0:
                self.t_retry = self.now() + self.retry_delay
            elif self.now() >= self.t_retry:
                self.t_retry = 0.0
                self.event(f'retry {self.retries}/{self.max_retries}: re-sending waypoint {self.index + 1}')
                self.send_goal()
        elif self.state == 'WAIT_NAV2' and self.now() >= self.t_retry:
            self.send_goal()
        self.publish_status()

    def send_goal(self):
        w = self.waypoints[self.index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(w['x'])
        goal.pose.pose.position.y = float(w['y'])
        yaw = float(w.get('yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.state = 'RUNNING' if self.system_state != 'CRITICAL' else 'PAUSED'
        self.distance = None
        if self.retries == 0 and self.t_rejected is None:
            self.t_goal = self.now()
        self.event(f'waypoint {self.index + 1}/{len(self.waypoints)} -> {w["name"]} ({w["x"]:.2f}, {w["y"]:.2f})')
        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        fut.add_done_callback(self.on_goal_response)

    def on_feedback(self, msg):
        self.distance = round(msg.feedback.distance_remaining, 2)

    def on_goal_response(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.finish_goal('REJECTED')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.finish_goal(STATUS_NAMES.get(f.result().status, str(f.result().status))))

    def finish_goal(self, result):
        self.goal_handle = None
        w = self.waypoints[self.index]
        if result == 'REJECTED':
            now = self.now()
            self.t_rejected = self.t_rejected or now
            if now - self.t_rejected < self.nav2_wait_timeout:
                self.state = 'WAIT_NAV2'
                self.t_retry = now + 2.0
                self.get_logger().info(f'waypoint {self.index + 1} rejected (Nav2 not active yet), '
                                       'retrying in 2 s')
                self.publish_status()
                return
        self.t_rejected = None
        if result == 'SUCCEEDED':
            self.results.append({'name': w['name'], 'result': result, 'retries': self.retries,
                                 'time_s': round(self.now() - self.t_goal, 2)})
            self.event(f'waypoint {self.index + 1} {w["name"]} REACHED')
            self.retries = 0
            self.index += 1
            if self.index >= len(self.waypoints):
                self.state = 'COMPLETE'
                self.index = len(self.waypoints)
                self.pub_complete.publish(Bool(data=True))
                self.event(f'mission COMPLETE in {self.now() - self.t_mission:.1f} s '
                           f'({self.total_retries} retries, {self.pauses} fault pauses)')
            else:
                self.send_goal()
        elif self.retries < self.max_retries:
            self.retries += 1
            self.total_retries += 1
            self.state = 'RETRY_WAIT'
            self.event(f'waypoint {self.index + 1} {w["name"]} {result}; retry when system is NORMAL')
        else:
            self.results.append({'name': w['name'], 'result': result, 'retries': self.retries,
                                 'time_s': round(self.now() - self.t_goal, 2)})
            self.state = 'FAILED'
            self.event(f'mission FAILED at waypoint {self.index + 1} {w["name"]} ({result}, '
                       f'{self.retries} retries used)')
        self.publish_status()


def main():
    rclpy.init()
    node = MissionRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
