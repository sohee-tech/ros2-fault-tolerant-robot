"""Nav2 lifecycle fault injector (test tool).

Services (std_srvs/Trigger):
  /nav2_fault/controller_deactivate   real lifecycle DEACTIVATE of /controller_server
  /nav2_fault/planner_deactivate      real lifecycle DEACTIVATE of /planner_server
  /nav2_fault/bt_deactivate           real lifecycle DEACTIVATE of /bt_navigator
  /nav2_fault/controller_crash        SIGKILL the controller_server process (launch respawns it).
                                      Only a process in THIS launch's process group whose command
                                      line is the controller_server executable is targeted.
  /nav2_fault/recovery_block_enable   while enabled, controller_server is deactivated again as soon
  /nav2_fault/recovery_block_disable  as it becomes ACTIVE, so every recovery attempt fails.
Topic:
  /nav2_fault/recovery_block  std_msgs/Bool (transient local)
"""
import os
import signal
import subprocess

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

CONTROLLER_EXE = '/opt/ros/humble/lib/nav2_controller/controller_server'


def controller_pids():
    """controller_server PIDs in this process group (= this simulation launch)."""
    out = subprocess.run(['pgrep', '-g', str(os.getpgid(0)), '-f', f'^{CONTROLLER_EXE}( |$)'],
                         capture_output=True, text=True).stdout
    return [int(p) for p in out.split()]


class Nav2FaultInjector(Node):
    def __init__(self):
        super().__init__('nav2_fault_injector')
        self.change = {n: self.create_client(ChangeState, f'/{n}/change_state')
                       for n in ('controller_server', 'planner_server', 'bt_navigator')}
        self.get_ctrl = self.create_client(GetState, '/controller_server/get_state')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_block = self.create_publisher(Bool, 'nav2_fault/recovery_block', latched)
        for name, node in (('controller', 'controller_server'), ('planner', 'planner_server'),
                           ('bt', 'bt_navigator')):
            self.create_service(Trigger, f'nav2_fault/{name}_deactivate',
                                lambda req, res, n=node: self.on_deactivate(n, res))
        self.create_service(Trigger, 'nav2_fault/controller_crash', self.on_crash)
        self.create_service(Trigger, 'nav2_fault/recovery_block_enable', lambda q, r: self.set_block(True, r))
        self.create_service(Trigger, 'nav2_fault/recovery_block_disable', lambda q, r: self.set_block(False, r))
        self.block = False
        self.block_busy = False
        self.pub_block.publish(Bool(data=False))
        self.create_timer(0.1, self.enforce_block)

    def deactivate(self, node):
        cli = self.change[node]
        if not cli.service_is_ready():
            return False
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        cli.call_async(req)
        return True

    def on_deactivate(self, node, res):
        res.success = self.deactivate(node)
        res.message = (f'lifecycle DEACTIVATE requested on /{node}' if res.success
                       else f'/{node}/change_state not available')
        self.get_logger().warn(res.message)
        return res

    def on_crash(self, _req, res):
        pids = controller_pids()
        if len(pids) != 1:
            res.success, res.message = False, f'expected one controller_server in this launch, found {pids}'
        else:
            os.kill(pids[0], signal.SIGKILL)
            res.success, res.message = True, f'controller_server pid={pids[0]} killed (SIGKILL)'
        self.get_logger().error(res.message)
        return res

    def set_block(self, on, res):
        self.block = on
        self.pub_block.publish(Bool(data=on))
        res.success = True
        res.message = f'Nav2 recovery block {"ENABLED" if on else "DISABLED"}'
        self.get_logger().warn(res.message)
        return res

    def enforce_block(self):
        if not self.block or self.block_busy or not self.get_ctrl.service_is_ready():
            return
        self.block_busy = True
        fut = self.get_ctrl.call_async(GetState.Request())

        def done(f):
            self.block_busy = False
            r = f.result()
            if self.block and r is not None and r.current_state.id == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().warn('recovery block: controller_server became ACTIVE -> deactivating again')
                self.deactivate('controller_server')
        fut.add_done_callback(done)


def main():
    rclpy.init()
    node = Nav2FaultInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
