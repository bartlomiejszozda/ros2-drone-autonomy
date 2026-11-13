"""
Watch the vehicle and take control away from Collider when something goes wrong.

It does not stop other nodes. It changes ArduPilot's flight mode, so whatever they
send is ignored (cmd_vel needs GUIDED, RC steering needs LOITER). Then it asks them to
stop: steering is disabled and takeoff goals are cancelled.

Once tripped it stays tripped (latched) and keeps the vehicle in the safe mode until
/safety/reset is called.
"""

import math
import signal
import time

import rclpy
from action_msgs.srv import CancelGoal
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ModeSwitch
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from collider.src.node_time import age, now_sec
from collider.src.safety.SafetyRules import (
    BRAKE, FINAL_MODES, LAND, MODE_NAMES, SafetyLimits, Telemetry, check)


class SafetyNode(Node):
    def __init__(self, **kwargs):
        super().__init__('safety', **kwargs)
        self._limits = SafetyLimits(
            telemetry_timeout_sec=float(
                self.declare_parameter('telemetry_timeout_sec', 1.0).value))
        action = self.declare_parameter('violation_action', 'brake').value
        if action not in FINAL_MODES:
            raise ValueError(f'violation_action must be one of {list(FINAL_MODES)}')
        self._violation_mode = FINAL_MODES[action]
        self._brake_hold = float(self.declare_parameter('brake_hold_sec', 2.0).value)
        self._retry_interval = float(self.declare_parameter('retry_interval_sec', 1.0).value)
        status_topic = self.declare_parameter('status_topic', '/ap/v1/status').value
        pose_topic = self.declare_parameter('pose_topic', '/ap/v1/pose/filtered').value
        mode_service = self.declare_parameter('mode_service', '/ap/v1/mode_switch').value
        steering_service = self.declare_parameter(
            'steering_enable_service', '/steering/enable').value
        takeoff_action = self.declare_parameter('takeoff_action', '/starter/takeoff').value

        self._mode_client = self.create_client(ModeSwitch, mode_service)
        self._steering_client = self.create_client(SetBool, steering_service)
        # Every action server offers this hidden service. An empty request (zero goal id,
        # zero time) cancels all of its goals, whoever sent them.
        self._cancel_client = self.create_client(
            CancelGoal, takeoff_action + '/_action/cancel_goal')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(String, '/safety/state', latched)
        self.create_subscription(Status, status_topic, self._on_status, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)
        for name, mode in FINAL_MODES.items():
            self.create_service(Trigger, f'/safety/{name}', self._manual(mode))
        self.create_service(Trigger, '/safety/reset', self._on_reset)
        # On simulation time this timer stops while the simulation is paused, and so do
        # all timeouts: pausing Gazebo cannot trip the supervisor.
        self.create_timer(0.1, self._tick)
        # This one runs on the computer's clock to notice that simulation time stands still.
        self._clock_stall_warning = float(
            self.declare_parameter('clock_stall_warning_sec', 3.0).value)
        self._last_sim_time = None
        self._sim_time_changed = time.monotonic()
        self.create_timer(1.0, self._watch_clock, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self._telemetry = Telemetry()
        self._status_time = -math.inf
        self._pose_time = -math.inf
        self._was_armed = False
        self._tripped = False
        self._reason = ''
        self._final_mode = None      # None: leave the mode to ArduPilot's own failsafe
        self._trip_time = 0.0
        self._brake_rejected = False
        self._mode_future = None
        self._requested_mode = None
        self._last_request = -math.inf
        self._publish_state()
        then = '' if self._violation_mode == BRAKE else \
            f' then {MODE_NAMES[self._violation_mode]}'
        self.get_logger().info(
            f'Watching telemetry and ArduPilot failsafes; on violation BRAKE{then}. '
            f'Manual: /safety/brake, /safety/land, /safety/rtl')

    # --- simulation clock -------------------------------------------------------

    def _watch_clock(self):
        """
        Warn (never trip) when simulation time stops while the vehicle may be flying.

        Pausing Gazebo is deliberate, but if Gazebo hangs, our timeouts stop as well and
        lost telemetry would go unnoticed. ArduPilot's own failsafes still apply.
        """
        if not self.get_parameter('use_sim_time').value:
            return
        sim_time = now_sec(self)
        if sim_time != self._last_sim_time:
            self._last_sim_time = sim_time
            self._sim_time_changed = time.monotonic()
            return
        stalled = time.monotonic() - self._sim_time_changed
        if stalled > self._clock_stall_warning and self._was_armed:
            self.get_logger().warning(
                f'Simulation time has not advanced for {stalled:.0f} s (paused, or Gazebo '
                f'stopped): telemetry loss cannot be detected meanwhile',
                throttle_duration_sec=10.0)

    # --- telemetry --------------------------------------------------------------

    def _on_status(self, msg: Status):
        t = self._telemetry
        t.armed, t.flying, t.mode = msg.armed, msg.flying, msg.mode
        t.failsafes = tuple(msg.failsafe)
        self._status_time = now_sec(self)
        self._was_armed = msg.armed

    def _on_pose(self, msg: PoseStamped):
        # Only its arrival matters: a missing pose means the estimate or the link is gone.
        self._pose_time = now_sec(self)

    def _current_telemetry(self) -> Telemetry:
        now = now_sec(self)
        self._telemetry.status_age = age(now, self._status_time)
        self._telemetry.pose_age = age(now, self._pose_time)
        return self._telemetry

    # --- main loop --------------------------------------------------------------

    def _tick(self):
        if not self._tripped:
            violation = check(self._current_telemetry(), self._limits, self._was_armed)
            if violation is not None:
                final = None if violation.ardupilot_handles_it else self._violation_mode
                self._trip(violation.reason, final)
        if self._tripped:
            self._enforce_mode()

    def _trip(self, reason: str, final_mode):
        self._tripped = True
        self._reason = reason
        self._final_mode = final_mode
        self._trip_time = now_sec(self)
        self._brake_rejected = False
        self._last_request = -math.inf
        self.get_logger().error(f'SAFETY TRIPPED: {reason}')
        self._stop_collider_nodes()
        self._publish_state()

    def _stop_collider_nodes(self):
        """Layer 2: ask our own nodes to stop. Safety does not depend on this."""
        if self._steering_client.service_is_ready():
            self._steering_client.call_async(SetBool.Request(data=False))
            self.get_logger().info('Requested steering disable')
        if self._cancel_client.service_is_ready():
            self._cancel_client.call_async(CancelGoal.Request())
            self.get_logger().info('Requested cancellation of takeoff goals')

    def _desired_mode(self):
        if self._final_mode is None:
            return None
        if self._brake_rejected:
            return LAND
        # BRAKE first: it stops at once and, unlike LAND and RTL, ignores pilot sticks.
        in_hold = age(now_sec(self), self._trip_time) < self._brake_hold
        if in_hold or self._final_mode == BRAKE:
            return BRAKE
        return self._final_mode

    def _enforce_mode(self):
        """Layer 1: keep ArduPilot in the safe mode; repeat if someone switches away."""
        desired = self._desired_mode()
        if desired is None or self._telemetry.mode == desired:
            return
        fresh = self._current_telemetry().status_age <= self._limits.telemetry_timeout_sec
        if fresh and not self._telemetry.armed:
            return  # disarmed: nothing left to protect, and no reason to command it
        if self._mode_future is not None and not self._mode_future.done():
            return
        if age(now_sec(self), self._last_request) < self._retry_interval:
            return
        if not self._mode_client.service_is_ready():
            self.get_logger().warning('Mode service unavailable; retrying',
                                      throttle_duration_sec=2.0)
            return
        self._requested_mode = desired
        self._last_request = now_sec(self)
        self._mode_future = self._mode_client.call_async(ModeSwitch.Request(mode=desired))
        self._mode_future.add_done_callback(self._on_mode_reply)
        self.get_logger().warning(f'Requesting {MODE_NAMES.get(desired, desired)}')

    def _on_mode_reply(self, future):
        reply = future.result()
        if reply is not None and reply.status and reply.curr_mode == self._requested_mode:
            self.get_logger().info(f'Vehicle in {MODE_NAMES.get(reply.curr_mode)}')
            return
        if self._requested_mode == BRAKE and not self._brake_rejected:
            # BRAKE needs a position estimate; LAND works without one.
            self._brake_rejected = True
            self._last_request = -math.inf
            self.get_logger().error('BRAKE rejected; falling back to LAND')
        else:
            self.get_logger().error(
                f'{MODE_NAMES.get(self._requested_mode)} rejected; retrying')

    # --- services ---------------------------------------------------------------

    def _manual(self, mode):
        def handle(request, response):
            if self._tripped:
                # Already tripped: just change where we end up, e.g. BRAKE, then LAND.
                self._final_mode = mode
                self._last_request = -math.inf
            else:
                self._trip(f'manual {MODE_NAMES[mode]}', mode)
            self._publish_state()
            response.success = True
            response.message = f'Safety: {MODE_NAMES[mode]}'
            return response
        return handle

    def _on_reset(self, request, response):
        violation = check(self._current_telemetry(), self._limits, self._was_armed)
        if violation is not None:
            response.success = False
            response.message = f'Cannot reset, still violating: {violation.reason}'
            return response
        self._tripped = False
        self._reason = ''
        self._final_mode = None
        self._publish_state()
        self.get_logger().info('Safety reset; Collider may command the vehicle again')
        response.success = True
        response.message = 'Safety reset. Steering stays disabled until enabled again.'
        return response

    def _publish_state(self):
        if not self._tripped:
            text = 'OK'
        else:
            target = MODE_NAMES.get(self._final_mode, 'ArduPilot failsafe')
            text = f'TRIPPED ({target}): {self._reason}'
        self._state_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = SafetyNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Under ros2 launch, Ctrl+C reaches the node twice (terminal and launch);
        # ignore the second one so it cannot interrupt the clean-up below.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
