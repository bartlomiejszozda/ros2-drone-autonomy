"""Run one telemetry-confirmed takeoff action for Copter SITL."""

from enum import Enum, auto
import math
import signal
from threading import Lock

import rclpy
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff as TakeoffService
from collider_interfaces.action import Takeoff
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from rclpy.task import Future
from std_srvs.srv import Trigger

from collider.src.ardupilot_related.ModeChanger import ModeChanger
from collider.src.node_time import age, now_sec


class StartupStage(Enum):
    IDLE = auto()
    WAIT_TELEMETRY = auto()
    MODE_CHANGE = auto()
    PREARM = auto()
    ARMING = auto()
    TAKEOFF = auto()
    CLIMBING = auto()
    WAIT_COMMAND = auto()
    RECONCILE = auto()
    DISARMING = auto()
    LANDING = auto()
    WAIT_LANDING = auto()


class StarterNode(Node):
    """Own the startup sequence and its cancellation cleanup."""

    def __init__(self, **kwargs):
        super().__init__('starter', **kwargs)
        self._timeout = self._positive_parameter('timeout_sec', 10.0)
        self._retry_interval = self._positive_parameter('retry_interval_sec', 1.0)
        self._telemetry_timeout = self._positive_parameter('telemetry_timeout_sec', 2.0)
        self._cleanup_timeout = self._positive_parameter('cleanup_timeout_sec', 60.0)
        self._ground_height = self._positive_parameter('ground_height_m', 0.3)
        self._altitude_tolerance = self._positive_parameter('altitude_tolerance_m', 0.2)
        self._settle_time = self._positive_parameter('altitude_hold_sec', 0.5)
        self._max_altitude = self._positive_parameter('max_altitude_m', 50.0)
        self._state_group = MutuallyExclusiveCallbackGroup()
        self._client_group = MutuallyExclusiveCallbackGroup()
        mode_service = self.declare_parameter('mode_service', '/ap/v1/mode_switch').value
        self._mode_changer = ModeChanger(self, mode_service, self._client_group)
        self._prearm_client = self._client(Trigger, 'prearm_service', '/ap/v1/prearm_check')
        self._arm_client = self._client(ArmMotors, 'arm_service', '/ap/v1/arm_motors')
        self._takeoff_client = self._client(
            TakeoffService, 'takeoff_service', '/ap/v1/experimental/takeoff')
        status_topic = self.declare_parameter('status_topic', '/ap/v1/status').value
        pose_topic = self.declare_parameter('pose_topic', '/ap/v1/pose/filtered').value
        self.create_subscription(Status, status_topic, self._on_status,
                                 qos_profile_sensor_data, callback_group=self._state_group)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose,
                                 qos_profile_sensor_data, callback_group=self._state_group)
        # The safety supervisor's state is latched (transient local): we receive the last
        # value even if it was published before this node started.
        safety_topic = self.declare_parameter('safety_state_topic', '/safety/state').value
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, safety_topic, self._on_safety_state, latched,
                                 callback_group=self._state_group)
        self._safety_state = None  # None: no supervisor running, nothing to respect
        self._status = None
        self._status_time = -math.inf
        # Counts status messages. Ordering by count is exact; simulation timestamps are
        # too coarse to tell which of two events in the same clock tick came first.
        self._status_count = 0
        self._altitude = math.nan
        self._pose_time = -math.inf
        self._stage = StartupStage.IDLE
        self._goal = None
        self._completion = None
        self._busy = False
        self._goal_lock = Lock()
        self._cleanup_failed = False
        self._future = None
        self._pending_client = None
        self._pending_motor_command = False
        self._reply_observed = False
        self._status_count_at_reply = -1
        self._arm_sent = False
        self._takeoff_sent = False
        self._cleanup_started = False
        self._timer = self.create_timer(0.1, self._tick, callback_group=self._state_group)
        self._timer.cancel()
        self._action_server = ActionServer(
            self, Takeoff, '/starter/takeoff', execute_callback=self._execute,
            goal_callback=self._accept_goal, handle_accepted_callback=self._start_goal,
            cancel_callback=self._accept_cancel, callback_group=self._state_group)
        self.get_logger().info('Waiting for goals on /starter/takeoff')

    def _positive_parameter(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and greater than zero')
        return value

    def _client(self, service_type, parameter, default):
        name = self.declare_parameter(parameter, default).value
        return self.create_client(service_type, name, callback_group=self._client_group)

    def _on_status(self, message):
        self._status = message
        self._status_time = now_sec(self)
        self._status_count += 1

    def _on_pose(self, message):
        if math.isfinite(message.pose.position.z):
            self._altitude = message.pose.position.z
            self._pose_time = now_sec(self)

    def _fresh_telemetry(self):
        now = now_sec(self)
        return (self._status is not None
                and age(now, self._status_time) <= self._telemetry_timeout
                and age(now, self._pose_time) <= self._telemetry_timeout)

    def _on_safety_state(self, message):
        self._safety_state = message.data

    def _safety_tripped(self):
        return self._safety_state is not None and self._safety_state != 'OK'

    def _grounded(self):
        return (self._fresh_telemetry() and not self._status.flying
                and abs(self._altitude) <= self._ground_height)

    def _accept_goal(self, request):
        if (not math.isfinite(request.altitude_m)
                or request.altitude_m <= self._ground_height + self._altitude_tolerance
                or request.altitude_m > self._max_altitude
                or not math.isfinite(request.timeout_sec) or request.timeout_sec < 0):
            self.get_logger().warning('Rejecting invalid altitude or timeout')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._busy or self._cleanup_failed:
                self.get_logger().warning('Rejecting goal: busy or unresolved cleanup failure')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _start_goal(self, goal_handle):
        self._goal = goal_handle
        self._completion = Future()
        self._arm_sent = False
        self._takeoff_sent = False
        self._cleanup_started = False
        self._status_count_at_reply = -1
        self._at_target_since = None
        now = now_sec(self)
        timeout = goal_handle.request.timeout_sec
        self._goal_deadline = now + timeout if timeout > 0 else None
        goal_handle.execute()
        if self._safety_tripped():
            # Accept, then abort at once: a rejected goal cannot carry a reason, a result can.
            # Arming now would only end in BRAKE, where ArduPilot refuses the takeoff.
            self._complete(False, f'Takeoff refused, nothing sent to the vehicle: safety is '
                                  f'{self._safety_state}. Call /safety/reset first.')
            return
        self._transition(StartupStage.WAIT_TELEMETRY, 'Waiting for fresh ground telemetry')
        self._timer.reset()

    async def _execute(self, goal_handle):
        try:
            return await self._completion
        finally:
            with self._goal_lock:
                self._busy = False

    def _accept_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _feedback(self, message):
        if self._goal is not None:
            self._goal.publish_feedback(Takeoff.Feedback(
                stage=self._stage.name, altitude_m=self._altitude, message=message))
            self.get_logger().info(f'{self._stage.name}: {message}')

    def _forget_request(self):
        if self._future is not None and not self._future.done():
            self._pending_client.remove_pending_request(self._future)
            self._future.cancel()
        self._future = None
        self._pending_client = None

    def _transition(self, stage, message):
        self._forget_request()
        self._stage = stage
        self._next_attempt = now_sec(self)
        self._deadline = self._next_attempt + self._timeout
        self._feedback(message)

    def _rpc(self, client, request, motor=False):
        """Start or poll one request. Never block an executor worker."""
        now = now_sec(self)
        if self._future is None:
            if now < self._next_attempt:
                return None
            if client.service_is_ready():
                self._future = client.call_async(request)
                self._pending_client = client
                self._pending_motor_command = motor
                self._reply_observed = False
                self._deadline = now + self._timeout
                if self._stage == StartupStage.ARMING:
                    self._arm_sent = True
                elif self._stage == StartupStage.TAKEOFF:
                    self._takeoff_sent = True
            elif now >= self._deadline:
                raise TimeoutError('Service unavailable')
            return None
        if self._future.done():
            if not self._reply_observed:
                self._reply_observed = True
                if self._pending_motor_command:
                    self._status_count_at_reply = self._status_count
            response = self._future.result()
            if response is None:
                raise RuntimeError('Service returned no response')
            return response
        if now >= self._deadline:
            # Motor requests stay tracked for cleanup: remote work may finish late.
            raise TimeoutError('Service response timed out')
        return None

    def _retry(self, reason):
        self._forget_request()
        self._next_attempt = now_sec(self) + self._retry_interval
        self._deadline = self._next_attempt + self._timeout
        self._feedback(f'{reason}; retrying in {self._retry_interval:g} seconds')

    def _complete(self, success, message, cleanup_completed=False):
        self._timer.cancel()
        self._forget_request()
        if self._goal.is_cancel_requested:
            self._goal.canceled()
        elif success:
            self._goal.succeed()
        else:
            self._goal.abort()
        result = Takeoff.Result(success=success, cleanup_completed=cleanup_completed,
                                altitude_m=self._altitude, message=message)
        self._goal = None
        self._stage = StartupStage.IDLE
        self._completion.set_result(result)
        self.get_logger().info(message)

    def _begin_cleanup(self, reason):
        if self._cleanup_started:
            return
        if not self._arm_sent:
            ground_and_disarmed = self._grounded() and not self._status.armed
            self._complete(False, reason + '; no motor commands sent', ground_and_disarmed)
            return
        self._cleanup_started = True
        self._cleanup_reason = reason
        self._cleanup_deadline = now_sec(self) + self._cleanup_timeout
        if self._future is not None and not self._future.done():
            # Do not cancel a Future and then race a late arm/takeoff command.
            self._stage = StartupStage.WAIT_COMMAND
            self._feedback('Waiting for the in-flight command before cleanup')
        else:
            self._observe_motor_reply()
            self._transition(StartupStage.RECONCILE, 'Checking vehicle state for cleanup')

    def _observe_motor_reply(self):
        if (self._future is not None and self._future.done()
                and self._pending_motor_command and not self._reply_observed):
            self._reply_observed = True
            self._status_count_at_reply = self._status_count

    def _tick(self):
        if self._goal is None:
            return
        if self._cleanup_started:
            self._cleanup_tick()
            return
        if self._goal.is_cancel_requested:
            self._begin_cleanup('Takeoff canceled')
            return
        if self._goal_deadline is not None and now_sec(self) >= self._goal_deadline:
            self._begin_cleanup('Takeoff exceeded the goal timeout')
            return
        try:
            if self._stage != StartupStage.WAIT_TELEMETRY and not self._fresh_telemetry():
                raise RuntimeError('Vehicle telemetry is stale')
            if self._status is not None and self._status.failsafe:
                raise RuntimeError('ArduPilot reports a failsafe')
            self._startup_tick()
        except Exception as error:
            self._begin_cleanup(f'Takeoff failed: {error}')

    def _startup_tick(self):
        if self._stage == StartupStage.WAIT_TELEMETRY:
            if not self._fresh_telemetry():
                return
            if self._status.vehicle_type != Status.APM_ARDUCOPTER:
                raise RuntimeError('This starter supports Copter SITL')
            if not self._grounded() or self._status.armed:
                raise RuntimeError('Takeoff must start disarmed and on the ground near home')
            self._transition(StartupStage.MODE_CHANGE, 'Requesting GUIDED mode')
        elif self._stage == StartupStage.MODE_CHANGE:
            try:
                response = self._rpc(self._mode_changer.client,
                                     self._mode_request('guided'))
                if response is not None:
                    if response.status and response.curr_mode == ModeChanger.mode_map['guided']:
                        self._transition(StartupStage.PREARM, 'GUIDED confirmed; checking prearm')
                    else:
                        self._retry('GUIDED mode not confirmed')
            except Exception as error:
                self._retry(str(error))
        elif self._stage == StartupStage.PREARM:
            try:
                response = self._rpc(self._prearm_client, Trigger.Request())
                if response is not None:
                    if response.success:
                        self._transition(StartupStage.ARMING,
                                         f'Prearm check passed: {response.message}; arming')
                    else:
                        self._retry(response.message)
            except Exception as error:
                self._retry(str(error))
        elif self._stage == StartupStage.ARMING:
            if self._future is None and (not self._grounded() or self._status.armed
                                         or self._status.mode != ModeChanger.mode_map['guided']):
                raise RuntimeError('Vehicle state changed before arming')
            response = self._rpc(self._arm_client, ArmMotors.Request(arm=True), motor=True)
            if response is not None:
                if not response.result:
                    raise RuntimeError('Arming rejected')
                if self._status_count > self._status_count_at_reply and self._status.armed:
                    self._transition(StartupStage.TAKEOFF, 'Armed confirmed; requesting takeoff')
                elif now_sec(self) >= self._deadline:
                    raise TimeoutError('Armed telemetry was not confirmed')
        elif self._stage == StartupStage.TAKEOFF:
            response = self._rpc(self._takeoff_client, TakeoffService.Request(
                alt=float(self._goal.request.altitude_m)), motor=True)
            if response is not None:
                if not response.status:
                    raise RuntimeError('Takeoff rejected')
                self._transition(StartupStage.CLIMBING, 'Takeoff accepted; monitoring altitude')
        elif self._stage == StartupStage.CLIMBING:
            if not self._status.armed or self._status.mode != ModeChanger.mode_map['guided']:
                raise RuntimeError('Vehicle left the armed GUIDED state during takeoff')
            target = self._goal.request.altitude_m
            if self._status.flying and self._altitude >= target - self._altitude_tolerance:
                if self._at_target_since is None:
                    self._at_target_since = now_sec(self)
                if now_sec(self) - self._at_target_since >= self._settle_time:
                    self._complete(True, f'Target altitude reached: {self._altitude:.2f} m')
            else:
                self._at_target_since = None
            if self._goal is not None:
                self._feedback(f'Climbing to {target:g} m above home')

    def _mode_request(self, mode):
        # Keep mode numbers in ModeChanger rather than duplicating the mapping.
        return ModeSwitch.Request(mode=ModeChanger.mode_map[mode])

    def _cleanup_tick(self):
        if now_sec(self) >= self._cleanup_deadline:
            self._cleanup_failed = True
            self._complete(False, self._cleanup_reason +
                           '; cleanup timed out; vehicle state unresolved', False)
            return
        try:
            if self._stage == StartupStage.WAIT_COMMAND:
                if self._future.done():
                    self._observe_motor_reply()
                    self._transition(StartupStage.RECONCILE, 'Command settled; checking telemetry')
            elif self._stage == StartupStage.RECONCILE:
                if not self._fresh_telemetry():
                    self._transition(StartupStage.LANDING,
                                     'Telemetry unavailable; requesting LAND')
                    return
                if self._status_count <= self._status_count_at_reply:
                    return
                # Once an arm command was sent, explicitly undo it on the ground.
                # An old disarmed observation must not skip the disarm command.
                if self._takeoff_sent or not self._grounded():
                    self._transition(StartupStage.LANDING, 'Requesting LAND for cleanup')
                else:
                    self._transition(StartupStage.DISARMING, 'On ground; requesting disarm')
            elif self._stage == StartupStage.LANDING:
                response = self._rpc(self._mode_changer.client, self._mode_request('land'))
                if response is not None:
                    if response.status and response.curr_mode == ModeChanger.mode_map['land']:
                        self._transition(StartupStage.WAIT_LANDING,
                                         'LAND confirmed; waiting for ground')
                    else:
                        self._retry('LAND not confirmed')
            elif self._stage == StartupStage.WAIT_LANDING:
                if self._grounded():
                    if not self._status.armed:
                        self._complete(False,
                                       self._cleanup_reason + '; landing and disarm confirmed',
                                       True)
                    else:
                        self._transition(StartupStage.DISARMING,
                                         'Ground confirmed; requesting disarm')
            elif self._stage == StartupStage.DISARMING:
                if self._future is None and not self._grounded():
                    self._transition(StartupStage.LANDING, 'Ground state lost; requesting LAND')
                    return
                response = self._rpc(self._arm_client, ArmMotors.Request(arm=False), motor=True)
                if response is not None:
                    if not response.result:
                        self._retry('Disarm rejected')
                    elif (self._grounded() and not self._status.armed
                          and self._status_count > self._status_count_at_reply):
                        self._complete(False,
                                       self._cleanup_reason + '; ground and disarmed confirmed',
                                       True)
                    elif now_sec(self) >= self._deadline:
                        self._transition(StartupStage.RECONCILE, 'Checking disarm outcome')
        except Exception as error:
            if (self._future is not None and not self._future.done()
                    and self._pending_motor_command):
                self._stage = StartupStage.WAIT_COMMAND
                self._feedback(f'{error}; waiting for motor command to settle')
            else:
                self._retry(f'Cleanup: {error}')

    def destroy_node(self):
        self._timer.cancel()
        self._forget_request()
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = StarterNode()
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
