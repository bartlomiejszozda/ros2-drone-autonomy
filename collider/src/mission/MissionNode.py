"""
Run a whole flight from one action goal: take off, steer, end the flight.

    CHECK ──► TAKEOFF ──► STEERING ──► FINISH ──► LANDING / RTL / hover ──► DONE

It only coordinates: the Starter takes off, the steering node steers, ArduPilot lands.
It never fights the safety supervisor: once safety is tripped, the mission stops
commanding flight modes and aborts.

Unlike the Starter's timer-driven state machine, the sequence is one async function
that reads top to bottom. Each `await` suspends it without blocking a thread.
"""

import math
import signal
from threading import Lock

import rclpy
from action_msgs.msg import GoalStatus
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ModeSwitch
from collider_interfaces.action import Mission, Takeoff
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import SetBool

from collider.src.node_time import age, now_sec

GUIDED, LOITER, RTL, LAND = 4, 5, 6, 9
END_MODES = {'land': LAND, 'rtl': RTL, 'hover': None}
MODE_NAMES = {GUIDED: 'GUIDED', LOITER: 'LOITER', RTL: 'RTL', LAND: 'LAND'}


class MissionStopped(Exception):
    """Ends the sequence early; the message becomes the result message."""


class MissionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('mission', **kwargs)

        def parameter(name, default):
            return self.declare_parameter(name, default).value

        self._steering_mode = int(parameter('steering_mode', GUIDED))
        self._require_safety = bool(parameter('require_safety', True))
        self._service_timeout = float(parameter('service_timeout_sec', 5.0))
        self._takeoff_timeout = float(parameter('takeoff_timeout_sec', 60.0))
        self._landing_timeout = float(parameter('landing_timeout_sec', 120.0))
        self._telemetry_timeout = float(parameter('telemetry_timeout_sec', 2.0))

        # Subscriptions, clients and the wake-up timer share one group; the action server
        # gets its own, so their callbacks keep running while the mission is suspended.
        io = MutuallyExclusiveCallbackGroup()
        self._takeoff_client = ActionClient(
            self, Takeoff, parameter('takeoff_action', '/starter/takeoff'), callback_group=io)
        self._steering_client = self.create_client(
            SetBool, parameter('steering_enable_service', '/steering/enable'),
            callback_group=io)
        self._mode_client = self.create_client(
            ModeSwitch, parameter('mode_service', '/ap/v1/mode_switch'), callback_group=io)
        self.create_subscription(
            Status, parameter('status_topic', '/ap/v1/status'), self._on_status,
            qos_profile_sensor_data, callback_group=io)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, parameter('safety_state_topic', '/safety/state'), self._on_safety,
            latched, callback_group=io)
        self._waiters = []
        self.create_timer(0.05, self._wake_waiters, callback_group=io)

        self._status = None
        self._status_time = -math.inf
        self._safety_state = None
        self._goal = None
        self._steering_enabled = False
        self._stage = 'IDLE'
        self._stage_start = now_sec(self)
        self._busy = False
        self._lock = Lock()
        self._server = ActionServer(
            self, Mission, '/mission/run', execute_callback=self._execute,
            goal_callback=self._accept, cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info('Waiting for missions on /mission/run')

    # --- inputs -----------------------------------------------------------------

    def _on_status(self, msg):
        self._status = msg
        self._status_time = now_sec(self)

    def _on_safety(self, msg):
        self._safety_state = msg.data

    def _safety_tripped(self):
        return self._safety_state is not None and self._safety_state != 'OK'

    def _fresh_status(self):
        return (self._status is not None
                and age(now_sec(self), self._status_time) <= self._telemetry_timeout)

    # --- small async tools ------------------------------------------------------

    def _delay(self, seconds) -> Future:
        """
        Return a Future that completes after `seconds`; `await` it to pause the mission.

        The wake-up timer runs on the node clock, so on simulation time the delay is in
        simulated seconds and stretches while the simulation is paused.
        """
        future = Future()
        self._waiters.append((now_sec(self) + seconds, future))
        return future

    def _wake_waiters(self):
        now = now_sec(self)
        due = [w for w in self._waiters if w[0] <= now]
        self._waiters = [w for w in self._waiters if w[0] > now]
        for _, future in due:
            future.set_result(None)

    async def _within(self, future, timeout, what):
        """Wait for a Future, but not longer than `timeout`."""
        deadline = now_sec(self) + timeout
        while not future.done():
            if now_sec(self) > deadline:
                raise MissionStopped(f'{what}: no answer within {timeout:g} s')
            await self._delay(0.05)
        return future.result()

    async def _wait_ready(self, is_ready, what):
        deadline = now_sec(self) + self._service_timeout
        while not is_ready():
            if now_sec(self) > deadline:
                raise MissionStopped(f'{what} is not available; is its node running?')
            await self._delay(0.1)

    # --- goal handling ----------------------------------------------------------

    def _accept(self, request):
        valid = (math.isfinite(request.altitude_m) and request.altitude_m > 0
                 and math.isfinite(request.duration_sec) and request.duration_sec >= 0
                 and (request.end_action or 'land') in END_MODES)
        if not valid:
            self.get_logger().warning('Rejecting mission: invalid altitude, duration or '
                                      f'end_action (use one of {list(END_MODES)})')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warning('Rejecting mission: another one is running')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    async def _execute(self, goal_handle):
        self._goal = goal_handle
        self._steering_enabled = False
        try:
            return await self._run(goal_handle)
        finally:
            self._goal = None
            self._set_stage('IDLE')
            with self._lock:
                self._busy = False

    async def _run(self, goal_handle):
        request = goal_handle.request
        end_action = request.end_action or 'land'
        try:
            await self._check()
            await self._takeoff(request.altitude_m)
            await self._steer(request.duration_sec)
            await self._finish(end_action)
        except MissionStopped as stop:
            stage = self._stage
            await self._release_steering()
            message = f'{stage}: {stop}'
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self.get_logger().error(f'Mission ended: {message}')
            return Mission.Result(success=False, message=message, last_stage=stage)
        ending = 'hovering' if END_MODES[end_action] is None else f'{end_action.upper()} done'
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            message = f'Cancelled while steering; flight ended, {ending}'
        else:
            goal_handle.succeed()
            message = f'Mission complete, {ending}'
        self.get_logger().info(message)
        return Mission.Result(success=not goal_handle.is_cancel_requested,
                              message=message, last_stage='DONE')

    # --- stages -----------------------------------------------------------------

    async def _check(self):
        self._set_stage('CHECK', 'Checking nodes and safety')
        if self._require_safety:
            await self._wait_ready(lambda: self._safety_state is not None,
                                   'Safety state (/safety/state)')
        if self._safety_tripped():
            raise MissionStopped(f'safety is {self._safety_state}; call /safety/reset first')
        await self._wait_ready(self._takeoff_client.server_is_ready, 'Takeoff action')
        await self._wait_ready(self._steering_client.service_is_ready, 'Steering service')
        await self._wait_ready(self._mode_client.service_is_ready, 'Mode service')

    async def _takeoff(self, altitude):
        self._set_stage('TAKEOFF', f'Taking off to {altitude:g} m')
        handle = await self._within(
            self._takeoff_client.send_goal_async(
                Takeoff.Goal(altitude_m=altitude, timeout_sec=self._takeoff_timeout),
                feedback_callback=self._on_takeoff_feedback),
            self._service_timeout, 'Takeoff action')
        if not handle.accepted:
            raise MissionStopped('the Starter rejected the takeoff (busy or invalid?)')
        result_future = handle.get_result_async()
        cancel_sent = False
        while not result_future.done():
            if self._goal.is_cancel_requested and not cancel_sent:
                # The Starter knows how to undo a half-finished takeoff; let it.
                handle.cancel_goal_async()
                cancel_sent = True
                self._feedback('Cancelling takeoff; the Starter cleans up')
            await self._delay(0.1)
        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise MissionStopped(f'takeoff did not succeed: {result.result.message}')

    def _on_takeoff_feedback(self, message):
        fb = message.feedback
        self._feedback(f'{fb.stage}: {fb.message} ({fb.altitude_m:.1f} m)')

    async def _steer(self, duration):
        if self._steering_mode != GUIDED:
            await self._set_mode(self._steering_mode)
        await self._set_steering(True)
        limit = f'for {duration:g} s' if duration > 0 else 'until cancelled'
        self._set_stage('STEERING', f'Steering {limit}')
        last_feedback = now_sec(self)
        while not self._goal.is_cancel_requested:
            if duration > 0 and now_sec(self) - self._stage_start >= duration:
                return
            self._check_vehicle()
            if now_sec(self) - last_feedback >= 1.0:
                last_feedback = now_sec(self)
                self._feedback(f'Steering {limit}')
            await self._delay(0.1)

    async def _finish(self, end_action):
        self._set_stage('FINISH', 'Stopping steering')
        if self._steering_mode != GUIDED:
            # Hand over to GUIDED first: it hovers by itself, so releasing the RC sticks
            # afterwards cannot make LOITER descend on a low simulated throttle.
            await self._set_mode(GUIDED)
        await self._set_steering(False)
        mode = END_MODES[end_action]
        if mode is None:
            self._set_stage('DONE', 'Hovering in GUIDED')
            return
        self._set_stage(MODE_NAMES[mode], f'Requesting {MODE_NAMES[mode]}')
        await self._set_mode(mode)
        deadline = now_sec(self) + self._landing_timeout
        last_feedback = now_sec(self)
        while not (self._fresh_status() and not self._status.armed):
            if self._safety_tripped():
                raise MissionStopped(f'safety tripped: {self._safety_state}')
            if now_sec(self) > deadline:
                raise MissionStopped(f'not landed and disarmed within {self._landing_timeout:g} s')
            if now_sec(self) - last_feedback >= 1.0:
                last_feedback = now_sec(self)
                self._feedback(f'{MODE_NAMES[mode]}: waiting for landing and disarm')
            await self._delay(0.2)
        self._set_stage('DONE', 'Landed and disarmed')

    def _check_vehicle(self):
        if self._safety_tripped():
            raise MissionStopped(f'safety tripped: {self._safety_state}')
        if not self._fresh_status():
            raise MissionStopped('vehicle status telemetry lost')
        if not (self._status.armed and self._status.flying):
            raise MissionStopped('vehicle is no longer armed and flying')

    # --- commands ---------------------------------------------------------------

    async def _set_mode(self, mode):
        name = MODE_NAMES.get(mode, mode)
        for attempt in range(3):
            if self._safety_tripped():
                # The supervisor owns the flight mode now; never fight it.
                raise MissionStopped(f'safety tripped: {self._safety_state}')
            reply = await self._within(self._mode_client.call_async(ModeSwitch.Request(mode=mode)),
                                       self._service_timeout, f'Mode switch to {name}')
            if reply.status and reply.curr_mode == mode:
                return
            self._feedback(f'{name} refused (attempt {attempt + 1}/3)')
            await self._delay(1.0)
        raise MissionStopped(f'ArduPilot refused {name}')

    async def _set_steering(self, enabled):
        reply = await self._within(self._steering_client.call_async(SetBool.Request(data=enabled)),
                                   self._service_timeout, 'Steering enable')
        if not reply.success:
            raise MissionStopped(f'steering refused: {reply.message}')
        self._steering_enabled = enabled

    async def _release_steering(self):
        """After a stop: switch steering off if we switched it on. Modes are left alone."""
        if not self._steering_enabled or not self._steering_client.service_is_ready():
            return
        try:
            await self._set_steering(False)
        except MissionStopped as error:
            self.get_logger().warning(f'Could not disable steering: {error}')

    # --- feedback ---------------------------------------------------------------

    def _set_stage(self, stage, message=None):
        self._stage = stage
        self._stage_start = now_sec(self)
        if message is not None:
            self._feedback(message)

    def _feedback(self, message):
        if self._goal is None:
            return
        self._goal.publish_feedback(Mission.Feedback(
            stage=self._stage, elapsed_sec=now_sec(self) - self._stage_start,
            message=message))
        self.get_logger().info(f'{self._stage}: {message}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = MissionNode()
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
