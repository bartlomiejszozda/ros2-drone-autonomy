"""Mission runs against a fake Starter, steering service and vehicle."""

import threading
import time
from uuid import uuid4

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ModeSwitch
from collider_interfaces.action import Mission, Takeoff
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool

from collider.src.mission.MissionNode import GUIDED, LAND, LOITER, RTL, MissionNode


class MissionRig:
    def __init__(self, steering_mode=GUIDED, safety='OK', service_timeout=2.0, clock_speed=None):
        self.context = Context()
        rclpy.init(context=self.context)
        prefix = '/mission_test_' + uuid4().hex
        self.events = []
        self.event_times = {}
        self.armed = False
        self.flying = False
        self.mode = 0
        self.takeoff_succeeds = True
        self.takeoff_duration = 0.2
        self.fake = Node('mission_test_fake', context=self.context)
        group = ReentrantCallbackGroup()
        self.running = True
        self.server = ActionServer(
            self.fake, Takeoff, prefix + '/takeoff', self._takeoff,
            cancel_callback=lambda goal: CancelResponse.ACCEPT, callback_group=group)
        self.fake.create_service(SetBool, prefix + '/steering', self._steering,
                                 callback_group=group)
        self.fake.create_service(ModeSwitch, prefix + '/mode', self._mode, callback_group=group)
        self.status_pub = self.fake.create_publisher(Status, prefix + '/status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.safety_pub = self.fake.create_publisher(String, prefix + '/safety', latched)
        if safety is not None:
            self.safety_pub.publish(String(data=safety))
        self.fake.create_timer(0.05, self._publish_status, callback_group=group)
        self.mission = MissionNode(context=self.context, parameter_overrides=[
            Parameter('takeoff_action', value=prefix + '/takeoff'),
            Parameter('steering_enable_service', value=prefix + '/steering'),
            Parameter('mode_service', value=prefix + '/mode'),
            Parameter('status_topic', value=prefix + '/status'),
            Parameter('safety_state_topic', value=prefix + '/safety'),
            Parameter('steering_mode', value=steering_mode),
            Parameter('service_timeout_sec', value=service_timeout),
            Parameter('landing_timeout_sec', value=3.0),
            Parameter('use_sim_time', value=clock_speed is not None)])
        self.clock_speed = clock_speed
        self.clock_pub = self.fake.create_publisher(Clock, '/clock', 10)
        self.client_node = Node('mission_test_client', context=self.context)
        self.client = ActionClient(self.client_node, Mission, '/mission/run')
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        for node in (self.fake, self.mission, self.client_node):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        if clock_speed is not None:
            threading.Thread(target=self._run_clock, daemon=True).start()
        if not self.client.wait_for_server(timeout_sec=10):
            # Clean up before failing: a rig left running crashes the process at exit.
            self.close()
            raise AssertionError('mission action server not discovered')

    def _takeoff(self, goal_handle):
        self.events.append('takeoff')
        self.armed = True
        self.mode = GUIDED
        deadline = time.monotonic() + self.takeoff_duration
        while time.monotonic() < deadline and self.running:
            if goal_handle.is_cancel_requested:
                self.events.append('takeoff cancelled')
                self.armed = False
                goal_handle.canceled()
                return Takeoff.Result(success=False, message='cancelled, landed')
            time.sleep(0.02)
        if not self.takeoff_succeeds:
            self.armed = False
            goal_handle.abort()
            return Takeoff.Result(success=False, message='prearm failed')
        self.flying = True
        # Like the real Starter: succeed only after telemetry has reported flying.
        time.sleep(0.15)
        goal_handle.succeed()
        return Takeoff.Result(success=True, message='at altitude')

    def _run_clock(self):
        """Publish simulated time running clock_speed times faster than the wall clock."""
        start = time.monotonic()
        while self.running:
            sim = 1000.0 + self.clock_speed * (time.monotonic() - start)
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = int(sim), int(sim % 1 * 1e9)
            self.clock_pub.publish(msg)
            time.sleep(0.005)

    def _steering(self, request, response):
        self.events.append(f'steering {request.data}')
        self.event_times[f'steering {request.data}'] = time.monotonic()
        response.success = True
        return response

    def _mode(self, request, response):
        self.events.append({GUIDED: 'GUIDED', LOITER: 'LOITER', LAND: 'LAND',
                            RTL: 'RTL'}.get(request.mode, request.mode))
        self.mode = request.mode
        if request.mode in (LAND, RTL):
            threading.Timer(0.3, self._touch_down).start()
        response.status = True
        response.curr_mode = request.mode
        return response

    def _touch_down(self):
        self.flying = False
        self.armed = False

    def _publish_status(self):
        msg = Status()
        msg.armed, msg.flying, msg.mode = self.armed, self.flying, self.mode
        self.status_pub.publish(msg)

    def start(self, duration=0.5, end_action='land', altitude=5.0):
        future = self.client.send_goal_async(Mission.Goal(
            altitude_m=altitude, duration_sec=duration, end_action=end_action))
        return self.wait(future)

    def wait(self, future, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not future.done():
            assert time.monotonic() < deadline, 'timed out'
            time.sleep(0.02)
        return future.result()

    def run(self, **kwargs):
        handle = self.start(**kwargs)
        assert handle.accepted
        return self.wait(handle.get_result_async())

    def wait_for(self, event, timeout=5.0):
        deadline = time.monotonic() + timeout
        while event not in self.events:
            assert time.monotonic() < deadline, f'{event} never happened: {self.events}'
            time.sleep(0.02)

    def close(self):
        # Stop the fake's loops first, then the executor, and only then destroy what the
        # executor thread might still be using; the other order can crash the process.
        self.running = False
        self.executor.shutdown(timeout_sec=5)
        self.thread.join(timeout=5)
        self.client.destroy()
        self.server.destroy()
        for node in (self.fake, self.mission, self.client_node):
            node.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.fixture
def make_rig():
    rigs = []

    def make(**kwargs):
        rigs.append(MissionRig(**kwargs))
        return rigs[-1]
    yield make
    for rig in rigs:
        rig.close()


def test_full_mission_takes_off_steers_and_lands(make_rig):
    rig = make_rig()
    result = rig.run(duration=0.5)
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.success and result.result.last_stage == 'DONE'
    assert rig.events == ['takeoff', 'steering True', 'steering False', 'LAND']
    assert not rig.armed


def test_hover_end_keeps_flying(make_rig):
    rig = make_rig()
    result = rig.run(duration=0.3, end_action='hover')
    assert result.result.success
    assert rig.events == ['takeoff', 'steering True', 'steering False']
    assert rig.armed and rig.flying


def test_rtl_end(make_rig):
    rig = make_rig()
    assert rig.run(duration=0.3, end_action='rtl').result.success
    assert rig.events[-1] == 'RTL'


def test_rc_steering_uses_loiter_and_hands_back_to_guided(make_rig):
    rig = make_rig(steering_mode=LOITER)
    assert rig.run(duration=0.3).result.success
    assert rig.events == ['takeoff', 'LOITER', 'steering True', 'GUIDED', 'steering False',
                          'LAND']


def test_tripped_safety_stops_before_anything_is_sent(make_rig):
    rig = make_rig(safety='TRIPPED (BRAKE): manual BRAKE')
    result = rig.run()
    assert result.status == GoalStatus.STATUS_ABORTED
    assert 'manual BRAKE' in result.result.message and '/safety/reset' in result.result.message
    assert rig.events == []


def test_missing_safety_node_is_reported(make_rig):
    rig = make_rig(safety=None, service_timeout=0.5)
    result = rig.run()
    assert result.status == GoalStatus.STATUS_ABORTED
    assert 'Safety state' in result.result.message
    assert rig.events == []


def test_failed_takeoff_never_enables_steering(make_rig):
    rig = make_rig()
    rig.takeoff_succeeds = False
    result = rig.run()
    assert result.status == GoalStatus.STATUS_ABORTED
    assert 'prearm failed' in result.result.message
    assert 'steering True' not in rig.events


def test_cancel_while_steering_ends_the_flight(make_rig):
    rig = make_rig()
    handle = rig.start(duration=0.0)
    rig.wait_for('steering True')
    rig.wait(handle.cancel_goal_async())
    result = rig.wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert rig.events[-2:] == ['steering False', 'LAND']


def test_cancel_during_takeoff_lets_the_starter_clean_up(make_rig):
    rig = make_rig()
    rig.takeoff_duration = 2.0
    handle = rig.start()
    rig.wait_for('takeoff')
    rig.wait(handle.cancel_goal_async())
    result = rig.wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert 'takeoff cancelled' in rig.events
    assert 'steering True' not in rig.events
    assert 'LAND' not in rig.events


def test_safety_trip_while_steering_aborts_without_touching_modes(make_rig):
    rig = make_rig()
    handle = rig.start(duration=0.0)
    rig.wait_for('steering True')
    rig.safety_pub.publish(String(data='TRIPPED (BRAKE): pose telemetry lost'))
    result = rig.wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert 'pose telemetry lost' in result.result.message
    assert rig.events[-1] == 'steering False'
    assert 'LAND' not in rig.events


def test_vehicle_landing_by_itself_aborts(make_rig):
    rig = make_rig()
    handle = rig.start(duration=0.0)
    rig.wait_for('steering True')
    rig.flying = False
    result = rig.wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert 'no longer armed and flying' in result.result.message


def test_second_mission_is_rejected_while_one_runs(make_rig):
    rig = make_rig()
    first = rig.start(duration=0.0)
    rig.wait_for('steering True')
    assert not rig.start().accepted
    rig.wait(first.cancel_goal_async())
    rig.wait(first.get_result_async())


@pytest.mark.parametrize('goal', [dict(end_action='crash'), dict(altitude=0.0),
                                  dict(duration=-1.0)])
def test_invalid_goals_are_rejected(make_rig, goal):
    assert not make_rig().start(**goal).accepted


def test_duration_is_in_simulated_seconds(make_rig):
    # The fake clock runs 5x faster than real time: 2 simulated seconds of steering
    # take about 0.4 real seconds. On wall time the same goal would steer for 2 s.
    rig = make_rig(clock_speed=5.0)
    result = rig.run(duration=2.0, end_action='hover')
    assert result.result.success
    steering_wall = rig.event_times['steering False'] - rig.event_times['steering True']
    assert 0.3 < steering_wall < 1.0
