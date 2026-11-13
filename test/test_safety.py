"""Safety rules, and how the supervisor reacts when fake ArduPilot services misbehave."""

import math
import time
from uuid import uuid4

import pytest
import rclpy
from action_msgs.srv import CancelGoal
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ModeSwitch
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from collider.src.safety.SafetyNode import SafetyNode
from collider.src.safety.SafetyRules import (
    BRAKE, LAND, RTL, SafetyLimits, Telemetry, check)

GUIDED = 4
LIMITS = SafetyLimits(telemetry_timeout_sec=0.5)


def flying(**changes):
    values = dict(armed=True, flying=True, mode=GUIDED, status_age=0.0, pose_age=0.0)
    values.update(changes)
    return Telemetry(**values)


def test_inside_limits_is_fine():
    assert check(flying(), LIMITS, was_armed=True) is None


def test_on_the_ground_disarmed_is_fine_even_without_pose():
    telemetry = Telemetry(armed=False, flying=False, status_age=0.0)
    assert check(telemetry, LIMITS, was_armed=False) is None


def test_no_telemetry_ever_is_not_a_violation():
    assert check(Telemetry(), LIMITS, was_armed=False) is None


def test_lost_status_matters_only_after_arming():
    assert check(flying(status_age=math.inf), LIMITS, was_armed=True).reason == \
        'status telemetry lost'
    assert check(flying(status_age=math.inf), LIMITS, was_armed=False) is None


def test_lost_pose_while_flying():
    assert check(flying(pose_age=1.0), LIMITS, True).reason == 'pose telemetry lost'


def test_ardupilot_failsafe_is_left_to_ardupilot():
    violation = check(flying(failsafes=(22,)), LIMITS, True)
    assert violation.ardupilot_handles_it


class SafetyRig:
    def __init__(self, violation_action='brake', sim_time=False):
        self.context = Context()
        rclpy.init(context=self.context)
        prefix = '/safety_test_' + uuid4().hex
        self.vehicle = Node('safety_test_vehicle', context=self.context)
        self.mode = GUIDED
        self.armed = True
        self.flying = True
        self.pose_publishing = True
        self.sim_time = sim_time
        self.sim_now = 100.0
        self.clock_running = True
        self.failsafes = []
        self.publishing = True
        self.rejected_modes = set()
        self.mode_requests = []
        self.steering_calls = []
        self.cancel_calls = 0
        self.vehicle.create_service(ModeSwitch, prefix + '/mode', self._mode_switch)
        self.vehicle.create_service(SetBool, prefix + '/steering', self._steering)
        self.vehicle.create_service(
            CancelGoal, prefix + '/takeoff/_action/cancel_goal', self._cancel)
        self.status_pub = self.vehicle.create_publisher(Status, prefix + '/status', 10)
        self.pose_pub = self.vehicle.create_publisher(PoseStamped, prefix + '/pose', 10)
        self.clock_pub = self.vehicle.create_publisher(Clock, '/clock', 10)
        self.vehicle.create_timer(0.02, self._publish)
        self.states = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.vehicle.create_subscription(
            String, '/safety/state', lambda m: self.states.append(m.data), latched)
        self.safety = SafetyNode(context=self.context, parameter_overrides=[
            Parameter('status_topic', value=prefix + '/status'),
            Parameter('pose_topic', value=prefix + '/pose'),
            Parameter('mode_service', value=prefix + '/mode'),
            Parameter('steering_enable_service', value=prefix + '/steering'),
            Parameter('takeoff_action', value=prefix + '/takeoff'),
            Parameter('telemetry_timeout_sec', value=0.5),
            Parameter('violation_action', value=violation_action),
            Parameter('brake_hold_sec', value=0.3),
            Parameter('retry_interval_sec', value=0.1),
            Parameter('use_sim_time', value=sim_time)])
        self.client = Node('safety_test_client', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        for node in (self.vehicle, self.safety, self.client):
            self.executor.add_node(node)
        self.spin(0.3)

    def _mode_switch(self, request, response):
        self.mode_requests.append(request.mode)
        if request.mode not in self.rejected_modes:
            self.mode = request.mode
        response.status = request.mode not in self.rejected_modes
        response.curr_mode = self.mode
        return response

    def _steering(self, request, response):
        self.steering_calls.append(request.data)
        response.success = True
        return response

    def _cancel(self, request, response):
        self.cancel_calls += 1
        return response

    def _publish(self):
        if self.sim_time:
            # Like Gazebo: /clock keeps being published; while paused it does not advance.
            if self.clock_running:
                self.sim_now += 0.02
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = int(self.sim_now), int(self.sim_now % 1 * 1e9)
            self.clock_pub.publish(clock)
        if not self.publishing:
            return
        status = Status()
        status.armed, status.flying, status.mode = self.armed, self.flying, self.mode
        status.failsafe = self.failsafes
        self.status_pub.publish(status)
        if self.pose_publishing:
            self.pose_pub.publish(PoseStamped())

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def call(self, name):
        client = self.client.create_client(Trigger, name)
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 2
        while not future.done():
            assert time.monotonic() < deadline
            self.executor.spin_once(timeout_sec=0.01)
        return future.result()

    def close(self):
        self.executor.shutdown()
        for node in (self.vehicle, self.safety, self.client):
            node.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.fixture
def make_rig():
    rigs = []

    def make(**kwargs):
        rigs.append(SafetyRig(**kwargs))
        return rigs[-1]
    yield make
    for rig in rigs:
        rig.close()


def test_normal_flight_is_left_alone(make_rig):
    rig = make_rig()
    rig.spin(0.5)
    assert rig.mode_requests == [] and rig.steering_calls == [] and rig.cancel_calls == 0
    assert rig.states[-1] == 'OK'


def test_lost_pose_brakes_and_stops_collider(make_rig):
    rig = make_rig()
    rig.pose_publishing = False
    rig.spin(0.9)
    assert rig.mode == BRAKE
    assert rig.mode_requests == [BRAKE]
    assert rig.steering_calls == [False]
    assert rig.cancel_calls == 1
    assert rig.states[-1] == 'TRIPPED (BRAKE): pose telemetry lost'


def test_brake_then_land_when_configured(make_rig):
    rig = make_rig(violation_action='land')
    rig.pose_publishing = False
    rig.spin(1.1)
    assert rig.mode_requests[:2] == [BRAKE, LAND]
    assert rig.mode == LAND


def test_rejected_brake_falls_back_to_land(make_rig):
    rig = make_rig()
    rig.rejected_modes = {BRAKE}
    rig.pose_publishing = False
    rig.spin(0.9)
    assert rig.mode_requests[:2] == [BRAKE, LAND]
    assert rig.mode == LAND


def test_mode_is_held_while_tripped(make_rig):
    rig = make_rig()
    rig.pose_publishing = False
    rig.spin(0.7)
    rig.mode = GUIDED  # someone switches back
    rig.spin(0.4)
    assert rig.mode == BRAKE
    assert rig.mode_requests.count(BRAKE) == 2


def test_ardupilot_failsafe_keeps_its_mode_but_stops_collider(make_rig):
    rig = make_rig()
    rig.failsafes = [22]
    rig.mode = RTL  # what ArduPilot's battery failsafe might have chosen
    rig.spin(0.5)
    assert rig.mode_requests == []
    assert rig.steering_calls == [False]
    assert 'ArduPilot failsafe' in rig.states[-1]


def test_telemetry_loss_while_armed_trips(make_rig):
    rig = make_rig()
    rig.publishing = False
    rig.spin(0.9)
    assert BRAKE in rig.mode_requests
    assert 'telemetry lost' in rig.states[-1]


def test_manual_land_brakes_first(make_rig):
    rig = make_rig()
    assert rig.call('/safety/land').success
    rig.spin(0.8)
    assert rig.mode_requests[:2] == [BRAKE, LAND]
    assert rig.steering_calls == [False]


def test_reset_only_when_safe(make_rig):
    rig = make_rig()
    rig.pose_publishing = False
    rig.spin(0.7)
    refused = rig.call('/safety/reset')
    assert not refused.success and 'pose' in refused.message
    rig.pose_publishing = True
    rig.spin(0.1)
    assert rig.call('/safety/reset').success
    rig.spin(0.2)
    assert rig.states[-1] == 'OK'
    rig.mode = GUIDED
    count = len(rig.mode_requests)
    rig.spin(0.4)
    assert len(rig.mode_requests) == count  # no longer enforcing


def test_disarmed_on_ground_is_never_commanded(make_rig):
    rig = make_rig()
    rig.armed = rig.flying = False
    rig.pose_publishing = False  # no pose on the ground does not matter
    rig.spin(0.9)
    assert rig.mode_requests == [] and rig.steering_calls == []


def test_paused_simulation_does_not_trip(make_rig):
    rig = make_rig(sim_time=True)
    rig.spin(0.4)                        # flying, telemetry arriving, clock running
    rig.clock_running = False            # Gazebo paused: clock frozen ...
    rig.publishing = False               # ... and no telemetry
    rig.spin(1.2)                        # longer than the 0.5 s timeout in real time
    assert rig.mode_requests == []
    assert rig.states[-1] == 'OK'
    rig.clock_running = True             # resumed, but telemetry stays missing
    rig.spin(1.0)                        # about 1 simulated second passes
    assert BRAKE in rig.mode_requests
    assert 'telemetry lost' in rig.states[-1]
