"""Takeoff and recovery through the Starter action, with fake Copter services."""

import math
import time
from uuid import uuid4

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff as TakeoffService
from collider_interfaces.action import Takeoff
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from collider.src.steering.StarterNode import StarterNode, StartupStage


class ActionRig:
    def __init__(self):
        self.context = Context()
        rclpy.init(context=self.context)
        prefix = '/takeoff_test_' + uuid4().hex
        self.services = Node('takeoff_test_services', context=self.context)
        self.events = []
        self.feedback = []
        self.mode_replies = [(True, 4)]
        self.mode_count = 0
        self.prearm_count = 0
        self.ready_after = 1
        self.arm_delay = 0.0
        self.takeoff_delay = 0.0
        self.mode_delay = 0.0
        self.arm_changes_state = True
        self.takeoff_allowed = True
        self.allow_land = True
        self.publish_enabled = True
        self.climb_enabled = True
        self.armed = False
        self.flying = False
        self.altitude = 0.0
        self.target = None
        self.mode = 0
        self.arm_calls = []
        self.arm_reply_time = None
        self.disarm_time = None
        self.status_pub = self.services.create_publisher(Status, prefix + '/status', 10)
        self.pose_pub = self.services.create_publisher(PoseStamped, prefix + '/pose', 10)
        self.mode_service = self.services.create_service(ModeSwitch, prefix + '/mode', self._mode)
        self.services.create_service(Trigger, prefix + '/prearm', self._prearm)
        self.services.create_service(ArmMotors, prefix + '/arm', self._arm)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.safety_pub = self.services.create_publisher(String, prefix + '/safety', latched)
        self.takeoff_service = self.services.create_service(
            TakeoffService, prefix + '/takeoff', self._takeoff)
        self.services.create_timer(0.05, self._telemetry)
        self.starter = StarterNode(context=self.context, parameter_overrides=[
            Parameter('mode_service', value=prefix + '/mode'),
            Parameter('prearm_service', value=prefix + '/prearm'),
            Parameter('arm_service', value=prefix + '/arm'),
            Parameter('takeoff_service', value=prefix + '/takeoff'),
            Parameter('status_topic', value=prefix + '/status'),
            Parameter('pose_topic', value=prefix + '/pose'),
            Parameter('safety_state_topic', value=prefix + '/safety'),
            Parameter('timeout_sec', value=0.35),
            Parameter('retry_interval_sec', value=0.1),
            Parameter('telemetry_timeout_sec', value=2.0),
            Parameter('cleanup_timeout_sec', value=2.0),
            Parameter('altitude_hold_sec', value=0.1)])
        self.client_node = Node('takeoff_test_client', context=self.context)
        self.client = ActionClient(self.client_node, Takeoff, '/starter/takeoff')
        self.executor = MultiThreadedExecutor(num_threads=3, context=self.context)
        for node in (self.services, self.starter, self.client_node):
            self.executor.add_node(node)
        assert self.client.wait_for_server(timeout_sec=3)

    def _mode(self, request, response):
        if request.mode == 9:
            self.events.append('land')
            response.status = self.allow_land
            response.curr_mode = 9 if self.allow_land else self.mode
            if self.allow_land:
                self.mode = 9
        else:
            assert request.mode == 4
            self.events.append('guided')
            self.mode_count += 1
            if self.mode_count == 1:
                time.sleep(self.mode_delay)
            index = min(self.mode_count - 1, len(self.mode_replies) - 1)
            response.status, response.curr_mode = self.mode_replies[index]
            if response.status:
                self.mode = response.curr_mode
        return response

    def _prearm(self, request, response):
        self.events.append('prearm')
        self.prearm_count += 1
        response.success = self.prearm_count >= self.ready_after
        response.message = 'Vehicle is Armable' if response.success else 'GPS not ready'
        return response

    def _arm(self, request, response):
        self.arm_calls.append((request.arm, self.altitude, self.flying))
        self.events.append('arm' if request.arm else 'disarm')
        if request.arm:
            time.sleep(self.arm_delay)
            if self.arm_changes_state:
                self.armed = True
            self.arm_reply_time = time.monotonic()
        else:
            assert not self.flying and abs(self.altitude) <= 0.3, 'Disarm requested in flight'
            self.armed = False
            self.disarm_time = time.monotonic()
        response.result = True
        return response

    def _takeoff(self, request, response):
        self.events.append('takeoff')
        assert self.armed
        time.sleep(self.takeoff_delay)
        if self.takeoff_allowed:
            self.target = request.alt
            self.flying = True
            self.altitude = 0.4
        response.status = self.takeoff_allowed
        return response

    def _telemetry(self):
        if self.mode == 9:
            self.altitude = max(0.0, self.altitude - 0.4)
            self.flying = self.altitude > 0.0
        elif self.target is not None and self.climb_enabled and self.armed:
            self.altitude = min(self.target, self.altitude + 0.4)
        if not self.publish_enabled:
            return
        status = Status(vehicle_type=Status.APM_ARDUCOPTER, armed=self.armed,
                        mode=self.mode, flying=self.flying)
        self.status_pub.publish(status)
        pose = PoseStamped()
        pose.pose.position.z = self.altitude
        self.pose_pub.publish(pose)

    def until(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, (self.events, self.starter._stage)
            self.executor.spin_once(timeout_sec=0.02)

    def wait(self, future):
        self.until(future.done)
        return future.result()

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def send(self, timeout=0.0, altitude=2.0):
        return self.wait(self.client.send_goal_async(
            Takeoff.Goal(altitude_m=altitude, timeout_sec=timeout),
            feedback_callback=lambda msg: self.feedback.append(msg.feedback)))

    def cancel(self, goal):
        assert self.wait(goal.cancel_goal_async()).goals_canceling
        return self.wait(goal.get_result_async())

    def close(self):
        self.executor.shutdown()
        self.client.destroy()
        for node in (self.client_node, self.starter, self.services):
            node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def rig():
    instance = ActionRig()
    try:
        yield instance
    finally:
        instance.close()


def test_idle_then_full_takeoff_with_stage_feedback(rig):
    rig.spin_for(0.3)
    assert not rig.events
    rig.mode_replies = [(False, 4), (True, 0), (True, 4)]
    rig.ready_after = 2
    goal = rig.send()
    result = rig.wait(goal.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.success
    assert result.result.altitude_m >= 1.8
    assert rig.events == ['guided'] * 3 + ['prearm'] * 2 + ['arm', 'takeoff']
    assert {'MODE_CHANGE', 'PREARM', 'ARMING', 'TAKEOFF', 'CLIMBING'} <= {
        f.stage for f in rig.feedback}
    assert any('GPS not ready' in f.message for f in rig.feedback)
    assert rig.armed and rig.flying
    count = len(rig.events)
    rig.spin_for(0.3)
    assert len(rig.events) == count


def test_takeoff_ack_does_not_finish_before_altitude(rig):
    rig.climb_enabled = False
    goal = rig.send()
    result = goal.get_result_async()
    rig.until(lambda: rig.starter._stage == StartupStage.CLIMBING)
    rig.spin_for(0.4)
    assert not result.done()
    rig.climb_enabled = True
    assert rig.wait(result).status == GoalStatus.STATUS_SUCCEEDED


def test_busy_rejection_and_cancel_during_prearm(rig):
    rig.ready_after = 1000
    goal = rig.send()
    rig.until(lambda: rig.prearm_count > 0)
    assert not rig.send().accepted
    result = rig.cancel(goal)
    assert result.status == GoalStatus.STATUS_CANCELED
    assert result.result.cleanup_completed
    assert 'arm' not in rig.events
    rig.ready_after = 1
    assert rig.wait(rig.send().get_result_async()).status == GoalStatus.STATUS_SUCCEEDED


def test_cancel_pending_arm_waits_then_disarms_on_ground(rig):
    rig.arm_delay = 0.6
    goal = rig.send()
    rig.until(lambda: 'arm' in rig.events)
    result = rig.cancel(goal)
    assert result.status == GoalStatus.STATUS_CANCELED
    assert result.result.cleanup_completed
    assert not rig.armed
    assert rig.disarm_time >= rig.arm_reply_time
    assert 'takeoff' not in rig.events and 'land' not in rig.events
    assert rig.events[-1] == 'disarm'


def test_timed_out_arm_reply_is_tracked_until_cleanup(rig):
    rig.arm_delay = 0.6
    result = rig.wait(rig.send().get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.cleanup_completed
    assert rig.disarm_time >= rig.arm_reply_time
    assert rig.events[-2:] == ['arm', 'disarm']
    assert 'takeoff' not in rig.events


def test_cancel_pending_takeoff_lands_before_disarming(rig):
    rig.takeoff_delay = 0.5
    goal = rig.send()
    rig.until(lambda: 'takeoff' in rig.events)
    result = rig.cancel(goal)
    assert result.status == GoalStatus.STATUS_CANCELED
    assert result.result.cleanup_completed
    assert rig.events.index('land') < rig.events.index('disarm')
    assert not rig.armed and not rig.flying


def test_cancel_in_flight_lands_then_disarms(rig):
    rig.climb_enabled = False
    goal = rig.send()
    rig.until(lambda: rig.starter._stage == StartupStage.CLIMBING)
    result = rig.cancel(goal)
    assert result.result.cleanup_completed
    assert result.status == GoalStatus.STATUS_CANCELED
    assert rig.events[-2:] == ['land', 'disarm']
    assert all(not flying and height <= 0.3 for arm, height, flying in rig.arm_calls if not arm)


def test_missing_takeoff_service_cleans_up_armed_ground_state(rig):
    rig.services.destroy_service(rig.takeoff_service)
    rig.spin_for(0.2)
    result = rig.wait(rig.send().get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.cleanup_completed
    assert rig.events[-1] == 'disarm'
    assert 'takeoff' not in rig.events and 'land' not in rig.events


def test_rejected_takeoff_returns_failure_after_cleanup(rig):
    rig.takeoff_allowed = False
    result = rig.wait(rig.send().get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.cleanup_completed
    assert rig.events[-2:] == ['land', 'disarm']


def test_arm_ack_requires_post_reply_armed_telemetry(rig):
    rig.arm_changes_state = False
    result = rig.wait(rig.send().get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.cleanup_completed
    assert 'takeoff' not in rig.events
    assert rig.events[-1] == 'disarm'


def test_overall_timeout_in_flight_cleans_up(rig):
    rig.climb_enabled = False
    result = rig.wait(rig.send(timeout=1.3).get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.cleanup_completed
    assert rig.events[-2:] == ['land', 'disarm']


def test_telemetry_loss_in_flight_requests_land(rig):
    rig.climb_enabled = False
    goal = rig.send()
    rig.until(lambda: rig.starter._stage == StartupStage.CLIMBING)
    rig.starter._telemetry_timeout = 0.15
    rig.publish_enabled = False
    rig.until(lambda: 'land' in rig.events)
    rig.publish_enabled = True
    result = rig.wait(goal.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.cleanup_completed
    assert not rig.armed


def test_failed_cleanup_is_reported_and_blocks_new_goals(rig):
    rig.climb_enabled = False
    rig.allow_land = False
    rig.starter._cleanup_timeout = 0.6
    goal = rig.send()
    rig.until(lambda: rig.starter._stage == StartupStage.CLIMBING)
    result = rig.cancel(goal)
    assert result.status == GoalStatus.STATUS_CANCELED
    assert not result.result.cleanup_completed
    assert rig.armed
    assert 'disarm' not in rig.events
    assert not rig.send().accepted


def test_no_telemetry_means_no_commands(rig):
    rig.publish_enabled = False
    result = rig.wait(rig.send(timeout=0.4).get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.cleanup_completed
    assert not rig.events


def test_initially_armed_vehicle_is_not_taken_over(rig):
    rig.armed = True
    result = rig.wait(rig.send().get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.cleanup_completed
    assert rig.armed
    assert not rig.events


def test_cleanup_timeout_does_not_race_a_pending_arm_command(rig):
    rig.arm_delay = 1.0
    rig.starter._cleanup_timeout = 0.25
    goal = rig.send()
    rig.until(lambda: 'arm' in rig.events)
    result = rig.cancel(goal)
    assert result.status == GoalStatus.STATUS_CANCELED
    assert not result.result.cleanup_completed
    assert 'disarm' not in rig.events and 'takeoff' not in rig.events
    assert not rig.send().accepted


@pytest.mark.parametrize('altitude', [-1.0, 0.0, 0.5, 51.0, math.nan, math.inf])
def test_invalid_altitude_rejected_before_commands(rig, altitude):
    assert not rig.send(altitude=altitude).accepted
    assert not rig.events


@pytest.mark.parametrize('timeout', [-1.0, math.nan, math.inf])
def test_invalid_timeout_rejected_before_commands(rig, timeout):
    assert not rig.send(timeout=timeout).accepted
    assert not rig.events


def test_tripped_safety_refuses_takeoff_with_reason(rig):
    rig.safety_pub.publish(String(data='TRIPPED (BRAKE): manual BRAKE'))
    rig.spin_for(0.3)
    goal = rig.send()
    assert goal.accepted
    result = rig.wait(goal.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.success
    assert 'TRIPPED (BRAKE): manual BRAKE' in result.result.message
    assert '/safety/reset' in result.result.message
    assert not rig.events  # no mode change, prearm, arm or takeoff request


def test_takeoff_allowed_again_after_safety_reset(rig):
    rig.safety_pub.publish(String(data='TRIPPED (BRAKE): manual BRAKE'))
    rig.spin_for(0.3)
    assert not rig.wait(rig.send().get_result_async()).result.success
    rig.safety_pub.publish(String(data='OK'))
    rig.spin_for(0.3)
    result = rig.wait(rig.send().get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
