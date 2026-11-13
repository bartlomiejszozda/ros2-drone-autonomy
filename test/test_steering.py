"""Tests for the centring controller, both steering backends and the steering node."""

import math
import time
from uuid import uuid4

import pytest
import rclpy
from ardupilot_msgs.msg import Status
from collider_interfaces.msg import TargetTrack
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from collider.src.steering.Centering import (
    CenteringController, CenteringGains, DesiredMotion, heading_deg)
from collider.src.steering.SteeringBackends import (
    IGNORE, NEUTRAL, CmdVelBackend, RcBackend, StickScale, rc_channels, stick, to_map_twist)
from collider.src.steering.SteeringNode import SteeringNode
from collider.src.tracker.TargetMotionEstimator import TrackEstimate


def facing(heading):
    half = math.radians(90.0 - heading) / 2
    return (0.0, 0.0, math.sin(half), math.cos(half))


def track(bearing, elevation=0.0, bearing_rate=0.0, stamp=10.0):
    return TrackEstimate(stamp_sec=stamp, measured=True, measurements=10, age_sec=0.0,
                         bearing_deg=bearing, elevation_deg=elevation,
                         bearing_rate_dps=bearing_rate, elevation_rate_dps=0.0)


@pytest.mark.parametrize('heading', [0.0, 90.0, 225.0])
def test_heading_from_attitude(heading):
    assert heading_deg(facing(heading)) == pytest.approx(heading)


def test_no_track_means_hover():
    assert CenteringController().compute(None, facing(0.0), 10.0) == DesiredMotion()


def test_turns_toward_target_on_the_right():
    motion = CenteringController().compute(track(10.0), facing(0.0), 10.0)
    assert motion.yaw_rate_dps == pytest.approx(10.0)
    assert motion.up_mps == 0.0
    assert motion.forward_mps == DesiredMotion().forward_mps and motion.left_mps == 0.0


def test_turns_the_short_way_across_north():
    motion = CenteringController().compute(track(10.0), facing(350.0), 10.0)
    assert motion.yaw_rate_dps == pytest.approx(20.0)
    motion = CenteringController().compute(track(350.0), facing(10.0), 10.0)
    assert motion.yaw_rate_dps == pytest.approx(-20.0)


def test_climbs_toward_target_above_the_horizon():
    motion = CenteringController().compute(track(0.0, elevation=5.0), facing(0.0), 10.0)
    assert motion.up_mps == pytest.approx(0.5)
    assert motion.yaw_rate_dps == 0.0


def test_deadband_and_limits():
    controller = CenteringController(CenteringGains(deadband_deg=2.0))
    assert controller.compute(track(1.5, elevation=-1.5), facing(0.0), 10.0) == DesiredMotion()
    motion = controller.compute(track(170.0, elevation=80.0), facing(0.0), 10.0)
    assert motion.yaw_rate_dps == pytest.approx(45.0)
    assert motion.up_mps == pytest.approx(1.0)


def test_follows_a_moving_bearing_with_feedforward_and_prediction():
    controller = CenteringController()
    # Target was centred, moving clockwise at 5 deg/s; the image is 0.25 s old.
    motion = controller.compute(track(0.0, bearing_rate=5.0, stamp=10.0), facing(0.0), 10.25)
    # 1.25 deg predicted error * gain 1 + 5 deg/s feed-forward.
    assert motion.yaw_rate_dps == pytest.approx(6.25)
    # Prediction is limited to max_prediction_sec (0.3 s): an old track is not extrapolated far.
    motion = controller.compute(track(0.0, bearing_rate=10.0, stamp=10.0), facing(0.0), 15.0)
    assert motion.yaw_rate_dps == pytest.approx(3.0 + 10.0)


@pytest.mark.parametrize('heading, east, north', [(0.0, 0.0, 1.0), (90.0, 1.0, 0.0),
                                                  (180.0, 0.0, -1.0)])
def test_map_twist_rotates_forward_by_heading(heading, east, north):
    msg = to_map_twist(DesiredMotion(forward_mps=1.0), heading)
    assert msg.header.frame_id == 'map'
    assert msg.twist.linear.x == pytest.approx(east, abs=1e-9)
    assert msg.twist.linear.y == pytest.approx(north, abs=1e-9)


def test_map_twist_left_climb_and_yaw_sign():
    msg = to_map_twist(DesiredMotion(left_mps=1.0, up_mps=0.5, yaw_rate_dps=30.0), 0.0)
    assert msg.twist.linear.x == pytest.approx(-1.0)  # left of north is west
    assert msg.twist.linear.y == pytest.approx(DesiredMotion().forward_mps)  # default cruise
    assert msg.twist.linear.z == 0.5
    assert msg.twist.angular.z == pytest.approx(-math.radians(30.0))  # clockwise = negative


def test_stick_mapping_respects_dead_zone_and_full_scale():
    assert stick(0.0, 100.0, 20) == NEUTRAL
    assert stick(0.001, 100.0, 20) == 1520
    assert stick(100.0, 100.0, 20) == 2000
    assert stick(500.0, 100.0, 20) == 2000
    assert stick(-50.0, 100.0, 20) == 1500 - (20 + 240)


def test_rc_channels_directions():
    scale = StickScale()
    roll, pitch, throttle, yaw, *rest = rc_channels(
        DesiredMotion(forward_mps=1.0, left_mps=1.0, up_mps=1.0, yaw_rate_dps=10.0), scale)
    assert pitch < NEUTRAL       # forward = pitch stick pushed (low PWM)
    assert roll < NEUTRAL        # left = roll stick left (low PWM)
    assert throttle > NEUTRAL + scale.throttle_dead_zone  # climb, outside throttle dead zone
    assert yaw > NEUTRAL         # clockwise = yaw stick right
    assert rest == [IGNORE] * 4
    roll, pitch, throttle, yaw, *_ = rc_channels(DesiredMotion(), scale)
    assert pitch < NEUTRAL       # default cruise = pitch stick pushed
    assert [roll, throttle, yaw] == [NEUTRAL] * 3


class FakeMav:
    def __init__(self):
        self.sent = []

    def rc_channels_override_send(self, system, component, *channels):
        self.sent.append(list(channels))


class FakeConnection:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = FakeMav()


def test_rc_backend_sends_overrides_and_hovers_on_stop():
    connection = FakeConnection()
    backend = RcBackend(connection)
    assert backend.required_mode() == 5
    backend.send(DesiredMotion(yaw_rate_dps=20.0), 0.0)
    assert connection.mav.sent[-1][3] > NEUTRAL
    backend.stop()
    assert connection.mav.sent[-1][:4] == [NEUTRAL] * 4


class FakeBackend:
    def __init__(self, mode=4):
        self.mode = mode
        self.sent = []
        self.stops = 0

    def required_mode(self):
        return self.mode

    def send(self, motion, heading):
        self.sent.append((motion, heading))

    def stop(self):
        self.stops += 1

    def name(self):
        return 'FakeBackend'


class SteeringRig:
    def __init__(self, backend=None, kind='cmd_vel'):
        self.context = Context()
        rclpy.init(context=self.context)
        prefix = '/steering_test_' + uuid4().hex
        self.cmd_topic = prefix + '/cmd_vel'
        self.steering = SteeringNode(backend=backend, context=self.context, parameter_overrides=[
            Parameter('backend', value=kind),
            Parameter('track_topic', value=prefix + '/track'),
            Parameter('pose_topic', value=prefix + '/pose'),
            Parameter('status_topic', value=prefix + '/status'),
            Parameter('cmd_vel_topic', value=self.cmd_topic),
            Parameter('rate_hz', value=20.0)])
        self.sim = Node('steering_test_sim', context=self.context)
        self.status_pub = self.sim.create_publisher(Status, prefix + '/status', 10)
        self.pose_pub = self.sim.create_publisher(PoseStamped, prefix + '/pose', 10)
        self.track_pub = self.sim.create_publisher(TargetTrack, prefix + '/track', 10)
        self.enable = self.sim.create_client(SetBool, '/steering/enable')
        self.commands = []
        self.sim.create_subscription(TwistStamped, self.cmd_topic, self.commands.append, 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.steering)
        self.executor.add_node(self.sim)
        self.mode = 4
        self.armed = True
        self.heading = 0.0
        self.bearing = 10.0
        self.sim.create_timer(0.02, self._publish)
        assert self.enable.wait_for_service(timeout_sec=3)

    def _publish(self):
        stamp = self.sim.get_clock().now().to_msg()
        status = Status()
        status.header.stamp = stamp
        status.armed = status.flying = self.armed
        status.mode = self.mode
        self.status_pub.publish(status)
        pose = PoseStamped()
        pose.header.stamp = stamp
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = facing(self.heading)
        self.pose_pub.publish(pose)
        if self.bearing is not None:
            msg = TargetTrack()
            msg.header.stamp = stamp
            msg.valid = msg.measured = True
            msg.measurements = 10
            msg.bearing_deg = self.bearing
            self.track_pub.publish(msg)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def set_enabled(self, value):
        future = self.enable.call_async(SetBool.Request(data=value))
        deadline = time.monotonic() + 3
        while not future.done():
            assert time.monotonic() < deadline
            self.executor.spin_once(timeout_sec=0.01)
        assert future.result().success

    def close(self):
        self.executor.shutdown()
        self.steering.destroy_node()
        self.sim.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.fixture
def make_rig():
    rigs = []

    def make(**kwargs):
        rigs.append(SteeringRig(**kwargs))
        return rigs[-1]
    yield make
    for rig in rigs:
        rig.close()


def test_disabled_steering_sends_nothing(make_rig):
    backend = FakeBackend()
    rig = make_rig(backend=backend)
    rig.spin(0.4)
    assert backend.sent == [] and backend.stops == 0


def test_enabled_steering_turns_toward_target(make_rig):
    backend = FakeBackend()
    rig = make_rig(backend=backend)
    rig.set_enabled(True)
    rig.spin(0.4)
    assert backend.sent
    motion, heading = backend.sent[-1]
    assert motion.yaw_rate_dps == pytest.approx(10.0, abs=0.5)
    assert heading == pytest.approx(0.0, abs=1e-6)


def test_lost_track_hovers_instead_of_stopping(make_rig):
    backend = FakeBackend()
    rig = make_rig(backend=backend)
    rig.set_enabled(True)
    rig.spin(0.3)
    rig.bearing = None
    rig.spin(0.8)  # longer than track_timeout_sec
    assert backend.sent[-1][0] == DesiredMotion()
    assert backend.stops == 0


def test_wrong_mode_stops_once_then_waits(make_rig):
    backend = FakeBackend(mode=5)
    rig = make_rig(backend=backend)
    rig.mode = 5
    rig.set_enabled(True)
    rig.spin(0.3)
    count = len(backend.sent)
    assert count > 0
    rig.mode = 4
    rig.spin(0.5)
    assert backend.stops == 1
    assert len(backend.sent) <= count + 1  # at most one tick before the mode change arrived


def test_disarmed_vehicle_is_never_commanded(make_rig):
    backend = FakeBackend()
    rig = make_rig(backend=backend)
    rig.armed = False
    rig.set_enabled(True)
    rig.spin(0.4)
    assert backend.sent == []


def test_disable_sends_stop(make_rig):
    backend = FakeBackend()
    rig = make_rig(backend=backend)
    rig.set_enabled(True)
    rig.spin(0.3)
    rig.set_enabled(False)
    count = len(backend.sent)
    rig.spin(0.3)
    assert backend.stops == 1
    assert len(backend.sent) == count


def test_cmd_vel_backend_publishes_map_frame_twist(make_rig):
    rig = make_rig()
    assert isinstance(rig.steering._backend, CmdVelBackend)
    rig.set_enabled(True)
    rig.spin(0.5)
    assert rig.commands
    command = rig.commands[-1]
    assert command.header.frame_id == 'map'
    assert command.twist.angular.z == pytest.approx(-math.radians(10.0), abs=0.01)


def test_cmd_vel_stop_sends_zero_velocity(make_rig):
    rig = make_rig()
    rig.set_enabled(True)
    rig.spin(0.5)
    rig.set_enabled(False)
    rig.spin(0.3)
    linear = rig.commands[-1].twist.linear
    # Not the default forward cruise: stopping must not keep flying for GUID_TIMEOUT.
    assert (linear.x, linear.y, linear.z) == (0.0, 0.0, 0.0)
