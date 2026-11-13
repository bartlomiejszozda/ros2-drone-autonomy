"""Target direction geometry, the alpha-beta filter and the target motion node."""

import math
import time
from uuid import uuid4

import pytest
import rclpy
from collider_interfaces.msg import TargetObservation, TargetTrack
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from collider.src.tracker.TargetMotionEstimator import (
    AlphaBetaFilter, TargetMotionEstimator, TrackEstimate, world_direction, wrap_180)
from collider.src.tracker.TargetMotionNode import AttitudeHistory, TargetMotionNode


def about(axis, degrees):
    """Quaternion (x, y, z, w) rotating by `degrees` about body axis 'x', 'y' or 'z'."""
    half = math.radians(degrees) / 2
    q = [0.0, 0.0, 0.0, math.cos(half)]
    q['xyz'.index(axis)] = math.sin(half)
    return tuple(q)


def facing(heading_deg):
    """Level attitude with the nose at a compass heading (ENU yaw = 90 - heading)."""
    return about('z', 90.0 - heading_deg)


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


@pytest.mark.parametrize('heading', [0.0, 45.0, 90.0, 270.0])
def test_centred_target_lies_along_the_nose(heading):
    bearing, elevation = world_direction(0.0, 0.0, facing(heading))
    assert bearing == pytest.approx(heading)
    assert elevation == pytest.approx(0.0, abs=1e-9)


def test_target_right_of_centre_is_clockwise():
    bearing, _ = world_direction(10.0, 0.0, facing(0.0))
    assert bearing == pytest.approx(10.0)
    bearing, _ = world_direction(-10.0, 0.0, facing(0.0))
    assert bearing == pytest.approx(350.0)


def test_nose_up_pitch_adds_to_elevation():
    # In FLU, a negative rotation about y (left) raises the nose.
    nose_up_10 = multiply(facing(0.0), about('y', -10.0))
    bearing, elevation = world_direction(0.0, 5.0, nose_up_10)
    assert bearing == pytest.approx(0.0, abs=1e-9)
    assert elevation == pytest.approx(15.0)


def test_roll_turns_image_up_into_sideways():
    # Rolled 90 deg with the right wing down, "up" in the image points right of the nose.
    rolled = multiply(facing(90.0), about('x', 90.0))
    bearing, elevation = world_direction(0.0, 10.0, rolled)
    assert bearing == pytest.approx(100.0)
    assert elevation == pytest.approx(0.0, abs=1e-9)


def test_still_target_keeps_bearing_while_drone_turns():
    # The drone turns 20 deg left, so the same target appears 20 deg further right.
    assert world_direction(0.0, 0.0, facing(0.0))[0] == pytest.approx(0.0)
    assert world_direction(20.0, 0.0, facing(340.0))[0] == pytest.approx(0.0)


def test_wrap_180_takes_the_short_way():
    assert wrap_180(1.0 - 359.0) == pytest.approx(2.0)
    assert wrap_180(359.0 - 1.0) == pytest.approx(-2.0)


def test_filter_learns_a_constant_rate_across_north():
    f = AlphaBetaFilter(alpha=0.5, beta=0.1, wrap_degrees=True)
    dt = 0.05
    f.reset(340.0)
    for step in range(1, 100):
        f.update((340.0 + 10.0 * step * dt) % 360.0, dt)
    assert f.rate == pytest.approx(10.0, abs=0.1)
    assert f.value == pytest.approx((340.0 + 10.0 * 99 * dt) % 360.0, abs=0.1)


def test_filter_smooths_noise():
    f = AlphaBetaFilter(alpha=0.5, beta=0.1)
    f.reset(0.0)
    for step in range(1, 200):
        f.update(1.0 if step % 2 else -1.0, 0.05)  # noise around zero, no motion
    assert abs(f.value) < 1.0
    assert abs(f.rate) < 5.0


def feed(estimator, start, count, rate=10.0, dt=0.05):
    estimate = None
    for step in range(count):
        t = start + step * dt
        estimate = estimator.update(t, (rate * t) % 360.0, 2.0)
    return estimate


def test_estimator_counts_measurements_and_reports_rate():
    estimate = feed(TargetMotionEstimator(), start=0.0, count=60)
    assert estimate.measured
    assert estimate.measurements == 60
    assert estimate.age_sec == 0.0
    assert estimate.bearing_rate_dps == pytest.approx(10.0, abs=0.2)
    assert estimate.elevation_rate_dps == pytest.approx(0.0, abs=1e-9)


def test_estimator_predicts_through_a_short_loss_then_drops_the_track():
    estimator = TargetMotionEstimator(max_coast_sec=0.5)
    last = feed(estimator, start=0.0, count=60)
    coasting = estimator.coast(last.stamp_sec + 0.3)
    assert not coasting.measured
    assert coasting.age_sec == pytest.approx(0.3)
    assert coasting.bearing_deg == pytest.approx(last.bearing_deg + 3.0, abs=0.1)
    assert estimator.coast(last.stamp_sec + 0.6) is None
    restarted = estimator.update(last.stamp_sec + 0.7, 100.0, 0.0)
    assert restarted.measurements == 1
    assert restarted.bearing_rate_dps == 0.0


def test_estimator_restarts_when_time_goes_backwards():
    estimator = TargetMotionEstimator()
    feed(estimator, start=100.0, count=20)
    assert estimator.update(1.0, 50.0, 0.0).measurements == 1


def test_coast_without_any_track_has_no_estimate():
    assert TargetMotionEstimator().coast(1.0) is None


def test_track_estimate_predicts_forward_in_time():
    estimate = TrackEstimate(stamp_sec=10.0, measured=True, measurements=5, age_sec=0.0,
                             bearing_deg=355.0, elevation_deg=1.0,
                             bearing_rate_dps=20.0, elevation_rate_dps=-2.0)
    bearing, elevation = estimate.predict(10.5)
    assert bearing == pytest.approx(5.0)
    assert elevation == pytest.approx(0.0)


def test_attitude_history_interpolates_between_poses():
    history = AttitudeHistory(max_gap_sec=0.1)
    history.add(1.0, facing(0.0))
    history.add(1.1, facing(20.0))
    middle = history.at(1.05)
    assert world_direction(0.0, 0.0, middle)[0] == pytest.approx(10.0, abs=0.1)
    assert history.at(1.15) == facing(20.0)
    assert history.at(1.3) is None
    assert history.at(0.5) is None


class MotionRig:
    def __init__(self):
        self.context = Context()
        rclpy.init(context=self.context)
        prefix = '/motion_test_' + uuid4().hex
        self.motion = TargetMotionNode(context=self.context, parameter_overrides=[
            Parameter('observation_topic', value=prefix + '/target'),
            Parameter('pose_topic', value=prefix + '/pose'),
            Parameter('track_topic', value=prefix + '/track')])
        self.sim = Node('motion_test_sim', context=self.context)
        self.pose_pub = self.sim.create_publisher(PoseStamped, prefix + '/pose', 10)
        self.observation_pub = self.sim.create_publisher(
            TargetObservation, prefix + '/target', 10)
        self.tracks = []
        self.sim.create_subscription(TargetTrack, prefix + '/track', self.tracks.append, 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.motion)
        self.executor.add_node(self.sim)
        self._spin_until(lambda: self.pose_pub.get_subscription_count() > 0
                         and self.observation_pub.get_subscription_count() > 0
                         and self.motion.count_subscribers(prefix + '/track') > 0)

    def _spin_until(self, condition, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not condition():
            assert time.monotonic() < deadline, 'timed out'
            self.executor.spin_once(timeout_sec=0.05)

    def pose(self, t, quaternion):
        msg = PoseStamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int(round(t % 1 * 1e9))
        msg.pose.orientation.x, msg.pose.orientation.y, \
            msg.pose.orientation.z, msg.pose.orientation.w = quaternion
        self.pose_pub.publish(msg)
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.01)

    def observe(self, t, azimuth=0.0, elevation=0.0, tracking=True):
        msg = TargetObservation()
        msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int(round(t % 1 * 1e9))
        msg.tracking = tracking
        msg.azimuth_deg, msg.elevation_deg = azimuth, elevation
        count = len(self.tracks)
        self.observation_pub.publish(msg)
        self._spin_until(lambda: len(self.tracks) > count)
        return self.tracks[-1]

    def close(self):
        self.executor.shutdown()
        self.motion.destroy_node()
        self.sim.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.fixture
def rig():
    rig = MotionRig()
    yield rig
    rig.close()


def test_node_removes_drone_rotation(rig):
    rig.pose(10.0, facing(0.0))
    rig.pose(10.05, facing(340.0))
    first = rig.observe(10.0, azimuth=0.0)
    second = rig.observe(10.05, azimuth=20.0)
    assert first.valid and second.valid
    assert first.measured and second.measurements == 2
    assert first.header.stamp.sec == 10
    assert second.bearing_deg == pytest.approx(0.0, abs=1e-6) or \
        second.bearing_deg == pytest.approx(360.0, abs=1e-6)
    assert second.bearing_rate_dps == pytest.approx(0.0, abs=1e-6)


def test_node_without_pose_publishes_invalid_track(rig):
    track = rig.observe(5.0, azimuth=3.0)
    assert not track.valid
    assert track.bearing_deg == 0.0


def test_node_coasts_when_target_is_lost(rig):
    rig.pose(20.0, facing(90.0))
    rig.observe(20.0)
    lost = rig.observe(20.2, tracking=False)
    assert lost.valid and not lost.measured
    assert lost.age_sec == pytest.approx(0.2)
    assert not rig.observe(21.0, tracking=False).valid
