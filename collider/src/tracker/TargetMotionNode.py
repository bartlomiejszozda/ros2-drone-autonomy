"""
Combine tracker observations with the drone's attitude and publish the target's motion.

This node only observes. It sends no commands to the vehicle.
"""

from bisect import bisect_left
from collections import deque
import signal

import rclpy
from collider_interfaces.msg import TargetObservation, TargetTrack
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from collider.src.tracker.TargetMotionEstimator import (
    TargetMotionEstimator, TrackEstimate, interpolate_quaternion, world_direction)


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class AttitudeHistory:
    """Recent attitudes, so an image can be paired with the attitude at its own time."""

    def __init__(self, max_gap_sec: float, size: int = 200):
        self._max_gap = max_gap_sec
        self._stamps = deque(maxlen=size)
        self._quaternions = deque(maxlen=size)

    def add(self, stamp_sec: float, quaternion_xyzw):
        if self._stamps and stamp_sec <= self._stamps[-1]:
            if stamp_sec < self._stamps[-1]:
                # Simulation time went backwards (restart); old attitudes are meaningless.
                self._stamps.clear()
                self._quaternions.clear()
            else:
                return
        self._stamps.append(stamp_sec)
        self._quaternions.append(quaternion_xyzw)

    def at(self, stamp_sec: float):
        """Attitude at stamp_sec, interpolated between neighbours; None if too far away."""
        if not self._stamps:
            return None
        index = bisect_left(self._stamps, stamp_sec)
        if index == 0 or index == len(self._stamps):
            # Before the oldest or after the newest pose: use the nearest one if close.
            nearest = 0 if index == 0 else -1
            if abs(self._stamps[nearest] - stamp_sec) > self._max_gap:
                return None
            return self._quaternions[nearest]
        t0, t1 = self._stamps[index - 1], self._stamps[index]
        if t1 - t0 > 2 * self._max_gap:
            return None
        fraction = (stamp_sec - t0) / (t1 - t0)
        return interpolate_quaternion(
            self._quaternions[index - 1], self._quaternions[index], fraction)


class TargetMotionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('target_motion', **kwargs)
        observation_topic = self.declare_parameter(
            'observation_topic', '/collider/target').value
        pose_topic = self.declare_parameter('pose_topic', '/ap/v1/pose/filtered').value
        track_topic = self.declare_parameter('track_topic', '/collider/target_track').value
        alpha = float(self.declare_parameter('alpha', 0.5).value)
        beta = float(self.declare_parameter('beta', 0.1).value)
        max_coast = float(self.declare_parameter('max_coast_sec', 0.5).value)
        max_pose_gap = float(self.declare_parameter('max_pose_gap_sec', 0.1).value)
        self._estimator = TargetMotionEstimator(alpha, beta, max_coast)
        self._attitudes = AttitudeHistory(max_pose_gap)
        self._publisher = self.create_publisher(TargetTrack, track_topic, 10)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)
        self.create_subscription(
            TargetObservation, observation_topic, self._on_observation, 10)
        self.get_logger().info(f'Publishing target motion on {track_topic}')

    def _on_pose(self, msg: PoseStamped):
        o = msg.pose.orientation
        self._attitudes.add(stamp_to_sec(msg.header.stamp), (o.x, o.y, o.z, o.w))

    def _on_observation(self, msg: TargetObservation):
        stamp = stamp_to_sec(msg.header.stamp)
        estimate = None
        if msg.tracking:
            attitude = self._attitudes.at(stamp)
            if attitude is None:
                self.get_logger().warning(
                    'No pose near the image time; treating the image as a missed detection',
                    throttle_duration_sec=2.0)
                estimate = self._estimator.coast(stamp)
            else:
                bearing, elevation = world_direction(
                    msg.azimuth_deg, msg.elevation_deg, attitude)
                estimate = self._estimator.update(stamp, bearing, elevation)
        else:
            estimate = self._estimator.coast(stamp)
        self._publisher.publish(self._to_message(msg, estimate))

    @staticmethod
    def _to_message(observation: TargetObservation, estimate: TrackEstimate | None):
        track = TargetTrack()
        track.header = observation.header
        if estimate is None:
            track.valid = False  # other fields stay zero, as in TargetObservation
            return track
        track.valid = True
        track.measured = estimate.measured
        track.measurements = estimate.measurements
        track.age_sec = estimate.age_sec
        track.bearing_deg = estimate.bearing_deg
        track.elevation_deg = estimate.elevation_deg
        track.bearing_rate_dps = estimate.bearing_rate_dps
        track.elevation_rate_dps = estimate.elevation_rate_dps
        return track


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = TargetMotionNode()
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
