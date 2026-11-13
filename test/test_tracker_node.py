"""Tracker node tests driven by synthetic camera images."""

import time
from uuid import uuid4

import numpy as np
import pytest
import rclpy
from collider_interfaces.msg import TargetObservation
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from collider.src.Helpers import DenormalizedBbox
from collider.src.tracker.TrackerNode import TrackerNode, bbox_angles, image_to_frame

WIDTH, HEIGHT = 160, 120


def bbox_centered_at(cx, cy, size=10):
    return DenormalizedBbox(x=cx - size // 2, y=cy - size // 2, w=size, h=size,
                            frame_w=WIDTH, frame_h=HEIGHT)


def test_centered_target_has_zero_angles():
    assert bbox_angles(bbox_centered_at(80, 60), focal_px=100.0) == (0.0, 0.0)


def test_angle_signs_follow_right_and_up():
    azimuth, elevation = bbox_angles(bbox_centered_at(130, 10), focal_px=100.0)
    assert azimuth > 0
    assert elevation > 0
    azimuth, elevation = bbox_angles(bbox_centered_at(30, 110), focal_px=100.0)
    assert azimuth < 0
    assert elevation < 0


def test_offset_equal_to_focal_length_is_45_degrees():
    azimuth, elevation = bbox_angles(bbox_centered_at(80 + 50, 60), focal_px=50.0)
    assert azimuth == pytest.approx(45.0)
    assert elevation == pytest.approx(0.0)


def make_image(square_at=None, stamp_sec=0, padding=0):
    frame = np.full((HEIGHT, WIDTH, 3), 255, dtype=np.uint8)
    if square_at is not None:
        x, y = square_at
        frame[y:y + 10, x:x + 10] = 0
    rows = np.zeros((HEIGHT, WIDTH * 3 + padding), dtype=np.uint8)
    rows[:, :WIDTH * 3] = frame.reshape(HEIGHT, -1)
    msg = Image()
    msg.header.stamp.sec = stamp_sec
    msg.header.frame_id = 'camera'
    msg.height, msg.width = HEIGHT, WIDTH
    msg.encoding = 'rgb8'
    msg.step = WIDTH * 3 + padding
    msg.data = rows.tobytes()
    return msg


def test_image_to_frame_ignores_row_padding():
    frame = image_to_frame(make_image(square_at=(20, 30), padding=4))
    assert frame.shape == (HEIGHT, WIDTH, 3)
    assert (frame[30:40, 20:30] == 0).all()
    assert (frame[0, :] == 255).all()


class TrackerRig:
    def __init__(self):
        self.context = Context()
        rclpy.init(context=self.context)
        prefix = '/tracker_test_' + uuid4().hex
        self.tracker = TrackerNode(context=self.context, parameter_overrides=[
            Parameter('image_topic', value=prefix + '/image'),
            Parameter('observation_topic', value=prefix + '/target'),
            Parameter('debug_image_topic', value=prefix + '/debug'),
            Parameter('focal_px', value=100.0)])
        self.camera = Node('tracker_test_camera', context=self.context)
        latest_only = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        self.image_pub = self.camera.create_publisher(Image, prefix + '/image', latest_only)
        self.observations = []
        self.camera.create_subscription(
            TargetObservation, prefix + '/target', self.observations.append, 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.tracker)
        self.executor.add_node(self.camera)
        self._spin_until(lambda: self.image_pub.get_subscription_count() > 0)

    def _spin_until(self, condition, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not condition():
            assert time.monotonic() < deadline, 'timed out'
            self.executor.spin_once(timeout_sec=0.05)

    def send(self, image):
        count = len(self.observations)
        self.image_pub.publish(image)
        self._spin_until(lambda: len(self.observations) > count)
        return self.observations[-1]

    def close(self):
        self.executor.shutdown()
        self.tracker.destroy_node()
        self.camera.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.fixture
def rig():
    rig = TrackerRig()
    yield rig
    rig.close()


def test_publishes_observation_with_image_stamp(rig):
    observation = rig.send(make_image(square_at=(120, 20), stamp_sec=42))
    assert observation.tracking
    assert observation.header.stamp.sec == 42
    assert observation.header.frame_id == 'camera'
    assert (observation.x, observation.y, observation.width, observation.height) == (
        120, 20, 10, 10)
    assert (observation.image_width, observation.image_height) == (WIDTH, HEIGHT)
    # Square centre (125, 25) is right of and above the image centre (80, 60).
    assert observation.azimuth_deg > 0
    assert observation.elevation_deg > 0


def test_lost_target_is_reported_then_reacquired_anywhere(rig):
    assert rig.send(make_image(square_at=(10, 10))).tracking
    lost = rig.send(make_image())
    assert not lost.tracking
    assert (lost.azimuth_deg, lost.elevation_deg, lost.width) == (0.0, 0.0, 0)
    # Far from the old position: found only because loss resets the search region.
    found = rig.send(make_image(square_at=(140, 100)))
    assert found.tracking
    assert (found.x, found.y) == (140, 100)


def test_ignores_unsupported_encoding(rig):
    image = make_image(square_at=(10, 10))
    image.encoding = 'mono8'
    rig.image_pub.publish(image)
    for _ in range(10):
        rig.executor.spin_once(timeout_sec=0.02)
    assert rig.observations == []
