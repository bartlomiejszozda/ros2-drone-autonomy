"""Car detector and detector node tests, using a stub model instead of YOLO weights."""

import time
from types import SimpleNamespace
from uuid import uuid4

import numpy as np
import pytest
import rclpy
from collider_interfaces.msg import CarDetections
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from collider.src.detector.CarDetectorNode import CarDetectorNode
from collider.src.detector.YoloCarDetector import Detection, YoloCarDetector
from collider.src.Helpers import DenormalizedBbox

WIDTH, HEIGHT = 160, 120
MODEL_CLASSES = {0: 'person', 2: 'car', 4: 'van', 5: 'bus', 7: 'truck'}


class FakeYolo:
    """Answers predict() with fixed boxes, shaped like Ultralytics Results (lists, not tensors)."""

    names = MODEL_CLASSES

    def __init__(self, xyxy=(), cls=(), conf=(), ids=None):
        self.boxes = SimpleNamespace(xyxy=np.array(xyxy, dtype=float).reshape(-1, 4),
                                     cls=np.array(cls, dtype=float),
                                     conf=np.array(conf, dtype=float),
                                     id=None if ids is None else np.array(ids, dtype=float))
        self.calls = []

    def predict(self, frame, **kwargs):
        self.calls.append((frame, kwargs))
        return [SimpleNamespace(boxes=self.boxes, names=self.names)]


def test_converts_corners_to_bboxes():
    model = FakeYolo(xyxy=[[10.4, 20.0, 50.6, 40.0]], cls=[2], conf=[0.9])
    detections = YoloCarDetector(model).detect(np.zeros((HEIGHT, WIDTH, 3), np.uint8))
    assert len(detections) == 1
    car = detections[0]
    assert (car.bbox.x, car.bbox.y, car.bbox.w, car.bbox.h) == (10, 20, 40, 20)
    assert (car.bbox.frame_w, car.bbox.frame_h) == (WIDTH, HEIGHT)
    assert car.class_name == 'car'
    assert car.confidence == pytest.approx(0.9)
    assert car.track_id == -1


def test_passes_class_ids_confidence_and_device():
    model = FakeYolo()
    detector = YoloCarDetector(model, classes=['car', 'truck'], confidence=0.6,
                               device='cpu', image_size=1280)
    assert detector.detect(np.zeros((HEIGHT, WIDTH, 3), np.uint8)) == []
    _, kwargs = model.calls[0]
    assert kwargs['classes'] == [2, 7]
    assert kwargs['conf'] == 0.6
    assert kwargs['device'] == 'cpu'
    assert kwargs['imgsz'] == 1280


def test_keeps_track_ids_when_the_model_tracks():
    model = FakeYolo(xyxy=[[0, 0, 10, 10], [20, 20, 30, 30]], cls=[2, 7], conf=[0.5, 0.8],
                     ids=[3, 4])
    detections = YoloCarDetector(model).detect(np.zeros((HEIGHT, WIDTH, 3), np.uint8))
    assert [(d.class_name, d.track_id) for d in detections] == [('car', 3), ('truck', 4)]


def test_rejects_unknown_class_names():
    with pytest.raises(ValueError, match='bicycle'):
        YoloCarDetector(FakeYolo(), classes=['car', 'bicycle'])


class FakeDetector:
    """Returns prepared detections and remembers the frames it saw."""

    def __init__(self, detections=()):
        self.detections = list(detections)
        self.frames = []

    def detect(self, bgr_frame):
        self.frames.append(bgr_frame)
        return self.detections


def make_image(stamp_sec=0, encoding='rgb8'):
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    frame[:, :, 0] = 200  # red in rgb8
    msg = Image()
    msg.header.stamp.sec = stamp_sec
    msg.header.frame_id = 'camera'
    msg.height, msg.width = HEIGHT, WIDTH
    msg.encoding = encoding
    msg.step = WIDTH * 3
    msg.data = frame.tobytes()
    return msg


class DetectorRig:
    def __init__(self, detector):
        self.context = Context()
        rclpy.init(context=self.context)
        prefix = '/car_detector_test_' + uuid4().hex
        self.node = CarDetectorNode(detector=detector, context=self.context,
                                    parameter_overrides=[
                                        Parameter('image_topic', value=prefix + '/image'),
                                        Parameter('detections_topic', value=prefix + '/cars'),
                                        Parameter('debug_image_topic', value=prefix + '/debug'),
                                        Parameter('focal_px', value=100.0)])
        self.camera = Node('car_detector_test_camera', context=self.context)
        latest_only = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        self.image_pub = self.camera.create_publisher(Image, prefix + '/image', latest_only)
        self.received = []
        self.camera.create_subscription(
            CarDetections, prefix + '/cars', self.received.append, 10)
        self.debug_images = []
        self.camera.create_subscription(
            Image, prefix + '/debug', self.debug_images.append, 10)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.executor.add_node(self.camera)
        self._spin_until(lambda: self.image_pub.get_subscription_count() > 0)

    def _spin_until(self, condition, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not condition():
            assert time.monotonic() < deadline, 'timed out'
            self.executor.spin_once(timeout_sec=0.05)

    def send(self, image):
        count = len(self.received)
        self.image_pub.publish(image)
        self._spin_until(lambda: len(self.received) > count)
        return self.received[-1]

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.camera.destroy_node()
        rclpy.shutdown(context=self.context)


def car_at(x, y, w=20, h=10, track_id=-1):
    return Detection(DenormalizedBbox(x, y, w, h, WIDTH, HEIGHT), 'car', 0.75, track_id)


@pytest.fixture
def rig_factory():
    rigs = []

    def make(detections=()):
        detector = FakeDetector(detections)
        rig = DetectorRig(detector)
        rigs.append(rig)
        return rig, detector

    yield make
    for rig in rigs:
        rig.close()


def test_publishes_every_car_with_image_stamp(rig_factory):
    rig, _ = rig_factory([car_at(120, 10), car_at(10, 100, track_id=7)])
    msg = rig.send(make_image(stamp_sec=42))
    assert msg.header.stamp.sec == 42
    assert msg.header.frame_id == 'camera'
    assert (msg.image_width, msg.image_height) == (WIDTH, HEIGHT)
    assert len(msg.cars) == 2
    right_up, left_down = msg.cars
    assert (right_up.x, right_up.y, right_up.width, right_up.height) == (120, 10, 20, 10)
    assert right_up.class_name == 'car'
    assert right_up.confidence == pytest.approx(0.75)
    # Box centre (130, 15) is right of and above the image centre (80, 60).
    assert right_up.azimuth_deg > 0 and right_up.elevation_deg > 0
    assert left_down.azimuth_deg < 0 and left_down.elevation_deg < 0
    assert (right_up.track_id, left_down.track_id) == (-1, 7)


def test_no_cars_is_still_published(rig_factory):
    rig, _ = rig_factory()
    assert rig.send(make_image()).cars == []


def test_detector_gets_bgr(rig_factory):
    rig, detector = rig_factory()
    rig.send(make_image(encoding='rgb8'))
    rig.send(make_image(encoding='bgr8'))
    rgb_input, bgr_input = detector.frames
    # The same bytes: as rgb8 channel 0 is red and must be moved to channel 2.
    assert (rgb_input[0, 0] == [0, 0, 200]).all()
    assert (bgr_input[0, 0] == [200, 0, 0]).all()


def test_draws_boxes_when_someone_watches(rig_factory):
    rig, _ = rig_factory([car_at(40, 40)])
    rig._spin_until(lambda: rig.node._debug_image_pub.get_subscription_count() > 0)
    rig.send(make_image())
    rig._spin_until(lambda: rig.debug_images)
    debug = rig.debug_images[-1]
    assert debug.encoding == 'rgb8'
    frame = np.frombuffer(debug.data, np.uint8).reshape(HEIGHT, WIDTH, 3)
    # The top edge of the box is drawn in (0, 255, 255).
    assert (frame[40, 50] == [0, 255, 255]).all()


def test_ignores_unsupported_encoding(rig_factory):
    rig, detector = rig_factory()
    rig.image_pub.publish(make_image(encoding='mono8'))
    for _ in range(10):
        rig.executor.spin_once(timeout_sec=0.02)
    assert rig.received == [] and detector.frames == []
