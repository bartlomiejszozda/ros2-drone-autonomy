"""
Track the target in camera images and publish what the tracker sees.

This node only observes. It sends no commands to the vehicle.
"""

import math
import signal

import cv2
import numpy as np
import rclpy
from collider_interfaces.msg import TargetObservation
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from collider.src.Helpers import FOCAL, DenormalizedBbox
from collider.src.tracker.BlackSpotTracker import BlackSpotTracker

SUPPORTED_ENCODINGS = ('rgb8', 'bgr8')


def bbox_angles(bbox: DenormalizedBbox, focal_px: float) -> tuple[float, float]:
    """Return (azimuth, elevation) in degrees; positive means right of / above centre."""
    x_px, y_px = bbox.get_pixels_from_center()
    azimuth = math.degrees(math.atan(x_px / focal_px))
    # Image rows grow downwards, so a target above centre has negative y_px.
    elevation = -math.degrees(math.atan(y_px / focal_px))
    return azimuth, elevation


def image_to_frame(msg: Image) -> np.ndarray:
    """Copy a 3-channel, 8-bit image message into a writable (height, width, 3) array."""
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    # A row can be padded beyond width * 3 bytes; step is the real row length.
    return rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()


def frame_to_image(frame: np.ndarray, source: Image) -> Image:
    msg = Image()
    msg.header = source.header
    msg.height, msg.width = frame.shape[0], frame.shape[1]
    msg.encoding = source.encoding
    msg.is_bigendian = source.is_bigendian
    msg.step = frame.strides[0]
    msg.data = frame.tobytes()
    return msg


class TrackerNode(Node):
    """Run a Tracker on every received image and publish one TargetObservation per image."""

    def __init__(self, tracker=None, **kwargs):
        super().__init__('tracker', **kwargs)
        image_topic = self.declare_parameter(
            'image_topic', '/iris/static_camera_sensor/image').value
        observation_topic = self.declare_parameter(
            'observation_topic', '/collider/target').value
        debug_image_topic = self.declare_parameter(
            'debug_image_topic', '/collider/tracker/image').value
        self._focal = float(self.declare_parameter('focal_px', FOCAL).value)
        if not math.isfinite(self._focal) or self._focal <= 0:
            raise ValueError('focal_px must be finite and greater than zero')
        self._tracker = tracker if tracker is not None else BlackSpotTracker()
        # Only the newest image matters: an old frame describes where the target was.
        latest_only = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        # Observations are small; a reliable publisher matches reliable and best-effort readers.
        self._observation_pub = self.create_publisher(TargetObservation, observation_topic, 10)
        # Reliable so every viewer can connect (web_video_server subscribes reliable by
        # default); depth 1 still keeps only the newest image.
        newest_image_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                           history=HistoryPolicy.KEEP_LAST, depth=1)
        self._debug_image_pub = self.create_publisher(
            Image, debug_image_topic, newest_image_reliable)
        self.create_subscription(Image, image_topic, self._on_image, latest_only)
        self._tracking = None
        self.get_logger().info(
            f'Tracking images from {image_topic}; publishing {observation_topic}')

    def _on_image(self, msg: Image):
        if msg.encoding not in SUPPORTED_ENCODINGS:
            self.get_logger().warning(
                f'Ignoring image with encoding {msg.encoding}; expected {SUPPORTED_ENCODINGS}',
                throttle_duration_sec=5.0)
            return
        frame = image_to_frame(msg)
        bbox = self._tracker.track(frame)
        if bbox is None:
            # Search the whole next frame instead of only around the lost position.
            self._tracker.reset()
        observation = self._observation(msg, bbox)
        self._observation_pub.publish(observation)
        self._report_change(observation)
        # Drawing and copying a frame costs time; skip it when nobody watches.
        if self._debug_image_pub.get_subscription_count() > 0:
            self._draw(frame, bbox, msg.encoding)
            self._debug_image_pub.publish(frame_to_image(frame, msg))

    def _observation(self, msg: Image, bbox: DenormalizedBbox | None) -> TargetObservation:
        observation = TargetObservation()
        observation.header = msg.header
        observation.image_width = msg.width
        observation.image_height = msg.height
        if bbox is None:
            observation.tracking = False
            return observation
        observation.tracking = True
        observation.azimuth_deg, observation.elevation_deg = bbox_angles(bbox, self._focal)
        observation.x, observation.y = int(bbox.x), int(bbox.y)
        observation.width, observation.height = int(bbox.w), int(bbox.h)
        return observation

    def _report_change(self, observation: TargetObservation):
        if observation.tracking == self._tracking:
            return
        self._tracking = observation.tracking
        if observation.tracking:
            self.get_logger().info(
                f'Target acquired at azimuth {observation.azimuth_deg:.1f} deg, '
                f'elevation {observation.elevation_deg:.1f} deg')
        else:
            self.get_logger().warning('Target lost; searching the whole image')

    def _draw(self, frame: np.ndarray, bbox: DenormalizedBbox | None, encoding: str):
        if bbox is None:
            red = (255, 0, 0) if encoding == 'rgb8' else (0, 0, 255)
            cv2.putText(frame, 'Target lost', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, red, 2)
            return
        green = (0, 255, 0)
        cv2.rectangle(frame, (bbox.x, bbox.y), (bbox.x + bbox.w, bbox.y + bbox.h), green, 2)
        cv2.putText(frame, self._tracker.name(), (bbox.x, max(bbox.y - 10, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, green, 2)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = TrackerNode()
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
