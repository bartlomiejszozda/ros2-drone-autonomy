"""
Detect vehicles in camera images with YOLO and publish where they are.

This node only observes. It sends no commands to the vehicle.
"""

import math
import signal

import cv2
import numpy as np
import rclpy
from collider_interfaces.msg import CarDetection, CarDetections
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from collider.src.detector.YoloCarDetector import (DEFAULT_CLASSES, Detection,
                                                   YoloCarDetector, load_yolo)
from collider.src.Helpers import FOCAL
from collider.src.tracker.TrackerNode import (SUPPORTED_ENCODINGS, bbox_angles,
                                              frame_to_image, image_to_frame)


class CarDetectorNode(Node):
    """Run the detector on every received image and publish one CarDetections per image."""

    def __init__(self, detector=None, **kwargs):
        super().__init__('car_detector', **kwargs)
        image_topic = self.declare_parameter(
            'image_topic', '/iris/static_camera_sensor/image').value
        detections_topic = self.declare_parameter(
            'detections_topic', '/collider/cars').value
        debug_image_topic = self.declare_parameter(
            'debug_image_topic', '/collider/cars/image').value
        model_path = self.declare_parameter(
            'model', '/opt/models/yolo11n-visdrone.pt').value
        classes = list(self.declare_parameter('classes', list(DEFAULT_CLASSES)).value)
        confidence = float(self.declare_parameter('confidence', 0.25).value)
        device = self.declare_parameter('device', '').value
        image_size = int(self.declare_parameter('imgsz', 1280).value)
        if image_size <= 0:
            raise ValueError('imgsz must be greater than zero')
        self._focal = float(self.declare_parameter('focal_px', FOCAL).value)
        if not math.isfinite(self._focal) or self._focal <= 0:
            raise ValueError('focal_px must be finite and greater than zero')
        if detector is None:
            # Loading takes seconds (torch, CUDA); images arriving meanwhile are not queued.
            self.get_logger().info(f'Loading {model_path}')
            detector = YoloCarDetector(
                load_yolo(model_path), classes, confidence, device, image_size)
        self._detector = detector
        # Same QoS as TrackerNode: newest image in, reliable out.
        latest_only = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        newest_image_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                           history=HistoryPolicy.KEEP_LAST, depth=1)
        self._detections_pub = self.create_publisher(CarDetections, detections_topic, 10)
        self._debug_image_pub = self.create_publisher(
            Image, debug_image_topic, newest_image_reliable)
        self.create_subscription(Image, image_topic, self._on_image, latest_only)
        self._count = None
        self.get_logger().info(
            f'Detecting {classes} in {image_topic}; publishing {detections_topic}')

    def _on_image(self, msg: Image):
        if msg.encoding not in SUPPORTED_ENCODINGS:
            self.get_logger().warning(
                f'Ignoring image with encoding {msg.encoding}; expected {SUPPORTED_ENCODINGS}',
                throttle_duration_sec=5.0)
            return
        frame = image_to_frame(msg)
        # YOLO, like OpenCV, expects BGR; Gazebo sends rgb8. Without this red and blue swap.
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else frame
        detections = self._detector.detect(bgr)
        self._detections_pub.publish(self._detections(msg, detections))
        self._report_change(len(detections))
        # Drawing and copying a frame costs time; skip it when nobody watches.
        if self._debug_image_pub.get_subscription_count() > 0:
            self._draw(frame, detections)
            self._debug_image_pub.publish(frame_to_image(frame, msg))

    def _detections(self, msg: Image, detections: list[Detection]) -> CarDetections:
        out = CarDetections()
        out.header = msg.header
        out.image_width, out.image_height = msg.width, msg.height
        for detection in detections:
            car = CarDetection()
            car.class_name = detection.class_name
            car.confidence = detection.confidence
            car.track_id = detection.track_id
            car.azimuth_deg, car.elevation_deg = bbox_angles(detection.bbox, self._focal)
            bbox = detection.bbox
            car.x, car.y, car.width, car.height = bbox.x, bbox.y, bbox.w, bbox.h
            out.cars.append(car)
        return out

    def _report_change(self, count: int):
        if count == self._count:
            return
        self._count = count
        self.get_logger().info(f'Vehicles in view: {count}')

    @staticmethod
    def _draw(frame: np.ndarray, detections: list[Detection]):
        # Yellow is (255, 255, 0) in RGB and (0, 255, 255) in BGR; cyan reads well in both.
        color = (0, 255, 255)
        for detection in detections:
            bbox = detection.bbox
            label = f'{detection.class_name} {detection.confidence:.2f}'
            if detection.track_id >= 0:
                label = f'#{detection.track_id} {label}'
            cv2.rectangle(frame, (bbox.x, bbox.y), (bbox.x + bbox.w, bbox.y + bbox.h), color, 2)
            cv2.putText(frame, label, (bbox.x, max(bbox.y - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = CarDetectorNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Under ros2 launch, Ctrl+C reaches the node twice; see TrackerNode.main.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
