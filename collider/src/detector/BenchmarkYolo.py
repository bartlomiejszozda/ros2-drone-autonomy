"""Benchmark the real YOLO car detector on a local or remote image."""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import statistics
import time
from urllib.request import urlopen

import cv2
import numpy as np

from collider.src.detector.YoloCarDetector import YoloCarDetector, load_yolo


DEFAULT_IMAGE = ('https://source.roboflow.com/WD8jyT4rYRekug52HV2T/'
                 '01QmL7sdP4j5CPoNxUiB/original.jpg')


@dataclass
class BenchmarkResult:
    """Timing and detection results from one benchmark run."""

    load_seconds: float
    cold_seconds: float
    samples_seconds: list[float]
    detections: list

    @property
    def median_seconds(self):
        """Return the median warmed inference time."""
        return statistics.median(self.samples_seconds)

    @property
    def p95_seconds(self):
        """Return the nearest-rank 95th percentile warmed inference time."""
        ordered = sorted(self.samples_seconds)
        return ordered[math.ceil(0.95 * len(ordered)) - 1]


def load_image(source: str) -> np.ndarray:
    """Read an image path or URL as an OpenCV BGR array."""
    if source.startswith(('http://', 'https://')):
        with urlopen(source, timeout=30) as response:
            encoded = np.frombuffer(response.read(), dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    else:
        frame = cv2.imread(str(Path(source).expanduser()))
    if frame is None:
        raise ValueError(f'Could not decode image: {source}')
    return frame


def benchmark(model_path: str, frame: np.ndarray, device='', warmup=2,
              iterations=10, image_size=1280) -> BenchmarkResult:
    """Load a real model and measure cold and warmed end-to-end detection latency."""
    if warmup < 0 or iterations < 1:
        raise ValueError('warmup must be >= 0 and iterations must be >= 1')

    started = time.perf_counter()
    detector = YoloCarDetector(
        load_yolo(model_path), device=device, image_size=image_size)
    load_seconds = time.perf_counter() - started

    started = time.perf_counter()
    detections = detector.detect(frame)
    cold_seconds = time.perf_counter() - started

    for _ in range(warmup):
        detector.detect(frame)

    samples = []
    for _ in range(iterations):
        started = time.perf_counter()
        detections = detector.detect(frame)
        samples.append(time.perf_counter() - started)
    return BenchmarkResult(load_seconds, cold_seconds, samples, detections)


def _milliseconds(seconds):
    return f'{seconds * 1000:.1f} ms'


def main(args=None):
    """Run the detector benchmark and print a compact report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='/opt/models/yolo11n-visdrone.pt')
    parser.add_argument('--image', default=DEFAULT_IMAGE,
                        help='local path or URL (default: top-down VisDrone frame)')
    parser.add_argument('--device', default='', help="'cpu', 'cuda:0', or auto when empty")
    parser.add_argument('--warmup', type=int, default=2)
    parser.add_argument('--iterations', type=int, default=10)
    parser.add_argument('--imgsz', type=int, default=1280,
                        help='Ultralytics inference size (default: 1280)')
    options = parser.parse_args(args)

    frame = load_image(options.image)
    result = benchmark(options.model, frame, options.device, options.warmup,
                       options.iterations, options.imgsz)
    print(f'image: {frame.shape[1]}x{frame.shape[0]} from {options.image}')
    print(f'model load: {_milliseconds(result.load_seconds)}')
    print(f'cold inference: {_milliseconds(result.cold_seconds)}')
    print(f'warmed inference: median {_milliseconds(result.median_seconds)}, '
          f'p95 {_milliseconds(result.p95_seconds)}, '
          f'{1 / result.median_seconds:.1f} images/s')
    if result.detections:
        found = ', '.join(
            f'{item.class_name} {item.confidence:.2f}' for item in result.detections)
    else:
        found = 'none'
    print(f'vehicles: {found}')


if __name__ == '__main__':
    main()
