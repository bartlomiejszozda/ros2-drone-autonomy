"""Opt-in integration test using real PyTorch YOLO weights and an aerial image."""

import os
from pathlib import Path

import pytest

from collider.src.detector.BenchmarkYolo import benchmark, load_image


pytestmark = pytest.mark.skipif(
    os.environ.get('RUN_YOLO_INTEGRATION') != '1',
    reason='set RUN_YOLO_INTEGRATION=1 to load real VisDrone YOLO weights')


def test_real_yolo_detects_vehicle_and_reports_timing():
    model_path = os.environ.get('YOLO_MODEL', '/opt/models/yolo11n-visdrone.pt')
    device = os.environ.get('YOLO_DEVICE', '')
    image_size = int(os.environ.get('YOLO_IMGSZ', '1280'))
    fixture = Path(__file__).with_name('drone_vehicles.jpg')
    result = benchmark(model_path, load_image(str(fixture)), device=device,
                       warmup=1, iterations=3, image_size=image_size)
    print(f'load={result.load_seconds:.3f}s cold={result.cold_seconds:.3f}s '
          f'median={result.median_seconds:.3f}s p95={result.p95_seconds:.3f}s')
    assert any(
        item.class_name in {'car', 'van', 'bus', 'truck'} for item in result.detections)
    assert result.load_seconds > 0
    assert all(sample > 0 for sample in result.samples_seconds)
