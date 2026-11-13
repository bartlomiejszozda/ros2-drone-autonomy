"""
Find vehicles in one image with a YOLO model (Ultralytics).

Plain Python, no ROS. The model is passed in, so tests can use a fake one without torch.
"""

from dataclasses import dataclass

import numpy as np

from collider.src.Helpers import DenormalizedBbox

# Road-vehicle classes in the VisDrone model. It distinguishes vans from cars.
DEFAULT_CLASSES = ('car', 'van', 'bus', 'truck')


@dataclass
class Detection:
    bbox: DenormalizedBbox
    class_name: str
    confidence: float
    track_id: int = -1  # -1: not tracked


def load_yolo(model_path: str):
    """Load a .pt, .onnx or .engine model; imported here so tests do not need torch."""
    from ultralytics import YOLO
    return YOLO(model_path)


class YoloCarDetector:
    """Run a YOLO model on one BGR frame and return the vehicles it found."""

    def __init__(self, model, classes=DEFAULT_CLASSES, confidence=0.25, device='',
                 image_size=1280):
        self._model = model
        # Class numbers differ by dataset, so always resolve them from model metadata.
        ids_by_name = {name: class_id for class_id, name in model.names.items()}
        unknown = [name for name in classes if name not in ids_by_name]
        if unknown:
            raise ValueError(f'Classes not known to the model: {unknown}')
        self._class_ids = [ids_by_name[name] for name in classes]
        self._confidence = confidence
        self._device = device or None  # None: Ultralytics picks cuda:0 if it can, else cpu
        self._image_size = image_size

    def detect(self, bgr_frame: np.ndarray) -> list[Detection]:
        result = self._model.predict(bgr_frame, classes=self._class_ids, conf=self._confidence,
                                     device=self._device, imgsz=self._image_size,
                                     verbose=False)[0]
        frame_h, frame_w = bgr_frame.shape[:2]
        boxes = result.boxes
        # Corners as floats (x1, y1, x2, y2) in pixels of the input frame.
        corners = boxes.xyxy.tolist()
        track_ids = boxes.id.tolist() if boxes.id is not None else [-1] * len(corners)
        detections = []
        for (x1, y1, x2, y2), class_id, confidence, track_id in zip(
                corners, boxes.cls.tolist(), boxes.conf.tolist(), track_ids):
            bbox = DenormalizedBbox(x=round(x1), y=round(y1), w=round(x2 - x1),
                                    h=round(y2 - y1), frame_w=frame_w, frame_h=frame_h)
            detections.append(Detection(bbox, result.names[int(class_id)], float(confidence),
                                        int(track_id)))
        return detections
