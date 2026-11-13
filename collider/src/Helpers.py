import copy
import logging
from dataclasses import dataclass

import numpy as np

log = logging.getLogger("collider")
logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s', level="DEBUG")

# Pinhole focal length in pixels: (640 / 2) / tan(2 rad / 2), matching camera_info fx.
FOCAL = 205.47


@dataclass
class DenormalizedBbox:
    x: int
    y: int
    w: int
    h: int
    frame_w: int
    frame_h: int

    def get_center(self) -> (int, int):
        return self.x + self.w // 2, self.y + self.h // 2

    def distance(self, other: 'DenormalizedBbox') -> float:
        cx0, cy0 = other.get_center()
        cx1, cy1 = self.get_center()
        return np.sqrt((cx1 - cx0) ** 2 + (cy1 - cy0) ** 2)

    def get_pixels_from_center(self) -> (int, int):
        x_from_center = self.x + self.w / 2 - self.frame_w / 2
        y_from_center = self.y + self.h / 2 - self.frame_h / 2
        return x_from_center, y_from_center

    def get_expanded_by(self, pixels: int) -> 'DenormalizedBbox':
        enlarged_bbox = copy.copy(self)
        enlarged_bbox.x -= pixels
        enlarged_bbox.y -= pixels
        enlarged_bbox.w += 2 * pixels
        enlarged_bbox.h += 2 * pixels
        enlarged_bbox._limit_to_frame_size()
        return enlarged_bbox

    def _limit_to_frame_size(self):
        if self.x < 0:
            self.x = 0
        if self.y < 0:
            self.y = 0
        if self.x + self.w > self.frame_w:
            self.w = self.frame_w - self.x
        if self.y + self.h > self.frame_h:
            self.h = self.frame_h - self.y
