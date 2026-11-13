import cv2
import numpy as np
from collider.src.Helpers import DenormalizedBbox


class BlackSpotFinder:
    @classmethod
    def find_tracking_candidates(cls, tracking_region: DenormalizedBbox, frame: np.ndarray,
                                 threshold: int) -> list[DenormalizedBbox]:
        frame_height, frame_width = frame.shape[0], frame.shape[1]
        tracking_frame_gray = cls._prepare_tracking_frame(frame, tracking_region)
        _, roi_thresholded = cv2.threshold(tracking_frame_gray, threshold, 255,
                                           cv2.THRESH_BINARY_INV)
        countours, _ = cv2.findContours(roi_thresholded, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        candidates = cls._contours_to_bboxes(countours, tracking_region, frame_width, frame_height)
        return candidates

    @staticmethod
    def _prepare_tracking_frame(frame, tracking_region):
        tracking_frame = frame[tracking_region.y: tracking_region.y + tracking_region.h,
                               tracking_region.x: tracking_region.x + tracking_region.w]
        return cv2.cvtColor(tracking_frame, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def _contours_to_bboxes(countours, tracking_region, frame_width, frame_height):
        candidates = []
        for cnt in countours:
            x, y, w, h = cv2.boundingRect(cnt)
            candidates.append(
                DenormalizedBbox(x=tracking_region.x + x, y=tracking_region.y + y, w=w, h=h,
                                 frame_w=frame_width, frame_h=frame_height))
        return candidates
