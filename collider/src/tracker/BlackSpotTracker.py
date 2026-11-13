import numpy as np

from collider.src.Helpers import DenormalizedBbox, log
from collider.src.tracker.Tracker import Tracker
from collider.src.tracker.BlackSpotFinder import BlackSpotFinder


class BlackSpotTracker(Tracker):
    def __init__(self):
        self._prev_bbox: DenormalizedBbox = None

    THRESHOLD = 1
    TRACKING_REGION_EXPANDED_BY = 100

    def track(self, frame: np.ndarray) -> DenormalizedBbox | None:
        if self._prev_bbox is None:
            frame_height, frame_width = frame.shape[0], frame.shape[1]
            self._prev_bbox = DenormalizedBbox(x=0, y=0, w=frame_width, h=frame_height,
                                               frame_w=frame_width, frame_h=frame_height)
        tracking_region = self._prev_bbox.get_expanded_by(self.TRACKING_REGION_EXPANDED_BY)
        candidates = BlackSpotFinder.find_tracking_candidates(tracking_region, frame,
                                                              self.THRESHOLD)
        if len(candidates) == 0:
            log.debug("BlackSpotFinder found no tracking candidate")
            return None
        new_bbox = min(candidates, key=lambda new: new.distance(self._prev_bbox))
        self._prev_bbox = new_bbox
        return new_bbox

    def reset(self):
        self._prev_bbox = None

    # def _start(self, frame: np.ndarray):
    #     x, y, h, w = cv2.selectROI("Select Object", frame, fromCenter=False, showCrosshair=True)
    #     frame_height, frame_width = frame.shape[0], frame.shape[1]
    #     self._prev_bbox = DenormalizedBbox(x=x, y=y, w=w, h=h, frame_w=frame_width,
    #                                        frame_h=frame_height)
    #     cv2.destroyWindow("Select Object")
    #     self._started = True

    def name(self):
        return "Black Spot Tracker"
