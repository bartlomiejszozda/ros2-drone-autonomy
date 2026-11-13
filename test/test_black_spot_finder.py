from pathlib import Path

import cv2

from collider.src.Helpers import DenormalizedBbox
from collider.src.tracker.BlackSpotFinder import BlackSpotFinder


def test_find_tracking_candidates_prints_results():
    image_path = Path(__file__).parent / "resources" / "drone_frame.png"
    frame = cv2.imread(str(image_path))

    frame_height, frame_width = frame.shape[:2]
    tracking_region = DenormalizedBbox(
        x=0,
        y=0,
        w=frame_width,
        h=frame_height,
        frame_w=frame_width,
        frame_h=frame_height,
    )

    candidates = BlackSpotFinder.find_tracking_candidates(
        tracking_region=tracking_region,
        frame=frame,
        threshold=1,
    )

    print(f"Loaded image: {image_path}")
    print(f"Candidate count: {len(candidates)}")
    for index, candidate in enumerate(candidates, start=1):
        print(
            f"Candidate {index}: x={candidate.x}, y={candidate.y}, "
            f"w={candidate.w}, h={candidate.h}"
        )

    assert len(candidates) == 1
    assert candidates[0].x == 142
    assert candidates[0].y == 45
    assert candidates[0].w == 14
    assert candidates[0].h == 13

    # Save annotated image with bounding boxes for visual verification
    output_path = Path(__file__).parent / "tmp" / "drone_frame_with_bboxes.png"
    output_path.parent.mkdir(exist_ok=True)
    annotated_frame = frame.copy()
    for candidate in candidates:
        top_left = (candidate.x, candidate.y)
        bottom_right = (candidate.x + candidate.w, candidate.y + candidate.h)
        cv2.rectangle(annotated_frame, top_left, bottom_right, (0, 255, 0), 2)

    cv2.imwrite(str(output_path), annotated_frame)
    print(f"Annotated image saved: {output_path}")
