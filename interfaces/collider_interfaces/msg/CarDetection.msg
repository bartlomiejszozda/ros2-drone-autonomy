# One vehicle found in one camera image (an element of CarDetections).

# Class name from the model metadata, e.g. car, van, truck, bus.
string class_name
# Model confidence, 0..1.
float32 confidence
# Stable ID across images from the tracker; -1 when the detector does not track.
int32 track_id

# Angles from the image centre to the bbox centre, in camera axes.
# Positive azimuth: right of centre. Positive elevation: above centre.
float64 azimuth_deg
float64 elevation_deg

# Bounding box in pixels (top-left corner, size).
int32 x
int32 y
int32 width
int32 height
