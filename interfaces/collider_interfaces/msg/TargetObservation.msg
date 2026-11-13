# One tracker result for one camera image.
# header.stamp is copied from the image, so it is in simulation time.
std_msgs/Header header

# False when the tracker lost the target in this image; other fields are then zero.
bool tracking

# Angles from the image centre to the bbox centre, in camera axes.
# Positive azimuth: target is right of centre. Positive elevation: target is above centre.
float64 azimuth_deg
float64 elevation_deg

# Bounding box in pixels (top-left corner, size) and the image size it belongs to.
int32 x
int32 y
int32 width
int32 height
int32 image_width
int32 image_height
