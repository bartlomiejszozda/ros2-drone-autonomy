# All vehicles the detector found in one camera image.
# header.stamp is copied from the image, so it is in simulation time.
std_msgs/Header header

# Size of the image the boxes belong to.
int32 image_width
int32 image_height

# Empty when no vehicle was found in this image.
CarDetection[] cars
