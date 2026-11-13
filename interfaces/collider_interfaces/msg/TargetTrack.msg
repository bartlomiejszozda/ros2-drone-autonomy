# Filtered direction to the target in the world, and how fast that direction changes.
# header.stamp is the image time this estimate refers to (simulation time).
std_msgs/Header header

# False when there is no track: the target was never seen, or was lost for too long.
bool valid
# True if this image contained the target; false while predicting through a short loss.
bool measured
# Number of measurements in the current track; rates need a few before they settle.
uint32 measurements
# Seconds since the last image that contained the target.
float64 age_sec

# Compass direction from the drone to the target: 0 = north, 90 = east (clockwise).
float64 bearing_deg
# Angle above the horizon; negative means below.
float64 elevation_deg
# Rates of change in degrees per second; positive bearing rate = target moves clockwise.
float64 bearing_rate_dps
float64 elevation_rate_dps
