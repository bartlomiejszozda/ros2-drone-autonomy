"""
Decide how the drone should move to keep the target centred.

Pure Python: no ROS and no knowledge of how commands reach ArduPilot.
"""

import math
from dataclasses import dataclass

from collider.src.tracker.TargetMotionEstimator import TrackEstimate, world_direction, wrap_180


@dataclass
class DesiredMotion:
    """
    What the drone should do, in physical units and independent of any backend.

    forward/left are relative to the drone's heading. yaw_rate_dps is positive
    clockwise (turning right), matching compass bearings.
    """

    forward_mps: float = 18.0
    left_mps: float = 0.0
    up_mps: float = 0.0
    yaw_rate_dps: float = 0.0


def heading_deg(body_to_world_xyzw) -> float:
    """Compass heading of the drone's nose: 0 = north, 90 = east."""
    return world_direction(0.0, 0.0, body_to_world_xyzw)[0]


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass
class CenteringGains:
    yaw_gain: float = 1.0               # deg/s of yaw rate per degree of bearing error
    yaw_rate_feedforward: float = 1.0   # share of the target's bearing rate to follow
    max_yaw_rate_dps: float = 45.0
    climb_gain: float = 0.1             # m/s of climb per degree of elevation
    max_climb_mps: float = 1.0
    deadband_deg: float = 1.0           # ignore smaller errors, so the drone does not twitch
    max_prediction_sec: float = 0.3     # never extrapolate the track further than this


class CenteringController:
    """
    Turn toward the target's bearing and climb or descend toward its elevation.

    Yaw: proportional to the bearing error, plus feed-forward of the bearing rate so a
    target that keeps moving sideways does not leave a constant lag.
    Climb: proportional to elevation above the horizon, so the drone matches the
    target's height and the target sits on the camera's horizontal centre line.
    """

    def __init__(self, gains: CenteringGains = CenteringGains()):
        self.gains = gains

    def compute(self, track: TrackEstimate | None, attitude_xyzw, now_sec: float
                ) -> DesiredMotion:
        if track is None:
            return DesiredMotion()
        g = self.gains
        # The track describes the image time; predict it to now to make up for latency.
        dt = min(max(now_sec - track.stamp_sec, 0.0), g.max_prediction_sec)
        bearing, elevation = track.predict(track.stamp_sec + dt)

        bearing_error = self._deadband(wrap_180(bearing - heading_deg(attitude_xyzw)))
        yaw_rate = g.yaw_gain * bearing_error + g.yaw_rate_feedforward * track.bearing_rate_dps
        up = g.climb_gain * self._deadband(elevation)
        return DesiredMotion(yaw_rate_dps=clamp(yaw_rate, g.max_yaw_rate_dps),
                             up_mps=clamp(up, g.max_climb_mps))

    def _deadband(self, error: float) -> float:
        return 0.0 if abs(error) < self.gains.deadband_deg else error


def is_finite_motion(motion: DesiredMotion) -> bool:
    return all(math.isfinite(v) for v in (
        motion.forward_mps, motion.left_mps, motion.up_mps, motion.yaw_rate_dps))
