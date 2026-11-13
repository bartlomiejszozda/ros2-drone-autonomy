"""
Turn camera angles into a world direction and estimate how that direction moves.

Pure Python: no ROS, so every piece can be tested with numbers.

Frames follow ROS REP 103:
- body (FLU): x forward, y left, z up; the camera looks along body x.
- world (ENU): x east, y north, z up; ArduPilot's pose orientation rotates body to world.
"""

import math
from dataclasses import dataclass

import numpy as np


def camera_ray(azimuth_deg: float, elevation_deg: float) -> np.ndarray:
    """
    Direction to the target in the body frame (not normalised).

    A pinhole camera at distance 1 along the optical axis: tan(angle) is the offset.
    """
    return np.array([1.0,
                     -math.tan(math.radians(azimuth_deg)),  # right of centre = -y (y is left)
                     math.tan(math.radians(elevation_deg))])


def rotate(quaternion_xyzw, vector: np.ndarray) -> np.ndarray:
    """Rotate a vector by a unit quaternion (x, y, z, w)."""
    q = np.array(quaternion_xyzw[:3], dtype=float)
    w = float(quaternion_xyzw[3])
    t = 2.0 * np.cross(q, vector)
    return vector + w * t + np.cross(q, t)


def world_direction(azimuth_deg: float, elevation_deg: float,
                    body_to_world_xyzw) -> tuple[float, float]:
    """
    Return (bearing, elevation) of the target in degrees.

    bearing: compass direction, 0 = north, 90 = east, range [0, 360).
    elevation: angle above the horizon.
    Using the full attitude means the drone's own yaw, pitch and roll are removed:
    a still target keeps the same bearing while the drone turns.
    """
    east, north, up = rotate(body_to_world_xyzw, camera_ray(azimuth_deg, elevation_deg))
    bearing = math.degrees(math.atan2(east, north)) % 360.0
    elevation = math.degrees(math.atan2(up, math.hypot(east, north)))
    return bearing, elevation


def interpolate_quaternion(q0, q1, fraction: float):
    """Normalised linear interpolation; accurate for the small rotations between poses."""
    q0 = np.array(q0, dtype=float)
    q1 = np.array(q1, dtype=float)
    if np.dot(q0, q1) < 0:
        q1 = -q1  # q and -q are the same rotation; take the short way
    q = (1.0 - fraction) * q0 + fraction * q1
    return tuple(q / np.linalg.norm(q))


def wrap_180(angle_deg: float) -> float:
    """Map an angle difference to [-180, 180), so 359 -> 1 is +2, not -358."""
    return (angle_deg + 180.0) % 360.0 - 180.0


class AlphaBetaFilter:
    """
    Estimate a value and its rate from noisy samples, assuming a roughly constant rate.

    Each update first predicts (value + rate * dt), then corrects by the residual:
    alpha controls how much the value trusts the new sample, beta how fast the rate adapts.
    """

    def __init__(self, alpha: float, beta: float, wrap_degrees: bool = False):
        if not 0 < alpha <= 1 or not 0 <= beta <= 1:
            raise ValueError('alpha must be in (0, 1] and beta in [0, 1]')
        self.alpha = alpha
        self.beta = beta
        self.wrap_degrees = wrap_degrees
        self.value = 0.0
        self.rate = 0.0

    def reset(self, value: float):
        self.value = self._wrap(value)
        self.rate = 0.0

    def update(self, value: float, dt: float):
        predicted = self.value + self.rate * dt
        residual = value - predicted
        if self.wrap_degrees:
            residual = wrap_180(residual)
        self.value = self._wrap(predicted + self.alpha * residual)
        self.rate += self.beta * residual / dt

    def predict(self, dt: float) -> float:
        return self._wrap(self.value + self.rate * dt)

    def _wrap(self, value: float) -> float:
        return value % 360.0 if self.wrap_degrees else value


@dataclass
class TrackEstimate:
    stamp_sec: float
    measured: bool
    measurements: int
    age_sec: float
    bearing_deg: float
    elevation_deg: float
    bearing_rate_dps: float
    elevation_rate_dps: float

    def predict(self, time_sec: float) -> tuple[float, float]:
        """Where the target direction is expected at another time, e.g. now."""
        dt = time_sec - self.stamp_sec
        bearing = (self.bearing_deg + self.bearing_rate_dps * dt) % 360.0
        return bearing, self.elevation_deg + self.elevation_rate_dps * dt


class TargetMotionEstimator:
    """Keep one track of the target direction over time."""

    def __init__(self, alpha: float = 0.5, beta: float = 0.1, max_coast_sec: float = 0.5):
        self._bearing = AlphaBetaFilter(alpha, beta, wrap_degrees=True)
        self._elevation = AlphaBetaFilter(alpha, beta)
        self._max_coast = max_coast_sec
        self._last_update = None
        self._last_measurement = None
        self._measurements = 0

    def update(self, stamp_sec: float, bearing_deg: float, elevation_deg: float) -> TrackEstimate:
        """Add a measurement from an image in which the target was found."""
        if self._should_start_new_track(stamp_sec):
            self._bearing.reset(bearing_deg)
            self._elevation.reset(elevation_deg)
            self._measurements = 0
        elif stamp_sec > self._last_update:
            dt = stamp_sec - self._last_update
            self._bearing.update(bearing_deg, dt)
            self._elevation.update(elevation_deg, dt)
        else:
            # The same image time again: nothing new to learn.
            return self._estimate(stamp_sec, measured=True)
        self._measurements += 1
        self._last_update = stamp_sec
        self._last_measurement = stamp_sec
        return self._estimate(stamp_sec, measured=True)

    def coast(self, stamp_sec: float) -> TrackEstimate | None:
        """No target in this image: predict for a short time, then drop the track."""
        if self._last_measurement is None or stamp_sec < self._last_update:
            return None
        if stamp_sec - self._last_measurement > self._max_coast:
            self._last_measurement = None
            return None
        return self._estimate(stamp_sec, measured=False)

    def _should_start_new_track(self, stamp_sec: float) -> bool:
        return (self._last_measurement is None
                or stamp_sec < self._last_update  # time went backwards, e.g. simulator reset
                or stamp_sec - self._last_measurement > self._max_coast)

    def _estimate(self, stamp_sec: float, measured: bool) -> TrackEstimate:
        dt = stamp_sec - self._last_update
        return TrackEstimate(
            stamp_sec=stamp_sec,
            measured=measured,
            measurements=self._measurements,
            age_sec=stamp_sec - self._last_measurement,
            bearing_deg=self._bearing.predict(dt),
            elevation_deg=self._elevation.predict(dt),
            bearing_rate_dps=self._bearing.rate,
            elevation_rate_dps=self._elevation.rate)
