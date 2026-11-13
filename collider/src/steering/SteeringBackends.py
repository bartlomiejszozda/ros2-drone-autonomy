"""
Two ways to deliver a DesiredMotion to ArduPilot.

Both share one interface, so the steering node does not care which one it uses.
"""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from collider.src.steering.Centering import DesiredMotion

GUIDED = 4
LOITER = 5


class SteeringBackend(ABC):
    @abstractmethod
    def required_mode(self) -> int:
        """Copter flight mode in which ArduPilot obeys this backend."""

    @abstractmethod
    def send(self, motion: DesiredMotion, heading_deg: float):
        """Command the motion; called repeatedly while steering."""

    @abstractmethod
    def stop(self):
        """Stop commanding. The drone should end up hovering under ArduPilot's control."""

    def name(self) -> str:
        return type(self).__name__


def to_map_twist(motion: DesiredMotion, heading_deg: float) -> TwistStamped:
    """
    Express the motion in the world (ENU) frame ArduPilot calls 'map'.

    We rotate by heading only. The 'base_link' frame would rotate by the full
    attitude, so while the drone tilts, part of a horizontal command becomes vertical.
    """
    h = math.radians(heading_deg)
    # Compass heading h: forward points (east, north) = (sin h, cos h); left = (-cos h, sin h).
    msg = TwistStamped()
    msg.header.frame_id = 'map'
    msg.twist.linear.x = motion.forward_mps * math.sin(h) - motion.left_mps * math.cos(h)
    msg.twist.linear.y = motion.forward_mps * math.cos(h) + motion.left_mps * math.sin(h)
    msg.twist.linear.z = motion.up_mps
    # ROS yaw rate is positive counter-clockwise; DesiredMotion is positive clockwise.
    msg.twist.angular.z = -math.radians(motion.yaw_rate_dps)
    return msg


class CmdVelBackend(SteeringBackend):
    """Velocity commands over DDS in GUIDED mode."""

    def __init__(self, node: Node, topic: str):
        self._node = node
        self._publisher = node.create_publisher(TwistStamped, topic, 10)

    def required_mode(self) -> int:
        return GUIDED

    def send(self, motion: DesiredMotion, heading_deg: float):
        msg = to_map_twist(motion, heading_deg)
        msg.header.stamp = self._node.get_clock().now().to_msg()
        self._publisher.publish(msg)

    def stop(self):
        # Zero velocity now, instead of continuing for GUID_TIMEOUT.
        self.send(DesiredMotion(forward_mps=0.0), 0.0)


@dataclass
class StickScale:
    """
    How ArduPilot's LOITER mode turns stick positions into motion.

    The defaults are Copter's: PILOT_Y_RATE, PILOT_SPD_UP, LOIT_SPEED_MS and the stick
    dead zones (RC4_DZ, THR_DZ, RC1_DZ/RC2_DZ). Check them on your vehicle.
    """

    max_yaw_rate_dps: float = 202.5
    max_climb_mps: float = 2.5
    max_horizontal_mps: float = 12.5
    yaw_dead_zone: int = 20
    throttle_dead_zone: int = 100
    roll_pitch_dead_zone: int = 20


NEUTRAL = 1500
IGNORE = 65535  # in RC_CHANNELS_OVERRIDE, UINT16_MAX leaves that channel untouched


def stick(value: float, full_scale: float, dead_zone: int) -> int:
    """
    PWM that asks for `value`, where `full_scale` is reached at full stick, 500 from centre.

    ArduPilot ignores the first `dead_zone` microseconds, so a small request starts there.
    """
    if value == 0 or full_scale <= 0:
        return NEUTRAL
    fraction = min(abs(value) / full_scale, 1.0)
    offset = dead_zone + fraction * (500 - dead_zone)
    return int(round(NEUTRAL + math.copysign(offset, value)))


def rc_channels(motion: DesiredMotion, scale: StickScale) -> list[int]:
    """Channels 1-8: roll, pitch, throttle, yaw, then four channels we leave untouched."""
    return [
        # Roll stick high = right, so left is negative. Pitch stick low = forward.
        stick(-motion.left_mps, scale.max_horizontal_mps, scale.roll_pitch_dead_zone),
        stick(-motion.forward_mps, scale.max_horizontal_mps, scale.roll_pitch_dead_zone),
        stick(motion.up_mps, scale.max_climb_mps, scale.throttle_dead_zone),
        stick(motion.yaw_rate_dps, scale.max_yaw_rate_dps, scale.yaw_dead_zone),
        IGNORE, IGNORE, IGNORE, IGNORE,
    ]


class RcBackend(SteeringBackend):
    """
    Stick commands over MAVLink RC override in LOITER mode.

    Overrides are sent only from send(), which the steering node's timer calls often enough.
    Nothing is sent before steering starts, so no low throttle is sent.
    """

    def __init__(self, mavlink_connection, scale: StickScale = StickScale()):
        self._connection = mavlink_connection
        self._scale = scale

    def required_mode(self) -> int:
        return LOITER

    def send(self, motion: DesiredMotion, heading_deg: float):
        self._send(rc_channels(motion, self._scale))

    def stop(self):
        # Centred sticks mean "hover" in LOITER. We do not release the channels: in SITL
        # the simulated radio's throttle may sit low, and LOITER would then descend.
        # The override expires after RC_OVERRIDE_TIME; leave LOITER before that.
        # DesiredMotion() cruises forward by default, so ask for zero explicitly.
        self.send(DesiredMotion(forward_mps=0.0), 0.0)

    def _send(self, channels: list[int]):
        self._connection.mav.rc_channels_override_send(
            self._connection.target_system, self._connection.target_component, *channels)
