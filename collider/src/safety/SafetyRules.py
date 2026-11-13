"""
Decide whether the vehicle is outside its safe envelope.

Pure Python: the node feeds in the latest telemetry and acts on the answer.
"""

import math
from dataclasses import dataclass

BRAKE = 17
LAND = 9
RTL = 6
FINAL_MODES = {'brake': BRAKE, 'land': LAND, 'rtl': RTL}
MODE_NAMES = {BRAKE: 'BRAKE', LAND: 'LAND', RTL: 'RTL', 4: 'GUIDED', 5: 'LOITER'}


@dataclass
class SafetyLimits:
    telemetry_timeout_sec: float = 1.0


@dataclass
class Telemetry:
    """The latest observations and their ages in seconds (math.inf if never received)."""

    armed: bool = False
    flying: bool = False
    mode: int = -1
    failsafes: tuple = ()
    status_age: float = math.inf
    pose_age: float = math.inf


@dataclass
class Violation:
    reason: str
    # True when ArduPilot already runs its own failsafe: we must not override its mode.
    ardupilot_handles_it: bool = False


def check(telemetry: Telemetry, limits: SafetyLimits, was_armed: bool) -> Violation | None:
    """
    Return the first rule broken, or None.

    was_armed: the last fresh status said armed. Losing telemetry matters only then;
    a vehicle that was on the ground disarmed is not in danger.
    """
    if telemetry.status_age > limits.telemetry_timeout_sec:
        return Violation('status telemetry lost') if was_armed else None
    if not telemetry.armed:
        return None
    if telemetry.failsafes:
        return Violation(f'ArduPilot failsafe active {list(telemetry.failsafes)}',
                         ardupilot_handles_it=True)
    if not telemetry.flying:
        return None
    if telemetry.pose_age > limits.telemetry_timeout_sec:
        return Violation('pose telemetry lost')
    return None
