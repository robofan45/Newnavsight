"""Data types passed between NavSight modules.

Conventions used everywhere:
- Times are ``time.monotonic()`` seconds unless a field name says otherwise.
- Bearings are degrees relative to the cane/camera axis: negative = left, positive = right.
- Headings are compass degrees: 0 = north, clockwise, in [0, 360).
- Pitch is the angle of the cane axis above horizontal; a cane pointing at the ground is negative.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, IntEnum

STANDARD_GRAVITY = 9.80665  # m/s^2


class AlertLevel(IntEnum):
    INFO = 0
    CAUTION = 1
    WARNING = 2
    DANGER = 3


class PowerState(Enum):
    ACTIVE = "active"
    LOW_POWER = "low_power"
    SLEEP = "sleep"


class ButtonId(Enum):
    MODE = "mode"
    ACTION = "action"
    SOS = "sos"


class PressType(Enum):
    SHORT = "short"
    LONG = "long"


class HazardType(Enum):
    NONE = "none"
    DROP_OFF = "drop_off"  # stairs down, curb down, pothole
    STEP_UP = "step_up"  # curb up, step, low obstacle at ground level


class Verbosity(Enum):
    FULL = "full"
    BRIEF = "brief"
    HAPTIC_ONLY = "haptic_only"


@dataclass(frozen=True)
class ToFReading:
    sensor: str  # "forward" or "down"
    distance_m: float | None  # None = no target in range, or an invalid measurement
    timestamp: float


@dataclass(frozen=True)
class ImuSample:
    timestamp: float
    heading_deg: float
    pitch_deg: float
    roll_deg: float
    accel_mps2: tuple[float, float, float]  # includes gravity, sensor frame
    linear_accel_mps2: tuple[float, float, float]  # gravity removed, sensor frame
    gyro_dps: tuple[float, float, float]
    step_count: int  # cumulative since the IMU started; never decreases
    stationary: bool

    @property
    def accel_magnitude_g(self) -> float:
        return math.sqrt(sum(a * a for a in self.accel_mps2)) / STANDARD_GRAVITY


@dataclass(frozen=True)
class GpsFix:
    timestamp: float
    latitude: float | None
    longitude: float | None
    altitude_m: float | None = None
    speed_mps: float | None = None
    course_deg: float | None = None
    hdop: float | None = None
    satellites_used: int = 0
    fix_quality: int = 0  # NMEA GGA quality: 0 = invalid, 1 = GPS, 2 = DGPS, ...

    @property
    def has_fix(self) -> bool:
        return self.fix_quality > 0 and self.latitude is not None and self.longitude is not None


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]  # pixels in the analysed frame
    bearing_deg: float
    distance_m: float | None = None  # metric estimate from depth fusion, if available


@dataclass(frozen=True)
class Obstacle:
    sector: int
    bearing_deg: float
    distance_m: float | None  # None for a vision label that has no range estimate
    closing_speed_mps: float  # positive = getting closer
    time_to_collision_s: float | None
    label: str | None
    source: str  # "tof", "vision" or "fused"
    confidence: float


@dataclass(frozen=True)
class GroundHazard:
    hazard: HazardType
    delta_m: float  # vertical-equivalent change versus the learned ground baseline; + = drop
    timestamp: float


@dataclass(frozen=True)
class Pose:
    timestamp: float
    east_m: float
    north_m: float
    heading_deg: float
    latitude: float | None
    longitude: float | None
    position_std_m: float
    source: str  # "gps" or "dead_reckoning"


@dataclass(frozen=True)
class BatteryStatus:
    timestamp: float
    bus_voltage_v: float
    current_ma: float
    power_w: float
    state_of_charge: float  # 0..1
    remaining_minutes: float | None


@dataclass(frozen=True)
class ButtonEvent:
    button: ButtonId
    press: PressType
    timestamp: float


@dataclass(frozen=True)
class Alert:
    """A system event to announce, such as low battery, GPS lost or an SOS countdown."""

    key: str  # cooldown and de-duplication key
    level: AlertLevel
    message: str  # spoken text; may be empty for haptic-only events
    haptic_pattern: str | None = None
    bearing_deg: float = 0.0
    timestamp: float = 0.0


@dataclass(frozen=True)
class Beep:
    frequency_hz: float
    duration_s: float
    bearing_deg: float


@dataclass(frozen=True)
class Speech:
    text: str
    interrupt: bool = False  # stop the current speech and clear the queue first


@dataclass(frozen=True)
class AlertOutputs:
    haptic_left: float  # continuous level 0..1
    haptic_right: float
    haptic_pattern: str | None
    beeps: tuple[Beep, ...]
    speech: tuple[Speech, ...]


@dataclass(frozen=True)
class PowerProfile:
    state: PowerState
    camera_fps: float  # 0 = camera off
    depth_enabled: bool
    tof_rate_hz: float  # 0 = ToF off
    gps_poll_interval_s: float | None  # None = GPS off, 0 = continuous, > 0 = duty cycled
    fusion_rate_hz: float
