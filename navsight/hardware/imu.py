"""BNO085 9-DoF IMU: cane heading, pitch and roll, step count and stillness."""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable, Iterable, Sequence
from typing import Any, Protocol

import numpy as np

from navsight.config import AXIS_CHOICES, NavSightConfig
from navsight.hardware.i2c import I2C_LOCK
from navsight.types import STANDARD_GRAVITY, ImuSample
from navsight.util import Clock, clamp, wrap_heading_deg

log = logging.getLogger(__name__)

# SH-2 report IDs (the same values as adafruit_bno08x.BNO_REPORT_*)
REPORT_ACCELEROMETER = 0x01
REPORT_GYROSCOPE = 0x02
REPORT_LINEAR_ACCELERATION = 0x04
REPORT_ROTATION_VECTOR = 0x05
REPORT_STEP_COUNTER = 0x11
REPORT_STABILITY_CLASSIFIER = 0x13

# The rotation vector is enabled last: read() rejects the all-zero quaternion the driver holds
# until the first real report, and by the time one arrives the other reports have arrived too.
ENABLED_REPORTS = (
    REPORT_ACCELEROMETER,
    REPORT_GYROSCOPE,
    REPORT_LINEAR_ACCELERATION,
    REPORT_STEP_COUNTER,
    REPORT_STABILITY_CLASSIFIER,
    REPORT_ROTATION_VECTOR,
)
STATIONARY_CLASSIFICATIONS = frozenset({"On Table", "Stationary"})

_AXIS_VECTORS = {
    "x": (1.0, 0.0, 0.0),
    "-x": (-1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "-y": (0.0, -1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
    "-z": (0.0, 0.0, -1.0),
}
# Roll is measured on a sensor axis perpendicular to the forward axis.
_SIDE_AXES = {"x": "y", "-x": "y", "y": "x", "-y": "x", "z": "x", "-z": "x"}
_MIN_QUATERNION_NORM = 1e-6


class ImuDriver(Protocol):
    """The subset of ``adafruit_bno08x.BNO08X`` used here."""

    quaternion: tuple[float, float, float, float]  # (i, j, k, real); world frame East-North-Up
    acceleration: tuple[float, float, float]  # m/s^2 including gravity
    linear_acceleration: tuple[float, float, float]  # m/s^2
    gyro: tuple[float, float, float]  # rad/s
    steps: int
    stability_classification: str  # "Unknown", "On Table", "Stationary", "Stable", "In motion"

    def enable_feature(self, feature_id: int) -> None: ...


def orientation_from_quaternion(
    quat_ijkr: Sequence[float],
    forward_axis: str = "y",
    heading_offset_deg: float = 0.0,
) -> tuple[float, float, float]:
    """Return the cane's ``(heading_deg, pitch_deg, roll_deg)`` from a BNO085 rotation vector.

    ``quat_ijkr`` rotates sensor coordinates into the East-North-Up world frame and
    ``forward_axis`` is the sensor axis that points along the cane towards its tip. Raises
    ``ValueError`` for an unknown axis or a quaternion that cannot be normalised.
    """
    forward, side = _axis_vectors(forward_axis)
    rotation = _rotation_matrix(quat_ijkr)
    east, north, up = rotation @ forward
    side_up = (rotation @ side)[2]
    heading = wrap_heading_deg(math.degrees(math.atan2(east, north)) + heading_offset_deg)
    pitch = math.degrees(math.asin(clamp(up, -1.0, 1.0)))
    roll = math.degrees(math.asin(clamp(side_up, -1.0, 1.0)))
    return heading, pitch, roll


def _axis_vectors(forward_axis: str) -> tuple[np.ndarray, np.ndarray]:
    """Return the forward axis and the side axis used for roll as sensor-frame unit vectors."""
    if forward_axis not in _AXIS_VECTORS:
        raise ValueError(f"forward_axis must be one of {AXIS_CHOICES}, got {forward_axis!r}")
    side_axis = _SIDE_AXES[forward_axis]
    return np.array(_AXIS_VECTORS[forward_axis]), np.array(_AXIS_VECTORS[side_axis])


def _rotation_matrix(quat_ijkr: Sequence[float]) -> np.ndarray:
    """Body-to-world rotation matrix of the quaternion (x, y, z, w) = (i, j, k, real)."""
    x, y, z, w = (float(value) for value in quat_ijkr)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < _MIN_QUATERNION_NORM:
        raise ValueError(
            f"Quaternion {tuple(quat_ijkr)} cannot be normalised (adafruit_bno08x reports all "
            "zeros until the first rotation vector arrives)"
        )
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _vector3(values: Iterable[float]) -> tuple[float, float, float]:
    x, y, z = (float(value) for value in values)
    return x, y, z


class _StepCounter:
    """Turns the BNO085 step counter, which can reset or wrap, into a count that never drops."""

    def __init__(self) -> None:
        self._offset = 0
        self._last_raw = 0

    def update(self, raw: object) -> int:
        # adafruit_bno08x holds a placeholder tuple until the first step-counter report.
        if isinstance(raw, int) and raw >= 0:
            if raw < self._last_raw:
                self._offset += self._last_raw
            self._last_raw = raw
        return self._offset + self._last_raw


def make_bno085_factory(
    i2c: Any, address: int, *, report_rate_hz: float | None = None
) -> Callable[[], ImuDriver]:
    """Return a factory that opens an ``adafruit_bno08x`` BNO085 at ``address`` on ``i2c``.

    With ``report_rate_hz=None`` the driver's default report interval applies (50 ms, so
    20 Hz). Asking for a rate needs adafruit-circuitpython-bno08x 1.3 or newer.
    """
    if report_rate_hz is not None and report_rate_hz <= 0:
        raise ValueError(f"report_rate_hz must be positive, got {report_rate_hz}")

    def factory() -> ImuDriver:
        from adafruit_bno08x.i2c import BNO08X_I2C  # only available on the Pi

        driver = BNO08X_I2C(i2c, address=address)
        if report_rate_hz is None:
            return driver
        return _ReportRateDriver(driver, round(1_000_000 / report_rate_hz))

    return factory


class _ReportRateDriver:
    """An adafruit_bno08x driver whose reports are all enabled at one interval."""

    def __init__(self, driver: Any, report_interval_us: int) -> None:
        self._driver = driver
        self._report_interval_us = report_interval_us

    def enable_feature(self, feature_id: int) -> None:
        self._driver.enable_feature(feature_id, report_interval=self._report_interval_us)

    @property
    def quaternion(self) -> tuple[float, float, float, float]:
        return self._driver.quaternion

    @property
    def acceleration(self) -> tuple[float, float, float]:
        return self._driver.acceleration

    @property
    def linear_acceleration(self) -> tuple[float, float, float]:
        return self._driver.linear_acceleration

    @property
    def gyro(self) -> tuple[float, float, float]:
        return self._driver.gyro

    @property
    def steps(self) -> int:
        return self._driver.steps

    @property
    def stability_classification(self) -> str:
        return self._driver.stability_classification


class Imu:
    """The BNO085 on the shared I2C bus."""

    def __init__(
        self,
        config: NavSightConfig,
        driver_factory: Callable[[], ImuDriver],
        *,
        clock: Clock = time.monotonic,
    ) -> None:
        self._config = config
        self._driver_factory = driver_factory
        self._clock = clock
        self._driver: ImuDriver | None = None
        self._step_counter = _StepCounter()

    def start(self) -> None:
        """Create the driver and enable the six reports. Does nothing if already started."""
        if self._driver is not None:
            return
        address = self._config.i2c.imu
        try:
            with I2C_LOCK:
                driver = self._driver_factory()
                for report_id in ENABLED_REPORTS:
                    driver.enable_feature(report_id)
        except Exception as exc:
            raise RuntimeError(
                f"Could not start the BNO085 IMU at {address:#04x}: {exc}. Check its power and "
                "SDA (GPIO 2) / SCL (GPIO 3) wiring, and that I2C runs at 400 kHz "
                "(dtparam=i2c_arm_baudrate=400000)."
            ) from exc
        self._driver = driver
        log.info("BNO085 IMU started at %#04x", address)

    def read(self) -> ImuSample:
        """Read the latest reports.

        Raises ``ValueError`` until the BNO085 has sent its first rotation vector.
        """
        driver = self._driver
        if driver is None:
            raise RuntimeError("Imu.start() must be called before read()")
        with I2C_LOCK:
            quaternion = driver.quaternion
            acceleration = driver.acceleration
            linear_acceleration = driver.linear_acceleration
            gyro = driver.gyro
            steps = driver.steps
            stability = driver.stability_classification
        imu = self._config.imu
        heading, pitch, roll = orientation_from_quaternion(
            quaternion, imu.forward_axis, imu.heading_offset_deg
        )
        return ImuSample(
            timestamp=self._clock(),
            heading_deg=heading,
            pitch_deg=pitch,
            roll_deg=roll,
            accel_mps2=_vector3(acceleration),
            linear_accel_mps2=_vector3(linear_acceleration),
            gyro_dps=_vector3(math.degrees(rate) for rate in gyro),
            step_count=self._step_counter.update(steps),
            stationary=stability in STATIONARY_CLASSIFICATIONS,
        )

    def stop(self) -> None:
        """Release the driver. After a restart the step count carries on where it was."""
        if self._driver is not None:
            self._driver = None
            log.info("BNO085 IMU stopped")


class SimulatedImu:
    """Simulated twin: a stationary cane pointing north, tilted 45 degrees down, until changed."""

    def __init__(self, config: NavSightConfig, *, clock: Clock = time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._heading_deg = 0.0
        self._pitch_deg = -45.0
        self._roll_deg = 0.0
        self._accel_mps2 = (0.0, 0.0, STANDARD_GRAVITY)
        self._linear_accel_mps2 = (0.0, 0.0, 0.0)
        self._gyro_dps = (0.0, 0.0, 0.0)
        self._step_count = 0
        self._stationary = True

    def set_state(
        self,
        *,
        heading_deg: float | None = None,
        pitch_deg: float | None = None,
        roll_deg: float | None = None,
        accel_mps2: Sequence[float] | None = None,
        linear_accel_mps2: Sequence[float] | None = None,
        gyro_dps: Sequence[float] | None = None,
        add_steps: int = 0,
        stationary: bool | None = None,
    ) -> None:
        """Change the given values; ``None`` keeps the current one."""
        if add_steps < 0:
            raise ValueError(f"add_steps cannot be negative (got {add_steps}): steps never drop")
        with self._lock:
            if heading_deg is not None:
                self._heading_deg = wrap_heading_deg(heading_deg)
            if pitch_deg is not None:
                self._pitch_deg = float(pitch_deg)
            if roll_deg is not None:
                self._roll_deg = float(roll_deg)
            if accel_mps2 is not None:
                self._accel_mps2 = _vector3(accel_mps2)
            if linear_accel_mps2 is not None:
                self._linear_accel_mps2 = _vector3(linear_accel_mps2)
            if gyro_dps is not None:
                self._gyro_dps = _vector3(gyro_dps)
            if stationary is not None:
                self._stationary = stationary
            self._step_count += add_steps

    def start(self) -> None:
        log.info("Simulated IMU started")

    def read(self) -> ImuSample:
        with self._lock:
            return ImuSample(
                timestamp=self._clock(),
                heading_deg=self._heading_deg,
                pitch_deg=self._pitch_deg,
                roll_deg=self._roll_deg,
                accel_mps2=self._accel_mps2,
                linear_accel_mps2=self._linear_accel_mps2,
                gyro_dps=self._gyro_dps,
                step_count=self._step_count,
                stationary=self._stationary,
            )

    def stop(self) -> None:
        log.info("Simulated IMU stopped")
