"""Pose estimation: an EKF over (east, north, heading) from IMU step dead reckoning and GPS."""

from __future__ import annotations

import logging
import math

import numpy as np

from navsight.config import NavSightConfig, PoseConfig
from navsight.types import GpsFix, ImuSample, Pose
from navsight.util import wrap_angle_deg, wrap_heading_deg

log = logging.getLogger(__name__)

EARTH_RADIUS_M = 6378137.0  # WGS84 equatorial radius
GPS_REJECTIONS_BEFORE_RESET = 3

_GPS_OBSERVATION = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
_HEADING_OBSERVATION = np.array([[0.0, 0.0, 1.0]])


class LocalTangentPlane:
    """Equirectangular projection about an origin, in metres east and north of it.

    Distances match the great circle to a few centimetres at 1 km; the error grows with the
    square of the distance. Longitudes wrap at +/-180 degrees.
    """

    def __init__(self, origin_latitude: float, origin_longitude: float) -> None:
        if not -90.0 < origin_latitude < 90.0:
            raise ValueError(
                f"Tangent plane origin latitude must be strictly between -90 and 90, "
                f"got {origin_latitude!r}"
            )
        if not math.isfinite(origin_longitude):
            raise ValueError(
                f"Tangent plane origin longitude must be finite, got {origin_longitude!r}"
            )
        self._origin_latitude = origin_latitude
        self._origin_longitude = origin_longitude
        self._parallel_radius_m = EARTH_RADIUS_M * math.cos(math.radians(origin_latitude))

    def to_enu(self, latitude: float, longitude: float) -> tuple[float, float]:
        delta_longitude = wrap_angle_deg(longitude - self._origin_longitude)
        east_m = self._parallel_radius_m * math.radians(delta_longitude)
        north_m = EARTH_RADIUS_M * math.radians(latitude - self._origin_latitude)
        return east_m, north_m

    def to_geodetic(self, east_m: float, north_m: float) -> tuple[float, float]:
        latitude = self._origin_latitude + math.degrees(north_m / EARTH_RADIUS_M)
        longitude = self._origin_longitude + math.degrees(east_m / self._parallel_radius_m)
        return latitude, wrap_angle_deg(longitude)


class PoseEKF:
    """Extended Kalman filter over x = [east_m, north_m, psi].

    psi is the compass heading in radians, clockwise from north, so a step of length L moves the
    position by (L sin psi, L cos psi).
    """

    def __init__(self, config: PoseConfig) -> None:
        _validate_pose_config(config)
        self._config = config
        self._state = np.zeros(3)
        self._covariance = np.zeros((3, 3))
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def state(self) -> np.ndarray:
        return self._state.copy()

    @property
    def covariance(self) -> np.ndarray:
        return self._covariance.copy()

    @property
    def heading_deg(self) -> float:
        return wrap_heading_deg(math.degrees(self._state[2]))

    @property
    def position_std_m(self) -> float:
        """One-sigma position error along the least certain direction."""
        largest_variance = float(np.linalg.eigvalsh(self._covariance[:2, :2])[-1])
        return math.sqrt(max(largest_variance, 0.0))

    def initialize(
        self, east_m: float, north_m: float, heading_deg: float, position_std_m: float
    ) -> None:
        """Start or restart the filter. The heading starts with the IMU's heading std."""
        _require_finite(east_m=east_m, north_m=north_m, heading_deg=heading_deg)
        _require_positive(position_std_m=position_std_m)
        heading_variance = math.radians(self._config.imu_heading_std_deg) ** 2
        self._state = np.array([east_m, north_m, _heading_rad(heading_deg)])
        self._covariance = np.diag([position_std_m**2, position_std_m**2, heading_variance])
        self._initialized = True

    def predict_step(self, step_length_m: float, heading_deg: float) -> None:
        """Move one step along the IMU heading, which first replaces psi."""
        self._require_initialized()
        _require_finite(heading_deg=heading_deg)
        if not (math.isfinite(step_length_m) and step_length_m >= 0.0):
            raise ValueError(f"step_length_m must be a finite number >= 0, got {step_length_m!r}")
        psi = _heading_rad(heading_deg)
        sin_psi, cos_psi = math.sin(psi), math.cos(psi)
        transition = np.array(
            [
                [1.0, 0.0, step_length_m * cos_psi],
                [0.0, 1.0, -step_length_m * sin_psi],
                [0.0, 0.0, 1.0],
            ]
        )
        noise_jacobian = np.array(
            [
                [sin_psi, step_length_m * cos_psi],
                [cos_psi, -step_length_m * sin_psi],
                [0.0, 1.0],
            ]
        )
        step_noise = np.diag(
            [
                (self._config.step_length_std_frac * step_length_m) ** 2,
                math.radians(self._config.heading_std_per_step_deg) ** 2,
            ]
        )
        east_m = self._state[0] + step_length_m * sin_psi
        north_m = self._state[1] + step_length_m * cos_psi
        self._state = np.array([east_m, north_m, psi])
        self._covariance = _symmetric(
            transition @ self._covariance @ transition.T
            + noise_jacobian @ step_noise @ noise_jacobian.T
        )

    def update_heading(self, heading_deg: float, std_deg: float) -> None:
        self._require_initialized()
        _require_finite(heading_deg=heading_deg)
        _require_positive(std_deg=std_deg)
        innovation_deg = wrap_angle_deg(heading_deg - self.heading_deg)
        self._correct(
            _HEADING_OBSERVATION,
            np.array([math.radians(innovation_deg)]),
            np.array([[math.radians(std_deg) ** 2]]),
        )

    def update_gps(self, east_m: float, north_m: float, std_m: float) -> bool:
        """Correct the position with a fix; returns False if the Mahalanobis gate rejects it."""
        self._require_initialized()
        _require_finite(east_m=east_m, north_m=north_m)
        _require_positive(std_m=std_m)
        innovation = np.array([east_m, north_m]) - self._state[:2]
        noise = np.eye(2) * std_m**2
        innovation_covariance = _GPS_OBSERVATION @ self._covariance @ _GPS_OBSERVATION.T + noise
        distance_sq = float(innovation @ np.linalg.solve(innovation_covariance, innovation))
        if distance_sq > self._config.gps_gate_chi2:
            log.debug(
                "GPS fix gated out: d^2 = %.1f > %.2f", distance_sq, self._config.gps_gate_chi2
            )
            return False
        self._correct(_GPS_OBSERVATION, innovation, noise)
        return True

    def _correct(self, observation: np.ndarray, innovation: np.ndarray, noise: np.ndarray) -> None:
        """Kalman update; the Joseph form keeps the covariance symmetric positive definite."""
        covariance = self._covariance
        innovation_covariance = observation @ covariance @ observation.T + noise
        gain = np.linalg.solve(innovation_covariance, observation @ covariance).T
        state = self._state + gain @ innovation
        state[2] = _heading_rad(math.degrees(state[2]))
        self._state = state
        i_minus_kh = np.eye(3) - gain @ observation
        self._covariance = _symmetric(
            i_minus_kh @ covariance @ i_minus_kh.T + gain @ noise @ gain.T
        )

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise RuntimeError("PoseEKF.initialize() must be called before predicting or updating")


class PoseEstimator:
    """Turns IMU samples and GPS fixes into a Pose.

    Dead reckoning starts at (0, 0) with the first IMU sample, so the pose works indoors from
    power-on. The first usable GPS fix becomes the tangent-plane origin: the position is
    re-initialised there and latitude/longitude are reported from then on.
    """

    def __init__(self, config: NavSightConfig) -> None:
        self._pose_config = config.pose
        self._gps_config = config.gps
        self._ekf = PoseEKF(config.pose)
        self._plane: LocalTangentPlane | None = None
        self._last_step_count = 0
        self._imu_heading_deg = 0.0
        self._last_fix_time: float | None = None
        self._rejections_in_a_row = 0
        self._invalid_heading_logged = False
        self._missing_hdop_logged = False

    def on_imu(self, sample: ImuSample) -> None:
        """Predict the sample's new steps, then update the heading.

        Samples without new steps leave the filter alone. Updating on every 30-100 Hz sample
        would treat correlated headings as independent and collapse the heading variance, and
        a turn on the spot would be misread, through the step cross-covariance, as an error in
        the heading of past steps: about 1.5 m of false movement per 90 degree turn.
        """
        if not math.isfinite(sample.heading_deg):
            if not self._invalid_heading_logged:
                log.warning("Ignoring IMU samples with an invalid heading (%r)", sample.heading_deg)
                self._invalid_heading_logged = True
            return
        self._invalid_heading_logged = False
        self._imu_heading_deg = wrap_heading_deg(sample.heading_deg)
        if not self._ekf.initialized:
            self._ekf.initialize(
                0.0, 0.0, sample.heading_deg, self._pose_config.initial_position_std_m
            )
            self._last_step_count = sample.step_count
            return
        new_steps = self._new_steps(sample.step_count)
        if new_steps == 0:
            return
        for _ in range(new_steps):
            self._ekf.predict_step(self._pose_config.step_length_m, sample.heading_deg)
        self._ekf.update_heading(sample.heading_deg, self._pose_config.imu_heading_std_deg)

    def on_gps(self, fix: GpsFix) -> bool:
        """Use a fix; returns True if it set the position (update, first fix or reset)."""
        if not self._ekf.initialized:
            log.debug("Ignoring a GPS fix that arrived before the first IMU sample")
            return False
        measurement = self._usable_measurement(fix)
        if measurement is None:
            return False
        latitude, longitude, std_m = measurement
        if self._plane is None:
            self._set_origin(latitude, longitude, std_m)
        elif not self._update_or_reset(self._plane, latitude, longitude, std_m):
            return False
        self._last_fix_time = fix.timestamp
        self._rejections_in_a_row = 0
        return True

    def pose(self, now: float) -> Pose | None:
        """The current pose, or None before the first IMU sample.

        heading_deg is the latest IMU heading: the filter's heading only changes at steps, so it
        would lag behind a user turning on the spot.
        """
        if not self._ekf.initialized:
            return None
        state = self._ekf.state
        east_m, north_m = float(state[0]), float(state[1])
        latitude: float | None = None
        longitude: float | None = None
        if self._plane is not None:
            latitude, longitude = self._plane.to_geodetic(east_m, north_m)
        return Pose(
            timestamp=now,
            east_m=east_m,
            north_m=north_m,
            heading_deg=self._imu_heading_deg,
            latitude=latitude,
            longitude=longitude,
            position_std_m=self._ekf.position_std_m,
            source="dead_reckoning" if self.using_dead_reckoning(now) else "gps",
        )

    def using_dead_reckoning(self, now: float) -> bool:
        if self._last_fix_time is None:
            return True
        return now - self._last_fix_time > self._pose_config.gps_timeout_s

    def _new_steps(self, step_count: int) -> int:
        previous = self._last_step_count
        self._last_step_count = step_count
        new_steps = step_count - previous
        if new_steps < 0:
            log.info("Step count fell from %d to %d; restarting the baseline", previous, step_count)
            return 0
        limit = self._pose_config.max_steps_per_update
        if new_steps > limit:
            log.warning("Step count jumped by %d; dead reckoning only %d", new_steps, limit)
            return limit
        return new_steps

    def _usable_measurement(self, fix: GpsFix) -> tuple[float, float, float] | None:
        """(latitude, longitude, sigma_m) if the fix is good enough to use, else None."""
        latitude, longitude, hdop = fix.latitude, fix.longitude, fix.hdop
        if not fix.has_fix or latitude is None or longitude is None:
            return None
        if hdop is None:
            if not self._missing_hdop_logged:
                log.warning("GPS fixes without HDOP are not used: enable GGA or GSA output")
                self._missing_hdop_logged = True
            return None
        if not 0.0 <= hdop <= self._gps_config.max_hdop:
            log.debug("Ignoring a GPS fix with HDOP %.1f", hdop)
            return None
        if not (-90.0 < latitude < 90.0 and -180.0 <= longitude <= 180.0):
            log.warning("Ignoring a GPS fix with impossible coordinates")
            return None
        std_m = max(hdop * self._gps_config.uere_m, self._pose_config.gps_min_std_m)
        return latitude, longitude, std_m

    def _set_origin(self, latitude: float, longitude: float, std_m: float) -> None:
        self._plane = LocalTangentPlane(latitude, longitude)
        self._ekf.initialize(0.0, 0.0, self._ekf.heading_deg, std_m)
        log.info("First usable GPS fix: position re-anchored to it (sigma %.1f m)", std_m)

    def _update_or_reset(
        self, plane: LocalTangentPlane, latitude: float, longitude: float, std_m: float
    ) -> bool:
        """Apply a fix; returns False while the gate keeps rejecting it.

        After GPS_REJECTIONS_BEFORE_RESET rejections in a row the position resets to the fix, so
        a long dead-reckoning drift can never lock GPS out.
        """
        east_m, north_m = plane.to_enu(latitude, longitude)
        if self._ekf.update_gps(east_m, north_m, std_m):
            return True
        self._rejections_in_a_row += 1
        if self._rejections_in_a_row < GPS_REJECTIONS_BEFORE_RESET:
            log.debug("GPS fix rejected by the gate (%d in a row)", self._rejections_in_a_row)
            return False
        log.info(
            "GPS disagreed with dead reckoning %d times in a row; resetting the position to it",
            self._rejections_in_a_row,
        )
        self._ekf.initialize(east_m, north_m, self._ekf.heading_deg, std_m)
        return True


def _validate_pose_config(config: PoseConfig) -> None:
    rules = {
        "step_length_m > 0": config.step_length_m > 0,
        "step_length_std_frac >= 0": config.step_length_std_frac >= 0,
        "heading_std_per_step_deg >= 0": config.heading_std_per_step_deg >= 0,
        "imu_heading_std_deg > 0": config.imu_heading_std_deg > 0,
        "gps_min_std_m > 0": config.gps_min_std_m > 0,
        "gps_gate_chi2 > 0": config.gps_gate_chi2 > 0,
        "gps_timeout_s >= 0": config.gps_timeout_s >= 0,
        "initial_position_std_m > 0": config.initial_position_std_m > 0,
        "max_steps_per_update >= 1": config.max_steps_per_update >= 1,
    }
    broken = [f"pose.{rule}" for rule, ok in rules.items() if not ok]
    if broken:
        raise ValueError(f"Invalid pose config: need {', '.join(broken)}")


def _require_finite(**values: float) -> None:
    for name, value in values.items():
        if not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number, got {value!r}")


def _require_positive(**values: float) -> None:
    for name, value in values.items():
        if not (math.isfinite(value) and value > 0.0):
            raise ValueError(f"{name} must be a positive number, got {value!r}")


def _heading_rad(heading_deg: float) -> float:
    return math.radians(wrap_heading_deg(heading_deg))


def _symmetric(matrix: np.ndarray) -> np.ndarray:
    return (matrix + matrix.T) / 2.0
