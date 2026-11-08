"""Drop-off and step-up detection from the down-facing ToF."""

from __future__ import annotations

import logging
import math
import statistics
from collections import deque

from navsight.config import HazardConfig, NavSightConfig
from navsight.types import GroundHazard, HazardType, ToFReading

log = logging.getLogger(__name__)

# No echo at all means a drop-off only when the ground normally sits well inside the range.
NO_ECHO_BASELINE_FRACTION = 0.9


class GroundHazardDetector:
    """Compares the down ToF's slant distance with a learned ground baseline.

    The baseline is frozen while a hazard is active, so the hazard clears once readings return
    to the ground level seen before it. Baseline samples older than ``baseline_window_s`` are
    forgotten, so after a long hazard or a gap in readings the baseline is learned again.
    A reading taken with the sensor 90 degrees or more from vertical is ignored.
    """

    def __init__(self, config: NavSightConfig) -> None:
        _check_hazard_config(config.hazard)
        self._config = config.hazard
        self._max_range_m = config.tof.max_range_m
        self._samples: deque[tuple[float, float]] = deque()  # (timestamp, distance)
        self._current = HazardType.NONE
        self._frozen_baseline: float | None = None
        self._candidate = HazardType.NONE
        self._candidate_count = 0
        self._normal_since: float | None = None

    @property
    def baseline_m(self) -> float | None:
        if self._current is not HazardType.NONE:
            return self._frozen_baseline
        if len(self._samples) < self._config.min_baseline_samples:
            return None
        return statistics.median(distance for _, distance in self._samples)

    @property
    def current(self) -> HazardType:
        return self._current

    def update(
        self,
        reading: ToFReading,
        tilt_from_vertical_deg: float | None = None,
        pitch_rate_dps: float | None = None,
    ) -> GroundHazard | None:
        """Process one down ToF reading; return a hazard only when one is newly confirmed."""
        if reading.sensor != "down":
            raise ValueError(f"expected a down ToF reading, got sensor {reading.sensor!r}")
        if pitch_rate_dps is not None and abs(pitch_rate_dps) > self._config.max_pitch_rate_dps:
            return None
        cos_tilt = _cos_tilt(tilt_from_vertical_deg)
        if cos_tilt is None:
            log.debug("Ignoring a down ToF reading at %s deg from vertical", tilt_from_vertical_deg)
            return None
        now = reading.timestamp
        distance = _valid_distance(reading.distance_m)
        self._forget_samples_before(now - self._config.baseline_window_s)
        hazard = self._detect(distance, cos_tilt, now)
        if distance is not None and self._current is HazardType.NONE:
            self._samples.append((now, distance))
        return hazard

    def _detect(self, distance: float | None, cos_tilt: float, now: float) -> GroundHazard | None:
        baseline = self.baseline_m
        if baseline is None:
            return None
        sample = self._classify(distance, baseline, cos_tilt)
        if sample is None:
            return None
        kind, delta = sample
        if kind is HazardType.NONE:
            self._on_normal_sample(now)
            return None
        return self._on_hazard_sample(kind, delta, baseline, now)

    def _classify(
        self, distance: float | None, baseline: float, cos_tilt: float
    ) -> tuple[HazardType, float] | None:
        """Return the sample's type and vertical delta, or ``None`` if it carries no information."""
        if distance is None:
            if baseline < NO_ECHO_BASELINE_FRACTION * self._max_range_m:
                return HazardType.DROP_OFF, self._max_range_m - baseline
            return None
        delta = (distance - baseline) * cos_tilt
        if delta > self._config.drop_threshold_m:
            return HazardType.DROP_OFF, delta
        if -delta > self._config.step_up_threshold_m:
            return HazardType.STEP_UP, delta
        return HazardType.NONE, delta

    def _on_normal_sample(self, now: float) -> None:
        self._candidate = HazardType.NONE
        self._candidate_count = 0
        if self._current is HazardType.NONE:
            return
        if self._normal_since is None:
            self._normal_since = now
        if now - self._normal_since >= self._config.clear_after_s:
            log.debug("Ground hazard cleared: %s", self._current.value)
            self._current = HazardType.NONE
            self._frozen_baseline = None
            self._normal_since = None

    def _on_hazard_sample(
        self, kind: HazardType, delta: float, baseline: float, now: float
    ) -> GroundHazard | None:
        self._normal_since = None
        if kind is self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = kind
            self._candidate_count = 1
        if self._candidate_count < self._config.confirm_samples or kind is self._current:
            return None
        self._current = kind
        self._frozen_baseline = baseline
        log.info("Ground hazard confirmed: %s, %.2f m", kind.value, delta)
        return GroundHazard(hazard=kind, delta_m=delta, timestamp=now)

    def _forget_samples_before(self, cutoff: float) -> None:
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()


def _valid_distance(distance_m: float | None) -> float | None:
    """A measured distance, or ``None`` for no echo or an invalid value."""
    if distance_m is None or not math.isfinite(distance_m) or distance_m <= 0.0:
        return None
    return distance_m


def _cos_tilt(tilt_deg: float | None) -> float | None:
    """``cos(tilt)``: 1 when the tilt is unknown, ``None`` when the sensor cannot see the ground."""
    if tilt_deg is None:
        return 1.0
    if not math.isfinite(tilt_deg) or abs(tilt_deg) >= 90.0:
        return None
    return math.cos(math.radians(tilt_deg))


def _check_hazard_config(hazard: HazardConfig) -> None:
    errors = []
    if hazard.min_baseline_samples < 1:
        errors.append("hazard.min_baseline_samples must be at least 1")
    if hazard.confirm_samples < 1:
        errors.append("hazard.confirm_samples must be at least 1")
    if hazard.baseline_window_s <= 0:
        errors.append("hazard.baseline_window_s must be positive")
    if errors:
        raise ValueError("Invalid ground hazard config: " + "; ".join(errors))
