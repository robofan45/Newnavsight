"""Sector obstacle map: Kalman range tracks per bearing sector, with time to collision."""

from __future__ import annotations

import copy
import logging
import math
from dataclasses import dataclass

import numpy as np

from navsight.config import FusionConfig, NavSightConfig
from navsight.types import Detection, Obstacle, ToFReading

log = logging.getLogger(__name__)

INITIAL_VELOCITY_VARIANCE = 1.0  # m^2/s^2 for a new track, which starts at rest
_H = np.array([1.0, 0.0])


class RangeTrack:
    """A 1-D constant-velocity Kalman filter on the range to one obstacle.

    The state is ``[range, range_rate]``. A measurement older than the state is applied without
    predicting backwards, and ``last_update`` never moves back in time.
    """

    def __init__(self, distance_m: float, std_m: float, now: float, accel_std_mps2: float) -> None:
        _check_measurement(distance_m, std_m)
        self._x = np.array([distance_m, 0.0])
        self._p = np.diag([std_m**2, INITIAL_VELOCITY_VARIANCE])
        self._q = accel_std_mps2**2
        self._time = now
        self._last_update = now

    @property
    def distance_m(self) -> float:
        return float(self._x[0])

    @property
    def closing_speed_mps(self) -> float:
        return -float(self._x[1])

    @property
    def last_update(self) -> float:
        return self._last_update

    def predict(self, now: float) -> None:
        dt = now - self._time
        if dt <= 0.0:
            return
        f = np.array([[1.0, dt], [0.0, 1.0]])
        q = self._q * np.array([[dt**4 / 4.0, dt**3 / 2.0], [dt**3 / 2.0, dt**2]])
        self._x = f @ self._x
        self._p = f @ self._p @ f.T + q
        self._time = now

    def update(self, distance_m: float, std_m: float, now: float) -> None:
        _check_measurement(distance_m, std_m)
        self.predict(now)
        r = std_m**2
        gain = self._p @ _H / (self._p[0, 0] + r)
        self._x = self._x + gain * (distance_m - self._x[0])
        # Joseph form: keeps the covariance symmetric and positive definite.
        i_kh = np.eye(2) - np.outer(gain, _H)
        self._p = i_kh @ self._p @ i_kh.T + r * np.outer(gain, gain)
        self._last_update = max(self._last_update, now)

    def predicted(self, now: float) -> RangeTrack:
        """Return a copy predicted to ``now``, leaving this track unchanged."""
        track = copy.deepcopy(self)
        track.predict(now)
        return track

    def time_to_collision(self, min_closing_mps: float) -> float | None:
        closing = self.closing_speed_mps
        if closing <= 0.0 or closing < min_closing_mps:
            return None
        return max(self.distance_m, 0.0) / closing


@dataclass
class _SectorTrack:
    track: RangeTrack
    last_source: str  # "tof" or "vision": the source of the most recent update
    tof_time: float | None = None
    vision_time: float | None = None


@dataclass(frozen=True)
class _SectorLabel:
    label: str
    confidence: float
    timestamp: float


class ObstacleMap:
    """Obstacles ahead of the cane, as one range track and one vision label per bearing sector.

    The forward ToF feeds the centre sector; each vision detection feeds the sector of its
    bearing. When several detections share a sector in one frame, the most confident one gives
    the label and the nearest distance updates the track. A label from a newer frame replaces
    the older one. Obstacles report the sector centre as their bearing and never a negative
    distance.
    """

    def __init__(self, config: NavSightConfig) -> None:
        _check_fusion_config(config.fusion)
        self._fusion = config.fusion
        self._width = self._fusion.sector_span_deg / self._fusion.sector_count
        self._centre = self._fusion.sector_count // 2
        self._tracks: dict[int, _SectorTrack] = {}
        self._labels: dict[int, _SectorLabel] = {}

    def sector_for_bearing(self, bearing_deg: float) -> int | None:
        half_span = self._fusion.sector_span_deg / 2.0
        if not -half_span <= bearing_deg <= half_span:
            return None
        sector = math.floor((bearing_deg + half_span) / self._width)
        return min(sector, self._fusion.sector_count - 1)

    def sector_center(self, sector: int) -> float:
        count = self._fusion.sector_count
        if not 0 <= sector < count:
            raise ValueError(f"sector must be between 0 and {count - 1}, got {sector}")
        return -self._fusion.sector_span_deg / 2.0 + (sector + 0.5) * self._width

    def update_tof(self, reading: ToFReading) -> None:
        if reading.sensor != "forward":
            raise ValueError(f"expected a forward ToF reading, got sensor {reading.sensor!r}")
        distance = _valid_distance(reading.distance_m)
        if distance is None:
            self._remove_tof_track()
            return
        std = max(self._fusion.tof_range_std_frac * distance, self._fusion.tof_range_std_min_m)
        self._update_track(self._centre, distance, std, reading.timestamp, "tof")

    def update_detections(self, detections: list[Detection], now: float) -> None:
        best: dict[int, Detection] = {}
        nearest: dict[int, float] = {}
        for detection in detections:
            sector = self.sector_for_bearing(detection.bearing_deg)
            if sector is None:
                log.debug(
                    "Ignoring %s at bearing %.1f deg: outside the obstacle map",
                    detection.label,
                    detection.bearing_deg,
                )
                continue
            if sector not in best or detection.confidence > best[sector].confidence:
                best[sector] = detection
            distance = _valid_distance(detection.distance_m)
            if distance is not None and distance < nearest.get(sector, math.inf):
                nearest[sector] = distance
        for sector, detection in best.items():
            self._labels[sector] = _SectorLabel(detection.label, detection.confidence, now)
        for sector, distance in nearest.items():
            std = self._fusion.vision_range_std_frac * distance
            self._update_track(sector, distance, std, now, "vision")

    def obstacles(self, now: float) -> list[Obstacle]:
        """Obstacles at ``now``, nearest first, label-only last. Stored state is not changed."""
        found = []
        for sector in range(self._fusion.sector_count):
            obstacle = self._obstacle_in(sector, now)
            if obstacle is not None:
                found.append(obstacle)
        found.sort(key=_sort_key)
        return found

    def nearest(self, now: float) -> Obstacle | None:
        for obstacle in self.obstacles(now):
            if obstacle.distance_m is not None:
                return obstacle
        return None

    def _update_track(
        self, sector: int, distance: float, std: float, now: float, source: str
    ) -> None:
        entry = self._tracks.get(sector)
        if entry is None or not self._is_recent(entry.track.last_update, now):
            track = RangeTrack(distance, std, now, self._fusion.range_accel_std_mps2)
            entry = _SectorTrack(track, source)
            self._tracks[sector] = entry
        else:
            entry.track.update(distance, std, now)
            entry.last_source = source
        if source == "tof":
            entry.tof_time = now
        else:
            entry.vision_time = now

    def _remove_tof_track(self) -> None:
        entry = self._tracks.get(self._centre)
        if entry is not None and entry.last_source == "tof":
            del self._tracks[self._centre]

    def _obstacle_in(self, sector: int, now: float) -> Obstacle | None:
        label = self._recent_label(sector, now)
        entry = self._tracks.get(sector)
        if entry is not None and self._is_recent(entry.track.last_update, now):
            return self._track_obstacle(sector, entry, label, now)
        if label is None:
            return None
        return Obstacle(
            sector=sector,
            bearing_deg=self.sector_center(sector),
            distance_m=None,
            closing_speed_mps=0.0,
            time_to_collision_s=None,
            label=label.label,
            source="vision",
            confidence=label.confidence,
        )

    def _track_obstacle(
        self, sector: int, entry: _SectorTrack, label: _SectorLabel | None, now: float
    ) -> Obstacle:
        track = entry.track.predicted(now)
        source = self._source(entry, now)
        confidence = 1.0 if source == "tof" or label is None else label.confidence
        return Obstacle(
            sector=sector,
            bearing_deg=self.sector_center(sector),
            distance_m=max(track.distance_m, 0.0),
            closing_speed_mps=track.closing_speed_mps,
            time_to_collision_s=track.time_to_collision(self._fusion.ttc_min_closing_mps),
            label=None if label is None else label.label,
            source=source,
            confidence=confidence,
        )

    def _source(self, entry: _SectorTrack, now: float) -> str:
        from_tof = self._is_recent(entry.tof_time, now)
        from_vision = self._is_recent(entry.vision_time, now)
        if from_tof and from_vision:
            return "fused"
        return "tof" if from_tof else "vision"

    def _recent_label(self, sector: int, now: float) -> _SectorLabel | None:
        label = self._labels.get(sector)
        if label is None or now - label.timestamp > self._fusion.label_timeout_s:
            return None
        return label

    def _is_recent(self, timestamp: float | None, now: float) -> bool:
        return timestamp is not None and now - timestamp <= self._fusion.track_timeout_s


def _sort_key(obstacle: Obstacle) -> tuple[bool, float, float]:
    distance = obstacle.distance_m
    return (distance is None, 0.0 if distance is None else distance, abs(obstacle.bearing_deg))


def _valid_distance(distance_m: float | None) -> float | None:
    """A measured distance, or ``None`` for no target or an invalid value."""
    if distance_m is None or not math.isfinite(distance_m) or distance_m <= 0.0:
        return None
    return distance_m


def _check_measurement(distance_m: float, std_m: float) -> None:
    if not math.isfinite(distance_m):
        raise ValueError(f"range measurement must be finite, got {distance_m}")
    if not (math.isfinite(std_m) and std_m > 0.0):
        raise ValueError(f"range standard deviation must be positive and finite, got {std_m}")


def _check_fusion_config(fusion: FusionConfig) -> None:
    errors = []
    if fusion.sector_count < 1:
        errors.append("fusion.sector_count must be at least 1")
    if fusion.sector_span_deg <= 0:
        errors.append("fusion.sector_span_deg must be positive")
    if fusion.tof_range_std_min_m <= 0:
        errors.append("fusion.tof_range_std_min_m must be positive")
    if fusion.vision_range_std_frac <= 0:
        errors.append("fusion.vision_range_std_frac must be positive")
    if errors:
        raise ValueError("Invalid obstacle map config: " + "; ".join(errors))
