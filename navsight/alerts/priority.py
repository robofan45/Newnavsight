"""Alert prioritisation for NavSight: haptic > spatial audio > speech.

``AlertManager.update`` runs once per core-loop tick. It turns the fused obstacles, a new ground
hazard and system events into that tick's outputs: continuous haptic levels, at most one haptic
pattern, panned beeps and speech, filtered by cooldowns and the user's verbosity setting.
"""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Sequence
from dataclasses import dataclass

from navsight.config import AlertConfig, NavSightConfig
from navsight.types import (
    Alert,
    AlertLevel,
    AlertOutputs,
    Beep,
    GroundHazard,
    HazardType,
    Obstacle,
    Speech,
    Verbosity,
)
from navsight.util import Clock, clamp

log = logging.getLogger(__name__)

BEEP_DURATION_S = 0.06
TTC_BOOST_LEVEL = 0.8  # minimum haptic level while the time to collision is below ttc_danger_s
REPEAT_WHEN_CLOSER_FRACTION = 0.3  # a key may repeat early once its distance shrinks more than this
MAX_PENDING_EVENTS = 4  # spoken events waiting for the speech gap; the least urgent is dropped

_DANGER_BEEP = (2000.0, 0.08)  # (frequency_hz, repeat_interval_s)
_WARNING_BEEP = (1500.0, 0.2)
_CAUTION_BEEP = (1000.0, 0.5)

_AHEAD_DEG = 10.0
_SLIGHTLY_DEG = 35.0

_HAZARD_PATTERNS = {HazardType.DROP_OFF: "drop_off", HazardType.STEP_UP: "step_up"}
_HAZARD_PHRASES = {HazardType.DROP_OFF: "Drop ahead.", HazardType.STEP_UP: "Step up ahead."}

_TIME_TOLERANCE_S = 1e-6  # interval checks forgive float rounding in summed timestamps


def direction_phrase(bearing_deg: float) -> str:
    """Spoken direction of a bearing (negative = left)."""
    magnitude = abs(bearing_deg)
    if magnitude < _AHEAD_DEG:
        return "ahead"
    side = "left" if bearing_deg < 0 else "right"
    if magnitude <= _SLIGHTLY_DEG:
        return f"slightly {side}"
    return f"on your {side}"


def distance_phrase(distance_m: float) -> str:
    """Spoken distance: "very close" below 1 m, otherwise whole meters rounded half up."""
    if not math.isfinite(distance_m):
        raise ValueError(f"distance_m must be a finite number, got {distance_m}")
    if distance_m < 1.0:
        return "very close"
    meters = math.floor(distance_m + 0.5)
    return "1 meter" if meters == 1 else f"{meters} meters"


def distance_to_beep(distance_m: float, config: AlertConfig) -> tuple[float, float] | None:
    """``(frequency_hz, repeat_interval_s)`` of the zone ``distance_m`` is in, or ``None``."""
    if distance_m < config.danger_m:
        return _DANGER_BEEP
    if distance_m < config.warning_m:
        return _WARNING_BEEP
    if distance_m < config.caution_m:
        return _CAUTION_BEEP
    return None


@dataclass(frozen=True)
class _Spoken:
    time: float
    distance_m: float | None


class AlertManager:
    """Chooses each tick's haptic, beep and speech outputs.

    Obstacles are told apart by sector (the obstacle map keeps one track per sector), so the
    warning-zone hysteresis follows each sector's obstacle. Speech that has to wait for
    ``min_speech_gap_s`` is not lost: spoken events wait in a short queue and go before obstacle
    descriptions, which are simply re-evaluated on later ticks. ``update`` takes ``now``
    explicitly; ``clock`` is accepted like in every other NavSight class.
    """

    def __init__(self, config: NavSightConfig, *, clock: Clock = time.monotonic) -> None:
        self._config = config.alerts
        self._clock = clock
        self._verbosity = _parse_verbosity(config.alerts.verbosity)
        self._warning_sectors: set[int] = set()
        self._last_hazard: GroundHazard | None = None
        self._last_beep_time = -math.inf
        self._last_speech_time = -math.inf
        self._spoken: dict[str, _Spoken] = {}
        self._pending_events: list[Alert] = []

    @property
    def verbosity(self) -> Verbosity:
        return self._verbosity

    @verbosity.setter
    def verbosity(self, value: Verbosity | str) -> None:
        verbosity = _parse_verbosity(value)
        if verbosity is not self._verbosity:
            log.info("Alert verbosity: %s", verbosity.value)
        self._verbosity = verbosity

    def update(
        self,
        now: float,
        obstacles: Sequence[Obstacle],
        hazard: GroundHazard | None,
        events: Sequence[Alert],
    ) -> AlertOutputs:
        """Compute this tick's outputs.

        ``hazard`` is the ground detector's edge-triggered result; passing the same hazard again
        does nothing. Each event is considered once, on the tick it is passed in.
        """
        ranged = [obstacle for obstacle in obstacles if _valid_distance(obstacle) is not None]
        in_warning = self._update_warning_zone(ranged)
        new_hazard = self._take_new_hazard(hazard)
        haptic_left, haptic_right = self._haptic_levels(ranged)
        urgent = [] if new_hazard is None else [_HAZARD_PHRASES[new_hazard.hazard]]
        urgent += self._take_events(now, events)
        if urgent:
            self._last_speech_time = now
        calm = self._calm_speech(now, obstacles, in_warning)
        return AlertOutputs(
            haptic_left=haptic_left,
            haptic_right=haptic_right,
            haptic_pattern=self._haptic_pattern(new_hazard, events),
            beeps=self._beeps(now, in_warning),
            speech=_speech_items(urgent, calm),
        )

    def _update_warning_zone(self, ranged: list[Obstacle]) -> list[Obstacle]:
        """Obstacles inside ``warning_m``; one that entered stays until ``hysteresis_m`` past it."""
        inside = [
            obstacle
            for obstacle in ranged
            if _distance_or_inf(obstacle) < self._warning_limit_m(obstacle)
        ]
        self._warning_sectors = {obstacle.sector for obstacle in inside}
        return inside

    def _warning_limit_m(self, obstacle: Obstacle) -> float:
        if obstacle.sector in self._warning_sectors:
            return self._config.warning_m + self._config.hysteresis_m
        return self._config.warning_m

    def _take_new_hazard(self, hazard: GroundHazard | None) -> GroundHazard | None:
        if hazard is None or hazard.hazard not in _HAZARD_PATTERNS or hazard == self._last_hazard:
            return None
        self._last_hazard = hazard
        return hazard

    def _haptic_levels(self, ranged: list[Obstacle]) -> tuple[float, float]:
        config = self._config
        nearest = min(ranged, key=_distance_or_inf, default=None)
        if nearest is None or _distance_or_inf(nearest) >= config.caution_m:
            return 0.0, 0.0
        span_m = config.caution_m - config.danger_m
        level = clamp((config.caution_m - _distance_or_inf(nearest)) / span_m, 0.0, 1.0)
        ttc_s = nearest.time_to_collision_s
        if ttc_s is not None and ttc_s < config.ttc_danger_s:
            level = max(level, TTC_BOOST_LEVEL)
        return _motor_levels(level, nearest.bearing_deg, config.side_bearing_deg)

    def _haptic_pattern(
        self, new_hazard: GroundHazard | None, events: Sequence[Alert]
    ) -> str | None:
        """The new hazard's pattern first, otherwise the most urgent event's pattern."""
        with_pattern = [event for event in events if event.haptic_pattern]
        if new_hazard is not None:
            if with_pattern:
                skipped = [event.haptic_pattern for event in with_pattern]
                log.debug("Ground hazard pattern takes this tick; skipped %s", skipped)
            return _HAZARD_PATTERNS[new_hazard.hazard]
        if not with_pattern:
            return None
        return max(with_pattern, key=lambda event: event.level).haptic_pattern

    def _beeps(self, now: float, in_warning: list[Obstacle]) -> tuple[Beep, ...]:
        if self._verbosity is Verbosity.HAPTIC_ONLY or not in_warning:
            return ()
        nearest = min(in_warning, key=_distance_or_inf)
        zone = distance_to_beep(_distance_or_inf(nearest), self._config)
        if zone is None:
            return ()
        frequency_hz, repeat_interval_s = zone
        if not _has_elapsed(self._last_beep_time, now, repeat_interval_s):
            return ()
        self._last_beep_time = now
        return (Beep(frequency_hz, BEEP_DURATION_S, nearest.bearing_deg),)

    def _take_events(self, now: float, events: Sequence[Alert]) -> list[str]:
        """Return the DANGER messages to speak now and queue the other spoken events."""
        urgent: list[str] = []
        seen_keys: set[str] = set()
        for event in sorted(events, key=lambda event: event.level, reverse=True):
            text = event.message.strip()
            if not text or event.key in seen_keys:
                continue
            seen_keys.add(event.key)
            if event.level >= AlertLevel.DANGER:
                urgent.append(text)
            elif self._event_speech_allowed(event) and self._key_ready(event.key, now, None):
                self._queue_event(event)
        return urgent

    def _queue_event(self, event: Alert) -> None:
        if any(pending.key == event.key for pending in self._pending_events):
            return
        self._pending_events.append(event)
        if len(self._pending_events) > MAX_PENDING_EVENTS:
            dropped = min(self._pending_events, key=lambda pending: pending.level)
            self._pending_events.remove(dropped)
            log.warning("Too many announcements waiting; dropped %r", dropped.message)

    def _calm_speech(
        self, now: float, obstacles: Sequence[Obstacle], in_warning: list[Obstacle]
    ) -> str | None:
        """One non-interrupting item once ``min_speech_gap_s`` has passed: an event first."""
        if not _has_elapsed(self._last_speech_time, now, self._config.min_speech_gap_s):
            return None
        text = self._next_pending_event(now)
        if text is None:
            text = self._obstacle_description(now, obstacles, in_warning)
        if text is not None:
            self._last_speech_time = now
        return text

    def _next_pending_event(self, now: float) -> str | None:
        while self._pending_events:
            event = max(self._pending_events, key=lambda pending: pending.level)
            self._pending_events.remove(event)
            if self._event_speech_allowed(event):
                self._spoken[event.key] = _Spoken(now, None)
                return event.message.strip()
            log.debug("Not announcing %r at verbosity %s", event.message, self._verbosity.value)
        return None

    def _obstacle_description(
        self, now: float, obstacles: Sequence[Obstacle], in_warning: list[Obstacle]
    ) -> str | None:
        if self._verbosity is Verbosity.HAPTIC_ONLY:
            return None
        if any(_distance_or_inf(obstacle) < self._config.danger_m for obstacle in obstacles):
            return None
        for obstacle in sorted(obstacles, key=_distance_or_inf):
            if not obstacle.label or not self._describable(obstacle, in_warning):
                continue
            direction = direction_phrase(obstacle.bearing_deg)
            key = f"{obstacle.label}:{direction}"
            distance_m = _valid_distance(obstacle)
            if self._key_ready(key, now, distance_m):
                self._spoken[key] = _Spoken(now, distance_m)
                return _obstacle_phrase(obstacle.label, direction, distance_m)
        return None

    def _describable(self, obstacle: Obstacle, in_warning: list[Obstacle]) -> bool:
        distance_m = _valid_distance(obstacle)
        within_range = distance_m is None or distance_m <= self._config.label_announce_m
        if self._verbosity is Verbosity.BRIEF:
            return within_range and obstacle in in_warning
        return within_range

    def _event_speech_allowed(self, event: Alert) -> bool:
        return self._verbosity is not Verbosity.HAPTIC_ONLY or event.level >= AlertLevel.WARNING

    def _key_ready(self, key: str, now: float, distance_m: float | None) -> bool:
        """True when ``key`` is off cooldown, or its distance shrank enough to repeat early."""
        last = self._spoken.get(key)
        if last is None or _has_elapsed(last.time, now, self._config.tts_cooldown_s):
            return True
        if distance_m is None or last.distance_m is None:
            return False
        return distance_m < last.distance_m * (1.0 - REPEAT_WHEN_CLOSER_FRACTION)


def _parse_verbosity(value: Verbosity | str) -> Verbosity:
    try:
        return Verbosity(value)
    except ValueError as error:
        choices = ", ".join(verbosity.value for verbosity in Verbosity)
        raise ValueError(f"verbosity must be one of {choices}; got {value!r}") from error


def _valid_distance(obstacle: Obstacle) -> float | None:
    """The obstacle's distance; a non-finite value counts as unknown."""
    distance_m = obstacle.distance_m
    if distance_m is None or not math.isfinite(distance_m):
        return None
    return distance_m


def _distance_or_inf(obstacle: Obstacle) -> float:
    distance_m = _valid_distance(obstacle)
    return math.inf if distance_m is None else distance_m


def _has_elapsed(since: float, now: float, interval_s: float) -> bool:
    return now - since >= interval_s - _TIME_TOLERANCE_S


def _motor_levels(level: float, bearing_deg: float, side_bearing_deg: float) -> tuple[float, float]:
    if bearing_deg < -side_bearing_deg:
        return level, 0.0
    if bearing_deg > side_bearing_deg:
        return 0.0, level
    return level, level


def _obstacle_phrase(label: str, direction: str, distance_m: float | None) -> str:
    if distance_m is None:
        return f"{label} {direction}."
    return f"{label} {direction}, {distance_phrase(distance_m)}."


def _speech_items(urgent: list[str], calm: str | None) -> tuple[Speech, ...]:
    # Only the first urgent item interrupts: a second interrupt would cut off the first one.
    items = [Speech(text, interrupt=index == 0) for index, text in enumerate(urgent)]
    if calm is not None:
        items.append(Speech(calm))
    return tuple(items)
