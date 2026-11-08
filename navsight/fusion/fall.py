"""Fall detection from the cane's IMU: free fall, impact, then stillness while lying flat.

The IMU cannot tell a fall from a dropped cane, so a detected fall starts a cancellable SOS
countdown (``navsight.safety.sos``) rather than sending an SOS straight away.
"""

from __future__ import annotations

import logging

from navsight.config import NavSightConfig
from navsight.types import ImuSample

log = logging.getLogger(__name__)

_IDLE = "idle"
_FREEFALL = "freefall"
_IMPACT = "impact"
_CONFIRMING = "confirming"


class FallDetector:
    """Recognises a fall in a stream of IMU samples, using the total acceleration in g.

    - free fall: below ``freefall_g`` for at least ``freefall_min_s``;
    - impact: above ``impact_g`` within ``impact_window_s`` after the free fall ends, or above
      ``hard_impact_g`` at any time;
    - the cane may bounce and roll for up to ``stillness_s`` after the impact, then it must stay
      still for ``stillness_s``; the fall is confirmed if it is lying roughly flat at that point.

    Movement once the cane has come to rest returns the detector to idle. Feed it every IMU
    sample: impact peaks last only tens of milliseconds.
    """

    def __init__(self, config: NavSightConfig) -> None:
        self._config = config.fall
        self._state = _IDLE
        self._low_since: float | None = None  # first sample of the current low-g run
        self._freefall_end: float | None = None  # latest sample of a free fall long enough
        self._impact_time = 0.0
        self._still_since = 0.0

    @property
    def state(self) -> str:
        """``"idle"``, ``"freefall"``, ``"impact"`` or ``"confirming"``."""
        return self._state

    def reset(self) -> None:
        self._state = _IDLE
        self._low_since = None
        self._freefall_end = None

    def update(self, sample: ImuSample) -> bool:
        """Process one sample; returns True exactly once per confirmed fall."""
        magnitude = sample.accel_magnitude_g
        if self._state == _CONFIRMING:
            if self._is_still(sample, magnitude):
                return self._try_confirm(sample)
            log.info("The cane moved after the impact: not a fall")
            self.reset()
        if self._state == _IMPACT:
            self._wait_for_stillness(sample, magnitude)
        else:
            self._watch(sample.timestamp, magnitude)
        return False

    def _watch(self, now: float, magnitude: float) -> None:
        cfg = self._config
        if magnitude < cfg.freefall_g:
            if self._low_since is None:
                self._low_since = now
            if now - self._low_since >= cfg.freefall_min_s:
                self._freefall_end = now
            self._state = _FREEFALL
            return
        self._low_since = None
        after_freefall = (
            self._freefall_end is not None and now - self._freefall_end <= cfg.impact_window_s
        )
        if magnitude > cfg.hard_impact_g or (after_freefall and magnitude > cfg.impact_g):
            log.info("Impact of %.1f g", magnitude)
            self._state = _IMPACT
            self._impact_time = now
            self._freefall_end = None
        elif not after_freefall:
            self.reset()

    def _wait_for_stillness(self, sample: ImuSample, magnitude: float) -> None:
        if self._is_still(sample, magnitude):
            self._state = _CONFIRMING
            self._still_since = sample.timestamp
        elif sample.timestamp - self._impact_time > self._config.stillness_s:
            log.info("The cane kept moving after the impact: not a fall")
            self.reset()

    def _try_confirm(self, sample: ImuSample) -> bool:
        if sample.timestamp - self._still_since < self._config.stillness_s:
            return False
        self.reset()
        if abs(sample.pitch_deg) < self._config.lying_pitch_deg:
            log.warning("Fall confirmed: impact, then the cane lay still")
            return True
        log.info("Cane still but not lying flat (pitch %.0f deg): not a fall", sample.pitch_deg)
        return False

    def _is_still(self, sample: ImuSample, magnitude: float) -> bool:
        cfg = self._config
        if abs(magnitude - 1.0) > cfg.stillness_tolerance_g:
            return False
        return all(abs(rate) <= cfg.max_still_gyro_dps for rate in sample.gyro_dps)
