"""Dual ERM vibration motors on PWM: continuous levels plus short vibration patterns.

Each motor needs a transistor driver and a flyback diode: a GPIO pin cannot drive it directly.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from navsight.config import NavSightConfig
from navsight.hardware.gpio import GpioBackend
from navsight.util import Clock, clamp

log = logging.getLogger(__name__)

PatternStep = tuple[float, float, float]  # (left, right, seconds)


def _pulses(count: int, level: float, on_s: float, off_s: float) -> tuple[PatternStep, ...]:
    """``count`` pulses on both motors, each followed by a pause.

    The pauses keep the last pulse distinct from the continuous level that resumes afterwards.
    """
    return ((level, level, on_s), (0.0, 0.0, off_s)) * count


PATTERNS: dict[str, tuple[PatternStep, ...]] = {
    "drop_off": _pulses(3, level=1.0, on_s=0.15, off_s=0.10),
    "step_up": _pulses(2, level=0.6, on_s=0.20, off_s=0.15),
    "confirm": _pulses(2, level=0.8, on_s=0.08, off_s=0.08),
    "mode_change": _pulses(1, level=0.8, on_s=0.12, off_s=0.10),
    "low_battery": _pulses(2, level=0.5, on_s=0.50, off_s=0.40),
    "fall_detected": _pulses(2, level=1.0, on_s=0.80, off_s=0.30),
    "sos_countdown": _pulses(1, level=1.0, on_s=0.30, off_s=0.20),
    "sos_sent": _pulses(3, level=1.0, on_s=0.10, off_s=0.08),
}


def intensity_to_duty(intensity: float, min_duty: float, max_duty: float) -> float:
    """PWM duty for a vibration intensity in 0..1.

    ERM motors do not spin below roughly 30 % duty, so 0 (or NaN) maps to 0 = off and any
    positive intensity to ``min_duty + (max_duty - min_duty) * clamp(intensity, 0, 1)``.
    """
    if math.isnan(intensity) or intensity <= 0.0:
        return 0.0
    return min_duty + (max_duty - min_duty) * clamp(intensity, 0.0, 1.0)


def _levels_at(steps: tuple[PatternStep, ...], elapsed_s: float) -> tuple[float, float] | None:
    """The (left, right) levels ``elapsed_s`` into a pattern, or None once it has ended."""
    step_end_s = 0.0
    for left, right, duration_s in steps:
        step_end_s += duration_s
        if elapsed_s < step_end_s:
            return left, right
    return None


def _clean_level(level: float) -> float:
    """Clamp a requested level to 0..1, treating NaN as off."""
    return 0.0 if math.isnan(level) else clamp(level, 0.0, 1.0)


class HapticController:
    """Left (GPIO 12) and right (GPIO 13) ERM motors driven by PWM.

    ``set_continuous`` sets the background levels and ``play`` starts a pattern that overrides
    them until it ends; a new pattern replaces the current one. Nothing reaches the motors until
    ``update`` runs (the app calls it ``haptics.update_hz`` times a second), and it writes a pin
    only when that pin's duty changes.
    """

    def __init__(
        self, config: NavSightConfig, gpio: GpioBackend, *, clock: Clock = time.monotonic
    ) -> None:
        settings = config.haptics
        if not 0.0 <= settings.min_duty <= settings.max_duty <= 1.0:
            raise ValueError(
                "haptics: need 0 <= min_duty <= max_duty <= 1, "
                f"got min_duty={settings.min_duty}, max_duty={settings.max_duty}"
            )
        if settings.pwm_frequency_hz <= 0:
            raise ValueError(
                f"haptics.pwm_frequency_hz must be positive, got {settings.pwm_frequency_hz}"
            )
        self._gpio = gpio
        self._clock = clock
        self._pins = (config.pins.haptic_left, config.pins.haptic_right)
        self._frequency_hz = settings.pwm_frequency_hz
        self._min_duty = settings.min_duty
        self._max_duty = settings.max_duty
        self._lock = threading.Lock()
        self._continuous = (0.0, 0.0)
        self._pattern: str | None = None
        self._pattern_start = 0.0
        self._duties = [0.0, 0.0]  # the duty last written to each pin
        self._started = False

    @property
    def playing(self) -> str | None:
        """The pattern playing now, or None."""
        with self._lock:
            if self._pattern_levels(self._clock()) is None:
                return None
            return self._pattern

    def start(self) -> None:
        """Claim both motor pins as outputs, driven low (motors off)."""
        with self._lock:
            if self._started:
                return
            for pin in self._pins:
                try:
                    self._gpio.claim_output(pin, 0)
                except Exception as exc:
                    raise RuntimeError(
                        f"Could not claim GPIO {pin} for a haptic motor ({exc}). "
                        "Is another program using that pin?"
                    ) from exc
            self._duties = [0.0, 0.0]
            self._started = True
        log.info(
            "Haptic motors ready on GPIO %d (left) and GPIO %d (right), %.0f Hz PWM",
            self._pins[0],
            self._pins[1],
            self._frequency_hz,
        )

    def set_continuous(self, left: float, right: float) -> None:
        """Background levels 0..1 (clamped), in effect whenever no pattern is playing."""
        with self._lock:
            self._continuous = (_clean_level(left), _clean_level(right))

    def play(self, pattern: str) -> None:
        """Start ``pattern`` now, replacing any pattern that is playing."""
        if pattern not in PATTERNS:
            known = ", ".join(sorted(PATTERNS))
            raise ValueError(f"Unknown haptic pattern {pattern!r}; known patterns: {known}")
        with self._lock:
            self._pattern = pattern
            self._pattern_start = self._clock()
        log.debug("Playing haptic pattern %s", pattern)

    def update(self, now: float | None = None) -> tuple[float, float]:
        """Drive the motors with the levels in effect at ``now``; returns them as (left, right).

        Raises RuntimeError before ``start()`` and after ``stop()``, so a late call from a
        worker can never switch the motors back on.
        """
        if now is None:
            now = self._clock()
        with self._lock:
            if not self._started:
                raise RuntimeError("HapticController.update() called before start()")
            levels = self._pattern_levels(now)
            if levels is None:
                self._pattern = None
                levels = self._continuous
            for index, level in enumerate(levels):
                self._write_duty(index, intensity_to_duty(level, self._min_duty, self._max_duty))
        return levels

    def stop(self) -> None:
        """Turn both motors off and clear the continuous levels and any pattern."""
        with self._lock:
            if not self._started:
                return
            self._started = False
            self._continuous = (0.0, 0.0)
            self._pattern = None
            for index, pin in enumerate(self._pins):
                try:
                    self._gpio.pwm(pin, self._frequency_hz, 0.0)
                    self._duties[index] = 0.0
                except Exception:
                    log.exception("Could not switch off the haptic motor on GPIO %d", pin)

    def _pattern_levels(self, now: float) -> tuple[float, float] | None:
        if self._pattern is None:
            return None
        return _levels_at(PATTERNS[self._pattern], now - self._pattern_start)

    def _write_duty(self, index: int, duty: float) -> None:
        if duty == self._duties[index]:
            return
        pin = self._pins[index]
        try:
            self._gpio.pwm(pin, self._frequency_hz, duty)
        except Exception as exc:
            raise RuntimeError(f"Could not set the PWM duty of GPIO {pin}: {exc}") from exc
        self._duties[index] = duty
