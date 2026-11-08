"""Mode, Action and SOS buttons: debounced short and long presses.

Each button connects its GPIO pin to ground and the pin's internal pull-up holds it high, so a
pressed button reads 0 (active low).
"""

from __future__ import annotations

import logging
import threading
import time

from navsight.config import NavSightConfig
from navsight.hardware.gpio import GpioBackend
from navsight.types import ButtonEvent, ButtonId, PressType
from navsight.util import Clock

log = logging.getLogger(__name__)

# Absorbs floating-point error in accumulated timestamps, so a level held for exactly
# debounce_s or long_press_s counts as having lasted that long.
_TIME_TOLERANCE_S = 1e-9


def _has_lasted(since: float, now: float, duration_s: float) -> bool:
    return now - since >= duration_s - _TIME_TOLERANCE_S


class ButtonDebouncer:
    """Debounces one button and recognises SHORT and LONG presses.

    A raw level change counts only once it has been stable for ``debounce_s``. LONG fires once,
    while the button is still held, when the press has lasted ``long_press_s`` (timed from the
    start of the stable press). Releasing before that gives SHORT; releasing after a LONG gives
    nothing. LONG waits while a release is being debounced, so a press released just before
    ``long_press_s`` still counts as SHORT.
    """

    def __init__(
        self,
        button: ButtonId,
        *,
        debounce_s: float = 0.03,
        long_press_s: float = 1.0,
        active_low: bool = True,
    ) -> None:
        if debounce_s < 0:
            raise ValueError(f"debounce_s must be >= 0, got {debounce_s}")
        if long_press_s <= 0:
            raise ValueError(f"long_press_s must be > 0, got {long_press_s}")
        self.button = button
        self._debounce_s = debounce_s
        self._long_press_s = long_press_s
        self._active_low = active_low
        self._raw_pressed = False
        self._raw_since = 0.0
        self._pressed = False
        self._pressed_since = 0.0
        self._long_sent = False

    @property
    def pressed(self) -> bool:
        """The debounced state."""
        return self._pressed

    def update(self, level: int, now: float) -> ButtonEvent | None:
        """Feed one raw pin sample taken at ``now``; returns the event it completes, if any."""
        raw_pressed = (level == 0) if self._active_low else (level != 0)
        if raw_pressed != self._raw_pressed:
            self._raw_pressed = raw_pressed
            self._raw_since = now
        if raw_pressed != self._pressed and _has_lasted(self._raw_since, now, self._debounce_s):
            if not raw_pressed:
                return self._release(now)
            self._press()
        return self._check_long_press(now)

    def _press(self) -> None:
        self._pressed = True
        self._pressed_since = self._raw_since
        self._long_sent = False

    def _release(self, now: float) -> ButtonEvent | None:
        self._pressed = False
        if self._long_sent:
            return None
        return ButtonEvent(self.button, PressType.SHORT, now)

    def _check_long_press(self, now: float) -> ButtonEvent | None:
        if not (self._pressed and self._raw_pressed) or self._long_sent:
            return None
        if not _has_lasted(self._pressed_since, now, self._long_press_s):
            return None
        self._long_sent = True
        return ButtonEvent(self.button, PressType.LONG, now)


class Buttons:
    """The Mode (GPIO 17), Action (GPIO 22) and SOS (GPIO 27) buttons.

    The button worker calls ``poll()`` (``buttons.poll_hz``, 200 Hz by default) and the core loop
    collects the events with ``drain()``. A LONG press of the SOS button needs
    ``sos_long_press_s``; the other buttons need ``long_press_s``.
    """

    def __init__(
        self, config: NavSightConfig, gpio: GpioBackend, *, clock: Clock = time.monotonic
    ) -> None:
        self._timing = config.buttons
        self._gpio = gpio
        self._clock = clock
        self._pins = {
            ButtonId.MODE: config.pins.button_mode,
            ButtonId.ACTION: config.pins.button_action,
            ButtonId.SOS: config.pins.button_sos,
        }
        self._lock = threading.Lock()
        self._debouncers = self._new_debouncers()
        self._queue: list[ButtonEvent] = []
        self._started = False

    def start(self) -> None:
        """Claim the button pins as inputs with pull-ups."""
        with self._lock:
            if self._started:
                return
            for button, pin in self._pins.items():
                try:
                    self._gpio.claim_input(pin, pull_up=True)
                except Exception as exc:
                    raise RuntimeError(
                        f"Could not claim GPIO {pin} for the {button.value} button ({exc}). "
                        "Is another program using that pin?"
                    ) from exc
            self._started = True
        pins = ", ".join(f"{button.value} GPIO {pin}" for button, pin in self._pins.items())
        log.info("Buttons ready (%s)", pins)

    def poll(self) -> list[ButtonEvent]:
        """Sample every button pin once; new events are returned and also queued for drain()."""
        with self._lock:
            if not self._started:
                raise RuntimeError("Buttons.poll() called before start()")
            now = self._clock()
            levels = self._read_levels()
            events: list[ButtonEvent] = []
            for button, level in levels.items():
                event = self._debouncers[button].update(level, now)
                if event is not None:
                    events.append(event)
            self._queue.extend(events)
        for event in events:
            log.info("%s button: %s press", event.button.value, event.press.value)
        return events

    def drain(self) -> list[ButtonEvent]:
        """Events queued since the last drain, oldest first. Safe to call from any thread."""
        with self._lock:
            events, self._queue = self._queue, []
        return events

    def is_held(self, button: ButtonId) -> bool:
        """True while ``button`` is pressed (debounced)."""
        with self._lock:
            return self._debouncers[button].pressed

    def stop(self) -> None:
        """Stop sampling and forget held buttons; queued events can still be drained."""
        with self._lock:
            self._started = False
            self._debouncers = self._new_debouncers()

    def _new_debouncers(self) -> dict[ButtonId, ButtonDebouncer]:
        return {
            button: ButtonDebouncer(
                button,
                debounce_s=self._timing.debounce_s,
                long_press_s=self._long_press_s(button),
            )
            for button in self._pins
        }

    def _long_press_s(self, button: ButtonId) -> float:
        if button is ButtonId.SOS:
            return self._timing.sos_long_press_s
        return self._timing.long_press_s

    def _read_levels(self) -> dict[ButtonId, int]:
        """Read every pin before any debouncer changes, so a failed read leaves no partial state."""
        levels: dict[ButtonId, int] = {}
        for button, pin in self._pins.items():
            try:
                levels[button] = self._gpio.read(pin)
            except Exception as exc:
                raise RuntimeError(
                    f"Could not read GPIO {pin} ({button.value} button): {exc}"
                ) from exc
        return levels


class SimulatedButtons:
    """Twin of ``Buttons`` for the simulator and tests: the script injects finished events.

    Each injected event is returned once by ``poll()`` and once by ``drain()``, like a real one.
    ``config`` and ``clock`` are optional and unused; they keep the constructor interchangeable
    with the other simulated devices.
    """

    def __init__(
        self, config: NavSightConfig | None = None, *, clock: Clock = time.monotonic
    ) -> None:
        self._lock = threading.Lock()
        self._unpolled: list[ButtonEvent] = []
        self._queue: list[ButtonEvent] = []
        self._held: set[ButtonId] = set()

    def inject(self, event: ButtonEvent) -> None:
        with self._lock:
            self._unpolled.append(event)
            self._queue.append(event)
        log.debug("Simulated %s button: %s press", event.button.value, event.press.value)

    def set_held(self, button: ButtonId, held: bool = True) -> None:
        """Simulator setter for what ``is_held`` reports."""
        with self._lock:
            if held:
                self._held.add(button)
            else:
                self._held.discard(button)

    def start(self) -> None:
        """Nothing to claim: simulated buttons are always ready."""

    def poll(self) -> list[ButtonEvent]:
        """Events injected since the last poll."""
        with self._lock:
            events, self._unpolled = self._unpolled, []
        return events

    def drain(self) -> list[ButtonEvent]:
        """Events injected since the last drain, oldest first. Safe to call from any thread."""
        with self._lock:
            events, self._queue = self._queue, []
        return events

    def is_held(self, button: ButtonId) -> bool:
        with self._lock:
            return button in self._held

    def stop(self) -> None:
        """Release every held button."""
        with self._lock:
            self._held.clear()
