"""Small helpers shared across NavSight modules."""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from collections.abc import Callable
from typing import Generic, TypeVar

T = TypeVar("T")

Clock = Callable[[], float]
Sleeper = Callable[[float], None]

log = logging.getLogger(__name__)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle_deg(angle: float) -> float:
    """Wrap an angle to (-180, 180]."""
    wrapped = math.fmod(angle + 180.0, 360.0)
    if wrapped <= 0.0:
        wrapped += 360.0
    return wrapped - 180.0


def wrap_heading_deg(angle: float) -> float:
    """Wrap a compass heading to [0, 360)."""
    wrapped = math.fmod(angle, 360.0)
    if wrapped < 0.0:
        wrapped += 360.0
    return 0.0 if wrapped >= 360.0 else wrapped


class ManualClock:
    """A clock for tests and the simulator: time moves only when ``advance`` is called."""

    def __init__(self, start: float = 0.0) -> None:
        self._now = start

    def __call__(self) -> float:
        return self._now

    def advance(self, seconds: float) -> float:
        if seconds < 0:
            raise ValueError("time cannot go backwards")
        self._now += seconds
        return self._now


class LatestValue(Generic[T]):
    """Thread-safe slot holding the newest value from a producer thread.

    ``version`` increases on every ``set``, so a consumer can tell new data from a repeat.
    """

    def __init__(self, initial: T | None = None) -> None:
        self._lock = threading.Lock()
        self._value = initial
        self._version = 0 if initial is None else 1

    def set(self, value: T) -> None:
        with self._lock:
            self._value = value
            self._version += 1

    def get(self) -> T | None:
        with self._lock:
            return self._value

    def get_with_version(self) -> tuple[T | None, int]:
        with self._lock:
            return self._value, self._version

    @property
    def version(self) -> int:
        with self._lock:
            return self._version


class SampleQueue(Generic[T]):
    """Thread-safe bounded FIFO: sensor threads put samples, the core loop drains them all.

    Used where every sample matters (IMU at 100 Hz, ToF at 50 Hz) and the 30 Hz core loop
    would miss samples if it only looked at the newest one. When full, the oldest is dropped.
    """

    def __init__(self, maxlen: int = 256) -> None:
        self._lock = threading.Lock()
        self._items: deque[T] = deque(maxlen=maxlen)
        self.dropped = 0

    def put(self, item: T) -> None:
        with self._lock:
            if len(self._items) == self._items.maxlen:
                self.dropped += 1
            self._items.append(item)

    def drain(self) -> list[T]:
        with self._lock:
            items = list(self._items)
            self._items.clear()
            return items

    def __len__(self) -> int:
        with self._lock:
            return len(self._items)


class Worker:
    """Calls ``step`` repeatedly on a daemon thread at ``rate_hz`` until stopped.

    An exception from ``step`` is counted and logged (at most once per ``error_log_interval_s``)
    and the loop carries on, so one failed sensor read never kills a sensor thread.
    A rate of 0 pauses the loop without stopping the thread.
    """

    def __init__(
        self,
        name: str,
        step: Callable[[], object],
        rate_hz: float,
        *,
        clock: Clock = time.monotonic,
        error_log_interval_s: float = 5.0,
    ) -> None:
        self.name = name
        self._step = step
        self._rate_hz = max(0.0, rate_hz)
        self._clock = clock
        self._error_log_interval_s = error_log_interval_s
        self._last_error_log = -math.inf
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error_count = 0

    @property
    def rate_hz(self) -> float:
        return self._rate_hz

    @rate_hz.setter
    def rate_hz(self, value: float) -> None:
        self._rate_hz = max(0.0, value)

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.is_running:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=self.name, daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None

    def _run(self) -> None:
        next_tick = self._clock()
        while not self._stop.is_set():
            rate = self._rate_hz
            if rate <= 0.0:
                self._stop.wait(0.2)
                next_tick = self._clock()
                continue
            try:
                self._step()
            except Exception:
                self.error_count += 1
                now = self._clock()
                if now - self._last_error_log >= self._error_log_interval_s:
                    self._last_error_log = now
                    log.exception("%s step failed (%d errors so far)", self.name, self.error_count)
            period = 1.0 / rate
            next_tick += period
            delay = next_tick - self._clock()
            if delay < -period:
                # Fell far behind (slow step or a paused system): resynchronise instead of bursting.
                next_tick = self._clock()
                delay = 0.0
            if delay > 0.0:
                self._stop.wait(delay)
