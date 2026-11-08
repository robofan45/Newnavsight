"""VL53L1X time-of-flight pair: the forward obstacle sensor and the down-looking ground sensor.

Both sensors boot at I2C address 0x29 and only the down sensor's XSHUT pin is wired, so every
start gives them their own addresses in software (``assign_tof_addresses``).
"""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import Any, Protocol

from navsight.config import I2CAddressConfig, NavSightConfig
from navsight.hardware.gpio import GpioBackend
from navsight.hardware.i2c import I2C_LOCK
from navsight.types import ToFReading
from navsight.util import Clock, Sleeper

log = logging.getLogger(__name__)

DEFAULT_ADDRESS = 0x29  # every VL53L1X answers here after power-up or reset
XSHUT_SETTLE_S = 0.01
ERROR_LOG_INTERVAL_S = 5.0

_FORWARD_HINT = "Check the forward sensor's power and its SDA (GPIO 2) and SCL (GPIO 3) wiring."


class ToFDriver(Protocol):
    """The subset of ``adafruit_vl53l1x.VL53L1X`` used here."""

    distance_mode: int
    timing_budget: int

    @property
    def data_ready(self) -> bool: ...

    @property
    def distance(self) -> float | None: ...  # centimetres; None = no target

    def start_ranging(self) -> None: ...

    def stop_ranging(self) -> None: ...

    def clear_interrupt(self) -> None: ...

    def set_address(self, new_address: int) -> None: ...


DriverFactory = Callable[[int], ToFDriver]  # I2C address -> driver; raises if nothing answers


def make_vl53l1x_factory(i2c: Any) -> DriverFactory:
    """Return a factory that opens ``adafruit_vl53l1x`` drivers on ``i2c`` (from ``open_i2c``)."""

    def factory(address: int) -> ToFDriver:
        import adafruit_vl53l1x  # only available on the Pi

        return adafruit_vl53l1x.VL53L1X(i2c, address=address)

    return factory


def assign_tof_addresses(
    config: NavSightConfig,
    gpio: GpioBackend,
    factory: DriverFactory,
    sleep: Sleeper = time.sleep,
) -> tuple[ToFDriver, ToFDriver]:
    """Move the forward sensor to ``i2c.tof_forward`` and the down sensor to ``i2c.tof_down``.

    Returns ``(forward, down)``. Addresses set by an earlier run survive until power-off, so the
    forward sensor is looked for at every address it may have been left at. A failed step
    raises ``RuntimeError`` naming the step and what to check.
    """
    addresses = config.i2c
    xshut = config.pins.tof_down_xshut

    with _setup_step(
        1,
        f"hold the down sensor in reset with XSHUT (GPIO {xshut}) low",
        f"Check that no other program is using GPIO {xshut}.",
    ):
        gpio.claim_output(xshut, 0)
    sleep(XSHUT_SETTLE_S)

    forward, forward_address = _find_forward_sensor(addresses, factory, xshut)

    if forward_address != addresses.tof_temporary:
        with (
            _setup_step(
                3, f"park the forward sensor at {addresses.tof_temporary:#04x}", _FORWARD_HINT
            ),
            I2C_LOCK,
        ):
            forward.set_address(addresses.tof_temporary)

    with _setup_step(
        4,
        f"release the down sensor from reset with XSHUT (GPIO {xshut}) high",
        f"Check that no other program is using GPIO {xshut}.",
    ):
        gpio.write(xshut, 1)
    sleep(XSHUT_SETTLE_S)

    with (
        _setup_step(
            5,
            f"open the down sensor at {DEFAULT_ADDRESS:#04x} and move it to "
            f"{addresses.tof_down:#04x}",
            "Check the down sensor's power and SDA/SCL wiring, and that its XSHUT pin is wired "
            f"to GPIO {xshut}.",
        ),
        I2C_LOCK,
    ):
        down = factory(DEFAULT_ADDRESS)
        down.set_address(addresses.tof_down)

    with (
        _setup_step(6, f"move the forward sensor to {addresses.tof_forward:#04x}", _FORWARD_HINT),
        I2C_LOCK,
    ):
        forward.set_address(addresses.tof_forward)

    log.info(
        "ToF sensors addressed: forward at %#04x, down at %#04x",
        addresses.tof_forward,
        addresses.tof_down,
    )
    return forward, down


@contextmanager
def _setup_step(number: int, action: str, hint: str) -> Iterator[None]:
    try:
        yield
    except Exception as exc:
        raise RuntimeError(f"ToF setup step {number} failed ({action}): {exc}. {hint}") from exc


def _forward_candidates(addresses: I2CAddressConfig) -> list[int]:
    ordered = (DEFAULT_ADDRESS, addresses.tof_temporary, addresses.tof_forward, addresses.tof_down)
    return list(dict.fromkeys(ordered))


def _find_forward_sensor(
    addresses: I2CAddressConfig, factory: DriverFactory, xshut: int
) -> tuple[ToFDriver, int]:
    """Step 2: open the forward sensor at the first candidate address that answers."""
    candidates = _forward_candidates(addresses)
    failures: list[str] = []
    for address in candidates:
        try:
            with I2C_LOCK:
                return factory(address), address
        except Exception as exc:
            failures.append(f"{address:#04x}: {exc}")
    tried = ", ".join(f"{address:#04x}" for address in candidates)
    raise RuntimeError(
        f"ToF setup step 2 failed (find the forward sensor): nothing answered at {tried} "
        f"({'; '.join(failures)}). {_FORWARD_HINT} If both sensors answer at "
        f"{DEFAULT_ADDRESS:#04x} together, the down sensor's XSHUT pin is not wired to "
        f"GPIO {xshut}, so it is never held in reset."
    )


def _valid_distance(distance_m: float | None, max_range_m: float) -> float | None:
    if distance_m is None or not 0.0 < distance_m <= max_range_m:
        return None
    return distance_m


class ToFPair:
    """The forward and down VL53L1X sensors on the shared I2C bus."""

    def __init__(
        self,
        config: NavSightConfig,
        gpio: GpioBackend,
        factory: DriverFactory,
        *,
        clock: Clock = time.monotonic,
        sleep: Sleeper = time.sleep,
    ) -> None:
        self._config = config
        self._gpio = gpio
        self._factory = factory
        self._clock = clock
        self._sleep = sleep
        self._drivers: tuple[ToFDriver, ToFDriver] | None = None
        self._ranging = False
        self._last_error_log = {"forward": -math.inf, "down": -math.inf}
        self.error_count = 0

    def start(self) -> None:
        """Address the pair (on the first start only), apply the config and start ranging."""
        if self._ranging:
            return
        if self._drivers is None:
            self._drivers = assign_tof_addresses(
                self._config, self._gpio, self._factory, self._sleep
            )
        forward, down = self._drivers
        self._start_ranging("forward", forward)
        self._start_ranging("down", down)
        self._ranging = True
        tof = self._config.tof
        log.info(
            "ToF ranging: distance mode %d, timing budget %d ms",
            tof.distance_mode,
            tof.timing_budget_ms,
        )

    def read(self) -> tuple[ToFReading | None, ToFReading | None]:
        """Return ``(forward, down)``; a sensor with no new measurement gives ``None``."""
        if self._drivers is None:
            raise RuntimeError("ToFPair.start() must be called before read()")
        forward, down = self._drivers
        return self._read_sensor("forward", forward), self._read_sensor("down", down)

    def stop(self) -> None:
        """Stop ranging. The addresses stay assigned until the sensors lose power."""
        if self._drivers is None:
            return
        self._ranging = False
        forward, down = self._drivers
        self._stop_ranging("forward", forward)
        self._stop_ranging("down", down)

    def _start_ranging(self, name: str, driver: ToFDriver) -> None:
        tof = self._config.tof
        try:
            with I2C_LOCK:
                # The driver re-applies the old timing budget when the mode changes: mode first.
                driver.distance_mode = tof.distance_mode
                driver.timing_budget = tof.timing_budget_ms
                driver.start_ranging()
        except Exception as exc:
            raise RuntimeError(f"Could not start the {name} ToF sensor: {exc}") from exc

    def _stop_ranging(self, name: str, driver: ToFDriver) -> None:
        try:
            with I2C_LOCK:
                driver.stop_ranging()
        except Exception as exc:
            log.warning("Could not stop the %s ToF sensor: %s", name, exc)

    def _read_sensor(self, name: str, driver: ToFDriver) -> ToFReading | None:
        try:
            with I2C_LOCK:
                if not driver.data_ready:
                    return None
                distance_cm = driver.distance
                driver.clear_interrupt()
        except OSError as exc:
            self._record_error(name, exc)
            return None
        distance_m = None if distance_cm is None else distance_cm / 100.0
        return ToFReading(
            sensor=name,
            distance_m=_valid_distance(distance_m, self._config.tof.max_range_m),
            timestamp=self._clock(),
        )

    def _record_error(self, name: str, exc: OSError) -> None:
        self.error_count += 1
        now = self._clock()
        if now - self._last_error_log[name] >= ERROR_LOG_INTERVAL_S:
            self._last_error_log[name] = now
            log.warning(
                "%s ToF read failed (%d ToF errors so far): %s", name, self.error_count, exc
            )


class SimulatedToFPair:
    """Simulated twin: ``read()`` returns the distances last given to ``set_distances``.

    Both distances start as ``None`` (nothing in range).
    """

    def __init__(self, config: NavSightConfig, *, clock: Clock = time.monotonic) -> None:
        self._max_range_m = config.tof.max_range_m
        self._clock = clock
        self._distances: tuple[float | None, float | None] = (None, None)
        self.error_count = 0

    def set_distances(self, forward_m: float | None, down_m: float | None) -> None:
        self._distances = (forward_m, down_m)

    def start(self) -> None:
        log.info("Simulated ToF pair started")

    def read(self) -> tuple[ToFReading | None, ToFReading | None]:
        forward_m, down_m = self._distances
        now = self._clock()
        return (
            ToFReading("forward", _valid_distance(forward_m, self._max_range_m), now),
            ToFReading("down", _valid_distance(down_m, self._max_range_m), now),
        )

    def stop(self) -> None:
        log.info("Simulated ToF pair stopped")
