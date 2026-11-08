"""INA219 power monitor and coulomb-counting state of charge for the USB power bank.

The INA219 measures the bank's 5 V output, which stays near 5.1 V until the bank cuts out, so the
voltage says almost nothing about the charge left. The charge is counted instead: the current
drawn is integrated over time and the running total is saved across restarts.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from collections.abc import Callable
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Protocol

from navsight.config import NavSightConfig, PowerConfig
from navsight.hardware.i2c import I2C_LOCK
from navsight.types import BatteryStatus
from navsight.util import Clock, clamp

log = logging.getLogger(__name__)

FULL_CHARGE_HINT = "Run 'navsight --battery-full' after charging the power bank."
MIN_ESTIMATE_CURRENT_MA = 1.0  # below this average current there is no runtime estimate
REVERSED_CURRENT_WARN_MA = -50.0  # the Pi always draws current, so this means swapped wiring
INA219_DEFAULT_SHUNT_OHMS = 0.1  # the breakout's shunt, which adafruit_ina219 assumes
INA219_CURRENT_LSB_A = 0.0001  # adafruit_ina219's 32 V / 2 A calibration: 0.1 mA per bit
_SECONDS_PER_HOUR = 3600.0
_POSITIVE_SETTINGS = (
    "battery_capacity_mah",
    "battery_nominal_v",
    "output_v",
    "conversion_efficiency",
    "max_integration_gap_s",
)


class PowerSensor(Protocol):
    """The subset of ``adafruit_ina219.INA219`` used here."""

    @property
    def bus_voltage(self) -> float: ...  # V, from VIN- to GND

    @property
    def shunt_voltage(self) -> float: ...  # V, across the shunt

    @property
    def current(self) -> float: ...  # mA

    @property
    def power(self) -> float: ...  # W


class StateOfChargeEstimator:
    """Coulomb counter for the power bank.

    ``usable_capacity_mah`` is the charge the bank delivers at its output voltage:
    ``capacity_mah * nominal_v / output_v * efficiency`` (6290 mAh at 5 V by default). The consumed
    charge is saved as JSON at ``state_path`` every ``save_interval_s``; without a readable file
    the bank is assumed to be full.
    """

    def __init__(
        self,
        config: PowerConfig,
        state_path: Path | None,
        *,
        clock: Clock = time.monotonic,
    ) -> None:
        _check_power_config(config)
        self._config = config
        self._usable_mah = (
            config.battery_capacity_mah
            * config.battery_nominal_v
            / config.output_v
            * config.conversion_efficiency
        )
        self._state_path = None if state_path is None else Path(state_path).expanduser()
        self._clock = clock
        self._lock = threading.RLock()
        self._last_time: float | None = None
        self._last_current_ma = 0.0
        self._average_current_ma = 0.0
        self._last_save_time: float | None = None
        self._consumed_mah = self._load()

    @property
    def usable_capacity_mah(self) -> float:
        return self._usable_mah

    @property
    def consumed_mah(self) -> float:
        return self._consumed_mah

    @property
    def state_of_charge(self) -> float:
        return clamp(1.0 - self._consumed_mah / self._usable_mah, 0.0, 1.0)

    @property
    def average_current_ma(self) -> float | None:
        """Exponential moving average of the current drawn; None before the first sample."""
        return None if self._last_time is None else self._average_current_ma

    def update(self, current_ma: float, now: float | None = None) -> float:
        """Count the charge drawn since the previous sample and return the state of charge.

        Negative current counts as zero. An interval longer than ``max_integration_gap_s`` is
        not counted, and a sample older than the previous one is ignored.
        """
        if now is None:
            now = self._clock()
        current_ma = max(0.0, current_ma)
        with self._lock:
            if self._last_time is not None and now < self._last_time:
                log.warning("Ignoring a current sample older than the previous one")
                return self.state_of_charge
            if self._last_time is None:
                self._average_current_ma = current_ma
                self._last_save_time = now
            else:
                elapsed_s = now - self._last_time
                self._integrate(current_ma, elapsed_s)
                self._smooth(current_ma, elapsed_s)
            self._last_time = now
            self._last_current_ma = current_ma
            self._save_if_due(now)
            return self.state_of_charge

    def remaining_minutes(self) -> float | None:
        """Minutes until empty at the average current; None below 1 mA or before any sample."""
        average_ma = self.average_current_ma
        if average_ma is None or average_ma < MIN_ESTIMATE_CURRENT_MA:
            return None
        remaining_mah = max(0.0, self._usable_mah - self._consumed_mah)
        return remaining_mah / average_ma * 60.0

    def reset_full(self) -> None:
        """Mark the bank as fully charged and save that at once (``OSError`` if saving fails)."""
        with self._lock:
            self._consumed_mah = 0.0
            self.save()
        log.info("Battery marked as full")

    def save(self) -> None:
        """Write the consumed charge to ``state_path``; does nothing without a path.

        Raises ``OSError`` when the file cannot be written.
        """
        if self._state_path is None:
            return
        with self._lock:
            state = {
                "consumed_mah": self._consumed_mah,
                "saved_at_utc": datetime.now(UTC).isoformat(timespec="seconds"),
            }
            _write_json_atomically(self._state_path, state)

    def _integrate(self, current_ma: float, elapsed_s: float) -> None:
        if elapsed_s > self._config.max_integration_gap_s:
            log.warning("No current sample for %.0f s; that interval is not counted", elapsed_s)
            return
        mean_ma = (self._last_current_ma + current_ma) / 2.0
        self._consumed_mah += mean_ma * elapsed_s / _SECONDS_PER_HOUR

    def _smooth(self, current_ma: float, elapsed_s: float) -> None:
        time_constant_s = self._config.current_ema_s
        weight = 1.0 if time_constant_s <= 0 else 1.0 - math.exp(-elapsed_s / time_constant_s)
        self._average_current_ma += weight * (current_ma - self._average_current_ma)

    def _save_if_due(self, now: float) -> None:
        if self._state_path is None or self._last_save_time is None:
            return
        if now - self._last_save_time < self._config.save_interval_s:
            return
        self._last_save_time = now
        try:
            self.save()
        except OSError as exc:
            log.warning("Could not save the battery state to %s: %s", self._state_path, exc)

    def _load(self) -> float:
        path = self._state_path
        if path is None:
            return 0.0
        try:
            consumed_mah = _parse_consumed_mah(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            log.warning(
                "No saved battery state at %s; assuming the power bank is full. %s",
                path,
                FULL_CHARGE_HINT,
            )
            return 0.0
        except (OSError, ValueError) as exc:
            log.warning(
                "Could not read the battery state at %s (%s); assuming the power bank is full. %s",
                path,
                exc,
                FULL_CHARGE_HINT,
            )
            return 0.0
        log.info("Loaded the battery state from %s: %.0f mAh used", path, consumed_mah)
        return consumed_mah


class PowerMonitor:
    """Reads the INA219 and feeds every current sample to the state-of-charge estimator."""

    def __init__(
        self,
        config: NavSightConfig,
        sensor_factory: Callable[[], PowerSensor],
        estimator: StateOfChargeEstimator,
        *,
        clock: Clock = time.monotonic,
    ) -> None:
        self._address = config.i2c.power_monitor
        self._i2c_bus = config.pins.i2c_bus
        self._sensor_factory = sensor_factory
        self._estimator = estimator
        self._clock = clock
        self._sensor: PowerSensor | None = None
        self._warned_reversed = False

    def start(self) -> None:
        if self._sensor is not None:
            return
        try:
            with I2C_LOCK:
                self._sensor = self._sensor_factory()
        except (OSError, ValueError) as exc:
            raise RuntimeError(
                f"The INA219 power monitor did not answer at I2C address 0x{self._address:02X} "
                f"({exc}). Check its VCC (3.3 V), GND, SDA (GPIO 2) and SCL (GPIO 3) wiring; "
                f"'i2cdetect -y {self._i2c_bus}' should list {self._address:02x}."
            ) from exc
        log.info("INA219 power monitor started at 0x%02X", self._address)

    def read(self) -> BatteryStatus:
        sensor = self._sensor
        if sensor is None:
            raise RuntimeError("PowerMonitor.read() needs start() first (or again after stop())")
        with I2C_LOCK:
            bus_voltage_v = sensor.bus_voltage
            current_ma = sensor.current
            power_w = sensor.power
        now = self._clock()
        self._warn_if_reversed(current_ma)
        state_of_charge = self._estimator.update(current_ma, now)
        return BatteryStatus(
            timestamp=now,
            bus_voltage_v=bus_voltage_v,
            current_ma=current_ma,
            power_w=power_w,
            state_of_charge=state_of_charge,
            remaining_minutes=self._estimator.remaining_minutes(),
        )

    def stop(self) -> None:
        try:
            self._estimator.save()
        except OSError as exc:
            log.error("Could not save the battery state on shutdown: %s", exc)
        self._sensor = None

    def _warn_if_reversed(self, current_ma: float) -> None:
        if current_ma >= REVERSED_CURRENT_WARN_MA or self._warned_reversed:
            return
        self._warned_reversed = True
        log.warning(
            "The INA219 reads %.0f mA: VIN+ and VIN- look swapped (VIN+ goes to the power bank, "
            "VIN- to the Pi). Negative current is not counted, so the battery level cannot drop.",
            current_ma,
        )


def ina219_calibration(shunt_ohms: float) -> int:
    """INA219 calibration register value that keeps the 0.1 mA current LSB with this shunt."""
    if shunt_ohms <= 0:
        raise ValueError(f"shunt_ohms must be positive, got {shunt_ohms}")
    value = round(0.04096 / (INA219_CURRENT_LSB_A * shunt_ohms))  # datasheet equation 1
    if not 1 <= value <= 0xFFFE:
        raise ValueError(f"A {shunt_ohms} ohm shunt is outside the INA219 calibration range")
    return value


def make_ina219_factory(
    i2c: Any, address: int, *, shunt_ohms: float = INA219_DEFAULT_SHUNT_OHMS
) -> Callable[[], PowerSensor]:
    """Return a factory that opens the INA219 with adafruit_ina219, imported when it is called.

    Both ADCs average 128 samples (68 ms) per reading: the Pi's current changes much faster than
    the 1 Hz reads, and coulomb counting needs averages rather than snapshots. A shunt other than
    the breakout's 0.1 ohm gets a matching calibration; the current still saturates near 3.2 A.
    """
    calibration = ina219_calibration(shunt_ohms)

    def factory() -> PowerSensor:
        from adafruit_ina219 import INA219, ADCResolution  # only available on the Pi

        sensor = INA219(i2c, addr=address)
        sensor.bus_adc_resolution = ADCResolution.ADCRES_12BIT_128S
        sensor.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_128S
        if shunt_ohms != INA219_DEFAULT_SHUNT_OHMS:
            sensor.calibration = calibration
        return sensor

    return factory


class SimulatedPowerSensor:
    """An INA219 stand-in with the breakout's 0.1 ohm shunt, for the simulator and tests."""

    def __init__(self, bus_voltage: float = 5.1, current_ma: float = 900.0) -> None:
        self._bus_voltage = bus_voltage
        self._current_ma = current_ma

    def set(self, *, bus_voltage: float | None = None, current_ma: float | None = None) -> None:
        if bus_voltage is not None:
            self._bus_voltage = bus_voltage
        if current_ma is not None:
            self._current_ma = current_ma

    @property
    def bus_voltage(self) -> float:
        return self._bus_voltage

    @property
    def shunt_voltage(self) -> float:
        return self._current_ma / 1000.0 * INA219_DEFAULT_SHUNT_OHMS

    @property
    def current(self) -> float:
        return self._current_ma

    @property
    def power(self) -> float:
        return self._bus_voltage * self._current_ma / 1000.0


def _check_power_config(config: PowerConfig) -> None:
    for name in _POSITIVE_SETTINGS:
        value = getattr(config, name)
        if value <= 0:
            raise ValueError(f"power.{name} must be positive, got {value}")


def _parse_consumed_mah(text: str) -> float:
    data = json.loads(text)
    value = data.get("consumed_mah") if isinstance(data, dict) else None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"consumed_mah must be a number, got {value!r}")
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"consumed_mah must be a finite number >= 0, got {value!r}")
    return float(value)


def _write_json_atomically(path: Path, data: dict[str, Any]) -> None:
    """Write through a temporary file and a rename, so a power cut never leaves half a file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f"{path.name}.tmp")
    with temporary.open("w", encoding="utf-8") as file:
        json.dump(data, file)
        file.flush()
        os.fsync(file.fileno())
    temporary.replace(path)
