from __future__ import annotations

import json
import logging
import math
import sys
import threading
import types
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import Any

import pytest

from navsight.config import NavSightConfig, PowerConfig
from navsight.hardware.i2c import I2C_LOCK
from navsight.hardware.power_monitor import (
    PowerMonitor,
    SimulatedPowerSensor,
    StateOfChargeEstimator,
    ina219_calibration,
    make_ina219_factory,
)
from navsight.types import BatteryStatus
from navsight.util import ManualClock

T0 = 1000.0  # the conftest clock's start time
HOUR_S = 3600.0
USABLE_MAH = 6290.0
HINT = "navsight --battery-full"


def make_estimator(
    path: Path | None = None, *, clock: ManualClock | None = None, **settings: Any
) -> StateOfChargeEstimator:
    return StateOfChargeEstimator(PowerConfig(**settings), path, clock=clock or ManualClock(T0))


def feed(
    estimator: StateOfChargeEstimator,
    current_ma: float,
    start: float,
    duration_s: float,
    step_s: float = 1.0,
) -> float:
    """Send ``current_ma`` every ``step_s`` after ``start`` until ``start + duration_s``."""
    steps = round(duration_s / step_s)
    for index in range(1, steps + 1):
        estimator.update(current_ma, start + index * step_s)
    return start + steps * step_s


def saved_consumed_mah(path: Path) -> float:
    return json.loads(path.read_text(encoding="utf-8"))["consumed_mah"]


def blocked_path(tmp_path: Path) -> Path:
    """A state path that cannot be written: its parent directory is a regular file."""
    blocker = tmp_path / "blocker"
    blocker.write_text("a file, not a directory", encoding="utf-8")
    return blocker / "battery.json"


def i2c_lock_is_free_for_other_threads() -> bool:
    result: list[bool] = []

    def probe() -> None:
        acquired = I2C_LOCK.acquire(blocking=False)
        if acquired:
            I2C_LOCK.release()
        result.append(acquired)

    thread = threading.Thread(target=probe)
    thread.start()
    thread.join()
    return result[0]


class LockCheckingSensor:
    """Records, for every reading, whether another thread could have used the I2C bus."""

    def __init__(self) -> None:
        self.reads: list[tuple[str, bool]] = []

    def _reading(self, name: str, value: float) -> float:
        self.reads.append((name, i2c_lock_is_free_for_other_threads()))
        return value

    @property
    def bus_voltage(self) -> float:
        return self._reading("bus_voltage", 5.05)

    @property
    def shunt_voltage(self) -> float:
        return self._reading("shunt_voltage", 0.05)

    @property
    def current(self) -> float:
        return self._reading("current", 500.0)

    @property
    def power(self) -> float:
        return self._reading("power", 2.525)


class FakeIna219:
    def __init__(self, i2c: Any, addr: int = 0x40) -> None:
        self.i2c = i2c
        self.addr = addr
        self.bus_adc_resolution = 0x03
        self.shunt_adc_resolution = 0x03
        self.calibration = 4096


@pytest.fixture
def fake_ina219_module(monkeypatch: pytest.MonkeyPatch) -> types.ModuleType:
    module = types.ModuleType("adafruit_ina219")
    module.INA219 = FakeIna219
    module.ADCResolution = types.SimpleNamespace(ADCRES_12BIT_128S=0x0F)
    monkeypatch.setitem(sys.modules, "adafruit_ina219", module)
    return module


@pytest.fixture
def estimator(config: NavSightConfig, clock: ManualClock) -> StateOfChargeEstimator:
    return StateOfChargeEstimator(config.power, None, clock=clock)


# StateOfChargeEstimator: capacity and integration


def test_usable_capacity_with_defaults_is_6290_mah() -> None:
    estimator = make_estimator()
    assert estimator.usable_capacity_mah == pytest.approx(USABLE_MAH)
    assert estimator.consumed_mah == 0.0
    assert estimator.state_of_charge == 1.0


def test_usable_capacity_follows_the_config() -> None:
    estimator = make_estimator(
        battery_capacity_mah=20000.0,
        battery_nominal_v=3.6,
        output_v=5.0,
        conversion_efficiency=0.9,
    )
    assert estimator.usable_capacity_mah == pytest.approx(20000.0 * 3.6 / 5.0 * 0.9)


def test_constant_629_ma_for_one_hour_leaves_ninety_percent() -> None:
    estimator = make_estimator()
    assert estimator.update(629.0, T0) == 1.0
    feed(estimator, 629.0, T0, HOUR_S)
    assert estimator.consumed_mah == pytest.approx(629.0)
    assert estimator.state_of_charge == pytest.approx(0.9)


def test_update_returns_the_state_of_charge() -> None:
    estimator = make_estimator()
    estimator.update(629.0, T0)
    end = feed(estimator, 629.0, T0, HOUR_S - 1.0)
    assert estimator.update(629.0, end + 1.0) == pytest.approx(0.9)


def test_trapezoid_rule_averages_the_two_samples() -> None:
    estimator = make_estimator()
    estimator.update(0.0, T0)
    estimator.update(1000.0, T0 + 180.0)
    # A left-rectangle rule would count 0 mAh here and a right-rectangle rule 50 mAh.
    assert estimator.consumed_mah == pytest.approx(500.0 * 180.0 / HOUR_S)


def test_trapezoid_rule_is_exact_for_a_linear_ramp() -> None:
    estimator = make_estimator()
    for minute in range(61):
        estimator.update(1000.0 * minute / 60.0, T0 + 60.0 * minute)
    assert estimator.consumed_mah == pytest.approx(500.0)


def test_negative_current_is_not_counted() -> None:
    estimator = make_estimator()
    estimator.update(-500.0, T0)
    feed(estimator, -500.0, T0, 600.0)
    assert estimator.consumed_mah == 0.0
    assert estimator.state_of_charge == 1.0


def test_negative_current_counts_as_zero_in_the_trapezoid() -> None:
    estimator = make_estimator()
    estimator.update(1000.0, T0)
    estimator.update(-1000.0, T0 + 60.0)
    assert estimator.consumed_mah == pytest.approx(500.0 * 60.0 / HOUR_S)


def test_gap_longer_than_max_integration_gap_is_not_counted(
    caplog: pytest.LogCaptureFixture,
) -> None:
    estimator = make_estimator()  # max_integration_gap_s = 300
    estimator.update(1000.0, T0)
    estimator.update(1000.0, T0 + 300.5)
    assert estimator.consumed_mah == 0.0
    assert "not counted" in caplog.text
    estimator.update(1000.0, T0 + 360.5)  # counting resumes from the sample after the gap
    assert estimator.consumed_mah == pytest.approx(1000.0 * 60.0 / HOUR_S)


def test_gap_of_exactly_max_integration_gap_is_counted() -> None:
    estimator = make_estimator()
    estimator.update(1000.0, T0)
    estimator.update(1000.0, T0 + 300.0)
    assert estimator.consumed_mah == pytest.approx(1000.0 * 300.0 / HOUR_S)


def test_max_integration_gap_comes_from_the_config() -> None:
    estimator = make_estimator(max_integration_gap_s=10.0)
    estimator.update(1000.0, T0)
    estimator.update(1000.0, T0 + 11.0)
    assert estimator.consumed_mah == 0.0
    estimator.update(1000.0, T0 + 21.0)
    assert estimator.consumed_mah == pytest.approx(1000.0 * 10.0 / HOUR_S)


def test_state_of_charge_never_goes_below_zero() -> None:
    estimator = make_estimator(battery_capacity_mah=100.0)  # 62.9 mAh usable
    estimator.update(1000.0, T0)
    feed(estimator, 1000.0, T0, 600.0)
    assert estimator.consumed_mah == pytest.approx(1000.0 / 6.0)
    assert estimator.state_of_charge == 0.0
    assert estimator.remaining_minutes() == 0.0


def test_sample_older_than_the_previous_one_is_ignored() -> None:
    estimator = make_estimator()
    estimator.update(1000.0, T0 + 10.0)
    estimator.update(5000.0, T0 + 5.0)
    assert estimator.consumed_mah == 0.0
    assert estimator.average_current_ma == 1000.0
    estimator.update(1000.0, T0 + 70.0)
    assert estimator.consumed_mah == pytest.approx(1000.0 * 60.0 / HOUR_S)


def test_update_reads_the_clock_when_now_is_omitted() -> None:
    clock = ManualClock(T0)
    estimator = make_estimator(clock=clock)
    estimator.update(629.0)
    for _ in range(3600):
        clock.advance(1.0)
        estimator.update(629.0)
    assert estimator.state_of_charge == pytest.approx(0.9)


@pytest.mark.parametrize(
    "name",
    [
        "battery_capacity_mah",
        "battery_nominal_v",
        "output_v",
        "conversion_efficiency",
        "max_integration_gap_s",
    ],
)
def test_settings_that_must_be_positive_are_checked(name: str) -> None:
    with pytest.raises(ValueError, match=f"power.{name} must be positive"):
        make_estimator(**{name: 0.0})


# StateOfChargeEstimator: average current and remaining time


def test_average_current_is_an_exponential_moving_average() -> None:
    estimator = make_estimator()  # current_ema_s = 60
    estimator.update(600.0, T0)
    assert estimator.average_current_ma == 600.0
    feed(estimator, 1200.0, T0, 60.0)
    assert estimator.average_current_ma == pytest.approx(1200.0 - 600.0 * math.exp(-1.0))


def test_average_current_does_not_depend_on_the_sample_spacing() -> None:
    estimator = make_estimator()
    estimator.update(600.0, T0)
    for offset in (0.5, 2.0, 10.0, 31.0, 60.0):
        estimator.update(1200.0, T0 + offset)
    assert estimator.average_current_ma == pytest.approx(1200.0 - 600.0 * math.exp(-1.0))


def test_average_current_ignores_negative_samples() -> None:
    estimator = make_estimator()
    estimator.update(-300.0, T0)
    assert estimator.average_current_ma == 0.0


def test_zero_time_constant_tracks_the_latest_current() -> None:
    estimator = make_estimator(current_ema_s=0.0)
    estimator.update(600.0, T0)
    estimator.update(1200.0, T0 + 1.0)
    assert estimator.average_current_ma == 1200.0


def test_remaining_minutes_uses_the_average_current() -> None:
    estimator = make_estimator()
    estimator.update(629.0, T0)
    assert estimator.remaining_minutes() == pytest.approx(600.0)  # 6290 mAh at 629 mA = 10 h
    feed(estimator, 629.0, T0, HOUR_S)
    assert estimator.remaining_minutes() == pytest.approx(540.0)


def test_remaining_minutes_is_none_before_any_sample() -> None:
    estimator = make_estimator()
    assert estimator.average_current_ma is None
    assert estimator.remaining_minutes() is None


@pytest.mark.parametrize("current_ma", [0.0, 0.99, -200.0])
def test_remaining_minutes_is_none_below_one_ma(current_ma: float) -> None:
    estimator = make_estimator()
    estimator.update(current_ma, T0)
    assert estimator.remaining_minutes() is None


def test_remaining_minutes_at_exactly_one_ma() -> None:
    estimator = make_estimator()
    estimator.update(1.0, T0)
    assert estimator.remaining_minutes() == pytest.approx(USABLE_MAH * 60.0)


def test_remaining_minutes_becomes_none_when_the_load_stops() -> None:
    estimator = make_estimator()
    estimator.update(500.0, T0)
    feed(estimator, 0.0, T0, 1200.0)  # 20 time constants later the average is far below 1 mA
    assert estimator.remaining_minutes() is None


# StateOfChargeEstimator: persistence


def test_save_and_load_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "battery.json"
    first = make_estimator(path)
    first.update(900.0, T0)
    feed(first, 900.0, T0, 1800.0)
    first.save()

    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["consumed_mah"] == first.consumed_mah
    saved_at = datetime.fromisoformat(data["saved_at_utc"])
    assert saved_at.utcoffset() == timedelta(0)
    assert abs(datetime.now(UTC) - saved_at) < timedelta(minutes=5)

    second = make_estimator(path)
    assert second.consumed_mah == first.consumed_mah
    assert second.state_of_charge == pytest.approx(1.0 - 450.0 / USABLE_MAH)


def test_save_leaves_no_temporary_file(tmp_path: Path) -> None:
    path = tmp_path / "battery.json"
    estimator = make_estimator(path)
    estimator.save()
    estimator.save()
    assert [entry.name for entry in tmp_path.iterdir()] == ["battery.json"]


def test_state_is_saved_every_save_interval(tmp_path: Path) -> None:
    path = tmp_path / "battery.json"
    estimator = make_estimator(path)  # save_interval_s = 60
    estimator.update(1200.0, T0)
    estimator.update(1200.0, T0 + 59.0)
    assert not path.exists()
    estimator.update(1200.0, T0 + 60.0)
    assert saved_consumed_mah(path) == pytest.approx(20.0)  # 1200 mA for 60 s
    estimator.update(1200.0, T0 + 119.0)
    assert saved_consumed_mah(path) == pytest.approx(20.0)
    estimator.update(1200.0, T0 + 120.0)
    assert saved_consumed_mah(path) == pytest.approx(40.0)


def test_save_interval_comes_from_the_config(tmp_path: Path) -> None:
    path = tmp_path / "battery.json"
    estimator = make_estimator(path, save_interval_s=5.0)
    estimator.update(1200.0, T0)
    estimator.update(1200.0, T0 + 4.0)
    assert not path.exists()
    estimator.update(1200.0, T0 + 5.0)
    assert path.exists()


def test_missing_state_file_means_full(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    estimator = make_estimator(tmp_path / "battery.json")
    assert estimator.consumed_mah == 0.0
    assert estimator.state_of_charge == 1.0
    assert HINT in caplog.text


@pytest.mark.parametrize(
    "content",
    [
        b"",
        b"not json",
        b"[12.5]",
        b"{}",
        b'{"consumed_mah": "lots"}',
        b'{"consumed_mah": true}',
        b'{"consumed_mah": -5}',
        b'{"consumed_mah": NaN}',
        b'{"consumed_mah": Infinity}',
        b"\xff\xfe\x00\x01",
    ],
)
def test_corrupt_state_file_means_full(
    tmp_path: Path, caplog: pytest.LogCaptureFixture, content: bytes
) -> None:
    path = tmp_path / "battery.json"
    path.write_bytes(content)
    estimator = make_estimator(path)
    assert estimator.consumed_mah == 0.0
    assert estimator.state_of_charge == 1.0
    assert HINT in caplog.text


def test_unreadable_state_path_means_full(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    path = tmp_path / "battery.json"
    path.mkdir()
    estimator = make_estimator(path)
    assert estimator.state_of_charge == 1.0
    assert HINT in caplog.text


def test_saved_state_beyond_the_capacity_reads_as_empty(tmp_path: Path) -> None:
    path = tmp_path / "battery.json"
    path.write_text(json.dumps({"consumed_mah": 7000}), encoding="utf-8")
    estimator = make_estimator(path)
    assert estimator.consumed_mah == 7000.0
    assert estimator.state_of_charge == 0.0


def test_reset_full_marks_the_bank_full_and_saves(tmp_path: Path) -> None:
    path = tmp_path / "battery.json"
    estimator = make_estimator(path)
    estimator.update(1000.0, T0)
    feed(estimator, 1000.0, T0, 30.0)
    assert estimator.consumed_mah > 0.0

    estimator.reset_full()

    assert estimator.consumed_mah == 0.0
    assert estimator.state_of_charge == 1.0
    assert saved_consumed_mah(path) == 0.0
    estimator.update(1000.0, T0 + 66.0)  # counting carries on from the last sample
    assert estimator.consumed_mah == pytest.approx(1000.0 * 36.0 / HOUR_S)


def test_reset_full_without_a_state_path() -> None:
    estimator = make_estimator()
    estimator.update(1000.0, T0)
    feed(estimator, 1000.0, T0, 30.0)
    estimator.reset_full()
    assert estimator.state_of_charge == 1.0


def test_save_creates_missing_directories(tmp_path: Path) -> None:
    path = tmp_path / "home" / ".navsight" / "battery.json"
    make_estimator(path).save()
    assert saved_consumed_mah(path) == 0.0


def test_state_path_expands_the_home_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("USERPROFILE", str(tmp_path))
    make_estimator(Path("~/battery.json")).save()
    assert (tmp_path / "battery.json").exists()


def test_failed_periodic_save_is_logged_not_raised(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    estimator = make_estimator(blocked_path(tmp_path))
    estimator.update(1000.0, T0)
    estimator.update(1000.0, T0 + 60.0)  # the periodic save fails here
    assert "Could not save the battery state" in caplog.text
    assert estimator.consumed_mah == pytest.approx(1000.0 * 60.0 / HOUR_S)


def test_explicit_save_raises_when_the_file_cannot_be_written(tmp_path: Path) -> None:
    estimator = make_estimator(blocked_path(tmp_path))
    with pytest.raises(OSError):
        estimator.save()


# PowerMonitor


def test_read_returns_a_battery_status(
    config: NavSightConfig, clock: ManualClock, estimator: StateOfChargeEstimator
) -> None:
    sensor = SimulatedPowerSensor()
    monitor = PowerMonitor(config, lambda: sensor, estimator, clock=clock)
    monitor.start()
    status = monitor.read()
    assert isinstance(status, BatteryStatus)
    assert status.timestamp == clock()
    assert status.bus_voltage_v == 5.1
    assert status.current_ma == 900.0
    assert status.power_w == pytest.approx(4.59)
    assert status.state_of_charge == 1.0
    assert status.remaining_minutes == pytest.approx(USABLE_MAH / 900.0 * 60.0)


def test_reads_count_the_charge_drawn(
    config: NavSightConfig, clock: ManualClock, estimator: StateOfChargeEstimator
) -> None:
    sensor = SimulatedPowerSensor(current_ma=629.0)
    monitor = PowerMonitor(config, lambda: sensor, estimator, clock=clock)
    monitor.start()
    status = monitor.read()
    for _ in range(3600):
        clock.advance(1.0)
        status = monitor.read()
    assert status.timestamp == T0 + HOUR_S
    assert status.state_of_charge == pytest.approx(0.9)
    assert status.remaining_minutes == pytest.approx(540.0)

    sensor.set(bus_voltage=4.7, current_ma=0.0)
    clock.advance(1.0)
    status = monitor.read()
    assert (status.bus_voltage_v, status.current_ma, status.power_w) == (4.7, 0.0, 0.0)


def test_read_before_start_raises(
    config: NavSightConfig, clock: ManualClock, estimator: StateOfChargeEstimator
) -> None:
    monitor = PowerMonitor(config, SimulatedPowerSensor, estimator, clock=clock)
    with pytest.raises(RuntimeError, match="needs start"):
        monitor.read()


def test_sensor_is_opened_and_read_while_holding_the_i2c_lock(
    config: NavSightConfig, clock: ManualClock, estimator: StateOfChargeEstimator
) -> None:
    sensor = LockCheckingSensor()
    lock_free_while_opening: list[bool] = []

    def factory() -> LockCheckingSensor:
        lock_free_while_opening.append(i2c_lock_is_free_for_other_threads())
        return sensor

    monitor = PowerMonitor(config, factory, estimator, clock=clock)
    monitor.start()
    status = monitor.read()

    assert lock_free_while_opening == [False]
    assert sensor.reads
    assert not any(lock_free for _, lock_free in sensor.reads)
    assert i2c_lock_is_free_for_other_threads()
    assert (status.bus_voltage_v, status.current_ma, status.power_w) == (5.05, 500.0, 2.525)


@pytest.mark.parametrize(
    "error", [OSError(121, "Remote I/O error"), ValueError("No I2C device at address: 0x40")]
)
def test_start_reports_a_missing_ina219(
    config: NavSightConfig,
    clock: ManualClock,
    estimator: StateOfChargeEstimator,
    error: Exception,
) -> None:
    def factory() -> SimulatedPowerSensor:
        raise error

    monitor = PowerMonitor(config, factory, estimator, clock=clock)
    with pytest.raises(RuntimeError, match="0x40") as excinfo:
        monitor.start()
    assert excinfo.value.__cause__ is error
    assert "i2cdetect -y 1" in str(excinfo.value)


def test_start_twice_opens_the_sensor_once(
    config: NavSightConfig, clock: ManualClock, estimator: StateOfChargeEstimator
) -> None:
    opened: list[SimulatedPowerSensor] = []

    def factory() -> SimulatedPowerSensor:
        opened.append(SimulatedPowerSensor())
        return opened[-1]

    monitor = PowerMonitor(config, factory, estimator, clock=clock)
    monitor.start()
    monitor.start()
    assert len(opened) == 1


def test_stop_saves_the_battery_state_and_releases_the_sensor(
    config: NavSightConfig, clock: ManualClock, tmp_path: Path
) -> None:
    path = tmp_path / "battery.json"
    estimator = StateOfChargeEstimator(config.power, path, clock=clock)
    sensor = SimulatedPowerSensor(current_ma=1800.0)
    monitor = PowerMonitor(config, lambda: sensor, estimator, clock=clock)
    monitor.start()
    monitor.read()
    clock.advance(10.0)
    monitor.read()

    monitor.stop()

    assert saved_consumed_mah(path) == pytest.approx(5.0)  # 1800 mA for 10 s
    with pytest.raises(RuntimeError):
        monitor.read()


def test_stop_logs_a_failed_save(
    config: NavSightConfig,
    clock: ManualClock,
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
) -> None:
    estimator = StateOfChargeEstimator(config.power, blocked_path(tmp_path), clock=clock)
    monitor = PowerMonitor(config, SimulatedPowerSensor, estimator, clock=clock)
    monitor.start()
    monitor.stop()
    errors = [record for record in caplog.records if record.levelno == logging.ERROR]
    assert len(errors) == 1
    assert "battery state" in errors[0].getMessage()


def test_swapped_shunt_wiring_is_reported_once(
    config: NavSightConfig,
    clock: ManualClock,
    estimator: StateOfChargeEstimator,
    caplog: pytest.LogCaptureFixture,
) -> None:
    sensor = SimulatedPowerSensor(current_ma=-450.0)
    monitor = PowerMonitor(config, lambda: sensor, estimator, clock=clock)
    monitor.start()
    for _ in range(3):
        status = monitor.read()
        clock.advance(1.0)
    warnings = [record for record in caplog.records if "swapped" in record.getMessage()]
    assert len(warnings) == 1
    assert status.current_ma == -450.0
    assert status.state_of_charge == 1.0


def test_small_negative_noise_is_not_reported(
    config: NavSightConfig,
    clock: ManualClock,
    estimator: StateOfChargeEstimator,
    caplog: pytest.LogCaptureFixture,
) -> None:
    monitor = PowerMonitor(
        config, lambda: SimulatedPowerSensor(current_ma=-0.3), estimator, clock=clock
    )
    monitor.start()
    monitor.read()
    assert "swapped" not in caplog.text


# SimulatedPowerSensor and the INA219 factory


def test_simulated_sensor_behaves_like_an_ina219_with_a_0_1_ohm_shunt() -> None:
    sensor = SimulatedPowerSensor()
    assert sensor.bus_voltage == 5.1
    assert sensor.current == 900.0
    assert sensor.shunt_voltage == pytest.approx(0.09)
    assert sensor.power == pytest.approx(4.59)

    sensor.set(current_ma=500.0)
    assert (sensor.bus_voltage, sensor.current) == (5.1, 500.0)
    sensor.set(bus_voltage=4.8)
    assert (sensor.bus_voltage, sensor.current) == (4.8, 500.0)
    assert sensor.power == pytest.approx(2.4)
    assert sensor.shunt_voltage == pytest.approx(0.05)


@pytest.mark.parametrize(
    "shunt_ohms, expected", [(0.1, 4096), (0.05, 8192), (0.02, 20480), (0.01, 40960)]
)
def test_ina219_calibration_keeps_the_0_1_ma_current_lsb(shunt_ohms: float, expected: int) -> None:
    assert ina219_calibration(shunt_ohms) == expected


@pytest.mark.parametrize("shunt_ohms", [0.0, -0.1, 0.001])
def test_ina219_calibration_rejects_impossible_shunts(shunt_ohms: float) -> None:
    with pytest.raises(ValueError, match="shunt"):
        ina219_calibration(shunt_ohms)


def test_ina219_factory_opens_the_driver_with_averaging(
    fake_ina219_module: types.ModuleType,
) -> None:
    sensor = make_ina219_factory("i2c-bus", 0x40)()
    assert isinstance(sensor, FakeIna219)
    assert (sensor.i2c, sensor.addr) == ("i2c-bus", 0x40)
    assert sensor.bus_adc_resolution == 0x0F
    assert sensor.shunt_adc_resolution == 0x0F
    assert sensor.calibration == 4096


def test_ina219_factory_recalibrates_for_another_shunt(
    fake_ina219_module: types.ModuleType,
) -> None:
    sensor = make_ina219_factory("i2c-bus", 0x41, shunt_ohms=0.05)()
    assert isinstance(sensor, FakeIna219)
    assert sensor.addr == 0x41
    assert sensor.calibration == 8192


def test_ina219_factory_rejects_a_bad_shunt_at_once() -> None:
    with pytest.raises(ValueError, match="shunt"):
        make_ina219_factory("i2c-bus", 0x40, shunt_ohms=0.0)


def test_ina219_factory_imports_the_driver_only_when_called(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setitem(sys.modules, "adafruit_ina219", None)  # any import of it now fails
    factory = make_ina219_factory("i2c-bus", 0x40)
    with pytest.raises(ImportError):
        factory()
