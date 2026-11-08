from __future__ import annotations

import logging
import sys
import threading
import types

import pytest

from navsight.config import NavSightConfig
from navsight.hardware.gpio import FakeGpio
from navsight.hardware.i2c import I2C_LOCK
from navsight.hardware.tof import (
    DEFAULT_ADDRESS,
    SimulatedToFPair,
    ToFPair,
    assign_tof_addresses,
    make_vl53l1x_factory,
)
from navsight.types import ToFReading
from navsight.util import ManualClock

XSHUT = 25
FRESH_BOOT_EVENTS = [
    "sleep 0.01 xshut=0",
    "open 0x29",
    "move forward 0x29->0x31",
    "sleep 0.01 xshut=1",
    "open 0x29",
    "move down 0x29->0x30",
    "move forward 0x31->0x29",
]


def i2c_lock_held() -> bool:
    """True when I2C_LOCK is held, i.e. another thread cannot take it."""
    taken: list[bool] = []

    def probe() -> None:
        got = I2C_LOCK.acquire(blocking=False)
        taken.append(got)
        if got:
            I2C_LOCK.release()

    thread = threading.Thread(target=probe)
    thread.start()
    thread.join()
    return not taken[0]


class FakeSensor:
    def __init__(self, name: str, address: int) -> None:
        self.name = name
        self.address = address
        self.powered = True
        self.distance_cm: float | None = 150.0
        self.data_ready = True
        self.distance_mode: int | None = None
        self.timing_budget: int | None = None
        self.ranging = False
        self.interrupts_cleared = 0
        self.config_calls: list[str] = []
        self.failing: set[str] = set()  # driver operations that raise error_type
        self.error_type: type[Exception] = OSError


class FakeBus:
    """Two VL53L1X on one I2C bus.

    The down sensor has power only while XSHUT is high (an unclaimed pin counts as high, like
    the breakout's pull-up) and losing power resets its address to 0x29. With
    ``xshut_wired=False`` it ignores XSHUT and is always powered.
    """

    def __init__(
        self,
        gpio: FakeGpio,
        *,
        forward_address: int = DEFAULT_ADDRESS,
        down_address: int = DEFAULT_ADDRESS,
        forward_present: bool = True,
        down_present: bool = True,
        xshut_wired: bool = True,
    ) -> None:
        self.gpio = gpio
        self.forward = FakeSensor("forward", forward_address) if forward_present else None
        self.down = FakeSensor("down", down_address) if down_present else None
        self.xshut_wired = xshut_wired
        self.events: list[str] = []
        self.unlocked_calls: list[str] = []
        self._writes_seen = 0

    def factory(self, address: int) -> FakeDriver:
        self.check_lock(f"open {address:#04x}")
        self.events.append(f"open {address:#04x}")
        self.answering(address)
        return FakeDriver(self, address)

    def sleep(self, seconds: float) -> None:
        self.events.append(f"sleep {seconds} xshut={self.gpio.outputs.get(XSHUT)}")

    def check_lock(self, operation: str) -> None:
        if not i2c_lock_held():
            self.unlocked_calls.append(operation)

    def answering(self, address: int) -> FakeSensor:
        self._update_power()
        sensors = [s for s in (self.forward, self.down) if s is not None]
        found = [s for s in sensors if s.powered and s.address == address]
        if not found:
            raise OSError(121, f"Remote I/O error at {address:#04x}")
        if len(found) > 1:
            raise RuntimeError(f"bus conflict: {len(found)} devices answer at {address:#04x}")
        return found[0]

    def _update_power(self) -> None:
        new_writes = self.gpio.writes[self._writes_seen :]
        self._writes_seen = len(self.gpio.writes)
        if self.down is None or not self.xshut_wired:
            return
        if any(pin == XSHUT and level == 0 for pin, level in new_writes):
            self.down.address = DEFAULT_ADDRESS
        self.down.powered = self.gpio.outputs.get(XSHUT, 1) == 1


class FakeDriver:
    """Talks to whichever fake sensor answers at its current address, like the real driver."""

    def __init__(self, bus: FakeBus, address: int) -> None:
        self._bus = bus
        self.address = address

    def _sensor(self, operation: str) -> FakeSensor:
        self._bus.check_lock(operation)
        sensor = self._bus.answering(self.address)
        if operation in sensor.failing:
            raise sensor.error_type(f"I/O error during {operation}")
        return sensor

    @property
    def distance_mode(self) -> int | None:
        return self._sensor("distance_mode").distance_mode

    @distance_mode.setter
    def distance_mode(self, value: int) -> None:
        sensor = self._sensor("distance_mode")
        sensor.distance_mode = value
        sensor.config_calls.append(f"distance_mode={value}")

    @property
    def timing_budget(self) -> int | None:
        return self._sensor("timing_budget").timing_budget

    @timing_budget.setter
    def timing_budget(self, value: int) -> None:
        sensor = self._sensor("timing_budget")
        sensor.timing_budget = value
        sensor.config_calls.append(f"timing_budget={value}")

    @property
    def data_ready(self) -> bool:
        return self._sensor("data_ready").data_ready

    @property
    def distance(self) -> float | None:
        return self._sensor("distance").distance_cm

    def start_ranging(self) -> None:
        sensor = self._sensor("start_ranging")
        sensor.ranging = True
        sensor.config_calls.append("start_ranging")

    def stop_ranging(self) -> None:
        self._sensor("stop_ranging").ranging = False

    def clear_interrupt(self) -> None:
        self._sensor("clear_interrupt").interrupts_cleared += 1

    def set_address(self, new_address: int) -> None:
        sensor = self._sensor("set_address")
        self._bus.events.append(f"move {sensor.name} {self.address:#04x}->{new_address:#04x}")
        sensor.address = new_address
        self.address = new_address


class BusyGpio(FakeGpio):
    def claim_output(self, pin: int, level: int = 0) -> None:
        raise RuntimeError(f"GPIO {pin} is busy")


@pytest.fixture
def bus(gpio: FakeGpio) -> FakeBus:
    return FakeBus(gpio)


def assign(config: NavSightConfig, gpio: FakeGpio, bus: FakeBus) -> tuple:
    return assign_tof_addresses(config, gpio, bus.factory, sleep=bus.sleep)


def started_pair(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> ToFPair:
    pair = ToFPair(config, gpio, bus.factory, clock=clock, sleep=bus.sleep)
    pair.start()
    return pair


def test_lock_probe_detects_unlocked_driver_calls(gpio: FakeGpio) -> None:
    bus = FakeBus(gpio, down_present=False)

    bus.factory(DEFAULT_ADDRESS)
    assert bus.unlocked_calls == ["open 0x29"]
    with I2C_LOCK:
        bus.factory(DEFAULT_ADDRESS)
    assert bus.unlocked_calls == ["open 0x29"]


def test_fresh_boot_gives_forward_0x29_and_down_0x30(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus
) -> None:
    forward, down = assign(config, gpio, bus)

    assert (bus.forward.address, bus.down.address) == (0x29, 0x30)
    assert (forward.address, down.address) == (0x29, 0x30)
    assert bus.answering(0x29) is bus.forward
    assert bus.answering(0x30) is bus.down
    assert bus.events == FRESH_BOOT_EVENTS
    assert gpio.writes == [(XSHUT, 0), (XSHUT, 1)]
    assert gpio.outputs[XSHUT] == 1
    assert bus.unlocked_calls == []


def test_restart_with_forward_parked_at_temporary_address(
    config: NavSightConfig, gpio: FakeGpio
) -> None:
    bus = FakeBus(gpio, forward_address=0x31, down_address=0x30)

    forward, down = assign(config, gpio, bus)

    assert (bus.forward.address, bus.down.address) == (0x29, 0x30)
    assert (forward.address, down.address) == (0x29, 0x30)
    assert bus.events == [
        "sleep 0.01 xshut=0",
        "open 0x29",
        "open 0x31",
        "sleep 0.01 xshut=1",
        "open 0x29",
        "move down 0x29->0x30",
        "move forward 0x31->0x29",
    ]
    assert bus.unlocked_calls == []


def test_clean_restart_resets_the_down_sensor_first(config: NavSightConfig, gpio: FakeGpio) -> None:
    bus = FakeBus(gpio, forward_address=0x29, down_address=0x30)

    assign(config, gpio, bus)

    assert (bus.forward.address, bus.down.address) == (0x29, 0x30)
    assert bus.events == FRESH_BOOT_EVENTS


@pytest.mark.parametrize("forward_address", [0x29, 0x31, 0x30])
def test_forward_sensor_is_found_wherever_a_crash_left_it(
    config: NavSightConfig, gpio: FakeGpio, forward_address: int
) -> None:
    bus = FakeBus(gpio, forward_address=forward_address, down_address=0x31)

    assign(config, gpio, bus)

    assert (bus.forward.address, bus.down.address) == (0x29, 0x30)
    assert bus.unlocked_calls == []


def test_forward_search_order_is_deduplicated(config: NavSightConfig, gpio: FakeGpio) -> None:
    bus = FakeBus(gpio, forward_present=False)

    with pytest.raises(RuntimeError, match="step 2") as excinfo:
        assign(config, gpio, bus)

    opens = [event for event in bus.events if event.startswith("open")]
    assert opens == ["open 0x29", "open 0x31", "open 0x30"]
    message = str(excinfo.value)
    assert "forward sensor" in message
    assert "0x29, 0x31, 0x30" in message


def test_forward_search_tries_tof_forward_before_tof_down(
    config: NavSightConfig, gpio: FakeGpio
) -> None:
    config.i2c.tof_forward = 0x2A
    bus = FakeBus(gpio, forward_present=False)

    with pytest.raises(RuntimeError, match="step 2"):
        assign(config, gpio, bus)

    opens = [event for event in bus.events if event.startswith("open")]
    assert opens == ["open 0x29", "open 0x31", "open 0x2a", "open 0x30"]


def test_custom_forward_address_is_found_and_restored(
    config: NavSightConfig, gpio: FakeGpio
) -> None:
    config.i2c.tof_forward = 0x2A
    bus = FakeBus(gpio, forward_address=0x2A, down_address=0x30)

    forward, down = assign(config, gpio, bus)

    assert (bus.forward.address, bus.down.address) == (0x2A, 0x30)
    assert (forward.address, down.address) == (0x2A, 0x30)
    assert "move forward 0x2a->0x31" in bus.events


def test_missing_down_sensor_names_step_5_and_the_xshut_pin(
    config: NavSightConfig, gpio: FakeGpio
) -> None:
    bus = FakeBus(gpio, down_present=False)

    with pytest.raises(RuntimeError, match="step 5") as excinfo:
        assign(config, gpio, bus)

    message = str(excinfo.value)
    assert "XSHUT" in message
    assert "GPIO 25" in message
    assert isinstance(excinfo.value.__cause__, OSError)


def test_unwired_xshut_shows_up_as_two_sensors_at_0x29(
    config: NavSightConfig, gpio: FakeGpio
) -> None:
    bus = FakeBus(gpio, xshut_wired=False)

    with pytest.raises(RuntimeError, match="step 2") as excinfo:
        assign(config, gpio, bus)

    message = str(excinfo.value)
    assert "bus conflict" in message
    assert "XSHUT" in message
    assert "GPIO 25" in message


def test_xshut_pin_comes_from_config(config: NavSightConfig) -> None:
    config.pins.tof_down_xshut = 24
    gpio = FakeGpio()
    bus = FakeBus(gpio, down_present=False)

    with pytest.raises(RuntimeError, match="GPIO 24"):
        assign(config, gpio, bus)
    assert gpio.writes == [(24, 0), (24, 1)]


@pytest.mark.parametrize("forward_address, failed_step", [(0x29, "step 3"), (0x31, "step 6")])
def test_failed_address_change_names_the_step(
    config: NavSightConfig, gpio: FakeGpio, forward_address: int, failed_step: str
) -> None:
    bus = FakeBus(gpio, forward_address=forward_address)
    bus.forward.failing.add("set_address")

    with pytest.raises(RuntimeError, match=failed_step) as excinfo:
        assign(config, gpio, bus)

    assert "forward sensor" in str(excinfo.value)
    assert isinstance(excinfo.value.__cause__, OSError)


def test_gpio_claim_failure_names_step_1(config: NavSightConfig) -> None:
    gpio = BusyGpio()
    bus = FakeBus(gpio)

    with pytest.raises(RuntimeError, match="step 1") as excinfo:
        assign(config, gpio, bus)

    assert "GPIO 25" in str(excinfo.value)
    assert bus.events == []


def test_start_applies_distance_mode_then_timing_budget(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> None:
    config.tof.distance_mode = 1
    config.tof.timing_budget_ms = 33

    started_pair(config, gpio, bus, clock)

    for sensor in (bus.forward, bus.down):
        assert sensor.distance_mode == 1
        assert sensor.timing_budget == 33
        assert sensor.ranging
        assert sensor.config_calls == ["distance_mode=1", "timing_budget=33", "start_ranging"]
    assert (bus.forward.address, bus.down.address) == (0x29, 0x30)
    assert bus.unlocked_calls == []


def test_start_twice_does_nothing_more(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> None:
    pair = started_pair(config, gpio, bus, clock)
    events = list(bus.events)

    pair.start()

    assert bus.events == events
    assert bus.forward.config_calls == ["distance_mode=2", "timing_budget=20", "start_ranging"]


def test_stop_then_start_restarts_ranging_without_readdressing(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> None:
    pair = started_pair(config, gpio, bus, clock)
    events = list(bus.events)

    pair.stop()
    assert not bus.forward.ranging
    assert not bus.down.ranging

    pair.start()
    assert bus.forward.ranging
    assert bus.down.ranging
    assert bus.events == events
    assert bus.unlocked_calls == []


def test_failed_start_is_retried_without_readdressing(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> None:
    bus.down.failing.add("timing_budget")
    pair = ToFPair(config, gpio, bus.factory, clock=clock, sleep=bus.sleep)

    with pytest.raises(RuntimeError, match="down ToF sensor"):
        pair.start()

    events = list(bus.events)
    bus.down.failing.clear()
    pair.start()
    assert bus.down.ranging
    assert bus.events == events


def test_failed_addressing_propagates_from_start(
    config: NavSightConfig, gpio: FakeGpio, clock: ManualClock
) -> None:
    bus = FakeBus(gpio, down_present=False)
    pair = ToFPair(config, gpio, bus.factory, clock=clock, sleep=bus.sleep)

    with pytest.raises(RuntimeError, match="XSHUT"):
        pair.start()
    with pytest.raises(RuntimeError, match="start"):
        pair.read()


def test_read_before_start_raises(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> None:
    pair = ToFPair(config, gpio, bus.factory, clock=clock, sleep=bus.sleep)

    with pytest.raises(RuntimeError, match="start"):
        pair.read()
    pair.stop()  # stopping a pair that never started is harmless


def test_read_converts_centimetres_to_metres(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> None:
    pair = started_pair(config, gpio, bus, clock)
    bus.forward.distance_cm = 123.4
    bus.down.distance_cm = 45.0
    clock.advance(0.5)

    forward, down = pair.read()

    assert forward == ToFReading("forward", pytest.approx(1.234), 1000.5)
    assert down == ToFReading("down", pytest.approx(0.45), 1000.5)
    assert bus.forward.interrupts_cleared == 1
    assert bus.down.interrupts_cleared == 1
    assert pair.error_count == 0
    assert bus.unlocked_calls == []


@pytest.mark.parametrize("distance_cm", [None, 0.0, -3.0, 400.1, 819.0])
def test_read_maps_invalid_distances_to_none(
    config: NavSightConfig,
    gpio: FakeGpio,
    bus: FakeBus,
    clock: ManualClock,
    distance_cm: float | None,
) -> None:
    pair = started_pair(config, gpio, bus, clock)
    bus.forward.distance_cm = distance_cm

    forward, down = pair.read()

    assert forward == ToFReading("forward", None, 1000.0)
    assert down == ToFReading("down", pytest.approx(1.5), 1000.0)
    assert bus.forward.interrupts_cleared == 1


def test_max_range_is_inclusive_and_comes_from_config(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> None:
    config.tof.max_range_m = 1.3
    pair = started_pair(config, gpio, bus, clock)
    bus.forward.distance_cm = 130.0
    bus.down.distance_cm = 130.1

    forward, down = pair.read()

    assert forward.distance_m == pytest.approx(1.3)
    assert down.distance_m is None


def test_sensor_without_new_data_gives_none(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> None:
    pair = started_pair(config, gpio, bus, clock)
    bus.down.data_ready = False

    forward, down = pair.read()

    assert forward is not None
    assert down is None
    assert bus.down.interrupts_cleared == 0
    assert pair.error_count == 0


@pytest.mark.parametrize("operation", ["data_ready", "distance", "clear_interrupt"])
def test_os_error_from_one_sensor_is_counted_and_isolated(
    config: NavSightConfig,
    gpio: FakeGpio,
    bus: FakeBus,
    clock: ManualClock,
    operation: str,
) -> None:
    pair = started_pair(config, gpio, bus, clock)
    bus.forward.failing.add(operation)

    forward, down = pair.read()
    assert forward is None
    assert down == ToFReading("down", pytest.approx(1.5), 1000.0)
    assert pair.error_count == 1

    bus.forward.failing.clear()
    forward, down = pair.read()
    assert forward is not None
    assert down is not None
    assert pair.error_count == 1


def test_both_sensors_failing_count_two_errors_per_read(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> None:
    pair = started_pair(config, gpio, bus, clock)
    bus.forward.failing.add("distance")
    bus.down.failing.add("data_ready")

    assert pair.read() == (None, None)
    assert pair.read() == (None, None)
    assert pair.error_count == 4


def test_read_errors_are_logged_at_most_every_five_seconds_per_sensor(
    config: NavSightConfig,
    gpio: FakeGpio,
    bus: FakeBus,
    clock: ManualClock,
    caplog: pytest.LogCaptureFixture,
) -> None:
    caplog.set_level(logging.WARNING, logger="navsight.hardware.tof")
    pair = started_pair(config, gpio, bus, clock)
    bus.forward.failing.add("distance")

    pair.read()
    clock.advance(4.5)
    pair.read()
    assert len(caplog.records) == 1
    assert "forward ToF read failed" in caplog.records[0].getMessage()

    clock.advance(0.5)
    pair.read()
    assert len(caplog.records) == 2
    assert "3 ToF errors" in caplog.records[1].getMessage()
    assert pair.error_count == 3


def test_non_os_errors_propagate_from_read(
    config: NavSightConfig, gpio: FakeGpio, bus: FakeBus, clock: ManualClock
) -> None:
    pair = started_pair(config, gpio, bus, clock)
    bus.forward.failing.add("distance")
    bus.forward.error_type = ValueError

    with pytest.raises(ValueError, match="distance"):
        pair.read()
    assert pair.error_count == 0


def test_stop_failure_is_logged_and_the_other_sensor_still_stops(
    config: NavSightConfig,
    gpio: FakeGpio,
    bus: FakeBus,
    clock: ManualClock,
    caplog: pytest.LogCaptureFixture,
) -> None:
    caplog.set_level(logging.WARNING, logger="navsight.hardware.tof")
    pair = started_pair(config, gpio, bus, clock)
    bus.forward.failing.add("stop_ranging")

    pair.stop()

    assert bus.forward.ranging
    assert not bus.down.ranging
    assert "Could not stop the forward ToF sensor" in caplog.text


def test_simulated_pair_starts_with_nothing_in_range(
    config: NavSightConfig, clock: ManualClock
) -> None:
    sim = SimulatedToFPair(config, clock=clock)
    sim.start()

    assert sim.read() == (ToFReading("forward", None, 1000.0), ToFReading("down", None, 1000.0))
    assert sim.error_count == 0
    sim.stop()


def test_simulated_pair_returns_fresh_readings_for_both_sensors(
    config: NavSightConfig, clock: ManualClock
) -> None:
    sim = SimulatedToFPair(config, clock=clock)
    sim.set_distances(1.5, 0.8)

    assert sim.read() == (ToFReading("forward", 1.5, 1000.0), ToFReading("down", 0.8, 1000.0))
    clock.advance(0.02)
    assert sim.read() == (ToFReading("forward", 1.5, 1000.02), ToFReading("down", 0.8, 1000.02))


@pytest.mark.parametrize(
    "forward_m, down_m, expected",
    [
        (4.0, 0.01, (4.0, 0.01)),
        (4.01, 0.0, (None, None)),
        (None, -1.0, (None, None)),
        (float("nan"), float("inf"), (None, None)),
    ],
)
def test_simulated_pair_applies_the_same_range_limits(
    config: NavSightConfig,
    clock: ManualClock,
    forward_m: float | None,
    down_m: float | None,
    expected: tuple[float | None, float | None],
) -> None:
    sim = SimulatedToFPair(config, clock=clock)
    sim.set_distances(forward_m, down_m)

    forward, down = sim.read()

    assert (forward.distance_m, down.distance_m) == expected


def test_vl53l1x_factory_opens_drivers_only_when_called(monkeypatch: pytest.MonkeyPatch) -> None:
    opened: list[tuple[object, int]] = []

    class FakeVl53l1x:
        def __init__(self, i2c: object, address: int = DEFAULT_ADDRESS) -> None:
            opened.append((i2c, address))

    module = types.ModuleType("adafruit_vl53l1x")
    module.VL53L1X = FakeVl53l1x  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "adafruit_vl53l1x", module)

    factory = make_vl53l1x_factory("i2c-bus")
    assert opened == []

    driver = factory(0x30)

    assert isinstance(driver, FakeVl53l1x)
    assert opened == [("i2c-bus", 0x30)]
