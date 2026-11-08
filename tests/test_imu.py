from __future__ import annotations

import math
import sys
import threading
import types
from typing import Any

import pytest

from navsight.config import AXIS_CHOICES, NavSightConfig
from navsight.hardware.i2c import I2C_LOCK
from navsight.hardware.imu import (
    ENABLED_REPORTS,
    REPORT_ACCELEROMETER,
    REPORT_GYROSCOPE,
    REPORT_LINEAR_ACCELERATION,
    REPORT_ROTATION_VECTOR,
    REPORT_STABILITY_CLASSIFIER,
    REPORT_STEP_COUNTER,
    Imu,
    SimulatedImu,
    make_bno085_factory,
    orientation_from_quaternion,
)
from navsight.types import STANDARD_GRAVITY, ImuSample
from navsight.util import ManualClock

IDENTITY = (0.0, 0.0, 0.0, 1.0)
EAST = (1.0, 0.0, 0.0)
NORTH = (0.0, 1.0, 0.0)
UP = (0.0, 0.0, 1.0)


def about(axis: tuple[float, float, float], angle_deg: float) -> tuple[float, ...]:
    """Quaternion (i, j, k, real) of a right-handed rotation about a world axis."""
    half = math.radians(angle_deg) / 2.0
    s = math.sin(half)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))


def then(first: tuple[float, ...], second: tuple[float, ...]) -> tuple[float, ...]:
    """Quaternion that applies ``first`` and then ``second`` (the product second * first)."""
    ax, ay, az, aw = second
    bx, by, bz, bw = first
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


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


class FakeImuDriver:
    """In-memory BNO085 that records any call made without I2C_LOCK."""

    def __init__(self) -> None:
        self.values: dict[str, Any] = {
            "quaternion": IDENTITY,
            "acceleration": (0.0, 0.0, STANDARD_GRAVITY),
            "linear_acceleration": (0.0, 0.0, 0.0),
            "gyro": (0.0, 0.0, 0.0),
            "steps": 0,
            "stability_classification": "Stable",
        }
        self.enabled: list[int] = []
        self.unlocked_calls: list[str] = []

    def _get(self, name: str) -> Any:
        if not i2c_lock_held():
            self.unlocked_calls.append(name)
        return self.values[name]

    def enable_feature(self, feature_id: int) -> None:
        if not i2c_lock_held():
            self.unlocked_calls.append("enable_feature")
        self.enabled.append(feature_id)

    @property
    def quaternion(self) -> Any:
        return self._get("quaternion")

    @property
    def acceleration(self) -> Any:
        return self._get("acceleration")

    @property
    def linear_acceleration(self) -> Any:
        return self._get("linear_acceleration")

    @property
    def gyro(self) -> Any:
        return self._get("gyro")

    @property
    def steps(self) -> Any:
        return self._get("steps")

    @property
    def stability_classification(self) -> Any:
        return self._get("stability_classification")


class FakeBno08xI2C:
    """Stands in for adafruit_bno08x.i2c.BNO08X_I2C."""

    def __init__(self, i2c: Any, address: int = 0x4A) -> None:
        self.i2c = i2c
        self.address = address
        self.enable_calls: list[tuple[int, dict[str, Any]]] = []
        self.quaternion = about(UP, 90)
        self.acceleration = (0.1, 0.2, 9.8)
        self.linear_acceleration = (0.1, 0.2, 0.0)
        self.gyro = (0.0, 0.0, math.pi)
        self.steps = 7
        self.stability_classification = "On Table"

    def enable_feature(self, feature_id: int, **kwargs: Any) -> None:
        self.enable_calls.append((feature_id, kwargs))


@pytest.fixture
def driver() -> FakeImuDriver:
    return FakeImuDriver()


@pytest.fixture
def imu(config: NavSightConfig, driver: FakeImuDriver, clock: ManualClock) -> Imu:
    sensor = Imu(config, lambda: driver, clock=clock)
    sensor.start()
    return sensor


@pytest.fixture
def created_bno08x(monkeypatch: pytest.MonkeyPatch) -> list[FakeBno08xI2C]:
    """Install a fake adafruit_bno08x package; returns the drivers it creates."""
    created: list[FakeBno08xI2C] = []

    class RecordingBno08xI2C(FakeBno08xI2C):
        def __init__(self, i2c: Any, address: int = 0x4A) -> None:
            super().__init__(i2c, address)
            created.append(self)

    package = types.ModuleType("adafruit_bno08x")
    i2c_module = types.ModuleType("adafruit_bno08x.i2c")
    i2c_module.BNO08X_I2C = RecordingBno08xI2C  # type: ignore[attr-defined]
    package.i2c = i2c_module  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "adafruit_bno08x", package)
    monkeypatch.setitem(sys.modules, "adafruit_bno08x.i2c", i2c_module)
    return created


def test_report_ids_match_the_sh2_values() -> None:
    assert REPORT_ACCELEROMETER == 0x01
    assert REPORT_GYROSCOPE == 0x02
    assert REPORT_LINEAR_ACCELERATION == 0x04
    assert REPORT_ROTATION_VECTOR == 0x05
    assert REPORT_STEP_COUNTER == 0x11
    assert REPORT_STABILITY_CLASSIFIER == 0x13


def test_identity_quaternion_points_north_and_level() -> None:
    assert orientation_from_quaternion(IDENTITY) == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.parametrize(
    "angle_deg, heading_deg", [(90, 270), (-90, 90), (180, 180), (-45, 45), (30, 330)]
)
def test_rotation_about_up_turns_the_heading(angle_deg: float, heading_deg: float) -> None:
    heading, pitch, roll = orientation_from_quaternion(about(UP, angle_deg))

    assert heading == pytest.approx(heading_deg)
    assert pitch == pytest.approx(0.0, abs=1e-9)
    assert roll == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("angle_deg, pitch_deg", [(-45, -45), (30, 30), (-90, -90)])
def test_tilting_the_forward_axis_sets_pitch(angle_deg: float, pitch_deg: float) -> None:
    _, pitch, roll = orientation_from_quaternion(about(EAST, angle_deg))

    assert pitch == pytest.approx(pitch_deg)
    assert roll == pytest.approx(0.0, abs=1e-9)


def test_pitched_down_cane_facing_east() -> None:
    quaternion = then(about(EAST, -45), about(UP, -90))

    assert orientation_from_quaternion(quaternion) == pytest.approx((90.0, -45.0, 0.0), abs=1e-9)


@pytest.mark.parametrize(
    "axis, heading_deg", [("y", 0.0), ("x", 90.0), ("-y", 180.0), ("-x", 270.0)]
)
def test_horizontal_forward_axis_variants(axis: str, heading_deg: float) -> None:
    heading, pitch, roll = orientation_from_quaternion(IDENTITY, axis)

    assert heading == pytest.approx(heading_deg)
    assert pitch == pytest.approx(0.0)
    assert roll == pytest.approx(0.0)


@pytest.mark.parametrize("axis, pitch_deg", [("z", 90.0), ("-z", -90.0)])
def test_vertical_forward_axis_variants(axis: str, pitch_deg: float) -> None:
    _, pitch, _ = orientation_from_quaternion(IDENTITY, axis)

    assert pitch == pytest.approx(pitch_deg)


def test_x_forward_board_turned_left_points_north() -> None:
    heading, pitch, _ = orientation_from_quaternion(about(UP, 90), "x")

    assert heading == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)


def test_minus_y_forward_tilted_down() -> None:
    heading, pitch, _ = orientation_from_quaternion(about(EAST, 45), "-y")

    assert heading == pytest.approx(180.0)
    assert pitch == pytest.approx(-45.0)


def test_roll_uses_the_x_axis_for_a_y_forward_axis() -> None:
    heading, pitch, roll = orientation_from_quaternion(about(NORTH, 30), "y")

    assert heading == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)
    assert roll == pytest.approx(-30.0)


def test_roll_uses_the_y_axis_for_an_x_forward_axis() -> None:
    heading, pitch, roll = orientation_from_quaternion(about(EAST, 30), "x")

    assert heading == pytest.approx(90.0)
    assert pitch == pytest.approx(0.0, abs=1e-9)
    assert roll == pytest.approx(30.0)


def test_roll_uses_the_x_axis_for_a_z_forward_axis() -> None:
    heading, pitch, roll = orientation_from_quaternion(about(NORTH, 30), "z")

    assert heading == pytest.approx(90.0)
    assert pitch == pytest.approx(60.0)
    assert roll == pytest.approx(-30.0)


@pytest.mark.parametrize(
    "quaternion, offset_deg, heading_deg",
    [
        (IDENTITY, 370.0, 10.0),
        (IDENTITY, -30.0, 330.0),
        (IDENTITY, 360.0, 0.0),
        (about(UP, 90), 100.0, 10.0),
        (about(UP, -90), -100.0, 350.0),
    ],
)
def test_heading_offset_wraps_to_0_360(
    quaternion: tuple[float, ...], offset_deg: float, heading_deg: float
) -> None:
    heading, _, _ = orientation_from_quaternion(quaternion, "y", offset_deg)

    assert 0.0 <= heading < 360.0
    assert heading == pytest.approx(heading_deg, abs=1e-9)


def test_quaternion_is_normalised_first() -> None:
    quaternion = then(about(EAST, -30), about(UP, 60))
    scaled = tuple(3.0 * value for value in quaternion)

    assert orientation_from_quaternion(scaled) == pytest.approx(
        orientation_from_quaternion(quaternion)
    )


@pytest.mark.parametrize(
    "quaternion",
    [(0.0, 0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0, 1.0), (float("inf"), 0.0, 0.0, 1.0)],
)
def test_quaternion_that_cannot_be_normalised_raises(quaternion: tuple[float, ...]) -> None:
    with pytest.raises(ValueError, match="normalised"):
        orientation_from_quaternion(quaternion)


def test_unknown_forward_axis_raises() -> None:
    with pytest.raises(ValueError, match="forward_axis"):
        orientation_from_quaternion(IDENTITY, "w")


@pytest.mark.parametrize("axis", AXIS_CHOICES)
def test_every_configurable_axis_is_supported(axis: str) -> None:
    heading, pitch, roll = orientation_from_quaternion(about(UP, 10), axis)

    assert 0.0 <= heading < 360.0
    assert -90.0 <= pitch <= 90.0
    assert -90.0 <= roll <= 90.0


def test_start_enables_the_six_reports_under_the_lock(
    config: NavSightConfig, driver: FakeImuDriver, clock: ManualClock
) -> None:
    calls = 0

    def factory() -> FakeImuDriver:
        nonlocal calls
        calls += 1
        assert i2c_lock_held()
        return driver

    sensor = Imu(config, factory, clock=clock)
    sensor.start()
    sensor.start()

    assert calls == 1
    assert driver.enabled == list(ENABLED_REPORTS)
    assert set(driver.enabled) == {0x01, 0x02, 0x04, 0x05, 0x11, 0x13}
    assert driver.enabled[-1] == REPORT_ROTATION_VECTOR
    assert driver.unlocked_calls == []


def test_missing_device_at_start_raises_with_a_wiring_hint(
    config: NavSightConfig, clock: ManualClock
) -> None:
    def factory() -> FakeImuDriver:
        raise ValueError("No I2C device at address: 0x4a")

    sensor = Imu(config, factory, clock=clock)

    with pytest.raises(RuntimeError, match="0x4a") as excinfo:
        sensor.start()
    assert "SDA (GPIO 2)" in str(excinfo.value)
    assert isinstance(excinfo.value.__cause__, ValueError)
    with pytest.raises(RuntimeError, match="start"):
        sensor.read()


def test_feature_that_cannot_be_enabled_fails_start(
    config: NavSightConfig, driver: FakeImuDriver, clock: ManualClock
) -> None:
    def enable_feature(feature_id: int) -> None:
        raise RuntimeError("Was not able to enable feature", feature_id)

    driver.enable_feature = enable_feature  # type: ignore[method-assign]
    sensor = Imu(config, lambda: driver, clock=clock)

    with pytest.raises(RuntimeError, match="Could not start the BNO085"):
        sensor.start()


def test_read_before_start_raises(
    config: NavSightConfig, driver: FakeImuDriver, clock: ManualClock
) -> None:
    sensor = Imu(config, lambda: driver, clock=clock)

    with pytest.raises(RuntimeError, match="start"):
        sensor.read()


def test_read_level_cane_pointing_north(
    imu: Imu, driver: FakeImuDriver, clock: ManualClock
) -> None:
    clock.advance(0.25)

    sample = imu.read()

    assert sample == ImuSample(
        timestamp=1000.25,
        heading_deg=pytest.approx(0.0),
        pitch_deg=pytest.approx(0.0),
        roll_deg=pytest.approx(0.0),
        accel_mps2=(0.0, 0.0, STANDARD_GRAVITY),
        linear_accel_mps2=(0.0, 0.0, 0.0),
        gyro_dps=(0.0, 0.0, 0.0),
        step_count=0,
        stationary=False,
    )
    assert sample.accel_magnitude_g == pytest.approx(1.0)
    assert driver.unlocked_calls == []


def test_read_uses_the_configured_axis_and_offset(
    config: NavSightConfig, driver: FakeImuDriver, clock: ManualClock
) -> None:
    config.imu.forward_axis = "x"
    config.imu.heading_offset_deg = 15.0
    sensor = Imu(config, lambda: driver, clock=clock)
    sensor.start()
    driver.values["quaternion"] = about(EAST, 20)

    sample = sensor.read()

    assert sample.heading_deg == pytest.approx(105.0)
    assert sample.pitch_deg == pytest.approx(0.0, abs=1e-9)
    assert sample.roll_deg == pytest.approx(20.0)


def test_read_pitched_down_cane(imu: Imu, driver: FakeImuDriver) -> None:
    driver.values["quaternion"] = then(about(EAST, -45), about(UP, 90))

    sample = imu.read()

    assert sample.heading_deg == pytest.approx(270.0)
    assert sample.pitch_deg == pytest.approx(-45.0)


def test_gyro_is_converted_to_degrees_per_second(imu: Imu, driver: FakeImuDriver) -> None:
    driver.values["gyro"] = (math.pi, -math.pi / 2, 0.1)

    sample = imu.read()

    assert sample.gyro_dps == pytest.approx((180.0, -90.0, math.degrees(0.1)))


def test_accelerations_become_float_tuples(imu: Imu, driver: FakeImuDriver) -> None:
    driver.values["acceleration"] = [1, -2, 9]
    driver.values["linear_acceleration"] = [0.5, 0, -0.25]

    sample = imu.read()

    assert sample.accel_mps2 == (1.0, -2.0, 9.0)
    assert sample.linear_accel_mps2 == (0.5, 0.0, -0.25)
    assert all(type(value) is float for value in sample.accel_mps2)


@pytest.mark.parametrize(
    "classification, stationary",
    [
        ("On Table", True),
        ("Stationary", True),
        ("Stable", False),
        ("In motion", False),
        ("Unknown", False),
    ],
)
def test_stationary_comes_from_the_stability_classifier(
    imu: Imu, driver: FakeImuDriver, classification: str, stationary: bool
) -> None:
    driver.values["stability_classification"] = classification

    assert imu.read().stationary is stationary


def test_step_count_never_decreases_across_counter_resets(imu: Imu, driver: FakeImuDriver) -> None:
    counts = []
    for raw in [5, 12, 0, 3, 3, 1, 65535, 2]:
        driver.values["steps"] = raw
        counts.append(imu.read().step_count)

    assert counts == [5, 12, 12, 15, 15, 16, 65550, 65552]
    assert counts == sorted(counts)


def test_step_placeholder_before_the_first_report_counts_as_no_steps(
    imu: Imu, driver: FakeImuDriver
) -> None:
    driver.values["steps"] = (0.0, 0.0, 0.0)  # what adafruit_bno08x holds before a report
    assert imu.read().step_count == 0

    driver.values["steps"] = 4
    assert imu.read().step_count == 4

    driver.values["steps"] = (0.0, 0.0, 0.0)
    assert imu.read().step_count == 4


def test_negative_raw_step_count_is_ignored(imu: Imu, driver: FakeImuDriver) -> None:
    driver.values["steps"] = 9
    assert imu.read().step_count == 9

    driver.values["steps"] = -1
    assert imu.read().step_count == 9


def test_step_count_carries_on_after_a_restart(config: NavSightConfig, clock: ManualClock) -> None:
    drivers = [FakeImuDriver(), FakeImuDriver()]
    drivers[0].values["steps"] = 30
    sensor = Imu(config, lambda: drivers.pop(0), clock=clock)
    sensor.start()
    assert sensor.read().step_count == 30

    sensor.stop()
    second = drivers[0]
    sensor.start()  # a new driver soft-resets the BNO085, so its counter restarts at 0
    assert sensor.read().step_count == 30

    second.values["steps"] = 2
    assert sensor.read().step_count == 32


def test_zero_quaternion_before_the_first_rotation_vector_raises(
    imu: Imu, driver: FakeImuDriver
) -> None:
    driver.values["quaternion"] = (0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="rotation vector"):
        imu.read()

    driver.values["quaternion"] = IDENTITY
    assert imu.read().heading_deg == pytest.approx(0.0)


def test_stop_releases_the_driver(imu: Imu) -> None:
    imu.stop()
    imu.stop()

    with pytest.raises(RuntimeError, match="start"):
        imu.read()


def test_bno085_factory_opens_the_driver_only_when_called(
    created_bno08x: list[FakeBno08xI2C],
) -> None:
    factory = make_bno085_factory("i2c-bus", 0x4B)
    assert created_bno08x == []

    driver = factory()

    assert created_bno08x == [driver]
    assert (driver.i2c, driver.address) == ("i2c-bus", 0x4B)


def test_bno085_factory_keeps_the_driver_report_interval_by_default(
    config: NavSightConfig, clock: ManualClock, created_bno08x: list[FakeBno08xI2C]
) -> None:
    Imu(config, make_bno085_factory("i2c-bus", 0x4A), clock=clock).start()

    assert created_bno08x[0].enable_calls == [(report, {}) for report in ENABLED_REPORTS]


def test_bno085_factory_report_rate_sets_every_report_interval(
    config: NavSightConfig, clock: ManualClock, created_bno08x: list[FakeBno08xI2C]
) -> None:
    factory = make_bno085_factory("i2c-bus", 0x4A, report_rate_hz=100.0)
    sensor = Imu(config, factory, clock=clock)
    sensor.start()

    sample = sensor.read()

    assert created_bno08x[0].enable_calls == [
        (report, {"report_interval": 10_000}) for report in ENABLED_REPORTS
    ]
    assert sample.heading_deg == pytest.approx(270.0)
    assert sample.accel_mps2 == (0.1, 0.2, 9.8)
    assert sample.linear_accel_mps2 == (0.1, 0.2, 0.0)
    assert sample.gyro_dps == pytest.approx((0.0, 0.0, 180.0))
    assert sample.step_count == 7
    assert sample.stationary


@pytest.mark.parametrize("rate_hz", [0.0, -5.0])
def test_bno085_factory_rejects_a_rate_that_is_not_positive(rate_hz: float) -> None:
    with pytest.raises(ValueError, match="report_rate_hz"):
        make_bno085_factory("i2c-bus", 0x4A, report_rate_hz=rate_hz)


def test_simulated_imu_defaults_to_a_stationary_cane(
    config: NavSightConfig, clock: ManualClock
) -> None:
    sim = SimulatedImu(config, clock=clock)
    sim.start()

    sample = sim.read()

    assert sample == ImuSample(
        timestamp=1000.0,
        heading_deg=0.0,
        pitch_deg=-45.0,
        roll_deg=0.0,
        accel_mps2=(0.0, 0.0, 9.80665),
        linear_accel_mps2=(0.0, 0.0, 0.0),
        gyro_dps=(0.0, 0.0, 0.0),
        step_count=0,
        stationary=True,
    )
    assert sample.accel_magnitude_g == pytest.approx(1.0)
    sim.stop()


def test_simulated_imu_set_state_changes_only_the_given_values(
    config: NavSightConfig, clock: ManualClock
) -> None:
    sim = SimulatedImu(config, clock=clock)

    sim.set_state(heading_deg=90.0, gyro_dps=[5, 0, -5], stationary=False)
    clock.advance(0.5)
    sample = sim.read()

    assert sample.timestamp == 1000.5
    assert sample.heading_deg == 90.0
    assert sample.gyro_dps == (5.0, 0.0, -5.0)
    assert not sample.stationary
    assert sample.pitch_deg == -45.0
    assert sample.roll_deg == 0.0
    assert sample.accel_mps2 == (0.0, 0.0, STANDARD_GRAVITY)
    assert sample.step_count == 0


def test_simulated_imu_set_state_sets_every_value(
    config: NavSightConfig, clock: ManualClock
) -> None:
    sim = SimulatedImu(config, clock=clock)

    sim.set_state(
        heading_deg=45.0,
        pitch_deg=-10.0,
        roll_deg=5.0,
        accel_mps2=(0.0, 0.0, 0.0),
        linear_accel_mps2=(0.0, 0.0, -9.8),
        gyro_dps=(1.0, 2.0, 3.0),
        add_steps=2,
        stationary=False,
    )

    assert sim.read() == ImuSample(
        timestamp=1000.0,
        heading_deg=45.0,
        pitch_deg=-10.0,
        roll_deg=5.0,
        accel_mps2=(0.0, 0.0, 0.0),
        linear_accel_mps2=(0.0, 0.0, -9.8),
        gyro_dps=(1.0, 2.0, 3.0),
        step_count=2,
        stationary=False,
    )


@pytest.mark.parametrize("heading_deg, expected", [(370.0, 10.0), (-90.0, 270.0), (360.0, 0.0)])
def test_simulated_imu_wraps_the_heading(
    config: NavSightConfig, clock: ManualClock, heading_deg: float, expected: float
) -> None:
    sim = SimulatedImu(config, clock=clock)

    sim.set_state(heading_deg=heading_deg)

    assert sim.read().heading_deg == pytest.approx(expected)


def test_simulated_imu_steps_accumulate_and_never_drop(
    config: NavSightConfig, clock: ManualClock
) -> None:
    sim = SimulatedImu(config, clock=clock)

    sim.set_state(add_steps=3)
    sim.set_state(add_steps=2)
    sim.set_state(pitch_deg=-30.0)
    assert sim.read().step_count == 5

    with pytest.raises(ValueError, match="add_steps"):
        sim.set_state(add_steps=-1, pitch_deg=0.0)
    sample = sim.read()
    assert sample.step_count == 5
    assert sample.pitch_deg == -30.0
