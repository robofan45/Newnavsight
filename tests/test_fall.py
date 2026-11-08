from __future__ import annotations

import math
from typing import Any

import pytest

from navsight.config import NavSightConfig
from navsight.fusion.fall import FallDetector
from navsight.types import STANDARD_GRAVITY, ImuSample

DT = 0.01  # 100 Hz, the BNO085 report rate

STANDING = {"accel_g": 1.0, "gyro_dps": (4.0, 2.0, 1.0), "pitch_deg": -45.0}
FREE_FALL = {"accel_g": 0.2, "gyro_dps": (120.0, 40.0, 10.0), "pitch_deg": -20.0}
IMPACT = {"accel_g": 3.0, "gyro_dps": (250.0, 80.0, 30.0), "pitch_deg": 0.0}  # below hard_impact_g
HARD_IMPACT = {"accel_g": 4.5, "gyro_dps": (250.0, 80.0, 30.0), "pitch_deg": 0.0}
BOUNCE = {"accel_g": 1.6, "gyro_dps": (150.0, 60.0, 20.0), "pitch_deg": 5.0}
LYING_STILL = {"accel_g": 1.0, "gyro_dps": (0.3, -0.2, 0.1), "pitch_deg": 2.0}
MOVING = {"accel_g": 1.3, "gyro_dps": (80.0, 30.0, 10.0), "pitch_deg": -30.0}


def imu_sample(
    timestamp: float,
    accel_g: float = 1.0,
    gyro_dps: tuple[float, float, float] = (0.0, 0.0, 0.0),
    pitch_deg: float = 0.0,
) -> ImuSample:
    return ImuSample(
        timestamp=timestamp,
        heading_deg=0.0,
        pitch_deg=pitch_deg,
        roll_deg=0.0,
        accel_mps2=(0.0, 0.0, accel_g * STANDARD_GRAVITY),
        linear_accel_mps2=(0.0, 0.0, 0.0),
        gyro_dps=gyro_dps,
        step_count=0,
        stationary=False,
    )


class Feeder:
    """Feeds a detector phases of identical samples at 100 Hz and returns the results."""

    def __init__(self, detector: FallDetector, start: float = 100.0) -> None:
        self.detector = detector
        self.start = start
        self.count = 0

    @property
    def now(self) -> float:
        return self.start + self.count * DT

    def run(self, duration_s: float, **sample_args: Any) -> list[bool]:
        results = []
        for _ in range(round(duration_s / DT)):
            results.append(self.detector.update(imu_sample(self.now, **sample_args)))
            self.count += 1
        return results


@pytest.fixture
def detector(config: NavSightConfig) -> FallDetector:
    return FallDetector(config)


@pytest.fixture
def feeder(detector: FallDetector) -> Feeder:
    return Feeder(detector)


def fall_until_impact(feeder: Feeder) -> list[bool]:
    return (
        feeder.run(1.0, **STANDING)
        + feeder.run(0.3, **FREE_FALL)
        + feeder.run(0.02, **IMPACT)
        + feeder.run(0.3, **BOUNCE)
    )


def test_starts_idle(detector: FallDetector) -> None:
    assert detector.state == "idle"


def test_free_fall_impact_and_stillness_confirms_exactly_once(
    detector: FallDetector, feeder: Feeder
) -> None:
    feeder.run(1.0, **STANDING)
    assert detector.state == "idle"
    feeder.run(0.3, **FREE_FALL)
    assert detector.state == "freefall"
    feeder.run(0.02, **IMPACT)
    assert detector.state == "impact"
    assert not any(feeder.run(0.3, **BOUNCE))
    assert detector.state == "impact"

    results = feeder.run(5.0, **LYING_STILL)

    assert results.count(True) == 1
    assert results.index(True) in (200, 201)  # after 2 s of stillness at 100 Hz
    assert detector.state == "idle"


def test_confirming_state_while_waiting_for_two_seconds_of_stillness(
    detector: FallDetector, feeder: Feeder
) -> None:
    fall_until_impact(feeder)
    assert not any(feeder.run(1.9, **LYING_STILL))
    assert detector.state == "confirming"


def test_hard_impact_without_free_fall_confirms(detector: FallDetector, feeder: Feeder) -> None:
    feeder.run(1.0, **STANDING)
    feeder.run(0.02, **HARD_IMPACT)
    assert detector.state == "impact"
    feeder.run(0.2, **BOUNCE)

    results = feeder.run(3.0, **LYING_STILL)

    assert results.count(True) == 1


def test_moderate_impact_without_free_fall_is_ignored(
    detector: FallDetector, feeder: Feeder
) -> None:
    feeder.run(1.0, **STANDING)
    feeder.run(0.02, **IMPACT)
    assert detector.state == "idle"
    assert not any(feeder.run(3.0, **LYING_STILL))


def test_free_fall_shorter_than_minimum_does_not_arm_the_impact_window(
    detector: FallDetector, feeder: Feeder
) -> None:
    feeder.run(1.0, **STANDING)
    feeder.run(0.03, **FREE_FALL)  # 0.02 s from first to last low sample, below 0.06 s
    assert detector.state == "freefall"
    feeder.run(0.02, **IMPACT)
    assert detector.state == "idle"
    assert not any(feeder.run(3.0, **LYING_STILL))


def test_impact_within_window_after_free_fall_counts(
    detector: FallDetector, feeder: Feeder
) -> None:
    feeder.run(0.3, **FREE_FALL)
    feeder.run(0.5, **STANDING)  # the free fall has ended; the impact window is still open
    assert detector.state == "freefall"
    feeder.run(0.02, **IMPACT)
    assert detector.state == "impact"


def test_impact_after_window_does_not_count(detector: FallDetector, feeder: Feeder) -> None:
    feeder.run(0.3, **FREE_FALL)
    feeder.run(1.2, **STANDING)  # longer than impact_window_s
    assert detector.state == "idle"
    feeder.run(0.02, **IMPACT)
    assert detector.state == "idle"
    assert not any(feeder.run(3.0, **LYING_STILL))


def test_movement_before_confirmation_cancels(detector: FallDetector, feeder: Feeder) -> None:
    fall_until_impact(feeder)
    feeder.run(1.0, **LYING_STILL)
    assert detector.state == "confirming"

    feeder.run(0.2, **MOVING)
    assert detector.state == "idle"
    assert not any(feeder.run(5.0, **LYING_STILL))


def test_one_fast_gyro_axis_counts_as_movement(detector: FallDetector, feeder: Feeder) -> None:
    fall_until_impact(feeder)
    feeder.run(1.0, **LYING_STILL)
    feeder.run(0.01, accel_g=1.0, gyro_dps=(0.0, 0.0, 25.0), pitch_deg=2.0)
    assert detector.state == "idle"


def test_still_but_held_at_45_degrees_does_not_confirm(
    detector: FallDetector, feeder: Feeder
) -> None:
    fall_until_impact(feeder)

    results = feeder.run(5.0, accel_g=1.0, gyro_dps=(0.5, 0.5, 0.5), pitch_deg=-45.0)

    assert not any(results)
    assert detector.state == "idle"


@pytest.mark.parametrize("pitch_deg, confirmed", [(25.0, True), (-29.0, True), (40.0, False)])
def test_lying_flat_means_pitch_below_limit(
    feeder: Feeder, pitch_deg: float, confirmed: bool
) -> None:
    fall_until_impact(feeder)
    results = feeder.run(3.0, accel_g=1.0, gyro_dps=(0.0, 0.0, 0.0), pitch_deg=pitch_deg)
    assert any(results) is confirmed


def test_stillness_tolerates_small_acceleration_and_gyro_noise(feeder: Feeder) -> None:
    fall_until_impact(feeder)
    results = feeder.run(3.0, accel_g=1.1, gyro_dps=(15.0, -15.0, 15.0), pitch_deg=0.0)
    assert results.count(True) == 1


def test_cane_that_never_comes_to_rest_returns_to_idle(
    detector: FallDetector, feeder: Feeder
) -> None:
    fall_until_impact(feeder)
    feeder.run(1.5, accel_g=1.2, gyro_dps=(5.0, 5.0, 5.0), pitch_deg=0.0)  # 0.2 g off 1 g
    assert detector.state == "impact"  # still within stillness_s of the impact
    feeder.run(0.5, accel_g=1.2, gyro_dps=(5.0, 5.0, 5.0), pitch_deg=0.0)
    assert detector.state == "idle"
    assert not any(feeder.run(3.0, **LYING_STILL))


def test_user_picking_the_cane_up_and_walking_on_is_not_a_fall(
    detector: FallDetector, feeder: Feeder
) -> None:
    fall_until_impact(feeder)
    assert not any(feeder.run(3.0, **MOVING))
    assert detector.state == "idle"


def test_hard_impact_while_confirming_restarts_the_impact_phase(
    detector: FallDetector, feeder: Feeder
) -> None:
    fall_until_impact(feeder)
    feeder.run(1.0, **LYING_STILL)
    feeder.run(0.01, **HARD_IMPACT)
    assert detector.state == "impact"
    assert feeder.run(3.0, **LYING_STILL).count(True) == 1


def test_detector_rearms_after_a_confirmed_fall(feeder: Feeder) -> None:
    fall_until_impact(feeder)
    first = feeder.run(3.0, **LYING_STILL)
    feeder.run(2.0, **STANDING)
    fall_until_impact(feeder)
    second = feeder.run(3.0, **LYING_STILL)
    assert first.count(True) == 1
    assert second.count(True) == 1


def test_cane_laid_down_gently_is_not_a_fall(detector: FallDetector, feeder: Feeder) -> None:
    feeder.run(1.0, **STANDING)
    feeder.run(0.5, accel_g=1.1, gyro_dps=(30.0, 10.0, 5.0), pitch_deg=-10.0)
    assert not any(feeder.run(5.0, **LYING_STILL))
    assert detector.state == "idle"


def test_normal_walking_never_triggers(detector: FallDetector) -> None:
    for index in range(6000):  # one minute at 100 Hz
        timestamp = 100.0 + index * DT
        phase = 2.0 * math.pi * 1.8 * timestamp  # 1.8 steps per second
        accel_g = 1.05 + 0.35 * math.sin(phase)  # 0.70 to 1.40 g
        gyro = (60.0 * math.sin(phase), 35.0 * math.cos(phase), 15.0 * math.sin(2.0 * phase))
        pitch = -35.0 + 10.0 * math.sin(phase / 2.0)
        assert not detector.update(imu_sample(timestamp, accel_g, gyro, pitch))
        assert detector.state == "idle"


def test_walking_with_hard_cane_taps_never_confirms(detector: FallDetector) -> None:
    for index in range(6000):
        timestamp = 100.0 + index * DT
        phase = 2.0 * math.pi * 1.8 * timestamp
        accel_g = 1.05 + 0.35 * math.sin(phase)
        if index % 55 == 0:
            accel_g = 4.0  # the tip striking the pavement
        gyro = (60.0 * math.sin(phase), 35.0 * math.cos(phase), 15.0 * math.sin(2.0 * phase))
        assert not detector.update(imu_sample(timestamp, accel_g, gyro, -35.0))
        assert detector.state in ("idle", "impact")


@pytest.mark.parametrize("phase", ["freefall", "impact", "confirming"])
def test_reset_returns_to_idle(detector: FallDetector, feeder: Feeder, phase: str) -> None:
    feeder.run(0.3, **FREE_FALL)
    if phase != "freefall":
        feeder.run(0.02, **IMPACT)
    if phase == "confirming":
        feeder.run(1.0, **LYING_STILL)
    assert detector.state == phase

    detector.reset()

    assert detector.state == "idle"
    assert not any(feeder.run(3.0, **LYING_STILL))


def test_thresholds_come_from_config(config: NavSightConfig) -> None:
    config.fall.stillness_s = 1.0
    config.fall.lying_pitch_deg = 50.0
    detector = FallDetector(config)
    feeder = Feeder(detector)
    fall_until_impact(feeder)

    results = feeder.run(2.0, accel_g=1.0, gyro_dps=(0.0, 0.0, 0.0), pitch_deg=-45.0)

    assert results.count(True) == 1
    assert results.index(True) in (100, 101)
