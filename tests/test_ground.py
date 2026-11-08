from __future__ import annotations

import math

import pytest

from navsight.config import NavSightConfig
from navsight.fusion.ground import GroundHazardDetector
from navsight.types import GroundHazard, HazardType, ToFReading

PERIOD = 0.02  # down ToF at 50 Hz
GROUND_M = 1.0


class DownSensor:
    """Feeds a detector with down ToF readings at 50 Hz and collects the hazards it returns."""

    def __init__(self, detector: GroundHazardDetector, start: float = 100.0) -> None:
        self.detector = detector
        self.start = start
        self.count = 0

    @property
    def now(self) -> float:
        """The timestamp of the next reading."""
        return self.start + self.count * PERIOD

    def feed(
        self,
        distance_m: float | None,
        samples: int = 1,
        *,
        tilt_from_vertical_deg: float | None = None,
        pitch_rate_dps: float | None = None,
    ) -> list[GroundHazard]:
        hazards = []
        for _ in range(samples):
            reading = ToFReading(sensor="down", distance_m=distance_m, timestamp=self.now)
            hazard = self.detector.update(reading, tilt_from_vertical_deg, pitch_rate_dps)
            self.count += 1
            if hazard is not None:
                hazards.append(hazard)
        return hazards

    def skip(self, seconds: float) -> None:
        self.count += round(seconds / PERIOD)


@pytest.fixture
def sensor(config: NavSightConfig) -> DownSensor:
    return DownSensor(GroundHazardDetector(config))


@pytest.fixture
def learned(sensor: DownSensor) -> DownSensor:
    """A sensor whose detector has learned a 1 m ground baseline."""
    assert sensor.feed(GROUND_M, 20) == []
    assert sensor.detector.baseline_m == GROUND_M
    return sensor


def confirm_drop(sensor: DownSensor, config: NavSightConfig) -> None:
    hazards = sensor.feed(GROUND_M + 0.3, config.hazard.confirm_samples)
    assert [hazard.hazard for hazard in hazards] == [HazardType.DROP_OFF]


def test_starts_with_no_baseline_and_no_hazard(config: NavSightConfig) -> None:
    detector = GroundHazardDetector(config)
    assert detector.baseline_m is None
    assert detector.current is HazardType.NONE


def test_baseline_needs_min_baseline_samples(sensor: DownSensor, config: NavSightConfig) -> None:
    needed = config.hazard.min_baseline_samples
    sensor.feed(GROUND_M, needed - 1)
    assert sensor.detector.baseline_m is None
    sensor.feed(GROUND_M)
    assert sensor.detector.baseline_m == GROUND_M


def test_readings_before_the_baseline_exists_cannot_confirm(
    sensor: DownSensor, config: NavSightConfig
) -> None:
    hazard = config.hazard
    sensor.feed(GROUND_M, hazard.min_baseline_samples - 1)
    assert sensor.feed(GROUND_M + 0.3) == []  # no baseline yet, so not even a candidate
    assert sensor.feed(GROUND_M + 0.3, hazard.confirm_samples - 1) == []
    [found] = sensor.feed(GROUND_M + 0.3)
    assert found.hazard is HazardType.DROP_OFF


def test_baseline_is_the_median_of_the_readings(sensor: DownSensor) -> None:
    sensor.feed(GROUND_M, 8)
    sensor.feed(3.0)  # one stray reflection
    sensor.feed(GROUND_M, 2)
    assert sensor.detector.baseline_m == GROUND_M


def test_baseline_follows_slow_changes_of_the_ground(sensor: DownSensor) -> None:
    sensor.feed(GROUND_M, 50)
    assert sensor.feed(GROUND_M + 0.08, 100) == []  # inside the thresholds, longer than the window
    assert sensor.detector.baseline_m == pytest.approx(GROUND_M + 0.08)


def test_baseline_forgets_readings_older_than_the_window(
    sensor: DownSensor, config: NavSightConfig
) -> None:
    sensor.feed(GROUND_M, 20)
    sensor.skip(config.hazard.baseline_window_s + 0.5)
    assert sensor.feed(GROUND_M + 0.5, config.hazard.confirm_samples) == []
    assert sensor.detector.baseline_m is None
    sensor.feed(GROUND_M + 0.5, config.hazard.min_baseline_samples)
    assert sensor.detector.baseline_m == GROUND_M + 0.5


def test_drop_is_confirmed_after_confirm_samples_and_reported_once(
    learned: DownSensor, config: NavSightConfig
) -> None:
    confirm = config.hazard.confirm_samples
    assert learned.feed(GROUND_M + 0.15, confirm - 1) == []
    assert learned.detector.current is HazardType.NONE
    confirmed_at = learned.now
    [hazard] = learned.feed(GROUND_M + 0.15)
    assert hazard.hazard is HazardType.DROP_OFF
    assert hazard.delta_m == pytest.approx(0.15)
    assert hazard.timestamp == confirmed_at
    assert learned.feed(GROUND_M + 0.15, 30) == []
    assert learned.detector.current is HazardType.DROP_OFF


def test_step_up_is_detected(learned: DownSensor, config: NavSightConfig) -> None:
    hazards = learned.feed(GROUND_M - 0.15, config.hazard.confirm_samples)
    assert [hazard.hazard for hazard in hazards] == [HazardType.STEP_UP]
    assert hazards[0].delta_m == pytest.approx(-0.15)
    assert learned.detector.current is HazardType.STEP_UP


@pytest.mark.parametrize("distance", [GROUND_M + 0.11, GROUND_M - 0.09])
def test_changes_inside_the_thresholds_are_not_hazards(
    learned: DownSensor, distance: float
) -> None:
    assert learned.feed(distance, 20) == []
    assert learned.detector.current is HazardType.NONE


def test_drop_and_step_up_have_their_own_thresholds(
    sensor: DownSensor, config: NavSightConfig
) -> None:
    hazard = config.hazard
    change = 0.11
    assert hazard.step_up_threshold_m < change < hazard.drop_threshold_m
    sensor.feed(GROUND_M, 20)
    assert sensor.feed(GROUND_M + change, hazard.confirm_samples) == []
    sensor.feed(GROUND_M)
    hazards = sensor.feed(GROUND_M - change, hazard.confirm_samples)
    assert [found.hazard for found in hazards] == [HazardType.STEP_UP]


def test_thresholds_come_from_the_config() -> None:
    config = NavSightConfig()
    config.hazard.drop_threshold_m = 0.3
    sensor = DownSensor(GroundHazardDetector(config))
    sensor.feed(GROUND_M, 20)
    assert sensor.feed(GROUND_M + 0.25, 5) == []
    assert len(sensor.feed(GROUND_M + 0.35, config.hazard.confirm_samples)) == 1


def test_samples_must_be_consecutive(learned: DownSensor, config: NavSightConfig) -> None:
    confirm = config.hazard.confirm_samples
    drop = GROUND_M + 0.2
    assert learned.feed(drop, confirm - 1) == []
    assert learned.feed(GROUND_M) == []
    assert learned.feed(drop, confirm - 1) == []
    assert len(learned.feed(drop)) == 1


def test_drop_and_step_up_samples_do_not_add_up(
    learned: DownSensor, config: NavSightConfig
) -> None:
    confirm = config.hazard.confirm_samples
    assert learned.feed(GROUND_M + 0.2, confirm - 1) == []
    assert learned.feed(GROUND_M - 0.2, confirm - 1) == []
    assert learned.detector.current is HazardType.NONE


@pytest.mark.parametrize("bad", [None, 0.0, -1.0, math.nan, math.inf])
def test_no_echo_counts_as_a_drop_when_the_ground_is_near(
    learned: DownSensor, config: NavSightConfig, bad: float | None
) -> None:
    hazards = learned.feed(bad, config.hazard.confirm_samples)
    assert [hazard.hazard for hazard in hazards] == [HazardType.DROP_OFF]
    assert hazards[0].delta_m == pytest.approx(config.tof.max_range_m - GROUND_M)


@pytest.mark.parametrize("fraction_of_range, is_drop", [(0.85, True), (0.95, False)])
def test_no_echo_is_a_drop_only_below_90_percent_of_max_range(
    sensor: DownSensor, config: NavSightConfig, fraction_of_range: float, is_drop: bool
) -> None:
    ground = fraction_of_range * config.tof.max_range_m
    sensor.feed(ground, 20)
    hazards = sensor.feed(None, 10)
    assert len(hazards) == (1 if is_drop else 0)
    assert sensor.detector.baseline_m == pytest.approx(ground)


def test_fast_pitch_rate_readings_are_ignored(learned: DownSensor, config: NavSightConfig) -> None:
    fast = config.hazard.max_pitch_rate_dps + 1.0
    assert learned.feed(GROUND_M + 0.5, 20, pitch_rate_dps=fast) == []
    assert learned.feed(GROUND_M + 0.5, 20, pitch_rate_dps=-fast) == []
    assert learned.detector.current is HazardType.NONE
    assert learned.detector.baseline_m == GROUND_M  # not collected into the baseline


def test_ignored_readings_do_not_break_a_run(learned: DownSensor, config: NavSightConfig) -> None:
    fast = config.hazard.max_pitch_rate_dps + 10.0
    drop = GROUND_M + 0.3
    assert learned.feed(drop, config.hazard.confirm_samples - 1) == []
    assert learned.feed(GROUND_M, 5, pitch_rate_dps=fast) == []
    assert len(learned.feed(drop)) == 1


def test_pitch_rate_at_the_limit_is_used(learned: DownSensor, config: NavSightConfig) -> None:
    limit = config.hazard.max_pitch_rate_dps
    hazards = learned.feed(GROUND_M + 0.3, config.hazard.confirm_samples, pitch_rate_dps=limit)
    assert len(hazards) == 1


def test_fast_pitch_rate_does_not_hold_back_the_baseline(
    sensor: DownSensor, config: NavSightConfig
) -> None:
    fast = config.hazard.max_pitch_rate_dps * 2
    sensor.feed(GROUND_M, config.hazard.min_baseline_samples, pitch_rate_dps=fast)
    assert sensor.detector.baseline_m is None


def test_hazard_clears_after_clear_after_s_of_normal_readings(
    learned: DownSensor, config: NavSightConfig
) -> None:
    confirm_drop(learned, config)
    clear_samples = round(config.hazard.clear_after_s / PERIOD)
    learned.feed(GROUND_M, clear_samples - 5)
    assert learned.detector.current is HazardType.DROP_OFF
    assert learned.feed(GROUND_M, 10) == []
    assert learned.detector.current is HazardType.NONE


def test_a_hazard_reading_restarts_the_clear_timer(
    learned: DownSensor, config: NavSightConfig
) -> None:
    confirm_drop(learned, config)
    clear_samples = round(config.hazard.clear_after_s / PERIOD)
    learned.feed(GROUND_M, clear_samples - 10)
    assert learned.feed(GROUND_M + 0.3) == []
    learned.feed(GROUND_M, clear_samples - 10)
    assert learned.detector.current is HazardType.DROP_OFF
    learned.feed(GROUND_M, 15)
    assert learned.detector.current is HazardType.NONE


def test_baseline_is_frozen_while_a_hazard_is_active(
    learned: DownSensor, config: NavSightConfig
) -> None:
    confirm_drop(learned, config)
    learned.feed(GROUND_M + 0.3, 150)  # 3 s at the edge: longer than the baseline window
    assert learned.detector.current is HazardType.DROP_OFF
    assert learned.detector.baseline_m == GROUND_M
    learned.feed(GROUND_M, 60)
    assert learned.detector.current is HazardType.NONE


def test_hazard_readings_are_not_learned_into_the_baseline() -> None:
    config = NavSightConfig()
    config.hazard.clear_after_s = 0.2
    sensor = DownSensor(GroundHazardDetector(config))
    sensor.feed(GROUND_M, 60)
    confirm_drop(sensor, config)
    sensor.feed(GROUND_M + 0.3, 50)  # 1 s of drop readings, then back to the old ground
    assert sensor.feed(GROUND_M, 15) == []  # learning the drop would now report a step-up
    assert sensor.detector.current is HazardType.NONE
    assert sensor.detector.baseline_m == GROUND_M


def test_no_echo_readings_do_not_count_toward_the_baseline(
    sensor: DownSensor, config: NavSightConfig
) -> None:
    sensor.feed(GROUND_M, config.hazard.min_baseline_samples - 1)
    sensor.feed(None, 5)
    assert sensor.detector.baseline_m is None


def test_detection_resumes_after_a_long_hazard_clears(
    learned: DownSensor, config: NavSightConfig
) -> None:
    confirm_drop(learned, config)
    learned.feed(GROUND_M + 0.3, 150)
    learned.feed(GROUND_M, 60)
    learned.feed(GROUND_M, config.hazard.min_baseline_samples)
    assert learned.detector.baseline_m == GROUND_M
    hazards = learned.feed(GROUND_M - 0.2, config.hazard.confirm_samples)
    assert [hazard.hazard for hazard in hazards] == [HazardType.STEP_UP]


def test_the_same_hazard_is_reported_again_after_it_clears(
    learned: DownSensor, config: NavSightConfig
) -> None:
    confirm_drop(learned, config)
    learned.feed(GROUND_M, 60)
    assert learned.detector.current is HazardType.NONE
    confirm_drop(learned, config)


def test_drop_followed_by_step_up_reports_both(learned: DownSensor, config: NavSightConfig) -> None:
    confirm_drop(learned, config)
    hazards = learned.feed(GROUND_M - 0.2, config.hazard.confirm_samples)
    assert [hazard.hazard for hazard in hazards] == [HazardType.STEP_UP]
    assert learned.detector.current is HazardType.STEP_UP


@pytest.mark.parametrize(
    "tilt, expected_delta", [(None, 0.4), (0.0, 0.4), (60.0, 0.2), (-60.0, 0.2)]
)
def test_tilt_projects_the_change_onto_the_vertical(
    learned: DownSensor, config: NavSightConfig, tilt: float | None, expected_delta: float
) -> None:
    confirm = config.hazard.confirm_samples
    hazards = learned.feed(GROUND_M + 0.4, confirm, tilt_from_vertical_deg=tilt)
    assert hazards[0].delta_m == pytest.approx(expected_delta)


def test_60_degree_tilt_halves_the_vertical_change(config: NavSightConfig) -> None:
    upright = DownSensor(GroundHazardDetector(config))
    tilted = DownSensor(GroundHazardDetector(config))
    for sensor in (upright, tilted):
        sensor.feed(GROUND_M, 20)
    confirm = config.hazard.confirm_samples
    # 0.2 m more slant distance is a 0.1 m vertical drop at 60 degrees: under the 0.12 m threshold.
    assert len(upright.feed(GROUND_M + 0.2, confirm, tilt_from_vertical_deg=0.0)) == 1
    assert tilted.feed(GROUND_M + 0.2, confirm, tilt_from_vertical_deg=60.0) == []


@pytest.mark.parametrize("tilt", [90.0, 120.0, -95.0, math.nan])
def test_readings_with_the_sensor_not_facing_down_are_ignored(
    learned: DownSensor, tilt: float
) -> None:
    assert learned.feed(GROUND_M + 0.5, 30, tilt_from_vertical_deg=tilt) == []
    assert learned.detector.current is HazardType.NONE
    assert learned.detector.baseline_m == GROUND_M


def test_rejects_forward_readings(config: NavSightConfig) -> None:
    detector = GroundHazardDetector(config)
    with pytest.raises(ValueError, match="down"):
        detector.update(ToFReading(sensor="forward", distance_m=1.0, timestamp=0.0))


@pytest.mark.parametrize(
    "field, value",
    [("min_baseline_samples", 0), ("confirm_samples", 0), ("baseline_window_s", 0.0)],
)
def test_invalid_config_is_rejected(field: str, value: float) -> None:
    config = NavSightConfig()
    setattr(config.hazard, field, value)
    with pytest.raises(ValueError, match=f"hazard.{field}"):
        GroundHazardDetector(config)
