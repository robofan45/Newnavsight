from __future__ import annotations

import math

import numpy as np
import pytest

from navsight.config import NavSightConfig
from navsight.fusion.obstacle_map import ObstacleMap, RangeTrack
from navsight.types import Detection, Obstacle, ToFReading

CENTRE = 3  # default config: 7 sectors of 15 degrees
PERIOD = 0.02  # forward ToF at 50 Hz


def forward(distance_m: float | None, timestamp: float) -> ToFReading:
    return ToFReading(sensor="forward", distance_m=distance_m, timestamp=timestamp)


def detection(
    bearing_deg: float = 0.0,
    distance_m: float | None = None,
    label: str = "person",
    confidence: float = 0.8,
) -> Detection:
    return Detection(
        label=label,
        confidence=confidence,
        bbox_xyxy=(100.0, 80.0, 220.0, 400.0),
        bearing_deg=bearing_deg,
        distance_m=distance_m,
    )


def approach(
    obstacle_map: ObstacleMap, start_m: float, speed_mps: float, start_t: float, samples: int
) -> float:
    """Feed the forward ToF a target closing at ``speed_mps``; return the last timestamp."""
    for k in range(samples):
        obstacle_map.update_tof(forward(start_m - speed_mps * k * PERIOD, start_t + k * PERIOD))
    return start_t + (samples - 1) * PERIOD


def closing_track() -> RangeTrack:
    """A track that has followed a target closing at 1 m/s for one second."""
    track = RangeTrack(3.0, 0.05, now=0.0, accel_std_mps2=2.0)
    for k in range(1, 51):
        track.update(3.0 - k * PERIOD, 0.05, now=k * PERIOD)
    return track


@pytest.fixture
def obstacle_map(config: NavSightConfig) -> ObstacleMap:
    return ObstacleMap(config)


# RangeTrack


def test_new_track_starts_at_the_measurement_at_rest() -> None:
    track = RangeTrack(2.0, 0.1, now=5.0, accel_std_mps2=2.0)
    assert track.distance_m == 2.0
    assert track.closing_speed_mps == 0.0
    assert track.last_update == 5.0
    assert track.time_to_collision(0.2) is None


def test_track_follows_the_constant_velocity_kalman_equations() -> None:
    std, accel_std, dt = 0.1, 2.0, 0.5
    measurements = ((2.0, 0.5), (2.5, 0.2))
    track = RangeTrack(1.0, std, now=0.0, accel_std_mps2=accel_std)
    for k, (distance, measurement_std) in enumerate(measurements, start=1):
        track.update(distance, measurement_std, now=k * dt)

    x = np.array([1.0, 0.0])
    p = np.diag([std**2, 1.0])
    f = np.array([[1.0, dt], [0.0, 1.0]])
    q = accel_std**2 * np.array([[dt**4 / 4, dt**3 / 2], [dt**3 / 2, dt**2]])
    for distance, measurement_std in measurements:
        x = f @ x
        p = f @ p @ f.T + q
        gain = p[:, 0] / (p[0, 0] + measurement_std**2)
        x = x + gain * (distance - x[0])
        p = (np.eye(2) - np.outer(gain, [1.0, 0.0])) @ p
    assert track.distance_m == pytest.approx(x[0])
    assert track.closing_speed_mps == pytest.approx(-x[1])
    assert track.last_update == 2 * dt


def test_predict_moves_the_range_at_the_estimated_rate() -> None:
    track = closing_track()
    distance, closing = track.distance_m, track.closing_speed_mps
    track.predict(now=1.5)
    assert track.distance_m == pytest.approx(distance - 0.5 * closing)
    assert track.closing_speed_mps == pytest.approx(closing)
    assert track.last_update == pytest.approx(1.0)


def test_predict_never_goes_back_in_time() -> None:
    track = closing_track()
    track.predict(now=1.5)
    distance = track.distance_m
    track.predict(now=1.2)
    assert track.distance_m == distance


def test_older_measurement_is_applied_without_predicting_backwards() -> None:
    track = RangeTrack(2.0, 0.05, now=10.0, accel_std_mps2=2.0)
    track.update(2.0, 0.05, now=10.1)
    track.update(1.9, 0.05, now=10.05)
    assert track.last_update == 10.1
    assert 1.9 < track.distance_m < 2.0


def test_predicted_returns_a_copy_and_leaves_the_track_alone() -> None:
    track, twin = closing_track(), closing_track()
    ahead = track.predicted(now=1.3)
    assert ahead.distance_m == pytest.approx(track.distance_m - 0.3 * track.closing_speed_mps)
    assert track.distance_m == twin.distance_m
    track.update(1.95, 0.05, now=1.02)
    twin.update(1.95, 0.05, now=1.02)
    assert (track.distance_m, track.closing_speed_mps) == (twin.distance_m, twin.closing_speed_mps)


def test_time_to_collision_is_distance_over_closing_speed() -> None:
    track = closing_track()
    assert track.closing_speed_mps == pytest.approx(1.0, abs=0.05)
    expected = track.distance_m / track.closing_speed_mps
    assert track.time_to_collision(0.2) == pytest.approx(expected)


def test_time_to_collision_needs_the_minimum_closing_speed() -> None:
    track = closing_track()
    closing = track.closing_speed_mps
    assert track.time_to_collision(closing) is not None
    assert track.time_to_collision(closing + 0.01) is None


def test_no_time_to_collision_for_receding_or_static_targets() -> None:
    receding = RangeTrack(1.0, 0.05, now=0.0, accel_std_mps2=2.0)
    for k in range(1, 51):
        receding.update(1.0 + k * PERIOD, 0.05, now=k * PERIOD)
    assert receding.closing_speed_mps < 0.0
    assert receding.time_to_collision(0.0) is None
    static = RangeTrack(1.0, 0.05, now=0.0, accel_std_mps2=2.0)
    assert static.time_to_collision(0.0) is None


def test_time_to_collision_is_zero_once_predicted_past_contact() -> None:
    track = closing_track()
    track.predict(now=3.5)
    assert track.distance_m < 0.0
    assert track.time_to_collision(0.2) == 0.0


@pytest.mark.parametrize(
    "distance, std",
    [(1.0, 0.0), (1.0, -0.1), (1.0, math.nan), (1.0, math.inf), (math.nan, 0.1), (math.inf, 0.1)],
)
def test_invalid_measurements_are_rejected(distance: float, std: float) -> None:
    with pytest.raises(ValueError):
        RangeTrack(distance, std, now=0.0, accel_std_mps2=2.0)
    track = RangeTrack(1.0, 0.1, now=0.0, accel_std_mps2=2.0)
    with pytest.raises(ValueError):
        track.update(distance, std, now=0.1)


# Sectors


@pytest.mark.parametrize(
    "bearing, sector",
    [
        (-52.5, 0),
        (-45.0, 0),
        (-37.6, 0),
        (-37.5, 1),
        (-7.6, 2),
        (-7.5, 3),
        (0.0, 3),
        (7.4, 3),
        (7.5, 4),
        (45.0, 6),
        (52.5, 6),
    ],
)
def test_sector_for_bearing(obstacle_map: ObstacleMap, bearing: float, sector: int) -> None:
    assert obstacle_map.sector_for_bearing(bearing) == sector


@pytest.mark.parametrize("bearing", [-52.6, 52.6, -90.0, 180.0, math.nan, math.inf, -math.inf])
def test_bearings_outside_the_span_have_no_sector(
    obstacle_map: ObstacleMap, bearing: float
) -> None:
    assert obstacle_map.sector_for_bearing(bearing) is None


def test_sector_centres(obstacle_map: ObstacleMap) -> None:
    centres = [obstacle_map.sector_center(sector) for sector in range(7)]
    assert centres == pytest.approx([-45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0])
    assert obstacle_map.sector_center(CENTRE) == 0.0


@pytest.mark.parametrize("sector", [-1, 7])
def test_sector_centre_rejects_unknown_sectors(obstacle_map: ObstacleMap, sector: int) -> None:
    with pytest.raises(ValueError, match="sector"):
        obstacle_map.sector_center(sector)


def test_sector_geometry_follows_the_config() -> None:
    config = NavSightConfig()
    config.fusion.sector_count = 5
    config.fusion.sector_span_deg = 100.0
    obstacle_map = ObstacleMap(config)
    assert obstacle_map.sector_for_bearing(-50.0) == 0
    assert obstacle_map.sector_for_bearing(0.0) == 2
    assert obstacle_map.sector_for_bearing(50.0) == 4
    assert obstacle_map.sector_for_bearing(50.1) is None
    centres = [obstacle_map.sector_center(sector) for sector in range(5)]
    assert centres == pytest.approx([-40.0, -20.0, 0.0, 20.0, 40.0])
    obstacle_map.update_tof(forward(1.0, 0.0))
    assert [obstacle.sector for obstacle in obstacle_map.obstacles(0.0)] == [2]


# Forward ToF


def test_tof_reading_updates_the_centre_track(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_tof(forward(2.0, 10.0))
    assert obstacle_map.obstacles(10.0) == [
        Obstacle(
            sector=CENTRE,
            bearing_deg=0.0,
            distance_m=2.0,
            closing_speed_mps=0.0,
            time_to_collision_s=None,
            label=None,
            source="tof",
            confidence=1.0,
        )
    ]


def test_tof_readings_refine_the_same_track(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_tof(forward(2.0, 10.0))
    obstacle_map.update_tof(forward(1.98, 10.02))
    [obstacle] = obstacle_map.obstacles(10.02)
    assert 1.98 < obstacle.distance_m < 2.0
    assert obstacle.closing_speed_mps > 0.0


@pytest.mark.parametrize(
    "tof_m, vision_m, tof_std",
    [
        (2.0, 3.0, 0.06),  # 3 % of the range
        (0.2, 0.4, 0.01),  # 3 % would be 6 mm: the 1 cm floor applies
    ],
)
def test_tof_and_vision_are_weighted_by_their_standard_deviations(
    obstacle_map: ObstacleMap, tof_m: float, vision_m: float, tof_std: float
) -> None:
    obstacle_map.update_tof(forward(tof_m, 10.0))
    obstacle_map.update_detections([detection(distance_m=vision_m)], now=10.0)
    vision_std = 0.25 * vision_m
    gain = tof_std**2 / (tof_std**2 + vision_std**2)
    [obstacle] = obstacle_map.obstacles(10.0)
    assert obstacle.distance_m == pytest.approx(tof_m + gain * (vision_m - tof_m))


def test_no_target_removes_a_tof_only_centre_track(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_tof(forward(2.0, 10.0))
    obstacle_map.update_tof(forward(None, 10.02))
    assert obstacle_map.obstacles(10.02) == []


def test_no_target_keeps_a_vision_only_centre_track(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_detections([detection(bearing_deg=1.0, distance_m=3.0)], now=10.0)
    obstacle_map.update_tof(forward(None, 10.02))
    [obstacle] = obstacle_map.obstacles(10.02)
    assert obstacle.source == "vision"
    assert obstacle.distance_m == pytest.approx(3.0)


def test_no_target_removes_a_fused_track_last_updated_by_the_tof(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_detections([detection(bearing_deg=1.0, distance_m=2.1)], now=10.0)
    obstacle_map.update_tof(forward(2.0, 10.02))
    obstacle_map.update_tof(forward(None, 10.04))
    [obstacle] = obstacle_map.obstacles(10.04)
    assert obstacle.distance_m is None
    assert obstacle.label == "person"


@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf])
def test_invalid_tof_distance_counts_as_no_target(obstacle_map: ObstacleMap, bad: float) -> None:
    obstacle_map.update_tof(forward(2.0, 10.0))
    obstacle_map.update_tof(forward(bad, 10.02))
    assert obstacle_map.obstacles(10.02) == []


def test_update_tof_rejects_down_sensor_readings(obstacle_map: ObstacleMap) -> None:
    with pytest.raises(ValueError, match="forward"):
        obstacle_map.update_tof(ToFReading(sensor="down", distance_m=1.0, timestamp=10.0))


# Vision


def test_vision_detection_with_distance_tracks_its_sector(obstacle_map: ObstacleMap) -> None:
    found = detection(bearing_deg=-31.0, distance_m=2.5, label="bicycle", confidence=0.7)
    obstacle_map.update_detections([found], now=10.0)
    assert obstacle_map.obstacles(10.0) == [
        Obstacle(
            sector=1,
            bearing_deg=-30.0,
            distance_m=2.5,
            closing_speed_mps=0.0,
            time_to_collision_s=None,
            label="bicycle",
            source="vision",
            confidence=0.7,
        )
    ]


def test_vision_distance_in_the_centre_fuses_with_the_tof(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_tof(forward(2.0, 10.0))
    seen = detection(bearing_deg=3.0, distance_m=2.2, confidence=0.9)
    obstacle_map.update_detections([seen], now=10.01)
    [obstacle] = obstacle_map.obstacles(10.02)
    assert obstacle.sector == CENTRE
    assert obstacle.source == "fused"
    assert obstacle.label == "person"
    assert obstacle.confidence == 0.9
    assert 2.0 < obstacle.distance_m < 2.05  # the ToF is far more precise than vision


def test_source_returns_to_tof_when_vision_stops(
    obstacle_map: ObstacleMap, config: NavSightConfig
) -> None:
    obstacle_map.update_detections([detection(distance_m=2.0, confidence=0.9)], now=10.0)
    last = approach(obstacle_map, start_m=2.0, speed_mps=0.0, start_t=10.02, samples=40)
    assert last - 10.0 > config.fusion.track_timeout_s
    [obstacle] = obstacle_map.obstacles(last)
    assert obstacle.source == "tof"
    assert obstacle.confidence == 1.0
    assert obstacle.label == "person"  # still within label_timeout_s


def test_source_is_vision_when_the_tof_stops(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_tof(forward(2.0, 10.0))
    for k in range(1, 6):
        obstacle_map.update_detections([detection(distance_m=2.0, confidence=0.6)], 10.0 + 0.2 * k)
    [obstacle] = obstacle_map.obstacles(11.0)
    assert obstacle.source == "vision"
    assert obstacle.confidence == 0.6


def test_detection_without_distance_gives_a_label_only_obstacle(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_detections([detection(bearing_deg=29.0, label="dog", confidence=0.6)], 10.0)
    assert obstacle_map.obstacles(10.5) == [
        Obstacle(
            sector=5,
            bearing_deg=30.0,
            distance_m=None,
            closing_speed_mps=0.0,
            time_to_collision_s=None,
            label="dog",
            source="vision",
            confidence=0.6,
        )
    ]


def test_label_on_a_tof_only_track_keeps_full_confidence(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_tof(forward(1.5, 10.0))
    obstacle_map.update_detections([detection(bearing_deg=2.0, label="pole", confidence=0.5)], 10.0)
    [obstacle] = obstacle_map.obstacles(10.0)
    assert (obstacle.source, obstacle.label, obstacle.confidence) == ("tof", "pole", 1.0)


def test_most_confident_label_wins_within_a_frame(obstacle_map: ObstacleMap) -> None:
    frame = [
        detection(bearing_deg=28.0, label="chair", confidence=0.5),
        detection(bearing_deg=31.0, label="person", confidence=0.9),
        detection(bearing_deg=33.0, label="bench", confidence=0.7),
    ]
    obstacle_map.update_detections(frame, now=10.0)
    [obstacle] = obstacle_map.obstacles(10.0)
    assert (obstacle.label, obstacle.confidence) == ("person", 0.9)


def test_newer_frame_replaces_the_label(obstacle_map: ObstacleMap) -> None:
    person = detection(bearing_deg=30.0, label="person", confidence=0.9)
    chair = detection(bearing_deg=30.0, label="chair", confidence=0.5)
    obstacle_map.update_detections([person], now=10.0)
    obstacle_map.update_detections([chair], now=10.1)
    [obstacle] = obstacle_map.obstacles(10.1)
    assert (obstacle.label, obstacle.confidence) == ("chair", 0.5)


def test_label_survives_missed_frames_until_it_times_out(
    obstacle_map: ObstacleMap, config: NavSightConfig
) -> None:
    timeout = config.fusion.label_timeout_s
    obstacle_map.update_detections([detection(bearing_deg=30.0, label="dog")], now=10.0)
    obstacle_map.update_detections([], now=10.1)
    assert [obstacle.label for obstacle in obstacle_map.obstacles(10.0 + timeout - 0.01)] == ["dog"]
    assert obstacle_map.obstacles(10.0 + timeout + 0.01) == []


def test_nearest_distance_in_a_sector_updates_its_track(obstacle_map: ObstacleMap) -> None:
    frame = [
        detection(bearing_deg=-15.0, distance_m=3.0, label="person", confidence=0.9),
        detection(bearing_deg=-16.0, distance_m=1.5, label="chair", confidence=0.5),
    ]
    obstacle_map.update_detections(frame, now=10.0)
    [obstacle] = obstacle_map.obstacles(10.0)
    assert obstacle.sector == 2
    assert obstacle.distance_m == pytest.approx(1.5)
    assert obstacle.label == "person"


def test_detections_outside_the_span_are_ignored(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_detections([detection(bearing_deg=60.0, distance_m=1.0)], now=10.0)
    assert obstacle_map.obstacles(10.0) == []


@pytest.mark.parametrize("bad", [0.0, -2.0, math.nan, math.inf])
def test_invalid_vision_distance_only_records_the_label(
    obstacle_map: ObstacleMap, bad: float
) -> None:
    obstacle_map.update_detections([detection(bearing_deg=30.0, distance_m=bad)], now=10.0)
    [obstacle] = obstacle_map.obstacles(10.0)
    assert obstacle.distance_m is None
    assert obstacle.label == "person"


# Obstacle list


def test_obstacles_are_sorted_by_distance_with_label_only_last(obstacle_map: ObstacleMap) -> None:
    frame = [
        detection(bearing_deg=-45.0, label="dog"),
        detection(bearing_deg=45.0, distance_m=1.0, label="car"),
    ]
    obstacle_map.update_detections(frame, now=10.0)
    obstacle_map.update_tof(forward(3.0, 10.0))
    found = obstacle_map.obstacles(10.0)
    assert [obstacle.distance_m for obstacle in found] == [1.0, 3.0, None]
    assert [obstacle.sector for obstacle in found] == [6, CENTRE, 0]


def test_nearest_skips_label_only_obstacles(obstacle_map: ObstacleMap) -> None:
    obstacle_map.update_detections([detection(bearing_deg=-45.0, label="dog")], now=10.0)
    assert obstacle_map.nearest(10.0) is None
    obstacle_map.update_tof(forward(3.0, 10.0))
    nearest = obstacle_map.nearest(10.0)
    assert nearest is not None
    assert (nearest.sector, nearest.distance_m) == (CENTRE, 3.0)


def test_empty_map_has_no_obstacles(obstacle_map: ObstacleMap) -> None:
    assert obstacle_map.obstacles(0.0) == []
    assert obstacle_map.nearest(0.0) is None


def test_tracks_expire_after_the_track_timeout(
    obstacle_map: ObstacleMap, config: NavSightConfig
) -> None:
    timeout = config.fusion.track_timeout_s
    obstacle_map.update_tof(forward(2.0, 10.0))
    assert len(obstacle_map.obstacles(10.0 + timeout - 0.01)) == 1
    assert obstacle_map.obstacles(10.0 + timeout + 0.01) == []
    assert obstacle_map.nearest(10.0 + timeout + 0.01) is None


def test_reading_after_expiry_starts_a_fresh_track(
    obstacle_map: ObstacleMap, config: NavSightConfig
) -> None:
    last = approach(obstacle_map, start_m=3.0, speed_mps=1.0, start_t=10.0, samples=50)
    assert obstacle_map.nearest(last).closing_speed_mps > 0.5
    later = last + config.fusion.track_timeout_s + 0.1
    obstacle_map.update_tof(forward(1.5, later))
    nearest = obstacle_map.nearest(later)
    assert nearest.distance_m == 1.5
    assert nearest.closing_speed_mps == 0.0


def test_obstacles_do_not_change_the_stored_tracks(config: NavSightConfig) -> None:
    probed, untouched = ObstacleMap(config), ObstacleMap(config)
    last = approach(probed, start_m=3.0, speed_mps=1.0, start_t=10.0, samples=30)
    approach(untouched, start_m=3.0, speed_mps=1.0, start_t=10.0, samples=30)
    first = probed.obstacles(last)
    probed.obstacles(last + 0.5)
    probed.nearest(last + 0.3)
    assert probed.obstacles(last + 5.0) == []
    assert probed.obstacles(last) == first
    for obstacle_map in (probed, untouched):
        obstacle_map.update_tof(forward(2.4, last + PERIOD))
    assert probed.obstacles(last + PERIOD) == untouched.obstacles(last + PERIOD)


def test_object_approaching_at_1_mps_gives_closing_speed_and_time_to_collision(
    obstacle_map: ObstacleMap,
) -> None:
    last = approach(obstacle_map, start_m=3.0, speed_mps=1.0, start_t=10.0, samples=76)
    nearest = obstacle_map.nearest(last)
    assert nearest.distance_m == pytest.approx(1.5, abs=0.01)
    assert nearest.closing_speed_mps == pytest.approx(1.0, abs=0.02)
    assert nearest.time_to_collision_s == pytest.approx(1.5, abs=0.05)


def test_noisy_approach_still_gives_a_sensible_closing_speed(obstacle_map: ObstacleMap) -> None:
    rng = np.random.default_rng(7)
    for k in range(76):
        true_m = 3.0 - k * PERIOD
        obstacle_map.update_tof(forward(true_m + rng.normal(0.0, 0.03 * true_m), 10.0 + k * PERIOD))
    nearest = obstacle_map.nearest(10.0 + 75 * PERIOD)
    assert nearest.closing_speed_mps == pytest.approx(1.0, abs=0.15)
    assert nearest.time_to_collision_s == pytest.approx(1.5, abs=0.3)


def test_obstacles_are_predicted_to_the_query_time(obstacle_map: ObstacleMap) -> None:
    last = approach(obstacle_map, start_m=3.0, speed_mps=1.0, start_t=10.0, samples=76)
    current = obstacle_map.nearest(last)
    later = obstacle_map.nearest(last + 0.3)
    expected = current.distance_m - 0.3 * current.closing_speed_mps
    assert later.distance_m == pytest.approx(expected)
    assert later.time_to_collision_s < current.time_to_collision_s


def test_predicted_distance_is_never_negative(obstacle_map: ObstacleMap) -> None:
    last = approach(obstacle_map, start_m=0.6, speed_mps=1.0, start_t=10.0, samples=26)
    obstacle = obstacle_map.nearest(last + 0.5)
    assert obstacle.distance_m == 0.0
    assert obstacle.time_to_collision_s == 0.0


@pytest.mark.parametrize(
    "field, value",
    [
        ("sector_count", 0),
        ("sector_span_deg", 0.0),
        ("tof_range_std_min_m", 0.0),
        ("vision_range_std_frac", 0.0),
    ],
)
def test_invalid_config_is_rejected(field: str, value: float) -> None:
    config = NavSightConfig()
    setattr(config.fusion, field, value)
    with pytest.raises(ValueError, match=f"fusion.{field}"):
        ObstacleMap(config)
