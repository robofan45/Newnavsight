from __future__ import annotations

import logging
import math

import numpy as np
import pytest

from navsight.config import NavSightConfig, PoseConfig
from navsight.fusion.ekf import (
    EARTH_RADIUS_M,
    GPS_REJECTIONS_BEFORE_RESET,
    LocalTangentPlane,
    PoseEKF,
    PoseEstimator,
)
from navsight.types import GpsFix, ImuSample
from navsight.util import wrap_angle_deg

ORIGIN_LATITUDE = 51.5007
ORIGIN_LONGITUDE = -0.1246
STEP_M = 0.65  # PoseConfig.step_length_m
HEADING_VARIANCE = math.radians(8.0) ** 2  # PoseConfig.imu_heading_std_deg squared


def imu(timestamp: float, heading_deg: float = 0.0, steps: int = 0) -> ImuSample:
    return ImuSample(
        timestamp=timestamp,
        heading_deg=heading_deg,
        pitch_deg=-45.0,
        roll_deg=0.0,
        accel_mps2=(0.0, 0.0, 9.80665),
        linear_accel_mps2=(0.0, 0.0, 0.0),
        gyro_dps=(0.0, 0.0, 0.0),
        step_count=steps,
        stationary=False,
    )


def gps(
    timestamp: float,
    east_m: float = 0.0,
    north_m: float = 0.0,
    *,
    hdop: float | None = 0.9,
    quality: int = 1,
) -> GpsFix:
    """A fix at an offset from the test origin."""
    plane = LocalTangentPlane(ORIGIN_LATITUDE, ORIGIN_LONGITUDE)
    latitude, longitude = plane.to_geodetic(east_m, north_m)
    return GpsFix(
        timestamp=timestamp,
        latitude=latitude,
        longitude=longitude,
        hdop=hdop,
        satellites_used=12,
        fix_quality=quality,
    )


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    half_dphi = (phi2 - phi1) / 2.0
    half_dlambda = math.radians(lon2 - lon1) / 2.0
    a = math.sin(half_dphi) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(half_dlambda) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


class Walker:
    """Feeds an estimator one IMU sample per step, starting with a standing sample at t = 0."""

    def __init__(self, estimator: PoseEstimator, heading_deg: float = 0.0) -> None:
        self.estimator = estimator
        self.time = 0.0
        self.steps = 0
        estimator.on_imu(imu(self.time, heading_deg, self.steps))

    def walk(self, steps: int, heading_deg: float, interval_s: float = 0.5) -> None:
        for _ in range(steps):
            self.steps += 1
            self.time += interval_s
            self.estimator.on_imu(imu(self.time, heading_deg, self.steps))

    def stand(self, samples: int, heading_deg: float, interval_s: float = 0.01) -> None:
        for _ in range(samples):
            self.time += interval_s
            self.estimator.on_imu(imu(self.time, heading_deg, self.steps))


@pytest.fixture
def ekf() -> PoseEKF:
    """A filter at the origin, heading north, position std 5 m."""
    pose_ekf = PoseEKF(PoseConfig())
    pose_ekf.initialize(0.0, 0.0, 0.0, 5.0)
    return pose_ekf


@pytest.fixture
def estimator(config: NavSightConfig) -> PoseEstimator:
    return PoseEstimator(config)


# LocalTangentPlane


def test_origin_maps_to_zero() -> None:
    plane = LocalTangentPlane(ORIGIN_LATITUDE, ORIGIN_LONGITUDE)
    assert plane.to_enu(ORIGIN_LATITUDE, ORIGIN_LONGITUDE) == (0.0, 0.0)
    assert plane.to_geodetic(0.0, 0.0) == pytest.approx((ORIGIN_LATITUDE, ORIGIN_LONGITUDE))


@pytest.mark.parametrize(
    "east_m, north_m", [(1000.0, 0.0), (0.0, -1000.0), (707.0, 707.0), (-600.0, 800.0)]
)
def test_round_trip_within_a_centimetre_at_one_kilometre(east_m: float, north_m: float) -> None:
    plane = LocalTangentPlane(ORIGIN_LATITUDE, ORIGIN_LONGITUDE)
    latitude, longitude = plane.to_geodetic(east_m, north_m)
    assert plane.to_enu(latitude, longitude) == pytest.approx((east_m, north_m), abs=0.01)
    assert plane.to_geodetic(*plane.to_enu(latitude, longitude)) == pytest.approx(
        (latitude, longitude), abs=1e-9
    )


@pytest.mark.parametrize(
    "east_m, north_m, tolerance_m",
    [(1000.0, 0.0, 0.01), (0.0, 1000.0, 0.01), (-1000.0, 0.0, 0.01), (707.0, 707.0, 0.05)],
)
def test_distances_match_the_great_circle_on_the_wgs84_radius(
    east_m: float, north_m: float, tolerance_m: float
) -> None:
    plane = LocalTangentPlane(ORIGIN_LATITUDE, ORIGIN_LONGITUDE)
    latitude, longitude = plane.to_geodetic(east_m, north_m)
    distance_m = haversine_m(ORIGIN_LATITUDE, ORIGIN_LONGITUDE, latitude, longitude)
    assert distance_m == pytest.approx(math.hypot(east_m, north_m), abs=tolerance_m)


def test_north_offset_uses_the_wgs84_equatorial_radius() -> None:
    plane = LocalTangentPlane(ORIGIN_LATITUDE, ORIGIN_LONGITUDE)
    latitude, longitude = plane.to_geodetic(0.0, 1000.0)
    assert EARTH_RADIUS_M == 6378137.0
    assert latitude == pytest.approx(ORIGIN_LATITUDE + math.degrees(1000.0 / 6378137.0))
    assert longitude == pytest.approx(ORIGIN_LONGITUDE)


def test_longitude_wraps_at_the_antimeridian() -> None:
    plane = LocalTangentPlane(-17.0, 179.9995)
    east_m, north_m = plane.to_enu(-17.0, -179.9995)
    expected_east_m = EARTH_RADIUS_M * math.radians(0.001) * math.cos(math.radians(-17.0))
    assert east_m == pytest.approx(expected_east_m, rel=1e-6)
    assert north_m == pytest.approx(0.0, abs=1e-9)
    assert plane.to_geodetic(east_m, north_m) == pytest.approx((-17.0, -179.9995))


@pytest.mark.parametrize(
    "latitude, longitude",
    [(90.0, 0.0), (-90.0, 0.0), (91.0, 0.0), (math.nan, 0.0), (0.0, math.inf)],
)
def test_invalid_origin_is_rejected(latitude: float, longitude: float) -> None:
    with pytest.raises(ValueError, match="origin"):
        LocalTangentPlane(latitude, longitude)


# PoseEKF


def test_initialize_sets_state_and_covariance() -> None:
    pose_ekf = PoseEKF(PoseConfig())
    assert not pose_ekf.initialized
    pose_ekf.initialize(3.0, -4.0, 90.0, 2.0)
    assert pose_ekf.initialized
    np.testing.assert_allclose(pose_ekf.state, [3.0, -4.0, math.pi / 2])
    np.testing.assert_allclose(pose_ekf.covariance, np.diag([4.0, 4.0, HEADING_VARIANCE]))
    assert pose_ekf.heading_deg == pytest.approx(90.0)
    assert pose_ekf.position_std_m == pytest.approx(2.0)


@pytest.mark.parametrize("heading_deg, expected", [(-90.0, 270.0), (720.0, 0.0), (359.5, 359.5)])
def test_initial_heading_is_wrapped(heading_deg: float, expected: float) -> None:
    pose_ekf = PoseEKF(PoseConfig())
    pose_ekf.initialize(0.0, 0.0, heading_deg, 1.0)
    assert pose_ekf.heading_deg == pytest.approx(expected)
    assert 0.0 <= pose_ekf.state[2] < 2 * math.pi


@pytest.mark.parametrize(
    "heading_deg, east_m, north_m",
    [(0.0, 0.0, 6.5), (90.0, 6.5, 0.0), (180.0, 0.0, -6.5), (270.0, -6.5, 0.0)],
)
def test_ten_steps_dead_reckon_along_the_heading(
    ekf: PoseEKF, heading_deg: float, east_m: float, north_m: float
) -> None:
    for _ in range(10):
        ekf.predict_step(STEP_M, heading_deg)
    assert ekf.state[0] == pytest.approx(east_m, abs=1e-9)
    assert ekf.state[1] == pytest.approx(north_m, abs=1e-9)
    assert ekf.heading_deg == pytest.approx(heading_deg)


def test_predict_step_first_takes_the_imu_heading(ekf: PoseEKF) -> None:
    ekf.predict_step(1.0, 90.0)
    np.testing.assert_allclose(ekf.state, [1.0, 0.0, math.pi / 2], atol=1e-12)


def test_predict_step_covariance_follows_the_contract() -> None:
    pose_ekf = PoseEKF(PoseConfig())
    pose_ekf.initialize(0.0, 0.0, 30.0, 2.0)
    pose_ekf.predict_step(0.6, 45.0)  # builds cross-covariances first
    before = pose_ekf.covariance
    length, psi = 0.7, math.radians(120.0)
    pose_ekf.predict_step(length, 120.0)

    sin_psi, cos_psi = math.sin(psi), math.cos(psi)
    transition = np.array(
        [[1.0, 0.0, length * cos_psi], [0.0, 1.0, -length * sin_psi], [0.0, 0.0, 1.0]]
    )
    jacobian = np.array([[sin_psi, length * cos_psi], [cos_psi, -length * sin_psi], [0.0, 1.0]])
    step_noise = np.diag([(0.10 * length) ** 2, math.radians(2.0) ** 2])
    expected = transition @ before @ transition.T + jacobian @ step_noise @ jacobian.T
    np.testing.assert_allclose(pose_ekf.covariance, expected, rtol=1e-12, atol=1e-15)


def test_position_uncertainty_grows_while_dead_reckoning(ekf: PoseEKF) -> None:
    previous = ekf.position_std_m
    for _ in range(20):
        ekf.predict_step(STEP_M, 45.0)
        assert ekf.position_std_m > previous
        previous = ekf.position_std_m


def test_gps_update_pulls_toward_the_fix_and_shrinks_the_covariance(ekf: PoseEKF) -> None:
    assert ekf.update_gps(3.0, 4.0, 2.0)
    gain = 25.0 / (25.0 + 4.0)
    np.testing.assert_allclose(ekf.state[:2], [3.0 * gain, 4.0 * gain])
    np.testing.assert_allclose(ekf.covariance[:2, :2], np.eye(2) * 25.0 * 4.0 / 29.0)
    assert ekf.position_std_m == pytest.approx(math.sqrt(100.0 / 29.0))
    assert abs(wrap_angle_deg(ekf.heading_deg)) < 1e-9


def test_gate_rejects_a_100_m_outlier_without_changing_the_filter(ekf: PoseEKF) -> None:
    state, covariance = ekf.state, ekf.covariance
    assert not ekf.update_gps(100.0, 0.0, 1.35)
    np.testing.assert_array_equal(ekf.state, state)
    np.testing.assert_array_equal(ekf.covariance, covariance)


@pytest.mark.parametrize("distance_sq, accepted", [(9.0, True), (9.2, True), (9.3, False)])
def test_gate_is_the_mahalanobis_distance_against_chi2(
    ekf: PoseEKF, distance_sq: float, accepted: bool
) -> None:
    # Position variance 25 plus GPS variance 4 gives S = 29 I, so d^2 = |offset|^2 / 29.
    offset_m = math.sqrt(distance_sq * 29.0) / math.sqrt(2.0)
    assert ekf.update_gps(offset_m, -offset_m, 2.0) is accepted


def test_gate_threshold_comes_from_the_config() -> None:
    pose_ekf = PoseEKF(PoseConfig(gps_gate_chi2=4.0))
    pose_ekf.initialize(0.0, 0.0, 0.0, 5.0)
    assert not pose_ekf.update_gps(math.sqrt(5.0 * 29.0), 0.0, 2.0)
    assert pose_ekf.update_gps(math.sqrt(3.0 * 29.0), 0.0, 2.0)


@pytest.mark.parametrize(
    "estimate, measurement, expected",
    [(359.0, 1.0, 359.5), (1.0, 359.0, 0.5), (10.0, 20.0, 12.5), (180.0, 170.0, 177.5)],
)
def test_heading_update_takes_the_short_way_round(
    estimate: float, measurement: float, expected: float
) -> None:
    pose_ekf = PoseEKF(PoseConfig())
    pose_ekf.initialize(0.0, 0.0, estimate, 5.0)
    pose_ekf.update_heading(measurement, 8.0 * math.sqrt(3.0))  # R = 3 P, so the gain is 0.25
    assert pose_ekf.heading_deg == pytest.approx(expected)


def test_heading_update_across_north_wraps_and_halves_the_variance() -> None:
    pose_ekf = PoseEKF(PoseConfig())
    pose_ekf.initialize(0.0, 0.0, 359.0, 5.0)
    pose_ekf.update_heading(1.0, 8.0)  # equal variances: gain 0.5, lands exactly on north
    assert abs(wrap_angle_deg(pose_ekf.heading_deg)) < 1e-9
    assert 0.0 <= pose_ekf.state[2] < 2 * math.pi
    assert pose_ekf.covariance[2, 2] == pytest.approx(HEADING_VARIANCE / 2.0)


def test_gps_corrects_heading_through_the_step_cross_covariance() -> None:
    pose_ekf = PoseEKF(PoseConfig())
    pose_ekf.initialize(0.0, 0.0, 0.0, 1.0)
    for _ in range(10):
        pose_ekf.predict_step(STEP_M, 0.0)
    assert pose_ekf.covariance[0, 2] > 0.0  # F's L cos(psi) term couples east and heading
    assert pose_ekf.update_gps(1.0, 6.5, 1.0)  # a fix east of the track turns the heading east
    assert 0.0 < pose_ekf.heading_deg < 10.0


def test_covariance_stays_symmetric_positive_definite_over_mixed_updates() -> None:
    rng = np.random.default_rng(7)
    pose_ekf = PoseEKF(PoseConfig())
    pose_ekf.initialize(0.0, 0.0, 0.0, 5.0)
    outcomes = set()
    for _ in range(3000):
        action = rng.integers(3)
        if action == 0:
            pose_ekf.predict_step(rng.uniform(0.3, 0.9), rng.uniform(0.0, 360.0))
        elif action == 1:
            pose_ekf.update_heading(rng.uniform(0.0, 360.0), rng.uniform(1.0, 20.0))
        else:
            east_m, north_m = rng.normal(0.0, 30.0, size=2)
            outcomes.add(pose_ekf.update_gps(east_m, north_m, rng.uniform(1.0, 10.0)))
        covariance = pose_ekf.covariance
        assert np.array_equal(covariance, covariance.T)
        assert np.linalg.eigvalsh(covariance)[0] > 0.0
        assert 0.0 <= pose_ekf.state[2] < 2 * math.pi
    assert outcomes == {True, False}  # both accepted and gated GPS updates were exercised


def test_state_and_covariance_are_copies(ekf: PoseEKF) -> None:
    ekf.state[0] = 99.0
    ekf.covariance[0, 0] = -1.0
    assert ekf.state[0] == 0.0
    assert ekf.covariance[0, 0] == 25.0


@pytest.mark.parametrize(
    "call",
    [
        lambda f: f.predict_step(STEP_M, 0.0),
        lambda f: f.update_heading(0.0, 8.0),
        lambda f: f.update_gps(0.0, 0.0, 1.0),
    ],
)
def test_uninitialized_filter_refuses_to_run(call) -> None:
    with pytest.raises(RuntimeError, match="initialize"):
        call(PoseEKF(PoseConfig()))


@pytest.mark.parametrize(
    "call, name",
    [
        (lambda f: f.initialize(0.0, 0.0, 0.0, 0.0), "position_std_m"),
        (lambda f: f.initialize(math.nan, 0.0, 0.0, 1.0), "east_m"),
        (lambda f: f.predict_step(-0.1, 0.0), "step_length_m"),
        (lambda f: f.predict_step(STEP_M, math.nan), "heading_deg"),
        (lambda f: f.update_heading(math.inf, 8.0), "heading_deg"),
        (lambda f: f.update_heading(0.0, 0.0), "std_deg"),
        (lambda f: f.update_gps(0.0, math.nan, 1.0), "north_m"),
        (lambda f: f.update_gps(0.0, 0.0, -1.0), "std_m"),
    ],
)
def test_invalid_inputs_raise_and_leave_the_filter_unchanged(ekf: PoseEKF, call, name) -> None:
    state, covariance = ekf.state, ekf.covariance
    with pytest.raises(ValueError, match=name):
        call(ekf)
    np.testing.assert_array_equal(ekf.state, state)
    np.testing.assert_array_equal(ekf.covariance, covariance)


@pytest.mark.parametrize(
    "field, value",
    [
        ("step_length_m", 0.0),
        ("step_length_std_frac", -0.1),
        ("heading_std_per_step_deg", -1.0),
        ("imu_heading_std_deg", 0.0),
        ("gps_min_std_m", 0.0),
        ("gps_gate_chi2", 0.0),
        ("gps_timeout_s", -1.0),
        ("initial_position_std_m", math.nan),
        ("max_steps_per_update", 0),
    ],
)
def test_invalid_pose_config_is_rejected_at_construction(field: str, value: float) -> None:
    with pytest.raises(ValueError, match=f"pose.{field}"):
        PoseEKF(PoseConfig(**{field: value}))
    config = NavSightConfig()
    setattr(config.pose, field, value)
    with pytest.raises(ValueError, match=f"pose.{field}"):
        PoseEstimator(config)


# PoseEstimator: dead reckoning


def test_no_pose_before_the_first_imu_sample(estimator: PoseEstimator) -> None:
    assert estimator.pose(0.0) is None
    assert estimator.using_dead_reckoning(0.0)
    assert not estimator.on_gps(gps(0.0))  # no heading yet, so the fix is not used
    assert estimator.pose(0.0) is None


def test_first_imu_sample_starts_dead_reckoning_at_the_origin(estimator: PoseEstimator) -> None:
    estimator.on_imu(imu(1.0, heading_deg=45.0, steps=1234))
    pose = estimator.pose(1.5)
    assert pose is not None
    assert pose.timestamp == 1.5
    assert (pose.east_m, pose.north_m) == (0.0, 0.0)
    assert pose.heading_deg == 45.0
    assert pose.position_std_m == pytest.approx(5.0)  # initial_position_std_m
    assert pose.latitude is None and pose.longitude is None
    assert pose.source == "dead_reckoning"


def test_dead_reckoning_before_gps(estimator: PoseEstimator) -> None:
    walker = Walker(estimator)
    walker.walk(10, heading_deg=0.0)
    pose = estimator.pose(walker.time)
    assert pose is not None
    assert pose.north_m == pytest.approx(6.5)
    assert pose.east_m == pytest.approx(0.0, abs=1e-9)
    assert pose.latitude is None and pose.longitude is None
    assert pose.source == "dead_reckoning"
    assert estimator.using_dead_reckoning(walker.time)


def test_steps_follow_each_samples_heading(estimator: PoseEstimator) -> None:
    walker = Walker(estimator)
    walker.walk(5, heading_deg=0.0)
    walker.walk(5, heading_deg=90.0)
    pose = estimator.pose(walker.time)
    assert pose is not None
    assert (pose.east_m, pose.north_m) == pytest.approx((3.25, 3.25))
    assert pose.heading_deg == 90.0


def test_several_steps_in_one_sample_are_all_predicted(estimator: PoseEstimator) -> None:
    estimator.on_imu(imu(0.0, 90.0, 100))
    estimator.on_imu(imu(1.0, 90.0, 103))
    estimator.on_imu(imu(2.0, 90.0, 107))
    pose = estimator.pose(2.0)
    assert pose is not None
    assert pose.east_m == pytest.approx(7 * STEP_M)


def test_decreasing_step_count_resets_the_baseline_without_moving(
    estimator: PoseEstimator, caplog: pytest.LogCaptureFixture
) -> None:
    estimator.on_imu(imu(0.0, 0.0, 0))
    estimator.on_imu(imu(1.0, 0.0, 5))
    with caplog.at_level(logging.INFO, logger="navsight.fusion.ekf"):
        estimator.on_imu(imu(2.0, 0.0, 2))
    assert "Step count fell from 5 to 2" in caplog.text
    pose = estimator.pose(2.0)
    assert pose is not None
    assert pose.north_m == pytest.approx(5 * STEP_M)
    estimator.on_imu(imu(3.0, 0.0, 4))  # counts again from the new baseline of 2
    pose = estimator.pose(3.0)
    assert pose is not None
    assert pose.north_m == pytest.approx(7 * STEP_M)


def test_max_steps_per_update_caps_a_burst(
    estimator: PoseEstimator, caplog: pytest.LogCaptureFixture
) -> None:
    estimator.on_imu(imu(0.0, 0.0, 0))
    with caplog.at_level(logging.WARNING, logger="navsight.fusion.ekf"):
        estimator.on_imu(imu(1.0, 0.0, 500))
    assert "jumped by 500" in caplog.text
    pose = estimator.pose(1.0)
    assert pose is not None
    assert pose.north_m == pytest.approx(10 * STEP_M)  # max_steps_per_update = 10
    estimator.on_imu(imu(2.0, 0.0, 501))  # the rest of the burst is dropped, not carried over
    pose = estimator.pose(2.0)
    assert pose is not None
    assert pose.north_m == pytest.approx(11 * STEP_M)


def test_max_steps_per_update_comes_from_the_config(config: NavSightConfig) -> None:
    config.pose.max_steps_per_update = 3
    estimator = PoseEstimator(config)
    estimator.on_imu(imu(0.0, 0.0, 0))
    estimator.on_imu(imu(1.0, 0.0, 50))
    pose = estimator.pose(1.0)
    assert pose is not None
    assert pose.north_m == pytest.approx(3 * STEP_M)


def test_step_length_comes_from_the_config(config: NavSightConfig) -> None:
    config.pose.step_length_m = 0.8
    estimator = PoseEstimator(config)
    Walker(estimator).walk(10, heading_deg=0.0)
    pose = estimator.pose(5.0)
    assert pose is not None
    assert pose.north_m == pytest.approx(8.0)


def test_each_step_sample_predicts_then_updates_the_heading(estimator: PoseEstimator) -> None:
    estimator.on_imu(imu(0.0, 10.0, 0))
    estimator.on_imu(imu(0.5, 20.0, 2))
    expected = PoseEKF(PoseConfig())
    expected.initialize(0.0, 0.0, 10.0, 5.0)
    expected.predict_step(STEP_M, 20.0)
    expected.predict_step(STEP_M, 20.0)
    expected.update_heading(20.0, 8.0)
    np.testing.assert_allclose(estimator._ekf.state, expected.state)
    np.testing.assert_allclose(estimator._ekf.covariance, expected.covariance)


def test_turning_on_the_spot_neither_moves_nor_changes_the_filter(
    estimator: PoseEstimator,
) -> None:
    walker = Walker(estimator)
    walker.walk(20, heading_deg=0.0)
    state, covariance = estimator._ekf.state, estimator._ekf.covariance
    for heading_deg in range(0, 91, 3):
        walker.stand(1, heading_deg=float(heading_deg))
    walker.stand(150, heading_deg=90.0)
    np.testing.assert_array_equal(estimator._ekf.state, state)
    np.testing.assert_array_equal(estimator._ekf.covariance, covariance)
    pose = estimator.pose(walker.time)
    assert pose is not None
    assert pose.heading_deg == 90.0  # the pose reports the IMU heading straight away
    walker.walk(1, heading_deg=90.0)  # and the next step goes the new way
    pose = estimator.pose(walker.time)
    assert pose is not None
    assert (pose.east_m, pose.north_m) == pytest.approx((STEP_M, 20 * STEP_M))


def test_a_square_walk_with_turns_on_the_spot_closes(estimator: PoseEstimator) -> None:
    walker = Walker(estimator)
    for corner in range(4):
        heading_deg = 90.0 * corner
        walker.walk(20, heading_deg=heading_deg)
        for turn_deg in range(0, 91, 10):
            walker.stand(3, heading_deg=heading_deg + turn_deg)
    pose = estimator.pose(walker.time)
    assert pose is not None
    assert math.hypot(pose.east_m, pose.north_m) < 0.01


def test_invalid_imu_heading_is_skipped_and_logged_once(
    estimator: PoseEstimator, caplog: pytest.LogCaptureFixture
) -> None:
    estimator.on_imu(imu(0.0, math.nan, 0))
    assert estimator.pose(0.0) is None  # an invalid first sample does not start the filter
    estimator.on_imu(imu(1.0, 0.0, 0))
    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="navsight.fusion.ekf"):
        estimator.on_imu(imu(2.0, math.nan, 3))
        estimator.on_imu(imu(3.0, math.inf, 3))
    assert caplog.text.count("invalid heading") == 1  # once per run of bad samples
    pose = estimator.pose(3.0)
    assert pose is not None
    assert pose.north_m == 0.0
    estimator.on_imu(imu(4.0, 0.0, 3))  # the steps are applied with the next valid sample
    pose = estimator.pose(4.0)
    assert pose is not None
    assert pose.north_m == pytest.approx(3 * STEP_M)


# PoseEstimator: GPS


def test_first_fix_sets_the_origin_and_reports_gps(estimator: PoseEstimator) -> None:
    walker = Walker(estimator)
    walker.walk(10, heading_deg=30.0)
    assert estimator.on_gps(gps(walker.time))
    pose = estimator.pose(walker.time)
    assert pose is not None
    assert pose.source == "gps"
    assert not estimator.using_dead_reckoning(walker.time)
    assert (pose.east_m, pose.north_m) == (0.0, 0.0)  # re-initialised at the fix
    assert pose.latitude == pytest.approx(ORIGIN_LATITUDE)
    assert pose.longitude == pytest.approx(ORIGIN_LONGITUDE)
    assert pose.position_std_m == pytest.approx(0.9 * 1.5)  # hdop * uere_m
    assert estimator._ekf.heading_deg == pytest.approx(30.0)  # heading kept


def test_walking_after_the_first_fix_moves_latitude_and_longitude(
    estimator: PoseEstimator,
) -> None:
    walker = Walker(estimator)
    assert estimator.on_gps(gps(0.0))
    walker.walk(10, heading_deg=90.0)
    pose = estimator.pose(walker.time)
    assert pose is not None
    plane = LocalTangentPlane(ORIGIN_LATITUDE, ORIGIN_LONGITUDE)
    assert (pose.latitude, pose.longitude) == pytest.approx(plane.to_geodetic(6.5, 0.0))


@pytest.mark.parametrize("hdop, std_m", [(0.5, 1.0), (2.0, 3.0), (5.0, 7.5)])
def test_gps_sigma_is_hdop_times_uere_with_a_floor(
    estimator: PoseEstimator, hdop: float, std_m: float
) -> None:
    Walker(estimator)
    assert estimator.on_gps(gps(1.0, hdop=hdop))  # hdop == max_hdop is still used
    pose = estimator.pose(1.0)
    assert pose is not None
    assert pose.position_std_m == pytest.approx(std_m)


def test_source_returns_to_dead_reckoning_after_the_gps_timeout(
    estimator: PoseEstimator,
) -> None:
    Walker(estimator)
    assert estimator.on_gps(gps(10.0))
    within = estimator.pose(20.0)  # exactly gps_timeout_s later still counts
    assert within is not None
    assert within.source == "gps"
    later = estimator.pose(20.01)
    assert later is not None
    assert later.source == "dead_reckoning"
    assert estimator.using_dead_reckoning(20.01)
    assert later.latitude == pytest.approx(ORIGIN_LATITUDE)  # the origin still exists
    assert estimator.on_gps(gps(25.0, 0.5, 0.5))
    recovered = estimator.pose(25.0)
    assert recovered is not None
    assert recovered.source == "gps"


def test_gps_timeout_comes_from_the_config(config: NavSightConfig) -> None:
    config.pose.gps_timeout_s = 2.0
    estimator = PoseEstimator(config)
    Walker(estimator)
    assert estimator.on_gps(gps(1.0))
    assert not estimator.using_dead_reckoning(3.0)
    assert estimator.using_dead_reckoning(3.1)


def test_later_fix_pulls_the_estimate_toward_it(estimator: PoseEstimator) -> None:
    Walker(estimator)
    assert estimator.on_gps(gps(1.0))
    assert estimator.on_gps(gps(2.0, east_m=2.0))  # same sigma as the origin fix: gain 0.5
    pose = estimator.pose(2.0)
    assert pose is not None
    assert pose.east_m == pytest.approx(1.0, abs=1e-6)
    assert pose.position_std_m < 0.9 * 1.5


@pytest.mark.parametrize(
    "fix",
    [
        gps(1.0, hdop=5.01),
        gps(1.0, hdop=-1.0),
        gps(1.0, quality=0),
        GpsFix(timestamp=1.0, latitude=None, longitude=None, hdop=0.9, fix_quality=1),
        GpsFix(timestamp=1.0, latitude=91.0, longitude=0.0, hdop=0.9, fix_quality=1),
        GpsFix(timestamp=1.0, latitude=10.0, longitude=-181.0, hdop=0.9, fix_quality=1),
    ],
)
def test_unusable_fixes_are_ignored(estimator: PoseEstimator, fix: GpsFix) -> None:
    Walker(estimator).walk(2, heading_deg=0.0)
    assert not estimator.on_gps(fix)
    pose = estimator.pose(1.0)
    assert pose is not None
    assert pose.latitude is None
    assert pose.source == "dead_reckoning"
    assert pose.north_m == pytest.approx(2 * STEP_M)


def test_fixes_without_hdop_are_ignored_with_one_warning(
    estimator: PoseEstimator, caplog: pytest.LogCaptureFixture
) -> None:
    Walker(estimator)
    with caplog.at_level(logging.WARNING, logger="navsight.fusion.ekf"):
        for second in range(3):
            assert not estimator.on_gps(gps(float(second), hdop=None))
    assert caplog.text.count("without HDOP") == 1
    assert estimator.using_dead_reckoning(3.0)


def test_ignored_fix_does_not_become_the_origin(estimator: PoseEstimator) -> None:
    Walker(estimator)
    assert not estimator.on_gps(gps(1.0, east_m=50.0, hdop=9.0))
    assert estimator.on_gps(gps(2.0, east_m=-20.0))
    pose = estimator.pose(2.0)
    assert pose is not None
    assert (pose.east_m, pose.north_m) == (0.0, 0.0)
    plane = LocalTangentPlane(ORIGIN_LATITUDE, ORIGIN_LONGITUDE)
    assert (pose.latitude, pose.longitude) == pytest.approx(plane.to_geodetic(-20.0, 0.0))


def test_gate_rejects_a_100_m_outlier(estimator: PoseEstimator) -> None:
    Walker(estimator)
    assert estimator.on_gps(gps(1.0))
    assert not estimator.on_gps(gps(2.0, east_m=100.0))
    pose = estimator.pose(2.0)
    assert pose is not None
    assert (pose.east_m, pose.north_m) == pytest.approx((0.0, 0.0), abs=1e-6)
    assert pose.source == "gps"  # the origin fix is still recent


def test_three_rejections_in_a_row_reset_the_position_to_the_fix(
    estimator: PoseEstimator, caplog: pytest.LogCaptureFixture
) -> None:
    assert GPS_REJECTIONS_BEFORE_RESET == 3
    walker = Walker(estimator, heading_deg=200.0)
    assert estimator.on_gps(gps(0.0))
    outlier = gps(20.0, east_m=100.0, north_m=-30.0, hdop=2.0)
    assert not estimator.on_gps(outlier)
    assert not estimator.on_gps(outlier)
    pose = estimator.pose(20.0)
    assert pose is not None
    assert (pose.east_m, pose.north_m) == pytest.approx((0.0, 0.0), abs=1e-6)
    assert pose.source == "dead_reckoning"  # nothing accepted since t = 0
    with caplog.at_level(logging.INFO, logger="navsight.fusion.ekf"):
        assert estimator.on_gps(outlier)
    assert "3 times in a row" in caplog.text
    pose = estimator.pose(20.0)
    assert pose is not None
    assert (pose.east_m, pose.north_m) == pytest.approx((100.0, -30.0), abs=1e-6)
    assert pose.position_std_m == pytest.approx(3.0)  # the fix's sigma
    assert pose.source == "gps"
    assert estimator._ekf.heading_deg == pytest.approx(200.0)  # heading kept
    walker.walk(1, heading_deg=200.0)
    assert not estimator.on_gps(gps(walker.time, east_m=-100.0))  # the count started again


def test_an_accepted_fix_restarts_the_rejection_count(estimator: PoseEstimator) -> None:
    Walker(estimator)
    assert estimator.on_gps(gps(1.0))
    assert not estimator.on_gps(gps(2.0, east_m=100.0))
    assert not estimator.on_gps(gps(3.0, east_m=100.0))
    assert estimator.on_gps(gps(4.0, east_m=0.5))
    assert not estimator.on_gps(gps(5.0, east_m=100.0))
    assert not estimator.on_gps(gps(6.0, east_m=100.0))
    pose = estimator.pose(6.0)
    assert pose is not None
    assert abs(pose.east_m) < 1.0
    assert estimator.on_gps(gps(7.0, east_m=100.0))


def test_ignored_fixes_do_not_break_a_rejection_streak(estimator: PoseEstimator) -> None:
    Walker(estimator)
    assert estimator.on_gps(gps(1.0))
    assert not estimator.on_gps(gps(2.0, east_m=100.0))
    assert not estimator.on_gps(gps(3.0, east_m=100.0))
    assert not estimator.on_gps(gps(4.0, quality=0))
    assert estimator.on_gps(gps(5.0, east_m=100.0))
    pose = estimator.pose(5.0)
    assert pose is not None
    assert pose.east_m == pytest.approx(100.0, abs=1e-6)


def test_consistent_gps_bounds_the_drift_of_a_mismatched_stride(config: NavSightConfig) -> None:
    rng = np.random.default_rng(3)
    estimator = PoseEstimator(config)
    walker = Walker(estimator)
    assert estimator.on_gps(gps(0.0))
    accepted = 0
    for _ in range(100):
        walker.walk(2, heading_deg=0.0)  # dead reckons 0.65 m per step
        true_north_m = walker.steps * 0.70  # the user's real stride is longer
        east_noise, north_noise = rng.normal(0.0, 1.35, size=2)
        accepted += estimator.on_gps(gps(walker.time, east_noise, true_north_m + north_noise))
    pose = estimator.pose(walker.time)
    assert pose is not None
    assert accepted >= 90  # the gate does not lock out a GPS that agrees with the walk
    assert abs(pose.north_m - 140.0) < 3.0  # dead reckoning alone would be 10 m short
    assert abs(pose.east_m) < 3.0
    assert pose.source == "gps"
