from __future__ import annotations

import contextlib
import logging
import math
import sys
import types

import numpy as np
import pytest

from navsight.config import NavSightConfig
from navsight.perception.depth import (
    DepthEstimator,
    attach_distances,
    metric_distance_for_box,
    tof_anchor_half_width_px,
)
from navsight.perception.detector import focal_length_px
from navsight.types import Detection
from navsight.util import ManualClock

# With the defaults (640 px, 102 deg camera, 27 deg ToF) the anchor half-width is 62.2 px, so
# the anchor window covers columns 257..382 and rows 177..302 of a 640x480 depth map.
ANCHOR_ROWS = slice(177, 303)
ANCHOR_COLUMNS = slice(257, 383)
LEFT_BOX = (40.0, 100.0, 160.0, 300.0)  # its central 50 %: rows 150..249, columns 70..129
RIGHT_BOX = (480.0, 100.0, 600.0, 300.0)  # its central 50 %: rows 150..249, columns 510..569


def depth_map(value: float = 2.0) -> np.ndarray:
    return np.full((480, 640), value, dtype=np.float32)


def fill_box(inv_depth: np.ndarray, box: tuple[float, float, float, float], value: float) -> None:
    x1, y1, x2, y2 = (int(v) for v in box)
    inv_depth[y1:y2, x1:x2] = value


class FakeMidas:
    def __init__(self, output) -> None:
        self.output = output
        self.batches: list = []

    def __call__(self, batch):
        self.batches.append(batch)
        return self.output


class FakeTransform:
    def __init__(self) -> None:
        self.images: list[np.ndarray] = []
        self.contiguous: list[bool] = []

    def __call__(self, image: np.ndarray) -> str:
        self.images.append(image.copy())
        self.contiguous.append(bool(image.flags["C_CONTIGUOUS"]))
        return "batch"


def identity(image: np.ndarray) -> np.ndarray:
    return image


# tof_anchor_half_width_px ------------------------------------------------------------------


def test_anchor_half_width_is_the_focal_length_times_tan_half_the_tof_fov() -> None:
    focal = 320.0 / math.tan(math.radians(51.0))
    half_width = tof_anchor_half_width_px(640, 102.0, 27.0)
    assert half_width == pytest.approx(focal * math.tan(math.radians(13.5)))
    assert half_width == pytest.approx(62.21, abs=0.01)
    assert half_width == pytest.approx(focal_length_px(640, 102.0) * math.tan(math.radians(13.5)))


def test_anchor_half_width_is_half_the_image_when_both_fields_of_view_match() -> None:
    assert tof_anchor_half_width_px(640, 90.0, 90.0) == pytest.approx(320.0)


def test_anchor_half_width_scales_with_the_image_width() -> None:
    assert tof_anchor_half_width_px(1280, 102.0, 27.0) == pytest.approx(
        2 * tof_anchor_half_width_px(640, 102.0, 27.0)
    )


@pytest.mark.parametrize(
    "width, hfov, tof_fov", [(640, 102.0, 0.0), (640, 102.0, 180.0), (0, 102.0, 27.0)]
)
def test_anchor_half_width_rejects_impossible_geometry(
    width: int, hfov: float, tof_fov: float
) -> None:
    with pytest.raises(ValueError):
        tof_anchor_half_width_px(width, hfov, tof_fov)


# metric_distance_for_box -------------------------------------------------------------------


def test_distance_doubles_when_the_box_inverse_depth_is_half_the_anchors(
    config: NavSightConfig,
) -> None:
    inv_depth = depth_map(2.0)
    fill_box(inv_depth, LEFT_BOX, 1.0)
    assert metric_distance_for_box(inv_depth, LEFT_BOX, 1.5, config) == pytest.approx(3.0)


def test_distance_halves_when_the_box_inverse_depth_is_twice_the_anchors(
    config: NavSightConfig,
) -> None:
    inv_depth = depth_map(2.0)
    fill_box(inv_depth, RIGHT_BOX, 4.0)
    assert metric_distance_for_box(inv_depth, RIGHT_BOX, 1.5, config) == pytest.approx(0.75)


def test_box_at_the_anchors_depth_is_at_the_anchor_distance(config: NavSightConfig) -> None:
    assert metric_distance_for_box(depth_map(3.0), LEFT_BOX, 2.4, config) == pytest.approx(2.4)


def test_medians_cover_the_central_half_of_the_box_and_the_tof_square(
    config: NavSightConfig,
) -> None:
    rng = np.random.default_rng(7)
    inv_depth = rng.uniform(0.5, 4.0, size=(480, 640)).astype(np.float32)
    box = (100.0, 100.0, 200.0, 200.0)
    inv_box = np.median(inv_depth[125:175, 125:175])
    inv_anchor = np.median(inv_depth[ANCHOR_ROWS, ANCHOR_COLUMNS])
    assert metric_distance_for_box(inv_depth, box, 2.0, config) == pytest.approx(
        2.0 * inv_anchor / inv_box
    )


def test_the_outer_quarter_of_a_box_is_ignored(config: NavSightConfig) -> None:
    inv_depth = depth_map(2.0)
    box = (100.0, 100.0, 200.0, 200.0)
    fill_box(inv_depth, box, 0.0)  # the box edges: background seen around the object
    inv_depth[125:175, 125:175] = 1.0  # the central 50 % of the box
    assert metric_distance_for_box(inv_depth, box, 1.0, config) == pytest.approx(2.0)


def test_only_the_tof_square_sets_the_anchor(config: NavSightConfig) -> None:
    inv_depth = depth_map(1.0)
    inv_depth[ANCHOR_ROWS, ANCHOR_COLUMNS] = 4.0
    assert metric_distance_for_box(inv_depth, LEFT_BOX, 0.5, config) == pytest.approx(2.0)


@pytest.mark.parametrize("anchor", [None, 0.0, -1.0, math.nan, math.inf])
def test_no_distance_without_a_valid_anchor(config: NavSightConfig, anchor: float | None) -> None:
    assert metric_distance_for_box(depth_map(), LEFT_BOX, anchor, config) is None


@pytest.mark.parametrize("value", [0.0, -1.0, math.nan])
def test_no_distance_when_the_box_median_is_not_positive(
    config: NavSightConfig, value: float
) -> None:
    inv_depth = depth_map(2.0)
    fill_box(inv_depth, LEFT_BOX, value)
    assert metric_distance_for_box(inv_depth, LEFT_BOX, 1.5, config) is None


@pytest.mark.parametrize("value", [0.0, -1.0])
def test_no_distance_when_the_anchor_median_is_not_positive(
    config: NavSightConfig, value: float
) -> None:
    inv_depth = depth_map(2.0)
    inv_depth[ANCHOR_ROWS, ANCHOR_COLUMNS] = value
    assert metric_distance_for_box(inv_depth, LEFT_BOX, 1.5, config) is None


def test_distances_are_clamped_to_the_depth_limits(config: NavSightConfig) -> None:
    inv_depth = depth_map(2.0)
    fill_box(inv_depth, LEFT_BOX, 0.01)  # 200 times farther than the anchor
    assert metric_distance_for_box(inv_depth, LEFT_BOX, 1.0, config) == 10.0
    fill_box(inv_depth, LEFT_BOX, 400.0)  # 200 times closer
    assert metric_distance_for_box(inv_depth, LEFT_BOX, 1.0, config) == 0.2
    config.depth.min_distance_m = 0.5
    config.depth.max_distance_m = 5.0
    assert metric_distance_for_box(inv_depth, LEFT_BOX, 1.0, config) == 0.5
    fill_box(inv_depth, LEFT_BOX, 0.01)
    assert metric_distance_for_box(inv_depth, LEFT_BOX, 1.0, config) == 5.0


def test_a_box_outside_the_image_has_no_distance(config: NavSightConfig) -> None:
    assert metric_distance_for_box(depth_map(), (700.0, 10.0, 800.0, 50.0), 1.5, config) is None


def test_a_box_partly_outside_the_image_uses_its_visible_part(config: NavSightConfig) -> None:
    inv_depth = depth_map(2.0)
    inv_depth[125:175, 625:640] = 1.0  # the visible part of the box's central 50 %
    box = (600.0, 100.0, 700.0, 200.0)
    assert metric_distance_for_box(inv_depth, box, 1.0, config) == pytest.approx(2.0)


def test_a_tiny_box_still_samples_one_pixel(config: NavSightConfig) -> None:
    inv_depth = depth_map(2.0)
    inv_depth[50, 50] = 1.0
    assert metric_distance_for_box(inv_depth, (50.2, 50.2, 50.6, 50.6), 1.0, config) == 2.0


def test_metric_distance_needs_a_two_dimensional_depth_map(config: NavSightConfig) -> None:
    with pytest.raises(ValueError, match="2-D"):
        metric_distance_for_box(np.ones((480, 640, 1)), LEFT_BOX, 1.5, config)


# attach_distances --------------------------------------------------------------------------


def test_attach_distances_returns_new_detections_with_distances(config: NavSightConfig) -> None:
    inv_depth = depth_map(2.0)
    fill_box(inv_depth, LEFT_BOX, 1.0)
    fill_box(inv_depth, RIGHT_BOX, 4.0)
    detections = [
        Detection("person", 0.9, LEFT_BOX, -30.0),
        Detection("chair", 0.6, RIGHT_BOX, 30.0),
    ]
    result = attach_distances(detections, inv_depth, 1.5, config)
    assert [d.distance_m for d in result] == pytest.approx([3.0, 0.75])
    assert all(new is not old for new, old in zip(result, detections, strict=True))
    assert all(detection.distance_m is None for detection in detections)
    assert [(d.label, d.confidence, d.bbox_xyxy, d.bearing_deg) for d in result] == [
        ("person", 0.9, LEFT_BOX, -30.0),
        ("chair", 0.6, RIGHT_BOX, 30.0),
    ]


def test_attach_distances_agrees_with_metric_distance_for_box(config: NavSightConfig) -> None:
    rng = np.random.default_rng(3)
    inv_depth = rng.uniform(0.5, 4.0, size=(480, 640)).astype(np.float32)
    boxes = [LEFT_BOX, RIGHT_BOX, (0.0, 0.0, 640.0, 480.0), (300.0, 200.0, 340.0, 280.0)]
    detections = [Detection("person", 0.8, box, 0.0) for box in boxes]
    result = attach_distances(detections, inv_depth, 2.5, config)
    assert [d.distance_m for d in result] == pytest.approx(
        [metric_distance_for_box(inv_depth, box, 2.5, config) for box in boxes]
    )


def test_attach_distances_without_depth_or_anchor_clears_the_distance(
    config: NavSightConfig,
) -> None:
    dog = Detection("dog", 0.8, LEFT_BOX, 5.0, distance_m=4.2)
    for inv_depth, anchor in ((None, 1.5), (depth_map(), None)):
        [result] = attach_distances([dog], inv_depth, anchor, config)
        assert result is not dog
        assert result.distance_m is None
        assert result.label == "dog"


def test_attach_distances_of_no_detections_is_empty(config: NavSightConfig) -> None:
    assert attach_distances([], depth_map(), 1.5, config) == []


# DepthEstimator ----------------------------------------------------------------------------


def test_depth_estimator_loads_lazily_and_feeds_rgb_to_midas(config: NavSightConfig) -> None:
    model = FakeMidas(np.full((1, 24, 32), 3.0, dtype=np.float32))
    transform = FakeTransform()
    loads: list[str] = []

    def factory():
        loads.append("load")
        return model, transform

    estimator = DepthEstimator(config, model_factory=factory)
    assert loads == []
    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    frame[..., 0], frame[..., 1], frame[..., 2] = 10, 20, 200  # BGR
    inv_depth = estimator.infer(frame)
    assert loads == ["load"]
    [image] = transform.images
    assert image[0, 0].tolist() == [200, 20, 10]  # RGB
    assert transform.contiguous == [True]
    assert model.batches == ["batch"]
    assert inv_depth.shape == (48, 64)
    assert inv_depth.dtype == np.float32
    assert np.allclose(inv_depth, 3.0)
    estimator.infer(frame)
    assert loads == ["load"]


def test_depth_estimator_resizes_the_prediction_to_the_frame_bilinearly(
    config: NavSightConfig,
) -> None:
    model = FakeMidas(np.array([[[1.0, 2.0], [3.0, 4.0]]], dtype=np.float32))
    estimator = DepthEstimator(config, model_factory=lambda: (model, identity))
    inv_depth = estimator.infer(np.zeros((4, 4, 3), dtype=np.uint8))
    expected = [
        [1.0, 1.25, 1.75, 2.0],
        [1.5, 1.75, 2.25, 2.5],
        [2.5, 2.75, 3.25, 3.5],
        [3.0, 3.25, 3.75, 4.0],
    ]
    assert np.allclose(inv_depth, expected)


def test_depth_estimator_keeps_a_prediction_that_already_has_the_frame_size(
    config: NavSightConfig,
) -> None:
    prediction = np.random.default_rng(5).uniform(0.0, 5.0, size=(1, 6, 8)).astype(np.float32)
    estimator = DepthEstimator(config, model_factory=lambda: (FakeMidas(prediction), identity))
    assert np.array_equal(estimator.infer(np.zeros((6, 8, 3), dtype=np.uint8)), prediction[0])


def test_depth_estimator_converts_torch_like_outputs(config: NavSightConfig) -> None:
    class TorchLikeOutput:
        def __init__(self) -> None:
            self.calls: list[str] = []

        def detach(self) -> TorchLikeOutput:
            self.calls.append("detach")
            return self

        def cpu(self) -> TorchLikeOutput:
            self.calls.append("cpu")
            return self

        def numpy(self) -> np.ndarray:
            self.calls.append("numpy")
            return np.ones((1, 3, 4), dtype=np.float32)

    output = TorchLikeOutput()
    estimator = DepthEstimator(config, model_factory=lambda: (FakeMidas(output), identity))
    assert estimator.infer(np.zeros((3, 4, 3), dtype=np.uint8)).shape == (3, 4)
    assert output.calls == ["detach", "cpu", "numpy"]


def test_depth_estimator_returns_none_without_a_frame(config: NavSightConfig) -> None:
    loads: list[str] = []

    def factory():
        loads.append("load")
        return FakeMidas(np.ones((1, 2, 2))), identity

    estimator = DepthEstimator(config, model_factory=factory)
    assert estimator.infer(None) is None
    assert estimator.infer(np.zeros((0, 0, 3), dtype=np.uint8)) is None
    assert loads == []


def test_depth_estimator_rejects_frames_that_are_not_bgr_images(config: NavSightConfig) -> None:
    estimator = DepthEstimator(config, model_factory=lambda: (FakeMidas(None), identity))
    with pytest.raises(ValueError, match="HxWx3"):
        estimator.infer(np.zeros((4, 4), dtype=np.uint8))


def test_depth_estimator_rejects_unexpected_output_shapes(config: NavSightConfig) -> None:
    model = FakeMidas(np.ones((2, 3, 4), dtype=np.float32))
    estimator = DepthEstimator(config, model_factory=lambda: (model, identity))
    with pytest.raises(ValueError, match="unexpected MiDaS output shape"):
        estimator.infer(np.zeros((3, 4, 3), dtype=np.uint8))


def test_depth_estimator_returns_none_while_the_model_is_unavailable(
    config: NavSightConfig, clock: ManualClock, caplog: pytest.LogCaptureFixture
) -> None:
    attempts: list[float] = []

    def factory():
        attempts.append(clock())
        raise OSError("no network to download the MiDaS weights")

    caplog.set_level(logging.ERROR, logger="navsight.perception.detector")
    estimator = DepthEstimator(config, model_factory=factory, clock=clock)
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    assert estimator.infer(frame) is None
    assert "Could not load MiDaS model 'MiDaS_small'" in caplog.text
    assert "no network" in caplog.text
    assert estimator.infer(frame) is None
    assert len(attempts) == 1
    clock.advance(30.0)
    assert estimator.infer(frame) is None
    assert len(attempts) == 2


def test_depth_estimator_without_torch_logs_how_to_install(
    monkeypatch, config: NavSightConfig, caplog: pytest.LogCaptureFixture
) -> None:
    monkeypatch.setitem(sys.modules, "torch", None)
    assert DepthEstimator(config).infer(np.zeros((4, 4, 3), dtype=np.uint8)) is None
    assert "pip install torch" in caplog.text


def fake_torch(calls: list) -> types.ModuleType:
    """A torch stand-in whose hub returns a fake MiDaS network and its transforms."""

    class FakeNetwork:
        def __init__(self) -> None:
            self.evaluating = False

        def eval(self) -> None:
            self.evaluating = True

        def __call__(self, batch):
            calls.append(("run", batch, self.evaluating))
            return np.ones((1, 2, 2), dtype=np.float32)

    def load(repo: str, name: str, trust_repo=None):
        calls.append(("load", repo, name, trust_repo))
        if name == "transforms":
            return types.SimpleNamespace(
                small_transform=lambda image: "small batch",
                dpt_transform=lambda image: "dpt batch",
            )
        return FakeNetwork()

    torch = types.ModuleType("torch")
    torch.hub = types.SimpleNamespace(load=load)
    torch.no_grad = contextlib.nullcontext
    return torch


@pytest.mark.parametrize(
    "model_type, batch", [("MiDaS_small", "small batch"), ("DPT_Hybrid", "dpt batch")]
)
def test_depth_estimator_loads_midas_and_its_transform_through_torch_hub(
    monkeypatch, config: NavSightConfig, model_type: str, batch: str
) -> None:
    calls: list = []
    monkeypatch.setitem(sys.modules, "torch", fake_torch(calls))
    config.depth.model_type = model_type
    inv_depth = DepthEstimator(config).infer(np.zeros((4, 4, 3), dtype=np.uint8))
    assert inv_depth.shape == (4, 4)
    assert calls == [
        ("load", "intel-isl/MiDaS", model_type, True),
        ("load", "intel-isl/MiDaS", "transforms", True),
        ("run", batch, True),
    ]
