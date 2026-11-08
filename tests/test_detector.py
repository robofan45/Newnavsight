from __future__ import annotations

import math
import sys
import types

import numpy as np
import pytest

from navsight.config import NavSightConfig
from navsight.perception.detector import (
    ModelUnavailableError,
    ObjectDetector,
    focal_length_px,
    pixel_to_bearing,
    results_to_detections,
)
from navsight.util import ManualClock

COCO_NAMES = {0: "person", 1: "bicycle", 2: "car", 16: "dog", 39: "bottle", 56: "chair"}


class FakeTensor:
    """Mimics a torch tensor: ``.cpu().numpy()`` gives the values."""

    def __init__(self, values) -> None:
        self._values = np.asarray(values, dtype=np.float32)
        self.moved_to_cpu = False

    def cpu(self) -> FakeTensor:
        self.moved_to_cpu = True
        return self

    def numpy(self) -> np.ndarray:
        return self._values


class FakeBoxes:
    def __init__(self, xyxy, cls, conf) -> None:
        self.xyxy = FakeTensor(xyxy)
        self.cls = FakeTensor(cls)
        self.conf = FakeTensor(conf)


class FakeResult:
    def __init__(self, boxes: FakeBoxes | None, names: dict[int, str] | None) -> None:
        self.boxes = boxes
        self.names = names


class FakeYolo:
    """The subset of ultralytics.YOLO that ObjectDetector uses."""

    def __init__(self, results: list[FakeResult]) -> None:
        self.results = results
        self.names = COCO_NAMES
        self.calls: list[tuple[np.ndarray, dict]] = []

    def predict(self, frame: np.ndarray, **options) -> list[FakeResult]:
        self.calls.append((frame, options))
        return self.results


def yolo_with(boxes, class_ids, confidences) -> FakeYolo:
    return FakeYolo([FakeResult(FakeBoxes(boxes, class_ids, confidences), COCO_NAMES)])


@pytest.fixture
def frame() -> np.ndarray:
    return np.zeros((480, 640, 3), dtype=np.uint8)


# pixel_to_bearing --------------------------------------------------------------------------


def test_pixel_to_bearing_is_zero_at_the_image_centre() -> None:
    assert pixel_to_bearing(320.0, 640, 102.0) == pytest.approx(0.0)


@pytest.mark.parametrize("hfov", [60.0, 102.0, 150.0])
def test_pixel_to_bearing_reaches_half_the_fov_at_the_edges(hfov: float) -> None:
    assert pixel_to_bearing(0.0, 640, hfov) == pytest.approx(-hfov / 2)
    assert pixel_to_bearing(640.0, 640, hfov) == pytest.approx(hfov / 2)


@pytest.mark.parametrize("offset", [1.0, 50.0, 160.0, 320.0])
def test_pixel_to_bearing_is_symmetric_about_the_centre(offset: float) -> None:
    right = pixel_to_bearing(320.0 + offset, 640, 102.0)
    left = pixel_to_bearing(320.0 - offset, 640, 102.0)
    assert right > 0
    assert left == pytest.approx(-right)


def test_pixel_to_bearing_increases_from_left_to_right() -> None:
    bearings = [pixel_to_bearing(x, 640, 102.0) for x in np.linspace(0.0, 640.0, 81)]
    assert np.all(np.diff(bearings) > 0)


def test_pixel_to_bearing_uses_the_pinhole_model_not_a_linear_map() -> None:
    bearing = pixel_to_bearing(480.0, 640, 102.0)  # three quarters of the way across
    assert bearing == pytest.approx(math.degrees(math.atan(0.5 * math.tan(math.radians(51.0)))))
    assert bearing == pytest.approx(31.69, abs=0.01)
    assert bearing > 102.0 / 4 + 5  # a linear map would give 25.5


def test_pixel_to_bearing_follows_the_focal_length_formula() -> None:
    focal = focal_length_px(640, 102.0)
    assert focal == pytest.approx(320.0 / math.tan(math.radians(51.0)))
    assert pixel_to_bearing(400.0, 640, 102.0) == pytest.approx(math.degrees(math.atan(80 / focal)))


@pytest.mark.parametrize(
    "width, hfov", [(0, 102.0), (-640, 102.0), (640, 0.0), (640, 180.0), (640, -10.0)]
)
def test_pixel_to_bearing_rejects_impossible_geometry(width: int, hfov: float) -> None:
    with pytest.raises(ValueError):
        pixel_to_bearing(10.0, width, hfov)


# results_to_detections ---------------------------------------------------------------------


def test_results_keep_confident_allowed_classes_sorted_by_confidence(
    config: NavSightConfig,
) -> None:
    boxes = [
        [0, 0, 100, 100],  # person 0.50: kept
        [300, 100, 340, 200],  # car 0.90: kept
        [500, 50, 640, 300],  # chair 0.45: kept, exactly the threshold
        [10, 10, 20, 20],  # bottle 0.99: dropped, not a configured class
        [200, 200, 260, 300],  # person 0.30: dropped, below the threshold
    ]
    detections = results_to_detections(
        boxes, [0, 2, 56, 39, 0], [0.50, 0.90, 0.45, 0.99, 0.30], COCO_NAMES, 640, config
    )
    assert [(d.label, d.confidence) for d in detections] == [
        ("car", 0.90),
        ("person", 0.50),
        ("chair", 0.45),
    ]
    car, person, chair = detections
    assert car.bbox_xyxy == (300.0, 100.0, 340.0, 200.0)
    assert car.bearing_deg == pytest.approx(0.0)  # box centre x = 320, the image centre
    assert person.bearing_deg == pytest.approx(pixel_to_bearing(50.0, 640, 102.0))
    assert chair.bearing_deg == pytest.approx(pixel_to_bearing(570.0, 640, 102.0))
    assert all(detection.distance_m is None for detection in detections)


def test_results_accept_numpy_arrays_and_a_list_of_names(config: NavSightConfig) -> None:
    detections = results_to_detections(
        np.array([[0, 0, 64, 48]], dtype=np.float32),
        np.array([2.0], dtype=np.float32),
        np.array([0.8], dtype=np.float32),
        ["person", "bicycle", "car"],
        640,
        config,
    )
    [car] = detections
    assert car.label == "car"
    assert isinstance(car.confidence, float)
    assert all(isinstance(value, float) for value in car.bbox_xyxy)


def test_results_use_the_configured_threshold_and_classes(config: NavSightConfig) -> None:
    config.detector.confidence = 0.8
    config.detector.classes = ("chair",)
    detections = results_to_detections(
        [[0, 0, 10, 10], [0, 0, 20, 20], [0, 0, 30, 30]],
        [56, 56, 0],
        [0.85, 0.70, 0.95],
        COCO_NAMES,
        640,
        config,
    )
    assert [(d.label, d.confidence) for d in detections] == [("chair", 0.85)]


def test_results_bearing_uses_the_configured_field_of_view(config: NavSightConfig) -> None:
    config.camera.horizontal_fov_deg = 60.0
    [person] = results_to_detections([[0, 0, 0, 10]], [0], [0.9], COCO_NAMES, 640, config)
    assert person.bearing_deg == pytest.approx(-30.0)


def test_results_drop_unknown_class_ids(config: NavSightConfig) -> None:
    assert results_to_detections([[0, 0, 10, 10]], [99], [0.9], COCO_NAMES, 640, config) == []
    assert results_to_detections([[0, 0, 10, 10]], [7], [0.9], ["person"], 640, config) == []


def test_results_with_no_boxes_are_empty(config: NavSightConfig) -> None:
    assert results_to_detections(np.zeros((0, 4)), [], [], COCO_NAMES, 640, config) == []


def test_results_reject_mismatched_lengths(config: NavSightConfig) -> None:
    with pytest.raises(ValueError, match="2 boxes, 1 class ids and 2 confidences"):
        results_to_detections(
            [[0, 0, 1, 1], [0, 0, 2, 2]], [0], [0.9, 0.8], COCO_NAMES, 640, config
        )


# ObjectDetector ----------------------------------------------------------------------------


def test_detector_loads_the_model_lazily_and_only_once(
    config: NavSightConfig, frame: np.ndarray
) -> None:
    loads: list[str] = []
    model = yolo_with([[300, 100, 340, 200]], [0], [0.9])

    def factory(path: str) -> FakeYolo:
        loads.append(path)
        return model

    detector = ObjectDetector(config, model_factory=factory)
    assert loads == []
    detector.detect(frame)
    detector.detect(frame)
    assert loads == ["yolov8n.pt"]


def test_detector_predicts_with_the_configured_size_and_confidence(
    config: NavSightConfig, frame: np.ndarray
) -> None:
    config.detector.image_size = 320
    config.detector.confidence = 0.6
    model = yolo_with([[300, 100, 340, 200]], [0], [0.9])
    ObjectDetector(config, model_factory=lambda path: model).detect(frame)
    [(frame_seen, options)] = model.calls
    assert frame_seen is frame
    assert options == {"imgsz": 320, "conf": 0.6, "verbose": False}


def test_detector_converts_tensors_and_returns_filtered_sorted_detections(
    config: NavSightConfig, frame: np.ndarray
) -> None:
    model = yolo_with(
        [[0, 0, 100, 100], [300, 100, 340, 200], [10, 10, 20, 20]], [0, 2, 39], [0.5, 0.9, 0.99]
    )
    detections = ObjectDetector(config, model_factory=lambda path: model).detect(frame)
    assert [d.label for d in detections] == ["car", "person"]
    assert detections[0].confidence == pytest.approx(0.9)
    assert detections[0].bearing_deg == pytest.approx(0.0)
    boxes = model.results[0].boxes
    assert boxes.xyxy.moved_to_cpu
    assert boxes.cls.moved_to_cpu
    assert boxes.conf.moved_to_cpu


def test_detector_bearings_follow_the_frame_width(config: NavSightConfig) -> None:
    model = yolo_with([[150, 0, 170, 10]], [0], [0.9])
    small_frame = np.zeros((240, 320, 3), dtype=np.uint8)
    [person] = ObjectDetector(config, model_factory=lambda path: model).detect(small_frame)
    assert person.bearing_deg == pytest.approx(0.0)


def test_detector_falls_back_to_the_model_class_names(
    config: NavSightConfig, frame: np.ndarray
) -> None:
    model = FakeYolo([FakeResult(FakeBoxes([[0, 0, 10, 10]], [16], [0.9]), None)])
    [dog] = ObjectDetector(config, model_factory=lambda path: model).detect(frame)
    assert dog.label == "dog"


@pytest.mark.parametrize("results", [[], [FakeResult(None, COCO_NAMES)]])
def test_detector_handles_empty_results(
    config: NavSightConfig, frame: np.ndarray, results: list[FakeResult]
) -> None:
    model = FakeYolo(results)
    assert ObjectDetector(config, model_factory=lambda path: model).detect(frame) == []


def test_detector_retries_a_failed_load_only_after_a_pause(
    config: NavSightConfig, frame: np.ndarray, clock: ManualClock, caplog: pytest.LogCaptureFixture
) -> None:
    attempts: list[str] = []

    def factory(path: str) -> FakeYolo:
        attempts.append(path)
        raise FileNotFoundError(f"{path} does not exist")

    detector = ObjectDetector(config, model_factory=factory, clock=clock)
    with pytest.raises(ModelUnavailableError, match="does not exist"):
        detector.detect(frame)
    with pytest.raises(ModelUnavailableError, match="next attempt in 30 s"):
        detector.detect(frame)
    clock.advance(29.0)
    with pytest.raises(ModelUnavailableError):
        detector.detect(frame)
    assert len(attempts) == 1
    clock.advance(1.0)
    with pytest.raises(ModelUnavailableError):
        detector.detect(frame)
    assert len(attempts) == 2
    assert "Could not load YOLO model 'yolov8n.pt'" in caplog.text


def test_detector_recovers_once_the_model_loads(
    config: NavSightConfig, frame: np.ndarray, clock: ManualClock
) -> None:
    outcomes = [OSError("network unreachable"), yolo_with([[0, 0, 10, 10]], [0], [0.9])]

    def factory(path: str) -> FakeYolo:
        outcome = outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome

    detector = ObjectDetector(config, model_factory=factory, clock=clock)
    with pytest.raises(ModelUnavailableError):
        detector.detect(frame)
    clock.advance(30.0)
    assert [d.label for d in detector.detect(frame)] == ["person"]


def test_detector_without_ultralytics_explains_how_to_install(
    monkeypatch, config: NavSightConfig, frame: np.ndarray
) -> None:
    monkeypatch.setitem(sys.modules, "ultralytics", None)
    with pytest.raises(ModelUnavailableError, match="pip install ultralytics"):
        ObjectDetector(config).detect(frame)


def test_detector_default_factory_builds_an_ultralytics_yolo_detector(
    monkeypatch, config: NavSightConfig, frame: np.ndarray
) -> None:
    created: list[tuple[str, str | None]] = []

    def yolo(path: str, task: str | None = None) -> FakeYolo:
        created.append((path, task))
        return yolo_with([], [], [])

    module = types.ModuleType("ultralytics")
    module.YOLO = yolo
    monkeypatch.setitem(sys.modules, "ultralytics", module)
    config.detector.model_path = "/opt/navsight/yolov8n_ncnn_model"
    assert ObjectDetector(config).detect(frame) == []
    assert created == [("/opt/navsight/yolov8n_ncnn_model", "detect")]


def test_load_loads_the_model_now(config: NavSightConfig) -> None:
    loads: list[str] = []

    def factory(path: str) -> FakeYolo:
        loads.append(path)
        return yolo_with([], [], [])

    ObjectDetector(config, model_factory=factory).load()
    assert loads == ["yolov8n.pt"]


@pytest.mark.parametrize("bad_frame", [None, np.zeros((480, 640), dtype=np.uint8)])
def test_detector_rejects_frames_that_are_not_images(
    config: NavSightConfig, bad_frame: np.ndarray | None
) -> None:
    detector = ObjectDetector(config, model_factory=lambda path: yolo_with([], [], []))
    with pytest.raises(ValueError, match="HxWx3"):
        detector.detect(bad_frame)
