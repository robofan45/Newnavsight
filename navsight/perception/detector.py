"""YOLOv8n object detection, with the bearing of each detection from the camera axis."""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable, Mapping, Sequence
from functools import partial
from typing import Any

import numpy as np
from numpy.typing import ArrayLike

from navsight.config import NavSightConfig
from navsight.types import Detection
from navsight.util import Clock

log = logging.getLogger(__name__)

MODEL_RETRY_S = 30.0


class ModelUnavailableError(RuntimeError):
    """A model could not be loaded; it is retried at most once every ``MODEL_RETRY_S``."""


class LazyModel:
    """Loads a model on first use and keeps it.

    A failed load is logged and raises ``ModelUnavailableError``; until ``retry_s`` has passed,
    later calls raise again without retrying, so a missing model or package does not stall
    every frame with a slow load attempt.
    """

    def __init__(
        self,
        description: str,
        loader: Callable[[], Any],
        *,
        clock: Clock = time.monotonic,
        retry_s: float = MODEL_RETRY_S,
    ) -> None:
        self._description = description
        self._loader = loader
        self._clock = clock
        self._retry_s = retry_s
        self._model: Any = None
        self._retry_at = -math.inf
        self._last_error = ""

    def get(self) -> Any:
        if self._model is not None:
            return self._model
        now = self._clock()
        if now < self._retry_at:
            raise ModelUnavailableError(
                f"{self._description} is unavailable ({self._last_error}); "
                f"next attempt in {self._retry_at - now:.0f} s"
            )
        log.info("Loading %s", self._description)
        try:
            self._model = self._loader()
        except Exception as exc:
            self._retry_at = now + self._retry_s
            self._last_error = f"{type(exc).__name__}: {exc}"
            log.error(
                "Could not load %s (%s); retrying in %.0f s",
                self._description,
                self._last_error,
                self._retry_s,
                exc_info=True,
            )
            raise ModelUnavailableError(
                f"Could not load {self._description}: {self._last_error}"
            ) from exc
        return self._model


def to_numpy(value: Any) -> np.ndarray:
    """A numpy array from a torch tensor (detached and moved to the CPU) or any array-like."""
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def focal_length_px(image_width: int, horizontal_fov_deg: float) -> float:
    """Pinhole focal length in pixels: ``f = (W / 2) / tan(hfov / 2)``."""
    if image_width <= 0:
        raise ValueError("image_width must be positive")
    if not 0.0 < horizontal_fov_deg < 180.0:
        raise ValueError("horizontal_fov_deg must be between 0 and 180")
    return (image_width / 2.0) / math.tan(math.radians(horizontal_fov_deg) / 2.0)


def pixel_to_bearing(x: float, image_width: int, horizontal_fov_deg: float) -> float:
    """Bearing in degrees of pixel column ``x`` (negative = left).

    A wide-angle lens is not linear in angle, so this uses the pinhole model:
    ``bearing = atan((x - W / 2) / f)``.
    """
    focal = focal_length_px(image_width, horizontal_fov_deg)
    return math.degrees(math.atan((x - image_width / 2.0) / focal))


def results_to_detections(
    boxes_xyxy: ArrayLike,
    class_ids: ArrayLike,
    confidences: ArrayLike,
    names: Mapping[int, str] | Sequence[str],
    image_width: int,
    config: NavSightConfig,
) -> list[Detection]:
    """Detections at or above ``detector.confidence`` whose class is in ``detector.classes``,
    most confident first. ``names`` maps class ids to labels (a dict or a list)."""
    boxes = np.asarray(boxes_xyxy, dtype=float).reshape(-1, 4)
    ids = np.asarray(class_ids).reshape(-1)
    scores = np.asarray(confidences, dtype=float).reshape(-1)
    if not len(boxes) == len(ids) == len(scores):
        raise ValueError(
            f"got {len(boxes)} boxes, {len(ids)} class ids and {len(scores)} confidences"
        )
    settings = config.detector
    allowed = set(settings.classes)
    detections = []
    for box, class_id, score in zip(boxes, ids, scores, strict=True):
        label = _class_name(names, int(class_id))
        if score < settings.confidence or label not in allowed:
            continue
        x1, y1, x2, y2 = (float(value) for value in box)
        bearing = pixel_to_bearing((x1 + x2) / 2.0, image_width, config.camera.horizontal_fov_deg)
        detections.append(Detection(label, float(score), (x1, y1, x2, y2), bearing))
    detections.sort(key=lambda detection: detection.confidence, reverse=True)
    return detections


def _class_name(names: Mapping[int, str] | Sequence[str], class_id: int) -> str:
    try:
        return str(names[class_id])
    except (KeyError, IndexError):
        return str(class_id)


class ObjectDetector:
    """YOLOv8 detections for ``config.detector``; the model loads on the first ``detect``."""

    def __init__(
        self,
        config: NavSightConfig,
        model_factory: Callable[[str], Any] | None = None,
        *,
        clock: Clock = time.monotonic,
    ) -> None:
        self._config = config
        path = config.detector.model_path
        factory = model_factory or _load_yolo
        self._model = LazyModel(f"YOLO model {path!r}", partial(factory, path), clock=clock)

    def load(self) -> None:
        """Load the model now (for example at start-up) instead of on the first frame."""
        self._model.get()

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Detections in a BGR frame. Raises ``ModelUnavailableError`` without a model."""
        if frame is None or frame.ndim != 3:
            raise ValueError("detect needs an HxWx3 BGR frame")
        model = self._model.get()
        settings = self._config.detector
        results = model.predict(
            frame, imgsz=settings.image_size, conf=settings.confidence, verbose=False
        )
        if not results:
            return []
        result = results[0]
        boxes = result.boxes
        if boxes is None:
            return []
        names = getattr(result, "names", None) or model.names
        return results_to_detections(
            to_numpy(boxes.xyxy),
            to_numpy(boxes.cls),
            to_numpy(boxes.conf),
            names,
            frame.shape[1],
            self._config,
        )


def _load_yolo(model_path: str) -> Any:
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("ultralytics is not installed: pip install ultralytics") from exc
    return YOLO(model_path, task="detect")
