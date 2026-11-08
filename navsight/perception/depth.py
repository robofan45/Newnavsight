"""MiDaS relative inverse depth, scaled to metres by the forward ToF.

MiDaS predicts inverse depth with an unknown scale. The forward ToF measures a real distance in
the middle of the image, so the median inverse depth over the square it sees anchors the scale:
a box whose median inverse depth is half the anchor's is twice as far away.
"""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable, Sequence
from dataclasses import replace
from functools import partial
from typing import Any

import numpy as np

from navsight.config import NavSightConfig
from navsight.perception.detector import (
    LazyModel,
    ModelUnavailableError,
    focal_length_px,
    to_numpy,
)
from navsight.types import Detection
from navsight.util import Clock, clamp

log = logging.getLogger(__name__)

BOX_CENTRAL_FRACTION = 0.5  # median over the middle half of a box's width and height
MIDAS_REPO = "intel-isl/MiDaS"
DPT_MODEL_TYPES = ("DPT_Large", "DPT_Hybrid")


def tof_anchor_half_width_px(
    image_width: int, horizontal_fov_deg: float, tof_fov_deg: float
) -> float:
    """Half-width in pixels of the centred square the forward ToF sees: ``f * tan(tof_fov / 2)``,
    with the pinhole focal length ``f`` of ``pixel_to_bearing``."""
    if not 0.0 < tof_fov_deg < 180.0:
        raise ValueError("tof_fov_deg must be between 0 and 180")
    focal = focal_length_px(image_width, horizontal_fov_deg)
    return focal * math.tan(math.radians(tof_fov_deg) / 2.0)


def metric_distance_for_box(
    inv_depth: np.ndarray,
    bbox_xyxy: Sequence[float],
    anchor_distance_m: float | None,
    config: NavSightConfig,
) -> float | None:
    """``anchor_distance_m * inv_anchor / inv_box``, clamped to the depth distance limits.

    ``inv_box`` is the median inverse depth over the central 50 % of the box (half its width and
    half its height) and ``inv_anchor`` the median over the ToF square. None without an anchor
    or when either median is not positive.
    """
    if not _is_valid_anchor(anchor_distance_m):
        return None
    depth_map = _as_depth_map(inv_depth)
    inv_anchor = _anchor_inverse_depth(depth_map, config)
    return _scaled_distance(
        anchor_distance_m, inv_anchor, _box_inverse_depth(depth_map, bbox_xyxy), config
    )


def attach_distances(
    detections: Sequence[Detection],
    inv_depth: np.ndarray | None,
    anchor_distance_m: float | None,
    config: NavSightConfig,
) -> list[Detection]:
    """New detections with ``distance_m`` from the depth map; None where it cannot be scaled."""
    if inv_depth is None or not _is_valid_anchor(anchor_distance_m):
        return [replace(detection, distance_m=None) for detection in detections]
    depth_map = _as_depth_map(inv_depth)
    inv_anchor = _anchor_inverse_depth(depth_map, config)
    return [
        replace(
            detection,
            distance_m=_scaled_distance(
                anchor_distance_m,
                inv_anchor,
                _box_inverse_depth(depth_map, detection.bbox_xyxy),
                config,
            ),
        )
        for detection in detections
    ]


def _is_valid_anchor(anchor_distance_m: float | None) -> bool:
    return (
        anchor_distance_m is not None
        and math.isfinite(anchor_distance_m)
        and anchor_distance_m > 0.0
    )


def _as_depth_map(inv_depth: np.ndarray) -> np.ndarray:
    depth_map = np.asarray(inv_depth)
    if depth_map.ndim != 2:
        raise ValueError(f"inv_depth must be a 2-D (H, W) array, got shape {depth_map.shape}")
    return depth_map


def _anchor_inverse_depth(depth_map: np.ndarray, config: NavSightConfig) -> float | None:
    height, width = depth_map.shape
    half = tof_anchor_half_width_px(
        width, config.camera.horizontal_fov_deg, config.tof.forward_fov_deg
    )
    return _median_in_window(depth_map, width / 2.0, height / 2.0, half, half)


def _box_inverse_depth(depth_map: np.ndarray, bbox_xyxy: Sequence[float]) -> float | None:
    x1, y1, x2, y2 = bbox_xyxy
    half_width = abs(x2 - x1) * BOX_CENTRAL_FRACTION / 2.0
    half_height = abs(y2 - y1) * BOX_CENTRAL_FRACTION / 2.0
    return _median_in_window(depth_map, (x1 + x2) / 2.0, (y1 + y2) / 2.0, half_width, half_height)


def _median_in_window(
    depth_map: np.ndarray, center_x: float, center_y: float, half_width: float, half_height: float
) -> float | None:
    """Median over the window clipped to the image; None if that is empty or not positive."""
    height, width = depth_map.shape
    columns = _pixel_span(center_x, half_width, width)
    rows = _pixel_span(center_y, half_height, height)
    if columns is None or rows is None:
        return None
    median = float(np.median(depth_map[rows[0] : rows[1], columns[0] : columns[1]]))
    return median if math.isfinite(median) and median > 0.0 else None


def _pixel_span(center: float, half: float, size: int) -> tuple[int, int] | None:
    """Indices [start, stop) of the pixels covering ``center +/- half`` (at least one pixel),
    clipped to the image; None if none of them is inside."""
    start = math.floor(center - half)
    stop = max(math.ceil(center + half), start + 1)
    start, stop = max(start, 0), min(stop, size)
    return (start, stop) if start < stop else None


def _scaled_distance(
    anchor_distance_m: float,
    inv_anchor: float | None,
    inv_box: float | None,
    config: NavSightConfig,
) -> float | None:
    if inv_anchor is None or inv_box is None:
        return None
    distance = anchor_distance_m * inv_anchor / inv_box
    return clamp(distance, config.depth.min_distance_m, config.depth.max_distance_m)


class DepthEstimator:
    """MiDaS relative inverse depth (bigger = closer); the model loads on the first ``infer``."""

    def __init__(
        self,
        config: NavSightConfig,
        model_factory: Callable[[], tuple[Any, Any]] | None = None,
        *,
        clock: Clock = time.monotonic,
    ) -> None:
        model_type = config.depth.model_type
        loader = model_factory or partial(_load_midas, model_type)
        self._model = LazyModel(f"MiDaS model {model_type!r}", loader, clock=clock)

    def infer(self, frame: np.ndarray) -> np.ndarray | None:
        """Relative inverse depth (H, W) float32 for a BGR frame.

        None without a frame, or while the model is unavailable: a failed load is logged and
        retried after ``MODEL_RETRY_S``, and depth is an optional extra, so it never raises then.
        """
        if frame is None or frame.size == 0:
            return None
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(f"infer needs an HxWx3 BGR frame, got shape {frame.shape}")
        try:
            model, transform = self._model.get()
        except ModelUnavailableError as exc:
            log.debug("No depth estimate: %s", exc)
            return None
        rgb = np.ascontiguousarray(frame[:, :, ::-1])
        prediction = _prediction_map(model(transform(rgb)))
        return _resize_bilinear(prediction, frame.shape[0], frame.shape[1])


def _prediction_map(output: Any) -> np.ndarray:
    prediction = to_numpy(output).astype(np.float32)
    if prediction.ndim < 2 or prediction.size != prediction.shape[-2] * prediction.shape[-1]:
        raise ValueError(f"unexpected MiDaS output shape {prediction.shape}; expected (1, H, W)")
    return prediction.reshape(prediction.shape[-2:])


def _resize_bilinear(image: np.ndarray, height: int, width: int) -> np.ndarray:
    """Bilinear resize of a 2-D array, sampling at pixel centres (like align_corners=False)."""
    rows = _source_positions(image.shape[0], height)
    columns = _source_positions(image.shape[1], width)
    row0 = np.floor(rows).astype(int)
    col0 = np.floor(columns).astype(int)
    row1 = np.minimum(row0 + 1, image.shape[0] - 1)
    col1 = np.minimum(col0 + 1, image.shape[1] - 1)
    row_weight = (rows - row0)[:, np.newaxis]
    col_weight = (columns - col0)[np.newaxis, :]
    top = image[row0][:, col0] * (1.0 - col_weight) + image[row0][:, col1] * col_weight
    bottom = image[row1][:, col0] * (1.0 - col_weight) + image[row1][:, col1] * col_weight
    return (top * (1.0 - row_weight) + bottom * row_weight).astype(np.float32)


def _source_positions(source_size: int, target_size: int) -> np.ndarray:
    positions = (np.arange(target_size) + 0.5) * (source_size / target_size) - 0.5
    return np.clip(positions, 0.0, source_size - 1)


def _load_midas(model_type: str) -> tuple[Any, Any]:
    """MiDaS and its input transform through torch.hub (downloads on first use)."""
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("PyTorch is not installed: pip install torch timm") from exc
    midas = torch.hub.load(MIDAS_REPO, model_type, trust_repo=True)
    midas.eval()
    transforms = torch.hub.load(MIDAS_REPO, "transforms", trust_repo=True)
    if model_type in DPT_MODEL_TYPES:
        transform = transforms.dpt_transform
    else:
        transform = transforms.small_transform

    def predict(batch: Any) -> Any:
        with torch.no_grad():
            return midas(batch)

    return predict, transform
