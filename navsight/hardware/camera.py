"""Camera frame sources: the CSI camera (Picamera2), a USB camera (OpenCV) or synthetic frames.

Every source returns HxWx3 uint8 frames in BGR order, the layout OpenCV and Ultralytics expect.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any, Protocol

import numpy as np

from navsight.config import CameraConfig, NavSightConfig

log = logging.getLogger(__name__)

# OpenCV VideoCaptureProperties values, so a fake capture in the tests needs no cv2 import.
CAP_PROP_FRAME_WIDTH = 3
CAP_PROP_FRAME_HEIGHT = 4
CAP_PROP_FPS = 5
CAP_PROP_BUFFERSIZE = 38


class FrameSource(Protocol):
    def read(self) -> np.ndarray | None: ...  # HxWx3 uint8, BGR order

    def close(self) -> None: ...


class PicameraSource:
    """The CSI camera through Picamera2, in an RGB888 video configuration.

    Picamera2's "RGB888" format stores each pixel as B, G, R bytes, so frames are already BGR.
    The sensor mode is the smallest one that sees the whole sensor at the requested frame rate:
    left to itself, libcamera picks the Camera Module 3's fast 1536x864 mode for small frames,
    and that mode is a centre crop that narrows the field of view.
    """

    def __init__(
        self, config: CameraConfig, *, camera_factory: Callable[[], Any] | None = None
    ) -> None:
        self._camera = (camera_factory or _new_picamera2)()
        try:
            self._camera.configure(self._video_configuration(config))
            self._camera.start()
        except Exception as exc:
            self._camera.close()
            raise RuntimeError(f"Could not start the CSI camera: {exc}") from exc
        log.info("CSI camera started at %dx%d, %.0f fps", config.width, config.height, config.fps)

    def read(self) -> np.ndarray | None:
        return self._camera.capture_array("main")

    def close(self) -> None:
        try:
            self._camera.stop()
        finally:
            self._camera.close()

    def _video_configuration(self, config: CameraConfig) -> Any:
        options: dict[str, Any] = {
            "main": {"size": (config.width, config.height), "format": "RGB888"},
        }
        if config.fps > 0:
            options["controls"] = {"FrameRate": config.fps}
        try:
            mode = _full_view_sensor_mode(self._camera, config.fps)
        except Exception as exc:
            log.warning("Could not read the camera's sensor modes (%s); using its default", exc)
            mode = None
        if mode is not None:
            options["sensor"] = {"output_size": tuple(mode["size"]), "bit_depth": mode["bit_depth"]}
        return self._camera.create_video_configuration(**options)


def _full_view_sensor_mode(camera: Any, fps: float) -> dict[str, Any] | None:
    """The smallest sensor mode whose crop covers the whole sensor and that reaches ``fps``."""
    width, height = camera.sensor_resolution
    full_view = [
        mode
        for mode in camera.sensor_modes
        if tuple(mode["crop_limits"]) == (0, 0, width, height) and mode["fps"] >= fps
    ]
    if not full_view:
        log.warning("No full field-of-view sensor mode reaches %.0f fps; using the default", fps)
        return None
    return min(full_view, key=lambda mode: mode["size"][0] * mode["size"][1])


def _new_picamera2() -> Any:
    try:
        from picamera2 import Picamera2
    except ImportError as exc:
        raise RuntimeError(
            "Picamera2 is not installed: sudo apt install python3-picamera2, and create the "
            "virtual environment with --system-site-packages"
        ) from exc
    try:
        return Picamera2()
    except Exception as exc:  # Picamera2 raises IndexError or RuntimeError without a camera
        raise RuntimeError(
            f"No CSI camera found ({exc}): check the ribbon cable and run "
            "'rpicam-hello --list-cameras'"
        ) from exc


class OpenCvSource:
    """A USB camera through ``cv2.VideoCapture(usb_index)``."""

    def __init__(
        self, config: CameraConfig, *, capture_factory: Callable[[int], Any] | None = None
    ) -> None:
        self._capture = (capture_factory or _new_video_capture)(config.usb_index)
        if not self._capture.isOpened():
            self._capture.release()
            raise RuntimeError(
                f"Could not open USB camera {config.usb_index}: find its index with "
                "'v4l2-ctl --list-devices' and set camera.usb_index"
            )
        self._capture.set(CAP_PROP_FRAME_WIDTH, config.width)
        self._capture.set(CAP_PROP_FRAME_HEIGHT, config.height)
        self._capture.set(CAP_PROP_FPS, config.fps)
        self._capture.set(CAP_PROP_BUFFERSIZE, 1)  # keep only the newest frame: less lag
        log.info("USB camera %d opened", config.usb_index)

    def read(self) -> np.ndarray | None:
        ok, frame = self._capture.read()
        if not ok or frame is None:
            return None
        return frame

    def close(self) -> None:
        self._capture.release()


def _new_video_capture(index: int) -> Any:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is not installed: pip install opencv-python-headless") from exc
    return cv2.VideoCapture(index)


class SyntheticSource:
    """Blank frames for the simulator, or a frame it sets with ``set_frame``."""

    def __init__(self, width: int = 640, height: int = 480) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("width and height must be positive")
        self.width = width
        self.height = height
        self._frame: np.ndarray | None = None
        self._closed = False

    def set_frame(self, frame: np.ndarray | None) -> None:
        """Serve copies of ``frame`` from now on; None goes back to blank frames."""
        if frame is None:
            self._frame = None
            return
        image = np.array(frame, dtype=np.uint8)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"a frame must be HxWx3, got shape {image.shape}")
        self._frame = image

    def read(self) -> np.ndarray | None:
        if self._closed:
            return None
        if self._frame is None:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        return self._frame.copy()

    def close(self) -> None:
        self._closed = True


def open_camera(config: NavSightConfig, simulate: bool = False) -> FrameSource:
    """The configured camera, or a SyntheticSource when simulating."""
    camera = config.camera
    if simulate or config.simulate:
        return SyntheticSource(camera.width, camera.height)
    if camera.source == "csi":
        return PicameraSource(camera)
    if camera.source == "usb":
        return OpenCvSource(camera)
    raise ValueError(f'camera.source must be "csi" or "usb", not {camera.source!r}')
