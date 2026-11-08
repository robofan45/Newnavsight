from __future__ import annotations

import sys
import types

import numpy as np
import pytest

from navsight.config import CameraConfig, NavSightConfig
from navsight.hardware.camera import (
    CAP_PROP_BUFFERSIZE,
    CAP_PROP_FPS,
    CAP_PROP_FRAME_HEIGHT,
    CAP_PROP_FRAME_WIDTH,
    OpenCvSource,
    PicameraSource,
    SyntheticSource,
    open_camera,
)

# The modes `rpicam-hello --list-cameras` reports for the Camera Module 3 (IMX708).
IMX708_MODES = [
    {"size": (1536, 864), "fps": 120.13, "crop_limits": (768, 432, 3072, 1728), "bit_depth": 10},
    {"size": (2304, 1296), "fps": 56.03, "crop_limits": (0, 0, 4608, 2592), "bit_depth": 10},
    {"size": (4608, 2592), "fps": 14.35, "crop_limits": (0, 0, 4608, 2592), "bit_depth": 10},
]


class FakePicamera2:
    """The subset of picamera2.Picamera2 that PicameraSource uses."""

    def __init__(self, *, fail_on_start: bool = False) -> None:
        self.sensor_resolution = (4608, 2592)
        self.sensor_modes = IMX708_MODES
        self.frame = np.full((480, 640, 3), 7, dtype=np.uint8)
        self.options: dict = {}
        self.configured: dict | None = None
        self.events: list[str] = []
        self._fail_on_start = fail_on_start

    def create_video_configuration(self, **options) -> dict:
        self.options = options
        return {"use_case": "video", **options}

    def configure(self, configuration: dict) -> None:
        self.configured = configuration
        self.events.append("configure")

    def start(self) -> None:
        if self._fail_on_start:
            raise RuntimeError("Camera frontend has timed out!")
        self.events.append("start")

    def capture_array(self, name: str = "main") -> np.ndarray:
        self.events.append(f"capture {name}")
        return self.frame

    def stop(self) -> None:
        self.events.append("stop")

    def close(self) -> None:
        self.events.append("close")


class FakeCapture:
    """The subset of cv2.VideoCapture that OpenCvSource uses."""

    def __init__(self, *, opened: bool = True, frames: tuple[np.ndarray, ...] = ()) -> None:
        self.opened = opened
        self.frames = list(frames)
        self.properties: dict[int, float] = {}
        self.released = False

    def isOpened(self) -> bool:
        return self.opened

    def set(self, prop: int, value: float) -> bool:
        self.properties[prop] = value
        return True

    def read(self) -> tuple[bool, np.ndarray | None]:
        if not self.frames:
            return False, None
        return True, self.frames.pop(0)

    def release(self) -> None:
        self.released = True


def picamera_module(factory) -> types.ModuleType:
    module = types.ModuleType("picamera2")
    module.Picamera2 = factory
    return module


# SyntheticSource ---------------------------------------------------------------------------


def test_synthetic_source_gives_blank_hxwx3_uint8_frames() -> None:
    frame = SyntheticSource(320, 240).read()
    assert frame.shape == (240, 320, 3)
    assert frame.dtype == np.uint8
    assert not frame.any()


def test_synthetic_source_defaults_to_the_camera_frame_size() -> None:
    assert SyntheticSource().read().shape == (480, 640, 3)


def test_synthetic_source_serves_the_frame_the_simulator_sets() -> None:
    source = SyntheticSource(4, 3)
    image = np.arange(36, dtype=np.uint8).reshape(3, 4, 3)
    source.set_frame(image)
    image[:] = 0  # the source kept its own copy
    assert np.array_equal(source.read(), np.arange(36, dtype=np.uint8).reshape(3, 4, 3))
    source.set_frame(None)
    assert not source.read().any()


def test_synthetic_source_frames_are_independent_copies() -> None:
    source = SyntheticSource(4, 3)
    source.read()[:] = 255
    assert not source.read().any()
    source.set_frame(np.full((3, 4, 3), 9, dtype=np.uint8))
    source.read()[:] = 0
    assert (source.read() == 9).all()


def test_synthetic_source_rejects_frames_that_are_not_hxwx3() -> None:
    with pytest.raises(ValueError, match="HxWx3"):
        SyntheticSource().set_frame(np.zeros((480, 640), dtype=np.uint8))


@pytest.mark.parametrize("width, height", [(0, 480), (640, 0), (-1, 10)])
def test_synthetic_source_rejects_bad_sizes(width: int, height: int) -> None:
    with pytest.raises(ValueError):
        SyntheticSource(width, height)


def test_synthetic_source_reads_none_once_closed() -> None:
    source = SyntheticSource()
    source.close()
    assert source.read() is None


# PicameraSource ----------------------------------------------------------------------------


def test_picamera_source_configures_rgb888_video_at_the_configured_size() -> None:
    camera = FakePicamera2()
    PicameraSource(CameraConfig(width=640, height=480, fps=15.0), camera_factory=lambda: camera)
    assert camera.options["main"] == {"size": (640, 480), "format": "RGB888"}
    assert camera.options["controls"] == {"FrameRate": 15.0}
    assert camera.configured == {"use_case": "video", **camera.options}
    assert camera.events == ["configure", "start"]


def test_picamera_source_picks_the_smallest_full_view_sensor_mode() -> None:
    camera = FakePicamera2()
    PicameraSource(CameraConfig(fps=15.0), camera_factory=lambda: camera)
    # Not the faster 1536x864 mode: it is a centre crop of the sensor.
    assert camera.options["sensor"] == {"output_size": (2304, 1296), "bit_depth": 10}


def test_picamera_source_uses_the_full_sensor_mode_for_slow_frame_rates() -> None:
    camera = FakePicamera2()
    camera.sensor_modes = [mode for mode in IMX708_MODES if mode["size"] != (2304, 1296)]
    PicameraSource(CameraConfig(fps=10.0), camera_factory=lambda: camera)
    assert camera.options["sensor"] == {"output_size": (4608, 2592), "bit_depth": 10}


def test_picamera_source_keeps_the_default_mode_when_no_full_view_mode_is_fast_enough(
    caplog: pytest.LogCaptureFixture,
) -> None:
    camera = FakePicamera2()
    PicameraSource(CameraConfig(fps=60.0), camera_factory=lambda: camera)
    assert "sensor" not in camera.options
    assert "full field-of-view" in caplog.text


def test_picamera_source_copes_without_sensor_mode_information(
    caplog: pytest.LogCaptureFixture,
) -> None:
    camera = FakePicamera2()
    del camera.sensor_modes
    PicameraSource(CameraConfig(), camera_factory=lambda: camera)
    assert "sensor" not in camera.options
    assert camera.events == ["configure", "start"]
    assert "sensor modes" in caplog.text


def test_picamera_source_reads_bgr_frames_from_the_main_stream_and_closes() -> None:
    camera = FakePicamera2()
    source = PicameraSource(CameraConfig(), camera_factory=lambda: camera)
    assert source.read() is camera.frame
    source.close()
    assert camera.events[-3:] == ["capture main", "stop", "close"]


def test_picamera_source_closes_the_camera_when_it_cannot_start() -> None:
    camera = FakePicamera2(fail_on_start=True)
    with pytest.raises(RuntimeError, match="Could not start the CSI camera: Camera frontend"):
        PicameraSource(CameraConfig(), camera_factory=lambda: camera)
    assert camera.events[-1] == "close"


def test_picamera_source_without_picamera2_explains_how_to_install(monkeypatch) -> None:
    monkeypatch.setitem(sys.modules, "picamera2", None)
    with pytest.raises(RuntimeError, match="python3-picamera2"):
        PicameraSource(CameraConfig())


def test_picamera_source_without_a_camera_points_at_the_cable(monkeypatch) -> None:
    def no_camera() -> None:
        raise IndexError("list index out of range")

    monkeypatch.setitem(sys.modules, "picamera2", picamera_module(no_camera))
    with pytest.raises(RuntimeError, match="ribbon cable"):
        PicameraSource(CameraConfig())


# OpenCvSource ------------------------------------------------------------------------------


def test_opencv_property_ids_are_the_opencv_values() -> None:
    assert (CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT, CAP_PROP_FPS, CAP_PROP_BUFFERSIZE) == (
        3,
        4,
        5,
        38,
    )


def test_opencv_source_opens_the_configured_index_with_size_fps_and_a_small_buffer() -> None:
    capture = FakeCapture()
    indexes: list[int] = []

    def factory(index: int) -> FakeCapture:
        indexes.append(index)
        return capture

    config = CameraConfig(source="usb", usb_index=8, width=320, height=240, fps=10.0)
    OpenCvSource(config, capture_factory=factory)
    assert indexes == [8]
    assert capture.properties == {
        CAP_PROP_FRAME_WIDTH: 320,
        CAP_PROP_FRAME_HEIGHT: 240,
        CAP_PROP_FPS: 10.0,
        CAP_PROP_BUFFERSIZE: 1,
    }


def test_opencv_source_returns_frames_then_none_when_a_grab_fails() -> None:
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    capture = FakeCapture(frames=(frame,))
    source = OpenCvSource(CameraConfig(source="usb"), capture_factory=lambda index: capture)
    assert source.read() is frame
    assert source.read() is None
    source.close()
    assert capture.released


def test_opencv_source_raises_when_the_camera_does_not_open() -> None:
    capture = FakeCapture(opened=False)
    with pytest.raises(RuntimeError, match="v4l2-ctl --list-devices"):
        OpenCvSource(CameraConfig(source="usb", usb_index=3), capture_factory=lambda index: capture)
    assert capture.released


def test_opencv_source_without_opencv_explains_how_to_install(monkeypatch) -> None:
    monkeypatch.setitem(sys.modules, "cv2", None)
    with pytest.raises(RuntimeError, match="opencv-python-headless"):
        OpenCvSource(CameraConfig(source="usb"))


# open_camera -------------------------------------------------------------------------------


def test_open_camera_simulated_gives_a_synthetic_source_of_the_configured_size(
    config: NavSightConfig,
) -> None:
    config.camera.width, config.camera.height = 320, 240
    source = open_camera(config, simulate=True)
    assert isinstance(source, SyntheticSource)
    frame = source.read()
    assert frame.shape == (240, 320, 3)
    assert frame.dtype == np.uint8


def test_open_camera_simulates_when_the_config_says_so(config: NavSightConfig) -> None:
    config.simulate = True
    assert isinstance(open_camera(config), SyntheticSource)


def test_open_camera_csi_uses_picamera2(monkeypatch, config: NavSightConfig) -> None:
    camera = FakePicamera2()
    monkeypatch.setitem(sys.modules, "picamera2", picamera_module(lambda: camera))
    source = open_camera(config)
    assert isinstance(source, PicameraSource)
    assert source.read() is camera.frame


def test_open_camera_usb_uses_opencv(monkeypatch, config: NavSightConfig) -> None:
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    capture = FakeCapture(frames=(frame,))
    module = types.ModuleType("cv2")
    module.VideoCapture = lambda index: capture
    monkeypatch.setitem(sys.modules, "cv2", module)
    config.camera.source = "usb"
    source = open_camera(config)
    assert isinstance(source, OpenCvSource)
    assert source.read() is frame


def test_open_camera_rejects_unknown_sources(config: NavSightConfig) -> None:
    config.camera.source = "rtsp"
    with pytest.raises(ValueError, match="camera.source"):
        open_camera(config)
