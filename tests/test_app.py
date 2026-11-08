"""Integration tests: the real CoreLoop and Sampler driven by simulated devices."""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from navsight.app import CoreLoop, Devices, NavSightApp, Sampler, TickReport, compass_point
from navsight.config import NavSightConfig, load_config
from navsight.hardware.audio import AudioOutput, NullBackend, RecordingTts
from navsight.hardware.buttons import SimulatedButtons
from navsight.hardware.camera import open_camera
from navsight.hardware.gpio import FakeGpio
from navsight.hardware.gps import SimulatedGps
from navsight.hardware.haptics import HapticController
from navsight.hardware.imu import SimulatedImu
from navsight.hardware.power_monitor import (
    PowerMonitor,
    SimulatedPowerSensor,
    StateOfChargeEstimator,
)
from navsight.hardware.tof import SimulatedToFPair
from navsight.safety.sos import SosMessage, SosState
from navsight.types import (
    BatteryStatus,
    ButtonEvent,
    ButtonId,
    Detection,
    PowerState,
    PressType,
    Verbosity,
)
from navsight.util import ManualClock

START_LATITUDE = 51.4779
START_LONGITUDE = -0.0015


class RecordingNotifier:
    name = "recording"
    remote = True

    def __init__(self) -> None:
        self.messages: list[SosMessage] = []

    def send(self, message: SosMessage) -> None:
        self.messages.append(message)


class FakeDetector:
    def __init__(self) -> None:
        self.detections: list[Detection] = []

    def detect(self, frame: np.ndarray) -> list[Detection]:
        return list(self.detections)


@dataclass
class Rig:
    config: NavSightConfig
    clock: ManualClock
    devices: Devices
    core: CoreLoop
    sampler: Sampler
    tts: RecordingTts
    notifier: RecordingNotifier
    gpio: FakeGpio
    detector: FakeDetector | None
    reports: list[TickReport] = field(default_factory=list)

    @property
    def spoken(self) -> list[str]:
        return list(self.tts.spoken)

    def press(self, button: ButtonId, press: PressType) -> None:
        self.devices.buttons.inject(ButtonEvent(button, press, self.clock()))

    def run(self, seconds: float, on_step: Callable[[float], None] | None = None) -> None:
        """Sample like the cane: IMU 100 Hz, ToF 50 Hz, battery 1 Hz, core loop about 33 Hz."""
        start = self.clock()
        for index in range(round(seconds / 0.01)):
            if on_step is not None:
                on_step(self.clock() - start)
            if self.devices.imu is not None:
                self.sampler.sample_imu()
            if index % 2 == 0 and self.devices.tof is not None:
                self.sampler.sample_tof()
            if index % 100 == 0 and self.devices.power is not None:
                self.sampler.sample_power()
            if index % 3 == 0:
                self.reports.append(self.core.tick())
            self.sampler.update_haptics()
            self.clock.advance(0.01)


def make_rig(
    tmp_path: Path,
    *,
    imu: bool = True,
    camera: bool = True,
    battery_soc: float | None = None,
    overrides: dict[str, Any] | None = None,
) -> Rig:
    settings: dict[str, Any] = {
        "power": {"state_file": str(tmp_path / "battery.json")},
        "sos": {"log_file": str(tmp_path / "sos.log")},
    }
    for section, values in (overrides or {}).items():
        settings.setdefault(section, {}).update(values)
    config = load_config(overrides=settings)
    clock = ManualClock(100.0)
    state_path = None
    if battery_soc is not None:
        usable = StateOfChargeEstimator(config.power, None).usable_capacity_mah
        state_path = tmp_path / "battery.json"
        state_path.write_text(json.dumps({"consumed_mah": (1 - battery_soc) * usable}))
    estimator = StateOfChargeEstimator(config.power, state_path, clock=clock)
    sensor = SimulatedPowerSensor()
    gpio = FakeGpio()
    tts = RecordingTts()
    detector = FakeDetector() if camera else None
    devices = Devices(
        tof=SimulatedToFPair(config, clock=clock),
        imu=SimulatedImu(config, clock=clock) if imu else None,
        gps=SimulatedGps(config.gps, clock=clock),
        power=PowerMonitor(config, lambda: sensor, estimator, clock=clock),
        buttons=SimulatedButtons(),
        haptics=HapticController(config, gpio, clock=clock),
        audio=AudioOutput(config, NullBackend(), tts, synchronous=True),
        camera=open_camera(config, simulate=True) if camera else None,
        detector=detector,
        gpio=gpio,
    )
    for device in (devices.tof, devices.imu, devices.gps, devices.power, devices.buttons):
        if device is not None:
            device.start()
    devices.haptics.start()
    devices.audio.start()
    notifier = RecordingNotifier()
    core = CoreLoop(
        config, devices, clock=clock, notifiers=[notifier], sos_runner=lambda job: job()
    )
    return Rig(
        config, clock, devices, core, Sampler(core, clock=clock), tts, notifier, gpio, detector
    )


@pytest.fixture
def rig(tmp_path: Path) -> Rig:
    return make_rig(tmp_path)


def test_compass_points() -> None:
    assert compass_point(0) == "north"
    assert compass_point(44) == "north east"
    assert compass_point(90) == "east"
    assert compass_point(337.6) == "north"
    assert compass_point(-90) == "west"


def test_startup_announcement_lists_unavailable_devices(rig: Rig) -> None:
    rig.devices.unavailable.append("camera")
    rig.core.announce_startup()
    rig.run(0.1)
    assert "NavSight ready. Camera not available." in rig.spoken


def test_mode_button_cycles_verbosity(rig: Rig) -> None:
    for _ in range(3):
        rig.press(ButtonId.MODE, PressType.SHORT)
        rig.run(2.0)
    assert rig.spoken == ["Brief alerts.", "Vibration only.", "Full alerts."]
    assert rig.core.alerts.verbosity is Verbosity.FULL


def test_mode_long_press_toggles_the_camera(rig: Rig) -> None:
    rig.press(ButtonId.MODE, PressType.LONG)
    rig.run(2.0)
    assert not rig.core.camera_enabled
    rig.press(ButtonId.MODE, PressType.LONG)
    rig.run(2.0)
    assert rig.core.camera_enabled
    assert rig.spoken == ["Camera off.", "Camera on."]


def test_mode_long_press_without_a_camera(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, camera=False)
    rig.press(ButtonId.MODE, PressType.LONG)
    rig.run(1.0)
    assert rig.spoken == ["Camera not available."]


def test_action_press_describes_what_is_ahead_and_the_heading(rig: Rig) -> None:
    rig.devices.imu.set_state(heading_deg=90.0)
    rig.devices.tof.set_distances(1.2, None)
    rig.run(0.5)
    rig.press(ButtonId.ACTION, PressType.SHORT)
    rig.run(1.0)
    assert "Obstacle ahead, 1 meter. Facing east." in rig.spoken


def test_action_press_with_a_clear_path(rig: Rig) -> None:
    rig.devices.imu.set_state(heading_deg=180.0)
    rig.run(0.5)
    rig.press(ButtonId.ACTION, PressType.SHORT)
    rig.run(1.0)
    assert rig.spoken == ["Clear ahead. Facing south."]


def test_action_long_press_reports_status(rig: Rig) -> None:
    rig.run(1.5)
    rig.press(ButtonId.ACTION, PressType.LONG)
    rig.run(1.0)
    assert rig.spoken == ["Battery 100 percent. Waiting for GPS. Full alerts."]


def test_sos_short_press_explains_the_hold(rig: Rig) -> None:
    rig.press(ButtonId.SOS, PressType.SHORT)
    rig.run(1.0)
    assert rig.spoken == ["Hold for two seconds to send SOS."]
    assert rig.core.sos.state is SosState.IDLE


def test_any_button_cancels_a_fall_countdown(rig: Rig) -> None:
    rig.core.sos.trigger_fall(rig.clock())
    rig.run(1.0)
    rig.press(ButtonId.ACTION, PressType.SHORT)
    rig.run(20.0)
    assert rig.core.sos.state is not SosState.COUNTDOWN
    assert "SOS cancelled." in rig.spoken
    assert rig.notifier.messages == []


def test_holding_sos_during_a_fall_countdown_sends_at_once(rig: Rig) -> None:
    rig.core.sos.trigger_fall(rig.clock())
    rig.run(1.0)
    rig.press(ButtonId.SOS, PressType.LONG)
    rig.run(1.0)
    assert len(rig.notifier.messages) == 1
    assert rig.notifier.messages[0].reason == "fall"
    assert "Sending SOS." in rig.spoken
    assert "SOS cancelled." not in rig.spoken


def test_holding_sos_sends_a_manual_sos_with_the_location(rig: Rig) -> None:
    rig.devices.gps.set_fix(START_LATITUDE, START_LONGITUDE)
    rig.run(2.0, on_step=lambda t: rig.devices.gps.set_fix(START_LATITUDE, START_LONGITUDE))
    rig.press(ButtonId.SOS, PressType.LONG)
    rig.run(1.0)
    [message] = rig.notifier.messages
    assert message.reason == "manual"
    assert message.latitude == pytest.approx(START_LATITUDE, abs=1e-4)
    assert message.longitude == pytest.approx(START_LONGITUDE, abs=1e-4)
    assert message.battery_percent == 100
    assert "SOS sent." in rig.spoken


def _learn_ground_then(rig: Rig, pitch_deg: float, down_m: float | None) -> None:
    rig.devices.imu.set_state(pitch_deg=-55.0, stationary=False)
    rig.devices.tof.set_distances(None, 1.0)
    rig.run(1.0)
    rig.devices.imu.set_state(pitch_deg=pitch_deg)
    rig.devices.tof.set_distances(None, down_m)
    rig.run(1.0)


def test_no_echo_from_the_down_sensor_while_walking_is_a_drop(rig: Rig) -> None:
    _learn_ground_then(rig, pitch_deg=-55.0, down_m=None)
    assert rig.spoken.count("Drop ahead.") == 1


def test_a_cane_lying_flat_never_reports_a_drop(rig: Rig) -> None:
    _learn_ground_then(rig, pitch_deg=0.0, down_m=None)
    assert "Drop ahead." not in rig.spoken


def test_ground_hazards_are_ignored_during_an_sos(rig: Rig) -> None:
    rig.devices.imu.set_state(pitch_deg=-55.0, stationary=False)
    rig.devices.tof.set_distances(None, 1.0)
    rig.run(1.0)
    rig.core.sos.trigger_fall(rig.clock())
    rig.devices.tof.set_distances(None, None)
    rig.run(1.0)
    assert "Drop ahead." not in rig.spoken


def _gps_every_second(rig: Rig, until_s: float) -> Callable[[float], None]:
    def step(t: float) -> None:
        if t < until_s and abs(t - round(t)) < 0.005:
            rig.devices.gps.set_fix(START_LATITUDE, START_LONGITUDE)
        elif t >= until_s and abs(t - round(t)) < 0.005:
            rig.devices.gps.set_no_fix()

    return step


def test_gps_lost_and_back_are_each_announced_once(rig: Rig) -> None:
    rig.devices.imu.set_state(stationary=False)
    rig.run(16.0, on_step=_gps_every_second(rig, until_s=3.0))
    assert rig.spoken.count("GPS lost. Using step tracking.") == 1
    rig.run(3.0, on_step=_gps_every_second(rig, until_s=10.0))
    assert rig.spoken.count("GPS signal back.") == 1
    assert rig.spoken.count("GPS lost. Using step tracking.") == 1


def test_gps_announcements_stay_quiet_while_the_gps_is_duty_cycled(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, battery_soc=0.15)
    rig.devices.imu.set_state(stationary=False)
    rig.run(16.0, on_step=_gps_every_second(rig, until_s=3.0))
    assert rig.core.power_state.state is PowerState.LOW_POWER
    assert rig.core.profile.gps_poll_interval_s > 0
    assert not any(text.startswith("GPS") for text in rig.spoken)


def test_each_gps_epoch_reaches_the_pose_filter_once(rig: Rig) -> None:
    calls: list[float] = []
    original = rig.core.pose.on_gps

    def counting(fix):  # noqa: ANN001, ANN202 - a thin spy
        calls.append(fix.timestamp)
        return original(fix)

    rig.core.pose.on_gps = counting
    rig.run(0.1)
    rig.devices.gps.set_fix(START_LATITUDE, START_LONGITUDE)  # GGA
    rig.run(0.1)
    rig.devices.gps.set_fix(START_LATITUDE, START_LONGITUDE)  # RMC of the same epoch
    rig.run(0.1)
    assert len(calls) == 1


def test_without_an_imu_the_cane_never_powers_down(tmp_path: Path) -> None:
    overrides = {"power": {"idle_to_low_power_s": 1.0, "idle_to_sleep_s": 2.0}}
    rig = make_rig(tmp_path, imu=False, overrides=overrides)
    rig.run(4.0)
    assert rig.core.power_state.state is PowerState.ACTIVE


def test_an_idle_cane_powers_down_and_says_so(tmp_path: Path) -> None:
    overrides = {"power": {"idle_to_low_power_s": 1.0, "idle_to_sleep_s": 2.0}}
    rig = make_rig(tmp_path, overrides=overrides)
    rig.devices.imu.set_state(stationary=True)
    rig.run(4.0)
    assert rig.core.power_state.state is PowerState.SLEEP
    assert rig.core.profile.tof_rate_hz == 0
    assert "Sleeping. Move the cane to wake it." in rig.spoken


def _battery(soc: float, voltage: float = 5.1) -> BatteryStatus:
    return BatteryStatus(0.0, voltage, 900.0, 4.5, soc, None)


def test_battery_warnings_round_down_and_do_not_repeat(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.devices.power = None  # drive the battery slot directly
    for soc in (0.30, 0.199, 0.19, 0.18):
        rig.core.battery.set(_battery(soc))
        rig.run(2.0)
    assert rig.spoken.count("Battery low, 19 percent.") == 1
    rig.core.battery.set(_battery(0.079))
    rig.run(2.0)
    assert "Battery critical, 7 percent. Please charge the cane." in rig.spoken
    rig.core.battery.set(_battery(0.30))
    rig.run(2.0)
    rig.core.battery.set(_battery(0.19))
    rig.run(2.0)
    assert rig.spoken.count("Battery low, 19 percent.") == 2


def test_low_supply_voltage_is_announced_once(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    rig.devices.power = None
    for _ in range(3):
        rig.core.battery.set(_battery(0.9, voltage=4.6))
        rig.run(2.0)
    assert rig.spoken.count("Power supply voltage is low.") == 1


def test_vision_uses_the_capture_time_and_a_fresh_tof_anchor(rig: Rig) -> None:
    class FakeDepth:
        def infer(self, frame: np.ndarray) -> np.ndarray:
            depth = np.ones(frame.shape[:2], dtype=np.float32)
            depth[:, :100] = 0.5  # the left side is twice as far as the centre
            return depth

    rig.devices.depth = FakeDepth()
    rig.devices.tof.set_distances(1.5, None)
    rig.sampler.sample_tof()
    box = (10.0, 100.0, 90.0, 300.0)
    rig.detector.detections = [Detection("person", 0.9, box, -40.0)]
    captured = rig.clock()
    rig.sampler.sample_vision()
    captured_at, [detection] = rig.core.detections.get()
    assert captured_at == captured
    assert detection.distance_m == pytest.approx(3.0, rel=0.05)

    rig.clock.advance(1.0)  # the forward reading is now too old to anchor the depth scale
    rig.sampler.sample_vision()
    _, [detection] = rig.core.detections.get()
    assert detection.distance_m is None


def test_camera_detections_are_ignored_while_the_camera_is_off(rig: Rig) -> None:
    rig.core.camera_enabled = False
    rig.detector.detections = [Detection("person", 0.9, (0, 0, 10, 10), -30.0, 2.0)]
    rig.sampler.sample_vision()
    rig.run(0.1)
    assert rig.core.obstacle_map.obstacles(rig.clock()) == []


class _Failing:
    def start(self) -> None:
        raise OSError("not connected")

    def stop(self) -> None:
        raise AssertionError("a device that failed to start must not be stopped")


def test_app_marks_devices_that_fail_to_start_and_stops_cleanly(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    app = NavSightApp(rig.config, rig.devices, clock=rig.clock)
    rig.devices.gps = _Failing()
    app.start()
    try:
        assert rig.devices.gps is None
        assert "GPS" in rig.devices.unavailable
        assert {"tof", "imu", "power", "buttons", "haptics", "vision"} <= set(app._workers)
    finally:
        app.stop()
    assert all(duty == 0 for _frequency, duty in rig.gpio.pwm_state.values())


def test_app_run_survives_a_failing_tick(tmp_path: Path) -> None:
    rig = make_rig(tmp_path)
    app = NavSightApp(rig.config, Devices(), clock=lambda: rig.clock.advance(0.05))
    calls = 0

    def failing_tick(now: float | None = None) -> None:
        nonlocal calls
        calls += 1
        raise RuntimeError("boom")

    app.core.tick = failing_tick
    app.run(duration_s=0.5)
    assert calls >= 2


def test_app_applies_the_gps_duty_cycle_from_the_power_profile(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, battery_soc=0.15)
    app = NavSightApp(rig.config, rig.devices, clock=rig.clock)
    rig.sampler = app.sampler
    rig.core = app.core
    rig.run(2.0)
    app._apply_profile()
    assert rig.devices.gps.poll_interval_s == rig.config.power.low_power_gps_interval_s
