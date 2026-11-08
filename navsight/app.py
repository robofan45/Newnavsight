"""Wires NavSight v2 together: devices, sensor sampling, the core loop and its outputs.

Sensor ``Worker`` threads only read devices and fill the core loop's queues and slots
(see ``Sampler``). ``CoreLoop.tick`` runs at the fusion rate and is the only code that touches
fusion, alert, power and SOS state, so none of that state needs locks.
"""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Protocol, TypeVar

import numpy as np

from navsight.alerts.priority import AlertManager, direction_phrase, distance_phrase
from navsight.config import NavSightConfig, expand_path
from navsight.fusion.ekf import PoseEstimator
from navsight.fusion.fall import FallDetector
from navsight.fusion.ground import GroundHazardDetector
from navsight.fusion.obstacle_map import ObstacleMap
from navsight.hardware.audio import AudioOutput, make_backend, make_tts
from navsight.hardware.buttons import Buttons, SimulatedButtons
from navsight.hardware.camera import FrameSource, open_camera
from navsight.hardware.gpio import GpioBackend, LgpioBackend
from navsight.hardware.gps import GpsReader, SimulatedGps
from navsight.hardware.haptics import HapticController
from navsight.hardware.i2c import open_i2c
from navsight.hardware.imu import Imu, SimulatedImu, make_bno085_factory
from navsight.hardware.power_monitor import (
    PowerMonitor,
    StateOfChargeEstimator,
    make_ina219_factory,
)
from navsight.hardware.tof import SimulatedToFPair, ToFPair, make_vl53l1x_factory
from navsight.perception.depth import DepthEstimator, attach_distances
from navsight.perception.detector import ObjectDetector
from navsight.power.state_machine import PowerInputs, PowerStateMachine
from navsight.safety.sos import Notifier, SosController, SosState, make_notifiers
from navsight.types import (
    Alert,
    AlertLevel,
    AlertOutputs,
    BatteryStatus,
    ButtonEvent,
    ButtonId,
    Detection,
    GroundHazard,
    ImuSample,
    PowerState,
    PressType,
    ToFReading,
    Verbosity,
)
from navsight.util import Clock, LatestValue, SampleQueue, Worker, clamp

log = logging.getLogger(__name__)

T = TypeVar("T")

COMPASS_POINTS = (
    "north",
    "north east",
    "east",
    "south east",
    "south",
    "south west",
    "west",
    "north west",
)
VERBOSITY_CYCLE = (Verbosity.FULL, Verbosity.BRIEF, Verbosity.HAPTIC_ONLY)
VERBOSITY_NAMES = {
    Verbosity.FULL: "Full alerts.",
    Verbosity.BRIEF: "Brief alerts.",
    Verbosity.HAPTIC_ONLY: "Vibration only.",
}
ANCHOR_MAX_AGE_S = 0.5  # an older forward ToF reading is not used to scale camera depth
# The receiver sends GGA and RMC for the same 1 Hz epoch; the EKF must see each epoch only once.
GPS_MIN_UPDATE_INTERVAL_S = 0.5


class Detector(Protocol):
    def detect(self, frame: np.ndarray) -> list[Detection]: ...


@dataclass
class Devices:
    """Everything the core loop talks to. A device that is missing or failed to start is None."""

    tof: ToFPair | SimulatedToFPair | None = None
    imu: Imu | SimulatedImu | None = None
    gps: GpsReader | SimulatedGps | None = None
    power: PowerMonitor | None = None
    buttons: Buttons | SimulatedButtons | None = None
    haptics: HapticController | None = None
    audio: AudioOutput | None = None
    camera: FrameSource | None = None
    detector: Detector | None = None
    depth: DepthEstimator | None = None
    gpio: GpioBackend | None = None
    unavailable: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class TickReport:
    now: float
    outputs: AlertOutputs
    power_state: PowerState
    events: tuple[Alert, ...]


def compass_point(heading_deg: float) -> str:
    return COMPASS_POINTS[int((heading_deg % 360.0 + 22.5) // 45.0) % 8]


def sentence(text: str) -> str:
    return text[:1].upper() + text[1:]


class CoreLoop:
    """One fusion step per ``tick``: sensors in, haptics, beeps and speech out."""

    def __init__(
        self,
        config: NavSightConfig,
        devices: Devices,
        *,
        clock: Clock = time.monotonic,
        notifiers: list[Notifier] | None = None,
        sos_runner: Callable[[Callable[[], None]], None] | None = None,
    ) -> None:
        self.config = config
        self.devices = devices
        self._clock = clock
        self.tof_samples: SampleQueue[ToFReading] = SampleQueue(maxlen=256)
        self.imu_samples: SampleQueue[ImuSample] = SampleQueue(maxlen=512)
        # (frame capture time, detections): ranges are placed at the moment the frame was taken,
        # not when YOLO and MiDaS finished 100-200 ms later.
        self.detections: LatestValue[tuple[float, list[Detection]]] = LatestValue()
        self.battery: LatestValue[BatteryStatus] = LatestValue()

        self.pose = PoseEstimator(config)
        self.obstacle_map = ObstacleMap(config)
        self.ground = GroundHazardDetector(config)
        self.fall = FallDetector(config)
        self.alerts = AlertManager(config, clock=clock)
        self.power_state = PowerStateMachine(config, clock=clock)
        if notifiers is None:
            notifiers = make_notifiers(config.sos)
        # The default runner sends on a background thread so a slow network never blocks alerts.
        self.sos = SosController(config, notifiers, clock=clock, runner=sos_runner)
        self.profile = self.power_state.profile_for(PowerState.ACTIVE)
        self.camera_enabled = devices.camera is not None and devices.detector is not None

        self._pending: SampleQueue[Alert] = SampleQueue(maxlen=64)
        self._detections_version = 0
        self._last_fix_time: float | None = None
        self._latest_imu: ImuSample | None = None
        self._pitch_rate_dps: float | None = None
        self._had_gps = False
        self._dead_reckoning = True
        self._gps_quiet_until = -float("inf")
        self._battery_low_warned = False
        self._battery_critical_warned = False
        self._low_voltage_warned = False

    def announce(
        self,
        key: str,
        message: str,
        level: AlertLevel = AlertLevel.INFO,
        haptic_pattern: str | None = None,
    ) -> None:
        """Queue a spoken event for the next tick. Safe to call from any thread."""
        self._pending.put(Alert(key, level, message, haptic_pattern, timestamp=self._clock()))

    def announce_startup(self) -> None:
        missing = " ".join(f"{sentence(name)} not available." for name in self.devices.unavailable)
        level = AlertLevel.WARNING if missing else AlertLevel.INFO
        self.announce("startup", f"NavSight ready. {missing}".strip(), level, "confirm")

    def tick(self, now: float | None = None) -> TickReport:
        now = self._clock() if now is None else now
        events = self._pending.drain()

        moving = self.devices.imu is None  # without an IMU, never assume the cane is idle
        for sample in self.imu_samples.drain():
            moving = self._process_imu(sample) or moving
        new_hazard: GroundHazard | None = None
        for reading in self.tof_samples.drain():
            new_hazard = self._process_tof(reading) or new_hazard
        self._process_detections()
        events.extend(self._process_gps(now))
        soc = self._process_battery(events)

        button_pressed = False
        for event in self._drain_buttons():
            button_pressed = True
            events.extend(self._handle_button(event, now))

        events.extend(self.sos.tick(now, self.location(now), soc))
        events.extend(self._update_power(now, soc, moving, button_pressed))

        outputs = self.alerts.update(now, self.obstacle_map.obstacles(now), new_hazard, events)
        self._drive_outputs(outputs)
        return TickReport(now, outputs, self.power_state.state, tuple(events))

    def location(self, now: float) -> tuple[float, float] | None:
        pose = self.pose.pose(now)
        if pose is not None and pose.latitude is not None and pose.longitude is not None:
            return pose.latitude, pose.longitude
        fix = self.devices.gps.latest if self.devices.gps is not None else None
        if fix is not None and fix.has_fix:
            return fix.latitude, fix.longitude
        return None

    def describe_surroundings(self, now: float) -> str:
        if self.devices.tof is None and not self.camera_enabled:
            parts = ["Obstacle sensing is not available."]
        else:
            nearest = self.obstacle_map.nearest(now)
            if nearest is None or nearest.distance_m is None:
                parts = ["Clear ahead."]
            else:
                label = nearest.label or "obstacle"
                direction = direction_phrase(nearest.bearing_deg)
                distance = distance_phrase(nearest.distance_m)
                parts = [sentence(f"{label} {direction}, {distance}.")]
        heading = self._heading(now)
        if heading is not None:
            parts.append(f"Facing {compass_point(heading)}.")
        return " ".join(parts)

    def status_report(self, now: float) -> str:
        status = self.battery.get()
        parts = [
            f"Battery {round(status.state_of_charge * 100)} percent."
            if status is not None
            else "Battery level unknown."
        ]
        if self.devices.gps is None:
            parts.append("GPS not available.")
        elif not self.pose.using_dead_reckoning(now):
            parts.append("GPS good.")
        elif self._had_gps:
            parts.append("No GPS, using step tracking.")
        else:
            parts.append("Waiting for GPS.")
        parts.append(VERBOSITY_NAMES[self.alerts.verbosity])
        if self.power_state.state is PowerState.LOW_POWER:
            parts.append("Low power mode.")
        return " ".join(parts)

    def _heading(self, now: float) -> float | None:
        pose = self.pose.pose(now)
        if pose is not None:
            return pose.heading_deg
        return self._latest_imu.heading_deg if self._latest_imu is not None else None

    def _process_imu(self, sample: ImuSample) -> bool:
        previous = self._latest_imu
        self._latest_imu = sample
        if previous is not None and sample.timestamp > previous.timestamp:
            elapsed = sample.timestamp - previous.timestamp
            self._pitch_rate_dps = (sample.pitch_deg - previous.pitch_deg) / elapsed
        self.pose.on_imu(sample)
        if self.fall.update(sample):
            log.warning("Fall detected; starting the SOS countdown")
            self.sos.trigger_fall(sample.timestamp)
        return not sample.stationary

    def _process_tof(self, reading: ToFReading) -> GroundHazard | None:
        if reading.sensor == "forward":
            self.obstacle_map.update_tof(reading)
            return None
        tilt = self._down_sensor_tilt()
        if not self._ground_sensing_possible(tilt):
            return None
        return self.ground.update(reading, tilt, self._pitch_rate_dps)

    def _down_sensor_tilt(self) -> float | None:
        """Angle of the down sensor's axis from vertical: 90 deg plus the (negative) cane pitch."""
        if self._latest_imu is None:
            return None
        return 90.0 + self._latest_imu.pitch_deg + self.config.hazard.down_sensor_offset_deg

    def _ground_sensing_possible(self, tilt: float | None) -> bool:
        """The down sensor sees the ground ahead only while the cane is held for walking.

        A cane lying on the floor (after a fall, or set down) gets no echo, which would
        otherwise read as a drop-off.
        """
        if self.sos.state in (SosState.COUNTDOWN, SosState.SENDING):
            return False
        return tilt is None or abs(tilt) <= self.config.hazard.max_tilt_deg

    def _process_detections(self) -> None:
        latest, version = self.detections.get_with_version()
        if latest is None or version == self._detections_version:
            return
        self._detections_version = version
        captured_at, detections = latest
        if self.camera_enabled:
            self.obstacle_map.update_detections(detections, captured_at)

    def _process_gps(self, now: float) -> list[Alert]:
        gps = self.devices.gps
        fix = gps.latest if gps is not None else None
        had_gps = self._had_gps
        if fix is not None and (
            self._last_fix_time is None
            or fix.timestamp - self._last_fix_time >= GPS_MIN_UPDATE_INTERVAL_S
        ):
            self._last_fix_time = fix.timestamp
            if self.pose.on_gps(fix):
                self._had_gps = True
        dead_reckoning = self.pose.using_dead_reckoning(now)
        events: list[Alert] = []
        # A duty-cycled GPS (low-power mode) is silent most of the time by design: stay quiet
        # until it has run continuously for a full timeout, or "GPS lost" would repeat each cycle.
        if self.profile.gps_poll_interval_s != 0:
            self._gps_quiet_until = now + self.config.pose.gps_timeout_s
        if now < self._gps_quiet_until:
            return events
        if had_gps and dead_reckoning != self._dead_reckoning:
            if dead_reckoning:
                events.append(
                    Alert(
                        "gps_lost",
                        AlertLevel.INFO,
                        "GPS lost. Using step tracking.",
                        None,
                        0.0,
                        now,
                    )
                )
            else:
                events.append(
                    Alert("gps_back", AlertLevel.INFO, "GPS signal back.", None, 0.0, now)
                )
        self._dead_reckoning = dead_reckoning
        return events

    def _process_battery(self, events: list[Alert]) -> float | None:
        status = self.battery.get()
        if status is None:
            return None
        power = self.config.power
        soc = status.state_of_charge
        percent = math.floor(clamp(soc, 0.0, 1.0) * 100)  # never over-report a low battery
        if soc < power.critical_soc and not self._battery_critical_warned:
            self._battery_critical_warned = self._battery_low_warned = True
            events.append(
                Alert(
                    "battery_critical",
                    AlertLevel.WARNING,
                    f"Battery critical, {percent} percent. Please charge the cane.",
                    "low_battery",
                    0.0,
                    status.timestamp,
                )
            )
        elif soc < power.low_soc and not self._battery_low_warned:
            self._battery_low_warned = True
            events.append(
                Alert(
                    "battery_low",
                    AlertLevel.WARNING,
                    f"Battery low, {percent} percent.",
                    "low_battery",
                    0.0,
                    status.timestamp,
                )
            )
        if soc >= power.low_soc + power.soc_hysteresis:
            self._battery_low_warned = self._battery_critical_warned = False
        elif soc >= power.critical_soc + power.soc_hysteresis:
            self._battery_critical_warned = False

        voltage = status.bus_voltage_v
        if 0.5 < voltage < power.low_voltage_v and not self._low_voltage_warned:
            self._low_voltage_warned = True
            events.append(
                Alert(
                    "low_voltage",
                    AlertLevel.WARNING,
                    "Power supply voltage is low.",
                    "low_battery",
                    0.0,
                    status.timestamp,
                )
            )
        elif voltage >= power.low_voltage_v + 0.1:
            self._low_voltage_warned = False
        return soc

    def _drain_buttons(self) -> list[ButtonEvent]:
        return self.devices.buttons.drain() if self.devices.buttons is not None else []

    def _handle_button(self, event: ButtonEvent, now: float) -> list[Alert]:
        button, press = event.button, event.press
        sos_hold = button is ButtonId.SOS and press is PressType.LONG
        if self.sos.state is SosState.COUNTDOWN and not sos_hold:
            self.sos.cancel(now)  # any button cancels a fall countdown; the SOS module announces it
            return []
        if sos_hold:
            # Holding SOS sends now, even during a fall countdown. The SOS module announces
            # "Sending SOS." and ignores the hold if a message is already being sent.
            self.sos.trigger_manual(now)
            return []
        key = f"button:{button.value}:{press.value}:{event.timestamp}"
        if button is ButtonId.MODE and press is PressType.SHORT:
            index = VERBOSITY_CYCLE.index(self.alerts.verbosity)
            self.alerts.verbosity = VERBOSITY_CYCLE[(index + 1) % len(VERBOSITY_CYCLE)]
            message = VERBOSITY_NAMES[self.alerts.verbosity]
            return [Alert(key, AlertLevel.WARNING, message, "mode_change", 0.0, now)]
        if button is ButtonId.MODE and press is PressType.LONG:
            return [Alert(key, AlertLevel.WARNING, self._toggle_camera(), "mode_change", 0.0, now)]
        if button is ButtonId.ACTION and press is PressType.SHORT:
            message = self.describe_surroundings(now)
            return [Alert(key, AlertLevel.WARNING, message, "confirm", 0.0, now)]
        if button is ButtonId.ACTION and press is PressType.LONG:
            return [Alert(key, AlertLevel.WARNING, self.status_report(now), "confirm", 0.0, now)]
        message = "Hold for two seconds to send SOS."
        return [Alert(key, AlertLevel.WARNING, message, None, 0.0, now)]

    def _toggle_camera(self) -> str:
        if self.devices.camera is None or self.devices.detector is None:
            return "Camera not available."
        self.camera_enabled = not self.camera_enabled
        return "Camera on." if self.camera_enabled else "Camera off."

    def _update_power(
        self, now: float, soc: float | None, moving: bool, button_pressed: bool
    ) -> list[Alert]:
        sos_active = self.sos.state in (SosState.COUNTDOWN, SosState.SENDING)
        inputs = PowerInputs(
            now=now,
            state_of_charge=soc,
            moving=moving,
            button_pressed=button_pressed,
            sos_active=sos_active,
        )
        self.profile = self.power_state.update(inputs)
        transition = self.power_state.last_transition
        if transition is None:
            return []
        old, new = transition
        if new is PowerState.SLEEP:
            message = "Sleeping. Move the cane to wake it."
        elif old is PowerState.SLEEP:
            message = "Awake." if new is PowerState.ACTIVE else "Awake. Low power mode."
        else:
            message = "Full power." if new is PowerState.ACTIVE else "Low power mode."
        return [Alert(f"power:{new.value}", AlertLevel.WARNING, message, None, 0.0, now)]

    def _drive_outputs(self, outputs: AlertOutputs) -> None:
        haptics = self.devices.haptics
        if haptics is not None:
            haptics.set_continuous(outputs.haptic_left, outputs.haptic_right)
            if outputs.haptic_pattern:
                try:
                    haptics.play(outputs.haptic_pattern)
                except ValueError:
                    log.error("Unknown haptic pattern %r", outputs.haptic_pattern)
        audio = self.devices.audio
        if audio is not None:
            for beep in outputs.beeps:
                audio.beep(beep.frequency_hz, beep.duration_s, beep.bearing_deg)
            for speech in outputs.speech:
                audio.say(speech.text, interrupt=speech.interrupt)


class Sampler:
    """Reads devices and feeds the core loop.

    On the cane each method runs on its own ``Worker`` thread at its own rate; the simulator
    calls the same methods from one thread, so both paths exercise identical code.
    """

    def __init__(self, core: CoreLoop, *, clock: Clock = time.monotonic) -> None:
        self.core = core
        self.devices = core.devices
        self._clock = clock
        self.forward_tof: LatestValue[ToFReading] = LatestValue()
        self._frame_count = 0
        self._inverse_depth: np.ndarray | None = None

    def sample_tof(self) -> None:
        forward, down = self.devices.tof.read()
        for reading in (forward, down):
            if reading is not None:
                self.core.tof_samples.put(reading)
        if forward is not None:
            self.forward_tof.set(forward)

    def sample_imu(self) -> None:
        self.core.imu_samples.put(self.devices.imu.read())

    def sample_power(self) -> None:
        self.core.battery.set(self.devices.power.read())

    def sample_buttons(self) -> None:
        self.devices.buttons.poll()

    def update_haptics(self) -> None:
        self.devices.haptics.update()

    def sample_vision(self) -> None:
        captured_at = self._clock()
        frame = self.devices.camera.read()
        if frame is None:
            return
        detections = self.devices.detector.detect(frame)
        depth = self.devices.depth
        if depth is not None and self.core.profile.depth_enabled:
            if self._frame_count % max(1, self.core.config.depth.every_n_frames) == 0:
                self._inverse_depth = depth.infer(frame)
            if self._inverse_depth is not None:
                detections = attach_distances(
                    detections, self._inverse_depth, self._anchor_distance(), self.core.config
                )
        self._frame_count += 1
        self.core.detections.set((captured_at, detections))

    def _anchor_distance(self) -> float | None:
        reading = self.forward_tof.get()
        if reading is None or self._clock() - reading.timestamp > ANCHOR_MAX_AGE_S:
            return None
        return reading.distance_m


class NavSightApp:
    """Runs the cane: one ``Worker`` per sensor plus the core loop on the calling thread."""

    def __init__(
        self,
        config: NavSightConfig,
        devices: Devices,
        *,
        clock: Clock = time.monotonic,
        notifiers: list[Notifier] | None = None,
    ) -> None:
        self.config = config
        self.devices = devices
        self._clock = clock
        self.core = CoreLoop(config, devices, clock=clock, notifiers=notifiers)
        self.sampler = Sampler(self.core, clock=clock)
        self._workers: dict[str, Worker] = {}
        self._applied_gps_interval: float | None = 0.0
        self._stop_requested = False

    def start(self) -> None:
        self._start_devices()
        devices, sampler, config = self.devices, self.sampler, self.config
        profile = self.core.profile
        self._add_worker("tof", devices.tof, sampler.sample_tof, profile.tof_rate_hz)
        self._add_worker("imu", devices.imu, sampler.sample_imu, config.imu.rate_hz)
        self._add_worker("power", devices.power, sampler.sample_power, config.power.read_hz)
        self._add_worker("buttons", devices.buttons, sampler.sample_buttons, config.buttons.poll_hz)
        self._add_worker(
            "haptics", devices.haptics, sampler.update_haptics, config.haptics.update_hz
        )
        if self.core.camera_enabled:
            self._add_worker("vision", devices.camera, sampler.sample_vision, profile.camera_fps)
        for worker in self._workers.values():
            worker.start()
        self.core.announce_startup()
        log.info("NavSight started (%d sensor threads)", len(self._workers))

    def run(self, duration_s: float | None = None) -> None:
        started = self._clock()
        last_error_log = -float("inf")
        while not self._stop_requested:
            now = self._clock()
            if duration_s is not None and now - started >= duration_s:
                break
            try:
                self.core.tick(now)
                self._apply_profile()
            except Exception:
                # A mobility aid must keep running: log (at most every 5 s) and carry on.
                if now - last_error_log >= 5.0:
                    last_error_log = now
                    log.exception("Core loop tick failed")
            period = 1.0 / max(self.core.profile.fusion_rate_hz, 0.5)
            time.sleep(max(0.0, period - (self._clock() - now)))

    def request_stop(self) -> None:
        self._stop_requested = True

    def stop(self) -> None:
        for worker in self._workers.values():
            worker.stop()
        self._workers.clear()
        devices = self.devices
        # Motors first, so the cane never keeps vibrating after a crash or shutdown.
        for name in ("haptics", "audio", "tof", "imu", "gps", "power", "buttons"):
            device = getattr(devices, name)
            if device is None:
                continue
            try:
                device.stop()
            except Exception:
                log.exception("Error while stopping %s", name)
        for closable in (devices.camera, devices.gpio):
            if closable is None:
                continue
            try:
                closable.close()
            except Exception:
                log.exception("Error while closing %s", type(closable).__name__)

    def _start_devices(self) -> None:
        devices = self.devices
        names = {
            "tof": "distance sensors",
            "imu": "motion sensor",
            "power": "battery monitor",
            "buttons": "buttons",
            "haptics": "vibration motors",
            "audio": "audio",
            "gps": "GPS",
        }
        for attribute, name in names.items():
            device = getattr(devices, attribute)
            if device is None:
                continue
            try:
                device.start()
            except Exception:
                log.exception("Could not start the %s", name)
                setattr(devices, attribute, None)
                devices.unavailable.append(name)

    def _add_worker(
        self, name: str, device: object | None, step: Callable[[], None], rate_hz: float
    ) -> None:
        if device is not None:
            self._workers[name] = Worker(name, step, rate_hz, clock=self._clock)

    def _apply_profile(self) -> None:
        profile = self.core.profile
        if "tof" in self._workers:
            self._workers["tof"].rate_hz = profile.tof_rate_hz
        if "vision" in self._workers:
            camera_on = self.core.camera_enabled
            self._workers["vision"].rate_hz = profile.camera_fps if camera_on else 0.0
        gps = self.devices.gps
        if gps is not None and profile.gps_poll_interval_s != self._applied_gps_interval:
            gps.set_poll_interval(profile.gps_poll_interval_s)
            self._applied_gps_interval = profile.gps_poll_interval_s


def build_devices(config: NavSightConfig) -> Devices:
    """Open the real Raspberry Pi hardware. Anything that fails is reported, not fatal."""
    devices = Devices()
    devices.gpio = _attempt(devices, "GPIO", lambda: LgpioBackend(config.pins.gpio_chip))
    i2c = _attempt(devices, "I2C bus", lambda: open_i2c(config.pins.i2c_bus))
    if devices.gpio is not None and i2c is not None:
        devices.tof = ToFPair(config, devices.gpio, make_vl53l1x_factory(i2c))
    if i2c is not None:
        # The BNO085 reports at 20 Hz unless asked; the fall detector needs every 10 ms sample.
        imu_factory = make_bno085_factory(i2c, config.i2c.imu, report_rate_hz=config.imu.rate_hz)
        devices.imu = Imu(config, imu_factory)
        estimator = StateOfChargeEstimator(config.power, expand_path(config.power.state_file))
        power_factory = make_ina219_factory(
            i2c, config.i2c.power_monitor, shunt_ohms=config.power.shunt_ohms
        )
        devices.power = PowerMonitor(config, power_factory, estimator)
    devices.gps = GpsReader(config.gps)
    if devices.gpio is not None:
        devices.buttons = Buttons(config, devices.gpio)
        devices.haptics = HapticController(config, devices.gpio)
    devices.audio = AudioOutput(config, make_backend(config.audio), make_tts(config.audio))
    if config.detector.enabled:
        devices.camera = _attempt(devices, "camera", lambda: open_camera(config))
        if devices.camera is not None:
            devices.detector = _attempt(
                devices, "object detector", lambda: _loaded_detector(config)
            )
        if devices.detector is not None and config.depth.enabled:
            devices.depth = _attempt(devices, "depth estimator", lambda: DepthEstimator(config))
    return devices


def _loaded_detector(config: NavSightConfig) -> ObjectDetector:
    """Load YOLO at start-up so a missing model is announced instead of failing every frame."""
    detector = ObjectDetector(config)
    detector.load()
    return detector


def _attempt(devices: Devices, name: str, factory: Callable[[], T]) -> T | None:
    try:
        return factory()
    except Exception:
        log.exception("Could not open the %s", name)
        devices.unavailable.append(name)
        return None
