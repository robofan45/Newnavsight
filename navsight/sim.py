"""A scripted walk that exercises every NavSight v2 feature on a PC, without hardware.

``navsight --sim`` runs it in real time (``--speed 0`` runs as fast as possible). It drives the
same ``CoreLoop`` and ``Sampler`` as the cane, with simulated devices fed by ``SimWorld``:

  0 s  walking north with a GPS fix
  2 s  a pole ahead, getting closer until the user stops 0.45 m in front of it
  6 s  the user turns east; a person is on the left from 8 s
 12 s  a curb drops away in front of the cane
 15 s  indoors: no GPS (announced after the 10 s timeout) and step tracking takes over
 20 s  the Mode button cycles brief alerts, vibration only, then full alerts again
 32 s  back outside: GPS returns
 35 s  Action button: a long press gives the status, a short press describes what is ahead
 40 s  the battery (whose clock runs 60x faster here) drops below 20 %: low power mode
 45 s  the cane falls and lies flat: fall detected, SOS countdown, SOS recorded locally
"""

from __future__ import annotations

import dataclasses
import json
import math
import tempfile
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from navsight.app import CoreLoop, Devices, Sampler, TickReport
from navsight.config import NavSightConfig, expand_path
from navsight.fusion.ekf import LocalTangentPlane
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
from navsight.safety.sos import make_notifiers
from navsight.types import (
    STANDARD_GRAVITY,
    ButtonEvent,
    ButtonId,
    Detection,
    PressType,
)
from navsight.util import ManualClock, wrap_heading_deg

SCENARIO_DURATION_S = 70.0
STEP_S = 0.01  # simulation resolution: 100 Hz, the IMU rate
START_LATITUDE = 51.4779  # Greenwich Park, London
START_LONGITUDE = -0.0015
STEP_PERIOD_S = 0.54
STEP_LENGTH_M = 0.65
CANE_PITCH_DEG = -55.0
GROUND_DISTANCE_M = 1.05
BATTERY_TIME_SCALE = 60.0
BATTERY_START_SOC = 0.295
BATTERY_CURRENT_MA = 900.0

POLE_APPEARS_S = 2.0
POLE_START_M = 4.0
STOP_DISTANCE_M = 0.45
WALK_SPEED_MPS = STEP_LENGTH_M / STEP_PERIOD_S
STOP_S = POLE_APPEARS_S + (POLE_START_M - STOP_DISTANCE_M) / WALK_SPEED_MPS
TURN_S = 6.0
PERSON_WINDOW_S = (8.0, 11.0)
CURB_WINDOW_S = (12.0, 12.8)
CURB_DROP_M = 0.18
INDOORS_WINDOW_S = (15.0, 32.0)
STAND_STILL_S = 40.0
FALL_S = 45.0
FREEFALL_S = 0.2
IMPACT_S = 0.04

BUTTON_SCRIPT: tuple[tuple[float, ButtonId, PressType], ...] = (
    (20.0, ButtonId.MODE, PressType.SHORT),
    (21.5, ButtonId.MODE, PressType.SHORT),
    (23.0, ButtonId.MODE, PressType.SHORT),
    (35.0, ButtonId.ACTION, PressType.LONG),
    (38.0, ButtonId.ACTION, PressType.SHORT),
)
MARKERS: tuple[tuple[float, str], ...] = (
    (0.0, "Walking north, GPS fix"),
    (POLE_APPEARS_S, "A pole ahead"),
    (STOP_S, "User stops in front of the pole"),
    (TURN_S, "User turns east"),
    (PERSON_WINDOW_S[0], "A person on the left"),
    (CURB_WINDOW_S[0], "A curb drops away ahead"),
    (INDOORS_WINDOW_S[0], "Indoors: no GPS"),
    (20.0, "Mode button presses"),
    (INDOORS_WINDOW_S[1], "Back outside"),
    (35.0, "Action button presses"),
    (STAND_STILL_S, "Standing still"),
    (FALL_S, "The cane falls and lies flat"),
)


def _wobble(t: float, period_s: float, amplitude: float, phase: float = 0.0) -> float:
    """Deterministic sensor noise and motion, so every run (and test) is identical."""
    return amplitude * math.sin(2.0 * math.pi * t / period_s + phase)


class SimWorld:
    """The scripted world. ``apply(t)`` puts the state at time ``t`` into the simulated devices."""

    def __init__(self, config: NavSightConfig) -> None:
        self.config = config
        self.plane = LocalTangentPlane(START_LATITUDE, START_LONGITUDE)
        self.east_m = 0.0
        self.north_m = 0.0
        self._steps = 0
        self._steps_reported = 0
        self._walk_time_s = 0.0
        self._last_t = 0.0
        self._buttons_sent = 0
        self._last_gps_second: int | None = None

    def heading_deg(self, t: float) -> float:
        return 0.0 if t < TURN_S else 90.0

    def walking(self, t: float) -> bool:
        return t < STOP_S or TURN_S <= t < STAND_STILL_S

    def apply(self, t: float, devices: Devices) -> None:
        self._advance_walk(t)
        self._apply_imu(t, devices.imu)
        self._apply_tof(t, devices.tof)
        self._apply_gps(t, devices.gps)
        self._apply_buttons(t, devices.buttons)

    def detections(self, t: float) -> list[Detection]:
        start, end = PERSON_WINDOW_S
        if not start <= t < end:
            return []
        camera = self.config.camera
        bearing = -25.0 + _wobble(t, 1.3, 1.0)
        focal_px = (camera.width / 2.0) / math.tan(math.radians(camera.horizontal_fov_deg / 2.0))
        centre_x = camera.width / 2.0 + focal_px * math.tan(math.radians(bearing))
        box = (centre_x - 40.0, 120.0, centre_x + 40.0, 360.0)
        distance = 3.0 - 0.3 * (t - start)
        return [Detection("person", 0.82, box, bearing, distance)]

    def _advance_walk(self, t: float) -> None:
        if self.walking(t):
            self._walk_time_s += t - self._last_t
            while self._walk_time_s >= (self._steps + 1) * STEP_PERIOD_S:
                self._steps += 1
                heading = math.radians(self.heading_deg(t))
                self.east_m += STEP_LENGTH_M * math.sin(heading)
                self.north_m += STEP_LENGTH_M * math.cos(heading)
        self._last_t = t

    def _apply_imu(self, t: float, imu: SimulatedImu | None) -> None:
        if imu is None:
            return
        gravity = STANDARD_GRAVITY
        if t >= FALL_S + FREEFALL_S + IMPACT_S:
            imu.set_state(
                pitch_deg=0.0,
                accel_mps2=(0.0, 0.0, gravity),
                linear_accel_mps2=(0.0, 0.0, 0.0),
                gyro_dps=(0.0, 0.0, 0.0),
                stationary=True,
            )
        elif t >= FALL_S + FREEFALL_S:
            imu.set_state(accel_mps2=(5.0, 2.0, 4.4 * gravity), gyro_dps=(250.0, 90.0, 40.0))
        elif t >= FALL_S:
            progress = (t - FALL_S) / FREEFALL_S
            imu.set_state(
                pitch_deg=CANE_PITCH_DEG * (1.0 - progress),
                accel_mps2=(0.3, 0.2, 0.5),
                gyro_dps=(180.0, 60.0, 20.0),
                stationary=False,
            )
        elif self.walking(t):
            bounce = _wobble(t, STEP_PERIOD_S, 1.2)
            imu.set_state(
                heading_deg=wrap_heading_deg(self.heading_deg(t) + _wobble(t, 2.1, 3.0)),
                pitch_deg=CANE_PITCH_DEG + _wobble(t, 2 * STEP_PERIOD_S, 4.0),
                accel_mps2=(_wobble(t, STEP_PERIOD_S, 0.4, 1.0), 0.0, gravity + bounce),
                linear_accel_mps2=(_wobble(t, STEP_PERIOD_S, 0.4, 1.0), 0.0, bounce),
                gyro_dps=(_wobble(t, 1.08, 12.0), 0.0, _wobble(t, 1.08, 25.0, 0.5)),
                add_steps=self._new_steps_for_imu(),
                stationary=False,
            )
        else:
            imu.set_state(
                heading_deg=self.heading_deg(t),
                pitch_deg=CANE_PITCH_DEG,
                accel_mps2=(0.0, 0.0, gravity),
                linear_accel_mps2=(0.0, 0.0, 0.0),
                gyro_dps=(0.0, 0.0, 0.0),
                stationary=True,
            )

    def _new_steps_for_imu(self) -> int:
        new = self._steps - self._steps_reported
        self._steps_reported = self._steps
        return new

    def _apply_tof(self, t: float, tof: SimulatedToFPair | None) -> None:
        if tof is None:
            return
        if t >= FALL_S:
            tof.set_distances(None, None)
            return
        forward: float | None = None
        if POLE_APPEARS_S <= t < TURN_S:
            travelled = WALK_SPEED_MPS * (min(t, STOP_S) - POLE_APPEARS_S)
            forward = max(STOP_DISTANCE_M, POLE_START_M - travelled) * (1 + _wobble(t, 0.37, 0.004))
        down = GROUND_DISTANCE_M + _wobble(t, 0.29, 0.004)
        if CURB_WINDOW_S[0] <= t < CURB_WINDOW_S[1]:
            down += CURB_DROP_M
        tof.set_distances(forward, down)

    def _apply_gps(self, t: float, gps: SimulatedGps | None) -> None:
        """One fix per second, like the real receiver."""
        second = int(t)
        if gps is None or second == self._last_gps_second:
            return
        self._last_gps_second = second
        if INDOORS_WINDOW_S[0] <= t < INDOORS_WINDOW_S[1]:
            gps.set_no_fix()
            return
        east = self.east_m + _wobble(t, 7.0, 0.8)
        north = self.north_m + _wobble(t, 9.0, 0.8, 1.0)
        latitude, longitude = self.plane.to_geodetic(east, north)
        gps.set_fix(latitude, longitude, hdop=0.9, satellites=14)

    def _apply_buttons(self, t: float, buttons: SimulatedButtons | None) -> None:
        while self._buttons_sent < len(BUTTON_SCRIPT) and BUTTON_SCRIPT[self._buttons_sent][0] <= t:
            _, button, press = BUTTON_SCRIPT[self._buttons_sent]
            if buttons is not None:
                buttons.inject(ButtonEvent(button, press, t))
            self._buttons_sent += 1


class ScriptedDetector:
    """Stands in for YOLO: returns the world's scripted detections for the current time."""

    def __init__(self, world: SimWorld, clock: Callable[[], float]) -> None:
        self._world = world
        self._clock = clock

    def detect(self, frame: object) -> list[Detection]:
        return self._world.detections(self._clock())


@dataclass
class SimulationResult:
    transcript: list[str] = field(default_factory=list)
    spoken: list[tuple[float, str]] = field(default_factory=list)
    patterns: list[tuple[float, str]] = field(default_factory=list)
    haptic_peaks: list[tuple[float, float, float]] = field(default_factory=list)
    beeps: int = 0
    power_states: list[tuple[float, str]] = field(default_factory=list)
    sos_log: Path | None = None


def simulation_config(config: NavSightConfig, work_dir: Path) -> NavSightConfig:
    """The user's config with every external side effect pointed at a scratch folder."""
    sos = dataclasses.replace(
        config.sos,
        webhook_url=None,
        telegram_bot_token=None,
        telegram_chat_id=None,
        log_file=str(work_dir / "sos.log"),
    )
    power = dataclasses.replace(config.power, state_file=str(work_dir / "battery.json"))
    return dataclasses.replace(config, sos=sos, power=power, simulate=True)


def build_simulated_devices(config: NavSightConfig, clock: ManualClock, world: SimWorld) -> Devices:
    def battery_clock() -> float:
        return clock() * BATTERY_TIME_SCALE

    estimator = StateOfChargeEstimator(
        config.power, _seed_battery_state(config, BATTERY_START_SOC), clock=battery_clock
    )
    sensor = SimulatedPowerSensor(bus_voltage=5.1, current_ma=BATTERY_CURRENT_MA)
    gpio = FakeGpio()
    return Devices(
        tof=SimulatedToFPair(config, clock=clock),
        imu=SimulatedImu(config, clock=clock),
        gps=SimulatedGps(config.gps, clock=clock),
        power=PowerMonitor(config, lambda: sensor, estimator, clock=battery_clock),
        buttons=SimulatedButtons(),
        haptics=HapticController(config, gpio, clock=clock),
        audio=AudioOutput(config, NullBackend(), RecordingTts(), synchronous=True),
        camera=open_camera(config, simulate=True),
        detector=ScriptedDetector(world, clock),
        gpio=gpio,
    )


def _seed_battery_state(config: NavSightConfig, soc: float) -> Path:
    """Write the estimator's state file so the simulated battery starts at ``soc``."""
    usable_mah = StateOfChargeEstimator(config.power, None).usable_capacity_mah
    path = expand_path(config.power.state_file)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"consumed_mah": (1.0 - soc) * usable_mah}), encoding="utf-8")
    return path


def _run_now(job: Callable[[], None]) -> None:
    job()


class _Printer:
    """Turns tick reports into a readable timeline without flooding it with repeats."""

    def __init__(self, result: SimulationResult, output: Callable[[str], None] | None) -> None:
        self.result = result
        self.output = output
        self._levels = (0.0, 0.0)
        self._last_beep_line = -math.inf
        self._marker_index = 0
        self._state: str | None = None

    def line(self, t: float, text: str) -> None:
        entry = f"[{t:5.1f} s] {text}"
        self.result.transcript.append(entry)
        if self.output is not None:
            self.output(entry)

    def markers(self, t: float) -> None:
        while self._marker_index < len(MARKERS) and MARKERS[self._marker_index][0] <= t:
            self.line(MARKERS[self._marker_index][0], f"--- {MARKERS[self._marker_index][1]} ---")
            self._marker_index += 1

    def report(self, report: TickReport) -> None:
        t, outputs = report.now, report.outputs
        state = report.power_state.value
        if state != self._state:
            self.result.power_states.append((t, state))
            if self._state is not None:
                self.line(t, f"POWER {state}")
            self._state = state
        for speech in outputs.speech:
            self.result.spoken.append((t, speech.text))
            self.line(t, f"SAY   {speech.text}" + ("  (interrupts)" if speech.interrupt else ""))
        if outputs.haptic_pattern:
            self.result.patterns.append((t, outputs.haptic_pattern))
            self.line(t, f"BUZZ  {outputs.haptic_pattern}")
        # Below 5 % the motors are effectively off; print only clear changes, not jitter.
        levels = tuple(
            level if level >= 0.05 else 0.0 for level in (outputs.haptic_left, outputs.haptic_right)
        )
        if max(abs(a - b) for a, b in zip(levels, self._levels, strict=True)) >= 0.2 or (
            any(levels) != any(self._levels)
        ):
            self.result.haptic_peaks.append((t, *levels))
            self.line(t, f"VIBE  left {levels[0]:.0%}  right {levels[1]:.0%}")
            self._levels = levels
        for beep in outputs.beeps:
            self.result.beeps += 1
            if t - self._last_beep_line >= 1.0:
                side = (
                    "left"
                    if beep.bearing_deg < -10
                    else "right"
                    if beep.bearing_deg > 10
                    else "ahead"
                )
                self.line(t, f"BEEP  {beep.frequency_hz:.0f} Hz, {side}")
                self._last_beep_line = t


def run_simulation(
    config: NavSightConfig,
    *,
    duration_s: float | None = None,
    speed: float = 1.0,
    output: Callable[[str], None] | None = print,
    work_dir: Path | None = None,
) -> SimulationResult:
    """Run the scripted walk. ``speed`` 1 = real time, 0 = as fast as possible."""
    duration = SCENARIO_DURATION_S if duration_s is None else duration_s
    work_dir = Path(tempfile.mkdtemp(prefix="navsight-sim-")) if work_dir is None else work_dir
    config = simulation_config(config, work_dir)
    clock = ManualClock(0.0)
    world = SimWorld(config)
    devices = build_simulated_devices(config, clock, world)
    core = CoreLoop(
        config,
        devices,
        clock=clock,
        notifiers=make_notifiers(config.sos),
        sos_runner=_run_now,  # deterministic: simulated time must not race a send thread
    )
    sampler = Sampler(core, clock=clock)
    result = SimulationResult(sos_log=Path(config.sos.log_file))
    printer = _Printer(result, output)

    for device in (devices.tof, devices.imu, devices.gps, devices.power, devices.buttons):
        device.start()
    devices.haptics.start()
    devices.audio.start()
    core.announce_startup()

    next_due: dict[str, float] = {}

    def due(name: str, rate_hz: float, t: float) -> bool:
        """True when a device is due; keeps its phase so 30 Hz averages 30 Hz on a 10 ms grid."""
        if rate_hz <= 0:
            next_due.pop(name, None)
            return False
        period = 1.0 / rate_hz
        scheduled = next_due.get(name)
        if scheduled is not None and t + 1e-9 < scheduled:
            return False
        base = scheduled if scheduled is not None and t - scheduled < period else t
        next_due[name] = base + period
        return True

    steps = int(round(duration / STEP_S))
    for index in range(steps):
        t = index * STEP_S
        printer.markers(t)
        world.apply(t, devices)
        if due("imu", config.imu.rate_hz, t):
            sampler.sample_imu()
        if due("tof", core.profile.tof_rate_hz, t):
            sampler.sample_tof()
        vision_rate = core.profile.camera_fps if core.camera_enabled else 0.0
        if due("vision", vision_rate, t):
            sampler.sample_vision()
        if due("power", config.power.read_hz, t):
            sampler.sample_power()
        if due("tick", core.profile.fusion_rate_hz, t):
            printer.report(core.tick(t))
        sampler.update_haptics()
        clock.advance(STEP_S)
        if speed > 0:
            time.sleep(STEP_S / speed)

    for device in (devices.haptics, devices.audio, devices.tof, devices.imu, devices.gps):
        device.stop()
    devices.power.stop()
    if result.sos_log is not None and result.sos_log.exists():
        printer.line(clock(), f"SOS log written to {result.sos_log}")
    return result
