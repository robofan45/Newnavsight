# NavSight v2 software architecture

This document explains how the NavSight v2 software is organised and the contract each module
implements. It follows the hardware in *NavSight V2 Improved System Architecture*
(Raspberry Pi 5, multi-sensor fusion, assistive navigation smart cane). The design targets are
95 %+ detection accuracy, 15 FPS vision and 6 hours of runtime.

## Hardware map

| Device | Interface | Pins / address | Role |
|---|---|---|---|
| VL53L1X ToF #1 (forward) | I2C | 0x29 | obstacles ahead, 0-4 m, 27° FoV, 50 Hz |
| VL53L1X ToF #2 (down) | I2C | 0x30, XSHUT on GPIO 25 | stairs, curbs and potholes |
| BNO085 IMU (9-DoF) | I2C | 0x4A | heading, steps, fall detection, dead reckoning |
| INA219 power monitor | I2C | 0x40 | battery voltage, current, state of charge |
| Pi Camera Module 3 Wide | CSI-2 | — | YOLOv8n objects and MiDaS depth |
| NEO-M9N GNSS | UART | GPIO 14 TX, GPIO 15 RX | position (GPS, GLONASS, Galileo, BeiDou) |
| Mode / Action / SOS buttons | GPIO in, pull-ups | GPIO 17, 22, 27 | user input |
| MAX98357A I2S DAC + bone conduction | I2S | GPIO 18 BCLK, 19 LRCLK, 21 DIN | beeps and speech |
| Dual ERM haptic motors | PWM | GPIO 12 (left), GPIO 13 (right) | vibration, intensity = distance |
| 10,000 mAh USB-C PD battery | USB-C | via INA219 | 5 V / 3 A supply |

The I2C bus is GPIO 2 (SDA) and GPIO 3 (SCL). GPIO 23 and 24 (the v1 HC-SR04) are now free.

## Conventions

- **Python 3.11** (Raspberry Pi OS Bookworm). No 3.12-only syntax.
- The core needs only the standard library and numpy. Hardware and ML libraries (`lgpio`,
  Blinka/Adafruit drivers, `pyserial`, `picamera2`, `cv2`, `ultralytics`, `torch`,
  `sounddevice`) are imported **inside** the functions that open the device, so the package
  imports and the tests run on any PC.
- Units: metres, seconds, degrees, unless a name says otherwise (`_ms`, `_mah`, `_dps`, `_mps2`).
- Time: `time.monotonic()` seconds. Every class that needs time takes `clock: Clock`
  (default `time.monotonic`), so tests drive it with `navsight.util.ManualClock`.
- Bearing: degrees from the cane/camera axis, negative = left, positive = right.
- Heading: compass degrees, 0 = north, clockwise, in [0, 360).
- Pitch: angle of the cane axis above horizontal; pointing at the ground is negative.
- Shared code: `navsight/types.py` (data types), `navsight/config.py` (all tunables),
  `navsight/util.py` (`clamp`, angle wrapping, `ManualClock`, `LatestValue`, `Worker`),
  `navsight/hardware/gpio.py` (`GpioBackend`, `LgpioBackend`, `FakeGpio`),
  `navsight/hardware/i2c.py` (`open_i2c`, `I2C_LOCK`).
- Every I2C driver call is made while holding `I2C_LOCK`.
- Every device class takes an injectable driver, factory or backend, and has a simulated twin
  used by `navsight --sim` and by the tests.
- Logging uses `logging.getLogger(__name__)`; no `print` outside the CLI and simulator console.

## Package layout

```
navsight/
  __main__.py          CLI: python -m navsight [--sim] [--config FILE] ...
  app.py               builds hardware, runs sensor workers and the core loop
  sim.py               scripted simulated world for --sim and integration tests
  config.py  types.py  util.py
  hardware/
    gpio.py  i2c.py    shared backends
    tof.py             VL53L1X pair, XSHUT re-addressing
    imu.py             BNO085 orientation, steps, stability
    gps.py             NEO-M9N NMEA reader, UBX power commands
    power_monitor.py   INA219 and state-of-charge estimation
    buttons.py         debounced short/long presses
    haptics.py         dual ERM PWM, vibration patterns
    audio.py           spatial beeps, text-to-speech
    camera.py          Picamera2 / OpenCV / synthetic frames
  perception/
    detector.py        YOLOv8n detections with bearings
    depth.py           MiDaS relative depth, scaled by the forward ToF
  fusion/
    ekf.py             pose EKF: GPS + IMU step dead reckoning
    obstacle_map.py    sector map with Kalman range tracks and time-to-collision
    ground.py          drop-off and step-up detection from the down ToF
    fall.py            free-fall, impact and stillness fall detector
  alerts/priority.py   haptic > spatial audio > speech, cooldowns, verbosity
  power/state_machine.py  ACTIVE > LOW_POWER > SLEEP with sensor profiles
  safety/sos.py        SOS countdown, cancel, notifiers with retries
v1/                    the original v1 scripts (HC-SR04 hardware)
```

## Runtime and data flow

```
 sensor workers (threads)                core loop (30 Hz)                     outputs
 ToF 50 Hz ──────┐                       ┌─────────────────────────────┐
 IMU 100 Hz ─────┤   LatestValue slots   │ PoseEstimator (EKF)         │──> haptics (PWM)
 GPS reader ─────┼──────────────────────>│ ObstacleMap + GroundHazard  │──> audio beeps (panned)
 camera+YOLO ────┤                       │ FallDetector  SosController │──> speech (TTS)
 INA219 1 Hz ────┤                       │ PowerStateMachine           │──> SOS notifiers
 buttons 200 Hz ─┘                       │ AlertManager                │
                                         └─────────────────────────────┘
```

Each sensor has a `Worker` thread that calls the device's `read()` and stores the result in a
`LatestValue`. The core loop reads those slots, uses `version` to tell new samples from repeats,
and is the only code that touches fusion, alert and power state. The power profile from
`PowerStateMachine` sets each worker's rate (0 pauses it).

## Module contracts

Unless noted, constructors take `(config: NavSightConfig, ...)` and an optional
`clock: Clock = time.monotonic`. Simulated twins expose the same reading methods as the real
class, plus setters the simulator uses.

### hardware/tof.py: VL53L1X pair

```python
class ToFDriver(Protocol):  # the subset of adafruit_vl53l1x.VL53L1X used here
    distance_mode: int
    timing_budget: int
    @property
    def data_ready(self) -> bool: ...
    @property
    def distance(self) -> float | None: ...  # centimetres; None = no target
    def start_ranging(self) -> None: ...
    def stop_ranging(self) -> None: ...
    def clear_interrupt(self) -> None: ...
    def set_address(self, new_address: int) -> None: ...

DriverFactory = Callable[[int], ToFDriver]  # I2C address -> driver; raises if nothing answers

def assign_tof_addresses(config, gpio, factory, sleep=time.sleep) -> tuple[ToFDriver, ToFDriver]
def make_vl53l1x_factory(i2c) -> DriverFactory
class ToFPair:
    def __init__(self, config, gpio, factory, *, clock=time.monotonic, sleep=time.sleep)
    def start(self) -> None
    def read(self) -> tuple[ToFReading | None, ToFReading | None]  # (forward, down)
    def stop(self) -> None
    error_count: int
class SimulatedToFPair:
    def __init__(self, config, *, clock=time.monotonic)
    def set_distances(self, forward_m: float | None, down_m: float | None) -> None
    start() / read() / stop()  # read() always returns fresh readings for both sensors
```

Both sensors boot at 0x29 and only the down sensor's XSHUT is wired, so
`assign_tof_addresses` must:

1. claim XSHUT (GPIO 25) as an output driven low (down sensor held in reset), sleep 10 ms;
2. open the forward sensor, trying 0x29, `tof_temporary`, `tof_forward`, `tof_down` in that
   order (deduplicated): addresses survive a program restart and only reset on power-off;
3. move the forward sensor to `tof_temporary` (skip if it is already there);
4. drive XSHUT high and sleep 10 ms: the down sensor boots at 0x29;
5. open the down sensor at 0x29 and move it to `tof_down`;
6. move the forward sensor from `tof_temporary` to `tof_forward`.

Every driver call happens under `I2C_LOCK`. A failed step raises `RuntimeError` naming the step
and a wiring hint. `start()` then sets `distance_mode` and `timing_budget` from config and starts
ranging on both. `read()` returns a reading for each sensor that has `data_ready`, converting
centimetres to metres and calling `clear_interrupt()`; a distance of `None`, `<= 0` or above
`max_range_m` becomes `distance_m=None`; a sensor with no new data gives `None`. An `OSError`
from one sensor is counted in `error_count` and gives `None` for that sensor only.

### hardware/imu.py: BNO085

```python
# SH-2 report IDs (the same values as adafruit_bno08x.BNO_REPORT_*)
REPORT_ACCELEROMETER = 0x01; REPORT_GYROSCOPE = 0x02; REPORT_LINEAR_ACCELERATION = 0x04
REPORT_ROTATION_VECTOR = 0x05; REPORT_STEP_COUNTER = 0x11; REPORT_STABILITY_CLASSIFIER = 0x13

class ImuDriver(Protocol):  # the subset of adafruit_bno08x.BNO08X used here
    def enable_feature(self, feature_id: int) -> None: ...
    quaternion: tuple[float, float, float, float]    # (i, j, k, real); world frame East-North-Up
    acceleration: tuple[float, float, float]         # m/s^2 including gravity
    linear_acceleration: tuple[float, float, float]  # m/s^2
    gyro: tuple[float, float, float]                 # rad/s
    steps: int
    stability_classification: str  # "Unknown", "On Table", "Stationary", "Stable", "In motion"

def orientation_from_quaternion(quat_ijkr, forward_axis="y", heading_offset_deg=0.0)
    -> tuple[float, float, float]  # (heading_deg, pitch_deg, roll_deg)
def make_bno085_factory(i2c, address) -> Callable[[], ImuDriver]
class Imu:
    def __init__(self, config, driver_factory, *, clock=time.monotonic)
    def start(self) -> None  # create the driver and enable the six reports
    def read(self) -> ImuSample
    def stop(self) -> None
class SimulatedImu:
    def __init__(self, config, *, clock=time.monotonic)
    def set_state(self, *, heading_deg=None, pitch_deg=None, roll_deg=None, accel_mps2=None,
                  linear_accel_mps2=None, gyro_dps=None, add_steps=0, stationary=None) -> None
    start() / read() / stop()
```

`orientation_from_quaternion` normalises q = (x, y, z, w) = (i, j, k, real), builds the
rotation matrix R (body to world ENU) and rotates the configured forward axis:
`f = R @ axis`. Heading = `atan2(f_east, f_north)` in degrees plus the offset, wrapped to
[0, 360) (clockwise from north). Pitch = `asin(clamp(f_up, -1, 1))`. Roll = the same `asin` of
the world "up" component of a side axis perpendicular to forward (x for a ±y forward axis,
y for ±x, x for ±z). Example: the identity quaternion with forward "y" gives heading 0 and
pitch 0; a +90° rotation about the up axis gives heading 270.

`read()` converts gyro rad/s to deg/s and sets `stationary` when the stability classification is
"On Table" or "Stationary". `step_count` must never decrease: the BNO085 counter can reset or
wrap, so keep an offset that absorbs any drop. Default stationary sample of the simulated twin:
heading 0, pitch -45, roll 0, acceleration (0, 0, 9.80665).

### hardware/gps.py: NEO-M9N

```python
def nmea_checksum_ok(sentence: str) -> bool
def nmea_with_checksum(body: str) -> str             # "PUBX,..." -> "$PUBX,...*HH\r\n"
def parse_lat_lon(value: str, hemisphere: str) -> float | None
class NmeaParser:
    def __init__(self, *, clock=time.monotonic)
    def feed(self, line: str) -> GpsFix | None
    latest: GpsFix | None   # property
    parse_errors: int
    satellites_in_view: int
def build_ubx(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes
def ubx_backup_request(duration_ms: int = 0) -> bytes  # UBX-RXM-PMREQ
class SerialLike(Protocol):
    is_open: bool
    def readline(self) -> bytes: ...
    def write(self, data: bytes) -> int | None: ...
    def close(self) -> None: ...
class GpsReader:
    def __init__(self, config: GpsConfig, *, serial_factory=None, clock=time.monotonic, sleep=time.sleep)
    def start(self) -> None; def stop(self) -> None
    def step(self) -> None                       # one iteration of the reader loop
    def set_poll_interval(self, interval_s: float | None) -> None
    latest: GpsFix | None   # property, thread-safe
    def is_stale(self, now: float | None = None) -> bool
class SimulatedGps:
    def __init__(self, config: GpsConfig, *, clock=time.monotonic)
    def set_fix(self, latitude, longitude, *, hdop=0.9, satellites=12, speed_mps=None, course_deg=None) -> None
    def set_no_fix(self) -> None
    latest / is_stale / start / stop / set_poll_interval
```

The parser is talker-agnostic (`GP`, `GN`, `GL`, `GA`, `GB`, `GQ`): the M9N sends `$GN...`
when several constellations are used. Lines that do not start with `$` are ignored; a bad
checksum increments `parse_errors`. GGA updates position, quality, satellites used, HDOP and
altitude; quality 0 clears latitude and longitude. RMC status `V` sets quality 0 and clears the
position; status `A` updates position, speed (knots to m/s) and course, and raises quality 0 to
1 when a position is present (for receivers configured without GGA). GSA updates HDOP. GSV
updates `satellites_in_view`. `feed` returns the merged fix (timestamp = clock) after a GGA or
RMC, otherwise `None`. Malformed fields never raise; they count as parse errors.

`build_ubx` frames `0xB5 0x62 class id length(LE16) payload ck_a ck_b` with the 8-bit Fletcher
checksum over class through payload. `ubx_backup_request` is class 0x02 id 0x41 with the
16-byte payload: version 0, 3 reserved bytes, duration (U4 ms), flags 0x02 (backup),
wakeupSources 0x08 (UART RX).

`GpsReader` opens `serial.Serial(port, baud_rate, timeout=read_timeout_s)` lazily through the
default factory. Its thread repeatedly calls `step()`, which reads one line and feeds it.
Poll interval `0` = continuous; `> 0` = listen for `listen_window_s`, then close the port for
the interval; `None` = GPS off (port closed). With `use_ubx_power_save`, turning GPS off first
writes `ubx_backup_request(0)`, and turning it back on writes one `0xFF` byte to wake the
receiver. Serial errors close the port, wait 2 s and reopen on a later step (logged at most
every 30 s). `is_stale` is true when the last fix is older than `stale_after_s`.

### hardware/power_monitor.py: INA219 and state of charge

```python
class PowerSensor(Protocol):  # the subset of adafruit_ina219.INA219 used here
    bus_voltage: float    # V
    shunt_voltage: float  # V
    current: float        # mA
    power: float          # W
class StateOfChargeEstimator:
    def __init__(self, config: PowerConfig, state_path: Path | None, *, clock=time.monotonic)
    def update(self, current_ma: float, now: float | None = None) -> float  # returns SoC 0..1
    def reset_full(self) -> None
    def save(self) -> None
    state_of_charge: float; consumed_mah: float; usable_capacity_mah: float  # properties
    def remaining_minutes(self) -> float | None
class PowerMonitor:
    def __init__(self, config: NavSightConfig, sensor_factory, estimator, *, clock=time.monotonic)
    def start(self) -> None; def read(self) -> BatteryStatus; def stop(self) -> None
def make_ina219_factory(i2c, address) -> Callable[[], PowerSensor]
class SimulatedPowerSensor:  # implements PowerSensor
    def __init__(self, bus_voltage=5.1, current_ma=900.0)
    def set(self, *, bus_voltage=None, current_ma=None) -> None
```

The INA219 sits on the 5 V output of a USB power bank, so voltage says almost nothing about
charge: the bank holds 5.1 V until it cuts out. Charge is estimated by coulomb counting:
`usable_capacity_mah = capacity_mah * nominal_v / output_v * efficiency` (6290 mAh at 5 V by
default). `update` integrates positive current with the trapezoid rule; a gap longer than
`max_integration_gap_s` is ignored. SoC = `clamp(1 - consumed / usable, 0, 1)`. An exponential
moving average of current (time constant `current_ema_s`) gives `remaining_minutes`
(`None` below 1 mA). The consumed charge persists as JSON at `state_path` (with an ISO UTC time),
saved every `save_interval_s` and on `stop()`; a missing or corrupt file means "full" with a
logged hint to run `navsight --battery-full` after charging. `PowerMonitor.read` holds
`I2C_LOCK` around the sensor reads.

### hardware/buttons.py

```python
class ButtonDebouncer:  # pure logic, one per button
    def __init__(self, button: ButtonId, *, debounce_s=0.03, long_press_s=1.0, active_low=True)
    def update(self, level: int, now: float) -> ButtonEvent | None
    pressed: bool  # property: debounced state
class Buttons:
    def __init__(self, config, gpio, *, clock=time.monotonic)
    def start(self) -> None  # claim GPIO 17, 22, 27 as inputs with pull-ups
    def poll(self) -> list[ButtonEvent]  # sample every pin once; also queues the events
    def drain(self) -> list[ButtonEvent]  # thread-safe: events queued since the last drain
    def is_held(self, button: ButtonId) -> bool
    def stop(self) -> None
class SimulatedButtons:
    def inject(self, event: ButtonEvent) -> None
    poll() / drain() / is_held() / start() / stop()
```

A level change counts only after it has been stable for `debounce_s`. While pressed, a `LONG`
event fires once as soon as the press has lasted `long_press_s`, without waiting for release
(SOS must trigger while still held). Releasing before that emits `SHORT`; releasing after a
`LONG` emits nothing. The SOS button uses `sos_long_press_s` (2 s), the others `long_press_s`.

### hardware/haptics.py

```python
def intensity_to_duty(intensity: float, min_duty: float, max_duty: float) -> float
PATTERNS: dict[str, tuple[tuple[float, float, float], ...]]  # (left, right, seconds) steps
class HapticController:
    def __init__(self, config, gpio, *, clock=time.monotonic)
    def start(self) -> None
    def set_continuous(self, left: float, right: float) -> None  # 0..1
    def play(self, pattern: str) -> None  # unknown name -> ValueError
    def update(self, now: float | None = None) -> tuple[float, float]
    playing: str | None  # property
    def stop(self) -> None
```

ERM motors do not spin below roughly 30 % duty, so `intensity_to_duty` maps 0 to 0 and any
positive intensity to `min_duty + (max_duty - min_duty) * clamp(intensity, 0, 1)`. Patterns:
`drop_off` (3 strong pulses on both motors), `step_up` (2 medium pulses), `confirm` (short
double tap), `mode_change` (one short pulse), `low_battery` (2 slow pulses), `fall_detected`
(two long buzzes), `sos_countdown` (one 0.3 s buzz), `sos_sent` (3 quick pulses). A pattern
overrides the continuous levels until it ends; a new pattern replaces the current one.
`update` computes the active intensities and writes PWM (`gpio.pwm(pin, pwm_frequency_hz,
duty)`) only when a duty value changes. `stop()` turns both motors off. The motors need a
transistor driver and a flyback diode each; a GPIO pin cannot drive them directly.

### hardware/audio.py

```python
def bearing_to_pan(bearing_deg: float, full_pan_bearing_deg: float = 45.0) -> float  # -1..1
def pan_gains(pan: float) -> tuple[float, float]  # constant power: (cos t, sin t), t = (pan+1)*pi/4
def synth_tone(frequency_hz, duration_s, pan, volume, sample_rate) -> np.ndarray  # int16 (n, 2)
class AudioBackend(Protocol):
    def play_pcm(self, pcm: np.ndarray, sample_rate: int) -> None: ...  # blocks until played
    def close(self) -> None: ...
class SoundDeviceBackend: ...   # lazy import sounddevice
class AplayBackend: ...         # aplay -q -t raw -f S16_LE -c 2 -r RATE [-D DEVICE]
class NullBackend:               # records what was played (sim and tests)
    played: list[tuple[int, int]]  # (frames, sample_rate)
class TtsEngine(Protocol):
    def speak(self, text: str) -> None: ...  # blocks until finished or stopped
    def stop(self) -> None: ...
class SubprocessTts:  # espeak-ng, espeak or pico2wave via subprocess; interruptible
    def __init__(self, engine: str = "auto", rate_wpm: int = 165, device: str | None = None)
class RecordingTts:   # records spoken text (sim and tests)
    spoken: list[str]
def make_backend(config: AudioConfig) -> AudioBackend
def make_tts(config: AudioConfig) -> TtsEngine
class AudioOutput:
    def __init__(self, config: NavSightConfig, backend, tts, *, synchronous=False)
    def start(self) -> None; def stop(self) -> None
    def beep(self, frequency_hz: float, duration_s: float, bearing_deg: float = 0.0) -> None
    def say(self, text: str, interrupt: bool = False) -> None
    is_speaking: bool  # property
```

Tones have 5 ms raised-cosine fades so they do not click. `AudioOutput` uses two queues and two threads, so a beep is never stuck behind a long sentence:
beeps (max 4 pending, oldest dropped) and speech (max 2 pending, oldest dropped).
`say(..., interrupt=True)` clears the speech queue and stops the current utterance.
With `synchronous=True` (tests and simulator) calls run immediately in the caller's thread.
Backend and TTS errors are logged, never raised to the caller. `make_backend("auto")` picks
sounddevice if importable, else aplay if on PATH, else `NullBackend`; `make_tts("auto")` picks
the first of espeak-ng, espeak, pico2wave found by `shutil.which`, else `RecordingTts`.
True left/right audio needs two MAX98357A boards (one set to the left channel, one to the right)
or a stereo DAC; a single MAX98357A is mono.

### hardware/camera.py

```python
class FrameSource(Protocol):
    def read(self) -> np.ndarray | None: ...  # HxWx3 uint8, BGR order
    def close(self) -> None: ...
class PicameraSource: ...    # Picamera2 RGB888 video configuration (BGR byte order)
class OpenCvSource: ...      # cv2.VideoCapture(usb_index)
class SyntheticSource: ...   # blank frames for the simulator
def open_camera(config: NavSightConfig, simulate: bool = False) -> FrameSource
```

### perception/detector.py

```python
def pixel_to_bearing(x: float, image_width: int, horizontal_fov_deg: float) -> float
def results_to_detections(boxes_xyxy, class_ids, confidences, names, image_width, config)
    -> list[Detection]
class ObjectDetector:
    def __init__(self, config, model_factory: Callable[[str], Any] | None = None)
    def detect(self, frame: np.ndarray) -> list[Detection]
```

A wide-angle lens is not linear in angle: `pixel_to_bearing` uses the pinhole model,
`f = (W / 2) / tan(hfov / 2)` and `bearing = atan((x - W / 2) / f)`. Detections below
`detector.confidence` or outside `detector.classes` are dropped; the rest are sorted by
confidence. `ObjectDetector` lazily loads `ultralytics.YOLO(model_path)` and calls
`predict(frame, imgsz=image_size, conf=confidence, verbose=False)`, converting tensors with
`.cpu().numpy()` when needed.

### perception/depth.py

```python
def tof_anchor_half_width_px(image_width: int, horizontal_fov_deg: float, tof_fov_deg: float) -> float
def metric_distance_for_box(inv_depth, bbox_xyxy, anchor_distance_m, config) -> float | None
def attach_distances(detections, inv_depth, anchor_distance_m, config) -> list[Detection]
class DepthEstimator:
    def __init__(self, config, model_factory: Callable[[], tuple[Any, Any]] | None = None)
    def infer(self, frame: np.ndarray) -> np.ndarray | None  # relative inverse depth (H, W) float32
```

MiDaS predicts relative inverse depth with an unknown scale. The forward ToF measures a real
distance in the middle of the image, so it anchors the scale: the anchor window is the centred
square of half-width `f * tan(tof_fov / 2)` pixels (`f` as in `pixel_to_bearing`). For a box,
take the median inverse depth over its central 50 % and over the anchor window; the distance is
`anchor_distance_m * inv_anchor / inv_box`, clamped to `[min_distance_m, max_distance_m]`.
Return `None` when there is no anchor or either median is not positive. `DepthEstimator` lazily
loads MiDaS through `torch.hub` (`intel-isl/MiDaS`, `model_type`, its `small_transform`),
converts BGR to RGB and resizes the prediction to the frame size.

### fusion/ekf.py: pose

```python
class LocalTangentPlane:
    def __init__(self, origin_latitude: float, origin_longitude: float)
    def to_enu(self, latitude, longitude) -> tuple[float, float]       # (east_m, north_m)
    def to_geodetic(self, east_m, north_m) -> tuple[float, float]
class PoseEKF:
    def __init__(self, config: PoseConfig)
    def initialize(self, east_m, north_m, heading_deg, position_std_m) -> None
    def predict_step(self, step_length_m: float, heading_deg: float) -> None
    def update_heading(self, heading_deg: float, std_deg: float) -> None
    def update_gps(self, east_m, north_m, std_m) -> bool  # False = rejected by the gate
    initialized: bool; state: np.ndarray; covariance: np.ndarray  # properties
    heading_deg: float; position_std_m: float                     # properties
class PoseEstimator:
    def __init__(self, config: NavSightConfig)
    def on_imu(self, sample: ImuSample) -> None
    def on_gps(self, fix: GpsFix) -> bool
    def pose(self, now: float) -> Pose | None
    def using_dead_reckoning(self, now: float) -> bool
```

State `x = [east, north, psi]` with psi the compass heading in radians (clockwise from north),
so a step moves `east += L sin(psi)`, `north += L cos(psi)`. Jacobian
`F = [[1, 0, L cos psi], [0, 1, -L sin psi], [0, 0, 1]]`; process noise
`Q = J diag(sigma_L^2, sigma_psi^2) J^T` with `J = [[sin psi, L cos psi], [cos psi, -L sin psi],
[0, 1]]`, `sigma_L = step_length_std_frac * L` and `sigma_psi = heading_std_per_step_deg`.
`predict_step` first sets psi to the given IMU heading (the IMU's fused heading is the best
heading source between corrections). Heading updates wrap the innovation to (-180, 180].
GPS updates use `H = [[1, 0, 0], [0, 1, 0]]` and a Mahalanobis gate (`d^2 > gps_gate_chi2`
rejects). The covariance update uses the Joseph form and stays symmetric.

`LocalTangentPlane` uses an equirectangular projection about the origin
(WGS84 radius 6378137 m), accurate to centimetres over a few kilometres of walking.

`PoseEstimator` starts dead reckoning from the first IMU sample at (0, 0) with
`initial_position_std_m`, so it works indoors from power-on. Each new step since the last sample
(at most `max_steps_per_update` per call; a decreasing count resets the baseline) predicts one
step, then the IMU heading updates psi. A GPS fix is used when `has_fix` and
`hdop <= max_hdop` (`sigma = max(hdop * uere_m, gps_min_std_m)`). The first usable fix sets the
tangent-plane origin and re-initialises the position there (heading kept). Three gate rejections
in a row reset the position to the fix, so a long dead-reckoning drift can never lock GPS out.
`pose()` reports latitude/longitude once an origin exists and `source = "gps"` if a fix was
accepted within `gps_timeout_s`, else `"dead_reckoning"`.

### fusion/obstacle_map.py

```python
class RangeTrack:  # 1-D constant-velocity Kalman filter on range
    def __init__(self, distance_m: float, std_m: float, now: float, accel_std_mps2: float)
    def predict(self, now: float) -> None
    def update(self, distance_m: float, std_m: float, now: float) -> None
    distance_m: float; closing_speed_mps: float; last_update: float  # properties
    def time_to_collision(self, min_closing_mps: float) -> float | None
class ObstacleMap:
    def __init__(self, config: NavSightConfig)
    def sector_for_bearing(self, bearing_deg: float) -> int | None
    def sector_center(self, sector: int) -> float
    def update_tof(self, reading: ToFReading) -> None
    def update_detections(self, detections: list[Detection], now: float) -> None
    def obstacles(self, now: float) -> list[Obstacle]
    def nearest(self, now: float) -> Obstacle | None
```

Sectors split `sector_span_deg` evenly, centred on 0 (default 7 sectors of 15°). A bearing
maps to `floor((b + span / 2) / width)`; bearings outside `[-span / 2, span / 2]` give `None`
and `+span / 2` belongs to the last sector. The forward ToF updates the centre sector
(`sector_count // 2`) with `std = max(tof_range_std_frac * d, tof_range_std_min_m)`; a reading
of `None` (nothing in range) removes the centre track if its last update came from the ToF.
A detection with `distance_m` updates the track of its sector with
`std = vision_range_std_frac * d`; every detection records its label for the sector
(highest confidence wins). Range tracks use `F = [[1, dt], [0, 1]]`,
`Q = q [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]` with `q = range_accel_std_mps2^2`, `H = [1, 0]`.
A new track starts at the measurement with zero velocity and velocity variance 1 m^2/s^2.
`closing_speed = -range_rate`; time to collision = `distance / closing_speed` when closing
speed >= `ttc_min_closing_mps`, else `None`.

`obstacles(now)` predicts live tracks to `now` without changing stored state, drops tracks not
updated within `track_timeout_s`, and attaches a sector's label if seen within
`label_timeout_s`. A sector with a recent label but no live track yields a label-only obstacle
(`distance_m=None`). Source is `"tof"`, `"vision"` or `"fused"` (both within the timeout).
Confidence is 1.0 for ToF-only tracks, else the label confidence. The list is sorted by distance
with `None` last; `nearest` returns the first obstacle that has a distance.

### fusion/ground.py

```python
class GroundHazardDetector:
    def __init__(self, config: NavSightConfig)
    def update(self, reading: ToFReading, tilt_from_vertical_deg: float | None = None,
               pitch_rate_dps: float | None = None) -> GroundHazard | None
    baseline_m: float | None; current: HazardType  # properties
```

The down ToF measures a slant distance to the ground. The baseline is the median of valid
readings from the last `baseline_window_s`, collected only while no hazard is active and the
pitch rate (if given) is within `max_pitch_rate_dps`; detection starts once
`min_baseline_samples` are collected. The vertical-equivalent change is
`delta = (d - baseline) * cos(tilt)` (`tilt` = the down sensor's angle from vertical, `None`
means `cos = 1`). `delta > drop_threshold_m` is a drop-off; `-delta > step_up_threshold_m`
is a step-up. A `None` distance (no echo at all) while the baseline is below 90 % of
`max_range_m` counts as a drop-off with `delta = max_range_m - baseline`. Readings taken while
the pitch rate exceeds the limit are ignored entirely. A hazard is confirmed after
`confirm_samples` consecutive samples of the same type; `update` returns a `GroundHazard` only on
that transition (edge-triggered) and `current` holds the level until readings stay inside the
thresholds for `clear_after_s`.

### fusion/fall.py

```python
class FallDetector:
    def __init__(self, config: NavSightConfig)
    def update(self, sample: ImuSample) -> bool  # True exactly once per confirmed fall
    def reset(self) -> None
    state: str  # property: "idle", "freefall", "impact" or "confirming"
```

Using the total acceleration magnitude in g: free fall is `< freefall_g` for at least
`freefall_min_s`; an impact is `> impact_g` within `impact_window_s` after free fall, or
`> hard_impact_g` at any time (some falls have no clean free fall). After an impact the detector
waits up to `stillness_s`, then confirms the fall if the cane has been still for that time
(`| |a| - 1 g | <= stillness_tolerance_g` and every gyro axis within `max_still_gyro_dps`) and is
lying roughly flat (`|pitch| < lying_pitch_deg`). Movement before confirmation returns the
detector to idle. The cane's IMU cannot tell a fall from a dropped cane, which is why a fall
starts a cancellable SOS countdown rather than sending immediately.

### safety/sos.py

```python
class SosState(Enum): IDLE, COUNTDOWN, SENDING, SENT, FAILED
@dataclass(frozen=True)
class SosMessage:
    reason: str  # "fall" or "manual"
    latitude: float | None
    longitude: float | None
    timestamp_utc: str  # ISO 8601
    battery_percent: int | None
    @property
    def maps_url(self) -> str | None: ...
def google_maps_url(latitude: float, longitude: float) -> str  # https://maps.google.com/?q=LAT,LON
def format_sos_text(message: SosMessage) -> str
class Notifier(Protocol):
    name: str
    remote: bool  # False for the local log
    def send(self, message: SosMessage) -> None: ...  # raises on failure
class LogNotifier: ...       # appends a JSON line to sos.log_file
class WebhookNotifier: ...   # POST JSON with urllib.request, timeout from config
class TelegramNotifier: ...  # Bot API sendMessage with urllib.request
def make_notifiers(config: SosConfig) -> list[Notifier]
class SosController:
    def __init__(self, config, notifiers, *, clock=time.monotonic, runner=None, utc_now=None)
    def trigger_fall(self, now: float) -> None
    def trigger_manual(self, now: float) -> None
    def cancel(self, now: float) -> bool
    def tick(self, now: float, location: tuple[float, float] | None,
             battery_soc: float | None) -> list[Alert]
    state: SosState; seconds_remaining: float | None  # properties
```

A fall starts `COUNTDOWN` for `fall_countdown_s`; `tick` announces "Fall detected. Sending SOS
in 15 seconds. Press any button to cancel." at the start and the remaining seconds every
`countdown_announce_every_s` (alerts at DANGER level with the `sos_countdown` pattern).
`cancel` works only during the countdown and returns `True` ("SOS cancelled."). When the
countdown ends, or immediately on `trigger_manual`, the state becomes `SENDING`: the message is
built with the latest location and battery, then sent to every notifier through `runner`
(default: a daemon thread; tests pass a synchronous runner). Remote notifiers that fail are
retried after each delay in `retry_delays_s`. Outcomes are announced: "SOS sent." when every
remote notifier succeeded, "SOS recorded. No emergency contact is set up." when there are no
remote notifiers, "SOS failed. Retrying." between retries and `FAILED` after the last one.
A manual SOS while one is already sending is ignored. Notifier secrets never appear in logs.

### alerts/priority.py

```python
class AlertManager:
    def __init__(self, config: NavSightConfig, *, clock=time.monotonic)
    verbosity: Verbosity  # property, settable; initial value from config.alerts.verbosity
    def update(self, now: float, obstacles: list[Obstacle], hazard: GroundHazard | None,
               events: list[Alert]) -> AlertOutputs
def direction_phrase(bearing_deg: float) -> str
def distance_phrase(distance_m: float) -> str
def distance_to_beep(distance_m: float, config: AlertConfig) -> tuple[float, float] | None
```

`distance_to_beep` returns `(frequency_hz, repeat_interval_s)` by zone: below `danger_m`
(2000 Hz, 0.08 s), below `warning_m` (1500 Hz, 0.2 s), below `caution_m` (1000 Hz, 0.5 s),
else `None`. Each beep lasts 0.06 s.

The V2 priority order is haptic > spatial audio > speech:

- **Haptic** (always on, every verbosity): the nearest obstacle with a distance inside
  `caution_m` sets the continuous level
  `clamp((caution_m - d) / (caution_m - danger_m), 0, 1)`, raised to at least 0.8 when its time
  to collision is below `ttc_danger_s`. Bearing below `-side_bearing_deg` drives the left motor,
  above `+side_bearing_deg` the right, otherwise both. A new ground hazard plays `drop_off` or
  `step_up`; an event's `haptic_pattern` plays too (hazard first).
- **Spatial audio** (not in HAPTIC_ONLY): the nearest obstacle inside `warning_m` beeps with
  `distance_to_beep`, panned by its bearing, at most once per repeat interval.
- **Speech**: a new ground hazard says "Drop ahead." or "Step up ahead." with
  `interrupt=True`. A labelled obstacle within `label_announce_m` says
  "{label} {direction}, {distance}." (for example "person slightly left, 2 meters.";
  label-only obstacles omit the distance). Keys are `label:direction`; a key repeats only after
  `tts_cooldown_s`, unless its distance has shrunk by more than 30 % since it was last spoken.
  Speech items are at least `min_speech_gap_s` apart unless they interrupt. While any obstacle is
  inside `danger_m`, obstacle descriptions are held back so speech never talks over the danger
  cue. Events speak their message (DANGER events interrupt); events share the per-key cooldown,
  except DANGER events, which always pass.
- **Verbosity**: FULL = everything; BRIEF = speech only for hazards, events and obstacles
  inside `warning_m`; HAPTIC_ONLY = no beeps, speech only for hazards and events at WARNING or
  above.
- **Hysteresis**: an obstacle that entered the warning zone stays in it until it is
  `hysteresis_m` beyond `warning_m`, so a reading hovering at the boundary does not flicker.

`direction_phrase`: |bearing| < 10° "ahead"; 10-35° "slightly left/right"; beyond that
"on your left/right". `distance_phrase`: below 1 m "very close", else "N meters" rounded
("1 meter" singular).

### power/state_machine.py

```python
@dataclass(frozen=True)
class PowerInputs:
    now: float
    state_of_charge: float | None
    moving: bool
    button_pressed: bool
    sos_active: bool
class PowerStateMachine:
    def __init__(self, config: NavSightConfig, *, clock=time.monotonic)
    state: PowerState  # property
    last_transition: tuple[PowerState, PowerState] | None  # property; set by the latest update
    def update(self, inputs: PowerInputs) -> PowerProfile
    def profile_for(self, state: PowerState) -> PowerProfile
```

Motion or a button press refreshes "last activity". ACTIVE goes to LOW_POWER when SoC drops
below `low_soc` or after `idle_to_low_power_s` without activity. LOW_POWER returns to ACTIVE on
activity if SoC is unknown or at least `low_soc + soc_hysteresis`. LOW_POWER goes to SLEEP after
`idle_to_sleep_s` without activity, never during an SOS. SLEEP wakes on activity or an SOS:
to ACTIVE if SoC allows, else LOW_POWER. Critical battery never forces sleep while the user is
walking; obstacle sensing stays on. Profiles: ACTIVE = camera `camera.fps`, depth
`depth.enabled`, ToF `tof.rate_hz`, GPS continuous (0), fusion `fusion.rate_hz`; LOW_POWER =
`low_power_camera_fps`, no depth, `low_power_tof_hz`, `low_power_gps_interval_s`,
`low_power_fusion_hz`; SLEEP = camera 0, no depth, ToF 0, GPS off (`None`),
`sleep_fusion_hz`, with the IMU still running to wake the cane.

## Buttons

| Button | Short press | Long press |
|---|---|---|
| Mode (GPIO 17) | cycle alerts: full, brief, vibration only | camera on/off |
| Action (GPIO 22) | describe what is ahead and the heading | battery, GPS and mode report |
| SOS (GPIO 27) | says "Hold for two seconds to send SOS" | send SOS (2 s hold) |

Any button cancels a fall countdown.

## Testing and simulation

`pytest` runs on any PC: every hardware class is tested through fakes, and the core loop is
tested by driving the simulator with a `ManualClock`. `navsight --sim` runs the whole system on a
laptop with a scripted walk (approaching obstacles, a person on the left, a curb, a GPS dropout
indoors, a draining battery and a fall), printing the haptic levels, beeps and speech it would
produce.
