"""NavSight v2 configuration.

Defaults follow the NavSight V2 architecture document: GPIO pins, I2C addresses and sensor
rates. Override any value with a JSON file (``navsight --config my.json``) that mirrors this
structure, for example ``{"alerts": {"verbosity": "brief"}, "sos": {"webhook_url": "https://..."}}``.
"""

from __future__ import annotations

import json
import typing
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

VERBOSITY_CHOICES = ("full", "brief", "haptic_only")
AXIS_CHOICES = ("x", "y", "z", "-x", "-y", "-z")
AUDIO_BACKEND_CHOICES = ("auto", "sounddevice", "aplay", "none")
TTS_ENGINE_CHOICES = ("auto", "espeak-ng", "espeak", "pico2wave", "none")
TOF_TIMING_BUDGETS_MS = (15, 20, 33, 50, 100, 200, 500)
# GPIOs that the V2 design gives to buses: I2C (2, 3), GPS UART (14, 15), I2S audio (18, 19, 21).
RESERVED_GPIO = frozenset({2, 3, 14, 15, 18, 19, 21})


@dataclass
class PinConfig:
    """BCM GPIO numbers on the Raspberry Pi 5 header."""

    i2c_bus: int = 1  # SDA = GPIO 2, SCL = GPIO 3
    tof_down_xshut: int = 25
    haptic_left: int = 12  # PWM0
    haptic_right: int = 13  # PWM1
    button_mode: int = 17
    button_action: int = 22
    button_sos: int = 27
    gpio_chip: int | None = None  # None = auto-detect the RP1 header chip


@dataclass
class I2CAddressConfig:
    tof_forward: int = 0x29
    tof_down: int = 0x30
    tof_temporary: int = 0x31  # parking address used while re-addressing the ToF pair
    imu: int = 0x4A
    power_monitor: int = 0x40


@dataclass
class TofConfig:
    rate_hz: float = 50.0
    timing_budget_ms: int = 20
    distance_mode: int = 2  # 1 = short (up to 1.3 m), 2 = long (up to 4 m)
    max_range_m: float = 4.0
    forward_fov_deg: float = 27.0


@dataclass
class ImuConfig:
    rate_hz: float = 100.0
    forward_axis: str = "y"  # sensor axis pointing along the cane towards its tip
    heading_offset_deg: float = 0.0  # magnetic declination plus any mounting offset


@dataclass
class GpsConfig:
    port: str = "/dev/serial0"  # UART: GPIO 14 TX, GPIO 15 RX
    baud_rate: int = 38400  # NEO-M9N factory default (the v1 NEO-8M used 9600)
    read_timeout_s: float = 1.0
    stale_after_s: float = 5.0
    max_hdop: float = 5.0
    uere_m: float = 1.5  # user equivalent range error: GPS sigma = HDOP * UERE
    listen_window_s: float = 3.0  # time spent listening per duty cycle in low-power mode
    use_ubx_power_save: bool = False  # put the receiver in backup mode while GPS is off


@dataclass
class CameraConfig:
    source: str = "csi"  # "csi" (Picamera2) or "usb" (OpenCV)
    usb_index: int = 0
    # 16:9 keeps the Camera Module 3 Wide's full 102 deg horizontal view (its 120 deg figure is
    # diagonal). A 4:3 size such as 640x480 is cropped from the 16:9 sensor to about 86 deg, so
    # set horizontal_fov_deg to match if you change the aspect ratio.
    width: int = 640
    height: int = 360
    fps: float = 15.0
    horizontal_fov_deg: float = 102.0


@dataclass
class DetectorConfig:
    enabled: bool = True
    model_path: str = "yolov8n.pt"  # an NCNN export (yolov8n_ncnn_model) runs ~2x faster on a Pi 5
    image_size: int = 416
    confidence: float = 0.45
    classes: tuple[str, ...] = (
        "person",
        "bicycle",
        "car",
        "motorcycle",
        "bus",
        "truck",
        "dog",
        "cat",
        "chair",
        "bench",
        "potted plant",
        "fire hydrant",
        "stop sign",
        "traffic light",
        "parking meter",
        "suitcase",
        "dining table",
    )


@dataclass
class DepthConfig:
    enabled: bool = True
    model_type: str = "MiDaS_small"
    every_n_frames: int = 3
    min_distance_m: float = 0.2
    max_distance_m: float = 10.0


@dataclass
class PoseConfig:
    step_length_m: float = 0.65
    step_length_std_frac: float = 0.10
    heading_std_per_step_deg: float = 2.0
    imu_heading_std_deg: float = 8.0
    gps_min_std_m: float = 1.0
    gps_gate_chi2: float = 9.21  # chi-square, 2 degrees of freedom, 99 %
    gps_timeout_s: float = 10.0  # no accepted fix for this long = dead reckoning
    initial_position_std_m: float = 5.0
    max_steps_per_update: int = 10


@dataclass
class FusionConfig:
    rate_hz: float = 30.0
    sector_count: int = 7
    sector_span_deg: float = 105.0  # covers the camera's 102 deg horizontal field of view
    tof_range_std_frac: float = 0.03  # VL53L1X: +/-3 %
    tof_range_std_min_m: float = 0.01
    vision_range_std_frac: float = 0.25
    range_accel_std_mps2: float = 2.0  # process noise of the constant-velocity range tracks
    track_timeout_s: float = 0.6
    label_timeout_s: float = 1.5
    ttc_min_closing_mps: float = 0.2


@dataclass
class HazardConfig:
    drop_threshold_m: float = 0.12
    step_up_threshold_m: float = 0.10
    baseline_window_s: float = 1.5
    min_baseline_samples: int = 10
    confirm_samples: int = 3
    clear_after_s: float = 1.0
    max_pitch_rate_dps: float = 60.0  # ignore readings while the cane is being swung up or down
    # Angle between the down ToF axis and the cane axis; positive = further from vertical.
    down_sensor_offset_deg: float = 0.0
    # Beyond this angle from vertical the down sensor cannot see the ground ahead (cane near flat).
    max_tilt_deg: float = 75.0


@dataclass
class FallConfig:
    freefall_g: float = 0.5
    freefall_min_s: float = 0.06
    impact_g: float = 2.5
    hard_impact_g: float = 3.5
    impact_window_s: float = 1.0
    stillness_s: float = 2.0
    stillness_tolerance_g: float = 0.15
    max_still_gyro_dps: float = 20.0
    lying_pitch_deg: float = 30.0  # |pitch| below this = cane lying roughly flat


@dataclass
class AlertConfig:
    danger_m: float = 0.5
    warning_m: float = 1.0
    caution_m: float = 2.0
    label_announce_m: float = 5.0
    hysteresis_m: float = 0.15
    side_bearing_deg: float = 10.0
    ttc_danger_s: float = 1.5
    tts_cooldown_s: float = 6.0
    min_speech_gap_s: float = 1.5
    verbosity: str = "full"  # "full", "brief" or "haptic_only"


@dataclass
class HapticConfig:
    pwm_frequency_hz: float = 200.0
    min_duty: float = 0.30  # ERM motors stall below roughly 30 % duty
    max_duty: float = 1.0
    update_hz: float = 100.0


@dataclass
class AudioConfig:
    backend: str = "auto"  # "auto", "sounddevice", "aplay" or "none"
    device: str | None = None
    sample_rate: int = 48000
    volume: float = 0.8
    tts_engine: str = "auto"  # "auto", "espeak-ng", "espeak", "pico2wave" or "none"
    tts_rate_wpm: int = 165
    full_pan_bearing_deg: float = 45.0


@dataclass
class ButtonConfig:
    poll_hz: float = 200.0
    debounce_s: float = 0.03
    long_press_s: float = 1.0
    sos_long_press_s: float = 2.0


@dataclass
class PowerConfig:
    battery_capacity_mah: float = 10000.0
    battery_nominal_v: float = 3.7
    output_v: float = 5.0
    conversion_efficiency: float = 0.85
    shunt_ohms: float = 0.1
    low_soc: float = 0.20
    critical_soc: float = 0.08
    soc_hysteresis: float = 0.03
    idle_to_low_power_s: float = 120.0
    idle_to_sleep_s: float = 600.0
    low_voltage_v: float = 4.75
    current_ema_s: float = 60.0
    max_integration_gap_s: float = 300.0
    save_interval_s: float = 60.0
    state_file: str = "~/.navsight/battery.json"
    read_hz: float = 1.0
    low_power_camera_fps: float = 5.0
    low_power_tof_hz: float = 25.0
    low_power_gps_interval_s: float = 10.0
    low_power_fusion_hz: float = 15.0
    sleep_fusion_hz: float = 2.0


@dataclass
class SosConfig:
    fall_countdown_s: float = 15.0
    countdown_announce_every_s: float = 5.0
    webhook_url: str | None = None
    telegram_bot_token: str | None = None
    telegram_chat_id: str | None = None
    log_file: str = "~/.navsight/sos.log"
    request_timeout_s: float = 10.0
    retry_delays_s: tuple[float, ...] = (5.0, 15.0, 30.0, 60.0)


@dataclass
class NavSightConfig:
    pins: PinConfig = field(default_factory=PinConfig)
    i2c: I2CAddressConfig = field(default_factory=I2CAddressConfig)
    tof: TofConfig = field(default_factory=TofConfig)
    imu: ImuConfig = field(default_factory=ImuConfig)
    gps: GpsConfig = field(default_factory=GpsConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    depth: DepthConfig = field(default_factory=DepthConfig)
    pose: PoseConfig = field(default_factory=PoseConfig)
    fusion: FusionConfig = field(default_factory=FusionConfig)
    hazard: HazardConfig = field(default_factory=HazardConfig)
    fall: FallConfig = field(default_factory=FallConfig)
    alerts: AlertConfig = field(default_factory=AlertConfig)
    haptics: HapticConfig = field(default_factory=HapticConfig)
    audio: AudioConfig = field(default_factory=AudioConfig)
    buttons: ButtonConfig = field(default_factory=ButtonConfig)
    power: PowerConfig = field(default_factory=PowerConfig)
    sos: SosConfig = field(default_factory=SosConfig)
    simulate: bool = False
    log_level: str = "INFO"


def load_config(
    path: Path | str | None = None, overrides: dict[str, Any] | None = None
) -> NavSightConfig:
    """Build a config from the defaults, then the JSON file at ``path``, then ``overrides``.

    Unknown keys and wrongly typed values raise ``ValueError`` so a typo never goes unnoticed.
    """
    config = NavSightConfig()
    if path is not None:
        file_path = Path(path).expanduser()
        data = json.loads(file_path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError(f"{file_path}: the top level must be a JSON object")
        _merge(config, data, "")
    if overrides:
        _merge(config, overrides, "")
    validate_config(config)
    return config


def config_to_dict(config: NavSightConfig) -> dict[str, Any]:
    return asdict(config)


def dump_config(config: NavSightConfig) -> str:
    return json.dumps(config_to_dict(config), indent=2)


def expand_path(path: str) -> Path:
    return Path(path).expanduser()


def validate_config(config: NavSightConfig) -> None:
    """Raise ``ValueError`` listing every invalid setting, so a bad file fails at start-up."""
    errors: list[str] = []
    for check in (
        _check_buses,
        _check_sensors,
        _check_estimation,
        _check_outputs,
        _check_power,
        _check_sos,
    ):
        check(config, errors)
    if errors:
        raise ValueError("Invalid NavSight config:\n  " + "\n  ".join(errors))


def _positive(errors: list[str], section: Any, prefix: str, *names: str) -> None:
    for name in names:
        if not getattr(section, name) > 0:
            errors.append(f"{prefix}.{name} must be greater than 0")


def _non_negative(errors: list[str], section: Any, prefix: str, *names: str) -> None:
    for name in names:
        if not getattr(section, name) >= 0:
            errors.append(f"{prefix}.{name} must be 0 or more")


def _check_buses(config: NavSightConfig, errors: list[str]) -> None:
    addresses = config.i2c
    address_values = [
        addresses.tof_forward,
        addresses.tof_down,
        addresses.tof_temporary,
        addresses.imu,
        addresses.power_monitor,
    ]
    if len(set(address_values)) != len(address_values):
        errors.append("i2c: every address must be different")
    if any(not 0x08 <= value <= 0x77 for value in address_values):
        errors.append("i2c: addresses must be 7-bit values between 0x08 and 0x77")

    pins = config.pins
    gpio_values = [
        pins.tof_down_xshut,
        pins.haptic_left,
        pins.haptic_right,
        pins.button_mode,
        pins.button_action,
        pins.button_sos,
    ]
    if len(set(gpio_values)) != len(gpio_values):
        errors.append("pins: every GPIO number must be different")
    clashes = sorted(RESERVED_GPIO.intersection(gpio_values))
    if clashes:
        errors.append(f"pins: GPIO {clashes} are used by I2C, the GPS UART or I2S audio")


def _check_sensors(config: NavSightConfig, errors: list[str]) -> None:
    tof = config.tof
    if tof.distance_mode not in (1, 2):
        errors.append("tof.distance_mode must be 1 (short) or 2 (long)")
    if tof.timing_budget_ms not in TOF_TIMING_BUDGETS_MS:
        errors.append(f"tof.timing_budget_ms must be one of {TOF_TIMING_BUDGETS_MS}")
    elif tof.distance_mode == 2 and tof.timing_budget_ms == 15:
        errors.append("tof: a 15 ms timing budget needs distance_mode 1")
    _positive(errors, tof, "tof", "rate_hz", "max_range_m", "forward_fov_deg")

    if config.imu.forward_axis not in AXIS_CHOICES:
        errors.append(f"imu.forward_axis must be one of {AXIS_CHOICES}")
    _positive(errors, config.imu, "imu", "rate_hz")

    _positive(
        errors,
        config.gps,
        "gps",
        "baud_rate",
        "read_timeout_s",
        "stale_after_s",
        "max_hdop",
        "uere_m",
        "listen_window_s",
    )

    camera = config.camera
    if not 0 < camera.horizontal_fov_deg < 180:
        errors.append("camera.horizontal_fov_deg must be between 0 and 180")
    if camera.source not in ("csi", "usb"):
        errors.append('camera.source must be "csi" or "usb"')
    _positive(errors, camera, "camera", "width", "height", "fps")

    if not 0 < config.detector.confidence <= 1:
        errors.append("detector.confidence must be in (0, 1]")
    _positive(errors, config.detector, "detector", "image_size")
    depth = config.depth
    if depth.every_n_frames < 1:
        errors.append("depth.every_n_frames must be at least 1")
    if not 0 < depth.min_distance_m < depth.max_distance_m:
        errors.append("depth: need 0 < min_distance_m < max_distance_m")


def _check_estimation(config: NavSightConfig, errors: list[str]) -> None:
    pose = config.pose
    _positive(
        errors,
        pose,
        "pose",
        "step_length_m",
        "imu_heading_std_deg",
        "gps_min_std_m",
        "gps_gate_chi2",
        "initial_position_std_m",
    )
    _non_negative(
        errors, pose, "pose", "step_length_std_frac", "heading_std_per_step_deg", "gps_timeout_s"
    )
    if pose.max_steps_per_update < 1:
        errors.append("pose.max_steps_per_update must be at least 1")

    fusion = config.fusion
    if fusion.sector_count < 3 or fusion.sector_count % 2 == 0:
        errors.append("fusion.sector_count must be odd and at least 3")
    _positive(
        errors,
        fusion,
        "fusion",
        "rate_hz",
        "sector_span_deg",
        "tof_range_std_min_m",
        "vision_range_std_frac",
        "range_accel_std_mps2",
        "track_timeout_s",
    )
    _non_negative(errors, fusion, "fusion", "tof_range_std_frac", "ttc_min_closing_mps")
    if fusion.label_timeout_s < fusion.track_timeout_s:
        errors.append("fusion.label_timeout_s must be at least track_timeout_s")

    hazard = config.hazard
    _positive(
        errors,
        hazard,
        "hazard",
        "drop_threshold_m",
        "step_up_threshold_m",
        "baseline_window_s",
        "clear_after_s",
        "max_pitch_rate_dps",
    )
    if not 0 < hazard.max_tilt_deg < 90:
        errors.append("hazard.max_tilt_deg must be between 0 and 90")
    if hazard.min_baseline_samples < 1 or hazard.confirm_samples < 1:
        errors.append("hazard.min_baseline_samples and confirm_samples must be at least 1")
    elif hazard.baseline_window_s > 0:
        slowest_tof_hz = min(config.tof.rate_hz, config.power.low_power_tof_hz)
        if hazard.min_baseline_samples / hazard.baseline_window_s > slowest_tof_hz:
            errors.append(
                "hazard: min_baseline_samples cannot fit in baseline_window_s at the slowest "
                f"ToF rate ({slowest_tof_hz} Hz)"
            )

    fall = config.fall
    _positive(
        errors,
        fall,
        "fall",
        "freefall_min_s",
        "impact_window_s",
        "stillness_s",
        "stillness_tolerance_g",
        "max_still_gyro_dps",
        "lying_pitch_deg",
    )
    if not 0 < fall.freefall_g < 1 < fall.impact_g <= fall.hard_impact_g:
        errors.append("fall: need 0 < freefall_g < 1 < impact_g <= hard_impact_g")


def _check_outputs(config: NavSightConfig, errors: list[str]) -> None:
    alerts = config.alerts
    if not 0 < alerts.danger_m < alerts.warning_m < alerts.caution_m:
        errors.append("alerts: need 0 < danger_m < warning_m < caution_m")
    if alerts.verbosity not in VERBOSITY_CHOICES:
        errors.append(f"alerts.verbosity must be one of {VERBOSITY_CHOICES}")
    _positive(errors, alerts, "alerts", "label_announce_m", "side_bearing_deg", "ttc_danger_s")
    _non_negative(errors, alerts, "alerts", "hysteresis_m", "tts_cooldown_s", "min_speech_gap_s")

    haptics = config.haptics
    if not 0 <= haptics.min_duty <= haptics.max_duty <= 1:
        errors.append("haptics: need 0 <= min_duty <= max_duty <= 1")
    _positive(errors, haptics, "haptics", "pwm_frequency_hz", "update_hz")

    audio = config.audio
    if audio.backend not in AUDIO_BACKEND_CHOICES:
        errors.append(f"audio.backend must be one of {AUDIO_BACKEND_CHOICES}")
    if audio.tts_engine not in TTS_ENGINE_CHOICES:
        errors.append(f"audio.tts_engine must be one of {TTS_ENGINE_CHOICES}")
    if not 0 <= audio.volume <= 1:
        errors.append("audio.volume must be between 0 and 1")
    _positive(errors, audio, "audio", "sample_rate", "tts_rate_wpm", "full_pan_bearing_deg")

    buttons = config.buttons
    _positive(errors, buttons, "buttons", "poll_hz", "long_press_s", "sos_long_press_s")
    _non_negative(errors, buttons, "buttons", "debounce_s")
    if buttons.debounce_s >= min(buttons.long_press_s, buttons.sos_long_press_s):
        errors.append("buttons.debounce_s must be shorter than the long-press times")


def _check_power(config: NavSightConfig, errors: list[str]) -> None:
    power = config.power
    if not 0 <= power.critical_soc < power.low_soc < 1:
        errors.append("power: need 0 <= critical_soc < low_soc < 1")
    if power.low_soc + power.soc_hysteresis > 1:
        errors.append("power: low_soc + soc_hysteresis must not exceed 1")
    if power.idle_to_sleep_s <= power.idle_to_low_power_s:
        errors.append("power: idle_to_sleep_s must be longer than idle_to_low_power_s")
    if not 0 < power.conversion_efficiency <= 1:
        errors.append("power.conversion_efficiency must be in (0, 1]")
    _positive(
        errors,
        power,
        "power",
        "battery_capacity_mah",
        "battery_nominal_v",
        "output_v",
        "max_integration_gap_s",
        "save_interval_s",
        "current_ema_s",
        "read_hz",
        "low_power_camera_fps",
        "low_power_tof_hz",
        "low_power_fusion_hz",
        "sleep_fusion_hz",
    )
    _non_negative(errors, power, "power", "soc_hysteresis", "low_power_gps_interval_s")


def _check_sos(config: NavSightConfig, errors: list[str]) -> None:
    sos = config.sos
    _non_negative(errors, sos, "sos", "fall_countdown_s", "countdown_announce_every_s")
    _positive(errors, sos, "sos", "request_timeout_s")
    if any(delay < 0 for delay in sos.retry_delays_s):
        errors.append("sos.retry_delays_s must all be 0 or more")
    if sos.webhook_url is not None:
        url = urlparse(sos.webhook_url)
        if url.scheme not in ("http", "https") or not url.netloc:
            errors.append("sos.webhook_url must be an http:// or https:// address")
    if (sos.telegram_bot_token is None) != (sos.telegram_chat_id is None):
        errors.append("sos: set both telegram_bot_token and telegram_chat_id, or neither")


def _merge(target: Any, data: dict[str, Any], prefix: str) -> None:
    hints = typing.get_type_hints(type(target))
    known = {f.name for f in fields(target)}
    for key, value in data.items():
        dotted = f"{prefix}{key}"
        if key not in known:
            raise ValueError(f"Unknown config key: {dotted}")
        current = getattr(target, key)
        if is_dataclass(current):
            if not isinstance(value, dict):
                raise ValueError(f"{dotted} must be an object")
            _merge(current, value, f"{dotted}.")
            continue
        setattr(target, key, _coerce(value, current, hints[key], dotted))


def _coerce(value: Any, current: Any, hint: Any, dotted: str) -> Any:
    if isinstance(current, tuple):
        if not isinstance(value, (list, tuple)):
            raise ValueError(f"{dotted} must be a list")
        return tuple(value)
    if isinstance(current, bool):
        if not isinstance(value, bool):
            raise ValueError(f"{dotted} must be true or false")
        return value
    if isinstance(current, float) and isinstance(value, int) and not isinstance(value, bool):
        return float(value)
    if current is None:
        allowed = tuple(arg for arg in typing.get_args(hint) if arg is not type(None))
        if value is not None and allowed and not isinstance(value, allowed):
            names = " or ".join(arg.__name__ for arg in allowed)
            raise ValueError(f"{dotted} must be {names} or null")
        return value
    if value is None or isinstance(value, bool) or not isinstance(value, type(current)):
        raise ValueError(f"{dotted} must be {type(current).__name__}, got {type(value).__name__}")
    return value
