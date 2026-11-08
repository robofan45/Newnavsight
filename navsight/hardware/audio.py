"""Spatial beeps and text-to-speech through the I2S DAC and the bone-conduction transducer.

True left/right audio needs two MAX98357A boards (one set to the left channel, one to the
right) or a stereo DAC: a single MAX98357A is mono, so panning has no effect with one board.
"""

from __future__ import annotations

import logging
import math
import shutil
import subprocess
import tempfile
import threading
import time
from collections import deque
from collections.abc import Callable
from pathlib import Path
from typing import Any, Generic, Protocol, TypeVar

import numpy as np

from navsight.config import AudioConfig, NavSightConfig
from navsight.types import Beep
from navsight.util import Clock, clamp

log = logging.getLogger(__name__)

T = TypeVar("T")

FADE_S = 0.005  # raised-cosine fade at each end of a tone, so it does not click
INT16_MAX = 32767
BEEP_QUEUE_SIZE = 4
SPEECH_QUEUE_SIZE = 2
STOP_TIMEOUT_S = 2.0
ERROR_LOG_INTERVAL_S = 30.0
APLAY_TIMEOUT_MARGIN_S = 5.0
TTS_STEP_TIMEOUT_S = 30.0
TTS_ENGINES = ("espeak-ng", "espeak", "pico2wave")
TTS_INSTALL_HINTS = {
    "espeak-ng": "sudo apt install espeak-ng",
    "espeak": "sudo apt install espeak",
    "pico2wave": "sudo apt install libttspico-utils",
}
PICO2WAVE_LANGUAGE = "en-US"


def bearing_to_pan(bearing_deg: float, full_pan_bearing_deg: float = 45.0) -> float:
    """Stereo pan for a bearing: -1 = full left, 0 = centre, +1 = full right.

    Bearings beyond ``full_pan_bearing_deg`` on either side are fully panned; NaN is centred.
    """
    if full_pan_bearing_deg <= 0.0:
        raise ValueError("full_pan_bearing_deg must be positive")
    if math.isnan(bearing_deg):
        return 0.0
    return clamp(bearing_deg / full_pan_bearing_deg, -1.0, 1.0)


def pan_gains(pan: float) -> tuple[float, float]:
    """Constant-power (left, right) gains: ``(cos t, sin t)`` with ``t = (pan + 1) * pi / 4``.

    ``left**2 + right**2 == 1`` for every pan, and both gains are equal at pan 0. Pans outside
    -1..1 are clamped; NaN is centred.
    """
    pan = 0.0 if math.isnan(pan) else clamp(pan, -1.0, 1.0)
    angle = (pan + 1.0) * math.pi / 4.0
    return math.cos(angle), math.sin(angle)


def synth_tone(
    frequency_hz: float, duration_s: float, pan: float, volume: float, sample_rate: int
) -> np.ndarray:
    """A panned sine tone as int16 stereo PCM shaped (frames, 2), with 5 ms fades."""
    if frequency_hz <= 0.0:
        raise ValueError("frequency_hz must be positive")
    if duration_s < 0.0:
        raise ValueError("duration_s cannot be negative")
    if sample_rate <= 0:
        raise ValueError("sample_rate must be positive")
    frames = round(duration_s * sample_rate)
    times = np.arange(frames) / sample_rate
    wave = np.sin(2.0 * np.pi * frequency_hz * times) * _fade_envelope(frames, sample_rate)
    left, right = pan_gains(pan)
    amplitude = clamp(volume, 0.0, 1.0) * INT16_MAX
    stereo = np.column_stack((wave * left, wave * right)) * amplitude
    return np.round(stereo).astype(np.int16)


def _fade_envelope(frames: int, sample_rate: int) -> np.ndarray:
    envelope = np.ones(frames)
    fade_frames = min(round(FADE_S * sample_rate), frames // 2)
    if fade_frames > 0:
        ramp = 0.5 * (1.0 - np.cos(np.pi * np.arange(fade_frames) / fade_frames))
        envelope[:fade_frames] = ramp
        envelope[-fade_frames:] = ramp[::-1]
    return envelope


class AudioBackend(Protocol):
    def play_pcm(self, pcm: np.ndarray, sample_rate: int) -> None: ...  # blocks until played

    def close(self) -> None: ...


class SoundDeviceBackend:
    """PortAudio output through the sounddevice package (needs the libportaudio2 library).

    ``device`` is a PortAudio device index or a substring of its name, such as "hw:2,0".
    """

    def __init__(self, device: str | None = None) -> None:
        import sounddevice  # raises OSError when the PortAudio library is missing

        self._sounddevice: Any = sounddevice
        self._device = device

    def play_pcm(self, pcm: np.ndarray, sample_rate: int) -> None:
        self._sounddevice.play(pcm, samplerate=sample_rate, device=self._device, blocking=True)

    def close(self) -> None:
        self._sounddevice.stop()


class AplayBackend:
    """Pipes raw 16-bit stereo PCM to aplay (alsa-utils); ``device`` is an ALSA PCM name."""

    def __init__(self, device: str | None = None) -> None:
        self._device = device

    def play_pcm(self, pcm: np.ndarray, sample_rate: int) -> None:
        if pcm.ndim != 2 or pcm.shape[1] != 2:
            raise ValueError(f"aplay needs stereo PCM shaped (frames, 2), got {pcm.shape}")
        command = ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-c", "2", "-r", str(sample_rate)]
        if self._device:
            command += ["-D", self._device]
        result = subprocess.run(
            command,
            input=np.ascontiguousarray(pcm, dtype="<i2").tobytes(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            timeout=len(pcm) / sample_rate + APLAY_TIMEOUT_MARGIN_S,
            check=False,
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"aplay exited with code {result.returncode}: {_text(result.stderr)}"
            )

    def close(self) -> None:
        """Nothing to release: every play_pcm call runs its own aplay process."""


class NullBackend:
    """Plays nothing; records each call as (frames, sample_rate) for the simulator and tests."""

    def __init__(self) -> None:
        self.played: list[tuple[int, int]] = []
        self.closed = False

    def play_pcm(self, pcm: np.ndarray, sample_rate: int) -> None:
        self.played.append((len(pcm), sample_rate))

    def close(self) -> None:
        self.closed = True


class TtsEngine(Protocol):
    def speak(self, text: str) -> None: ...  # blocks until finished or stopped

    def stop(self) -> None: ...


class SubprocessTts:
    """Text-to-speech with espeak-ng, espeak or pico2wave, played through aplay.

    The engine writes the speech to a temporary WAV file that aplay then plays, so ``device``
    (an ALSA PCM name) selects the output for every engine. ``stop()`` kills the running
    process and skips any step not yet started. pico2wave has no speed setting and ignores
    ``rate_wpm``.
    """

    def __init__(
        self, engine: str = "auto", rate_wpm: int = 165, device: str | None = None
    ) -> None:
        self.engine = _installed_tts_engine(engine)
        if shutil.which("aplay") is None:
            raise RuntimeError(
                "Speech is played with aplay, which is missing: sudo apt install alsa-utils"
            )
        self._rate_wpm = rate_wpm
        self._device = device
        self._lock = threading.Lock()
        self._process: subprocess.Popen[bytes] | None = None
        self._stopped = False

    def speak(self, text: str) -> None:
        with self._lock:
            self._stopped = False
        with tempfile.TemporaryDirectory(prefix="navsight-tts-") as folder:
            wav_path = Path(folder) / "speech.wav"
            if self._run(*self._synthesis_step(text, wav_path)):
                self._run(self._playback_command(wav_path), None)

    def stop(self) -> None:
        with self._lock:
            self._stopped = True
            if self._process is not None:
                self._process.terminate()

    def _synthesis_step(self, text: str, wav_path: Path) -> tuple[list[str], bytes | None]:
        """The command that writes ``text`` as speech to ``wav_path``, and its stdin data."""
        if self.engine == "pico2wave":
            return ["pico2wave", "-l", PICO2WAVE_LANGUAGE, "-w", str(wav_path), "--", text], None
        command = [self.engine, "-s", str(self._rate_wpm), "-w", str(wav_path), "--stdin"]
        return command, text.encode("utf-8")

    def _playback_command(self, wav_path: Path) -> list[str]:
        command = ["aplay", "-q"]
        if self._device:
            command += ["-D", self._device]
        return [*command, str(wav_path)]

    def _run(self, command: list[str], stdin_data: bytes | None) -> bool:
        """Run one step to the end: False if stop() interrupted it; raises if it failed."""
        with self._lock:
            if self._stopped:
                return False
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL if stdin_data is None else subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            self._process = process
        try:
            _, stderr = process.communicate(stdin_data, timeout=TTS_STEP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()
            raise RuntimeError(
                f"{command[0]} did not finish within {TTS_STEP_TIMEOUT_S:.0f} s"
            ) from None
        finally:
            with self._lock:
                self._process = None
        if self._stopped:
            return False
        if process.returncode != 0:
            raise RuntimeError(
                f"{command[0]} exited with code {process.returncode}: {_text(stderr)}"
            )
        return True


class RecordingTts:
    """Speaks nothing; records the text for the simulator and tests."""

    def __init__(self) -> None:
        self.spoken: list[str] = []
        self.stop_count = 0

    def speak(self, text: str) -> None:
        log.debug("Speech (not spoken aloud): %s", text)
        self.spoken.append(text)

    def stop(self) -> None:
        self.stop_count += 1


def make_backend(config: AudioConfig) -> AudioBackend:
    """The configured beep output; "auto" tries sounddevice, then aplay, then NullBackend."""
    choice = config.backend
    if choice == "none":
        return NullBackend()
    if choice == "sounddevice":
        try:
            return SoundDeviceBackend(config.device)
        except (ImportError, OSError) as exc:
            raise RuntimeError(
                f"audio.backend is 'sounddevice' but it cannot be loaded ({exc}): "
                "pip install sounddevice and sudo apt install libportaudio2"
            ) from exc
    if choice == "aplay":
        if shutil.which("aplay") is None:
            raise RuntimeError(
                "audio.backend is 'aplay' but aplay is missing: sudo apt install alsa-utils"
            )
        return AplayBackend(config.device)
    if choice != "auto":
        raise ValueError(f"audio.backend must be auto, sounddevice, aplay or none, not {choice!r}")
    try:
        return SoundDeviceBackend(config.device)
    except (ImportError, OSError) as exc:
        log.info("sounddevice is unavailable (%s); trying aplay", exc)
    if shutil.which("aplay") is not None:
        return AplayBackend(config.device)
    log.warning("No audio output found (install sounddevice or alsa-utils); beeps are disabled")
    return NullBackend()


def make_tts(config: AudioConfig) -> TtsEngine:
    """The configured speech engine; "auto" picks the first installed of espeak-ng, espeak and
    pico2wave, or a RecordingTts (speech disabled) when none is installed."""
    if config.tts_engine == "none":
        return RecordingTts()
    if config.tts_engine == "auto" and _find_tts_engine() is None:
        log.warning("No text-to-speech engine found (sudo apt install espeak-ng); speech is off")
        return RecordingTts()
    return SubprocessTts(config.tts_engine, config.tts_rate_wpm, config.device)


def _find_tts_engine() -> str | None:
    for engine in TTS_ENGINES:
        if shutil.which(engine) is not None:
            return engine
    return None


def _installed_tts_engine(engine: str) -> str:
    if engine == "auto":
        found = _find_tts_engine()
        if found is None:
            raise RuntimeError("No text-to-speech engine found: sudo apt install espeak-ng")
        return found
    if engine not in TTS_ENGINES:
        raise ValueError(f"TTS engine must be auto, espeak-ng, espeak or pico2wave, not {engine!r}")
    if shutil.which(engine) is None:
        raise RuntimeError(f"TTS engine {engine} is not installed: {TTS_INSTALL_HINTS[engine]}")
    return engine


def _text(data: bytes | None) -> str:
    return (data or b"").decode(errors="replace").strip()


class _PlaybackQueue(Generic[T]):
    """Pending items served in order by one daemon thread.

    At most ``max_pending`` items wait (the one being played does not count); adding another
    drops the oldest waiting item.
    """

    def __init__(self, name: str, max_pending: int, handler: Callable[[T], None]) -> None:
        self._name = name
        self._handler = handler
        self._pending: deque[T] = deque(maxlen=max_pending)
        self._condition = threading.Condition()
        self._active = 0
        self._running = False
        self._thread: threading.Thread | None = None

    @property
    def busy(self) -> bool:
        """True while an item is being handled or waiting."""
        with self._condition:
            return self._active > 0 or len(self._pending) > 0

    def start(self) -> None:
        with self._condition:
            self._running = True
            if self._thread is not None:
                return
            self._thread = threading.Thread(target=self._serve, name=self._name, daemon=True)
        self._thread.start()

    def put(self, item: T) -> None:
        with self._condition:
            if len(self._pending) == self._pending.maxlen:
                log.debug("%s queue is full; dropping the oldest item", self._name)
            self._pending.append(item)
            self._condition.notify()

    def clear(self) -> None:
        with self._condition:
            self._pending.clear()

    def run_now(self, item: T) -> None:
        """Handle ``item`` in the caller's thread (synchronous mode)."""
        with self._condition:
            self._active += 1
        self._handle(item)

    def stop(self) -> None:
        """Drop the waiting items and let the thread exit after the current one."""
        with self._condition:
            self._running = False
            self._pending.clear()
            self._condition.notify_all()

    def join(self, timeout: float) -> None:
        thread = self._thread
        if thread is None:
            return
        thread.join(timeout)
        if thread.is_alive():
            log.warning("%s thread did not stop within %.1f s", self._name, timeout)
        else:
            self._thread = None

    def _serve(self) -> None:
        while True:
            with self._condition:
                while self._running and not self._pending:
                    self._condition.wait()
                if not self._running:
                    return
                item = self._pending.popleft()
                self._active += 1
            self._handle(item)

    def _handle(self, item: T) -> None:
        try:
            self._handler(item)
        finally:
            with self._condition:
                self._active -= 1


class AudioOutput:
    """Spatial beeps and speech, each on its own queue and thread.

    A beep is never stuck behind a long sentence: beeps keep at most 4 pending and speech at
    most 2, dropping the oldest. ``say(..., interrupt=True)`` clears the speech queue and stops
    the current utterance. With ``synchronous=True`` (tests and simulator) every call runs in
    the caller's thread. Backend and TTS errors are logged (at most once per 30 s for each kind
    of failure, counted in ``error_count``) and never raised to the caller.
    """

    def __init__(
        self,
        config: NavSightConfig,
        backend: AudioBackend,
        tts: TtsEngine,
        *,
        synchronous: bool = False,
        clock: Clock = time.monotonic,
    ) -> None:
        self._settings = config.audio
        self._backend = backend
        self._tts = tts
        self._synchronous = synchronous
        self._clock = clock
        self._beeps: _PlaybackQueue[Beep] = _PlaybackQueue(
            "navsight-beeps", BEEP_QUEUE_SIZE, self._play_beep
        )
        self._speech: _PlaybackQueue[str] = _PlaybackQueue(
            "navsight-speech", SPEECH_QUEUE_SIZE, self._speak
        )
        self._error_lock = threading.Lock()
        self._last_error_log: dict[str, float] = {}
        self.error_count = 0

    @property
    def is_speaking(self) -> bool:
        """True while an utterance is playing or waiting to be played."""
        return self._speech.busy

    def start(self) -> None:
        if not self._synchronous:
            self._beeps.start()
            self._speech.start()

    def stop(self) -> None:
        """Drop pending audio, interrupt speech, close the backend and end the threads."""
        self._beeps.stop()
        self._speech.stop()
        self._stop_tts()
        try:
            self._backend.close()
        except Exception:
            self._log_error("backend close")
        self._beeps.join(STOP_TIMEOUT_S)
        self._speech.join(STOP_TIMEOUT_S)

    def beep(self, frequency_hz: float, duration_s: float, bearing_deg: float = 0.0) -> None:
        beep = Beep(frequency_hz, duration_s, bearing_deg)
        if self._synchronous:
            self._beeps.run_now(beep)
        else:
            self._beeps.put(beep)

    def say(self, text: str, interrupt: bool = False) -> None:
        """Queue ``text`` for speech; empty text only interrupts (when asked to)."""
        if interrupt:
            self._speech.clear()
            self._stop_tts()
        if not text.strip():
            return
        if self._synchronous:
            self._speech.run_now(text)
        else:
            self._speech.put(text)

    def _play_beep(self, beep: Beep) -> None:
        settings = self._settings
        try:
            pan = bearing_to_pan(beep.bearing_deg, settings.full_pan_bearing_deg)
            pcm = synth_tone(
                beep.frequency_hz, beep.duration_s, pan, settings.volume, settings.sample_rate
            )
            if len(pcm) > 0:
                self._backend.play_pcm(pcm, settings.sample_rate)
        except Exception:
            self._log_error("beep")

    def _speak(self, text: str) -> None:
        try:
            self._tts.speak(text)
        except Exception:
            self._log_error("speech")

    def _stop_tts(self) -> None:
        try:
            self._tts.stop()
        except Exception:
            self._log_error("speech stop")

    def _log_error(self, what: str) -> None:
        """Count a failure and log it with its traceback, at most once per interval per kind.

        Call only from an ``except`` block.
        """
        with self._error_lock:
            self.error_count += 1
            count = self.error_count
            now = self._clock()
            last = self._last_error_log.get(what)
            if last is not None and now - last < ERROR_LOG_INTERVAL_S:
                return
            self._last_error_log[what] = now
        log.exception("Audio %s failed (%d audio errors so far)", what, count)
