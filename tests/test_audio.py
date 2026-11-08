from __future__ import annotations

import logging
import math
import queue
import subprocess
import sys
import threading
import types
from pathlib import Path

import numpy as np
import pytest

from navsight.config import AudioConfig, NavSightConfig
from navsight.hardware import audio
from navsight.hardware.audio import (
    AplayBackend,
    AudioOutput,
    NullBackend,
    RecordingTts,
    SoundDeviceBackend,
    SubprocessTts,
    bearing_to_pan,
    make_backend,
    make_tts,
    pan_gains,
    synth_tone,
)
from navsight.util import ManualClock

WAIT_S = 2.0  # upper bound on every wait for a worker thread, so a failing test never hangs
LOGGER = "navsight.hardware.audio"
AUDIO_THREADS = {"navsight-beeps", "navsight-speech"}


def energy(samples: np.ndarray) -> float:
    return float(np.sum(samples.astype(float) ** 2))


def audio_records(caplog: pytest.LogCaptureFixture) -> list[logging.LogRecord]:
    return [record for record in caplog.records if record.name == LOGGER]


def running_audio_threads() -> int:
    return sum(1 for thread in threading.enumerate() if thread.name in AUDIO_THREADS)


class PcmRecorder:
    """A backend that keeps every buffer it is asked to play."""

    def __init__(self) -> None:
        self.pcms: list[np.ndarray] = []
        self.rates: list[int] = []
        self.played: queue.Queue[int] = queue.Queue()
        self.closed = False

    def play_pcm(self, pcm: np.ndarray, sample_rate: int) -> None:
        self.pcms.append(pcm)
        self.rates.append(sample_rate)
        self.played.put(len(pcm))

    def close(self) -> None:
        self.closed = True


class FailingBackend:
    def __init__(self, *, fail_on_close: bool = False) -> None:
        self.attempts: queue.Queue[int] = queue.Queue()
        self._fail_on_close = fail_on_close

    def play_pcm(self, pcm: np.ndarray, sample_rate: int) -> None:
        self.attempts.put(len(pcm))
        raise OSError("Device or resource busy")

    def close(self) -> None:
        if self._fail_on_close:
            raise OSError("device vanished")


class FailingTts:
    def speak(self, text: str) -> None:
        raise RuntimeError("espeak-ng exited with code 1")

    def stop(self) -> None:
        raise RuntimeError("could not signal the TTS process")


class BlockingTts:
    """speak() blocks until ``release`` is set (stop() sets it), so tests control timing."""

    def __init__(self) -> None:
        self.started: queue.Queue[str] = queue.Queue()
        self.finished: queue.Queue[str] = queue.Queue()
        self.release = threading.Event()
        self.stop_calls = 0

    def speak(self, text: str) -> None:
        self.started.put(text)
        released = self.release.wait(WAIT_S)
        self.finished.put(text if released else f"never released: {text}")

    def stop(self) -> None:
        self.stop_calls += 1
        self.release.set()


class BlockingBackend:
    """play_pcm() blocks until ``release`` is set; buffers are identified by their frame count."""

    def __init__(self) -> None:
        self.started: queue.Queue[int] = queue.Queue()
        self.finished: queue.Queue[int] = queue.Queue()
        self.release = threading.Event()
        self.closed = False

    def play_pcm(self, pcm: np.ndarray, sample_rate: int) -> None:
        self.started.put(len(pcm))
        released = self.release.wait(WAIT_S)
        self.finished.put(len(pcm) if released else -1)

    def close(self) -> None:
        self.closed = True
        self.release.set()


@pytest.fixture
def started():
    """Start AudioOutputs in threaded mode and always stop them, even when a test fails."""
    outputs: list[AudioOutput] = []

    def start(backend, tts, config: NavSightConfig | None = None) -> AudioOutput:
        output = AudioOutput(config or NavSightConfig(), backend, tts)
        outputs.append(output)
        output.start()
        return output

    yield start
    for output in outputs:
        output.stop()


@pytest.fixture
def installed(monkeypatch):
    """Pretend that exactly the given programs are on PATH."""

    def install(*programs: str) -> None:
        found = set(programs)
        monkeypatch.setattr(
            audio.shutil, "which", lambda name: f"/usr/bin/{name}" if name in found else None
        )

    return install


# Panning -----------------------------------------------------------------------------------


@pytest.mark.parametrize("pan", [-1.0, -0.75, -0.4, -0.1, 0.0, 0.2, 0.5, 0.9, 1.0])
def test_pan_gains_are_constant_power(pan: float) -> None:
    left, right = pan_gains(pan)
    assert left**2 + right**2 == pytest.approx(1.0)
    assert left >= 0.0
    assert right >= 0.0


def test_pan_gains_are_equal_at_the_centre() -> None:
    left, right = pan_gains(0.0)
    assert left == pytest.approx(right)
    assert left == pytest.approx(math.sqrt(0.5))


def test_pan_gains_reach_full_left_and_full_right() -> None:
    assert pan_gains(-1.0) == pytest.approx((1.0, 0.0))
    assert pan_gains(1.0) == pytest.approx((0.0, 1.0), abs=1e-12)


@pytest.mark.parametrize("pan", [-0.8, -0.2, 0.35, 0.9])
def test_pan_gains_follow_the_contract_formula(pan: float) -> None:
    angle = (pan + 1.0) * math.pi / 4.0
    assert pan_gains(pan) == pytest.approx((math.cos(angle), math.sin(angle)))


def test_pan_gains_shift_to_the_right_as_pan_increases() -> None:
    rights = [pan_gains(pan)[1] for pan in np.linspace(-1.0, 1.0, 11)]
    assert np.all(np.diff(rights) > 0)


def test_pan_gains_clamp_out_of_range_pans_and_centre_nan() -> None:
    assert pan_gains(2.5) == pytest.approx(pan_gains(1.0))
    assert pan_gains(-2.5) == pytest.approx(pan_gains(-1.0))
    assert pan_gains(math.nan) == pytest.approx(pan_gains(0.0))


@pytest.mark.parametrize(
    "bearing, expected",
    [
        (0.0, 0.0),
        (22.5, 0.5),
        (-22.5, -0.5),
        (45.0, 1.0),
        (-45.0, -1.0),
        (60.0, 1.0),
        (-120.0, -1.0),
        (179.0, 1.0),
    ],
)
def test_bearing_to_pan_is_linear_then_clamped(bearing: float, expected: float) -> None:
    assert bearing_to_pan(bearing) == pytest.approx(expected)


def test_bearing_to_pan_scales_with_full_pan_bearing() -> None:
    assert bearing_to_pan(15.0, full_pan_bearing_deg=30.0) == pytest.approx(0.5)
    assert bearing_to_pan(-31.0, full_pan_bearing_deg=30.0) == -1.0


def test_bearing_to_pan_centres_nan() -> None:
    assert bearing_to_pan(math.nan) == 0.0


@pytest.mark.parametrize("full_pan", [0.0, -10.0])
def test_bearing_to_pan_rejects_a_non_positive_full_pan(full_pan: float) -> None:
    with pytest.raises(ValueError, match="full_pan_bearing_deg"):
        bearing_to_pan(10.0, full_pan)


# Tone synthesis ----------------------------------------------------------------------------


def test_synth_tone_is_int16_stereo_with_one_row_per_frame() -> None:
    pcm = synth_tone(1000.0, 0.06, 0.0, 0.8, 48000)
    assert pcm.dtype == np.int16
    assert pcm.shape == (2880, 2)


def test_synth_tone_has_the_requested_frequency() -> None:
    pcm = synth_tone(1000.0, 0.1, 0.0, 0.8, 48000)
    spectrum = np.abs(np.fft.rfft(pcm[:, 0].astype(float)))
    assert np.argmax(spectrum) * 48000 / len(pcm) == pytest.approx(1000.0, abs=10.0)


def test_synth_tone_fades_in_and_out_over_5_ms() -> None:
    sample_rate = 48000
    pcm = synth_tone(1000.0, 0.1, 0.0, 1.0, sample_rate).astype(float)
    peak = np.abs(pcm).max()
    fade = round(0.005 * sample_rate)
    assert np.abs(pcm[0]).max() <= 0.01 * peak
    assert np.abs(pcm[-1]).max() <= 0.01 * peak
    assert np.abs(pcm[: fade // 10]).max() < 0.05 * peak
    assert np.abs(pcm[-(fade // 10) :]).max() < 0.05 * peak
    assert np.abs(pcm[fade : 2 * fade]).max() == pytest.approx(peak, rel=0.01)


def test_synth_tone_shorter_than_both_fades_still_starts_and_ends_silent() -> None:
    pcm = synth_tone(1000.0, 0.004, 0.0, 1.0, 48000)
    assert pcm.shape == (192, 2)
    assert not pcm[0].any()
    assert np.abs(pcm[-1]).max() <= 0.01 * np.abs(pcm).max()


def test_synth_tone_positive_pan_puts_more_energy_in_the_right_channel() -> None:
    pcm = synth_tone(1000.0, 0.1, 0.5, 0.8, 48000)
    left_gain, right_gain = pan_gains(0.5)
    assert energy(pcm[:, 1]) > energy(pcm[:, 0])
    assert energy(pcm[:, 1]) / energy(pcm[:, 0]) == pytest.approx(
        (right_gain / left_gain) ** 2, rel=0.01
    )


def test_synth_tone_negative_pan_mirrors_a_positive_pan() -> None:
    right = synth_tone(800.0, 0.05, 0.6, 0.8, 48000).astype(int)
    left = synth_tone(800.0, 0.05, -0.6, 0.8, 48000).astype(int)
    assert np.abs(left[:, 0] - right[:, 1]).max() <= 1
    assert np.abs(left[:, 1] - right[:, 0]).max() <= 1


def test_synth_tone_centred_channels_are_identical() -> None:
    pcm = synth_tone(1200.0, 0.05, 0.0, 0.8, 48000)
    assert np.array_equal(pcm[:, 0], pcm[:, 1])


def test_synth_tone_volume_scales_the_amplitude() -> None:
    loud = synth_tone(1000.0, 0.05, 0.0, 1.0, 48000)
    quiet = synth_tone(1000.0, 0.05, 0.0, 0.25, 48000)
    assert np.abs(loud).max() == pytest.approx(32767 * math.sqrt(0.5), abs=1)
    assert np.abs(quiet).max() == pytest.approx(0.25 * 32767 * math.sqrt(0.5), abs=1)


def test_synth_tone_clamps_the_volume_to_full_scale() -> None:
    too_loud = synth_tone(1000.0, 0.05, 1.0, 3.0, 48000)
    assert np.array_equal(too_loud, synth_tone(1000.0, 0.05, 1.0, 1.0, 48000))
    assert np.abs(too_loud.astype(int)).max() <= 32767


def test_synth_tone_of_zero_duration_is_empty() -> None:
    pcm = synth_tone(1000.0, 0.0, 0.0, 0.8, 48000)
    assert pcm.shape == (0, 2)
    assert pcm.dtype == np.int16


@pytest.mark.parametrize(
    "frequency_hz, duration_s, sample_rate",
    [(0.0, 0.1, 48000), (-100.0, 0.1, 48000), (1000.0, -0.1, 48000), (1000.0, 0.1, 0)],
)
def test_synth_tone_rejects_invalid_arguments(
    frequency_hz: float, duration_s: float, sample_rate: int
) -> None:
    with pytest.raises(ValueError):
        synth_tone(frequency_hz, duration_s, 0.0, 0.8, sample_rate)


# Recording twins ---------------------------------------------------------------------------


def test_null_backend_records_frames_and_sample_rate() -> None:
    backend = NullBackend()
    backend.play_pcm(synth_tone(1000.0, 0.06, 0.0, 0.8, 48000), 48000)
    backend.play_pcm(synth_tone(1000.0, 0.01, 0.0, 0.8, 16000), 16000)
    backend.close()
    assert backend.played == [(2880, 48000), (160, 16000)]
    assert backend.closed


def test_recording_tts_records_text_and_stops() -> None:
    tts = RecordingTts()
    tts.speak("Drop ahead.")
    tts.speak("SOS sent.")
    tts.stop()
    assert tts.spoken == ["Drop ahead.", "SOS sent."]
    assert tts.stop_count == 1


# AudioOutput, synchronous ------------------------------------------------------------------


def test_synchronous_beep_plays_on_the_backend(config: NavSightConfig) -> None:
    backend, tts = NullBackend(), RecordingTts()
    output = AudioOutput(config, backend, tts, synchronous=True)
    output.beep(1500.0, 0.06, bearing_deg=20.0)
    assert backend.played == [(2880, 48000)]
    assert tts.spoken == []


def test_synchronous_speech_goes_to_the_tts(config: NavSightConfig) -> None:
    backend, tts = NullBackend(), RecordingTts()
    output = AudioOutput(config, backend, tts, synchronous=True)
    output.say("person slightly left, 2 meters.")
    assert tts.spoken == ["person slightly left, 2 meters."]
    assert backend.played == []
    assert not output.is_speaking


def test_beeps_are_panned_by_bearing(config: NavSightConfig) -> None:
    backend = PcmRecorder()
    output = AudioOutput(config, backend, RecordingTts(), synchronous=True)
    for bearing in (30.0, -30.0, 0.0):
        output.beep(1000.0, 0.05, bearing_deg=bearing)
    right, left, centre = backend.pcms
    assert energy(right[:, 1]) > 3 * energy(right[:, 0])
    assert energy(left[:, 0]) > 3 * energy(left[:, 1])
    assert np.array_equal(centre[:, 0], centre[:, 1])


def test_beeps_use_the_audio_config(config: NavSightConfig) -> None:
    config.audio.sample_rate = 16000
    config.audio.volume = 0.5
    config.audio.full_pan_bearing_deg = 20.0
    backend = PcmRecorder()
    output = AudioOutput(config, backend, RecordingTts(), synchronous=True)
    output.beep(1000.0, 0.1, bearing_deg=25.0)
    [pcm] = backend.pcms
    assert backend.rates == [16000]
    assert pcm.shape == (1600, 2)
    assert not pcm[:, 0].any()  # 25 degrees is beyond the 20 degree full pan: right only
    assert np.abs(pcm[:, 1]).max() == pytest.approx(0.5 * 32767, abs=2)


def test_synchronous_interrupt_stops_the_tts_then_speaks(config: NavSightConfig) -> None:
    tts = RecordingTts()
    output = AudioOutput(config, NullBackend(), tts, synchronous=True)
    output.say("person ahead, 3 meters.")
    output.say("Drop ahead.", interrupt=True)
    assert tts.spoken == ["person ahead, 3 meters.", "Drop ahead."]
    assert tts.stop_count == 1


def test_blank_text_is_not_spoken_but_can_still_interrupt(config: NavSightConfig) -> None:
    tts = RecordingTts()
    output = AudioOutput(config, NullBackend(), tts, synchronous=True)
    output.say("   ")
    output.say("", interrupt=True)
    assert tts.spoken == []
    assert tts.stop_count == 1


def test_backend_errors_are_logged_not_raised(
    config: NavSightConfig, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.ERROR, logger=LOGGER)
    output = AudioOutput(config, FailingBackend(), RecordingTts(), synchronous=True)
    output.beep(1000.0, 0.05)
    assert output.error_count == 1
    assert "Audio beep failed" in caplog.text
    assert "Device or resource busy" in caplog.text


def test_tts_errors_are_logged_not_raised(
    config: NavSightConfig, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.ERROR, logger=LOGGER)
    output = AudioOutput(config, NullBackend(), FailingTts(), synchronous=True)
    output.say("hello")
    output.say("Drop ahead.", interrupt=True)
    assert output.error_count == 3
    assert "Audio speech failed" in caplog.text
    assert "Audio speech stop failed" in caplog.text


def test_invalid_beeps_are_logged_not_raised(
    config: NavSightConfig, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.ERROR, logger=LOGGER)
    backend = NullBackend()
    output = AudioOutput(config, backend, RecordingTts(), synchronous=True)
    output.beep(0.0, 0.06)
    assert backend.played == []
    assert output.error_count == 1
    assert "frequency_hz must be positive" in caplog.text


def test_repeated_errors_are_logged_at_most_every_30_s(
    config: NavSightConfig, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.ERROR, logger=LOGGER)
    clock = ManualClock(100.0)
    output = AudioOutput(config, FailingBackend(), RecordingTts(), synchronous=True, clock=clock)
    for _ in range(3):
        output.beep(1000.0, 0.05)
    clock.advance(29.0)
    output.beep(1000.0, 0.05)
    assert len(audio_records(caplog)) == 1
    clock.advance(1.0)
    output.beep(1000.0, 0.05)
    records = audio_records(caplog)
    assert len(records) == 2
    assert output.error_count == 5
    assert "5 audio errors so far" in records[-1].getMessage()


def test_each_kind_of_failure_is_logged_separately(
    config: NavSightConfig, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.ERROR, logger=LOGGER)
    output = AudioOutput(config, FailingBackend(), FailingTts(), synchronous=True)
    output.beep(1000.0, 0.05)
    output.say("hello")
    messages = [record.getMessage() for record in audio_records(caplog)]
    assert [message.split(" failed")[0] for message in messages] == ["Audio beep", "Audio speech"]


def test_stop_closes_the_backend_and_logs_close_errors(
    config: NavSightConfig, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.ERROR, logger=LOGGER)
    tts = RecordingTts()
    output = AudioOutput(config, FailingBackend(fail_on_close=True), tts, synchronous=True)
    output.stop()
    assert tts.stop_count == 1
    assert "Audio backend close failed" in caplog.text


# AudioOutput, threaded ---------------------------------------------------------------------


def test_threaded_speech_keeps_two_pending_and_drops_the_oldest(started) -> None:
    tts = BlockingTts()
    output = started(NullBackend(), tts)
    output.say("one")
    assert tts.started.get(timeout=WAIT_S) == "one"
    for text in ("two", "three", "four"):
        output.say(text)
    tts.release.set()
    spoken = [tts.finished.get(timeout=WAIT_S) for _ in range(3)]
    output.stop()
    assert spoken == ["one", "three", "four"]
    assert tts.finished.empty()


def test_interrupt_clears_pending_speech_and_stops_the_current_utterance(started) -> None:
    tts = BlockingTts()
    output = started(NullBackend(), tts)
    output.say("person ahead, 3 meters.")
    assert tts.started.get(timeout=WAIT_S) == "person ahead, 3 meters."
    output.say("chair on your left, 2 meters.")
    output.say("Drop ahead.", interrupt=True)
    assert tts.stop_calls == 1
    assert tts.finished.get(timeout=WAIT_S) == "person ahead, 3 meters."
    assert tts.started.get(timeout=WAIT_S) == "Drop ahead."
    assert tts.finished.get(timeout=WAIT_S) == "Drop ahead."
    output.stop()
    assert tts.started.empty()  # the chair was cleared from the queue, never spoken


def test_is_speaking_covers_the_current_and_pending_utterances(started) -> None:
    tts = BlockingTts()
    output = started(NullBackend(), tts)
    assert not output.is_speaking
    output.say("hello")
    assert output.is_speaking
    assert tts.started.get(timeout=WAIT_S) == "hello"
    assert output.is_speaking
    tts.release.set()
    assert tts.finished.get(timeout=WAIT_S) == "hello"
    output.stop()
    assert not output.is_speaking


def test_threaded_beeps_keep_four_pending_and_drop_the_oldest(started) -> None:
    backend = BlockingBackend()
    output = started(backend, RecordingTts())
    output.beep(1000.0, 0.01)  # 480 frames at 48 kHz identify this beep
    assert backend.started.get(timeout=WAIT_S) == 480
    for hundredths in range(2, 7):
        output.beep(1000.0, 0.01 * hundredths)  # 960, 1440, 1920, 2400 and 2880 frames
    backend.release.set()
    played = [backend.finished.get(timeout=WAIT_S) for _ in range(5)]
    output.stop()
    assert played == [480, 1440, 1920, 2400, 2880]
    assert backend.finished.empty()


def test_a_beep_is_not_stuck_behind_a_long_sentence(started) -> None:
    tts = BlockingTts()
    backend = PcmRecorder()
    output = started(backend, tts)
    output.say("A long description of everything ahead of you.")
    assert tts.started.get(timeout=WAIT_S)
    output.beep(2000.0, 0.06, bearing_deg=-30.0)
    assert backend.played.get(timeout=WAIT_S) == 2880
    assert tts.finished.empty()  # the sentence is still being spoken


def test_threaded_backend_errors_are_logged_and_the_thread_keeps_going(
    started, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.ERROR, logger=LOGGER)
    backend = FailingBackend()
    output = started(backend, RecordingTts())
    output.beep(1000.0, 0.05)
    assert backend.attempts.get(timeout=WAIT_S) == 2400
    output.beep(1000.0, 0.05)
    assert backend.attempts.get(timeout=WAIT_S) == 2400
    output.stop()
    assert output.error_count == 2
    assert "Audio beep failed" in caplog.text


def test_stop_interrupts_speech_drops_pending_items_and_ends_the_threads(
    config: NavSightConfig,
) -> None:
    before = running_audio_threads()
    tts, backend = BlockingTts(), NullBackend()
    output = AudioOutput(config, backend, tts)
    output.start()
    output.say("hello")
    assert tts.started.get(timeout=WAIT_S) == "hello"
    output.say("still waiting")
    output.stop()
    assert tts.finished.get(timeout=WAIT_S) == "hello"
    assert tts.started.empty()
    assert tts.stop_calls == 1
    assert backend.closed
    assert not output.is_speaking
    assert running_audio_threads() <= before


def test_start_is_idempotent_and_the_output_restarts_after_stop(config: NavSightConfig) -> None:
    before = running_audio_threads()
    tts = BlockingTts()
    tts.release.set()
    output = AudioOutput(config, NullBackend(), tts)
    try:
        output.start()
        output.start()
        assert running_audio_threads() == before + 2
        output.stop()
        output.start()
        output.say("again")
        assert tts.finished.get(timeout=WAIT_S) == "again"
    finally:
        output.stop()


def test_beeps_wait_in_the_queue_until_start(config: NavSightConfig) -> None:
    backend = PcmRecorder()
    output = AudioOutput(config, backend, RecordingTts())
    output.beep(1000.0, 0.01)
    assert backend.played.empty()
    try:
        output.start()
        assert backend.played.get(timeout=WAIT_S) == 480
    finally:
        output.stop()


# Backends ----------------------------------------------------------------------------------


def fake_sounddevice() -> types.ModuleType:
    module = types.ModuleType("sounddevice")
    module.calls = []

    def play(data, samplerate=None, device=None, blocking=False):
        module.calls.append(("play", data.shape, samplerate, device, blocking))

    def stop():
        module.calls.append(("stop",))

    module.play = play
    module.stop = stop
    return module


def test_sounddevice_backend_plays_blocking_and_stops_on_close(monkeypatch) -> None:
    module = fake_sounddevice()
    monkeypatch.setitem(sys.modules, "sounddevice", module)
    backend = SoundDeviceBackend("hw:2,0")
    backend.play_pcm(synth_tone(1000.0, 0.01, 0.0, 0.5, 48000), 48000)
    backend.close()
    assert module.calls == [("play", (480, 2), 48000, "hw:2,0", True), ("stop",)]


def fake_run(calls: list, returncode: int = 0, stderr: bytes = b""):
    def run(command, **options):
        calls.append((command, options))
        return subprocess.CompletedProcess(command, returncode, stdout=None, stderr=stderr)

    return run


def test_aplay_backend_pipes_raw_stereo_pcm(monkeypatch) -> None:
    calls: list = []
    monkeypatch.setattr(audio.subprocess, "run", fake_run(calls))
    pcm = synth_tone(1000.0, 0.01, 0.5, 0.5, 48000)
    AplayBackend("plughw:CARD=sndrpihifiberry").play_pcm(pcm, 48000)
    [(command, options)] = calls
    raw_format = ["-t", "raw", "-f", "S16_LE", "-c", "2", "-r", "48000"]
    assert command == ["aplay", "-q", *raw_format, "-D", "plughw:CARD=sndrpihifiberry"]
    assert options["input"] == pcm.astype("<i2").tobytes()
    assert options["timeout"] > 0.01


def test_aplay_backend_without_a_device_uses_the_alsa_default(monkeypatch) -> None:
    calls: list = []
    monkeypatch.setattr(audio.subprocess, "run", fake_run(calls))
    AplayBackend().play_pcm(synth_tone(1000.0, 0.01, 0.0, 0.5, 16000), 16000)
    [(command, _)] = calls
    assert "-D" not in command
    assert command[-2:] == ["-r", "16000"]


def test_aplay_failures_raise_with_the_error_text(monkeypatch) -> None:
    error = b"aplay: main:850: audio open error: Device or resource busy"
    monkeypatch.setattr(audio.subprocess, "run", fake_run([], returncode=1, stderr=error))
    with pytest.raises(RuntimeError, match="Device or resource busy"):
        AplayBackend().play_pcm(synth_tone(1000.0, 0.01, 0.0, 0.5, 48000), 48000)


def test_aplay_backend_needs_stereo_pcm() -> None:
    with pytest.raises(ValueError, match="stereo"):
        AplayBackend().play_pcm(np.zeros(480, dtype=np.int16), 48000)


def test_make_backend_none_is_silent() -> None:
    assert isinstance(make_backend(AudioConfig(backend="none")), NullBackend)


def test_make_backend_auto_prefers_sounddevice(monkeypatch, installed) -> None:
    monkeypatch.setitem(sys.modules, "sounddevice", fake_sounddevice())
    installed("aplay")
    assert isinstance(make_backend(AudioConfig()), SoundDeviceBackend)


def test_make_backend_auto_uses_aplay_when_sounddevice_is_missing(monkeypatch, installed) -> None:
    monkeypatch.setitem(sys.modules, "sounddevice", None)  # "import sounddevice" fails
    installed("aplay")
    assert isinstance(make_backend(AudioConfig()), AplayBackend)


def test_make_backend_auto_uses_aplay_when_portaudio_is_missing(
    monkeypatch, installed, tmp_path: Path
) -> None:
    (tmp_path / "sounddevice.py").write_text(
        'raise OSError("PortAudio library not found")\n', encoding="utf-8"
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    monkeypatch.delitem(sys.modules, "sounddevice", raising=False)
    installed("aplay")
    assert isinstance(make_backend(AudioConfig()), AplayBackend)


def test_make_backend_auto_falls_back_to_silence(
    monkeypatch, installed, caplog: pytest.LogCaptureFixture
) -> None:
    monkeypatch.setitem(sys.modules, "sounddevice", None)
    installed()
    assert isinstance(make_backend(AudioConfig()), NullBackend)
    assert "beeps are disabled" in caplog.text


def test_make_backend_passes_the_device_to_aplay(monkeypatch, installed) -> None:
    calls: list = []
    monkeypatch.setattr(audio.subprocess, "run", fake_run(calls))
    installed("aplay")
    backend = make_backend(AudioConfig(backend="aplay", device="hw:1,0"))
    backend.play_pcm(synth_tone(1000.0, 0.01, 0.0, 0.5, 48000), 48000)
    [(command, _)] = calls
    assert command[-2:] == ["-D", "hw:1,0"]


def test_make_backend_explicit_choices_fail_loudly(monkeypatch, installed) -> None:
    monkeypatch.setitem(sys.modules, "sounddevice", None)
    installed()
    with pytest.raises(RuntimeError, match="libportaudio2"):
        make_backend(AudioConfig(backend="sounddevice"))
    with pytest.raises(RuntimeError, match="alsa-utils"):
        make_backend(AudioConfig(backend="aplay"))
    with pytest.raises(ValueError, match="audio.backend"):
        make_backend(AudioConfig(backend="pulseaudio"))


# Text-to-speech ----------------------------------------------------------------------------


class FakeProcess:
    """Stands in for a subprocess.Popen object. With ``hold`` it runs until terminated."""

    def __init__(
        self,
        command: list[str],
        *,
        returncode: int = 0,
        stderr: bytes = b"",
        hold: bool = False,
        on_communicate=None,
    ) -> None:
        self.args = command
        self.returncode: int | None = None
        self.input: bytes | None = None
        self.terminated = threading.Event()
        self._result = returncode
        self._stderr = stderr
        self._hold = hold
        self._on_communicate = on_communicate

    def communicate(self, data: bytes | None = None, timeout: float | None = None):
        self.input = data
        if self._on_communicate is not None:
            self._on_communicate()
        if self._hold:
            self.returncode = -15 if self.terminated.wait(WAIT_S) else 0
        else:
            self.returncode = self._result
        return None, self._stderr

    def terminate(self) -> None:
        self.terminated.set()

    def kill(self) -> None:
        self.terminated.set()


class FakePopen:
    """Records every process started; ``behaviours`` configure them in order."""

    def __init__(self, *behaviours: dict) -> None:
        self.behaviours = list(behaviours)
        self.processes: list[FakeProcess] = []
        self.started: queue.Queue[FakeProcess] = queue.Queue()

    def __call__(self, command: list[str], **options) -> FakeProcess:
        process = FakeProcess(command, **(self.behaviours.pop(0) if self.behaviours else {}))
        self.processes.append(process)
        self.started.put(process)
        return process


@pytest.fixture
def popen(monkeypatch) -> FakePopen:
    fake = FakePopen()
    monkeypatch.setattr(audio.subprocess, "Popen", fake)
    return fake


def test_espeak_writes_a_wav_file_that_aplay_plays(installed, popen: FakePopen) -> None:
    installed("espeak-ng", "aplay")
    SubprocessTts("espeak-ng", rate_wpm=180, device="plughw:1,0").speak("Drop ahead.")
    synth, play = popen.processes
    wav = synth.args[4]
    assert synth.args == ["espeak-ng", "-s", "180", "-w", wav, "--stdin"]
    assert synth.input == b"Drop ahead."
    assert wav.endswith(".wav")
    assert play.args == ["aplay", "-q", "-D", "plughw:1,0", wav]
    assert not Path(wav).parent.exists()  # the temporary folder is removed afterwards


def test_pico2wave_gets_the_text_as_an_argument(installed, popen: FakePopen) -> None:
    installed("pico2wave", "aplay")
    SubprocessTts("pico2wave").speak("Step up ahead.")
    synth, play = popen.processes
    wav = synth.args[4]
    assert synth.args == ["pico2wave", "-l", "en-US", "-w", wav, "--", "Step up ahead."]
    assert synth.input is None
    assert play.args == ["aplay", "-q", wav]


def test_auto_engine_is_the_first_one_installed(installed) -> None:
    installed("espeak", "pico2wave", "aplay")
    assert SubprocessTts().engine == "espeak"


def test_a_failed_synthesis_raises_with_its_error_text(installed, popen: FakePopen) -> None:
    installed("espeak-ng", "aplay")
    popen.behaviours.append({"returncode": 1, "stderr": b"Failed to read voice 'en'"})
    with pytest.raises(RuntimeError, match="Failed to read voice"):
        SubprocessTts("espeak-ng").speak("hello")
    assert len(popen.processes) == 1  # nothing was played


def test_stop_kills_the_running_step_and_skips_playback(installed, popen: FakePopen) -> None:
    installed("espeak-ng", "aplay")
    popen.behaviours.append({"hold": True})
    tts = SubprocessTts("espeak-ng")
    speaker = threading.Thread(target=tts.speak, args=("A long sentence.",), daemon=True)
    speaker.start()
    synth = popen.started.get(timeout=WAIT_S)
    tts.stop()
    speaker.join(WAIT_S)
    assert not speaker.is_alive()
    assert synth.terminated.is_set()
    assert [process.args for process in popen.processes] == [synth.args]

    tts.speak("Speech works again after a stop.")
    assert len(popen.processes) == 3


def test_stop_between_synthesis_and_playback_skips_playback(installed, popen: FakePopen) -> None:
    installed("espeak-ng", "aplay")
    tts = SubprocessTts("espeak-ng")
    popen.behaviours.append({"on_communicate": tts.stop})  # stop() lands during synthesis
    tts.speak("hello")
    assert len(popen.processes) == 1


def test_subprocess_tts_needs_aplay(installed) -> None:
    installed("espeak-ng")
    with pytest.raises(RuntimeError, match="alsa-utils"):
        SubprocessTts("espeak-ng")


def test_make_tts_none_records() -> None:
    assert isinstance(make_tts(AudioConfig(tts_engine="none")), RecordingTts)


@pytest.mark.parametrize(
    "programs, expected",
    [
        (("espeak-ng", "espeak", "pico2wave"), "espeak-ng"),
        (("espeak", "pico2wave"), "espeak"),
        (("pico2wave",), "pico2wave"),
    ],
)
def test_make_tts_auto_picks_the_first_installed_engine(
    installed, programs: tuple[str, ...], expected: str
) -> None:
    installed(*programs, "aplay")
    tts = make_tts(AudioConfig())
    assert isinstance(tts, SubprocessTts)
    assert tts.engine == expected


def test_make_tts_auto_without_an_engine_records(
    installed, caplog: pytest.LogCaptureFixture
) -> None:
    installed("aplay")
    assert isinstance(make_tts(AudioConfig()), RecordingTts)
    assert "espeak-ng" in caplog.text


def test_make_tts_passes_rate_and_device(installed, popen: FakePopen) -> None:
    installed("espeak-ng", "aplay")
    make_tts(AudioConfig(tts_rate_wpm=200, device="hw:1,0")).speak("hi")
    synth, play = popen.processes
    assert synth.args[1:3] == ["-s", "200"]
    assert play.args[2:4] == ["-D", "hw:1,0"]


def test_make_tts_explicit_engine_must_be_installed(installed) -> None:
    installed("espeak-ng", "aplay")
    with pytest.raises(RuntimeError, match="libttspico-utils"):
        make_tts(AudioConfig(tts_engine="pico2wave"))


def test_make_tts_rejects_unknown_engines(installed) -> None:
    installed("festival", "aplay")
    with pytest.raises(ValueError, match="festival"):
        make_tts(AudioConfig(tts_engine="festival"))


def test_audio_output_speaks_through_subprocess_tts(
    config: NavSightConfig, installed, popen: FakePopen
) -> None:
    installed("espeak-ng", "aplay")
    output = AudioOutput(config, NullBackend(), SubprocessTts("espeak-ng"), synchronous=True)
    output.say("Fall detected.")
    assert [process.args[0] for process in popen.processes] == ["espeak-ng", "aplay"]
    assert popen.processes[0].input == b"Fall detected."
