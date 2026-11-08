from __future__ import annotations

import threading
import time

import pytest

from navsight.hardware.gpio import FakeGpio
from navsight.util import (
    LatestValue,
    ManualClock,
    SampleQueue,
    Worker,
    clamp,
    wrap_angle_deg,
    wrap_heading_deg,
)


@pytest.mark.parametrize(
    "angle, expected",
    [(0, 0), (180, 180), (-180, 180), (190, -170), (-190, 170), (540, 180), (359, -1)],
)
def test_wrap_angle(angle, expected) -> None:
    assert wrap_angle_deg(angle) == pytest.approx(expected)


@pytest.mark.parametrize("angle, expected", [(0, 0), (360, 0), (-90, 270), (725, 5), (-1e-15, 0)])
def test_wrap_heading(angle, expected) -> None:
    result = wrap_heading_deg(angle)
    assert 0 <= result < 360
    assert result == pytest.approx(expected, abs=1e-9)


def test_clamp() -> None:
    assert clamp(5, 0, 1) == 1
    assert clamp(-5, 0, 1) == 0
    assert clamp(0.5, 0, 1) == 0.5


def test_manual_clock() -> None:
    clock = ManualClock(10.0)
    assert clock() == 10.0
    assert clock.advance(0.5) == 10.5
    with pytest.raises(ValueError):
        clock.advance(-1)


def test_latest_value_versions() -> None:
    slot: LatestValue[int] = LatestValue()
    assert slot.get_with_version() == (None, 0)
    slot.set(3)
    slot.set(4)
    assert slot.get_with_version() == (4, 2)


def test_sample_queue_drains_in_order_and_drops_oldest() -> None:
    queue: SampleQueue[int] = SampleQueue(maxlen=3)
    for value in range(5):
        queue.put(value)
    assert len(queue) == 3
    assert queue.dropped == 2
    assert queue.drain() == [2, 3, 4]
    assert queue.drain() == []


def test_worker_survives_exceptions_and_stops() -> None:
    calls = 0
    done = threading.Event()

    def step() -> None:
        nonlocal calls
        calls += 1
        if calls >= 5:
            done.set()
        raise RuntimeError("sensor unplugged")

    worker = Worker("test", step, rate_hz=200.0, error_log_interval_s=60.0)
    worker.start()
    assert done.wait(2.0)
    worker.stop()
    assert not worker.is_running
    assert worker.error_count >= 5


def test_worker_rate_zero_pauses() -> None:
    calls = 0

    def step() -> None:
        nonlocal calls
        calls += 1

    worker = Worker("paused", step, rate_hz=0.0)
    worker.start()
    time.sleep(0.1)
    worker.stop()
    assert calls == 0


def test_fake_gpio_rejects_unclaimed_outputs() -> None:
    gpio = FakeGpio()
    gpio.claim_input(17)
    assert gpio.read(17) == 1
    with pytest.raises(RuntimeError):
        gpio.write(12, 1)
    gpio.claim_output(12)
    gpio.pwm(12, 200.0, 0.5)
    assert gpio.pwm_state[12] == (200.0, 0.5)
