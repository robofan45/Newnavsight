from __future__ import annotations

import sys
import threading

import pytest

from navsight.config import NavSightConfig
from navsight.hardware.buttons import ButtonDebouncer, Buttons, SimulatedButtons
from navsight.hardware.gpio import FakeGpio
from navsight.types import ButtonEvent, ButtonId, PressType
from navsight.util import ManualClock

PRESSED = 0  # active low: a pressed button pulls its pin to ground
RELEASED = 1
POLL_PERIOD_S = 0.005  # the 200 Hz button worker


def feed(debouncer: ButtonDebouncer, samples: list[tuple[float, int]]) -> list[ButtonEvent]:
    events = []
    for now, level in samples:
        event = debouncer.update(level, now)
        if event is not None:
            events.append(event)
    return events


def steady(level: int, start: float, end: float, step: float = 0.01) -> list[tuple[float, int]]:
    """Samples of one level every ``step`` from ``start`` to ``end`` inclusive."""
    count = round((end - start) / step)
    return [(start + i * step, level) for i in range(count + 1)]


def kinds(events: list[ButtonEvent]) -> list[tuple[ButtonId, PressType]]:
    return [(event.button, event.press) for event in events]


def poll_for(buttons: Buttons, clock: ManualClock, seconds: float) -> list[ButtonEvent]:
    """Poll now, then every 5 ms for ``seconds``, as the button worker does."""
    events = list(buttons.poll())
    for _ in range(round(seconds / POLL_PERIOD_S)):
        clock.advance(POLL_PERIOD_S)
        events.extend(buttons.poll())
    return events


@pytest.fixture
def buttons(config: NavSightConfig, gpio: FakeGpio, clock: ManualClock) -> Buttons:
    device = Buttons(config, gpio, clock=clock)
    device.start()
    return device


# ButtonDebouncer


def test_debouncer_starts_released_and_a_released_level_does_nothing() -> None:
    debouncer = ButtonDebouncer(ButtonId.MODE)
    assert not debouncer.pressed
    assert feed(debouncer, steady(RELEASED, 0.0, 5.0)) == []
    assert not debouncer.pressed


def test_bounce_shorter_than_debounce_is_ignored() -> None:
    debouncer = ButtonDebouncer(ButtonId.MODE, debounce_s=0.03)
    bounce = [(0.000, PRESSED), (0.010, RELEASED), (0.015, PRESSED), (0.025, RELEASED)]
    for now, level in bounce:
        assert debouncer.update(level, now) is None
        assert not debouncer.pressed
    assert feed(debouncer, steady(RELEASED, 0.03, 2.0)) == []
    assert not debouncer.pressed


def test_press_counts_once_stable_for_debounce_s() -> None:
    debouncer = ButtonDebouncer(ButtonId.MODE, debounce_s=0.03)
    assert debouncer.update(PRESSED, 10.0) is None
    assert debouncer.update(PRESSED, 10.029) is None
    assert not debouncer.pressed
    assert debouncer.update(PRESSED, 10.03) is None  # the press itself emits no event
    assert debouncer.pressed


def test_each_bounce_restarts_the_debounce_timer() -> None:
    debouncer = ButtonDebouncer(ButtonId.ACTION, debounce_s=0.03)
    feed(debouncer, [(0.0, PRESSED), (0.01, RELEASED), (0.02, PRESSED), (0.04, PRESSED)])
    assert not debouncer.pressed  # stable only since 0.02
    debouncer.update(PRESSED, 0.05)
    assert debouncer.pressed


def test_short_press_emits_short_on_release() -> None:
    debouncer = ButtonDebouncer(ButtonId.ACTION, debounce_s=0.03, long_press_s=1.0)
    assert feed(debouncer, steady(PRESSED, 0.0, 0.4)) == []
    assert debouncer.pressed
    assert debouncer.update(RELEASED, 0.41) is None  # the release is debounced too
    assert debouncer.pressed
    assert debouncer.update(RELEASED, 0.44) == ButtonEvent(ButtonId.ACTION, PressType.SHORT, 0.44)
    assert not debouncer.pressed
    assert feed(debouncer, steady(RELEASED, 0.45, 3.0)) == []


def test_holding_emits_long_once_at_long_press_s_and_nothing_on_release() -> None:
    debouncer = ButtonDebouncer(ButtonId.MODE, debounce_s=0.03, long_press_s=1.0)
    assert feed(debouncer, steady(PRESSED, 0.0, 0.99)) == []
    # Timed from the start of the press, not from the end of the debounce.
    assert debouncer.update(PRESSED, 1.0) == ButtonEvent(ButtonId.MODE, PressType.LONG, 1.0)
    assert feed(debouncer, steady(PRESSED, 1.01, 4.0)) == []  # no repeat while still held
    assert debouncer.pressed
    assert feed(debouncer, steady(RELEASED, 4.01, 5.0)) == []
    assert not debouncer.pressed


def test_release_glitch_during_a_hold_neither_ends_the_press_nor_restarts_the_timer() -> None:
    debouncer = ButtonDebouncer(ButtonId.MODE, debounce_s=0.03, long_press_s=1.0)
    samples = steady(PRESSED, 0.0, 0.5) + [(0.51, RELEASED)] + steady(PRESSED, 0.52, 0.99)
    assert feed(debouncer, samples) == []
    assert debouncer.pressed
    assert debouncer.update(PRESSED, 1.0) == ButtonEvent(ButtonId.MODE, PressType.LONG, 1.0)


def test_release_still_being_debounced_at_long_press_s_counts_as_short() -> None:
    debouncer = ButtonDebouncer(ButtonId.ACTION, debounce_s=0.03, long_press_s=1.0)
    feed(debouncer, steady(PRESSED, 0.0, 0.98))
    assert feed(debouncer, [(0.99, RELEASED), (1.0, RELEASED), (1.01, RELEASED)]) == []
    assert debouncer.update(RELEASED, 1.02) == ButtonEvent(ButtonId.ACTION, PressType.SHORT, 1.02)


def test_next_press_after_a_long_press_starts_fresh() -> None:
    debouncer = ButtonDebouncer(ButtonId.MODE, debounce_s=0.03, long_press_s=1.0)
    events = feed(debouncer, steady(PRESSED, 0.0, 1.5) + steady(RELEASED, 1.51, 2.0))
    events += feed(debouncer, steady(PRESSED, 2.01, 2.3) + steady(RELEASED, 2.31, 2.5))
    assert [event.press for event in events] == [PressType.LONG, PressType.SHORT]


def test_active_high_buttons_are_pressed_at_level_one() -> None:
    debouncer = ButtonDebouncer(ButtonId.MODE, debounce_s=0.03, active_low=False)
    assert feed(debouncer, steady(1, 0.0, 0.2)) == []
    assert debouncer.pressed
    events = feed(debouncer, steady(0, 0.21, 0.3))
    assert kinds(events) == [(ButtonId.MODE, PressType.SHORT)]
    assert events[0].timestamp == pytest.approx(0.24)


def test_zero_debounce_takes_changes_immediately() -> None:
    debouncer = ButtonDebouncer(ButtonId.MODE, debounce_s=0.0)
    assert debouncer.update(PRESSED, 1.0) is None
    assert debouncer.pressed
    assert debouncer.update(RELEASED, 1.2) == ButtonEvent(ButtonId.MODE, PressType.SHORT, 1.2)


@pytest.mark.parametrize(
    "timing, message",
    [({"debounce_s": -0.01}, "debounce_s"), ({"long_press_s": 0.0}, "long_press_s")],
)
def test_debouncer_rejects_invalid_timings(timing, message) -> None:
    with pytest.raises(ValueError, match=message):
        ButtonDebouncer(ButtonId.MODE, **timing)


# Buttons on FakeGpio


def test_start_claims_the_three_button_pins_as_pull_up_inputs(config, gpio, clock) -> None:
    Buttons(config, gpio, clock=clock).start()
    assert gpio.pull_ups == {17: True, 22: True, 27: True}
    assert gpio.inputs == {17: 1, 22: 1, 27: 1}
    assert gpio.outputs == {}


def test_start_twice_claims_the_pins_once(config, clock) -> None:
    class ClaimCountingGpio(FakeGpio):
        def __init__(self) -> None:
            super().__init__()
            self.claims: list[int] = []

        def claim_input(self, pin: int, pull_up: bool = True) -> None:
            super().claim_input(pin, pull_up)
            self.claims.append(pin)

    gpio = ClaimCountingGpio()
    buttons = Buttons(config, gpio, clock=clock)
    buttons.start()
    buttons.start()
    assert sorted(gpio.claims) == [17, 22, 27]


def test_poll_before_start_raises(config, gpio, clock) -> None:
    with pytest.raises(RuntimeError, match="start"):
        Buttons(config, gpio, clock=clock).poll()


def test_low_level_is_a_press_and_its_release_gives_short(buttons, gpio, clock) -> None:
    gpio.set_input(17, 0)
    assert poll_for(buttons, clock, 0.2) == []
    assert buttons.is_held(ButtonId.MODE)
    assert not buttons.is_held(ButtonId.ACTION)
    assert not buttons.is_held(ButtonId.SOS)
    gpio.set_input(17, 1)
    assert kinds(poll_for(buttons, clock, 0.1)) == [(ButtonId.MODE, PressType.SHORT)]
    assert not buttons.is_held(ButtonId.MODE)


def test_contact_bounce_on_a_pin_is_ignored(buttons, gpio, clock) -> None:
    for level in (0, 1, 0, 1):
        gpio.set_input(22, level)
        clock.advance(POLL_PERIOD_S)
        assert buttons.poll() == []
    assert poll_for(buttons, clock, 0.5) == []
    assert not buttons.is_held(ButtonId.ACTION)


def test_long_press_needs_long_press_s_and_sos_needs_sos_long_press_s(buttons, gpio, clock) -> None:
    for pin in (17, 22, 27):
        gpio.set_input(pin, 0)
    start = clock()
    events = poll_for(buttons, clock, 2.5)
    assert kinds(events) == [
        (ButtonId.MODE, PressType.LONG),
        (ButtonId.ACTION, PressType.LONG),
        (ButtonId.SOS, PressType.LONG),
    ]
    assert [event.timestamp - start for event in events] == pytest.approx([1.0, 1.0, 2.0])
    for pin in (17, 22, 27):
        gpio.set_input(pin, 1)
    assert poll_for(buttons, clock, 0.5) == []  # releasing after a LONG emits nothing


def test_sos_released_before_two_seconds_is_a_short_press(buttons, gpio, clock) -> None:
    gpio.set_input(27, 0)
    assert poll_for(buttons, clock, 1.5) == []
    assert buttons.is_held(ButtonId.SOS)
    gpio.set_input(27, 1)
    assert kinds(poll_for(buttons, clock, 0.1)) == [(ButtonId.SOS, PressType.SHORT)]


def test_pins_and_timings_come_from_the_config(gpio, clock) -> None:
    config = NavSightConfig()
    config.pins.button_mode, config.pins.button_action, config.pins.button_sos = 5, 6, 26
    config.buttons.debounce_s = 0.05
    config.buttons.long_press_s = 0.5
    config.buttons.sos_long_press_s = 3.0
    buttons = Buttons(config, gpio, clock=clock)
    buttons.start()
    assert gpio.pull_ups == {5: True, 6: True, 26: True}

    gpio.set_input(5, 0)
    gpio.set_input(26, 0)
    start = clock()
    assert poll_for(buttons, clock, 0.045) == []
    assert not buttons.is_held(ButtonId.MODE)  # 50 ms debounce here
    events = poll_for(buttons, clock, 3.5)
    assert buttons.is_held(ButtonId.MODE)
    assert kinds(events) == [(ButtonId.MODE, PressType.LONG), (ButtonId.SOS, PressType.LONG)]
    assert [event.timestamp - start for event in events] == pytest.approx([0.5, 3.0])


def test_drain_returns_queued_events_once(buttons, gpio, clock) -> None:
    gpio.set_input(22, 0)
    polled = poll_for(buttons, clock, 0.1)
    gpio.set_input(22, 1)
    polled += poll_for(buttons, clock, 0.1)
    gpio.set_input(27, 0)
    polled += poll_for(buttons, clock, 2.1)
    assert kinds(polled) == [(ButtonId.ACTION, PressType.SHORT), (ButtonId.SOS, PressType.LONG)]
    assert buttons.drain() == polled
    assert buttons.drain() == []


@pytest.fixture
def frequent_thread_switches():
    """Switch threads every microsecond, so a race between poll() and drain() shows up."""
    previous = sys.getswitchinterval()
    sys.setswitchinterval(1e-6)
    yield
    sys.setswitchinterval(previous)


@pytest.mark.usefixtures("frequent_thread_switches")
def test_drain_while_another_thread_polls_loses_and_repeats_nothing(buttons, gpio, clock) -> None:
    presses = 300

    def press_repeatedly() -> None:
        for _ in range(presses):
            for level in (0, 1):
                gpio.set_input(17, level)
                for _ in range(8):  # 40 ms per level, longer than the 30 ms debounce
                    clock.advance(POLL_PERIOD_S)
                    buttons.poll()

    poller = threading.Thread(target=press_repeatedly)
    drained: list[ButtonEvent] = []
    poller.start()
    while poller.is_alive():
        drained += buttons.drain()
    drained += buttons.drain()
    assert kinds(drained) == [(ButtonId.MODE, PressType.SHORT)] * presses
    timestamps = [event.timestamp for event in drained]
    assert timestamps == sorted(set(timestamps))


def test_stop_forgets_held_buttons_but_keeps_queued_events(buttons, gpio, clock) -> None:
    gpio.set_input(17, 0)
    poll_for(buttons, clock, 1.1)
    assert buttons.is_held(ButtonId.MODE)
    buttons.stop()
    assert not buttons.is_held(ButtonId.MODE)
    with pytest.raises(RuntimeError):
        buttons.poll()
    assert kinds(buttons.drain()) == [(ButtonId.MODE, PressType.LONG)]
    buttons.start()
    assert poll_for(buttons, clock, 0.1) == []
    assert buttons.is_held(ButtonId.MODE)  # still pressed, seen afresh after the restart


def test_claim_failure_names_the_pin_and_button(config, clock) -> None:
    class BusyGpio(FakeGpio):
        def claim_input(self, pin: int, pull_up: bool = True) -> None:
            if pin == 22:
                raise OSError("GPIO busy")
            super().claim_input(pin, pull_up)

    buttons = Buttons(config, BusyGpio(), clock=clock)
    with pytest.raises(RuntimeError, match="GPIO 22 for the action button"):
        buttons.start()
    with pytest.raises(RuntimeError, match="before start"):
        buttons.poll()


def test_read_failure_names_the_pin_and_later_polls_still_work(config, clock) -> None:
    class FlakyGpio(FakeGpio):
        fail_pin: int | None = None

        def read(self, pin: int) -> int:
            if pin == self.fail_pin:
                self.fail_pin = None
                raise OSError("read failed")
            return super().read(pin)

    gpio = FlakyGpio()
    buttons = Buttons(config, gpio, clock=clock)
    buttons.start()
    gpio.set_input(17, 0)
    buttons.poll()
    gpio.fail_pin = 27
    with pytest.raises(RuntimeError, match="GPIO 27"):
        buttons.poll()
    poll_for(buttons, clock, 0.1)
    gpio.set_input(17, 1)
    assert kinds(poll_for(buttons, clock, 0.1)) == [(ButtonId.MODE, PressType.SHORT)]


# SimulatedButtons


def test_simulated_inject_feeds_drain() -> None:
    sim = SimulatedButtons()
    sim.start()
    first = ButtonEvent(ButtonId.SOS, PressType.LONG, 5.0)
    second = ButtonEvent(ButtonId.MODE, PressType.SHORT, 6.0)
    sim.inject(first)
    sim.inject(second)
    assert sim.drain() == [first, second]
    assert sim.drain() == []


def test_simulated_poll_returns_each_injected_event_once() -> None:
    sim = SimulatedButtons()
    event = ButtonEvent(ButtonId.ACTION, PressType.SHORT, 1.0)
    sim.inject(event)
    assert sim.poll() == [event]
    assert sim.poll() == []
    assert sim.drain() == [event]  # polling does not consume what drain() reports


def test_simulated_is_held_follows_set_held_until_stop() -> None:
    sim = SimulatedButtons()
    assert not sim.is_held(ButtonId.SOS)
    sim.set_held(ButtonId.SOS)
    assert sim.is_held(ButtonId.SOS)
    assert not sim.is_held(ButtonId.MODE)
    sim.set_held(ButtonId.SOS, False)
    assert not sim.is_held(ButtonId.SOS)
    sim.set_held(ButtonId.MODE)
    sim.stop()
    assert not sim.is_held(ButtonId.MODE)


def test_simulated_buttons_accept_the_other_twins_constructor_arguments(config, clock) -> None:
    sim = SimulatedButtons(config, clock=clock)
    sim.start()
    assert sim.poll() == []
    assert sim.drain() == []
    sim.stop()
