from __future__ import annotations

import logging
import math

import pytest

from navsight.config import NavSightConfig
from navsight.hardware.gpio import FakeGpio
from navsight.hardware.haptics import PATTERNS, HapticController, intensity_to_duty
from navsight.util import ManualClock

LEFT_PIN = 12
RIGHT_PIN = 13
PWM_HZ = 200.0
MIN_DUTY = 0.3
MAX_DUTY = 1.0


class CountingGpio(FakeGpio):
    """FakeGpio that also records every PWM write."""

    def __init__(self) -> None:
        super().__init__()
        self.pwm_calls: list[tuple[int, float, float]] = []

    def pwm(self, pin: int, frequency_hz: float, duty_fraction: float) -> None:
        super().pwm(pin, frequency_hz, duty_fraction)
        self.pwm_calls.append((pin, frequency_hz, duty_fraction))


@pytest.fixture
def pwm_gpio() -> CountingGpio:
    return CountingGpio()


@pytest.fixture
def haptics(config: NavSightConfig, pwm_gpio: CountingGpio, clock: ManualClock) -> HapticController:
    controller = HapticController(config, pwm_gpio, clock=clock)
    controller.start()
    return controller


def duration(name: str) -> float:
    return sum(seconds for _, _, seconds in PATTERNS[name])


def pulses(name: str) -> list[tuple[float, float, float]]:
    """The steps of a pattern in which the motors run."""
    return [step for step in PATTERNS[name] if step[0] > 0 or step[1] > 0]


# intensity_to_duty


@pytest.mark.parametrize(
    "intensity, expected",
    [(1e-9, MIN_DUTY), (0.5, 0.65), (1.0, MAX_DUTY), (7.0, MAX_DUTY)],
)
def test_intensity_to_duty_maps_positive_intensities_into_the_duty_range(
    intensity, expected
) -> None:
    assert intensity_to_duty(intensity, MIN_DUTY, MAX_DUTY) == pytest.approx(expected)


@pytest.mark.parametrize("intensity", [0.0, -0.4, math.nan])
def test_intensity_to_duty_is_off_for_zero_negative_and_nan(intensity) -> None:
    assert intensity_to_duty(intensity, MIN_DUTY, MAX_DUTY) == 0.0


def test_intensity_to_duty_with_a_custom_range() -> None:
    assert intensity_to_duty(0.0, 0.2, 0.6) == 0.0
    assert intensity_to_duty(0.001, 0.2, 0.6) == pytest.approx(0.2004)
    assert intensity_to_duty(0.5, 0.2, 0.6) == pytest.approx(0.4)
    assert intensity_to_duty(1.0, 0.2, 0.6) == pytest.approx(0.6)
    assert intensity_to_duty(1.5, 0.2, 0.6) == pytest.approx(0.6)


# Pattern table


def test_patterns_match_their_descriptions() -> None:
    assert set(PATTERNS) == {
        "drop_off",
        "step_up",
        "confirm",
        "mode_change",
        "low_battery",
        "fall_detected",
        "sos_countdown",
        "sos_sent",
    }
    strong = pulses("drop_off")
    assert len(strong) == 3
    assert all(left == right == 1.0 for left, right, _ in strong)

    medium = pulses("step_up")
    assert len(medium) == 2
    assert all(0.0 < left < 1.0 for left, _, _ in medium)

    strong_s = strong[0][2]
    assert len(pulses("confirm")) == 2
    assert all(seconds < strong_s for *_, seconds in pulses("confirm"))
    assert len(pulses("mode_change")) == 1
    assert len(pulses("low_battery")) == 2
    assert all(seconds > strong_s for *_, seconds in pulses("low_battery"))
    assert len(pulses("fall_detected")) == 2
    assert all(seconds > strong_s for *_, seconds in pulses("fall_detected"))
    assert pulses("sos_countdown") == [(1.0, 1.0, 0.3)]
    assert len(pulses("sos_sent")) == 3
    assert all(seconds < strong_s for *_, seconds in pulses("sos_sent"))


@pytest.mark.parametrize("name", sorted(PATTERNS))
def test_every_pattern_is_well_formed(name) -> None:
    steps = PATTERNS[name]
    assert steps
    for left, right, seconds in steps:
        assert 0.0 <= left <= 1.0
        assert left == right  # every pattern runs both motors together
        assert seconds > 0
    assert steps[-1][:2] == (0.0, 0.0)  # a final pause keeps the last pulse distinct


# HapticController


def test_start_claims_both_motor_pins_as_outputs_driven_low(config, gpio, clock) -> None:
    HapticController(config, gpio, clock=clock).start()
    assert gpio.outputs == {LEFT_PIN: 0, RIGHT_PIN: 0}
    assert gpio.inputs == {}
    assert gpio.pwm_state == {}


def test_update_before_start_raises_without_touching_the_pins(config, pwm_gpio, clock) -> None:
    controller = HapticController(config, pwm_gpio, clock=clock)
    controller.set_continuous(1.0, 1.0)
    with pytest.raises(RuntimeError, match="start"):
        controller.update()
    assert pwm_gpio.pwm_calls == []


def test_continuous_levels_set_the_pwm_duty(haptics, pwm_gpio) -> None:
    haptics.set_continuous(0.5, 1.0)
    assert haptics.update() == (0.5, 1.0)
    assert pwm_gpio.pwm_state[LEFT_PIN] == (PWM_HZ, pytest.approx(0.65))
    assert pwm_gpio.pwm_state[RIGHT_PIN] == (PWM_HZ, 1.0)


def test_set_continuous_clamps_levels_and_treats_nan_as_off(haptics) -> None:
    haptics.set_continuous(1.7, -0.3)
    assert haptics.update() == (1.0, 0.0)
    haptics.set_continuous(math.nan, 0.25)
    assert haptics.update() == (0.0, 0.25)


def test_pwm_is_written_only_when_a_duty_changes(haptics, pwm_gpio) -> None:
    haptics.update()
    assert pwm_gpio.pwm_calls == []  # both motors are already off
    haptics.set_continuous(0.5, 0.5)
    for _ in range(5):
        haptics.update()
    assert len(pwm_gpio.pwm_calls) == 2
    haptics.set_continuous(0.5, 0.8)
    haptics.update()
    haptics.update()
    assert len(pwm_gpio.pwm_calls) == 3
    assert pwm_gpio.pwm_calls[-1] == (RIGHT_PIN, PWM_HZ, pytest.approx(0.86))
    haptics.set_continuous(0.0, 0.8)
    haptics.update()
    assert pwm_gpio.pwm_calls[-1] == (LEFT_PIN, PWM_HZ, 0.0)
    assert len(pwm_gpio.pwm_calls) == 4


def test_a_pattern_writes_pwm_only_when_its_step_changes(haptics, pwm_gpio, clock) -> None:
    start = clock()
    haptics.play("drop_off")
    for tick in range(round(duration("drop_off") / 0.01) + 20):  # 100 Hz, past the end
        haptics.update(start + tick * 0.01)
    # Each of the six on/off steps changes both duties once; the final pause already
    # matches the continuous level of 0, so the end of the pattern writes nothing.
    assert len(pwm_gpio.pwm_calls) == 2 * len(PATTERNS["drop_off"])


@pytest.mark.parametrize("name", sorted(PATTERNS))
def test_pattern_steps_play_at_the_right_times_then_continuous_resumes(
    name, haptics, clock
) -> None:
    haptics.set_continuous(0.2, 0.4)
    start = clock()
    haptics.play(name)
    step_start = 0.0
    for left, right, seconds in PATTERNS[name]:
        for fraction in (0.05, 0.5, 0.95):  # clear of the step boundaries
            assert haptics.update(start + step_start + fraction * seconds) == (left, right)
        step_start += seconds
    assert haptics.update(start + step_start + 0.001) == (0.2, 0.4)
    clock.advance(step_start + 0.001)
    assert haptics.playing is None


def test_pattern_overrides_continuous_and_drives_the_pwm(haptics, pwm_gpio, clock) -> None:
    haptics.set_continuous(0.5, 0.0)
    haptics.update()
    start = clock()
    haptics.play("step_up")
    level, _, seconds = PATTERNS["step_up"][0]
    assert haptics.update(start + seconds / 2) == (level, level)
    duty = intensity_to_duty(level, MIN_DUTY, MAX_DUTY)
    assert pwm_gpio.pwm_state == {LEFT_PIN: (PWM_HZ, duty), RIGHT_PIN: (PWM_HZ, duty)}
    assert haptics.update(start + duration("step_up") + 0.01) == (0.5, 0.0)
    assert pwm_gpio.pwm_state[LEFT_PIN] == (PWM_HZ, pytest.approx(0.65))
    assert pwm_gpio.pwm_state[RIGHT_PIN] == (PWM_HZ, 0.0)


def test_update_without_now_uses_the_clock(haptics, clock) -> None:
    haptics.play("low_battery")
    first_pulse, pause = PATTERNS["low_battery"][:2]
    clock.advance(first_pulse[2] + pause[2] / 2)
    assert haptics.update() == pause[:2]


def test_a_new_pattern_replaces_the_current_one(haptics, clock) -> None:
    haptics.set_continuous(0.3, 0.3)
    haptics.play("fall_detected")
    clock.advance(0.5)
    haptics.play("confirm")
    assert haptics.playing == "confirm"
    first_tap = PATTERNS["confirm"][0]
    clock.advance(first_tap[2] / 2)
    assert haptics.update() == first_tap[:2]
    clock.advance(duration("confirm"))  # fall_detected would still be playing by now
    assert haptics.update() == (0.3, 0.3)
    assert haptics.playing is None


def test_playing_the_same_pattern_again_restarts_it(haptics, clock) -> None:
    buzz_s = PATTERNS["sos_countdown"][0][2]
    haptics.play("sos_countdown")
    clock.advance(buzz_s * 0.8)
    haptics.play("sos_countdown")
    clock.advance(buzz_s * 0.5)  # past the end of the first buzz, inside the restarted one
    assert haptics.update() == (1.0, 1.0)


def test_unknown_pattern_raises_value_error_and_keeps_the_current_one(haptics) -> None:
    haptics.play("drop_off")
    with pytest.raises(ValueError, match="no_such_pattern"):
        haptics.play("no_such_pattern")
    assert haptics.playing == "drop_off"


def test_playing_reports_the_pattern_until_it_ends(haptics, clock) -> None:
    assert haptics.playing is None
    haptics.play("mode_change")
    assert haptics.playing == "mode_change"
    clock.advance(duration("mode_change") - 0.01)
    assert haptics.playing == "mode_change"
    clock.advance(0.02)
    assert haptics.playing is None


def test_stop_turns_both_motors_off(haptics, pwm_gpio) -> None:
    haptics.set_continuous(1.0, 0.6)
    haptics.play("fall_detected")
    haptics.update()
    assert pwm_gpio.pwm_state == {LEFT_PIN: (PWM_HZ, 1.0), RIGHT_PIN: (PWM_HZ, 1.0)}
    haptics.stop()
    assert pwm_gpio.pwm_state == {LEFT_PIN: (PWM_HZ, 0.0), RIGHT_PIN: (PWM_HZ, 0.0)}
    assert haptics.playing is None
    with pytest.raises(RuntimeError):
        haptics.update()  # a late worker tick must not switch the motors back on
    assert pwm_gpio.pwm_state == {LEFT_PIN: (PWM_HZ, 0.0), RIGHT_PIN: (PWM_HZ, 0.0)}


def test_stop_switches_off_even_if_nothing_was_written(haptics, pwm_gpio) -> None:
    haptics.stop()
    assert pwm_gpio.pwm_calls == [(LEFT_PIN, PWM_HZ, 0.0), (RIGHT_PIN, PWM_HZ, 0.0)]
    haptics.stop()
    assert len(pwm_gpio.pwm_calls) == 2


def test_stop_before_start_does_nothing(config, pwm_gpio, clock) -> None:
    HapticController(config, pwm_gpio, clock=clock).stop()
    assert pwm_gpio.pwm_calls == []


def test_stop_still_switches_off_one_motor_when_the_other_fails(config, clock, caplog) -> None:
    class BrokenLeftGpio(CountingGpio):
        def pwm(self, pin: int, frequency_hz: float, duty_fraction: float) -> None:
            if pin == LEFT_PIN:
                raise OSError("PWM failed")
            super().pwm(pin, frequency_hz, duty_fraction)

    gpio = BrokenLeftGpio()
    controller = HapticController(config, gpio, clock=clock)
    controller.start()
    with caplog.at_level(logging.ERROR, logger="navsight.hardware.haptics"):
        controller.stop()
    assert gpio.pwm_state == {RIGHT_PIN: (PWM_HZ, 0.0)}
    assert "GPIO 12" in caplog.text


def test_pwm_failure_names_the_pin_and_is_retried(config, clock) -> None:
    class FlakyGpio(CountingGpio):
        fail_pin: int | None = None

        def pwm(self, pin: int, frequency_hz: float, duty_fraction: float) -> None:
            if pin == self.fail_pin:
                self.fail_pin = None
                raise OSError("PWM failed")
            super().pwm(pin, frequency_hz, duty_fraction)

    gpio = FlakyGpio()
    controller = HapticController(config, gpio, clock=clock)
    controller.start()
    controller.set_continuous(0.0, 1.0)
    gpio.fail_pin = RIGHT_PIN
    with pytest.raises(RuntimeError, match="GPIO 13"):
        controller.update()
    controller.update()
    assert gpio.pwm_state == {RIGHT_PIN: (PWM_HZ, 1.0)}


def test_start_failure_names_the_pin(config, clock) -> None:
    class BusyGpio(FakeGpio):
        def claim_output(self, pin: int, level: int = 0) -> None:
            if pin == RIGHT_PIN:
                raise OSError("GPIO busy")
            super().claim_output(pin, level)

    controller = HapticController(config, BusyGpio(), clock=clock)
    with pytest.raises(RuntimeError, match="GPIO 13"):
        controller.start()
    with pytest.raises(RuntimeError, match="start"):
        controller.update()


def test_restart_after_stop_begins_with_the_motors_off(haptics, pwm_gpio) -> None:
    haptics.set_continuous(0.5, 0.5)
    haptics.update()
    haptics.stop()
    haptics.start()
    assert haptics.update() == (0.0, 0.0)  # stop() cleared the continuous levels
    haptics.set_continuous(0.0, 1.0)
    haptics.update()
    assert pwm_gpio.pwm_state[RIGHT_PIN] == (PWM_HZ, 1.0)


def test_pins_frequency_and_duty_range_come_from_the_config(pwm_gpio, clock) -> None:
    config = NavSightConfig()
    config.pins.haptic_left, config.pins.haptic_right = 5, 6
    config.haptics.pwm_frequency_hz = 150.0
    config.haptics.min_duty = 0.2
    config.haptics.max_duty = 0.6
    controller = HapticController(config, pwm_gpio, clock=clock)
    controller.start()
    assert pwm_gpio.outputs == {5: 0, 6: 0}
    controller.set_continuous(1.0, 0.5)
    controller.update()
    assert pwm_gpio.pwm_state == {5: (150.0, pytest.approx(0.6)), 6: (150.0, pytest.approx(0.4))}


@pytest.mark.parametrize(
    "settings, message",
    [
        ({"min_duty": 0.8, "max_duty": 0.5}, "min_duty"),
        ({"min_duty": -0.1}, "min_duty"),
        ({"max_duty": 1.2}, "max_duty"),
        ({"pwm_frequency_hz": 0.0}, "pwm_frequency_hz"),
    ],
)
def test_invalid_haptic_settings_are_rejected(settings, message, gpio, clock) -> None:
    config = NavSightConfig()
    for key, value in settings.items():
        setattr(config.haptics, key, value)
    with pytest.raises(ValueError, match=message):
        HapticController(config, gpio, clock=clock)
