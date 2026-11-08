from __future__ import annotations

import dataclasses
import logging

import pytest

from navsight.config import NavSightConfig
from navsight.power.state_machine import PowerInputs, PowerStateMachine
from navsight.types import PowerProfile, PowerState
from navsight.util import ManualClock

ACTIVE = PowerState.ACTIVE
LOW_POWER = PowerState.LOW_POWER
SLEEP = PowerState.SLEEP
T0 = 1000.0  # the conftest clock's start time, when the machine is created


def inputs(
    now: float,
    soc: float | None = 0.8,
    *,
    moving: bool = False,
    button: bool = False,
    sos: bool = False,
) -> PowerInputs:
    return PowerInputs(
        now=now, state_of_charge=soc, moving=moving, button_pressed=button, sos_active=sos
    )


@pytest.fixture
def machine(config: NavSightConfig, clock: ManualClock) -> PowerStateMachine:
    return PowerStateMachine(config, clock=clock)


def make_low_power_by_battery(machine: PowerStateMachine, now: float = T0) -> None:
    machine.update(inputs(now, 0.19, moving=True))
    assert machine.state is LOW_POWER


def put_to_sleep(machine: PowerStateMachine) -> None:
    """Idle from T0: LOW_POWER at T0 + 120 s, SLEEP at T0 + 600 s."""
    machine.update(inputs(T0 + 120.0))
    machine.update(inputs(T0 + 600.0))
    assert machine.state is SLEEP


# Profiles


def test_starts_active(machine: PowerStateMachine) -> None:
    assert machine.state is ACTIVE
    assert machine.last_transition is None


def test_active_profile(machine: PowerStateMachine) -> None:
    assert machine.profile_for(ACTIVE) == PowerProfile(
        state=ACTIVE,
        camera_fps=15.0,
        depth_enabled=True,
        tof_rate_hz=50.0,
        gps_poll_interval_s=0.0,
        fusion_rate_hz=30.0,
    )


def test_low_power_profile(machine: PowerStateMachine) -> None:
    assert machine.profile_for(LOW_POWER) == PowerProfile(
        state=LOW_POWER,
        camera_fps=5.0,
        depth_enabled=False,
        tof_rate_hz=25.0,
        gps_poll_interval_s=10.0,
        fusion_rate_hz=15.0,
    )


def test_sleep_profile(machine: PowerStateMachine) -> None:
    assert machine.profile_for(SLEEP) == PowerProfile(
        state=SLEEP,
        camera_fps=0.0,
        depth_enabled=False,
        tof_rate_hz=0.0,
        gps_poll_interval_s=None,
        fusion_rate_hz=2.0,
    )


def test_profiles_follow_the_config(config: NavSightConfig, clock: ManualClock) -> None:
    config.camera.fps = 10.0
    config.depth.enabled = False
    config.tof.rate_hz = 40.0
    config.fusion.rate_hz = 20.0
    config.power.low_power_camera_fps = 3.0
    config.power.low_power_tof_hz = 20.0
    config.power.low_power_gps_interval_s = 30.0
    config.power.low_power_fusion_hz = 10.0
    config.power.sleep_fusion_hz = 1.0
    machine = PowerStateMachine(config, clock=clock)

    active = machine.profile_for(ACTIVE)
    low_power = machine.profile_for(LOW_POWER)
    sleep = machine.profile_for(SLEEP)

    assert (active.camera_fps, active.depth_enabled, active.tof_rate_hz) == (10.0, False, 40.0)
    assert (active.gps_poll_interval_s, active.fusion_rate_hz) == (0.0, 20.0)
    assert (low_power.camera_fps, low_power.tof_rate_hz) == (3.0, 20.0)
    assert (low_power.gps_poll_interval_s, low_power.fusion_rate_hz) == (30.0, 10.0)
    assert sleep.fusion_rate_hz == 1.0


def test_update_returns_the_profile_of_the_new_state(machine: PowerStateMachine) -> None:
    assert machine.update(inputs(T0 + 1.0, moving=True)) == machine.profile_for(ACTIVE)
    assert machine.update(inputs(T0 + 2.0, 0.1, moving=True)) == machine.profile_for(LOW_POWER)


def test_power_inputs_are_immutable() -> None:
    sample = inputs(T0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        sample.moving = True  # type: ignore[misc]


# ACTIVE -> LOW_POWER


def test_low_battery_moves_active_to_low_power(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 1.0, 0.19, moving=True))
    assert machine.state is LOW_POWER
    assert machine.last_transition == (ACTIVE, LOW_POWER)


def test_battery_exactly_at_low_soc_stays_active(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 1.0, 0.20, moving=True))
    assert machine.state is ACTIVE


def test_unknown_battery_never_forces_low_power(machine: PowerStateMachine) -> None:
    for second in range(1, 3601):
        machine.update(inputs(T0 + second, None, moving=True))
    assert machine.state is ACTIVE


def test_idle_moves_active_to_low_power(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 119.9))
    assert machine.state is ACTIVE
    machine.update(inputs(T0 + 120.0))
    assert machine.state is LOW_POWER
    assert machine.last_transition == (ACTIVE, LOW_POWER)


def test_idle_timer_starts_when_the_machine_is_created(config: NavSightConfig) -> None:
    clock = ManualClock(50.0)
    machine = PowerStateMachine(config, clock=clock)
    machine.update(inputs(169.0))
    assert machine.state is ACTIVE
    machine.update(inputs(170.0))
    assert machine.state is LOW_POWER


def test_motion_refreshes_the_idle_timer(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 100.0, moving=True))
    machine.update(inputs(T0 + 219.0))
    assert machine.state is ACTIVE
    machine.update(inputs(T0 + 220.0))
    assert machine.state is LOW_POWER


def test_button_press_refreshes_the_idle_timer(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 100.0, button=True))
    machine.update(inputs(T0 + 219.0))
    assert machine.state is ACTIVE
    machine.update(inputs(T0 + 220.0))
    assert machine.state is LOW_POWER


def test_idle_timer_comes_from_the_config(config: NavSightConfig, clock: ManualClock) -> None:
    config.power.idle_to_low_power_s = 30.0
    machine = PowerStateMachine(config, clock=clock)
    machine.update(inputs(T0 + 29.0))
    assert machine.state is ACTIVE
    machine.update(inputs(T0 + 30.0))
    assert machine.state is LOW_POWER


# LOW_POWER -> ACTIVE


def test_low_power_returns_to_active_on_motion(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 120.0))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 130.0, moving=True))
    assert machine.state is ACTIVE
    assert machine.last_transition == (LOW_POWER, ACTIVE)


def test_low_power_returns_to_active_on_a_button_press(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 120.0))
    machine.update(inputs(T0 + 130.0, button=True))
    assert machine.state is ACTIVE


def test_low_power_needs_activity_to_return_to_active(machine: PowerStateMachine) -> None:
    make_low_power_by_battery(machine)
    machine.update(inputs(T0 + 1.0, 0.9))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 2.0, 0.9, moving=True))
    assert machine.state is ACTIVE


def test_soc_hysteresis_keeps_low_power_until_the_battery_recovers(
    machine: PowerStateMachine,
) -> None:
    make_low_power_by_battery(machine)  # dropped below 0.20
    machine.update(inputs(T0 + 1.0, 0.21, moving=True))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 2.0, 0.229, moving=True))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 3.0, 0.24, moving=True))
    assert machine.state is ACTIVE


def test_soc_hysteresis_threshold_is_inclusive(config: NavSightConfig, clock: ManualClock) -> None:
    config.power.low_soc = 0.25
    config.power.soc_hysteresis = 0.125  # threshold 0.375, exact in binary floating point
    machine = PowerStateMachine(config, clock=clock)
    machine.update(inputs(T0 + 1.0, 0.2, moving=True))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 2.0, 0.374, moving=True))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 3.0, 0.375, moving=True))
    assert machine.state is ACTIVE


def test_unknown_battery_allows_a_return_to_active(machine: PowerStateMachine) -> None:
    make_low_power_by_battery(machine)
    machine.update(inputs(T0 + 1.0, None, moving=True))
    assert machine.state is ACTIVE


# LOW_POWER -> SLEEP


def test_low_power_sleeps_after_idle_to_sleep_s(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 120.0))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 599.9))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 600.0))
    assert machine.state is SLEEP
    assert machine.last_transition == (LOW_POWER, SLEEP)


def test_sleep_timer_counts_from_the_last_activity(machine: PowerStateMachine) -> None:
    make_low_power_by_battery(machine, T0)
    machine.update(inputs(T0 + 300.0, 0.19, moving=True))  # still low: stays in LOW_POWER
    machine.update(inputs(T0 + 899.0, 0.19))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 900.0, 0.19))
    assert machine.state is SLEEP


def test_no_sleep_during_an_sos(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 120.0))
    machine.update(inputs(T0 + 600.0, sos=True))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 5000.0, sos=True))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 5001.0, sos=False))
    assert machine.state is SLEEP


def test_critical_battery_never_sleeps_while_the_user_walks(
    config: NavSightConfig, machine: PowerStateMachine
) -> None:
    assert config.power.critical_soc > 0.01
    for second in range(1, 7201):
        profile = machine.update(inputs(T0 + second, 0.01, moving=True))
        assert profile.state is LOW_POWER
        assert profile.tof_rate_hz == config.power.low_power_tof_hz


def test_critical_battery_alone_does_not_cause_sleep(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 1.0, 0.01))
    assert machine.state is LOW_POWER
    machine.update(inputs(T0 + 599.0, 0.01))
    assert machine.state is LOW_POWER


# SLEEP -> ACTIVE or LOW_POWER


def test_sleep_stays_asleep_without_activity_or_sos(machine: PowerStateMachine) -> None:
    put_to_sleep(machine)
    for offset, soc in ((601.0, 0.8), (700.0, 0.1), (5000.0, None)):
        machine.update(inputs(T0 + offset, soc))
        assert machine.state is SLEEP
        assert machine.last_transition is None


def test_sleep_wakes_on_motion_to_active(machine: PowerStateMachine) -> None:
    put_to_sleep(machine)
    profile = machine.update(inputs(T0 + 700.0, moving=True))
    assert machine.state is ACTIVE
    assert machine.last_transition == (SLEEP, ACTIVE)
    assert profile == machine.profile_for(ACTIVE)


def test_sleep_wakes_on_a_button_press_to_active(machine: PowerStateMachine) -> None:
    put_to_sleep(machine)
    machine.update(inputs(T0 + 700.0, button=True))
    assert machine.state is ACTIVE


def test_sleep_wakes_on_an_sos_to_active(machine: PowerStateMachine) -> None:
    put_to_sleep(machine)
    machine.update(inputs(T0 + 700.0, sos=True))
    assert machine.state is ACTIVE
    assert machine.last_transition == (SLEEP, ACTIVE)


@pytest.mark.parametrize("soc", [0.15, 0.21])
def test_sleep_wakes_to_low_power_when_the_battery_is_low(
    machine: PowerStateMachine, soc: float
) -> None:
    put_to_sleep(machine)
    machine.update(inputs(T0 + 700.0, soc, moving=True))
    assert machine.state is LOW_POWER
    assert machine.last_transition == (SLEEP, LOW_POWER)


def test_sos_wakes_to_low_power_when_the_battery_is_low(machine: PowerStateMachine) -> None:
    put_to_sleep(machine)
    machine.update(inputs(T0 + 700.0, 0.1, sos=True))
    assert machine.state is LOW_POWER


def test_sleep_wakes_to_active_when_the_battery_is_unknown(machine: PowerStateMachine) -> None:
    put_to_sleep(machine)
    machine.update(inputs(T0 + 700.0, None, moving=True))
    assert machine.state is ACTIVE


def test_idle_timer_restarts_after_waking(machine: PowerStateMachine) -> None:
    put_to_sleep(machine)
    machine.update(inputs(T0 + 700.0, moving=True))
    machine.update(inputs(T0 + 819.0))
    assert machine.state is ACTIVE
    machine.update(inputs(T0 + 820.0))
    assert machine.state is LOW_POWER


# last_transition and logging


def test_last_transition_is_set_only_by_the_update_that_changed_state(
    machine: PowerStateMachine,
) -> None:
    expected = [
        (T0 + 1.0, None),
        (T0 + 120.0, (ACTIVE, LOW_POWER)),
        (T0 + 121.0, None),
        (T0 + 600.0, (LOW_POWER, SLEEP)),
        (T0 + 601.0, None),
    ]
    for now, transition in expected:
        machine.update(inputs(now))
        assert machine.last_transition == transition
    machine.update(inputs(T0 + 602.0, moving=True))
    assert machine.last_transition == (SLEEP, ACTIVE)
    machine.update(inputs(T0 + 603.0, moving=True))
    assert machine.last_transition is None


def test_one_transition_per_update(machine: PowerStateMachine) -> None:
    machine.update(inputs(T0 + 700.0))  # idle past both timers
    assert machine.state is LOW_POWER
    assert machine.last_transition == (ACTIVE, LOW_POWER)
    machine.update(inputs(T0 + 700.1))
    assert machine.state is SLEEP


def test_transitions_are_logged(
    machine: PowerStateMachine, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.INFO, logger="navsight.power.state_machine")
    machine.update(inputs(T0 + 1.0, 0.15, moving=True))
    assert "active -> low_power: battery at 15%" in caplog.text
