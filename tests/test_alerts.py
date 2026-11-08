from __future__ import annotations

import logging
import math
from collections.abc import Sequence

import pytest

from navsight.alerts.priority import (
    BEEP_DURATION_S,
    MAX_PENDING_EVENTS,
    AlertManager,
    direction_phrase,
    distance_phrase,
    distance_to_beep,
)
from navsight.config import AlertConfig, NavSightConfig
from navsight.types import (
    Alert,
    AlertLevel,
    AlertOutputs,
    Beep,
    GroundHazard,
    HazardType,
    Obstacle,
    Speech,
    Verbosity,
)
from navsight.util import ManualClock

T0 = 1000.0
FALL_MESSAGE = "Fall detected. Sending SOS in 15 seconds. Press any button to cancel."


def make_obstacle(
    distance_m: float | None,
    bearing_deg: float = 0.0,
    *,
    label: str | None = None,
    ttc_s: float | None = None,
    sector: int | None = None,
) -> Obstacle:
    if sector is None:
        sector = min(6, max(0, math.floor((bearing_deg + 52.5) / 15.0)))
    return Obstacle(
        sector=sector,
        bearing_deg=bearing_deg,
        distance_m=distance_m,
        closing_speed_mps=0.0,
        time_to_collision_s=ttc_s,
        label=label,
        source="vision" if label else "tof",
        confidence=0.9 if label else 1.0,
    )


def make_event(
    key: str,
    level: AlertLevel = AlertLevel.INFO,
    message: str = "GPS signal lost.",
    pattern: str | None = None,
) -> Alert:
    return Alert(key=key, level=level, message=message, haptic_pattern=pattern)


def make_hazard(kind: HazardType = HazardType.DROP_OFF, timestamp: float = T0) -> GroundHazard:
    delta_m = 0.2 if kind is HazardType.DROP_OFF else -0.15
    return GroundHazard(hazard=kind, delta_m=delta_m, timestamp=timestamp)


def run(
    manager: AlertManager,
    now: float,
    obstacles: Sequence[Obstacle] = (),
    hazard: GroundHazard | None = None,
    events: Sequence[Alert] = (),
) -> AlertOutputs:
    return manager.update(now, list(obstacles), hazard, list(events))


def texts(outputs: AlertOutputs) -> list[str]:
    return [item.text for item in outputs.speech]


def motors(outputs: AlertOutputs) -> tuple[float, float]:
    return outputs.haptic_left, outputs.haptic_right


@pytest.fixture
def manager(config: NavSightConfig, clock: ManualClock) -> AlertManager:
    return AlertManager(config, clock=clock)


# Phrases and beep zones


@pytest.mark.parametrize(
    "bearing_deg, expected",
    [
        (0.0, "ahead"),
        (9.9, "ahead"),
        (-9.9, "ahead"),
        (10.0, "slightly right"),
        (-10.0, "slightly left"),
        (-20.0, "slightly left"),
        (35.0, "slightly right"),
        (-35.0, "slightly left"),
        (35.1, "on your right"),
        (-35.1, "on your left"),
        (90.0, "on your right"),
        (-60.0, "on your left"),
    ],
)
def test_direction_phrase(bearing_deg: float, expected: str) -> None:
    assert direction_phrase(bearing_deg) == expected


@pytest.mark.parametrize(
    "distance_m, expected",
    [
        (-0.1, "very close"),
        (0.0, "very close"),
        (0.99, "very close"),
        (1.0, "1 meter"),
        (1.49, "1 meter"),
        (1.5, "2 meters"),
        (2.0, "2 meters"),
        (2.4, "2 meters"),
        (2.5, "3 meters"),
        (10.2, "10 meters"),
    ],
)
def test_distance_phrase(distance_m: float, expected: str) -> None:
    assert distance_phrase(distance_m) == expected


@pytest.mark.parametrize("distance_m", [math.nan, math.inf, -math.inf])
def test_distance_phrase_rejects_non_finite_distances(distance_m: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        distance_phrase(distance_m)


@pytest.mark.parametrize(
    "distance_m, expected",
    [
        (-0.05, (2000.0, 0.08)),
        (0.0, (2000.0, 0.08)),
        (0.49, (2000.0, 0.08)),
        (0.5, (1500.0, 0.2)),
        (0.99, (1500.0, 0.2)),
        (1.0, (1000.0, 0.5)),
        (1.99, (1000.0, 0.5)),
        (2.0, None),
        (3.5, None),
        (math.nan, None),
    ],
)
def test_distance_to_beep_zones(
    config: NavSightConfig, distance_m: float, expected: tuple[float, float] | None
) -> None:
    assert distance_to_beep(distance_m, config.alerts) == expected


def test_distance_to_beep_uses_the_configured_zones() -> None:
    alerts = AlertConfig(danger_m=0.3, warning_m=0.6, caution_m=1.2)
    assert distance_to_beep(0.2, alerts) == (2000.0, 0.08)
    assert distance_to_beep(0.4, alerts) == (1500.0, 0.2)
    assert distance_to_beep(1.0, alerts) == (1000.0, 0.5)
    assert distance_to_beep(1.2, alerts) is None


# Haptics


def test_nothing_around_gives_quiet_outputs(manager: AlertManager) -> None:
    outputs = run(manager, T0)
    assert outputs == AlertOutputs(0.0, 0.0, None, (), ())


@pytest.mark.parametrize(
    "distance_m, level",
    [(1.9, 0.1 / 1.5), (1.25, 0.5), (0.8, 0.8), (0.5, 1.0), (0.1, 1.0), (-0.05, 1.0)],
)
def test_haptic_level_follows_distance(
    manager: AlertManager, distance_m: float, level: float
) -> None:
    outputs = run(manager, T0, [make_obstacle(distance_m)])
    assert motors(outputs) == pytest.approx((level, level))


@pytest.mark.parametrize("distance_m", [2.0, 2.5, 4.0])
def test_no_vibration_at_or_beyond_caution(manager: AlertManager, distance_m: float) -> None:
    assert motors(run(manager, T0, [make_obstacle(distance_m)])) == (0.0, 0.0)


@pytest.mark.parametrize(
    "ttc_s, level", [(1.0, 0.8), (0.2, 0.8), (1.5, 0.2), (3.0, 0.2), (None, 0.2)]
)
def test_short_time_to_collision_raises_the_level(
    manager: AlertManager, ttc_s: float | None, level: float
) -> None:
    outputs = run(manager, T0, [make_obstacle(1.7, ttc_s=ttc_s)])
    assert motors(outputs) == pytest.approx((level, level))


def test_time_to_collision_boost_never_lowers_the_level(manager: AlertManager) -> None:
    outputs = run(manager, T0, [make_obstacle(0.6, ttc_s=1.0)])
    assert motors(outputs) == pytest.approx((1.4 / 1.5, 1.4 / 1.5))


def test_time_to_collision_is_ignored_outside_caution(manager: AlertManager) -> None:
    assert motors(run(manager, T0, [make_obstacle(3.0, ttc_s=0.5)])) == (0.0, 0.0)


@pytest.mark.parametrize(
    "bearing_deg, sides",
    [
        (-40.0, (1, 0)),
        (-10.1, (1, 0)),
        (-10.0, (1, 1)),
        (0.0, (1, 1)),
        (10.0, (1, 1)),
        (10.1, (0, 1)),
        (40.0, (0, 1)),
    ],
)
def test_motor_selection_by_bearing(
    manager: AlertManager, bearing_deg: float, sides: tuple[int, int]
) -> None:
    outputs = run(manager, T0, [make_obstacle(1.25, bearing_deg)])
    assert motors(outputs) == pytest.approx((0.5 * sides[0], 0.5 * sides[1]))


def test_nearest_obstacle_drives_the_motors(manager: AlertManager) -> None:
    far_right = make_obstacle(1.5, 30.0)
    near_left = make_obstacle(1.25, -30.0)
    assert motors(run(manager, T0, [far_right, near_left])) == pytest.approx((0.5, 0.0))


def test_label_only_obstacles_do_not_vibrate(manager: AlertManager) -> None:
    outputs = run(manager, T0, [make_obstacle(None, label="person")])
    assert motors(outputs) == (0.0, 0.0)


def test_invalid_distance_does_not_hide_a_near_obstacle(manager: AlertManager) -> None:
    outputs = run(manager, T0, [make_obstacle(math.nan), make_obstacle(0.3, 30.0)])
    assert motors(outputs) == (0.0, 1.0)
    assert outputs.beeps == (Beep(2000.0, BEEP_DURATION_S, 30.0),)


@pytest.mark.parametrize("verbosity", list(Verbosity))
def test_haptics_work_at_every_verbosity(manager: AlertManager, verbosity: Verbosity) -> None:
    manager.verbosity = verbosity
    outputs = run(manager, T0, [make_obstacle(1.25, 20.0)])
    assert motors(outputs) == pytest.approx((0.0, 0.5))


# Ground hazards and haptic patterns


def test_drop_off_plays_its_pattern_and_interrupts_speech(manager: AlertManager) -> None:
    outputs = run(manager, T0, hazard=make_hazard(HazardType.DROP_OFF))
    assert outputs.haptic_pattern == "drop_off"
    assert outputs.speech == (Speech("Drop ahead.", interrupt=True),)


def test_step_up_plays_its_pattern_and_interrupts_speech(manager: AlertManager) -> None:
    outputs = run(manager, T0, hazard=make_hazard(HazardType.STEP_UP))
    assert outputs.haptic_pattern == "step_up"
    assert outputs.speech == (Speech("Step up ahead.", interrupt=True),)


def test_the_same_hazard_passed_again_is_not_new(manager: AlertManager) -> None:
    hazard = make_hazard(timestamp=T0)
    assert run(manager, T0, hazard=hazard).haptic_pattern == "drop_off"
    repeat = run(manager, T0 + 0.1, hazard=hazard)
    assert repeat.haptic_pattern is None
    assert repeat.speech == ()
    later = run(manager, T0 + 3.0, hazard=make_hazard(timestamp=T0 + 3.0))
    assert later.haptic_pattern == "drop_off"
    assert later.speech == (Speech("Drop ahead.", interrupt=True),)


def test_a_hazard_of_type_none_is_ignored(manager: AlertManager) -> None:
    outputs = run(manager, T0, hazard=GroundHazard(HazardType.NONE, 0.0, T0))
    assert outputs.haptic_pattern is None
    assert outputs.speech == ()


@pytest.mark.parametrize("verbosity", list(Verbosity))
def test_hazards_are_announced_at_every_verbosity(
    manager: AlertManager, verbosity: Verbosity
) -> None:
    manager.verbosity = verbosity
    outputs = run(manager, T0, hazard=make_hazard(HazardType.STEP_UP))
    assert outputs.haptic_pattern == "step_up"
    assert texts(outputs) == ["Step up ahead."]


def test_hazard_speech_skips_the_speech_gap(manager: AlertManager) -> None:
    assert texts(run(manager, T0, [make_obstacle(3.0, label="person")])) == [
        "person ahead, 3 meters."
    ]
    outputs = run(manager, T0 + 0.1, hazard=make_hazard())
    assert outputs.speech == (Speech("Drop ahead.", interrupt=True),)


def test_descriptions_wait_for_the_gap_after_a_hazard(manager: AlertManager) -> None:
    run(manager, T0, hazard=make_hazard())
    person = make_obstacle(3.0, label="person")
    assert run(manager, T0 + 1.0, [person]).speech == ()
    assert texts(run(manager, T0 + 1.5, [person])) == ["person ahead, 3 meters."]


def test_hazard_pattern_goes_before_an_event_pattern(manager: AlertManager) -> None:
    battery = make_event("battery", AlertLevel.WARNING, "Battery low.", "low_battery")
    outputs = run(manager, T0, hazard=make_hazard(HazardType.STEP_UP), events=[battery])
    assert outputs.haptic_pattern == "step_up"


def test_event_pattern_plays_without_a_hazard(manager: AlertManager) -> None:
    battery = make_event("battery", AlertLevel.WARNING, "Battery low.", "low_battery")
    assert run(manager, T0, events=[battery]).haptic_pattern == "low_battery"


def test_most_urgent_event_pattern_wins(manager: AlertManager) -> None:
    mode = make_event("mode", AlertLevel.INFO, "", "mode_change")
    fall = make_event("fall", AlertLevel.DANGER, FALL_MESSAGE, "fall_detected")
    assert run(manager, T0, events=[mode, fall]).haptic_pattern == "fall_detected"


def test_event_pattern_plays_even_while_its_speech_is_on_cooldown(manager: AlertManager) -> None:
    tap = make_event("action", AlertLevel.INFO, "Nothing ahead.", "confirm")
    assert texts(run(manager, T0, events=[tap])) == ["Nothing ahead."]
    outputs = run(manager, T0 + 2.0, events=[tap])
    assert outputs.haptic_pattern == "confirm"
    assert outputs.speech == ()


# Spatial audio


def test_warning_zone_beep(manager: AlertManager) -> None:
    outputs = run(manager, T0, [make_obstacle(0.8)])
    assert outputs.beeps == (Beep(1500.0, 0.06, 0.0),)


def test_danger_zone_beep(manager: AlertManager) -> None:
    outputs = run(manager, T0, [make_obstacle(0.3)])
    assert outputs.beeps == (Beep(2000.0, 0.06, 0.0),)


@pytest.mark.parametrize("distance_m", [1.0, 1.5, 3.0])
def test_no_beep_outside_the_warning_zone(manager: AlertManager, distance_m: float) -> None:
    assert run(manager, T0, [make_obstacle(distance_m)]).beeps == ()


def test_beeps_are_panned_by_bearing(manager: AlertManager) -> None:
    outputs = run(manager, T0, [make_obstacle(0.7, -30.0)])
    assert outputs.beeps == (Beep(1500.0, 0.06, -30.0),)


def test_the_nearest_obstacle_in_the_zone_beeps(manager: AlertManager) -> None:
    outputs = run(manager, T0, [make_obstacle(0.9, 30.0), make_obstacle(0.4, -40.0)])
    assert outputs.beeps == (Beep(2000.0, 0.06, -40.0),)


def test_warning_beeps_repeat_every_200_ms(manager: AlertManager) -> None:
    obstacle = make_obstacle(0.8)
    assert len(run(manager, T0, [obstacle]).beeps) == 1
    assert run(manager, T0 + 0.1, [obstacle]).beeps == ()
    assert run(manager, T0 + 0.19, [obstacle]).beeps == ()
    assert len(run(manager, T0 + 0.2, [obstacle]).beeps) == 1


def test_danger_beeps_repeat_every_80_ms(manager: AlertManager) -> None:
    obstacle = make_obstacle(0.3)
    assert len(run(manager, T0, [obstacle]).beeps) == 1
    assert run(manager, T0 + 0.05, [obstacle]).beeps == ()
    assert len(run(manager, T0 + 0.08, [obstacle]).beeps) == 1


@pytest.mark.parametrize("distance_m, expected_beeps", [(0.8, 6), (0.3, 11)])
def test_beeps_at_most_once_per_interval_in_a_30_hz_loop(
    manager: AlertManager, distance_m: float, expected_beeps: int
) -> None:
    obstacle = make_obstacle(distance_m)
    beeps = [run(manager, T0 + frame / 30.0, [obstacle]).beeps for frame in range(31)]
    assert sum(len(tick) for tick in beeps) == expected_beeps


def test_beep_interval_follows_the_current_zone(manager: AlertManager) -> None:
    assert run(manager, T0, [make_obstacle(0.8)]).beeps == (Beep(1500.0, 0.06, 0.0),)
    assert run(manager, T0 + 0.1, [make_obstacle(0.45)]).beeps == (Beep(2000.0, 0.06, 0.0),)


@pytest.mark.parametrize(
    "verbosity, beeps", [(Verbosity.FULL, 1), (Verbosity.BRIEF, 1), (Verbosity.HAPTIC_ONLY, 0)]
)
def test_beeps_by_verbosity(manager: AlertManager, verbosity: Verbosity, beeps: int) -> None:
    manager.verbosity = verbosity
    assert len(run(manager, T0, [make_obstacle(0.8)]).beeps) == beeps


# Warning-zone hysteresis


def test_warning_zone_hysteresis_keeps_beeping_until_past_the_margin(
    manager: AlertManager,
) -> None:
    assert run(manager, T0, [make_obstacle(1.05)]).beeps == ()
    assert run(manager, T0 + 1, [make_obstacle(0.95)]).beeps == (Beep(1500.0, 0.06, 0.0),)
    # 1.10 m is still inside warning_m + hysteresis_m; the tone comes from distance_to_beep.
    assert run(manager, T0 + 2, [make_obstacle(1.10)]).beeps == (Beep(1000.0, 0.06, 0.0),)
    assert run(manager, T0 + 3, [make_obstacle(1.16)]).beeps == ()
    assert run(manager, T0 + 4, [make_obstacle(1.10)]).beeps == ()
    assert run(manager, T0 + 5, [make_obstacle(0.99)]).beeps == (Beep(1500.0, 0.06, 0.0),)


def test_hysteresis_follows_each_obstacle(manager: AlertManager) -> None:
    run(manager, T0, [make_obstacle(0.9, 0.0)])
    entered = make_obstacle(1.1, 0.0)
    never_entered = make_obstacle(1.05, 30.0)
    outputs = run(manager, T0 + 1, [entered, never_entered])
    assert outputs.beeps == (Beep(1000.0, 0.06, 0.0),)


def test_an_obstacle_that_disappears_leaves_the_zone(manager: AlertManager) -> None:
    run(manager, T0, [make_obstacle(0.9)])
    run(manager, T0 + 1)
    assert run(manager, T0 + 2, [make_obstacle(1.1)]).beeps == ()


def test_hysteresis_applies_to_brief_descriptions(manager: AlertManager) -> None:
    manager.verbosity = Verbosity.BRIEF
    assert run(manager, T0, [make_obstacle(1.1, label="person")]).speech == ()
    entered = run(manager, T0 + 1, [make_obstacle(0.9, label="person")])
    assert texts(entered) == ["person ahead, very close."]
    held = run(manager, T0 + 7, [make_obstacle(1.1, label="person")])
    assert texts(held) == ["person ahead, 1 meter."]
    left = run(manager, T0 + 14, [make_obstacle(1.2, label="person")])
    assert left.speech == ()


# Obstacle descriptions


def test_labelled_obstacle_is_described(manager: AlertManager) -> None:
    outputs = run(manager, T0, [make_obstacle(2.0, -20.0, label="person")])
    assert outputs.speech == (Speech("person slightly left, 2 meters.", interrupt=False),)


def test_label_only_obstacle_omits_the_distance(manager: AlertManager) -> None:
    outputs = run(manager, T0, [make_obstacle(None, 50.0, label="dog")])
    assert outputs.speech == (Speech("dog on your right."),)


@pytest.mark.parametrize(
    "distance_m, phrase",
    [
        (0.7, "person ahead, very close."),
        (1.2, "person ahead, 1 meter."),
        (4.6, "person ahead, 5 meters."),
    ],
)
def test_description_distance_wording(
    manager: AlertManager, distance_m: float, phrase: str
) -> None:
    assert texts(run(manager, T0, [make_obstacle(distance_m, label="person")])) == [phrase]


@pytest.mark.parametrize(
    "distance_m, spoken", [(4.9, True), (5.0, True), (5.01, False), (7.0, False)]
)
def test_labels_are_announced_within_label_announce_m(
    manager: AlertManager, distance_m: float, spoken: bool
) -> None:
    outputs = run(manager, T0, [make_obstacle(distance_m, label="car")])
    assert texts(outputs) == (["car ahead, 5 meters."] if spoken else [])


def test_unlabelled_obstacles_are_not_described(manager: AlertManager) -> None:
    assert run(manager, T0, [make_obstacle(3.0)]).speech == ()
    assert run(manager, T0 + 2, [make_obstacle(3.0, label="")]).speech == ()


def test_a_key_repeats_only_after_the_cooldown(manager: AlertManager) -> None:
    person = make_obstacle(3.0, label="person")
    assert texts(run(manager, T0, [person])) == ["person ahead, 3 meters."]
    assert run(manager, T0 + 3.0, [person]).speech == ()
    assert run(manager, T0 + 5.9, [person]).speech == ()
    assert texts(run(manager, T0 + 6.0, [person])) == ["person ahead, 3 meters."]


def test_a_key_repeats_early_when_more_than_30_percent_closer(manager: AlertManager) -> None:
    def person(distance_m: float) -> Obstacle:
        return make_obstacle(distance_m, label="person")

    assert texts(run(manager, T0, [person(2.0)])) == ["person ahead, 2 meters."]
    assert run(manager, T0 + 1.5, [person(1.5)]).speech == ()
    assert run(manager, T0 + 2.0, [person(1.4)]).speech == ()
    assert texts(run(manager, T0 + 3.0, [person(1.38)])) == ["person ahead, 1 meter."]
    # The reference distance is now 1.38 m.
    assert run(manager, T0 + 4.5, [person(1.0)]).speech == ()
    assert texts(run(manager, T0 + 6.0, [person(0.9)])) == ["person ahead, very close."]


def test_label_only_keys_repeat_only_after_the_cooldown(manager: AlertManager) -> None:
    dog = make_obstacle(None, -50.0, label="dog")
    assert texts(run(manager, T0, [dog])) == ["dog on your left."]
    assert run(manager, T0 + 3.0, [make_obstacle(2.0, -50.0, label="dog")]).speech == ()
    assert texts(run(manager, T0 + 6.0, [dog])) == ["dog on your left."]


def test_each_direction_is_its_own_key(manager: AlertManager) -> None:
    run(manager, T0, [make_obstacle(3.0, 0.0, label="person")])
    outputs = run(manager, T0 + 2.0, [make_obstacle(3.0, -20.0, label="person")])
    assert texts(outputs) == ["person slightly left, 3 meters."]


def test_speech_items_are_spaced_by_min_speech_gap(manager: AlertManager) -> None:
    person = make_obstacle(3.0, 0.0, label="person")
    dog = make_obstacle(4.0, 40.0, label="dog")
    assert texts(run(manager, T0, [person, dog])) == ["person ahead, 3 meters."]
    assert run(manager, T0 + 1.0, [person, dog]).speech == ()
    assert run(manager, T0 + 1.4, [person, dog]).speech == ()
    assert texts(run(manager, T0 + 1.5, [person, dog])) == ["dog on your right, 4 meters."]


def test_nearest_labelled_obstacle_is_described_first(manager: AlertManager) -> None:
    obstacles = [
        make_obstacle(4.0, 40.0, label="dog"),
        make_obstacle(None, 0.0, label="bench"),
        make_obstacle(2.0, -20.0, label="person"),
    ]
    assert texts(run(manager, T0, obstacles)) == ["person slightly left, 2 meters."]
    assert texts(run(manager, T0 + 1.5, obstacles)) == ["dog on your right, 4 meters."]
    assert texts(run(manager, T0 + 3.0, obstacles)) == ["bench ahead."]


def test_descriptions_are_held_back_inside_danger(manager: AlertManager) -> None:
    person = make_obstacle(2.0, -20.0, label="person")
    wall = make_obstacle(0.4, 0.0)
    held = run(manager, T0, [wall, person])
    assert held.speech == ()
    assert held.beeps == (Beep(2000.0, 0.06, 0.0),)
    assert motors(held) == (1.0, 1.0)
    released = run(manager, T0 + 0.1, [person])
    assert texts(released) == ["person slightly left, 2 meters."]


def test_a_labelled_obstacle_inside_danger_is_not_described(manager: AlertManager) -> None:
    assert run(manager, T0, [make_obstacle(0.45, label="person")]).speech == ()


def test_danger_hold_back_does_not_block_hazards_or_events(manager: AlertManager) -> None:
    wall = make_obstacle(0.3)
    outputs = run(manager, T0, [wall], hazard=make_hazard(), events=[make_event("gps")])
    assert outputs.speech == (Speech("Drop ahead.", interrupt=True),)
    assert texts(run(manager, T0 + 1.5, [wall])) == ["GPS signal lost."]


# System events


def test_event_message_is_spoken(manager: AlertManager) -> None:
    outputs = run(manager, T0, events=[make_event("gps")])
    assert outputs.speech == (Speech("GPS signal lost.", interrupt=False),)


def test_event_without_a_message_is_haptic_only(manager: AlertManager) -> None:
    outputs = run(manager, T0, events=[make_event("tap", message="", pattern="confirm")])
    assert outputs.speech == ()
    assert outputs.haptic_pattern == "confirm"


def test_danger_events_interrupt_and_always_pass(manager: AlertManager) -> None:
    assert texts(run(manager, T0, [make_obstacle(3.0, label="person")]))
    fall = make_event("sos", AlertLevel.DANGER, FALL_MESSAGE, "sos_countdown")
    outputs = run(manager, T0 + 0.1, events=[fall])
    assert outputs.speech == (Speech(FALL_MESSAGE, interrupt=True),)
    assert outputs.haptic_pattern == "sos_countdown"
    again = run(manager, T0 + 0.2, events=[fall])
    assert again.speech == (Speech(FALL_MESSAGE, interrupt=True),)


def test_danger_events_do_not_start_a_cooldown(manager: AlertManager) -> None:
    run(manager, T0, events=[make_event("sos", AlertLevel.DANGER, "Sending SOS in 10 seconds.")])
    cancelled = run(manager, T0 + 2.0, events=[make_event("sos", message="SOS cancelled.")])
    assert texts(cancelled) == ["SOS cancelled."]


def test_events_share_the_per_key_cooldown(manager: AlertManager) -> None:
    low = make_event("battery", AlertLevel.WARNING, "Battery low.")
    assert texts(run(manager, T0, events=[low])) == ["Battery low."]
    assert run(manager, T0 + 3.0, events=[low]).speech == ()
    assert run(manager, T0 + 4.5).speech == ()
    assert texts(run(manager, T0 + 6.0, events=[low])) == ["Battery low."]


def test_an_event_waits_for_the_speech_gap(manager: AlertManager) -> None:
    run(manager, T0, [make_obstacle(3.0, label="person")])
    assert run(manager, T0 + 0.5, events=[make_event("gps")]).speech == ()
    assert run(manager, T0 + 1.0).speech == ()
    assert texts(run(manager, T0 + 1.5)) == ["GPS signal lost."]


def test_waiting_events_go_before_descriptions(manager: AlertManager) -> None:
    person = make_obstacle(3.0, label="person")
    assert texts(run(manager, T0, events=[make_event("a", message="First.")])) == ["First."]
    second = run(manager, T0 + 0.5, [person], events=[make_event("b", message="Second.")])
    assert second.speech == ()
    assert texts(run(manager, T0 + 1.5, [person])) == ["Second."]
    assert texts(run(manager, T0 + 3.0, [person])) == ["person ahead, 3 meters."]


def test_waiting_events_are_spoken_most_urgent_first(manager: AlertManager) -> None:
    run(manager, T0, events=[make_event("first", message="First.")])
    run(manager, T0 + 0.3, events=[make_event("info", AlertLevel.INFO, "Info.")])
    run(manager, T0 + 0.6, events=[make_event("warn", AlertLevel.WARNING, "Warning.")])
    assert texts(run(manager, T0 + 1.5)) == ["Warning."]
    assert texts(run(manager, T0 + 3.0)) == ["Info."]


def test_a_waiting_event_key_is_not_queued_twice(manager: AlertManager) -> None:
    run(manager, T0, events=[make_event("first", message="First.")])
    run(manager, T0 + 0.3, events=[make_event("gps")])
    run(manager, T0 + 0.6, events=[make_event("gps")])
    assert texts(run(manager, T0 + 1.5)) == ["GPS signal lost."]
    assert run(manager, T0 + 3.0).speech == ()


def test_the_waiting_queue_is_bounded(
    manager: AlertManager, caplog: pytest.LogCaptureFixture
) -> None:
    run(manager, T0, events=[make_event("first", message="First.")])
    infos = [
        make_event(f"info{i}", AlertLevel.INFO, f"Info {i}.") for i in range(MAX_PENDING_EVENTS)
    ]
    warning = make_event("warn", AlertLevel.WARNING, "Warning.")
    with caplog.at_level(logging.WARNING, logger="navsight.alerts.priority"):
        run(manager, T0 + 0.5, events=[*infos, warning])
    assert "Info 0." in caplog.text
    spoken = [texts(run(manager, T0 + 1.5 * step)) for step in range(1, MAX_PENDING_EVENTS + 2)]
    expected = [["Warning."]] + [[f"Info {i}."] for i in range(1, MAX_PENDING_EVENTS)] + [[]]
    assert spoken == expected


def test_only_the_first_urgent_item_interrupts(manager: AlertManager) -> None:
    fall = make_event("fall", AlertLevel.DANGER, FALL_MESSAGE)
    outputs = run(manager, T0, hazard=make_hazard(), events=[fall])
    assert outputs.speech == (
        Speech("Drop ahead.", interrupt=True),
        Speech(FALL_MESSAGE, interrupt=False),
    )


def test_duplicate_keys_in_one_update_are_spoken_once(manager: AlertManager) -> None:
    weak = make_event("gps", AlertLevel.INFO, "GPS weak.")
    lost = make_event("gps", AlertLevel.WARNING, "GPS lost.")
    assert texts(run(manager, T0, events=[weak, lost])) == ["GPS lost."]
    assert run(manager, T0 + 1.5).speech == ()
    sos = make_event("sos", AlertLevel.DANGER, "SOS.")
    assert run(manager, T0 + 3.0, events=[sos, sos]).speech == (Speech("SOS.", interrupt=True),)


# Verbosity


def test_initial_verbosity_comes_from_the_config() -> None:
    assert AlertManager(NavSightConfig()).verbosity is Verbosity.FULL
    config = NavSightConfig()
    config.alerts.verbosity = "haptic_only"
    assert AlertManager(config).verbosity is Verbosity.HAPTIC_ONLY


def test_verbosity_is_settable(manager: AlertManager) -> None:
    manager.verbosity = Verbosity.BRIEF
    assert manager.verbosity is Verbosity.BRIEF
    manager.verbosity = "haptic_only"
    assert manager.verbosity is Verbosity.HAPTIC_ONLY
    with pytest.raises(ValueError, match="verbosity must be one of"):
        manager.verbosity = "loud"
    assert manager.verbosity is Verbosity.HAPTIC_ONLY


def test_invalid_config_verbosity_is_rejected(config: NavSightConfig) -> None:
    config.alerts.verbosity = "chatty"
    with pytest.raises(ValueError, match="chatty"):
        AlertManager(config)


@pytest.mark.parametrize(
    "verbosity, distance_m, spoken",
    [
        (Verbosity.FULL, 3.0, True),
        (Verbosity.FULL, 0.8, True),
        (Verbosity.FULL, None, True),
        (Verbosity.BRIEF, 3.0, False),
        (Verbosity.BRIEF, 0.8, True),
        (Verbosity.BRIEF, None, False),
        (Verbosity.HAPTIC_ONLY, 3.0, False),
        (Verbosity.HAPTIC_ONLY, 0.8, False),
        (Verbosity.HAPTIC_ONLY, None, False),
    ],
)
def test_descriptions_by_verbosity(
    manager: AlertManager, verbosity: Verbosity, distance_m: float | None, spoken: bool
) -> None:
    manager.verbosity = verbosity
    outputs = run(manager, T0, [make_obstacle(distance_m, label="person")])
    assert bool(outputs.speech) is spoken


@pytest.mark.parametrize(
    "verbosity, level, spoken",
    [
        (Verbosity.FULL, AlertLevel.INFO, True),
        (Verbosity.BRIEF, AlertLevel.INFO, True),
        (Verbosity.BRIEF, AlertLevel.CAUTION, True),
        (Verbosity.HAPTIC_ONLY, AlertLevel.INFO, False),
        (Verbosity.HAPTIC_ONLY, AlertLevel.CAUTION, False),
        (Verbosity.HAPTIC_ONLY, AlertLevel.WARNING, True),
        (Verbosity.HAPTIC_ONLY, AlertLevel.DANGER, True),
    ],
)
def test_event_speech_by_verbosity(
    manager: AlertManager, verbosity: Verbosity, level: AlertLevel, spoken: bool
) -> None:
    manager.verbosity = verbosity
    outputs = run(manager, T0, events=[make_event("event", level, "Message.")])
    assert texts(outputs) == (["Message."] if spoken else [])


def test_haptic_only_keeps_vibration_patterns_and_urgent_speech(manager: AlertManager) -> None:
    manager.verbosity = Verbosity.HAPTIC_ONLY
    person = run(manager, T0, [make_obstacle(0.8, -30.0, label="person")])
    assert person.beeps == ()
    assert person.speech == ()
    assert motors(person) == pytest.approx((0.8, 0.0))
    mode_event = make_event("mode", message="Vibration only.", pattern="mode_change")
    mode = run(manager, T0 + 2.0, events=[mode_event])
    assert mode.haptic_pattern == "mode_change"
    assert mode.speech == ()
    assert run(manager, T0 + 4.0).speech == ()


def test_waiting_event_is_dropped_when_verbosity_no_longer_allows_it(
    manager: AlertManager,
) -> None:
    run(manager, T0, events=[make_event("first", message="First.")])
    run(manager, T0 + 0.5, events=[make_event("gps")])
    manager.verbosity = Verbosity.HAPTIC_ONLY
    assert run(manager, T0 + 1.5).speech == ()
    manager.verbosity = Verbosity.FULL
    assert run(manager, T0 + 3.0).speech == ()


# Output shape


def test_outputs_hold_tuples(manager: AlertManager) -> None:
    outputs = run(manager, T0, [make_obstacle(0.8, label="person")], events=[make_event("gps")])
    assert isinstance(outputs.beeps, tuple)
    assert isinstance(outputs.speech, tuple)
    assert len(outputs.beeps) == 1
    assert texts(outputs) == ["GPS signal lost."]
    quiet = run(manager, T0 + 0.01)
    assert quiet.beeps == ()
    assert quiet.speech == ()
