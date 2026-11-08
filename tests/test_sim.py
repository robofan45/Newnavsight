"""The scripted simulator run end to end, as a whole-system test."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from navsight.config import NavSightConfig, load_config
from navsight.sim import SimulationResult, run_simulation, simulation_config


@pytest.fixture(scope="module")
def result(tmp_path_factory: pytest.TempPathFactory) -> SimulationResult:
    work_dir = tmp_path_factory.mktemp("sim")
    return run_simulation(NavSightConfig(), speed=0, output=None, work_dir=work_dir)


def _times(result: SimulationResult, text: str) -> list[float]:
    return [t for t, spoken in result.spoken if spoken == text]


def _spoken(result: SimulationResult) -> list[str]:
    return [text for _, text in result.spoken]


def test_starts_with_the_ready_message(result: SimulationResult) -> None:
    assert _spoken(result)[0] == "NavSight ready."


def test_the_pole_ahead_vibrates_and_beeps(result: SimulationResult) -> None:
    peaks = [max(left, right) for t, left, right in result.haptic_peaks if 2.0 <= t < 6.0]
    assert peaks and max(peaks) >= 0.8
    assert result.beeps > 10


def test_the_person_is_described_on_the_left(result: SimulationResult) -> None:
    assert any(text.startswith("person slightly left, ") for text in _spoken(result))


def test_only_the_curb_is_a_drop_never_the_fallen_cane(result: SimulationResult) -> None:
    drops = _times(result, "Drop ahead.")
    assert drops and all(12.0 <= t < 13.0 for t in drops)
    assert any(pattern == "drop_off" for _, pattern in result.patterns)


def test_gps_loss_indoors_is_announced_once_each_way(result: SimulationResult) -> None:
    lost = _times(result, "GPS lost. Using step tracking.")
    back = _times(result, "GPS signal back.")
    assert len(lost) == 1 and 24.0 <= lost[0] < 26.0
    assert len(back) == 1 and 32.0 <= back[0] < 33.0


def test_mode_and_action_buttons_answer(result: SimulationResult) -> None:
    spoken = _spoken(result)
    for text in ("Brief alerts.", "Vibration only.", "Full alerts.", "Clear ahead. Facing east."):
        assert text in spoken
    assert any(text.startswith("Battery 2") and "GPS good." in text for text in spoken)


def test_low_battery_switches_to_low_power(result: SimulationResult) -> None:
    assert any(text.startswith("Battery low, ") for text in _spoken(result))
    assert "Low power mode." in _spoken(result)
    assert [state for _, state in result.power_states] == ["active", "low_power"]


def test_the_fall_runs_the_countdown_and_records_the_sos(result: SimulationResult) -> None:
    spoken = _spoken(result)
    countdown = [text for text in spoken if text.startswith("Fall detected. Sending SOS in 15")]
    assert len(countdown) == 1
    assert "Sending SOS in 10 seconds." in spoken
    assert "SOS recorded. No emergency contact is set up." in spoken
    lines = result.sos_log.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    assert json.loads(lines[0])["reason"] == "fall"


def test_runs_are_identical(tmp_path: Path, result: SimulationResult) -> None:
    again = run_simulation(NavSightConfig(), speed=0, output=None, work_dir=tmp_path)
    assert again.spoken == result.spoken
    assert again.patterns == result.patterns


def test_the_simulator_never_contacts_anyone(tmp_path: Path) -> None:
    config = load_config(
        overrides={
            "sos": {
                "webhook_url": "https://example.test/hook",
                "telegram_bot_token": "secret",
                "telegram_chat_id": "42",
            }
        }
    )
    safe = simulation_config(config, tmp_path)
    assert safe.sos.webhook_url is None
    assert safe.sos.telegram_bot_token is None
    assert safe.sos.telegram_chat_id is None
    assert Path(safe.sos.log_file).parent == tmp_path
    assert Path(safe.power.state_file).parent == tmp_path
