from __future__ import annotations

import json
from pathlib import Path

import pytest

from navsight import __version__
from navsight.__main__ import main


def test_print_config(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--print-config"]) == 0
    data = json.loads(capsys.readouterr().out)
    assert data["i2c"]["tof_down"] == 0x30
    assert data["pins"]["button_sos"] == 27


def test_version(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as exit_info:
        main(["--version"])
    assert exit_info.value.code == 0
    assert f"navsight {__version__}" in capsys.readouterr().out


def test_battery_full_resets_the_charge_counter(tmp_path: Path) -> None:
    state = tmp_path / "battery.json"
    state.write_text(json.dumps({"consumed_mah": 1234.0}))
    config = tmp_path / "navsight.json"
    config.write_text(json.dumps({"power": {"state_file": str(state)}}))
    assert main(["--config", str(config), "--battery-full"]) == 0
    assert json.loads(state.read_text())["consumed_mah"] == 0


def test_a_bad_config_file_exits_with_a_message(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    config = tmp_path / "navsight.json"
    config.write_text(json.dumps({"alerts": {"volume": 1}}))
    assert main(["--config", str(config)]) == 2
    assert "Unknown config key: alerts.volume" in capsys.readouterr().err


def test_a_missing_config_file_exits_with_a_message(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main(["--config", str(tmp_path / "missing.json")]) == 2
    assert "missing.json" in capsys.readouterr().err


def test_an_unknown_log_level_is_rejected(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--log-level", "LOUD", "--print-config"]) == 2
    assert "LOUD" in capsys.readouterr().err


def test_the_simulator_runs_from_the_command_line(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--sim", "--speed", "0", "--duration", "3", "--log-level", "ERROR"]) == 0
    assert "NavSight ready." in capsys.readouterr().out
