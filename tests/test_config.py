from __future__ import annotations

import json

import pytest

from navsight.config import NavSightConfig, dump_config, load_config


def test_defaults_match_the_v2_design() -> None:
    config = load_config()
    assert config.i2c.tof_forward == 0x29
    assert config.i2c.tof_down == 0x30
    assert config.i2c.imu == 0x4A
    assert config.i2c.power_monitor == 0x40
    assert config.pins.tof_down_xshut == 25
    assert (config.pins.haptic_left, config.pins.haptic_right) == (12, 13)
    assert (config.pins.button_mode, config.pins.button_action, config.pins.button_sos) == (
        17,
        22,
        27,
    )


def test_json_file_overrides_defaults(tmp_path) -> None:
    path = tmp_path / "navsight.json"
    path.write_text(
        json.dumps({"alerts": {"verbosity": "brief"}, "tof": {"rate_hz": 40}}), encoding="utf-8"
    )
    config = load_config(path)
    assert config.alerts.verbosity == "brief"
    assert config.tof.rate_hz == 40.0
    assert isinstance(config.tof.rate_hz, float)


def test_list_values_become_tuples() -> None:
    config = load_config(overrides={"detector": {"classes": ["person", "dog"]}})
    assert config.detector.classes == ("person", "dog")


def test_optional_fields_accept_values_and_null() -> None:
    config = load_config(overrides={"sos": {"webhook_url": "https://example.test/hook"}})
    assert config.sos.webhook_url == "https://example.test/hook"
    with pytest.raises(ValueError, match="webhook_url"):
        load_config(overrides={"sos": {"webhook_url": 5}})


@pytest.mark.parametrize(
    "overrides, message",
    [
        ({"alerts": {"volume": 3}}, "Unknown config key: alerts.volume"),
        ({"tof": {"rate_hz": "fast"}}, "tof.rate_hz must be float"),
        ({"gps": {"use_ubx_power_save": 1}}, "true or false"),
        ({"alerts": {"danger_m": 3.0}}, "danger_m < warning_m < caution_m"),
        ({"pins": {"button_mode": 14}}, "GPS UART"),
        ({"i2c": {"imu": 0x29}}, "every address must be different"),
        ({"fusion": {"sector_count": 6}}, "sector_count"),
        ({"tof": {"timing_budget_ms": 15}}, "15 ms timing budget"),
    ],
)
def test_invalid_values_are_rejected(overrides, message) -> None:
    with pytest.raises(ValueError, match=message):
        load_config(overrides=overrides)


def test_dump_round_trips_through_json() -> None:
    data = json.loads(dump_config(NavSightConfig()))
    data.pop("simulate")
    assert load_config(overrides=data) == NavSightConfig()
