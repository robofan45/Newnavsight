from __future__ import annotations

import io
import json
import logging
import threading
import urllib.error
import urllib.request
from collections.abc import Callable, Sequence
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import Any

import pytest

from navsight.config import NavSightConfig, SosConfig
from navsight.safety.sos import (
    Job,
    LogNotifier,
    Notifier,
    NotifierError,
    Runner,
    SosController,
    SosMessage,
    SosState,
    TelegramNotifier,
    WebhookNotifier,
    format_sos_text,
    google_maps_url,
    make_notifiers,
)
from navsight.types import Alert, AlertLevel
from navsight.util import ManualClock

FIXED_UTC = datetime(2026, 11, 8, 12, 30, 5, 250000, tzinfo=UTC)
FIXED_ISO = "2026-11-08T12:30:05+00:00"
TOKEN = "123456789:AAH-secret-token-value"
WEBHOOK_URL = "https://hooks.example.test/trigger/sos/with/key/K3Y-s3cret"
TELEGRAM_OK = b'{"ok": true, "result": {"message_id": 7}}'
START_TEXT = "Fall detected. Sending SOS in 15 seconds. Press any button to cancel."
FAILED_TEXT = "SOS failed. Hold the SOS button to try again."
MESSAGE = SosMessage("fall", 51.5014, -0.1419, FIXED_ISO, 42)
MESSAGE_FIELDS = {
    "reason": "fall",
    "latitude": 51.5014,
    "longitude": -0.1419,
    "timestamp_utc": FIXED_ISO,
    "battery_percent": 42,
    "maps_url": "https://maps.google.com/?q=51.501400,-0.141900",
}

UrlopenCalls = list[tuple[urllib.request.Request, Any]]


def run_now(job: Job) -> None:
    job()


class FakeNotifier:
    """Records each attempt; the first ``failures`` attempts raise."""

    def __init__(
        self, clock: ManualClock, name: str = "contact", remote: bool = True, failures: int = 0
    ) -> None:
        self.name = name
        self.remote = remote
        self.failures = failures
        self.clock = clock
        self.attempts: list[float] = []
        self.messages: list[SosMessage] = []

    def send(self, message: SosMessage) -> None:
        self.attempts.append(self.clock())
        if self.failures > 0:
            self.failures -= 1
            raise ConnectionError(f"{self.name} is unreachable")
        self.messages.append(message)


class FakeResponse:
    def __init__(self, body: bytes) -> None:
        self.body = body

    def read(self) -> bytes:
        return self.body

    def __enter__(self) -> FakeResponse:
        return self

    def __exit__(self, *exc_info: object) -> None:
        return None


def make_controller(
    config: NavSightConfig | SosConfig,
    clock: ManualClock,
    notifiers: Sequence[Notifier],
    runner: Runner = run_now,
) -> SosController:
    return SosController(config, notifiers, clock=clock, runner=runner, utc_now=lambda: FIXED_UTC)


def spoken(alerts: list[Alert]) -> list[str]:
    return [alert.message for alert in alerts]


def install_urlopen(monkeypatch: pytest.MonkeyPatch, reply: bytes = TELEGRAM_OK) -> UrlopenCalls:
    calls: UrlopenCalls = []

    def fake_urlopen(request: urllib.request.Request, timeout: Any = None) -> FakeResponse:
        calls.append((request, timeout))
        return FakeResponse(reply)

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    return calls


def install_failing_urlopen(
    monkeypatch: pytest.MonkeyPatch, make_error: Callable[[urllib.request.Request], Exception]
) -> None:
    def fake_urlopen(request: urllib.request.Request, timeout: Any = None) -> FakeResponse:
        raise make_error(request)

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)


def http_error(request: urllib.request.Request, code: int, reason: str, body: bytes) -> Exception:
    return urllib.error.HTTPError(request.full_url, code, reason, None, io.BytesIO(body))  # type: ignore[arg-type]


# --- message formatting ---------------------------------------------------------------------


@pytest.mark.parametrize(
    "latitude, longitude, expected",
    [
        (51.5, -0.12, "https://maps.google.com/?q=51.500000,-0.120000"),
        (12.3456789, 98.7654321, "https://maps.google.com/?q=12.345679,98.765432"),
        (-33.8688, 151.2093, "https://maps.google.com/?q=-33.868800,151.209300"),
        (0.0, 0.0, "https://maps.google.com/?q=0.000000,0.000000"),
    ],
)
def test_google_maps_url_uses_six_decimals(latitude, longitude, expected) -> None:
    assert google_maps_url(latitude, longitude) == expected


def test_maps_url_needs_both_coordinates() -> None:
    assert MESSAGE.maps_url == "https://maps.google.com/?q=51.501400,-0.141900"
    assert SosMessage("fall", None, -0.1, FIXED_ISO, 50).maps_url is None
    assert SosMessage("fall", 51.5, None, FIXED_ISO, 50).maps_url is None


def test_format_sos_text_for_a_fall() -> None:
    assert format_sos_text(MESSAGE) == (
        "NavSight SOS: a fall was detected.\n"
        "Location: https://maps.google.com/?q=51.501400,-0.141900\n"
        "Battery: 42%\n"
        f"Time (UTC): {FIXED_ISO}"
    )


def test_format_sos_text_for_a_manual_sos_without_gps_or_battery() -> None:
    text = format_sos_text(SosMessage("manual", None, None, FIXED_ISO, None))
    assert text.startswith("NavSight SOS: the SOS button was held.")
    assert "Location: unknown (no GPS fix)" in text
    assert "Battery: unknown" in text


# --- countdown ------------------------------------------------------------------------------


def test_countdown_announces_at_start_and_every_interval(config, clock) -> None:
    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [contact])
    sos.trigger_fall(clock())
    assert sos.state is SosState.COUNTDOWN

    heard = []
    for _ in range(61):  # 0 to 15 s in 0.25 s steps
        heard += [(clock() - 1000.0, alert) for alert in sos.tick(clock(), None, None)]
        clock.advance(0.25)

    assert [(elapsed, alert.message) for elapsed, alert in heard] == [
        (0.0, START_TEXT),
        (5.0, "Sending SOS in 10 seconds."),
        (10.0, "Sending SOS in 5 seconds."),
        (15.0, "Sending SOS."),
        (15.0, "SOS sent."),
    ]
    countdown = [alert for _, alert in heard[:3]]
    assert all(alert.level is AlertLevel.DANGER for alert in countdown)
    assert all(alert.haptic_pattern == "sos_countdown" for alert in countdown)
    assert [alert.timestamp for _, alert in heard[:3]] == [1000.0, 1005.0, 1010.0]
    assert len(contact.messages) == 1


def test_countdown_at_core_loop_rate_announces_each_point_once(config, clock) -> None:
    sos = make_controller(config, clock, [FakeNotifier(clock)])
    sos.trigger_fall(clock())
    heard = []
    for _ in range(14 * 30):  # 14 s at 30 Hz
        heard += spoken(sos.tick(clock(), None, None))
        clock.advance(1 / 30)
    assert heard == [START_TEXT, "Sending SOS in 10 seconds.", "Sending SOS in 5 seconds."]
    assert sos.state is SosState.COUNTDOWN


def test_countdown_follows_configured_lengths(config, clock) -> None:
    config.sos.fall_countdown_s = 6.0
    config.sos.countdown_announce_every_s = 5.0
    sos = make_controller(config, clock, [FakeNotifier(clock)])
    sos.trigger_fall(clock())
    first = spoken(sos.tick(clock(), None, None))
    clock.advance(5.0)
    second = spoken(sos.tick(clock(), None, None))
    assert first == ["Fall detected. Sending SOS in 6 seconds. Press any button to cancel."]
    assert second == ["Sending SOS in 1 second."]


def test_countdown_after_a_long_gap_announces_only_the_latest_point(config, clock) -> None:
    sos = make_controller(config, clock, [FakeNotifier(clock)])
    sos.trigger_fall(clock())
    clock.advance(6.0)  # the first tick comes late
    assert spoken(sos.tick(clock(), None, None)) == [
        "Fall detected. Sending SOS in 10 seconds. Press any button to cancel."
    ]
    clock.advance(5.0)
    assert spoken(sos.tick(clock(), None, None)) == ["Sending SOS in 5 seconds."]
    clock.advance(3.5)
    assert sos.tick(clock(), None, None) == []


def test_announce_interval_of_zero_gives_only_the_start_announcement(config, clock) -> None:
    config.sos.countdown_announce_every_s = 0.0
    sos = make_controller(config, clock, [FakeNotifier(clock)])
    sos.trigger_fall(clock())
    heard = []
    for _ in range(14):
        heard += spoken(sos.tick(clock(), None, None))
        clock.advance(1.0)
    assert heard == [START_TEXT]


def test_seconds_remaining(config, clock) -> None:
    sos = make_controller(config, clock, [FakeNotifier(clock)])
    assert sos.seconds_remaining is None
    sos.trigger_fall(clock())
    clock.advance(4.0)
    assert sos.seconds_remaining == pytest.approx(11.0)
    clock.advance(20.0)
    assert sos.seconds_remaining == 0.0  # the countdown ends at the next tick
    sos.tick(clock(), None, None)
    assert sos.seconds_remaining is None


def test_second_fall_during_countdown_keeps_the_original_deadline(config, clock) -> None:
    sos = make_controller(config, clock, [FakeNotifier(clock)])
    sos.trigger_fall(clock())
    clock.advance(5.0)
    sos.trigger_fall(clock())
    assert sos.seconds_remaining == pytest.approx(10.0)


def test_cancel_during_countdown(config, clock) -> None:
    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [contact])
    sos.trigger_fall(clock())
    sos.tick(clock(), None, None)
    clock.advance(3.0)

    assert sos.cancel(clock()) is True

    assert sos.state is SosState.IDLE
    alerts = sos.tick(clock(), None, None)
    assert spoken(alerts) == ["SOS cancelled."]
    assert alerts[0].level is AlertLevel.DANGER
    assert alerts[0].haptic_pattern == "confirm"
    clock.advance(30.0)
    assert sos.tick(clock(), None, None) == []
    assert contact.attempts == []
    assert sos.cancel(clock()) is False


def test_cancel_when_idle_returns_false(config, clock) -> None:
    sos = make_controller(config, clock, [FakeNotifier(clock)])
    assert sos.cancel(clock()) is False
    assert sos.tick(clock(), None, None) == []
    assert sos.state is SosState.IDLE


def test_cancel_after_the_sos_was_sent_returns_false(config, clock) -> None:
    sos = make_controller(config, clock, [FakeNotifier(clock)])
    sos.trigger_manual(clock())
    assert sos.state is SosState.SENT
    assert sos.cancel(clock()) is False
    assert sos.state is SosState.SENT


# --- sending --------------------------------------------------------------------------------


def test_countdown_expiry_sends_latest_location_and_battery(config, clock) -> None:
    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [contact])
    sos.trigger_fall(clock())
    sos.tick(clock(), (48.8584, 2.2945), 0.95)
    clock.advance(10.0)
    sos.tick(clock(), (51.5007292, -0.1246254), 0.734)
    clock.advance(4.5)
    sos.tick(clock(), (51.5010, -0.1300), 0.5)
    assert contact.messages == []

    clock.advance(0.5)
    alerts = sos.tick(clock(), (51.5014, -0.1419), 0.421)

    assert contact.messages == [MESSAGE]
    assert spoken(alerts) == ["Sending SOS.", "SOS sent."]
    assert alerts[1].haptic_pattern == "sos_sent"
    assert sos.state is SosState.SENT


def test_missing_location_or_battery_keeps_the_last_known_value(config, clock) -> None:
    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [contact])
    sos.tick(clock(), (40.4168, -3.7038), 0.5)
    clock.advance(1.0)
    sos.tick(clock(), None, None)
    sos.trigger_manual(clock())
    message = contact.messages[0]
    assert (message.latitude, message.longitude, message.battery_percent) == (40.4168, -3.7038, 50)


def test_sos_without_any_location_is_still_sent(config, clock) -> None:
    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [contact])
    sos.trigger_manual(clock())
    [message] = contact.messages
    assert (message.latitude, message.longitude, message.maps_url) == (None, None, None)
    assert message.battery_percent is None


@pytest.mark.parametrize(
    "soc, percent", [(0.734, 73), (0.006, 1), (1.2, 100), (-0.1, 0), (None, None)]
)
def test_battery_percent_is_rounded_and_clamped(config, clock, soc, percent) -> None:
    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [contact])
    sos.tick(clock(), None, soc)
    sos.trigger_manual(clock())
    assert contact.messages[0].battery_percent == percent


def test_timestamp_comes_from_utc_now(config, clock) -> None:
    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [contact])
    sos.trigger_manual(clock())
    assert contact.messages[0].timestamp_utc == FIXED_ISO


def test_default_utc_now_is_an_aware_iso_timestamp(config, clock) -> None:
    contact = FakeNotifier(clock)
    sos = SosController(config, [contact], clock=clock, runner=run_now)
    before = datetime.now(UTC).replace(microsecond=0)
    sos.trigger_manual(clock())
    after = datetime.now(UTC)
    stamp = datetime.fromisoformat(contact.messages[0].timestamp_utc)
    assert stamp.utcoffset() == timedelta(0)
    assert before <= stamp <= after


def test_manual_sos_sends_immediately(config, clock) -> None:
    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [contact])
    sos.tick(clock(), (10.0, 20.0), 0.5)

    sos.trigger_manual(clock())

    assert contact.messages == [SosMessage("manual", 10.0, 20.0, FIXED_ISO, 50)]
    assert sos.state is SosState.SENT
    assert spoken(sos.tick(clock(), None, None)) == ["Sending SOS.", "SOS sent."]


def test_manual_sos_during_countdown_sends_the_fall_sos_now(config, clock) -> None:
    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [contact])
    sos.trigger_fall(clock())
    sos.tick(clock(), None, None)
    clock.advance(2.0)

    sos.trigger_manual(clock())

    assert [message.reason for message in contact.messages] == ["fall"]
    assert sos.state is SosState.SENT
    heard = []
    for _ in range(20):
        heard += spoken(sos.tick(clock(), None, None))
        clock.advance(1.0)
    assert heard == ["Sending SOS.", "SOS sent."]
    assert len(contact.messages) == 1


def test_manual_sos_while_sending_is_ignored(config, clock) -> None:
    contact = FakeNotifier(clock)
    jobs: list[Job] = []
    sos = make_controller(config, clock, [contact], runner=jobs.append)
    sos.trigger_manual(clock())
    assert sos.state is SosState.SENDING

    sos.trigger_manual(clock())
    sos.trigger_fall(clock())

    assert len(jobs) == 1
    assert sos.state is SosState.SENDING
    assert sos.cancel(clock()) is False
    jobs[0]()
    assert sos.state is SosState.SENT
    assert len(contact.messages) == 1


def test_manual_sos_while_waiting_to_retry_is_ignored(config, clock) -> None:
    contact = FakeNotifier(clock, failures=1)
    sos = make_controller(config, clock, [contact])
    sos.trigger_manual(clock())
    clock.advance(2.0)

    sos.trigger_manual(clock())

    assert contact.attempts == [1000.0]
    assert sos.state is SosState.SENDING
    clock.advance(3.0)
    sos.tick(clock(), None, None)
    assert contact.attempts == [1000.0, 1005.0]
    assert sos.state is SosState.SENT


def test_retries_follow_retry_delays_and_end_in_failed(config, clock) -> None:
    contact = FakeNotifier(clock, failures=100)
    sos = make_controller(config, clock, [contact])
    sos.trigger_manual(clock())
    assert spoken(sos.tick(clock(), None, None)) == ["Sending SOS.", "SOS failed. Retrying."]

    heard = []
    for delay in config.sos.retry_delays_s:  # 5, 15, 30, 60 s
        clock.advance(delay - 0.5)
        assert sos.tick(clock(), None, None) == []
        assert sos.state is SosState.SENDING
        clock.advance(0.5)
        heard += spoken(sos.tick(clock(), None, None))

    assert contact.attempts == [1000.0, 1005.0, 1020.0, 1050.0, 1110.0]
    assert heard == ["SOS failed. Retrying."] * 3 + [FAILED_TEXT]
    assert sos.state is SosState.FAILED
    clock.advance(600.0)
    assert sos.tick(clock(), None, None) == []
    assert len(contact.attempts) == 5


def test_no_retry_delays_means_failed_after_the_first_attempt(config, clock) -> None:
    config.sos.retry_delays_s = ()
    sos = make_controller(config, clock, [FakeNotifier(clock, failures=1)])
    sos.trigger_manual(clock())
    assert spoken(sos.tick(clock(), None, None)) == ["Sending SOS.", FAILED_TEXT]
    assert sos.state is SosState.FAILED


def test_only_failed_remote_notifiers_are_retried(config, clock) -> None:
    local_log = FakeNotifier(clock, name="log", remote=False)
    telegram = FakeNotifier(clock, name="telegram")
    webhook = FakeNotifier(clock, name="webhook", failures=1)
    sos = make_controller(config, clock, [local_log, telegram, webhook])
    sos.trigger_manual(clock())
    assert spoken(sos.tick(clock(), None, None)) == ["Sending SOS.", "SOS failed. Retrying."]

    clock.advance(5.0)

    assert spoken(sos.tick(clock(), None, None)) == ["SOS sent."]
    assert sos.state is SosState.SENT
    assert (local_log.attempts, telegram.attempts) == ([1000.0], [1000.0])
    assert webhook.attempts == [1000.0, 1005.0]
    assert webhook.messages == telegram.messages


def test_failing_local_log_is_not_retried_and_does_not_block_sent(config, clock) -> None:
    local_log = FakeNotifier(clock, name="log", remote=False, failures=5)
    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [local_log, contact])
    sos.trigger_manual(clock())
    clock.advance(100.0)
    sos.tick(clock(), None, None)
    assert sos.state is SosState.SENT
    assert local_log.attempts == [1000.0]


def test_only_the_local_log_announces_sos_recorded(config, clock, tmp_path) -> None:
    log_path = tmp_path / "sos.log"
    sos = make_controller(config, clock, [LogNotifier(log_path)])

    sos.trigger_manual(clock())

    assert spoken(sos.tick(clock(), None, None)) == [
        "Sending SOS.",
        "SOS recorded. No emergency contact is set up.",
    ]
    assert sos.state is SosState.SENT
    assert len(log_path.read_text(encoding="utf-8").splitlines()) == 1


def test_failed_local_log_without_contacts_fails(config, clock) -> None:
    local_log = FakeNotifier(clock, name="log", remote=False, failures=1)
    sos = make_controller(config, clock, [local_log])
    sos.trigger_manual(clock())
    alerts = spoken(sos.tick(clock(), None, None))
    assert alerts == ["Sending SOS.", "SOS failed. No emergency contact is set up."]
    assert sos.state is SosState.FAILED


def test_no_notifiers_at_all_fails(config, clock) -> None:
    sos = make_controller(config, clock, [])
    sos.trigger_manual(clock())
    alerts = spoken(sos.tick(clock(), None, None))
    assert alerts == ["Sending SOS.", "SOS failed. No emergency contact is set up."]
    assert sos.state is SosState.FAILED


def test_new_sos_can_start_after_sent_or_failed(config, clock) -> None:
    contact = FakeNotifier(clock, failures=5)  # fails every attempt of the first SOS
    sos = make_controller(config, clock, [contact])
    sos.trigger_manual(clock())
    for delay in config.sos.retry_delays_s:
        clock.advance(delay)
        sos.tick(clock(), None, None)
    assert sos.state is SosState.FAILED

    sos.trigger_manual(clock())
    assert sos.state is SosState.SENT
    sos.tick(clock(), None, None)

    sos.trigger_fall(clock())
    assert sos.state is SosState.COUNTDOWN
    assert spoken(sos.tick(clock(), None, None)) == [START_TEXT]


def test_every_announcement_is_danger_level_with_its_own_key(config, clock) -> None:
    sos = make_controller(config, clock, [FakeNotifier(clock, failures=1)])
    alerts: list[Alert] = []
    sos.trigger_fall(clock())
    alerts += sos.tick(clock(), None, None)
    sos.cancel(clock())
    alerts += sos.tick(clock(), None, None)
    sos.trigger_manual(clock())
    alerts += sos.tick(clock(), None, None)
    clock.advance(5.0)
    alerts += sos.tick(clock(), None, None)

    assert spoken(alerts) == [
        START_TEXT,
        "SOS cancelled.",
        "Sending SOS.",
        "SOS failed. Retrying.",
        "SOS sent.",
    ]
    assert {alert.level for alert in alerts} == {AlertLevel.DANGER}
    patterns = [alert.haptic_pattern for alert in alerts]
    assert patterns == ["sos_countdown", "confirm", "sos_countdown", None, "sos_sent"]
    assert len({alert.key for alert in alerts}) == 5


def test_runner_that_cannot_start_counts_as_a_failed_attempt(config, clock, caplog) -> None:
    def broken_runner(job: Job) -> None:
        raise RuntimeError("can't start new thread")

    contact = FakeNotifier(clock)
    sos = make_controller(config, clock, [contact], runner=broken_runner)
    sos.trigger_manual(clock())
    assert spoken(sos.tick(clock(), None, None)) == ["Sending SOS.", "SOS failed. Retrying."]
    assert contact.attempts == []
    assert "Could not start sending the SOS" in caplog.text


def test_default_runner_sends_on_a_daemon_thread(config, clock) -> None:
    started = threading.Event()
    release = threading.Event()

    class SlowNetworkNotifier:
        name = "slow"
        remote = True
        thread: threading.Thread | None = None

        def send(self, message: SosMessage) -> None:
            self.thread = threading.current_thread()
            started.set()
            if not release.wait(5.0):
                raise TimeoutError("the test never released the notifier")

    notifier = SlowNetworkNotifier()
    sos = SosController(config, [notifier], clock=clock, utc_now=lambda: FIXED_UTC)

    sos.trigger_manual(clock())  # returns while the message is still on its way

    assert started.wait(5.0)
    assert sos.state is SosState.SENDING
    assert notifier.thread is not None
    assert notifier.thread is not threading.current_thread()
    assert notifier.thread.daemon
    release.set()
    notifier.thread.join(5.0)
    assert sos.state is SosState.SENT
    assert spoken(sos.tick(clock(), None, None)) == ["Sending SOS.", "SOS sent."]


def test_controller_accepts_the_sos_section_of_the_config(config, clock) -> None:
    config.sos.fall_countdown_s = 6.0
    sos = make_controller(config.sos, clock, [FakeNotifier(clock)])
    sos.trigger_fall(clock())
    assert sos.seconds_remaining == pytest.approx(6.0)


# --- notifiers ------------------------------------------------------------------------------


def test_log_notifier_appends_one_json_line_per_message(tmp_path) -> None:
    path = tmp_path / "logs" / "sos.log"
    notifier = LogNotifier(path)
    manual = SosMessage("manual", None, None, "2026-11-08T13:00:00+00:00", None)

    notifier.send(MESSAGE)
    notifier.send(manual)

    lines = path.read_text(encoding="utf-8").splitlines()
    assert [json.loads(line) for line in lines] == [
        MESSAGE_FIELDS,
        {
            "reason": "manual",
            "latitude": None,
            "longitude": None,
            "timestamp_utc": "2026-11-08T13:00:00+00:00",
            "battery_percent": None,
            "maps_url": None,
        },
    ]
    assert (notifier.name, notifier.remote) == ("log", False)


def test_log_notifier_keeps_earlier_records(tmp_path) -> None:
    path = tmp_path / "sos.log"
    path.write_text('{"reason": "manual"}\n', encoding="utf-8")
    LogNotifier(path).send(MESSAGE)
    lines = path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    assert json.loads(lines[1]) == MESSAGE_FIELDS


def test_webhook_posts_the_message_as_json(monkeypatch) -> None:
    calls = install_urlopen(monkeypatch, b"accepted")

    WebhookNotifier(WEBHOOK_URL, timeout_s=7.5).send(MESSAGE)

    [(request, timeout)] = calls
    assert request.full_url == WEBHOOK_URL
    assert request.get_method() == "POST"
    assert request.get_header("Content-type") == "application/json"
    assert request.get_header("User-agent").startswith("NavSight/")
    assert timeout == 7.5
    assert json.loads(request.data) == {**MESSAGE_FIELDS, "text": format_sos_text(MESSAGE)}


@pytest.mark.parametrize(
    "make_error, expected",
    [
        (lambda request: http_error(request, 500, "Server Error", b""), "HTTP Error 500"),
        (lambda request: urllib.error.URLError(f"no route to {request.full_url}"), "to ***"),
        (lambda request: ValueError(f"bad path {request.selector}"), "bad path ***"),
        (lambda request: TimeoutError("timed out"), "timed out"),
    ],
    ids=["http-error", "url-in-message", "path-in-message", "timeout"],
)
def test_webhook_failures_never_show_the_url(monkeypatch, make_error, expected) -> None:
    install_failing_urlopen(monkeypatch, make_error)

    with pytest.raises(NotifierError) as caught:
        WebhookNotifier(WEBHOOK_URL).send(MESSAGE)

    text = str(caught.value)
    assert expected in text
    assert "hooks.example.test" in text
    assert "K3Y-s3cret" not in text
    assert caught.value.__suppress_context__  # the original error, holding the URL, is hidden


@pytest.mark.parametrize(
    "url", ["hooks.example.test/K3Y", "ftp://example.test/K3Y", "https:///K3Y", "http://[::1/K3Y"]
)
def test_webhook_rejects_urls_that_are_not_http(url) -> None:
    with pytest.raises(ValueError, match="sos.webhook_url") as caught:
        WebhookNotifier(url)
    assert "K3Y" not in str(caught.value)


def test_webhook_repr_shows_only_the_host() -> None:
    notifier = WebhookNotifier(WEBHOOK_URL)
    assert repr(notifier) == "WebhookNotifier(host='hooks.example.test')"
    assert (notifier.name, notifier.remote) == ("webhook", True)


def test_telegram_uses_the_bot_api_send_message(monkeypatch) -> None:
    calls = install_urlopen(monkeypatch)

    TelegramNotifier(TOKEN, "42", timeout_s=4.0).send(MESSAGE)

    [(request, timeout)] = calls
    assert request.full_url == f"https://api.telegram.org/bot{TOKEN}/sendMessage"
    assert request.get_method() == "POST"
    assert request.get_header("Content-type") == "application/json"
    assert json.loads(request.data) == {"chat_id": "42", "text": format_sos_text(MESSAGE)}
    assert timeout == 4.0


def test_telegram_error_reply_is_reported_without_the_token(monkeypatch) -> None:
    body = b'{"ok": false, "error_code": 400, "description": "Bad Request: chat not found"}'
    install_failing_urlopen(monkeypatch, lambda request: http_error(request, 400, "Bad", body))

    with pytest.raises(NotifierError, match="HTTP 400: Bad Request: chat not found") as caught:
        TelegramNotifier(TOKEN, "42").send(MESSAGE)

    assert TOKEN not in str(caught.value)


def test_telegram_error_without_json_falls_back_to_the_http_reason(monkeypatch) -> None:
    install_failing_urlopen(
        monkeypatch, lambda request: http_error(request, 502, "Bad Gateway", b"<html>")
    )
    with pytest.raises(NotifierError, match="HTTP 502: Bad Gateway"):
        TelegramNotifier(TOKEN, "42").send(MESSAGE)


def test_telegram_reply_that_is_not_ok_raises(monkeypatch) -> None:
    install_urlopen(monkeypatch, b'{"ok": false}')
    with pytest.raises(NotifierError, match="did not confirm"):
        TelegramNotifier(TOKEN, "42").send(MESSAGE)


def test_telegram_network_error_hides_the_token(monkeypatch) -> None:
    install_failing_urlopen(
        monkeypatch, lambda request: urllib.error.URLError(f"cannot reach {request.full_url}")
    )
    with pytest.raises(NotifierError) as caught:
        TelegramNotifier(TOKEN, "42").send(MESSAGE)
    assert "cannot reach https://api.telegram.org/bot***/sendMessage" in str(caught.value)
    assert TOKEN not in str(caught.value)


def test_telegram_repr_hides_the_token() -> None:
    notifier = TelegramNotifier(TOKEN, "42")
    assert repr(notifier) == "TelegramNotifier(chat_id='42')"
    assert (notifier.name, notifier.remote) == ("telegram", True)


@pytest.mark.parametrize(
    "make_error",
    [
        lambda request: urllib.error.URLError(f"cannot reach {request.full_url}"),
        lambda request: ValueError(f"unknown url type: {request.full_url!r}"),
        lambda request: http_error(
            request, 401, "Unauthorized", b'{"ok": false, "description": "Unauthorized"}'
        ),
    ],
    ids=["network", "invalid-url", "http-401"],
)
def test_telegram_token_never_appears_in_logs(
    monkeypatch, caplog, clock, tmp_path, make_error
) -> None:
    caplog.set_level(logging.DEBUG)
    install_failing_urlopen(monkeypatch, make_error)
    sos_config = SosConfig(
        telegram_bot_token=TOKEN, telegram_chat_id="42", log_file=str(tmp_path / "sos.log")
    )
    notifiers = make_notifiers(sos_config)
    sos = make_controller(sos_config, clock, notifiers)

    sos.trigger_manual(clock())
    for delay in sos_config.retry_delays_s:
        clock.advance(delay)
        sos.tick(clock(), None, None)

    assert sos.state is SosState.FAILED
    assert "SOS via telegram failed" in caplog.text
    assert TOKEN not in caplog.text
    assert all(TOKEN not in record.getMessage() for record in caplog.records)
    assert TOKEN not in repr(notifiers)


def test_telegram_token_not_logged_when_sending_succeeds(
    monkeypatch, caplog, clock, tmp_path
) -> None:
    caplog.set_level(logging.DEBUG)
    install_urlopen(monkeypatch)
    sos_config = SosConfig(
        telegram_bot_token=TOKEN, telegram_chat_id="42", log_file=str(tmp_path / "sos.log")
    )
    sos = make_controller(sos_config, clock, make_notifiers(sos_config))
    sos.trigger_manual(clock())
    assert sos.state is SosState.SENT
    assert "telegram" in caplog.text
    assert TOKEN not in caplog.text


def test_make_notifiers_default_is_the_local_log_only(caplog) -> None:
    notifiers = make_notifiers(SosConfig())
    assert [notifier.name for notifier in notifiers] == ["log"]
    assert isinstance(notifiers[0], LogNotifier)
    assert notifiers[0].path == Path("~/.navsight/sos.log").expanduser()
    assert "No emergency contact is set up" in caplog.text


def test_make_notifiers_builds_every_configured_contact(monkeypatch, tmp_path) -> None:
    calls = install_urlopen(monkeypatch)
    sos_config = SosConfig(
        webhook_url=WEBHOOK_URL,
        telegram_bot_token=TOKEN,
        telegram_chat_id="42",
        log_file=str(tmp_path / "sos.log"),
        request_timeout_s=3.5,
    )

    notifiers = make_notifiers(sos_config)

    assert [(n.name, n.remote) for n in notifiers] == [
        ("log", False),
        ("webhook", True),
        ("telegram", True),
    ]
    for notifier in notifiers[1:]:
        notifier.send(MESSAGE)
    assert [timeout for _, timeout in calls] == [3.5, 3.5]


@pytest.mark.parametrize("token, chat_id", [(TOKEN, None), (None, "42")])
def test_make_notifiers_skips_half_configured_telegram(caplog, tmp_path, token, chat_id) -> None:
    sos_config = SosConfig(
        telegram_bot_token=token, telegram_chat_id=chat_id, log_file=str(tmp_path / "sos.log")
    )
    notifiers = make_notifiers(sos_config)
    assert [notifier.name for notifier in notifiers] == ["log"]
    assert "set both sos.telegram_bot_token and sos.telegram_chat_id" in caplog.text
    assert TOKEN not in caplog.text


def test_make_notifiers_skips_an_invalid_webhook_without_logging_it(caplog, tmp_path) -> None:
    sos_config = SosConfig(webhook_url="hooks.example.test/K3Y", log_file=str(tmp_path / "s.log"))
    notifiers = make_notifiers(sos_config)
    assert [notifier.name for notifier in notifiers] == ["log"]
    assert "SOS webhook disabled" in caplog.text
    assert "K3Y" not in caplog.text


def test_make_notifiers_without_a_log_file() -> None:
    assert make_notifiers(SosConfig(log_file="")) == []
