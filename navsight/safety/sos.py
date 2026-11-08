"""Emergency SOS: a cancellable fall countdown, then a message to every configured notifier.

Notifier secrets (the Telegram bot token, webhook URLs) never appear in logs, errors or reprs.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
import urllib.error
import urllib.request
from collections.abc import Callable, Iterable, Sequence
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from enum import Enum
from pathlib import Path
from typing import Any, Protocol
from urllib.parse import urlsplit

from navsight import __version__
from navsight.config import NavSightConfig, SosConfig
from navsight.types import Alert, AlertLevel
from navsight.util import Clock, clamp

log = logging.getLogger(__name__)

Job = Callable[[], None]
Runner = Callable[[Job], None]

_TELEGRAM_API = "https://api.telegram.org"
_USER_AGENT = f"NavSight/{__version__}"
_REASON_TEXT = {"fall": "a fall was detected", "manual": "the SOS button was held"}


class SosState(Enum):
    IDLE = "idle"
    COUNTDOWN = "countdown"
    SENDING = "sending"
    SENT = "sent"
    FAILED = "failed"


@dataclass(frozen=True)
class SosMessage:
    reason: str  # "fall" or "manual"
    latitude: float | None
    longitude: float | None
    timestamp_utc: str  # ISO 8601
    battery_percent: int | None

    @property
    def maps_url(self) -> str | None:
        if self.latitude is None or self.longitude is None:
            return None
        return google_maps_url(self.latitude, self.longitude)


def google_maps_url(latitude: float, longitude: float) -> str:
    return f"https://maps.google.com/?q={latitude:.6f},{longitude:.6f}"


def format_sos_text(message: SosMessage) -> str:
    """The human-readable SOS text sent to emergency contacts."""
    reason = _REASON_TEXT.get(message.reason, message.reason)
    location = message.maps_url or "unknown (no GPS fix)"
    battery = "unknown" if message.battery_percent is None else f"{message.battery_percent}%"
    return (
        f"NavSight SOS: {reason}.\n"
        f"Location: {location}\n"
        f"Battery: {battery}\n"
        f"Time (UTC): {message.timestamp_utc}"
    )


class NotifierError(RuntimeError):
    """An SOS message could not be delivered. The text never contains secrets."""


class Notifier(Protocol):
    name: str
    remote: bool  # False for the local log

    def send(self, message: SosMessage) -> None: ...  # raises on failure


class LogNotifier:
    """Appends each message as one JSON line to a local file, the record of last resort."""

    name = "log"
    remote = False

    def __init__(self, path: Path | str) -> None:
        self.path = Path(path).expanduser()

    def send(self, message: SosMessage) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(_message_record(message)) + "\n")
            handle.flush()
            os.fsync(handle.fileno())


class WebhookNotifier:
    """POSTs the message as JSON, plus a ready-made ``text`` field, to a webhook URL.

    Webhook URLs usually embed a secret key, so only the host is ever shown.
    """

    name = "webhook"
    remote = True

    def __init__(self, url: str, timeout_s: float = 10.0) -> None:
        try:
            parts = urlsplit(url)
        except ValueError:
            raise ValueError("sos.webhook_url is not a valid URL") from None
        if parts.scheme not in ("http", "https") or not parts.hostname:
            raise ValueError("sos.webhook_url must be an http:// or https:// URL")
        self._url = url
        self._host = parts.hostname
        self._secrets = [url, parts.path, parts.query, parts.password or ""]
        self._timeout_s = timeout_s

    def __repr__(self) -> str:
        return f"WebhookNotifier(host={self._host!r})"

    def send(self, message: SosMessage) -> None:
        payload = {**_message_record(message), "text": format_sos_text(message)}
        try:
            _post_json(self._url, payload, self._timeout_s)
        except Exception as exc:
            problem = _redact(f"{type(exc).__name__}: {exc}", self._secrets)
            raise NotifierError(f"webhook POST to {self._host} failed: {problem}") from None


class TelegramNotifier:
    """Sends the SOS text to one chat with the Telegram Bot API ``sendMessage`` method.

    The bot token works like a password: it never appears in logs, errors or ``repr``.
    """

    name = "telegram"
    remote = True

    def __init__(self, bot_token: str, chat_id: str, timeout_s: float = 10.0) -> None:
        self._token = bot_token
        self._chat_id = chat_id
        self._timeout_s = timeout_s

    def __repr__(self) -> str:
        return f"TelegramNotifier(chat_id={self._chat_id!r})"

    def send(self, message: SosMessage) -> None:
        url = f"{_TELEGRAM_API}/bot{self._token}/sendMessage"
        payload = {"chat_id": self._chat_id, "text": format_sos_text(message)}
        try:
            reply = json.loads(_post_json(url, payload, self._timeout_s))
        except urllib.error.HTTPError as exc:
            raise self._error(_http_error_detail(exc)) from None
        except Exception as exc:
            raise self._error(f"{type(exc).__name__}: {exc}") from None
        if not (isinstance(reply, dict) and reply.get("ok") is True):
            raise self._error("Telegram did not confirm the message")

    def _error(self, problem: str) -> NotifierError:
        return NotifierError(_redact(f"Telegram sendMessage failed: {problem}", [self._token]))


def make_notifiers(config: SosConfig) -> list[Notifier]:
    """Build the notifiers the config enables, local log first so a record always exists."""
    notifiers: list[Notifier] = []
    if config.log_file:
        notifiers.append(LogNotifier(config.log_file))
    if config.webhook_url:
        try:
            notifiers.append(WebhookNotifier(config.webhook_url, config.request_timeout_s))
        except ValueError as exc:
            log.error("SOS webhook disabled: %s", exc)
    token, chat_id = config.telegram_bot_token, config.telegram_chat_id
    if token and chat_id:
        notifiers.append(TelegramNotifier(token, chat_id, config.request_timeout_s))
    elif token or chat_id:
        log.error("SOS Telegram disabled: set both sos.telegram_bot_token and sos.telegram_chat_id")
    if not any(notifier.remote for notifier in notifiers):
        log.warning("No emergency contact is set up: an SOS is only written to the local log")
    return notifiers


@dataclass(frozen=True)
class _Attempt:
    message: SosMessage
    notifiers: tuple[Notifier, ...]
    not_before: float = 0.0


class SosController:
    """Runs an SOS: the fall countdown, sending with retries, and the spoken outcomes.

    Call ``tick`` from the core loop; it returns the alerts to announce (all at DANGER level so
    no verbosity setting or cooldown can hide them). ``runner`` is called with a zero-argument
    job and must run it, now or on another thread; the default starts a daemon thread so a slow
    network never blocks the core loop. ``utc_now`` returns the aware time stamped on messages.
    ``config`` may be the full ``NavSightConfig`` or just its ``SosConfig``.
    """

    def __init__(
        self,
        config: NavSightConfig | SosConfig,
        notifiers: Sequence[Notifier],
        *,
        clock: Clock = time.monotonic,
        runner: Runner | None = None,
        utc_now: Callable[[], datetime] | None = None,
    ) -> None:
        self._config = config.sos if isinstance(config, NavSightConfig) else config
        self._notifiers = tuple(notifiers)
        self._clock = clock
        self._runner = runner if runner is not None else _run_on_daemon_thread
        self._utc_now = utc_now if utc_now is not None else _utc_now
        self._lock = threading.Lock()
        self._state = SosState.IDLE
        self._alerts: list[Alert] = []
        self._location: tuple[float, float] | None = None
        self._battery_soc: float | None = None
        self._countdown_end = 0.0
        self._next_announcement = 0.0
        self._announced_start = False
        self._retries_used = 0
        self._retry: _Attempt | None = None

    @property
    def state(self) -> SosState:
        with self._lock:
            return self._state

    @property
    def seconds_remaining(self) -> float | None:
        """Seconds left in the fall countdown, or None when no countdown is running."""
        with self._lock:
            if self._state is not SosState.COUNTDOWN:
                return None
            return max(0.0, self._countdown_end - self._clock())

    def trigger_fall(self, now: float) -> None:
        with self._lock:
            if self._state in (SosState.COUNTDOWN, SosState.SENDING):
                log.info("Fall ignored: an SOS is already %s", self._state.value)
                return
            countdown_s = self._config.fall_countdown_s
            log.warning("Fall detected: sending SOS in %.0f s unless cancelled", countdown_s)
            self._state = SosState.COUNTDOWN
            self._countdown_end = now + countdown_s
            self._next_announcement = now
            self._announced_start = False

    def trigger_manual(self, now: float) -> None:
        """Send an SOS at once; during a fall countdown this sends the fall SOS immediately."""
        with self._lock:
            if self._state is SosState.SENDING:
                log.info("Manual SOS ignored: an SOS is already being sent")
                return
            reason = "fall" if self._state is SosState.COUNTDOWN else "manual"
            attempt = self._begin_sending(reason, now)
        self._launch(attempt)

    def cancel(self, now: float) -> bool:
        """Stop a fall countdown. Returns False when no countdown is running."""
        with self._lock:
            if self._state is not SosState.COUNTDOWN:
                return False
            log.info("SOS countdown cancelled")
            self._state = SosState.IDLE
            self._announce("sos_cancelled", "SOS cancelled.", now, "confirm")
            return True

    def tick(
        self, now: float, location: tuple[float, float] | None, battery_soc: float | None
    ) -> list[Alert]:
        """Advance the countdown and any retry, and return the alerts to announce.

        ``location`` is (latitude, longitude). ``None`` keeps the last known location or
        battery level, so an SOS always carries the best information available.
        """
        attempt: _Attempt | None = None
        with self._lock:
            if location is not None:
                self._location = location
            if battery_soc is not None:
                self._battery_soc = battery_soc
            if self._state is SosState.COUNTDOWN:
                attempt = self._update_countdown(now)
            elif self._retry_due(now):
                attempt = self._retry
                self._retry = None
        if attempt is not None:
            self._launch(attempt)
        with self._lock:
            alerts, self._alerts = self._alerts, []
        return alerts

    def _update_countdown(self, now: float) -> _Attempt | None:
        if now >= self._countdown_end:
            return self._begin_sending("fall", now)
        if now >= self._next_announcement:
            self._announce_countdown(now)
        return None

    def _announce_countdown(self, now: float) -> None:
        every = self._config.countdown_announce_every_s
        while every > 0 and self._next_announcement + every <= now:
            self._next_announcement += every  # after a long gap, announce only the latest point
        seconds = _seconds_phrase(self._countdown_end - self._next_announcement)
        if self._announced_start:
            text = f"Sending SOS in {seconds}."
        else:
            text = f"Fall detected. Sending SOS in {seconds}. Press any button to cancel."
            self._announced_start = True
        self._announce("sos_countdown", text, now, "sos_countdown")
        self._next_announcement = self._next_announcement + every if every > 0 else math.inf

    def _begin_sending(self, reason: str, now: float) -> _Attempt:
        names = ", ".join(notifier.name for notifier in self._notifiers) or "no notifiers"
        log.warning("Sending %s SOS via %s", reason, names)
        self._state = SosState.SENDING
        self._retries_used = 0
        self._retry = None
        self._announce("sos_sending", "Sending SOS.", now, "sos_countdown")
        return _Attempt(self._build_message(reason), self._notifiers)

    def _build_message(self, reason: str) -> SosMessage:
        latitude, longitude = self._location if self._location is not None else (None, None)
        battery_percent = None
        if self._battery_soc is not None:
            battery_percent = round(clamp(self._battery_soc, 0.0, 1.0) * 100)
        timestamp = self._utc_now().isoformat(timespec="seconds")
        return SosMessage(reason, latitude, longitude, timestamp, battery_percent)

    def _retry_due(self, now: float) -> bool:
        return (
            self._state is SosState.SENDING
            and self._retry is not None
            and now >= self._retry.not_before
        )

    def _launch(self, attempt: _Attempt) -> None:
        try:
            self._runner(lambda: self._attempt(attempt))
        except Exception:
            log.exception("Could not start sending the SOS")
            self._finish_attempt(attempt, attempt.notifiers)

    def _attempt(self, attempt: _Attempt) -> None:
        message = attempt.message
        failed = [notifier for notifier in attempt.notifiers if not _deliver(notifier, message)]
        self._finish_attempt(attempt, failed)

    def _finish_attempt(self, attempt: _Attempt, failed: Sequence[Notifier]) -> None:
        with self._lock:
            now = self._clock()
            if not any(notifier.remote for notifier in self._notifiers):
                self._finish_without_contacts(failed, now)
                return
            remote_failed = tuple(notifier for notifier in failed if notifier.remote)
            delays = self._config.retry_delays_s
            if not remote_failed:
                self._finish(SosState.SENT, "sos_sent", "SOS sent.", now, "sos_sent")
            elif self._retries_used < len(delays):
                delay = delays[self._retries_used]
                self._retries_used += 1
                self._retry = _Attempt(attempt.message, remote_failed, now + delay)
                log.warning("SOS retry %d of %d in %.0f s", self._retries_used, len(delays), delay)
                self._announce("sos_retrying", "SOS failed. Retrying.", now)
            else:
                text = "SOS failed. Hold the SOS button to try again."
                self._finish(SosState.FAILED, "sos_failed", text, now)

    def _finish_without_contacts(self, failed: Sequence[Notifier], now: float) -> None:
        if self._notifiers and not failed:
            text = "SOS recorded. No emergency contact is set up."
            self._finish(SosState.SENT, "sos_recorded", text, now)
        else:
            text = "SOS failed. No emergency contact is set up."
            self._finish(SosState.FAILED, "sos_failed", text, now)

    def _finish(
        self, state: SosState, key: str, text: str, now: float, pattern: str | None = None
    ) -> None:
        log.warning("SOS %s: %s", state.value, text)
        self._state = state
        self._retry = None
        self._announce(key, text, now, pattern)

    def _announce(self, key: str, text: str, now: float, pattern: str | None = None) -> None:
        alert = Alert(key, AlertLevel.DANGER, text, haptic_pattern=pattern, timestamp=now)
        self._alerts.append(alert)


def _deliver(notifier: Notifier, message: SosMessage) -> bool:
    try:
        notifier.send(message)
    except Exception as exc:
        log.warning("SOS via %s failed: %s", notifier.name, exc)
        return False
    log.info("SOS delivered via %s", notifier.name)
    return True


def _run_on_daemon_thread(job: Job) -> None:
    threading.Thread(target=job, name="sos-sender", daemon=True).start()


def _utc_now() -> datetime:
    return datetime.now(UTC)


def _seconds_phrase(seconds: float) -> str:
    whole = round(seconds)
    return "1 second" if whole == 1 else f"{whole} seconds"


def _message_record(message: SosMessage) -> dict[str, Any]:
    return {**asdict(message), "maps_url": message.maps_url}


def _post_json(url: str, payload: dict[str, Any], timeout_s: float) -> bytes:
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json", "User-Agent": _USER_AGENT},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        return response.read()


def _http_error_detail(error: urllib.error.HTTPError) -> str:
    """'HTTP 400: Bad Request: chat not found', using the description in Telegram's reply."""
    try:
        description = json.loads(error.read()).get("description")
    except (OSError, ValueError, AttributeError):
        log.debug("Telegram error reply had no JSON description")
        description = None
    return f"HTTP {error.code}: {description or error.reason}"


def _redact(text: str, secrets: Iterable[str]) -> str:
    for secret in secrets:
        if len(secret) > 1:
            text = text.replace(secret, "***")
    return text
