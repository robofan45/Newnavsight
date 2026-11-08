"""NEO-M9N GNSS receiver on the Pi's UART (GPIO 14 TX, GPIO 15 RX).

``NmeaParser`` merges the receiver's NMEA sentences into ``GpsFix`` values, ``build_ubx`` frames
the binary UBX commands used for power saving, and ``GpsReader`` reads the port on its own
thread, duty cycling it when the power profile asks for less GPS. ``SimulatedGps`` is the twin
used by the simulator and the tests.
"""

from __future__ import annotations

import functools
import logging
import math
import re
import string
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import replace
from typing import Protocol

from navsight.config import GpsConfig
from navsight.types import GpsFix
from navsight.util import Clock, Sleeper, wrap_heading_deg

log = logging.getLogger(__name__)

KNOTS_TO_MPS = 1852.0 / 3600.0

_UBX_SYNC = b"\xb5\x62"
_UBX_CLASS_RXM = 0x02
_UBX_ID_RXM_PMREQ = 0x41
_PMREQ_FLAG_BACKUP = 0x02
_PMREQ_WAKE_ON_UART_RX = 0x08
_WAKE_BYTE = b"\xff"

_RETRY_DELAY_S = 2.0  # after a serial error, the port stays closed this long
_ERROR_LOG_INTERVAL_S = 30.0
_IDLE_SLEEP_S = 0.5  # longest single sleep while the port is closed, so changes apply quickly
_JOIN_MARGIN_S = 1.0

_COORDINATE = re.compile(r"(\d+)(\d\d(?:\.\d*)?)", re.ASCII)
_HEMISPHERES = {"N": (1.0, 90.0), "S": (-1.0, 90.0), "E": (1.0, 180.0), "W": (-1.0, 180.0)}


def nmea_checksum_ok(sentence: str) -> bool:
    """True for ``$<body>*HH`` where HH is the XOR of the body's characters, in hex."""
    text = sentence.strip()
    if not text.startswith("$") or not text.isascii():
        return False
    body, star, checksum = text[1:].partition("*")
    if not star or len(checksum) != 2 or not all(c in string.hexdigits for c in checksum):
        return False
    return int(checksum, 16) == _xor_checksum(body)


def nmea_with_checksum(body: str) -> str:
    """Frame a sentence body such as ``"PUBX,40,GLL,0,0,0,0,0,0"`` as ``$<body>*HH`` plus CR LF."""
    if not body.isascii() or any(char in body for char in "$*\r\n"):
        raise ValueError(f"An NMEA body must be ASCII without '$', '*' or line breaks: {body!r}")
    return f"${body}*{_xor_checksum(body):02X}\r\n"


def _xor_checksum(body: str) -> int:
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return checksum


def parse_lat_lon(value: str, hemisphere: str) -> float | None:
    """Convert an NMEA ``ddmm.mmmm`` (or ``dddmm.mmmm``) value and N/S/E/W to signed degrees.

    An empty value means "no position" and gives ``None``; a malformed or out-of-range value
    raises ``ValueError``.
    """
    value = value.strip()
    if not value:
        return None
    match = _COORDINATE.fullmatch(value)
    limits = _HEMISPHERES.get(hemisphere.strip().upper())
    if match is None or limits is None:
        raise ValueError(f"malformed coordinate {value!r} {hemisphere!r}")
    sign, max_degrees = limits
    minutes = float(match.group(2))
    degrees = int(match.group(1)) + minutes / 60.0
    if minutes >= 60.0 or degrees > max_degrees:
        raise ValueError(f"coordinate out of range: {value!r} {hemisphere!r}")
    return sign * degrees


class NmeaParser:
    """Merges GGA, RMC, GSA and GSV sentences from any talker (GP, GN, GL, GA, GB, GQ).

    ``feed`` returns the merged fix after each GGA or RMC. The M9N sends one GSV group per
    constellation, so satellites in view are summed per talker; the sum restarts with the first
    GSV after a GGA or RMC, so a constellation that stops reporting is not counted forever.
    """

    def __init__(self, *, clock: Clock = time.monotonic) -> None:
        self._clock = clock
        self._state = GpsFix(timestamp=0.0, latitude=None, longitude=None)
        self._latest: GpsFix | None = None
        self._in_view_by_talker: dict[str, int] = {}
        self._restart_in_view = False
        self.parse_errors = 0
        self.satellites_in_view = 0

    @property
    def latest(self) -> GpsFix | None:
        return self._latest

    def feed(self, line: str) -> GpsFix | None:
        sentence = line.strip()
        if not sentence.startswith("$"):
            return None
        if not nmea_checksum_ok(sentence):
            self._reject(sentence, "bad checksum")
            return None
        fields = sentence[1:].partition("*")[0].split(",")
        talker, kind = fields[0][:2], fields[0][2:]
        try:
            if kind == "GGA":
                self._state = _apply_gga(self._state, fields)
            elif kind == "RMC":
                self._state = _apply_rmc(self._state, fields)
            elif kind == "GSA":
                self._state = _apply_gsa(self._state, fields)
            elif kind == "GSV":
                self._count_in_view(talker, fields)
        except ValueError as exc:
            self._reject(sentence, str(exc))
            return None
        if kind not in ("GGA", "RMC"):
            return None
        self._restart_in_view = True
        self._latest = replace(self._state, timestamp=self._clock())
        return self._latest

    def _count_in_view(self, talker: str, fields: list[str]) -> None:
        _require_fields(fields, 4)
        count = _parse_int(fields[3])
        if self._restart_in_view:
            self._in_view_by_talker.clear()
            self._restart_in_view = False
        self._in_view_by_talker[talker] = count
        self.satellites_in_view = sum(self._in_view_by_talker.values())

    def _reject(self, sentence: str, reason: str) -> None:
        self.parse_errors += 1
        log.debug("Rejected NMEA sentence (%s): %s", reason, sentence)


def _apply_gga(state: GpsFix, fields: list[str]) -> GpsFix:
    _require_fields(fields, 10)
    quality = _parse_int(fields[6])
    latitude, longitude = _parse_position(fields[2:6])
    if quality == 0:
        latitude = longitude = None
    return replace(
        state,
        latitude=latitude,
        longitude=longitude,
        fix_quality=quality,
        satellites_used=_parse_int(fields[7]),
        hdop=_parse_float(fields[8]),
        altitude_m=_parse_float(fields[9]),
    )


def _apply_rmc(state: GpsFix, fields: list[str]) -> GpsFix:
    _require_fields(fields, 9)
    status = fields[2]
    if status == "V":
        return replace(
            state, latitude=None, longitude=None, fix_quality=0, speed_mps=None, course_deg=None
        )
    if status != "A":
        raise ValueError(f"unknown RMC status {status!r}")
    latitude, longitude = _parse_position(fields[3:7])
    speed_knots = _parse_float(fields[7])
    course = _parse_float(fields[8])
    quality = state.fix_quality
    if quality == 0 and latitude is not None:
        quality = 1  # a receiver configured without GGA still has a usable fix
    return replace(
        state,
        latitude=latitude,
        longitude=longitude,
        fix_quality=quality,
        speed_mps=None if speed_knots is None else speed_knots * KNOTS_TO_MPS,
        course_deg=None if course is None else wrap_heading_deg(course),
    )


def _apply_gsa(state: GpsFix, fields: list[str]) -> GpsFix:
    _require_fields(fields, 17)
    return replace(state, hdop=_parse_float(fields[16]))


def _parse_position(fields: list[str]) -> tuple[float | None, float | None]:
    """Latitude and longitude from the four fields lat, N/S, lon, E/W: both or neither."""
    latitude = parse_lat_lon(fields[0], fields[1])
    longitude = parse_lat_lon(fields[2], fields[3])
    if (latitude is None) != (longitude is None):
        raise ValueError("position has a latitude or a longitude but not both")
    return latitude, longitude


def _require_fields(fields: list[str], count: int) -> None:
    if len(fields) < count:
        raise ValueError(f"{fields[0]} has {len(fields)} fields, expected at least {count}")


def _parse_int(text: str) -> int:
    """An NMEA integer field; empty means 0."""
    if not text:
        return 0
    if not text.isdigit():
        raise ValueError(f"not a whole number: {text!r}")
    return int(text)


def _parse_float(text: str) -> float | None:
    """An NMEA decimal field; empty means "not available"."""
    if not text:
        return None
    value = float(text)
    if not math.isfinite(value):
        raise ValueError(f"not a finite number: {text!r}")
    return value


def build_ubx(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes:
    """Frame a UBX message: ``B5 62 class id length(LE16) payload ck_a ck_b``.

    The checksum is the 8-bit Fletcher checksum over class, id, length and payload.
    """
    if not (0 <= msg_class <= 0xFF and 0 <= msg_id <= 0xFF and len(payload) <= 0xFFFF):
        raise ValueError("UBX class and id must fit in one byte and the payload in 65535 bytes")
    body = struct.pack("<BBH", msg_class, msg_id, len(payload)) + bytes(payload)
    ck_a = ck_b = 0
    for byte in body:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return _UBX_SYNC + body + bytes((ck_a, ck_b))


def ubx_backup_request(duration_ms: int = 0) -> bytes:
    """UBX-RXM-PMREQ: enter backup mode for ``duration_ms`` (0 = until woken by UART RX)."""
    if not 0 <= duration_ms <= 0xFFFFFFFF:
        raise ValueError(f"duration_ms must fit in 32 bits, got {duration_ms}")
    payload = struct.pack("<B3xIII", 0, duration_ms, _PMREQ_FLAG_BACKUP, _PMREQ_WAKE_ON_UART_RX)
    return build_ubx(_UBX_CLASS_RXM, _UBX_ID_RXM_PMREQ, payload)


class SerialLike(Protocol):
    """The subset of ``serial.Serial`` the reader uses."""

    is_open: bool

    def readline(self) -> bytes: ...

    def write(self, data: bytes) -> int | None: ...

    def close(self) -> None: ...


SerialFactory = Callable[[], SerialLike]


def _open_serial_port(config: GpsConfig) -> SerialLike:
    try:
        import serial  # pyserial, only needed on the Pi
    except ImportError as exc:
        raise RuntimeError("The GPS reader needs pyserial: pip install pyserial") from exc
    return serial.Serial(config.port, config.baud_rate, timeout=config.read_timeout_s)


def _check_poll_interval(interval_s: float | None) -> None:
    if interval_s is not None and not (math.isfinite(interval_s) and interval_s >= 0):
        raise ValueError(f"GPS poll interval must be None or a number >= 0, got {interval_s!r}")


def _is_stale(fix: GpsFix | None, now: float, stale_after_s: float) -> bool:
    return fix is None or now - fix.timestamp > stale_after_s


class GpsReader:
    """Reads the NEO-M9N on a background thread and keeps the latest fix.

    Poll interval 0 keeps the port open. An interval above 0 listens for ``listen_window_s``,
    then closes the port for the interval. ``None`` turns the GPS off. A new interval takes
    effect on the next step: a closed port reopens at once and a fresh listen window starts.
    With ``use_ubx_power_save`` the receiver is put into backup mode before every planned close
    (including ``stop``) and woken with one 0xFF byte after every open, the first one included,
    in case an earlier run left it asleep.
    """

    def __init__(
        self,
        config: GpsConfig,
        *,
        serial_factory: SerialFactory | None = None,
        clock: Clock = time.monotonic,
        sleep: Sleeper = time.sleep,
    ) -> None:
        self._config = config
        if serial_factory is None:
            serial_factory = functools.partial(_open_serial_port, config)
        self._serial_factory = serial_factory
        self._clock = clock
        self._sleep = sleep
        self._parser = NmeaParser(clock=clock)
        self._lock = threading.Lock()
        self._latest: GpsFix | None = None
        self._interval: float | None = 0.0
        self._interval_changed = False
        self._port: SerialLike | None = None
        self._window_start = 0.0
        self._reopen_at = -math.inf
        self._retry_at = -math.inf
        self._last_error_log = -math.inf
        self._recovering = False
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def latest(self) -> GpsFix | None:
        with self._lock:
            return self._latest

    @property
    def poll_interval_s(self) -> float | None:
        with self._lock:
            return self._interval

    def is_stale(self, now: float | None = None) -> bool:
        current = self._clock() if now is None else now
        return _is_stale(self.latest, current, self._config.stale_after_s)

    def set_poll_interval(self, interval_s: float | None) -> None:
        """0 = continuous, > 0 = seconds off between listen windows, None = GPS off."""
        _check_poll_interval(interval_s)
        with self._lock:
            if interval_s == self._interval:
                return
            self._interval = interval_s
            self._interval_changed = True
        log.info("GPS poll interval set to %s", "off" if interval_s is None else f"{interval_s} s")

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="gps-reader", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(self._config.read_timeout_s + _JOIN_MARGIN_S)
            if thread.is_alive():
                log.warning("The GPS reader thread did not stop in time")
        self._power_down()

    def step(self) -> None:
        """One iteration of the reader loop: read and parse one line, or wait while closed."""
        now = self._clock()
        interval = self._take_interval(now)
        if interval is None:
            self._power_down()
            self._sleep(_IDLE_SLEEP_S)
            return
        window_over = now - self._window_start >= self._config.listen_window_s
        if interval > 0 and self._port is not None and window_over:
            self._power_down()
            self._reopen_at = now + interval
        wake_at = max(self._reopen_at, self._retry_at)
        if self._port is None and now < wake_at:
            self._sleep(min(wake_at - now, _IDLE_SLEEP_S))
            return
        try:
            port = self._port if self._port is not None else self._open_port(now)
            self._read_line(port)
        except OSError as exc:  # pyserial's SerialException is an OSError
            self._on_serial_error(exc)

    def _take_interval(self, now: float) -> float | None:
        with self._lock:
            interval = self._interval
            changed, self._interval_changed = self._interval_changed, False
        if changed:
            self._reopen_at = -math.inf
            self._window_start = now
        return interval

    def _open_port(self, now: float) -> SerialLike:
        port = self._serial_factory()
        self._port = port
        self._window_start = now
        if self._config.use_ubx_power_save:
            port.write(_WAKE_BYTE)
        if self._recovering:
            self._recovering = False
            log.info("GPS serial port %s reopened", self._config.port)
        return port

    def _read_line(self, port: SerialLike) -> None:
        if not port.is_open:
            raise OSError("the serial port is not open")
        raw = port.readline()
        if not raw:
            return
        fix = self._parser.feed(raw.decode("ascii", errors="replace"))
        if fix is not None:
            with self._lock:
                self._latest = fix

    def _power_down(self) -> None:
        """Close the port, first putting the receiver into backup mode if configured."""
        port, self._port = self._port, None
        if port is None:
            return
        if self._config.use_ubx_power_save:
            try:
                port.write(ubx_backup_request(0))
            except OSError as exc:
                log.warning("Could not put the GPS receiver into backup mode: %s", exc)
        _close_port(port)

    def _on_serial_error(self, exc: OSError) -> None:
        """Close the port; a step at least 2 s later reopens it."""
        port, self._port = self._port, None
        if port is not None:
            _close_port(port)
        self._retry_at = self._clock() + _RETRY_DELAY_S
        self._recovering = True
        if self._error_log_due():
            log.warning(
                "GPS serial error on %s: %s (retrying every %.0f s)",
                self._config.port,
                exc,
                _RETRY_DELAY_S,
            )
        else:
            log.debug("GPS serial error on %s: %s", self._config.port, exc)

    def _error_log_due(self) -> bool:
        now = self._clock()
        if now - self._last_error_log < _ERROR_LOG_INTERVAL_S:
            return False
        self._last_error_log = now
        return True

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.step()
            except Exception:  # keep the thread alive; serial errors are handled in step()
                if self._error_log_due():
                    log.exception("GPS reader failed; retrying in %.0f s", _RETRY_DELAY_S)
                else:
                    log.debug("GPS reader failed again", exc_info=True)
                self._stop_event.wait(_RETRY_DELAY_S)


def _close_port(port: SerialLike) -> None:
    try:
        port.close()
    except OSError as exc:
        log.debug("Error while closing the GPS port: %s", exc)


class SimulatedGps:
    """Twin of ``GpsReader`` for the simulator and the tests.

    ``set_fix`` and ``set_no_fix`` stand for the receiver's next report. While the poll interval
    is ``None`` (GPS off) reports are dropped, as the real reader's closed port would drop them.
    """

    def __init__(self, config: GpsConfig, *, clock: Clock = time.monotonic) -> None:
        self._config = config
        self._clock = clock
        self._lock = threading.Lock()
        self._latest: GpsFix | None = None
        self._interval: float | None = 0.0

    @property
    def latest(self) -> GpsFix | None:
        with self._lock:
            return self._latest

    @property
    def poll_interval_s(self) -> float | None:
        with self._lock:
            return self._interval

    def is_stale(self, now: float | None = None) -> bool:
        current = self._clock() if now is None else now
        return _is_stale(self.latest, current, self._config.stale_after_s)

    def start(self) -> None:
        log.debug("Simulated GPS started")

    def stop(self) -> None:
        log.debug("Simulated GPS stopped")

    def set_poll_interval(self, interval_s: float | None) -> None:
        _check_poll_interval(interval_s)
        with self._lock:
            self._interval = interval_s

    def set_fix(
        self,
        latitude: float,
        longitude: float,
        *,
        hdop: float = 0.9,
        satellites: int = 12,
        speed_mps: float | None = None,
        course_deg: float | None = None,
    ) -> None:
        if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
            raise ValueError(f"Invalid position: latitude {latitude}, longitude {longitude}")
        fix = GpsFix(
            timestamp=self._clock(),
            latitude=latitude,
            longitude=longitude,
            speed_mps=speed_mps,
            course_deg=course_deg,
            hdop=hdop,
            satellites_used=satellites,
            fix_quality=1,
        )
        self._report(fix)

    def set_no_fix(self) -> None:
        self._report(GpsFix(timestamp=self._clock(), latitude=None, longitude=None))

    def _report(self, fix: GpsFix) -> None:
        with self._lock:
            if self._interval is None:
                log.debug("Simulated GPS is off; dropped %s", fix)
                return
            self._latest = fix
