from __future__ import annotations

import logging
import random
import string
import sys
import threading
import types
from collections import deque
from collections.abc import Callable

import pytest

from navsight.config import GpsConfig
from navsight.hardware.gps import (
    GpsReader,
    NmeaParser,
    SimulatedGps,
    build_ubx,
    nmea_checksum_ok,
    nmea_with_checksum,
    parse_lat_lon,
    ubx_backup_request,
)
from navsight.types import GpsFix
from navsight.util import ManualClock

LOGGER = "navsight.hardware.gps"
TALKERS = ("GP", "GN", "GL", "GA", "GB", "GQ")
KNOTS = 1852.0 / 3600.0
LAT = 48.1173  # 4807.038 N
LON = 11.516667  # 01131.000 E
WAKE = b"\xff"
BACKUP = ubx_backup_request(0)

CLASSIC_GGA = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
CLASSIC_RMC = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
NO_FIX_GGA = nmea_with_checksum("GPGGA,123520,,,,,0,00,99.99,,,,,,")


def gga(
    talker: str = "GP",
    *,
    quality: str = "1",
    lat: str = "4807.038",
    ns: str = "N",
    lon: str = "01131.000",
    ew: str = "E",
    sats: str = "08",
    hdop: str = "0.9",
    alt: str = "545.4",
) -> str:
    return nmea_with_checksum(
        f"{talker}GGA,123519,{lat},{ns},{lon},{ew},{quality},{sats},{hdop},{alt},M,46.9,M,,"
    )


def rmc(
    talker: str = "GP",
    *,
    status: str = "A",
    lat: str = "4807.038",
    ns: str = "N",
    lon: str = "01131.000",
    ew: str = "E",
    speed: str = "022.4",
    course: str = "084.4",
) -> str:
    return nmea_with_checksum(
        f"{talker}RMC,123519,{status},{lat},{ns},{lon},{ew},{speed},{course},230394,003.1,W"
    )


def gsa(talker: str = "GN", *, hdop: str = "1.3") -> str:
    satellites = ["04", "05", "", "09", "12", "", "", "24", "", "", "", ""]
    return nmea_with_checksum(f"{talker}GSA,A,3,{','.join(satellites)},2.5,{hdop},2.1")


def gsv(talker: str = "GP", total: str = "12", *, part: str = "1") -> str:
    return nmea_with_checksum(f"{talker}GSV,3,{part},{total},01,40,083,46,02,17,308,41")


def xor_of(body: str) -> int:
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return checksum


def fletcher(data: bytes) -> bytes:
    ck_a = ck_b = 0
    for byte in data:
        ck_a = (ck_a + byte) % 256
        ck_b = (ck_b + ck_a) % 256
    return bytes((ck_a, ck_b))


@pytest.fixture
def parser(clock: ManualClock) -> NmeaParser:
    return NmeaParser(clock=clock)


# --- NMEA framing -------------------------------------------------------------------------


@pytest.mark.parametrize(
    "sentence",
    [
        CLASSIC_GGA,
        CLASSIC_RMC,
        CLASSIC_GGA + "\r\n",
        "  " + CLASSIC_RMC + "\n",
        CLASSIC_RMC[:-2] + "6a",
    ],
)
def test_checksum_accepts_reference_sentences(sentence: str) -> None:
    assert nmea_checksum_ok(sentence)


@pytest.mark.parametrize(
    "sentence",
    [
        CLASSIC_GGA[:-2] + "48",
        CLASSIC_GGA.replace("4807.038", "4807.039"),
        CLASSIC_GGA[:-3],
        CLASSIC_GGA[1:],
        CLASSIC_GGA[:-1],
        CLASSIC_GGA + "0",
        CLASSIC_GGA[:-2] + "4G",
        CLASSIC_GGA[:-2] + "0x",
        CLASSIC_GGA + "*47",
        "",
        "$",
    ],
)
def test_checksum_rejects_bad_sentences(sentence: str) -> None:
    assert not nmea_checksum_ok(sentence)


def test_checksum_rejects_non_ascii_even_when_the_xor_matches() -> None:
    body = "GPTXT,Ñ"
    assert xor_of(body) <= 0xFF
    assert not nmea_checksum_ok(f"${body}*{xor_of(body):02X}")


def test_nmea_with_checksum_frames_a_body() -> None:
    assert nmea_with_checksum("PUBX,40,GLL,0,0,0,0,0,0") == "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n"
    assert nmea_with_checksum(CLASSIC_GGA[1:-3]) == CLASSIC_GGA + "\r\n"
    assert nmea_with_checksum("AB") == "$AB*03\r\n"


@pytest.mark.parametrize("body", ["$GPGGA,1", "GPGGA,1*00", "GPGGA,1\r\n", "GPTXT,Ñ"])
def test_nmea_with_checksum_rejects_framing_characters(body: str) -> None:
    with pytest.raises(ValueError):
        nmea_with_checksum(body)


# --- parse_lat_lon ------------------------------------------------------------------------


@pytest.mark.parametrize(
    "value, hemisphere, expected",
    [
        ("4807.038", "N", 48.1173),
        ("01131.000", "E", 11.516667),
        ("4807.038", "S", -48.1173),
        ("01131.000", "W", -11.516667),
        ("4717.11437", "N", 47.2852395),
        ("00833.91522", "E", 8.565253667),
        ("4807", "N", 48.116667),
        ("0000.000", "N", 0.0),
        ("9000.000", "S", -90.0),
        ("18000.000", "W", -180.0),
        (" 4807.038 ", "n", 48.1173),
    ],
)
def test_parse_lat_lon(value: str, hemisphere: str, expected: float) -> None:
    assert parse_lat_lon(value, hemisphere) == pytest.approx(expected, abs=1e-6)


@pytest.mark.parametrize("value, hemisphere", [("", ""), ("", "N"), ("  ", "E")])
def test_parse_lat_lon_empty_is_none(value: str, hemisphere: str) -> None:
    assert parse_lat_lon(value, hemisphere) is None


@pytest.mark.parametrize(
    "value, hemisphere",
    [
        ("48x7.038", "N"),
        ("4807.038", "Q"),
        ("4807.038", ""),
        ("4860.000", "N"),
        ("9000.001", "N"),
        ("18000.001", "E"),
        ("07.038", "N"),
        ("-4807.038", "N"),
        ("4807.0_38", "N"),
        ("4807.03e1", "N"),
        ("48 07.038", "N"),
    ],
)
def test_parse_lat_lon_rejects_malformed_values(value: str, hemisphere: str) -> None:
    with pytest.raises(ValueError):
        parse_lat_lon(value, hemisphere)


# --- NmeaParser: GGA and RMC --------------------------------------------------------------


@pytest.mark.parametrize("talker", TALKERS)
def test_gga_from_any_talker(talker: str, parser: NmeaParser, clock: ManualClock) -> None:
    fix = parser.feed(gga(talker))
    assert fix is not None
    assert fix.latitude == pytest.approx(LAT)
    assert fix.longitude == pytest.approx(LON)
    assert fix.fix_quality == 1
    assert fix.satellites_used == 8
    assert fix.hdop == 0.9
    assert fix.altitude_m == 545.4
    assert fix.timestamp == clock()
    assert fix.has_fix
    assert parser.latest is fix
    assert parser.parse_errors == 0


@pytest.mark.parametrize("talker", TALKERS)
def test_rmc_from_any_talker(talker: str, parser: NmeaParser) -> None:
    fix = parser.feed(rmc(talker))
    assert fix is not None
    assert fix.latitude == pytest.approx(LAT)
    assert fix.longitude == pytest.approx(LON)
    assert fix.speed_mps == pytest.approx(22.4 * KNOTS)
    assert fix.course_deg == pytest.approx(84.4)
    assert parser.parse_errors == 0


def test_reference_sentences_parse(parser: NmeaParser) -> None:
    parser.feed(CLASSIC_GGA)
    fix = parser.feed(CLASSIC_RMC)
    assert fix is not None
    assert fix.has_fix
    assert fix.altitude_m == 545.4
    assert fix.speed_mps == pytest.approx(11.523556)


def test_southern_and_western_hemispheres_are_negative(parser: NmeaParser) -> None:
    fix = parser.feed(gga(ns="S", ew="W"))
    assert fix is not None
    assert fix.latitude == pytest.approx(-LAT)
    assert fix.longitude == pytest.approx(-LON)


def test_gga_quality_zero_clears_the_position(parser: NmeaParser) -> None:
    parser.feed(gga())
    fix = parser.feed(NO_FIX_GGA)
    assert fix is not None
    assert fix.latitude is None
    assert fix.longitude is None
    assert fix.fix_quality == 0
    assert fix.satellites_used == 0
    assert fix.hdop == 99.99
    assert fix.altitude_m is None
    assert not fix.has_fix


def test_gga_quality_zero_clears_a_reported_position(parser: NmeaParser) -> None:
    fix = parser.feed(gga(quality="0"))
    assert fix is not None
    assert fix.latitude is None
    assert fix.longitude is None
    assert not fix.has_fix


def test_gga_keeps_a_dgps_quality(parser: NmeaParser) -> None:
    fix = parser.feed(gga(quality="2"))
    assert fix is not None
    assert fix.fix_quality == 2


def test_rmc_void_clears_the_position(parser: NmeaParser) -> None:
    parser.feed(gga())
    parser.feed(rmc())
    fix = parser.feed(rmc(status="V", lat="", ns="", lon="", ew="", speed="", course=""))
    assert fix is not None
    assert fix.fix_quality == 0
    assert fix.latitude is None
    assert fix.longitude is None
    assert fix.speed_mps is None
    assert fix.course_deg is None
    assert not fix.has_fix


def test_rmc_void_clears_a_reported_position(parser: NmeaParser) -> None:
    parser.feed(gga())
    fix = parser.feed(rmc(status="V"))
    assert fix is not None
    assert fix.latitude is None
    assert fix.fix_quality == 0


def test_rmc_valid_without_gga_raises_quality_to_1(parser: NmeaParser) -> None:
    fix = parser.feed(rmc())
    assert fix is not None
    assert fix.fix_quality == 1
    assert fix.has_fix
    assert fix.satellites_used == 0
    assert fix.hdop is None


def test_rmc_valid_after_a_no_fix_gga_raises_quality_to_1(parser: NmeaParser) -> None:
    parser.feed(NO_FIX_GGA)
    fix = parser.feed(rmc())
    assert fix is not None
    assert fix.fix_quality == 1


def test_rmc_keeps_a_higher_gga_quality(parser: NmeaParser) -> None:
    parser.feed(gga(quality="2"))
    fix = parser.feed(rmc())
    assert fix is not None
    assert fix.fix_quality == 2


def test_rmc_valid_without_a_position_keeps_quality_0(parser: NmeaParser) -> None:
    fix = parser.feed(rmc(lat="", ns="", lon="", ew=""))
    assert fix is not None
    assert fix.fix_quality == 0
    assert fix.latitude is None
    assert not fix.has_fix


def test_rmc_empty_speed_and_course_are_none(parser: NmeaParser) -> None:
    fix = parser.feed(rmc(speed="", course=""))
    assert fix is not None
    assert fix.speed_mps is None
    assert fix.course_deg is None


def test_rmc_course_is_wrapped_to_a_compass_heading(parser: NmeaParser) -> None:
    fix = parser.feed(rmc(course="360.0"))
    assert fix is not None
    assert fix.course_deg == 0.0


def test_fix_merges_rmc_and_gga_fields(parser: NmeaParser, clock: ManualClock) -> None:
    parser.feed(rmc())
    clock.advance(0.1)
    fix = parser.feed(gga(alt="12.5", sats="11"))
    assert fix is not None
    assert fix.speed_mps == pytest.approx(22.4 * KNOTS)
    assert fix.course_deg == pytest.approx(84.4)
    assert fix.altitude_m == 12.5
    assert fix.satellites_used == 11
    assert fix.timestamp == clock()


# --- NmeaParser: GSA and GSV --------------------------------------------------------------


def test_gsa_updates_hdop_for_the_next_fix(parser: NmeaParser) -> None:
    first = parser.feed(gga(hdop="0.9"))
    assert parser.feed(gsa(hdop="1.3")) is None
    assert parser.latest is first
    fix = parser.feed(rmc())
    assert fix is not None
    assert fix.hdop == 1.3


def test_gsa_without_hdop_clears_it(parser: NmeaParser) -> None:
    parser.feed(gga())
    parser.feed(gsa(hdop=""))
    fix = parser.feed(rmc())
    assert fix is not None
    assert fix.hdop is None


def test_gsv_updates_satellites_in_view(parser: NmeaParser) -> None:
    assert parser.satellites_in_view == 0
    assert parser.feed(gsv("GP", "12")) is None
    assert parser.satellites_in_view == 12
    assert parser.latest is None


def test_gsv_sums_constellations_and_restarts_each_epoch(parser: NmeaParser) -> None:
    parser.feed(gsv("GP", "12", part="1"))
    parser.feed(gsv("GP", "12", part="2"))
    parser.feed(gsv("GL", "08"))
    parser.feed(gsv("GA", "05"))
    assert parser.satellites_in_view == 25
    parser.feed(gga())
    parser.feed(gsv("GP", "10"))
    assert parser.satellites_in_view == 10
    parser.feed(gsv("GB", "04"))
    assert parser.satellites_in_view == 14


def m9n_epoch(gps_in_view: str, glonass_in_view: str | None) -> list[str]:
    """One second of NEO-M9N output (NMEA 4.10, four constellations), in its usual order."""
    used = ["05", "29", "26", "31", "16", "21"] + [""] * 6
    bodies = [
        "GNRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A,V",
        "GNVTG,77.52,T,,M,0.004,N,0.008,K,A",
        "GNGGA,083559.00,4717.11437,N,00833.91522,E,1,08,1.01,499.6,M,48.0,M,,",
        f"GNGSA,A,3,{','.join(used)},1.94,1.01,1.66,1",
        f"GPGSV,3,1,{gps_in_view},05,47,286,40,16,27,063,38,18,44,095,40,21,26,200,36,1",
        f"GPGSV,3,2,{gps_in_view},26,73,059,45,29,58,173,47,31,30,306,42,10,03,165,,1",
        "GAGSV,2,1,06,02,31,108,41,07,24,045,39,08,63,318,44,13,50,062,40,7",
        "GBGSV,1,1,04,11,46,140,43,12,18,056,37,23,33,296,40,25,10,180,,1",
        "GNGLL,4717.11437,N,00833.91522,E,083559.00,A,A",
    ]
    if glonass_in_view is not None:
        bodies.insert(6, f"GLGSV,1,1,{glonass_in_view},72,40,330,38,82,25,050,35,71,05,270,,1")
    return [nmea_with_checksum(body) for body in bodies]


def test_parses_a_full_m9n_epoch(parser: NmeaParser) -> None:
    fixes = [fix for fix in map(parser.feed, m9n_epoch("10", "03")) if fix is not None]
    assert len(fixes) == 2
    fix = parser.latest
    assert fix is fixes[-1]
    assert fix.latitude == pytest.approx(47.2852395)
    assert fix.longitude == pytest.approx(8.565253667)
    assert fix.fix_quality == 1
    assert fix.satellites_used == 8
    assert fix.hdop == 1.01
    assert fix.altitude_m == 499.6
    assert fix.speed_mps == pytest.approx(0.004 * KNOTS)
    assert fix.course_deg == pytest.approx(77.52)
    assert parser.satellites_in_view == 10 + 3 + 6 + 4
    assert parser.parse_errors == 0

    for line in m9n_epoch("09", None):
        parser.feed(line)
    assert parser.satellites_in_view == 9 + 6 + 4


# --- NmeaParser: what feed returns and ignores --------------------------------------------


def test_feed_returns_a_fix_only_after_gga_or_rmc(parser: NmeaParser) -> None:
    results = [parser.feed(line) for line in (gsa(), gsv(), gga(), gsa(), rmc(), gsv())]
    assert [result is not None for result in results] == [False, False, True, False, True, False]


@pytest.mark.parametrize(
    "body",
    [
        "GNVTG,77.52,T,,M,0.004,N,0.008,K,A",
        "GNGLL,4717.11437,N,00833.91522,E,083559.00,A,A",
        "GNTXT,01,01,02,u-blox AG - www.u-blox.com",
        "GNZDA,082710.00,16,09,2002,00,00",
        "PUBX,00,081350.00,4717.113210,N,00833.915187,E,546.589,G3,2.1,2.0,0.007,77.52",
    ],
)
def test_other_sentences_are_ignored(body: str, parser: NmeaParser) -> None:
    assert parser.feed(nmea_with_checksum(body)) is None
    assert parser.latest is None
    assert parser.parse_errors == 0


@pytest.mark.parametrize(
    "line", ["", "\r\n", "garbage", CLASSIC_GGA[1:], ",01131.000,E,1,08,0.9*47", "\x00\xb5b\x01"]
)
def test_lines_without_a_dollar_are_ignored(line: str, parser: NmeaParser) -> None:
    assert parser.feed(line) is None
    assert parser.parse_errors == 0


def test_bad_checksums_count_parse_errors(parser: NmeaParser) -> None:
    assert parser.feed(CLASSIC_GGA[:-2] + "00") is None
    assert parser.feed(CLASSIC_GGA.replace("4807.038", "4807.039")) is None
    assert parser.feed(CLASSIC_GGA[:-3]) is None
    assert parser.parse_errors == 3
    assert parser.latest is None


MALFORMED_BODIES = [
    "GPGGA,123519,4807.038,N,01131.000,E,x,08,0.9,545.4,M,46.9,M,,",
    "GPGGA,123519,48x7.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",
    "GPGGA,123519,4807.038,Q,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",
    "GPGGA,123519,9107.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",
    "GPGGA,123519,4807.038,N,,,1,08,0.9,545.4,M,46.9,M,,",
    "GPGGA,123519,4807.038,N,01131.000,E,1,-8,0.9,545.4,M,46.9,M,,",
    "GPGGA,123519,4807.038,N,01131.000,E,1,08,abc,545.4,M,46.9,M,,",
    "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,nan,M,46.9,M,,",
    "GPGGA,123519,4807.038,N",
    "GPGGA",
    "GPRMC,123519,X,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W",
    "GPRMC,123519,,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W",
    "GPRMC,123519,A,4807.038,N,01131.000,E,fast,084.4,230394,003.1,W",
    "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,inf,230394,003.1,W",
    "GPRMC,123519,A,4807.038,N",
    "GNGSA,A,3,04,05,,09,12,,,24,,,,,2.5,bad,2.1",
    "GNGSA,A,3,04,05",
    "GLGSV,3,1,many,01,40,083,46",
    "GLGSV,3,1",
]


@pytest.mark.parametrize("body", MALFORMED_BODIES)
def test_malformed_fields_count_parse_errors_and_keep_the_state(
    body: str, parser: NmeaParser
) -> None:
    parser.feed(gsv("GP", "12"))
    good = parser.feed(gga())
    assert parser.feed(nmea_with_checksum(body)) is None
    assert parser.parse_errors == 1
    assert parser.latest is good
    assert parser.satellites_in_view == 12
    after = parser.feed(rmc())
    assert after is not None
    assert after.altitude_m == 545.4
    assert after.hdop == 0.9
    assert after.satellites_used == 8
    assert after.fix_quality == 1


def test_feed_never_raises_on_random_input(parser: NmeaParser) -> None:
    rng = random.Random(2024)
    kinds = ["GPGGA", "GNGGA", "GPRMC", "GNRMC", "GNGSA", "GPGSV", "GLGSV", "GNVTG"]
    field_chars = "0123456789.-+eNSEWAVnaif "
    for _ in range(3000):
        fields = [
            "".join(rng.choice(field_chars) for _ in range(rng.randint(0, 8)))
            for _ in range(rng.randint(0, 20))
        ]
        parser.feed(nmea_with_checksum(",".join([rng.choice(kinds), *fields])))
    for _ in range(1000):
        parser.feed("$" + "".join(rng.choice(string.printable) for _ in range(rng.randint(0, 90))))
    assert parser.parse_errors > 0


# --- UBX ----------------------------------------------------------------------------------


def test_build_ubx_mon_ver_poll() -> None:
    assert build_ubx(0x0A, 0x04) == bytes.fromhex("b5620a0400000e34")


def test_build_ubx_frames_a_payload() -> None:
    payload = bytes(range(256)) + bytes(44)
    message = build_ubx(0x06, 0x8A, payload)
    assert message[:2] == b"\xb5\x62"
    assert message[2:4] == b"\x06\x8a"
    assert message[4:6] == b"\x2c\x01"  # 300, little-endian
    assert message[6:-2] == payload
    assert message[-2:] == fletcher(message[2:-2])


@pytest.mark.parametrize(
    "msg_class, msg_id, payload", [(256, 0, b""), (0, -1, b""), (0, 0, bytes(65536))]
)
def test_build_ubx_rejects_out_of_range_values(msg_class: int, msg_id: int, payload: bytes) -> None:
    with pytest.raises(ValueError):
        build_ubx(msg_class, msg_id, payload)


def test_ubx_backup_request_bytes() -> None:
    expected = bytes.fromhex(
        "b5 62 02 41 10 00"  # sync, RXM-PMREQ, 16-byte payload
        " 00 00 00 00"  # version 0, 3 reserved bytes
        " 00 00 00 00"  # duration 0 = until woken
        " 02 00 00 00"  # flags: backup
        " 08 00 00 00"  # wakeupSources: UART RX
        " 5d 4b"
    )
    assert ubx_backup_request() == expected
    assert ubx_backup_request(0) == expected


def test_ubx_backup_request_layout_with_a_duration() -> None:
    message = ubx_backup_request(5000)
    payload = message[6:-2]
    assert message[2:4] == b"\x02\x41"
    assert message[4:6] == b"\x10\x00"
    assert len(payload) == 16
    assert payload[0] == 0
    assert payload[1:4] == bytes(3)
    assert int.from_bytes(payload[4:8], "little") == 5000
    assert int.from_bytes(payload[8:12], "little") == 0x02
    assert int.from_bytes(payload[12:16], "little") == 0x08
    assert message[-2:] == fletcher(message[2:-2])


@pytest.mark.parametrize("duration_ms", [-1, 2**32])
def test_ubx_backup_request_rejects_bad_durations(duration_ms: int) -> None:
    with pytest.raises(ValueError):
        ubx_backup_request(duration_ms)


# --- GpsReader ----------------------------------------------------------------------------


class FakeSerialException(OSError):
    """Stands in for serial.SerialException, which is an OSError."""


class FakePort:
    def __init__(self, receiver: FakeReceiver) -> None:
        self._receiver = receiver
        self.is_open = True
        self.opened_at = receiver.clock()
        self.closed_at: float | None = None
        self.events: list[tuple[str, bytes]] = []
        self.read_error: Exception | None = None
        self.write_error: Exception | None = None

    @property
    def written(self) -> list[bytes]:
        return [data for event, data in self.events if event == "write"]

    def readline(self) -> bytes:
        assert self.is_open, "read from a closed port"
        if self.read_error is not None:
            error, self.read_error = self.read_error, None
            raise error
        if not self._receiver.lines:
            return b""
        return self._receiver.lines.popleft()

    def write(self, data: bytes) -> int:
        assert self.is_open, "write to a closed port"
        if self.write_error is not None:
            raise self.write_error
        self.events.append(("write", bytes(data)))
        return len(data)

    def close(self) -> None:
        self.is_open = False
        self.closed_at = self._receiver.clock()
        self.events.append(("close", b""))


class FakeReceiver:
    """A serial factory: each port it opens reads the receiver's queued output."""

    def __init__(self, clock: ManualClock) -> None:
        self.clock = clock
        self.lines: deque[bytes] = deque()
        self.ports: list[FakePort] = []
        self.open_errors: deque[Exception] = deque()
        self.always_fail: Exception | None = None

    def send(self, *lines: str | bytes) -> None:
        for line in lines:
            self.lines.append(line.encode("ascii") if isinstance(line, str) else line)

    def __call__(self) -> FakePort:
        if self.always_fail is not None:
            raise self.always_fail
        if self.open_errors:
            raise self.open_errors.popleft()
        port = FakePort(self)
        self.ports.append(port)
        return port

    @property
    def port(self) -> FakePort:
        return self.ports[-1]


class FakeSleep:
    """Stands in for time.sleep: records each call and advances the manual clock."""

    def __init__(self, clock: ManualClock) -> None:
        self.clock = clock
        self.calls: list[float] = []

    def __call__(self, seconds: float) -> None:
        self.calls.append(seconds)
        self.clock.advance(seconds)


@pytest.fixture
def receiver(clock: ManualClock) -> FakeReceiver:
    return FakeReceiver(clock)


@pytest.fixture
def sleep(clock: ManualClock) -> FakeSleep:
    return FakeSleep(clock)


def make_reader(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep, **overrides: object
) -> GpsReader:
    return GpsReader(GpsConfig(**overrides), serial_factory=receiver, clock=clock, sleep=sleep)


def step_once(reader: GpsReader, clock: ManualClock, dt: float) -> None:
    """One step; a step that did not sleep stands for a read that took ``dt``."""
    before = clock()
    reader.step()
    if clock() == before:
        clock.advance(dt)


def run_for(reader: GpsReader, clock: ManualClock, seconds: float, dt: float = 0.25) -> None:
    end = clock() + seconds
    while clock() < end:
        step_once(reader, clock, dt)


def run_until(
    reader: GpsReader, clock: ManualClock, condition: Callable[[], bool], dt: float = 0.25
) -> None:
    for _ in range(10_000):
        if condition():
            return
        step_once(reader, clock, dt)
    raise AssertionError("the condition never became true")


def test_reader_opens_the_port_lazily_and_feeds_the_parser(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    receiver.send(gga("GN"), rmc("GN"))
    reader = make_reader(receiver, clock, sleep)
    assert receiver.ports == []
    assert reader.latest is None
    reader.step()
    assert len(receiver.ports) == 1
    first = reader.latest
    assert first is not None
    assert first.has_fix
    assert first.altitude_m == 545.4
    clock.advance(0.5)
    reader.step()
    fix = reader.latest
    assert fix is not None
    assert fix.speed_mps == pytest.approx(22.4 * KNOTS)
    assert fix.timestamp == clock()


def test_continuous_mode_keeps_the_port_open(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    receiver.send(*[gga("GN")] * 100)
    reader = make_reader(receiver, clock, sleep)
    assert reader.poll_interval_s == 0
    run_for(reader, clock, 600.0)
    assert len(receiver.ports) == 1
    assert receiver.port.is_open
    assert sleep.calls == []
    assert not receiver.lines
    assert reader.latest is not None


def test_reader_survives_binary_and_non_ascii_lines(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    receiver.send(b"\xb5\x62\x01\x07\x5c\x00\x10\x0a", b"$GPGGA,\xff\xfe,*47\r\n", gga())
    reader = make_reader(receiver, clock, sleep)
    reader.step()
    reader.step()
    assert reader.latest is None
    reader.step()
    assert reader.latest is not None


def test_duty_cycle_listens_for_the_window_then_closes_for_the_interval(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep, listen_window_s=3.0)
    reader.set_poll_interval(10.0)
    start = clock()
    run_until(reader, clock, lambda: len(receiver.ports) == 3)
    first, second, third = receiver.ports
    assert first.opened_at == start
    assert first.closed_at == start + 3.0
    assert second.opened_at == start + 13.0
    assert second.closed_at == start + 16.0
    assert third.opened_at == start + 26.0
    assert third.is_open
    assert sum(sleep.calls) == pytest.approx(20.0)


def test_repeating_the_same_interval_does_not_extend_the_window(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep, listen_window_s=3.0)
    reader.set_poll_interval(10.0)
    start = clock()
    reader.step()
    for _ in range(12):
        reader.set_poll_interval(10.0)
        clock.advance(0.25)
        reader.step()
    assert receiver.port.closed_at == start + 3.0


def test_switching_from_continuous_to_duty_starts_a_listen_window(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep, listen_window_s=3.0)
    run_for(reader, clock, 60.0)
    reader.set_poll_interval(10.0)
    changed_at = clock()
    run_until(reader, clock, lambda: not receiver.port.is_open)
    assert len(receiver.ports) == 1
    assert receiver.port.closed_at == changed_at + 3.0


def test_switching_to_continuous_reopens_the_port_at_once(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep, listen_window_s=3.0)
    reader.set_poll_interval(10.0)
    run_until(reader, clock, lambda: len(receiver.ports) == 1 and not receiver.port.is_open)
    run_for(reader, clock, 2.0)
    assert len(receiver.ports) == 1
    reader.set_poll_interval(0.0)
    switched_at = clock()
    reader.step()
    assert len(receiver.ports) == 2
    assert receiver.port.opened_at == switched_at
    run_for(reader, clock, 60.0)
    assert len(receiver.ports) == 2
    assert receiver.port.is_open


def test_none_turns_the_gps_off_until_a_new_interval(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    receiver.send(gga())
    reader = make_reader(receiver, clock, sleep)
    reader.step()
    fix = reader.latest
    reader.set_poll_interval(None)
    assert reader.poll_interval_s is None
    run_for(reader, clock, 120.0)
    assert len(receiver.ports) == 1
    assert not receiver.port.is_open
    assert reader.latest is fix
    assert reader.is_stale()
    reader.set_poll_interval(0.0)
    reader.step()
    assert len(receiver.ports) == 2
    assert receiver.port.is_open


def test_ubx_power_save_sends_backup_before_closing_and_wakes_on_open(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep, use_ubx_power_save=True, listen_window_s=3.0)
    reader.set_poll_interval(10.0)
    run_until(reader, clock, lambda: len(receiver.ports) == 2)
    first, second = receiver.ports
    assert first.events == [("write", WAKE), ("write", BACKUP), ("close", b"")]
    assert second.events == [("write", WAKE)]


def test_ubx_power_save_when_the_gps_is_turned_off_and_on(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep, use_ubx_power_save=True)
    reader.step()
    reader.set_poll_interval(None)
    run_for(reader, clock, 5.0)
    assert len(receiver.ports) == 1
    assert receiver.port.events == [("write", WAKE), ("write", BACKUP), ("close", b"")]
    reader.set_poll_interval(0.0)
    reader.step()
    assert receiver.ports[1].events == [("write", WAKE)]


def test_stop_puts_the_receiver_into_backup_with_power_save(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep, use_ubx_power_save=True)
    reader.step()
    reader.stop()
    assert receiver.port.events == [("write", WAKE), ("write", BACKUP), ("close", b"")]


def test_without_power_save_nothing_is_written(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep, listen_window_s=3.0)
    reader.set_poll_interval(10.0)
    run_until(reader, clock, lambda: len(receiver.ports) == 2)
    reader.set_poll_interval(None)
    reader.step()
    reader.stop()
    assert all(port.written == [] for port in receiver.ports)
    assert not any(port.is_open for port in receiver.ports)


def test_backup_request_failure_is_logged_and_the_port_still_closes(
    receiver: FakeReceiver,
    clock: ManualClock,
    sleep: FakeSleep,
    caplog: pytest.LogCaptureFixture,
) -> None:
    reader = make_reader(receiver, clock, sleep, use_ubx_power_save=True)
    reader.step()
    receiver.port.write_error = FakeSerialException("write failed")
    reader.set_poll_interval(None)
    with caplog.at_level(logging.WARNING, logger=LOGGER):
        reader.step()
    assert not receiver.port.is_open
    assert "backup mode" in caplog.text


def test_serial_error_closes_the_port_and_reopens_2_s_later(
    receiver: FakeReceiver,
    clock: ManualClock,
    sleep: FakeSleep,
    caplog: pytest.LogCaptureFixture,
) -> None:
    caplog.set_level(logging.INFO, logger=LOGGER)
    receiver.send(gga())
    reader = make_reader(receiver, clock, sleep)
    reader.step()
    fix = reader.latest
    receiver.port.read_error = FakeSerialException("device disconnected")
    reader.step()
    failed_at = clock()
    assert not receiver.ports[0].is_open
    assert reader.latest is fix
    assert "device disconnected" in caplog.text
    run_until(reader, clock, lambda: len(receiver.ports) == 2)
    assert receiver.ports[1].opened_at == failed_at + 2.0
    assert "reopened" in caplog.text
    receiver.send(rmc(speed="001.0"))
    reader.step()
    latest = reader.latest
    assert latest is not None
    assert latest.speed_mps == pytest.approx(KNOTS)


def test_serial_error_close_skips_the_backup_request(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep, use_ubx_power_save=True)
    reader.step()
    receiver.port.read_error = FakeSerialException("device disconnected")
    reader.step()
    assert receiver.ports[0].events == [("write", WAKE), ("close", b"")]


def test_a_port_that_reports_closed_is_reopened_2_s_later(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep)
    reader.step()
    receiver.port.is_open = False
    reader.step()
    failed_at = clock()
    run_until(reader, clock, lambda: len(receiver.ports) == 2)
    assert receiver.ports[1].opened_at == failed_at + 2.0
    assert receiver.port.is_open


def test_a_failed_open_is_retried_every_2_s(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    receiver.open_errors.extend(
        [FakeSerialException("could not open port"), FakeSerialException("still busy")]
    )
    reader = make_reader(receiver, clock, sleep)
    start = clock()
    run_until(reader, clock, lambda: len(receiver.ports) == 1)
    assert receiver.ports[0].opened_at == start + 4.0


def test_serial_errors_are_logged_at_most_every_30_s(
    receiver: FakeReceiver,
    clock: ManualClock,
    sleep: FakeSleep,
    caplog: pytest.LogCaptureFixture,
) -> None:
    caplog.set_level(logging.DEBUG, logger=LOGGER)
    receiver.always_fail = FakeSerialException("could not open port")
    reader = make_reader(receiver, clock, sleep)
    run_for(reader, clock, 70.0)
    errors = [record for record in caplog.records if "serial error" in record.getMessage()]
    warnings = [record for record in errors if record.levelno == logging.WARNING]
    assert len(warnings) == 3  # at 0, 30 and 60 s
    assert len(errors) == 35  # one attempt every 2 s; the rest are logged at debug level


def test_is_stale_after_stale_after_s(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    receiver.send(gga())
    reader = make_reader(receiver, clock, sleep, stale_after_s=5.0)
    assert reader.is_stale()
    reader.step()
    fixed_at = clock()
    assert not reader.is_stale()
    assert not reader.is_stale(fixed_at + 5.0)
    assert reader.is_stale(fixed_at + 5.01)
    clock.advance(5.01)
    assert reader.is_stale()


def test_is_stale_is_about_the_age_of_the_data_not_the_fix(
    receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    receiver.send(NO_FIX_GGA)
    reader = make_reader(receiver, clock, sleep)
    reader.step()
    latest = reader.latest
    assert latest is not None
    assert not latest.has_fix
    assert not reader.is_stale()


@pytest.mark.parametrize("interval", [-1.0, float("nan"), float("inf")])
def test_set_poll_interval_rejects_invalid_values(
    interval: float, receiver: FakeReceiver, clock: ManualClock, sleep: FakeSleep
) -> None:
    reader = make_reader(receiver, clock, sleep)
    with pytest.raises(ValueError):
        reader.set_poll_interval(interval)
    assert reader.poll_interval_s == 0


def test_default_factory_opens_pyserial_with_the_config(
    monkeypatch: pytest.MonkeyPatch, clock: ManualClock, sleep: FakeSleep
) -> None:
    opened: list[tuple[str, int, float]] = []
    receiver = FakeReceiver(clock)

    def fake_serial(port: str, baud_rate: int, timeout: float) -> FakePort:
        opened.append((port, baud_rate, timeout))
        return receiver()

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=fake_serial))
    config = GpsConfig(port="/dev/ttyAMA0", baud_rate=9600, read_timeout_s=0.5)
    reader = GpsReader(config, clock=clock, sleep=sleep)
    assert opened == []
    reader.step()
    assert opened == [("/dev/ttyAMA0", 9600, 0.5)]


def test_default_factory_without_pyserial_says_how_to_fix_it(
    monkeypatch: pytest.MonkeyPatch, clock: ManualClock, sleep: FakeSleep
) -> None:
    monkeypatch.setitem(sys.modules, "serial", None)
    reader = GpsReader(GpsConfig(), clock=clock, sleep=sleep)
    with pytest.raises(RuntimeError, match="pyserial"):
        reader.step()


class LivePort:
    """A port for the threaded test: readline waits briefly when there is nothing to read."""

    def __init__(self, *lines: str) -> None:
        self._lines = deque(line.encode("ascii") for line in lines)
        self._closed = threading.Event()
        self.drained = threading.Event()
        self.is_open = True

    def readline(self) -> bytes:
        if self._lines:
            return self._lines.popleft()
        self.drained.set()
        self._closed.wait(0.01)
        return b""

    def write(self, data: bytes) -> int:
        return len(data)

    def close(self) -> None:
        self.is_open = False
        self._closed.set()


def test_start_runs_the_reader_thread_until_stop() -> None:
    port = LivePort(gga("GN"), rmc("GN"))
    reader = GpsReader(GpsConfig(), serial_factory=lambda: port)
    reader.start()
    reader.start()
    try:
        assert port.drained.wait(5.0)
        fix = reader.latest
        assert fix is not None
        assert fix.has_fix
        assert fix.speed_mps == pytest.approx(22.4 * KNOTS)
    finally:
        reader.stop()
    assert not port.is_open
    assert not any(thread.name == "gps-reader" for thread in threading.enumerate())


# --- SimulatedGps -------------------------------------------------------------------------


def test_simulated_gps_set_fix(clock: ManualClock) -> None:
    gps = SimulatedGps(GpsConfig(), clock=clock)
    assert gps.latest is None
    assert gps.is_stale()
    gps.set_fix(51.5, -0.12, speed_mps=1.3, course_deg=90.0)
    assert gps.latest == GpsFix(
        timestamp=clock(),
        latitude=51.5,
        longitude=-0.12,
        speed_mps=1.3,
        course_deg=90.0,
        hdop=0.9,
        satellites_used=12,
        fix_quality=1,
    )
    assert gps.latest.has_fix
    assert not gps.is_stale()


def test_simulated_gps_set_no_fix(clock: ManualClock) -> None:
    gps = SimulatedGps(GpsConfig(), clock=clock)
    gps.set_fix(51.5, -0.12, hdop=1.5, satellites=7)
    clock.advance(1.0)
    gps.set_no_fix()
    fix = gps.latest
    assert fix is not None
    assert fix.latitude is None
    assert fix.longitude is None
    assert fix.fix_quality == 0
    assert not fix.has_fix
    assert fix.timestamp == clock()


def test_simulated_gps_is_stale_after_stale_after_s(clock: ManualClock) -> None:
    gps = SimulatedGps(GpsConfig(stale_after_s=5.0), clock=clock)
    gps.set_fix(51.5, -0.12)
    fixed_at = clock()
    clock.advance(5.0)
    assert not gps.is_stale()
    clock.advance(0.1)
    assert gps.is_stale()
    assert not gps.is_stale(fixed_at + 1.0)


def test_simulated_gps_drops_reports_while_off(clock: ManualClock) -> None:
    gps = SimulatedGps(GpsConfig(), clock=clock)
    gps.set_fix(51.5, -0.12)
    before = gps.latest
    gps.set_poll_interval(None)
    assert gps.poll_interval_s is None
    clock.advance(1.0)
    gps.set_fix(52.0, 0.5)
    gps.set_no_fix()
    assert gps.latest is before
    gps.set_poll_interval(10.0)
    gps.set_fix(52.0, 0.5)
    latest = gps.latest
    assert latest is not None
    assert latest.latitude == 52.0


def test_simulated_gps_rejects_invalid_input(clock: ManualClock) -> None:
    gps = SimulatedGps(GpsConfig(), clock=clock)
    with pytest.raises(ValueError):
        gps.set_fix(91.0, 0.0)
    with pytest.raises(ValueError):
        gps.set_fix(0.0, -181.0)
    with pytest.raises(ValueError):
        gps.set_poll_interval(-5.0)
    assert gps.latest is None


def test_simulated_gps_start_and_stop_are_harmless(clock: ManualClock) -> None:
    gps = SimulatedGps(GpsConfig(), clock=clock)
    gps.start()
    gps.set_fix(1.0, 2.0)
    gps.stop()
    latest = gps.latest
    assert latest is not None
    assert latest.latitude == 1.0
