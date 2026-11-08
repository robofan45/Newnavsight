from __future__ import annotations

import pytest

from navsight.config import NavSightConfig
from navsight.hardware.gpio import FakeGpio
from navsight.util import ManualClock


@pytest.fixture
def config() -> NavSightConfig:
    return NavSightConfig()


@pytest.fixture
def clock() -> ManualClock:
    return ManualClock(1000.0)


@pytest.fixture
def gpio() -> FakeGpio:
    return FakeGpio()
