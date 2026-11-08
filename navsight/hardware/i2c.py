"""The shared I2C bus (GPIO 2 SDA, GPIO 3 SCL) used by the ToF pair, IMU and power monitor."""

from __future__ import annotations

import threading
from typing import Any

# Every sensor runs on its own thread. Hold this lock around each driver call so multi-step
# exchanges (such as BNO085 packet reads) never interleave with other traffic on the bus.
I2C_LOCK = threading.RLock()


def open_i2c(bus_number: int = 1) -> Any:
    """Open the bus with Adafruit Blinka (imported lazily: only available on the Pi).

    The bus speed comes from the device tree: set ``dtparam=i2c_arm_baudrate=400000``.
    """
    if bus_number == 1:
        import board
        import busio

        return busio.I2C(board.SCL, board.SDA)
    from adafruit_extended_bus import ExtendedI2C

    return ExtendedI2C(bus_number)
