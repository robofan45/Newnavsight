"""GPIO access for the Raspberry Pi 5 header, plus an in-memory backend for tests and simulation."""

from __future__ import annotations

import logging
from typing import Any, Protocol

log = logging.getLogger(__name__)


class GpioBackend(Protocol):
    def claim_input(self, pin: int, pull_up: bool = True) -> None: ...

    def claim_output(self, pin: int, level: int = 0) -> None: ...

    def read(self, pin: int) -> int: ...

    def write(self, pin: int, level: int) -> None: ...

    def pwm(self, pin: int, frequency_hz: float, duty_fraction: float) -> None: ...

    def close(self) -> None: ...


class LgpioBackend:
    """lgpio on the Pi 5's RP1 header chip.

    The header is gpiochip4 on early Pi 5 kernels and gpiochip0 on newer ones, so with no chip
    given this opens the chip whose label mentions "rp1", trying 4 first as v1 did.
    """

    CANDIDATE_CHIPS = (4, 0)

    def __init__(self, chip: int | None = None) -> None:
        import lgpio  # only available on the Pi

        self._lgpio: Any = lgpio
        self._handle = self._open(chip)
        self._claimed: set[int] = set()

    def _open(self, chip: int | None) -> int:
        lgpio = self._lgpio
        candidates = (chip,) if chip is not None else self.CANDIDATE_CHIPS
        fallback: tuple[int, int] | None = None
        for number in candidates:
            try:
                handle = lgpio.gpiochip_open(number)
            except lgpio.error:
                continue
            try:
                label = str(lgpio.gpio_get_chip_info(handle)[-1])
            except lgpio.error:
                label = ""
            if chip is not None or "rp1" in label.lower():
                log.info("Using gpiochip%d (%s)", number, label)
                if fallback is not None:
                    lgpio.gpiochip_close(fallback[1])
                return handle
            if fallback is None:
                fallback = (number, handle)
            else:
                lgpio.gpiochip_close(handle)
        if fallback is not None:
            log.warning("No RP1 GPIO chip found; using gpiochip%d", fallback[0])
            return fallback[1]
        raise RuntimeError(f"Could not open a GPIO chip (tried {candidates})")

    def claim_input(self, pin: int, pull_up: bool = True) -> None:
        flags = self._lgpio.SET_PULL_UP if pull_up else self._lgpio.SET_PULL_NONE
        self._lgpio.gpio_claim_input(self._handle, pin, flags)
        self._claimed.add(pin)

    def claim_output(self, pin: int, level: int = 0) -> None:
        self._lgpio.gpio_claim_output(self._handle, pin, level)
        self._claimed.add(pin)

    def read(self, pin: int) -> int:
        return int(self._lgpio.gpio_read(self._handle, pin))

    def write(self, pin: int, level: int) -> None:
        self._lgpio.gpio_write(self._handle, pin, level)

    def pwm(self, pin: int, frequency_hz: float, duty_fraction: float) -> None:
        duty_percent = max(0.0, min(1.0, duty_fraction)) * 100.0
        self._lgpio.tx_pwm(self._handle, pin, frequency_hz, duty_percent)

    def close(self) -> None:
        for pin in sorted(self._claimed):
            try:
                self._lgpio.gpio_free(self._handle, pin)
            except self._lgpio.error:
                log.debug("Could not free GPIO %d", pin)
        self._claimed.clear()
        self._lgpio.gpiochip_close(self._handle)


class FakeGpio:
    """In-memory GPIO. Inputs with pull-ups read high (a released, active-low button)."""

    def __init__(self) -> None:
        self.inputs: dict[int, int] = {}
        self.outputs: dict[int, int] = {}
        self.pull_ups: dict[int, bool] = {}
        self.pwm_state: dict[int, tuple[float, float]] = {}
        self.writes: list[tuple[int, int]] = []
        self.closed = False

    def claim_input(self, pin: int, pull_up: bool = True) -> None:
        self.pull_ups[pin] = pull_up
        self.inputs.setdefault(pin, 1 if pull_up else 0)

    def claim_output(self, pin: int, level: int = 0) -> None:
        self.outputs[pin] = level
        self.writes.append((pin, level))

    def read(self, pin: int) -> int:
        if pin in self.inputs:
            return self.inputs[pin]
        if pin in self.outputs:
            return self.outputs[pin]
        raise RuntimeError(f"GPIO {pin} is not claimed")

    def write(self, pin: int, level: int) -> None:
        if pin not in self.outputs:
            raise RuntimeError(f"GPIO {pin} is not claimed as an output")
        self.outputs[pin] = level
        self.writes.append((pin, level))

    def pwm(self, pin: int, frequency_hz: float, duty_fraction: float) -> None:
        if pin not in self.outputs:
            raise RuntimeError(f"GPIO {pin} is not claimed as an output")
        self.pwm_state[pin] = (frequency_hz, duty_fraction)

    def set_input(self, pin: int, level: int) -> None:
        self.inputs[pin] = level

    def close(self) -> None:
        self.closed = True
