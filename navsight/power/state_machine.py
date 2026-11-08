"""Battery- and activity-driven power states (ACTIVE > LOW_POWER > SLEEP) and their sensor rates."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from navsight.config import NavSightConfig
from navsight.types import PowerProfile, PowerState
from navsight.util import Clock

log = logging.getLogger(__name__)

_Transition = tuple[PowerState, str]  # the new state and the reason, for the log


@dataclass(frozen=True)
class PowerInputs:
    now: float
    state_of_charge: float | None  # 0..1; None = unknown
    moving: bool
    button_pressed: bool
    sos_active: bool


class PowerStateMachine:
    """Picks the power state from the battery level, user activity and SOS.

    Motion or a button press is activity. ACTIVE drops to LOW_POWER when the battery falls below
    ``low_soc`` or after ``idle_to_low_power_s`` without activity. LOW_POWER returns to ACTIVE on
    activity once the battery is unknown or at least ``low_soc + soc_hysteresis``, and sleeps after
    ``idle_to_sleep_s`` without activity unless an SOS is active. SLEEP wakes on activity or an
    SOS. The battery level never causes sleep on its own, so obstacle sensing stays on while the
    user walks however low the battery is. At most one transition happens per update.
    """

    def __init__(self, config: NavSightConfig, *, clock: Clock = time.monotonic) -> None:
        self._config = config
        self._state = PowerState.ACTIVE
        self._last_activity = clock()  # switching on counts as activity
        self._last_transition: tuple[PowerState, PowerState] | None = None

    @property
    def state(self) -> PowerState:
        return self._state

    @property
    def last_transition(self) -> tuple[PowerState, PowerState] | None:
        """``(old, new)`` if the latest update changed the state, else None."""
        return self._last_transition

    def update(self, inputs: PowerInputs) -> PowerProfile:
        active = inputs.moving or inputs.button_pressed
        if active:
            self._last_activity = inputs.now
        idle_s = inputs.now - self._last_activity
        if self._state is PowerState.ACTIVE:
            transition = self._from_active(inputs, idle_s)
        elif self._state is PowerState.LOW_POWER:
            transition = self._from_low_power(inputs, active, idle_s)
        else:
            transition = self._from_sleep(inputs, active)
        self._last_transition = None
        if transition is not None:
            new_state, reason = transition
            log.info("Power state %s -> %s: %s", self._state.value, new_state.value, reason)
            self._last_transition = (self._state, new_state)
            self._state = new_state
        return self.profile_for(self._state)

    def profile_for(self, state: PowerState) -> PowerProfile:
        """Sensor rates for a state. The IMU is not part of the profile: it always runs."""
        config = self._config
        power = config.power
        if state is PowerState.ACTIVE:
            return PowerProfile(
                state=state,
                camera_fps=config.camera.fps,
                depth_enabled=config.depth.enabled,
                tof_rate_hz=config.tof.rate_hz,
                gps_poll_interval_s=0.0,
                fusion_rate_hz=config.fusion.rate_hz,
            )
        if state is PowerState.LOW_POWER:
            return PowerProfile(
                state=state,
                camera_fps=power.low_power_camera_fps,
                depth_enabled=False,
                tof_rate_hz=power.low_power_tof_hz,
                gps_poll_interval_s=power.low_power_gps_interval_s,
                fusion_rate_hz=power.low_power_fusion_hz,
            )
        return PowerProfile(
            state=state,
            camera_fps=0.0,
            depth_enabled=False,
            tof_rate_hz=0.0,
            gps_poll_interval_s=None,
            fusion_rate_hz=power.sleep_fusion_hz,
        )

    def _from_active(self, inputs: PowerInputs, idle_s: float) -> _Transition | None:
        power = self._config.power
        soc = inputs.state_of_charge
        if soc is not None and soc < power.low_soc:
            return PowerState.LOW_POWER, f"battery at {soc:.0%}"
        if idle_s >= power.idle_to_low_power_s:
            return PowerState.LOW_POWER, f"no activity for {idle_s:.0f} s"
        return None

    def _from_low_power(
        self, inputs: PowerInputs, active: bool, idle_s: float
    ) -> _Transition | None:
        if active and self._battery_allows_active(inputs.state_of_charge):
            return PowerState.ACTIVE, "activity"
        if idle_s >= self._config.power.idle_to_sleep_s and not inputs.sos_active:
            return PowerState.SLEEP, f"no activity for {idle_s:.0f} s"
        return None

    def _from_sleep(self, inputs: PowerInputs, active: bool) -> _Transition | None:
        if not (active or inputs.sos_active):
            return None
        reason = "activity" if active else "SOS"
        if self._battery_allows_active(inputs.state_of_charge):
            return PowerState.ACTIVE, reason
        return PowerState.LOW_POWER, f"{reason} with a low battery"

    def _battery_allows_active(self, soc: float | None) -> bool:
        power = self._config.power
        return soc is None or soc >= power.low_soc + power.soc_hysteresis
