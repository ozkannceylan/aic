#
# Copyright (C) 2026 Intrinsic Innovation LLC / Ozkan Ceylan (egeliler).
# Licensed under the Apache License, Version 2.0.
#

"""SafetyWatchdog — G4 enforcement.

Polls compensated wrench magnitude in a background thread at 20 Hz. If the
linear force component exceeds the 18 N soft cap, sets ``orchestrator.abort_flag``
so every state-machine transition can preempt cleanly before the controller's
20 N hard cap fires (which causes the engine to abort the trial outside our
control).

Constants are pinned per ARCHITECTURE.md §1 (G4) and CLAUDE.md project section.
Raising ``SOFT_CAP_N`` requires an ADR — see CLAUDE.md "Hard Guardrails".
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import numpy as np


SOFT_CAP_N: float = 18.0     # G4 soft cap. Controller's hard cap is 20 N.
POLL_RATE_HZ: float = 20.0   # ARCHITECTURE.md §1 — fast enough to catch a 1 N/tick
                             # rise toward the 20 N hard cap.

CompensatedWrenchGetter = Callable[[], Optional[np.ndarray]]
BreachCallback = Callable[[float], None]


class SafetyWatchdog:
    """Background-thread force-magnitude watchdog.

    The watchdog never reads ROS topics directly. The orchestrator owns the
    observation subscription and supplies a callable that returns the latest
    compensated wrench (raw_wrench - fts_tare_offset) as a length-6 ndarray
    [fx, fy, fz, tx, ty, tz], or None if no observation has arrived yet.

    Usage:
        watchdog = SafetyWatchdog(
            get_compensated_wrench=lambda: orchestrator.last_observation.compensated_wrench,
            on_breach=lambda mag: orchestrator.set_abort("force breach"),
            logger=node.get_logger(),
        )
        watchdog.start()
        ...
        watchdog.stop()

    The on_breach callback is invoked exactly once per ``start()`` cycle: after
    the first breach the watchdog stops polling until ``reset()`` is called
    (typically at the start of the next trial). This avoids spamming the
    orchestrator with redundant aborts.
    """

    def __init__(
        self,
        get_compensated_wrench: CompensatedWrenchGetter,
        on_breach: BreachCallback,
        soft_cap_n: float = SOFT_CAP_N,
        poll_rate_hz: float = POLL_RATE_HZ,
        logger=None,
    ) -> None:
        self._get_wrench = get_compensated_wrench
        self._on_breach = on_breach
        self._soft_cap_n = soft_cap_n
        self._period_s = 1.0 / poll_rate_hz
        self._logger = logger

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._breached = False  # one-shot per cycle

    # ----- Public lifecycle -------------------------------------------------

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._breached = False
        self._thread = threading.Thread(
            target=self._loop, name="SafetyWatchdog", daemon=True,
        )
        self._thread.start()

    def stop(self, timeout_s: float = 1.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
            self._thread = None

    def reset(self) -> None:
        """Clear the breach latch so the watchdog can fire again next trial."""
        self._breached = False

    @property
    def has_breached(self) -> bool:
        return self._breached

    # ----- Internal loop ----------------------------------------------------

    def _loop(self) -> None:
        next_tick = time.monotonic() + self._period_s
        while not self._stop_event.is_set():
            if not self._breached:
                self._tick_once()
            now = time.monotonic()
            sleep_s = next_tick - now
            if sleep_s > 0:
                if self._stop_event.wait(timeout=sleep_s):
                    return
            next_tick = max(next_tick + self._period_s, time.monotonic())

    def _tick_once(self) -> None:
        try:
            wrench = self._get_wrench()
        except Exception as exc:
            if self._logger is not None:
                self._logger.warning(
                    f"SafetyWatchdog: get_compensated_wrench raised: {exc}; "
                    "treating as missing observation."
                )
            return
        if wrench is None:
            return
        # G4: only the linear component (fx, fy, fz) matters for the soft cap.
        # Torques scale with moment arm and are bounded by joint limits anyway.
        force_mag = float(np.linalg.norm(wrench[:3]))
        if force_mag > self._soft_cap_n:
            self._breached = True
            if self._logger is not None:
                self._logger.error(
                    f"G4 BREACH: |compensated_force|={force_mag:.2f} N "
                    f"> soft cap {self._soft_cap_n:.1f} N. Setting abort_flag."
                )
            try:
                self._on_breach(force_mag)
            except Exception as exc:
                if self._logger is not None:
                    self._logger.error(
                        f"SafetyWatchdog on_breach callback raised: {exc}; "
                        "watchdog stays latched, orchestrator must abort."
                    )
