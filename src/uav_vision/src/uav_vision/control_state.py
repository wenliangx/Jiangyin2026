"""Thread-safe enable state with an optional delayed disable window."""

import math
import threading
import time


class DelayedDisableState:
    """Apply enable requests immediately and delay only disable requests."""

    def __init__(
        self,
        initial_enabled=False,
        disable_delay_seconds=0.0,
        monotonic=None,
    ):
        delay = float(disable_delay_seconds)
        if not math.isfinite(delay) or delay < 0.0:
            raise ValueError(
                "disable_delay_seconds must be finite and non-negative"
            )
        self._disable_delay_seconds = delay
        self._clock = monotonic or time.monotonic
        self._requested_enabled = bool(initial_enabled)
        self._disable_deadline = None
        self._lock = threading.Lock()

    @property
    def requested_enabled(self):
        with self._lock:
            return self._requested_enabled

    @property
    def effective_enabled(self):
        with self._lock:
            return self._effective_enabled_locked(self._clock())

    @property
    def grace_active(self):
        with self._lock:
            return (
                not self._requested_enabled
                and self._disable_deadline is not None
                and self._clock() < self._disable_deadline
            )

    @property
    def grace_remaining_seconds(self):
        with self._lock:
            if self._requested_enabled or self._disable_deadline is None:
                return 0.0
            return max(0.0, self._disable_deadline - self._clock())

    def _effective_enabled_locked(self, now):
        if self._requested_enabled:
            return True
        return (
            self._disable_deadline is not None
            and now < self._disable_deadline
        )

    def update(self, enabled):
        """Update the external request and return ``(changed, requested)``."""
        requested = bool(enabled)
        with self._lock:
            if requested == self._requested_enabled:
                return False, requested
            self._requested_enabled = requested
            if requested:
                self._disable_deadline = None
            elif self._disable_delay_seconds > 0.0:
                self._disable_deadline = (
                    self._clock() + self._disable_delay_seconds
                )
            else:
                self._disable_deadline = None
            return True, requested

    def run_if_effectively_enabled(self, action):
        """Run a short action atomically with the final effective-state check."""
        with self._lock:
            if not self._effective_enabled_locked(self._clock()):
                return False
            action()
            return True

    def run_if_requested_enabled(self, action):
        """Run a short action only while the external request is enabled."""
        with self._lock:
            if not self._requested_enabled:
                return False
            action()
            return True
