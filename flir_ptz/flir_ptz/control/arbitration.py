"""Pure control-source arbitration (spec ARCHITECTURE.md §3.5).

Authority for "who is allowed to drive the camera right now" lives in
the PTZ node (the only process that can actually enforce exclusion).
This module is the pure decision logic behind that: last-claim-wins,
no priority hierarchy, time-limited lease that reverts to "no owner"
on expiry, and a stop command that is **never** blocked regardless of
ownership.

No I/O, no clock reads: every method that cares about time takes an
explicit ``now: float`` (monotonic seconds).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ClaimResult:
    granted: bool
    owner: str
    expires_at: float
    reason: str


class Arbiter:
    """Last-claim-wins control-source arbiter with an expiring lease.

    There is no priority hierarchy: whoever claims most recently owns
    the lease. The lease is implicitly renewed by accepted commands
    (see ``renew``) and expires ``lease_s`` seconds after the last
    claim/renew, at which point ownership reverts to "" (no owner) with
    no heartbeat protocol required from clients.
    """

    def __init__(self, lease_s: float = 60.0) -> None:
        self._lease_s = lease_s
        self._owner: str = ""
        self._expires_at: float = 0.0

    def claim(self, source: str, now: float) -> ClaimResult:
        """Claim control. Last claimant always wins — no priority checks."""
        self._owner = source
        self._expires_at = now + self._lease_s
        return ClaimResult(
            granted=True,
            owner=self._owner,
            expires_at=self._expires_at,
            reason="claimed",
        )

    def release(self, source: str, now: float) -> ClaimResult:
        """Release control. A no-op (but not an error) if ``source`` isn't
        the current owner."""
        current = self.owner(now)
        if current != "" and source == current:
            self._owner = ""
            self._expires_at = 0.0
            return ClaimResult(granted=False, owner="", expires_at=0.0, reason="released")
        return ClaimResult(
            granted=False,
            owner=current,
            expires_at=self._expires_at if current else 0.0,
            reason="not_owner",
        )

    def owner(self, now: float) -> str:
        """Current owner, or "" if no one holds the lease or it has expired."""
        if self._owner == "":
            return ""
        if now >= self._expires_at:
            return ""
        return self._owner

    def renew(self, source: str, now: float) -> None:
        """Extend the current owner's lease. No-op if ``source`` doesn't
        currently own it (including if the lease already expired)."""
        if self.owner(now) == source:
            self._expires_at = now + self._lease_s

    def allows(self, source: str, is_stop: bool, now: float) -> bool:
        """Return True if ``source`` is allowed to command the camera now.

        Safety rule (non-negotiable): ``is_stop=True`` is ALWAYS allowed,
        regardless of who owns the lease or whether it has expired. A
        stop command must never be blocked by arbitration.

        Non-stop commands require ``source`` to be the current lease
        owner (established via an explicit ``claim()``, e.g. through the
        ``ClaimControl`` service) — an unclaimed lease (owner == "")
        allows no non-stop source through, matching the PTZ node's rule
        of discarding any command whose ``source`` doesn't match the
        latched owner.
        """
        if is_stop:
            return True
        return self.owner(now) == source
