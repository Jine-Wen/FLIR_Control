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


def allows_command(
    owner: str, expires_at: float, source: str, is_stop: bool, now: float
) -> bool:
    """THE arbitration rule. Both the authority and its mirrors must use this.

    It lives as a free function, not as an Arbiter method, because two
    processes need to evaluate it: the PTZ node owns the lease and enforces
    the decision, while the web node evaluates the same rule against the
    latched ``flir/control_source`` it mirrors, purely so the browser gets
    instant feedback instead of a silently-dropped publish.

    That second copy used to be a hand-written re-implementation described as
    "structurally identical" to this one. It drifted the moment this rule
    changed: the authority was fixed to let an expired lease fall open, the
    mirror was not, and the web node went on rejecting every command with
    ``{"ok": false, "locked": ""}`` before it ever reached the authority. The
    camera stayed frozen and the fix to the real rule appeared to do nothing.
    One definition, imported by both, makes that failure impossible.

    Rules:
      * a stop is ALWAYS allowed, from anyone, regardless of ownership --
        non-negotiable safety rule;
      * the current owner is allowed;
      * an UNOWNED lease (never claimed, or expired through inactivity) is
        open to whoever commands next. Arbitration exists to stop two
        operators fighting over the camera, not to lock an idle one.
    """
    if is_stop:
        return True
    if owner == "":
        return True
    if now >= expires_at:
        return True          # lease lapsed -> unowned -> open
    return owner == source


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

        A command from the current owner is allowed. So is a command from
        anyone when the lease is UNOWNED — either never claimed, or expired
        through inactivity.

        That last part is deliberate and was a bug when it worked the other
        way. Arbitration exists to stop two operators fighting over the
        camera, not to lock it. Refusing an unowned lease protects nobody
        (there is no one to protect from) and instead deadlocks the system:
        after `lease_s` of inactivity the owner expires, every subsequent
        command is silently discarded, and the operator's joystick simply
        stops working with nothing logged and no way back except performing
        the unlock gesture again. The camera looked broken.

        So an idle lease falls open, and the next source to command it takes
        it (the caller is expected to follow up with :meth:`claim`).
        Contention is unchanged: while somebody holds a live lease, everyone
        else is still refused.
        """
        return allows_command(self._owner, self._expires_at, source, is_stop, now)
