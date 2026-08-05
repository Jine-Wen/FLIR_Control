"""Pure token policy state machine (spec ARCHITECTURE.md §3.3).

This module owns the decision of *when* the control plane should
acquire / revive / re-login / keepalive / release the Nexus remote
control token. It performs **no I/O** and never reads the clock itself:
every method that needs "now" takes it as an explicit ``now: float``
(monotonic seconds) parameter so tests can drive it deterministically.

Bug fixed here (see ARCHITECTURE.md §3.3): the old code force-claimed
control (``SERVERRemoteControlRequestAsync(Forced=1)``) unconditionally
every 10 seconds, even while completely idle, which steals control from
a physical JCU operator every 10 seconds. The new policy only keeps the
token alive while a mode is active or within ``grace_s`` seconds of the
last accepted command, uses the non-destructive
``SERVERLastNMEAGet?tokenoverride=1`` keepalive (exactly what FLIR's own
web UI does) instead of ``Forced=1``, and actively releases the token
once the grace window has elapsed so the JCU is left alone. Passing
``hold_token=True`` restores the old "always hold" behaviour for rigs
with no JCU.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class TokenState(Enum):
    """Our belief about who currently holds the Nexus control token."""

    UNKNOWN = auto()   # never observed a token id yet
    HELD = auto()       # token id matches our session
    IDLE_OURS = auto()  # token id matches our session but RC=15 said it went idle
    FOREIGN = auto()    # token id belongs to someone else (or is None while we want it)
    RELEASED = auto()   # we deliberately released it (idle past grace)


class TokenAction(Enum):
    """What the async layer should do next."""

    NONE = auto()
    ACQUIRE = auto()    # force-claim: token is someone else's / unknown
    REVIVE = auto()     # token is ours but idle (RC=15): SERVERLastNMEAGet?tokenoverride=1
    RELOGIN = auto()     # RC=21: session is no longer authenticated, must re-login
    KEEPALIVE = auto()   # periodic non-destructive keepalive (SERVERLastNMEAGet?tokenoverride=1)
    RELEASE = auto()     # actively release the token so the JCU is left alone


@dataclass(frozen=True)
class TokenPolicy:
    hold_token: bool = False
    grace_s: float = 30.0
    keepalive_s: float = 10.0


# Read-only Nexus commands never require holding the control token.
_READ_ONLY_ACTIONS = frozenset(
    {
        "PTLastNMEAGet",
        "SERVERLastNMEAGet",
        "SERVERWhoAmI",
    }
)


def needs_token(action: str) -> bool:
    """Return True if ``action`` (a Nexus command name) requires the token.

    Read commands (``PTLastNMEAGet`` / ``SERVERLastNMEAGet`` /
    ``SERVERWhoAmI``) never need it; every ``PT*Set`` / ``PTAutoScan*``
    command does.
    """
    if action in _READ_ONLY_ACTIONS:
        return False
    return action.startswith("PT") and ("Set" in action or action.startswith("PTAutoScan"))


class TokenTracker:
    """Pure state machine tracking Nexus control-token ownership.

    All time-dependent decisions take an explicit ``now`` (monotonic
    seconds) — this class never calls ``time.monotonic()`` itself.
    """

    def __init__(self, policy: TokenPolicy) -> None:
        self._policy = policy
        self.state: TokenState = TokenState.UNKNOWN
        self._last_command_at: float | None = None
        self._last_keepalive_at: float | None = None

    # -- observation -----------------------------------------------------

    def observe_token_id(
        self, token_id: int | None, our_session: int | None, now: float
    ) -> None:
        """Update belief state from a freshly observed Token_ID.

        A matching token id is NOT sufficient evidence that the token is
        usable: RC=15 means the token is still ours but the camera has
        let it go idle, and the camera keeps reporting ``Token_ID`` ==
        our session throughout that idle state — that is the whole
        nature of RC=15 (this mirrors ``_force_token_reacquire`` in the
        old ``flir_ptz_node.py``, which exists precisely because a
        matching Token_ID alone doesn't prove the token is active).

        So: IDLE_OURS is sticky. It is only cleared by an explicit
        ``note_token_acquired()`` call reporting that a REVIVE/ACQUIRE
        actually succeeded — never merely by observing a matching id
        here. A non-matching id always moves to FOREIGN (someone else
        has taken it, regardless of our previous state).
        """
        del now  # unused: pure ownership observation is not time-dependent
        if our_session is not None and token_id == our_session:
            if self.state == TokenState.IDLE_OURS:
                return  # still idle until a revive is confirmed — do not clear
            self.state = TokenState.HELD
        else:
            self.state = TokenState.FOREIGN

    def note_token_acquired(self, now: float) -> None:
        """Report that a REVIVE or ACQUIRE action actually succeeded.

        This is the only thing that clears IDLE_OURS / FOREIGN / RELEASED
        back to HELD. Also treats the acquisition itself as equivalent to
        a keepalive, so ``periodic()`` doesn't immediately fire a
        redundant KEEPALIVE on the very next tick.
        """
        self.state = TokenState.HELD
        self._last_keepalive_at = now

    def observe_rc(self, code: int | None, now: float) -> TokenAction:
        """React to a Nexus Return Code from the last command.

        RC=15 (token idle, still ours) -> REVIVE.
        RC=21 (not authenticated)      -> RELOGIN.
        Anything else                  -> NONE (no state change).

        The reaction is purely a function of ``code`` and current
        ``state`` — not of time — but ``now`` is accepted (and unused)
        to keep the method's signature consistent with the rest of this
        class's "everything time-dependent takes now explicitly"
        contract, and so future diagnostics can time-stamp transitions
        without an interface change.
        """
        if code == 15:
            self.state = TokenState.IDLE_OURS
            return TokenAction.REVIVE
        if code == 21:
            self.state = TokenState.UNKNOWN
            return TokenAction.RELOGIN
        return TokenAction.NONE

    def note_command(self, now: float) -> None:
        """Record that a command was accepted at ``now`` (drives grace window)."""
        self._last_command_at = now

    # -- decisions ---------------------------------------------------------

    def required_before(self, action: str, now: float) -> TokenAction:
        """What must happen before sending ``action`` at time ``now``.

        Read-only actions never need the token. Otherwise: if we don't
        currently hold it (FOREIGN/UNKNOWN/RELEASED) -> ACQUIRE; if it's
        ours but flagged idle -> REVIVE; if we already hold it -> NONE.

        This decision is purely a function of current belief ``state``,
        not of time, but ``now`` is accepted (and unused) for signature
        consistency with the rest of this class.
        """
        if not needs_token(action):
            return TokenAction.NONE
        if self.state == TokenState.IDLE_OURS:
            return TokenAction.REVIVE
        if self.state in (TokenState.HELD,):
            return TokenAction.NONE
        # UNKNOWN, FOREIGN, RELEASED all require (re)acquiring
        return TokenAction.ACQUIRE

    def periodic(self, now: float, mode_active: bool) -> TokenAction:
        """Called every tick to decide on keepalive / release.

        - While ``mode_active`` or within ``grace_s`` of the last command:
          keepalive every ``keepalive_s`` seconds (non-destructive
          ``SERVERLastNMEAGet?tokenoverride=1``).
        - Once idle past the grace window: RELEASE once, so the JCU is
          left alone (unless ``hold_token=True``, which restores the
          old always-hold behaviour and keeps issuing KEEPALIVE
          indefinitely).
        - If we don't currently hold the token at all, there's nothing
          to keep alive or release -> NONE.
        """
        if self.state not in (TokenState.HELD, TokenState.IDLE_OURS):
            return TokenAction.NONE

        within_grace = self._within_grace(now)

        if self._policy.hold_token or mode_active or within_grace:
            if self._keepalive_due(now):
                self._last_keepalive_at = now
                return TokenAction.KEEPALIVE
            return TokenAction.NONE

        # Idle past grace and not holding forever: release once.
        if self.state != TokenState.RELEASED:
            self.state = TokenState.RELEASED
            return TokenAction.RELEASE
        return TokenAction.NONE

    # -- internals -----------------------------------------------------

    def _within_grace(self, now: float) -> bool:
        if self._last_command_at is None:
            return False
        return (now - self._last_command_at) < self._policy.grace_s

    def _keepalive_due(self, now: float) -> bool:
        if self._last_keepalive_at is None:
            return True
        return (now - self._last_keepalive_at) >= self._policy.keepalive_s
