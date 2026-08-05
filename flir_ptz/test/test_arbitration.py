"""Tests for flir_ptz.control.arbitration — pure control-source Arbiter.

Standard library only. Every test drives the arbiter with an explicit
fake clock (plain floats), never the real clock.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from flir_ptz.control.arbitration import Arbiter  # noqa: E402


# -- the non-negotiable safety rule --------------------------------------


def test_stop_always_allowed_with_no_owner():
    a = Arbiter(lease_s=60.0)
    assert a.owner(now=0.0) == ""
    assert a.allows("web", is_stop=True, now=0.0) is True


def test_stop_always_allowed_even_when_someone_else_owns():
    a = Arbiter(lease_s=60.0)
    a.claim("joy", now=0.0)
    assert a.allows("web", is_stop=True, now=0.0) is True
    assert a.allows("nobody", is_stop=True, now=0.0) is True


def test_stop_always_allowed_after_lease_expired():
    a = Arbiter(lease_s=60.0)
    a.claim("joy", now=0.0)
    # Long after expiry.
    assert a.allows("web", is_stop=True, now=10_000.0) is True


def test_stop_always_allowed_even_for_the_expired_former_owner():
    a = Arbiter(lease_s=60.0)
    a.claim("joy", now=0.0)
    assert a.allows("joy", is_stop=True, now=10_000.0) is True


# -- last-claim-wins, no priority hierarchy ------------------------------


def test_last_claim_wins_web_then_joy():
    a = Arbiter(lease_s=60.0)
    r1 = a.claim("web", now=0.0)
    assert r1.granted is True
    assert r1.owner == "web"
    assert a.owner(now=0.0) == "web"

    # joy claims afterwards — always wins, no priority check, even
    # though web's lease hasn't expired yet.
    r2 = a.claim("joy", now=1.0)
    assert r2.granted is True
    assert r2.owner == "joy"
    assert a.owner(now=1.0) == "joy"


def test_last_claim_wins_joy_then_web_reverse_order():
    a = Arbiter(lease_s=60.0)
    a.claim("joy", now=0.0)
    a.claim("web", now=0.1)
    assert a.owner(now=0.1) == "web"


def test_non_stop_command_from_non_owner_is_blocked():
    a = Arbiter(lease_s=60.0)
    a.claim("web", now=0.0)
    assert a.allows("joy", is_stop=False, now=0.0) is False
    assert a.allows("web", is_stop=False, now=0.0) is True


def test_non_stop_command_with_no_owner_is_blocked():
    a = Arbiter(lease_s=60.0)
    assert a.owner(now=0.0) == ""
    assert a.allows("web", is_stop=False, now=0.0) is False


# -- lease expiry reverting to no owner ----------------------------------


def test_lease_expires_reverts_to_no_owner():
    a = Arbiter(lease_s=60.0)
    a.claim("web", now=0.0)
    assert a.owner(now=59.9) == "web"
    assert a.owner(now=60.0) == ""  # expires_at == now -> expired
    assert a.owner(now=61.0) == ""


def test_lease_expiry_boundary_exact():
    a = Arbiter(lease_s=10.0)
    r = a.claim("web", now=5.0)
    assert r.expires_at == 15.0
    assert a.owner(now=14.999999) == "web"
    assert a.owner(now=15.0) == ""


def test_after_expiry_new_source_can_claim():
    a = Arbiter(lease_s=60.0)
    a.claim("web", now=0.0)
    assert a.owner(now=100.0) == ""
    a.claim("joy", now=100.0)
    assert a.owner(now=100.0) == "joy"


# -- renew() extends the lease -------------------------------------------


def test_renew_extends_current_owners_lease():
    a = Arbiter(lease_s=60.0)
    a.claim("web", now=0.0)
    assert a.owner(now=59.0) == "web"
    a.renew("web", now=59.0)
    # Without renew this would have expired by now=60.
    assert a.owner(now=60.0) == "web"
    assert a.owner(now=118.9) == "web"
    assert a.owner(now=119.0) == ""


def test_renew_by_non_owner_is_noop():
    a = Arbiter(lease_s=60.0)
    a.claim("web", now=0.0)
    a.renew("joy", now=1.0)  # joy never owned it — must not steal the lease
    assert a.owner(now=1.0) == "web"


def test_renew_after_expiry_is_noop_does_not_resurrect_owner():
    a = Arbiter(lease_s=10.0)
    a.claim("web", now=0.0)
    assert a.owner(now=10.0) == ""
    a.renew("web", now=10.0)  # lease already expired — must not resurrect
    assert a.owner(now=10.0) == ""
    assert a.owner(now=10.1) == ""


def test_renew_at_exact_boundary_still_owner_extends():
    a = Arbiter(lease_s=10.0)
    a.claim("web", now=0.0)
    # At now == expires_at exactly, owner() already reports "" (expired,
    # see boundary test above) so renew() at that same instant is a
    # no-op too — expiry and renew share the same boundary semantics.
    a.renew("web", now=10.0)
    assert a.owner(now=10.0) == ""


# -- release() by non-owner is a no-op -----------------------------------


def test_release_by_owner_clears_ownership():
    a = Arbiter(lease_s=60.0)
    a.claim("web", now=0.0)
    result = a.release("web", now=1.0)
    assert result.granted is False
    assert result.owner == ""
    assert a.owner(now=1.0) == ""


def test_release_by_non_owner_is_noop():
    a = Arbiter(lease_s=60.0)
    a.claim("web", now=0.0)
    result = a.release("joy", now=1.0)
    assert result.owner == "web"
    assert a.owner(now=1.0) == "web"  # unaffected


def test_release_with_no_owner_is_noop():
    a = Arbiter(lease_s=60.0)
    result = a.release("web", now=0.0)
    assert result.owner == ""
    assert a.owner(now=0.0) == ""


def test_release_after_expiry_is_noop_not_an_error():
    a = Arbiter(lease_s=10.0)
    a.claim("web", now=0.0)
    assert a.owner(now=20.0) == ""
    result = a.release("web", now=20.0)  # lease already expired
    assert result.owner == ""
    assert a.owner(now=20.0) == ""
