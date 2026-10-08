"""Unit tests for per-source hysteretic trust."""

from ares_reliability.trust_model import HystereticTrustEvaluator


def evaluator() -> HystereticTrustEvaluator:
    return HystereticTrustEvaluator(
        window_size=1,
        degraded_enter=1.0,
        untrusted_enter=2.0,
        degraded_exit=1.5,
        healthy_exit=0.5,
        degrade_persistence=2,
        untrusted_persistence=2,
        recovery_persistence=3,
    )


def test_initial_and_missing_evidence_are_unavailable() -> None:
    trust = evaluator()
    assert trust.current_state == 'UNAVAILABLE'
    assert trust.mark_unavailable('no data').public_state == 'UNAVAILABLE'


def test_healthy_to_degraded_requires_persistence() -> None:
    trust = evaluator()
    assert trust.add_residual(1.2).public_state == 'HEALTHY'
    result = trust.add_residual(1.2)
    assert result.public_state == 'DEGRADED'
    assert result.transition == ('HEALTHY', 'DEGRADED')


def test_degraded_to_untrusted_requires_persistence() -> None:
    trust = evaluator()
    trust.add_residual(1.2)
    trust.add_residual(1.2)
    assert trust.add_residual(2.5).public_state == 'DEGRADED'
    assert trust.add_residual(2.5).public_state == 'UNTRUSTED'


def test_asymmetric_recovery() -> None:
    trust = evaluator()
    trust.add_residual(2.5)
    trust.add_residual(2.5)
    for _ in range(2):
        assert trust.add_residual(1.0).public_state == 'UNTRUSTED'
    assert trust.add_residual(1.0).public_state == 'DEGRADED'
    for _ in range(2):
        assert trust.add_residual(0.4).public_state == 'DEGRADED'
    assert trust.add_residual(0.4).public_state == 'HEALTHY'


def test_unavailable_does_not_become_untrusted() -> None:
    trust = evaluator()
    trust.add_residual(2.5)
    state = trust.mark_unavailable('reference missing')
    assert state.public_state == 'UNAVAILABLE'
    assert trust.consistency_state == 'HEALTHY'


def test_borderline_oscillation_does_not_flap_public_state() -> None:
    trust = evaluator()
    trust.add_residual(0.0)
    transitions = []
    for residual in (1.1, 0.9, 1.1, 0.9, 1.1, 0.9):
        result = trust.add_residual(residual)
        if result.transition is not None:
            transitions.append(result.transition)
    assert transitions == []
    assert trust.current_state == 'HEALTHY'


def test_authoritative_configured_anchor_rejection_bypasses_persistence() -> None:
    trust = evaluator()
    result = trust.force_state(
        'UNTRUSTED', 5.0, 'configured anchor rejected initial GNSS')
    assert result.public_state == 'UNTRUSTED'
    assert result.candidate_state == 'UNTRUSTED'
    assert result.transition == ('UNAVAILABLE', 'UNTRUSTED')
