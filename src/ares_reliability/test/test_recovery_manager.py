"""Unit tests for staged GNSS recovery and covariance policy."""

from ares_reliability.gnss_trusted_proxy import (
    inflated_fix,
    prefusion_guard_accept,
    validate_reentry_timing,
)
from ares_reliability.recovery_manager import GnssRecoveryPolicy
import pytest
from sensor_msgs.msg import NavSatFix


def test_covariance_inflation_then_gating() -> None:
    policy = GnssRecoveryPolicy(untrusted_observations_before_gate=3)
    attribution = 'LIKELY_GNSS_FAULT'
    assert policy.update_trust(
        'UNTRUSTED', attribution).covariance_factor == 100.0
    assert policy.update_trust('UNTRUSTED', attribution).forward
    gated = policy.update_trust('UNTRUSTED', attribution)
    assert not gated.forward
    assert gated.state == 'GATED'


def test_degraded_activates_lower_inflation() -> None:
    policy = GnssRecoveryPolicy()
    decision = policy.update_trust('DEGRADED')
    assert decision.forward
    assert decision.covariance_factor == 10.0
    assert decision.state == 'DEGRADED'


def test_gating_remains_primary_when_covariance_inflation_is_disabled() -> None:
    policy = GnssRecoveryPolicy(
        degraded_covariance_factor=1.0,
        untrusted_covariance_factor=1.0,
        probation_covariance_factor=1.0,
        untrusted_observations_before_gate=1,
    )
    degraded = policy.update_trust('DEGRADED', 'LIKELY_GNSS_FAULT')
    assert degraded.forward
    assert degraded.covariance_factor == 1.0
    gated = policy.update_trust('UNTRUSTED', 'LIKELY_GNSS_FAULT')
    assert not gated.forward
    assert gated.state == 'GATED'


def test_gated_recovery_requires_probation() -> None:
    policy = GnssRecoveryPolicy(
        untrusted_observations_before_gate=1,
        probation_observations=3,
    )
    assert not policy.update_trust(
        'UNTRUSTED', 'LIKELY_GNSS_FAULT').forward
    assert policy.update_trust('HEALTHY').state == 'PROBATION'
    assert policy.update_trust('HEALTHY').state == 'PROBATION'
    restored = policy.update_trust('HEALTHY')
    assert restored.state == 'NORMAL'
    assert restored.covariance_factor == 1.0


def test_gated_recovery_admits_degraded_only_as_high_covariance_probation(
) -> None:
    policy = GnssRecoveryPolicy(
        untrusted_observations_before_gate=1,
        probation_observations=2,
    )
    assert policy.update_trust(
        'UNTRUSTED', 'LIKELY_GNSS_FAULT').state == 'GATED'
    degraded = policy.update_trust('DEGRADED', 'LIKELY_GNSS_FAULT')
    assert degraded.state == 'PROBATION'
    assert degraded.forward
    assert degraded.covariance_factor == policy.probation_high_factor
    assert policy.probation_count == 0
    assert policy.update_trust('HEALTHY').state == 'PROBATION'
    assert policy.update_trust('HEALTHY').state == 'NORMAL'


def test_unavailable_deweights_but_does_not_gate() -> None:
    policy = GnssRecoveryPolicy(untrusted_observations_before_gate=1)
    decision = policy.update_trust('UNAVAILABLE')
    assert decision.state == 'DEGRADED'
    assert decision.forward


def test_unknown_covariance_is_made_explicit_when_inflated() -> None:
    fix = NavSatFix()
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    output = inflated_fix(fix, 10.0, unknown_variance_m2=2.0)
    assert output.position_covariance[0] == 20.0
    assert output.position_covariance[4] == 20.0
    assert output.position_covariance[8] == 20.0
    assert output.position_covariance_type == (
        NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    )


def test_probation_covariance_reduces_in_deterministic_stages() -> None:
    policy = GnssRecoveryPolicy(
        untrusted_observations_before_gate=1,
        probation_observations=5,
        probation_high_covariance_observations=2,
        probation_high_covariance_factor=10.0,
        probation_covariance_factor=5.0,
    )
    assert policy.update_trust(
        'UNTRUSTED', 'LIKELY_GNSS_FAULT').state == 'GATED'
    assert policy.update_trust('HEALTHY').covariance_factor == 10.0
    assert policy.update_trust('HEALTHY').covariance_factor == 10.0
    assert policy.update_trust('HEALTHY').covariance_factor == 5.0
    assert policy.update_trust('HEALTHY').covariance_factor == 5.0
    normal = policy.update_trust('HEALTHY')
    assert normal.state == 'NORMAL'
    assert normal.covariance_factor == 1.0


def test_known_covariance_is_multiplied_on_all_position_axes() -> None:
    fix = NavSatFix()
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    fix.position_covariance = [
        0.2, 0.0, 0.0,
        0.0, 0.3, 0.0,
        0.0, 0.0, 0.4,
    ]
    output = inflated_fix(fix, 25.0)
    assert output.position_covariance[0] == 5.0
    assert output.position_covariance[4] == 7.5
    assert output.position_covariance[8] == 10.0


def test_stale_gnss_is_rejected_at_reentry() -> None:
    accepted, age, reason, before_release = validate_reentry_timing(
        message_stamp_sec=9.0,
        current_time_sec=10.0,
        gate_release_sec=8.0,
        max_message_age_sec=0.5,
        future_tolerance_sec=0.05,
    )
    assert not accepted
    assert age == 1.0
    assert reason == 'STALE_MESSAGE'
    assert not before_release


def test_pre_release_gnss_is_rejected_even_when_recent() -> None:
    accepted, age, reason, before_release = validate_reentry_timing(
        message_stamp_sec=9.99,
        current_time_sec=10.01,
        gate_release_sec=10.0,
        max_message_age_sec=0.5,
        future_tolerance_sec=0.05,
    )
    assert not accepted
    assert age == pytest.approx(0.02)
    assert reason == 'GENERATED_BEFORE_GATE_RELEASE'
    assert before_release


def test_fresh_post_release_gnss_is_accepted() -> None:
    accepted, age, reason, before_release = validate_reentry_timing(
        message_stamp_sec=10.1,
        current_time_sec=10.11,
        gate_release_sec=10.0,
        max_message_age_sec=0.5,
        future_tolerance_sec=0.05,
    )
    assert accepted
    assert age == pytest.approx(0.01)
    assert reason == 'ACCEPTED'
    assert not before_release


def test_fault_returning_during_probation_regates_without_normal_flap() -> None:
    policy = GnssRecoveryPolicy(
        untrusted_observations_before_gate=1,
        probation_observations=3,
    )
    assert policy.update_trust(
        'UNTRUSTED', 'LIKELY_GNSS_FAULT').state == 'GATED'
    assert policy.update_trust('HEALTHY').state == 'PROBATION'
    assert policy.update_trust(
        'UNTRUSTED', 'LIKELY_GNSS_FAULT').state == 'GATED'
    assert policy.probation_count == 0


def test_repeated_gate_cycles_each_require_full_probation() -> None:
    policy = GnssRecoveryPolicy(
        untrusted_observations_before_gate=1,
        probation_observations=2,
    )
    for _ in range(3):
        assert policy.update_trust(
            'UNTRUSTED', 'LIKELY_GNSS_FAULT').state == 'GATED'
        assert policy.update_trust('HEALTHY').state == 'PROBATION'
        assert policy.update_trust('HEALTHY').state == 'NORMAL'


def test_imu_attribution_never_gates_gnss() -> None:
    policy = GnssRecoveryPolicy(untrusted_observations_before_gate=1)
    decision = policy.update_trust('UNTRUSTED', 'LIKELY_IMU_FAULT')
    assert decision.forward
    assert decision.state == 'DEGRADED'


def test_odometry_attribution_never_gates_gnss() -> None:
    policy = GnssRecoveryPolicy(untrusted_observations_before_gate=1)
    decision = policy.update_trust('UNTRUSTED', 'LIKELY_ODOMETRY_FAULT')
    assert decision.forward
    assert decision.state == 'DEGRADED'


def test_insufficient_evidence_never_hard_gates_gnss() -> None:
    policy = GnssRecoveryPolicy(untrusted_observations_before_gate=1)
    decision = policy.update_trust('UNTRUSTED', 'INSUFFICIENT_EVIDENCE')
    assert decision.forward
    assert decision.state == 'DEGRADED'


def test_startup_gate_requires_probation_before_normal() -> None:
    policy = GnssRecoveryPolicy(
        startup_forward=False, probation_observations=2)
    assert not policy.decision.forward
    assert policy.update_trust('UNTRUSTED', 'LIKELY_GNSS_FAULT').state == 'GATED'
    assert not policy.decision.forward
    assert policy.update_trust('UNAVAILABLE').state == 'GATED'
    assert not policy.decision.forward
    assert policy.update_trust('HEALTHY').state == 'PROBATION'
    assert policy.update_trust('HEALTHY').state == 'NORMAL'


def test_prefusion_guard_accepts_healthy_residual() -> None:
    assert prefusion_guard_accept(1.944, 3.0)


def test_prefusion_guard_rejects_five_meter_step() -> None:
    assert not prefusion_guard_accept(4.769, 3.0)


def test_prefusion_guard_rejects_threshold_boundary() -> None:
    assert not prefusion_guard_accept(3.0, 3.0)


def test_prefusion_guard_rejects_nonfinite_residual() -> None:
    assert not prefusion_guard_accept(float('nan'), 3.0)
    assert not prefusion_guard_accept(float('inf'), 3.0)


def test_prefusion_guard_rejects_invalid_threshold() -> None:
    with pytest.raises(ValueError):
        prefusion_guard_accept(1.0, 0.0)
