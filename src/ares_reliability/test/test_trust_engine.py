"""Unit tests for dependency-aware attribution."""

from ares_reliability.evidence_graph import (
    Evidence,
    EvidenceDependencyGraph,
)


def evidence(test_id: str, state: str, dependencies: tuple[str, ...],
             correlated: bool = False) -> Evidence:
    return Evidence(
        test_id, state, dependencies, 'test evidence', correlated,
        0.5 if correlated else 1.0
    )


def test_likely_gnss_fault() -> None:
    graph = EvidenceDependencyGraph()
    graph.update(evidence('gnss_motion', 'INCONSISTENT',
                          ('gnss', 'odometry')))
    graph.update(evidence('imu_wheel', 'CONSISTENT', ('imu', 'odometry')))
    graph.update(evidence('localization_motion', 'CONSISTENT',
                          ('localization', 'odometry'), True))
    result = graph.assess()
    assert result.hypothesis == 'LIKELY_GNSS_FAULT'
    assert result.confidence_label in ('MODERATE', 'HIGH')


def test_insufficient_evidence_for_one_pair() -> None:
    graph = EvidenceDependencyGraph()
    graph.update(evidence('gnss_motion', 'INCONSISTENT',
                          ('gnss', 'odometry')))
    assert graph.assess().hypothesis == 'INSUFFICIENT_EVIDENCE'


def test_likely_wheel_odometry_fault() -> None:
    graph = EvidenceDependencyGraph()
    graph.update(evidence('gnss_motion', 'INCONSISTENT',
                          ('gnss', 'odometry')))
    graph.update(evidence('imu_wheel', 'INCONSISTENT',
                          ('imu', 'odometry')))
    graph.update(evidence('localization_motion', 'INCONSISTENT',
                          ('localization', 'odometry'), True))
    result = graph.assess()
    assert result.hypothesis == 'LIKELY_ODOMETRY_FAULT'


def test_two_shared_odometry_disagreements_remain_ambiguous() -> None:
    graph = EvidenceDependencyGraph()
    graph.update(evidence('gnss_motion', 'INCONSISTENT',
                          ('gnss', 'odometry')))
    graph.update(evidence('imu_wheel', 'INCONSISTENT',
                          ('imu', 'odometry')))
    result = graph.assess()
    assert result.hypothesis == 'INSUFFICIENT_EVIDENCE'
    assert result.confidence < 0.5


def test_gnss_plus_linear_odometry_pattern_is_ambiguous() -> None:
    graph = EvidenceDependencyGraph()
    graph.update(evidence('gnss_motion', 'INCONSISTENT',
                          ('gnss', 'odometry')))
    graph.update(evidence('imu_wheel', 'CONSISTENT', ('imu', 'odometry')))
    graph.update(evidence('localization_motion', 'INCONSISTENT',
                          ('localization', 'odometry'), True))
    assert graph.assess().hypothesis == 'INSUFFICIENT_EVIDENCE'


def test_imu_and_localization_disagreeing_with_filtered_motion_points_to_odom() -> None:
    graph = EvidenceDependencyGraph()
    graph.update(evidence('gnss_motion', 'CONSISTENT',
                          ('gnss', 'filtered_motion')))
    graph.update(evidence('imu_wheel', 'INCONSISTENT',
                          ('imu', 'odometry')))
    graph.update(evidence('localization_motion', 'INCONSISTENT',
                          ('localization', 'odometry'), True))
    assert graph.assess().hypothesis == 'LIKELY_ODOMETRY_FAULT'


def test_correlated_amcl_is_discounted() -> None:
    graph = EvidenceDependencyGraph()
    graph.update(evidence('gnss_motion', 'CONSISTENT',
                          ('gnss', 'odometry')))
    graph.update(evidence('imu_wheel', 'CONSISTENT', ('imu', 'odometry')))
    graph.update(evidence('localization_motion', 'CONSISTENT',
                          ('localization', 'odometry'), True))
    result = graph.assess()
    assert result.effective_evidence_weight == 2.5
    assert result.hypothesis == 'SYSTEM_HEALTHY'


def test_direct_dropout_attributes_source() -> None:
    graph = EvidenceDependencyGraph()
    graph.update(evidence('gnss_freshness', 'INCONSISTENT', ('gnss',)))
    assert graph.assess().hypothesis == 'LIKELY_GNSS_FAULT'


def test_preinitialization_anchor_rejection_attributes_gnss() -> None:
    graph = EvidenceDependencyGraph()
    graph.update(evidence('gnss_initialization', 'INCONSISTENT',
                          ('gnss', 'configured_anchor')))
    assert graph.assess().hypothesis == 'LIKELY_GNSS_FAULT'


def test_direct_dropout_plus_other_fault_reduces_confidence() -> None:
    graph = EvidenceDependencyGraph()
    graph.update(evidence('gnss_freshness', 'INCONSISTENT', ('gnss',)))
    graph.update(evidence('imu_wheel', 'INCONSISTENT',
                          ('imu', 'odometry')))
    result = graph.assess()
    assert result.hypothesis == 'INSUFFICIENT_EVIDENCE'
    assert result.confidence_label == 'LOW'
