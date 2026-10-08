"""Unit tests for Week 4 statistical analysis."""

import csv
import json

from ares_reliability.week4_analysis import (
    _transition_metrics,
    analyze_csv,
    main,
    pearson,
    statistics,
)
import pytest


def test_statistics_include_all_required_percentiles() -> None:
    result = statistics([1.0, 2.0, 3.0, 4.0, 5.0])
    assert result['count'] == 5
    assert result['mean'] == 3.0
    assert result['median'] == 3.0
    assert result['p90'] == pytest.approx(4.6)
    assert result['p95'] == pytest.approx(4.8)
    assert result['p99'] == pytest.approx(4.96)
    assert result['maximum'] == 5.0


def test_pearson_reports_motion_dependence() -> None:
    assert pearson([1.0, 2.0, 3.0], [2.0, 4.0, 6.0]) == pytest.approx(1.0)
    assert pearson([1.0, 1.0, 1.0], [2.0, 3.0, 4.0]) is None


def test_transition_metrics_can_audit_each_sensor() -> None:
    rows = [
        {'timestamp_sec': '0.0', 'imu_trust_state': 'HEALTHY'},
        {'timestamp_sec': '1.0', 'imu_trust_state': 'DEGRADED'},
        {'timestamp_sec': '2.0', 'imu_trust_state': 'HEALTHY'},
        {'timestamp_sec': '3.0', 'imu_trust_state': 'UNTRUSTED'},
    ]
    result = _transition_metrics(
        rows, onset=None, final_time=3.0,
        state_column='imu_trust_state')
    assert result['false_degraded_transitions'] == 1
    assert result['false_untrusted_transitions'] == 1
    assert result['transition_count'] == 4


def test_transition_metrics_exclude_unlabeled_startup() -> None:
    rows = [
        {'timestamp_sec': '0.0', 'motion_regime': 'UNKNOWN',
         'odometry_trust_state': 'DEGRADED'},
        {'timestamp_sec': '1.0', 'motion_regime': 'stationary',
         'odometry_trust_state': 'HEALTHY'},
        {'timestamp_sec': '2.0', 'motion_regime': 'stationary',
         'odometry_trust_state': 'DEGRADED'},
    ]
    result = _transition_metrics(
        rows, onset=None, final_time=2.0,
        state_column='odometry_trust_state')
    assert result['false_degraded_transitions'] == 1
    assert result['transition_count'] == 2


def test_analysis_reports_strongest_active_fault_hypothesis(tmp_path) -> None:
    path = tmp_path / 'fault.csv'
    rows = [
        {
            'timestamp_sec': '1.0', 'profile': 'fault', 'seed': '0',
            'imu_fault_mode': 'bias', 'event': 'system_trust',
            'system_attribution': 'LIKELY_IMU_FAULT',
            'attribution_confidence': '0.8', 'motion_regime': 'gentle_left',
        },
        {
            'timestamp_sec': '2.0', 'profile': 'fault', 'seed': '0',
            'imu_fault_mode': 'bias', 'event': 'system_trust',
            'system_attribution': 'LIKELY_ODOMETRY_FAULT',
            'attribution_confidence': '0.65',
            'motion_regime': 'gentle_left',
        },
        {
            'timestamp_sec': '3.0', 'profile': 'fault', 'seed': '0',
            'imu_fault_mode': 'none', 'event': 'system_trust',
            'system_attribution': 'LIKELY_GNSS_FAULT',
            'attribution_confidence': '0.72', 'motion_regime': 'stationary',
        },
    ]
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    result = analyze_csv(
        path, expected_severity='DEGRADED',
        expected_attribution='LIKELY_IMU_FAULT')
    assert result['attribution_result'] == 'LIKELY_IMU_FAULT'
    assert result['attribution_confidence'] == pytest.approx(0.8)
    assert result['attribution_confidence_peak'] == pytest.approx(0.8)
    assert result['attribution_correct']


def test_expected_degraded_is_success_without_untrusted(tmp_path) -> None:
    path = tmp_path / 'degraded.csv'
    rows = [
        {'timestamp_sec': '1.0', 'profile': 'imu_bias',
         'imu_fault_mode': 'bias', 'imu_trust_state': 'HEALTHY'},
        {'timestamp_sec': '1.2', 'profile': 'imu_bias',
         'imu_fault_mode': 'bias', 'imu_trust_state': 'DEGRADED'},
        {'timestamp_sec': '2.0', 'profile': 'imu_bias',
         'imu_fault_mode': 'none', 'imu_trust_state': 'HEALTHY'},
    ]
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    result = analyze_csv(path, expected_severity='DEGRADED')
    assert result['degraded_detected']
    assert result['degraded_latency_sec'] == pytest.approx(0.2)
    assert not result['untrusted_detected']
    assert result['untrusted_latency_sec'] is None
    assert result['severity_response_correct']
    assert result['recovery_detected']


def test_expected_untrusted_requires_untrusted_response(tmp_path) -> None:
    path = tmp_path / 'untrusted.csv'
    rows = [
        {'timestamp_sec': '1.0', 'profile': 'gnss_step',
         'fault_enabled': 'true', 'fault_mode': 'step',
         'gnss_trust_state': 'HEALTHY'},
        {'timestamp_sec': '1.1', 'profile': 'gnss_step',
         'fault_enabled': 'true', 'fault_mode': 'step',
         'gnss_trust_state': 'DEGRADED'},
        {'timestamp_sec': '1.8', 'profile': 'gnss_step',
         'fault_enabled': 'true', 'fault_mode': 'step',
         'gnss_trust_state': 'UNTRUSTED'},
        {'timestamp_sec': '2.0', 'profile': 'gnss_step',
         'fault_enabled': 'false', 'fault_mode': 'none',
         'gnss_trust_state': 'UNTRUSTED'},
        {'timestamp_sec': '2.4', 'profile': 'gnss_step',
         'fault_enabled': 'false', 'fault_mode': 'none',
         'gnss_trust_state': 'HEALTHY'},
    ]
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    result = analyze_csv(path, expected_severity='UNTRUSTED')
    assert result['degraded_detected']
    assert result['untrusted_detected']
    assert result['untrusted_latency_sec'] == pytest.approx(0.8)
    assert result['severity_response_correct']
    assert result['recovery_detected']
    assert result['recovery_latency_sec'] == pytest.approx(0.4)


def test_summary_aggregates_severity_aware_latencies(tmp_path) -> None:
    csv_path = tmp_path / 'imu_bias.csv'
    rows = [
        {'timestamp_sec': '1.0', 'profile': 'imu_bias',
         'imu_fault_mode': 'bias', 'imu_trust_state': 'HEALTHY'},
        {'timestamp_sec': '1.2', 'profile': 'imu_bias',
         'imu_fault_mode': 'bias', 'imu_trust_state': 'DEGRADED'},
        {'timestamp_sec': '2.0', 'profile': 'imu_bias',
         'imu_fault_mode': 'none', 'imu_trust_state': 'HEALTHY'},
    ]
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    matrix_path = tmp_path / 'matrix.yaml'
    matrix_path.write_text(
        'scenarios:\n'
        '  imu_bias:\n'
        '    expected_severity: DEGRADED\n'
        '    expected_attribution_category: LIKELY_IMU_FAULT\n',
        encoding='utf-8',
    )
    summary_path = tmp_path / 'summary.json'
    calibration_path = tmp_path / 'calibration.json'

    main([
        str(csv_path), '--matrix', str(matrix_path),
        '--output', str(summary_path),
        '--calibration-output', str(calibration_path),
    ])

    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    experiment = summary['experiments'][0]
    assert summary['schema_version'] == 2
    assert summary['severity_response_counts']['correct'] == 1
    assert summary['degraded_latency_distribution_sec']['count'] == 1
    assert summary['untrusted_latency_distribution_sec']['count'] == 0
    assert experiment['degraded_detected']
    assert experiment['severity_response_correct']
    assert not experiment['detection_success']
