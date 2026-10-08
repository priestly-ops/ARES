"""Regression tests for Week 5 active-fusion evidence analysis."""

import csv

from ares_reliability.week5_analysis import (
    _comparison,
    _error_slope,
    _time_fraction_below,
    _time_to_sustained_threshold,
    _window_error_statistics,
    analyze_fusion_csv,
)
import pytest


def _write(path, rows) -> None:
    fields = sorted({key for row in rows for key in row})
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_analysis_measures_gating_probation_and_estimator_health(
    tmp_path,
) -> None:
    path = tmp_path / 'protected.csv'
    base = {
        'profile': 'gnss_step_5m', 'fusion_mode': 'protected',
        'seed': '2505', 'fault_enabled': 'true', 'fault_mode': 'step',
        'sensor_fault': 'gnss', 'estimator_finite': 'true',
        'estimator_stale': 'false', 'estimator_restart_count': '0',
    }
    rows = [
        base | {'timestamp_sec': '1.0', 'event': 'estimator_health',
                'fused_reference_error_m': '0.2',
                'estimator_status': 'HEALTHY'},
        base | {'timestamp_sec': '1.2', 'event': 'gating_state',
                'gnss_trust_state': 'DEGRADED', 'covariance_scale': '10'},
        base | {'timestamp_sec': '1.5', 'event': 'trust_state',
                'gnss_trust_state': 'UNTRUSTED'},
        base | {'timestamp_sec': '1.6', 'event': 'gating_state',
                'gnss_gated': 'true', 'gating_state': 'GATED'},
        base | {'timestamp_sec': '1.7', 'event': 'estimator_health',
                'fused_reference_error_m': '1.5',
                'estimator_status': 'HEALTHY'},
        base | {'timestamp_sec': '2.0', 'event': 'fault_status',
                'fault_enabled': 'false', 'fault_mode': 'none'},
        base | {'timestamp_sec': '2.1', 'event': 'recovery_state',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'recovery_state': 'PROBATION'},
        base | {'timestamp_sec': '2.5', 'event': 'recovery_state',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'recovery_state': 'NORMAL'},
        base | {'timestamp_sec': '2.6', 'event': 'estimator_health',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'fused_reference_error_m': '0.3',
                'estimator_status': 'HEALTHY'},
    ]
    _write(path, rows)
    result = analyze_fusion_csv(path)
    assert result['degraded_latency_sec'] == pytest.approx(0.2)
    assert result['untrusted_latency_sec'] == pytest.approx(0.5)
    assert result['gating_latency_sec'] == pytest.approx(0.6)
    assert result['probation_duration_sec'] == pytest.approx(0.4)
    assert result['peak_fused_error_m'] == pytest.approx(1.5)
    assert result['wrong_sensor_gate_count'] == 0


def test_analysis_computes_post_recovery_convergence_metrics(tmp_path) -> None:
    path = tmp_path / 'recovery.csv'
    base = {
        'profile': 'gnss_step_5m', 'fusion_mode': 'protected',
        'seed': '2508', 'fault_mode': 'step', 'sensor_fault': 'gnss',
        'estimator_restart_count': '0',
    }
    rows = [
        base | {'timestamp_sec': '0.0', 'event': 'estimator_health',
                'fault_enabled': 'false', 'fused_reference_error_m': '0.4',
                'fused_covariance_trace': '1.0',
                'gnss_measurement_count': '1'},
        base | {'timestamp_sec': '1.0', 'event': 'fault_status',
                'fault_enabled': 'true'},
        base | {'timestamp_sec': '1.1', 'event': 'gating_state',
                'fault_enabled': 'true', 'gnss_gated': 'true',
                'gating_state': 'GATED'},
        base | {'timestamp_sec': '1.2', 'event': 'estimator_health',
                'fault_enabled': 'true', 'fused_reference_error_m': '1.5',
                'fused_covariance_trace': '2.0',
                'gnss_measurement_count': '1'},
        base | {'timestamp_sec': '2.0', 'event': 'fault_status',
                'fault_enabled': 'false', 'fault_mode': 'none'},
        base | {'timestamp_sec': '2.0', 'event': 'estimator_health',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'fused_reference_error_m': '1.4',
                'fused_covariance_trace': '3.0',
                'gnss_measurement_count': '1'},
        base | {'timestamp_sec': '2.5', 'event': 'gating_state',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'gnss_gated': 'false', 'gating_state': 'PROBATION'},
        base | {'timestamp_sec': '2.6', 'event': 'gnss_reentry',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'gnss_forwarded': 'true', 'gnss_first_reentry': 'true',
                'gnss_message_stamp_sec': '2.58',
                'gnss_proxy_message_age_sec': '0.02',
                'gnss_generated_before_gate_release': 'false',
                'gnss_timing_reason': 'ACCEPTED'},
        base | {'timestamp_sec': '2.7', 'event': 'estimator_health',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'fused_reference_error_m': '1.8',
                'fused_covariance_trace': '2.5',
                'gnss_measurement_count': '2', 'fused_x_m': '1.0',
                'fused_y_m': '2.0', 'gnss_measurement_x_m': '1.3',
                'gnss_measurement_y_m': '2.4', 'gnss_innovation_m': '0.5',
                'gnss_measurement_covariance_x_m2': '5.0',
                'estimator_pose_jump_m': '0.1'},
        base | {'timestamp_sec': '3.0', 'event': 'recovery_state',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'recovery_state': 'PROBATION'},
        base | {'timestamp_sec': '3.0', 'event': 'estimator_health',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'fused_reference_error_m': '0.9',
                'fused_covariance_trace': '2.0',
                'gnss_measurement_count': '3'},
        base | {'timestamp_sec': '4.0', 'event': 'recovery_state',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'recovery_state': 'NORMAL'},
        base | {'timestamp_sec': '4.0', 'event': 'estimator_health',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'fused_reference_error_m': '0.4',
                'fused_covariance_trace': '1.2',
                'gnss_measurement_count': '4'},
        base | {'timestamp_sec': '7.0', 'event': 'estimator_health',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'fused_reference_error_m': '0.3',
                'fused_covariance_trace': '1.0',
                'gnss_measurement_count': '5'},
        base | {'timestamp_sec': '12.0', 'event': 'estimator_health',
                'fault_enabled': 'false', 'fault_mode': 'none',
                'fused_reference_error_m': '0.2',
                'fused_covariance_trace': '1.0',
                'gnss_measurement_count': '6'},
    ]
    _write(path, rows)
    result = analyze_fusion_csv(path)
    assert result['error_at_fault_clear_m'] == pytest.approx(1.4)
    assert result['error_at_gate_release_m'] == pytest.approx(1.8)
    assert result['error_at_first_gnss_reentry_m'] == pytest.approx(1.8)
    assert result['post_recovery_peak_error_m'] == pytest.approx(1.8)
    assert result['post_recovery_mean_error_m'] == pytest.approx(5.0 / 6.0)
    assert result['post_recovery_median_error_m'] == pytest.approx(0.65)
    assert result['recovery_overshoot_m'] == pytest.approx(0.4)
    assert result['time_to_error_below_1m'] == pytest.approx(1.0)
    assert result['time_to_error_below_0p5m'] == pytest.approx(2.0)
    assert result['error_recovery_plus_5s'] == pytest.approx(0.3)
    assert result['error_recovery_plus_10s'] == pytest.approx(0.2)
    assert result['reentry_evidence']['message_age_sec'] == pytest.approx(0.02)
    assert result['reentry_evidence']['position_discrepancy_m'] == pytest.approx(
        0.5)
    assert result['gating_covariance'][
        'immediately_before_reentry'] == pytest.approx(3.0)


def test_sustained_threshold_crossing_resets_on_excursion() -> None:
    errors = [
        (0.0, 1.2), (1.0, 0.9), (2.0, 0.8), (3.0, 1.1),
        (4.0, 0.9), (5.0, 0.8), (6.0, 0.7), (7.0, 0.6),
        (8.0, 0.5), (9.0, 0.4),
    ]
    assert _time_to_sustained_threshold(
        errors, 0.0, 1.0, 5.0, max_sample_gap_sec=1.1,
    ) == pytest.approx(4.0)
    assert _time_to_sustained_threshold(
        errors, 0.0, 1.0, 10.0, max_sample_gap_sec=1.1,
    ) is None


def test_post_normal_occupancy_is_time_weighted() -> None:
    errors = [(0.0, 0.5), (2.0, 1.5), (6.0, 0.5), (10.0, 0.5)]
    assert _time_fraction_below(errors, 0.0, 10.0, 1.0) == pytest.approx(
        0.6)


def test_late_window_statistics_and_error_slope() -> None:
    errors = [
        (0.0, 4.0), (10.0, 2.0), (20.0, 1.5),
        (29.0, 1.0), (30.0, 1.0), (45.0, 1.3), (59.0, 1.58),
    ]
    window = _window_error_statistics(errors, 0.0, 10.0, 30.0)
    assert window['mean'] == pytest.approx(1.5)
    assert window['median'] == pytest.approx(1.5)
    assert window['maximum'] == pytest.approx(2.0)
    assert _error_slope(errors, 0.0, 30.0, 60.0) == pytest.approx(0.02)


def _recovery_rows(*, late_error: float, slope_mps: float,
                   horizon_sec: float) -> list[dict[str, str]]:
    base = {
        'profile': 'gnss_step_5m', 'fusion_mode': 'protected',
        'seed': '2508', 'fault_mode': 'none', 'sensor_fault': 'gnss',
        'estimator_restart_count': '0', 'estimator_status': 'HEALTHY',
    }
    rows = [
        base | {'timestamp_sec': '0.0', 'event': 'estimator_health',
                'fault_enabled': 'false',
                'fused_reference_error_m': '0.4'},
        base | {'timestamp_sec': '0.5', 'event': 'estimator_health',
                'fault_enabled': 'false',
                'fused_reference_error_m': '0.5'},
        base | {'timestamp_sec': '1.0', 'event': 'fault_status',
                'fault_enabled': 'true', 'fault_mode': 'step'},
        base | {'timestamp_sec': '1.2', 'event': 'estimator_health',
                'fault_enabled': 'true', 'fault_mode': 'step',
                'fused_reference_error_m': '1.0'},
        base | {'timestamp_sec': '2.0', 'event': 'fault_status',
                'fault_enabled': 'false'},
        base | {'timestamp_sec': '3.0', 'event': 'recovery_state',
                'fault_enabled': 'false', 'recovery_state': 'NORMAL'},
    ]
    sample_count = int(horizon_sec / 0.2) + 1
    for index in range(sample_count):
        offset = index * 0.2
        error = late_error + slope_mps * offset
        rows.append(base | {
            'timestamp_sec': str(3.0 + offset),
            'event': 'estimator_health', 'fault_enabled': 'false',
            'recovery_state': 'NORMAL',
            'fused_reference_error_m': str(error),
            'fused_covariance_trace': '1.0',
        })
    return rows


def test_persistent_bias_and_drift_classification(tmp_path) -> None:
    biased_path = tmp_path / 'biased.csv'
    _write(biased_path, _recovery_rows(
        late_error=1.2, slope_mps=0.0, horizon_sec=60.0))
    biased = analyze_fusion_csv(biased_path)
    assert biased['post_normal_complete_60s']
    assert biased['persistent_recovery_bias']
    assert not biased['persistent_recovery_drift']
    assert biased['recovery_classification'] == 'PERSISTENT_BIAS'

    drifting_path = tmp_path / 'drifting.csv'
    _write(drifting_path, _recovery_rows(
        late_error=0.6, slope_mps=0.02, horizon_sec=60.0))
    drifting = analyze_fusion_csv(drifting_path)
    assert drifting['persistent_recovery_bias']
    assert drifting['persistent_recovery_drift']
    assert drifting['post_normal_30_60s_error_slope'] == pytest.approx(0.02)
    assert drifting['recovery_classification'] == 'PERSISTENT_DRIFT'


def test_sustained_convergence_uses_five_post_normal_seconds(
    tmp_path,
) -> None:
    path = tmp_path / 'converged.csv'
    _write(path, _recovery_rows(
        late_error=0.4, slope_mps=0.0, horizon_sec=60.0))
    result = analyze_fusion_csv(path)
    assert result['post_normal_time_to_sustained_healthy_band_5s'] == (
        pytest.approx(0.0))
    assert result['post_normal_time_to_sustained_healthy_band_10s'] == (
        pytest.approx(0.0))
    assert not result['persistent_recovery_bias']
    assert not result['persistent_recovery_drift']
    assert result['recovery_classification'] == 'SUSTAINED_CONVERGENCE'
    assert result['post_normal_mean_error'] == pytest.approx(0.4)
    assert result['post_normal_p95_error'] == pytest.approx(0.4)
    assert result['post_normal_error_statistics']['count'] == 300


def test_short_run_leaves_long_horizon_classification_unavailable(
    tmp_path,
) -> None:
    path = tmp_path / 'short.csv'
    _write(path, _recovery_rows(
        late_error=0.4, slope_mps=0.0, horizon_sec=20.0))
    result = analyze_fusion_csv(path)
    assert not result['post_normal_complete_60s']
    assert result['post_normal_error_plus_60s'] is None
    assert result['post_normal_30_60s_error_slope'] is None
    assert result['persistent_recovery_bias'] is None
    assert result['persistent_recovery_drift'] is None
    assert result['recovery_classification'] == (
        'INSUFFICIENT_POST_NORMAL_HORIZON')


def test_comparison_reports_matched_pair_recovery_metrics() -> None:
    result = _comparison([
        {'scenario': 'step', 'fusion_mode': 'protected', 'seed': '2508',
         'file': 'p.csv', 'peak_fused_error_m': 1.0,
         'position_error_during_fault': {'mean': 0.8},
         'post_recovery_peak_error_m': 1.5, 'final_error_m': 0.4,
         'time_to_error_below_1m': 1.0},
        {'scenario': 'step', 'fusion_mode': 'unprotected', 'seed': '2508',
         'file': 'u.csv', 'peak_fused_error_m': 5.0,
         'position_error_during_fault': {'mean': 4.0},
         'post_recovery_peak_error_m': 2.0, 'final_error_m': 0.6,
         'time_to_error_below_1m': 2.0},
    ])['step']
    pair = result['matched_pairs'][0]
    assert pair['protected_post_recovery_peak_m'] == pytest.approx(1.5)
    assert pair['protected_terminal_error_m'] == pytest.approx(0.4)
    assert pair['protected_time_below_1m'] == pytest.approx(1.0)
    assert result['protected_better_post_recovery_peak_count'] == 1
    assert result['protected_better_terminal_error_count'] == 1


def test_direct_untrusted_transition_counts_as_degraded_response(
        tmp_path) -> None:
    path = tmp_path / 'direct_untrusted.csv'
    _write(path, [
        {
            'timestamp_sec': '1.0', 'event': 'fault_status',
            'profile': 'gnss_step_5m', 'fusion_mode': 'protected',
            'seed': '2505', 'fault_enabled': 'true',
            'fault_mode': 'step', 'sensor_fault': 'gnss',
            'gnss_trust_state': 'HEALTHY',
        },
        {
            'timestamp_sec': '1.6', 'event': 'gnss_trust_transition',
            'profile': 'gnss_step_5m', 'fusion_mode': 'protected',
            'seed': '2505', 'fault_enabled': 'true',
            'fault_mode': 'step', 'sensor_fault': 'gnss',
            'gnss_trust_state': 'UNTRUSTED',
        },
    ])
    result = analyze_fusion_csv(path)
    assert result['degraded_latency_sec'] == pytest.approx(0.6)
    assert result['untrusted_latency_sec'] == pytest.approx(0.6)
    assert result['estimator_health']['invalid_samples'] == 0


def test_wrong_sensor_gate_is_counted(tmp_path) -> None:
    path = tmp_path / 'imu.csv'
    _write(path, [{
        'timestamp_sec': '1.0', 'event': 'gating_state',
        'profile': 'imu_bias', 'fusion_mode': 'protected',
        'imu_fault_mode': 'bias', 'sensor_fault': 'imu',
        'gnss_gated': 'true', 'gating_state': 'GATED',
    }])
    result = analyze_fusion_csv(path)
    assert result['wrong_sensor_gate_count'] == 1


def test_comparison_quantifies_protected_peak_reduction() -> None:
    result = _comparison([
        {'scenario': 'step', 'fusion_mode': 'unprotected',
         'peak_fused_error_m': 4.0},
        {'scenario': 'step', 'fusion_mode': 'protected',
         'peak_fused_error_m': 1.0},
    ])
    assert result['step']['peak_error_reduction_m'] == pytest.approx(3.0)
    assert result['step']['peak_error_reduction_fraction'] == pytest.approx(
        0.75)


def test_comparison_retains_repeated_trials() -> None:
    result = _comparison([
        {'scenario': 'step', 'fusion_mode': 'unprotected',
         'peak_fused_error_m': 4.0},
        {'scenario': 'step', 'fusion_mode': 'protected',
         'peak_fused_error_m': 1.0},
        {'scenario': 'step', 'fusion_mode': 'unprotected',
         'peak_fused_error_m': 6.0},
        {'scenario': 'step', 'fusion_mode': 'protected',
         'peak_fused_error_m': 2.0},
    ])
    comparison = result['step']
    assert comparison['trial_counts'] == {
        'protected': 2, 'unprotected': 2}
    assert comparison['peak_error_statistics_by_mode'][
        'protected']['mean'] == pytest.approx(1.5)
    assert comparison['peak_error_reduction_m'] == pytest.approx(3.5)
    assert comparison['repeat_peak_error_reductions_m'] == [3.0, 4.0]


def test_comparison_reports_covariance_sweep_response() -> None:
    result = _comparison([
        {'scenario': 'covariance_normal', 'fusion_mode': 'protected',
         'position_error_during_fault': {'mean': 2.0},
         'peak_fused_error_m': 3.0,
         'estimator_health': {'gnss_measurement_covariance_x_m2': {
             'mean': 0.0}}},
        {'scenario': 'covariance_moderate', 'fusion_mode': 'protected',
         'position_error_during_fault': {'mean': 3.0},
         'peak_fused_error_m': 4.0,
         'estimator_health': {'gnss_measurement_covariance_x_m2': {
             'mean': 10.0}}},
        {'scenario': 'covariance_strong', 'fusion_mode': 'protected',
         'position_error_during_fault': {'mean': 1.0},
         'peak_fused_error_m': 2.0,
         'estimator_health': {'gnss_measurement_covariance_x_m2': {
             'mean': 100.0}}},
    ])
    sweep = result['covariance_sweep']
    assert sweep['moderate']['measurement_covariance_x_m2']['mean'] == 10.0
    assert sweep['moderate']['fault_mean_error_m']['mean'] == 3.0
    assert not sweep['mean_fault_error_nonincreasing']


def test_analysis_reports_freeze_confirmation_and_gate_latency(tmp_path) -> None:
    path = tmp_path / 'freeze.csv'
    _write(path, [
        {
            'timestamp_sec': '1.0', 'event': 'fault_status',
            'profile': 'gnss_freeze', 'fusion_mode': 'protected',
            'seed': '2505', 'fault_enabled': 'true', 'fault_mode': 'freeze',
            'sensor_fault': 'gnss',
        },
        {
            'timestamp_sec': '2.0', 'event': 'gnss_diagnostic',
            'profile': 'gnss_freeze', 'fault_enabled': 'true',
            'fault_mode': 'freeze', 'gnss_trust_state': 'DEGRADED',
            'gnss_freeze_state': 'SUSPECT',
        },
        {
            'timestamp_sec': '3.0', 'event': 'gnss_diagnostic',
            'profile': 'gnss_freeze', 'fault_enabled': 'true',
            'fault_mode': 'freeze', 'gnss_trust_state': 'UNTRUSTED',
            'gnss_freeze_state': 'CONFIRMED',
        },
        {
            'timestamp_sec': '3.2', 'event': 'gating_state',
            'profile': 'gnss_freeze', 'fault_enabled': 'true',
            'fault_mode': 'freeze', 'gnss_gated': 'true',
        },
        {
            'timestamp_sec': '3.2', 'event': 'estimator_health',
            'profile': 'gnss_freeze', 'fault_enabled': 'true',
            'fault_mode': 'freeze', 'fused_reference_error_m': '0.9',
            'estimator_restart_count': '0',
        },
        {
            'timestamp_sec': '3.1', 'event': 'system_trust',
            'profile': 'gnss_freeze', 'fault_enabled': 'true',
            'fault_mode': 'freeze',
            'system_attribution': 'LIKELY_GNSS_FAULT',
        },
    ])
    result = analyze_fusion_csv(path)
    assert result['freeze_detected']
    assert result['freeze_suspected']
    assert result['freeze_suspicion_latency_sec'] == pytest.approx(1.0)
    assert result['freeze_detection_latency_sec'] == pytest.approx(2.0)
    assert result['freeze_gate_latency_sec'] == pytest.approx(2.2)
    assert result['gate_activated']
    assert result['severity_response_correct']
    assert result['attribution_correct']
    assert result['estimator_restart_count'] == 0


def test_comparison_matches_seeds_and_preserves_worse_protected_run() -> None:
    result = _comparison([
        {'scenario': 'step', 'fusion_mode': 'protected', 'seed': '2505',
         'file': 'protected-2505.csv', 'peak_fused_error_m': 1.0,
         'position_error_during_fault': {'mean': 0.8}},
        {'scenario': 'step', 'fusion_mode': 'unprotected', 'seed': '2505',
         'file': 'control-2505.csv', 'peak_fused_error_m': 4.0,
         'position_error_during_fault': {'mean': 3.0}},
        {'scenario': 'step', 'fusion_mode': 'protected', 'seed': '2506',
         'file': 'protected-2506.csv', 'peak_fused_error_m': 5.0,
         'position_error_during_fault': {'mean': 4.0}},
        {'scenario': 'step', 'fusion_mode': 'unprotected', 'seed': '2506',
         'file': 'control-2506.csv', 'peak_fused_error_m': 4.0,
         'position_error_during_fault': {'mean': 3.0}},
    ])
    comparison = result['step']
    assert comparison['unique_seed_count'] == 2
    assert comparison['protected_better_pair_count'] == 1
    assert comparison['protected_better_pair_fraction'] == pytest.approx(0.5)
    assert [pair['peak_error_improvement_m']
            for pair in comparison['matched_pairs']] == [3.0, -1.0]
    assert comparison['worst_protected_run']['file'] == 'protected-2506.csv'
    assert comparison['worst_protected_run']['peak_error_m'] == 5.0


def test_covariance_campaign_groups_by_configured_multiplier() -> None:
    result = _comparison([
        {
            'scenario': 'covariance_step5', 'fusion_mode': 'protected',
            'seed': '2505', 'covariance_multiplier': 1.0,
            'covariance_scale_during_fault': {'mean': 1.0},
            'position_error_during_fault': {'mean': 3.0, 'median': 2.8},
            'peak_fused_error_m': 4.0,
            'estimator_health': {
                'covariance_trace': {'mean': 0.5},
                'gnss_measurement_covariance_x_m2': {'mean': 0.0},
                'gnss_innovation_m': {'mean': 1.2},
            },
            'degraded_latency_sec': 0.5, 'untrusted_latency_sec': 1.0,
            'gate_latency_sec': None, 'final_error_m': 0.4,
        },
        {
            'scenario': 'covariance_step5', 'fusion_mode': 'protected',
            'seed': '2505', 'covariance_multiplier': 2.0,
            'covariance_scale_during_fault': {'mean': 2.0},
            'position_error_during_fault': {'mean': 2.5, 'median': 2.3},
            'peak_fused_error_m': 3.0,
            'estimator_health': {
                'covariance_trace': {'mean': 0.8},
                'gnss_measurement_covariance_x_m2': {'mean': 1.0},
                'gnss_innovation_m': {'mean': 1.4},
            },
            'degraded_latency_sec': 0.5, 'untrusted_latency_sec': 1.0,
            'gate_latency_sec': None, 'final_error_m': 0.5,
        },
    ])
    campaign = result['covariance_campaign']
    assert campaign['1']['trial_count'] == 1
    assert campaign['1']['fault_window_median_error_m']['mean'] == 2.8
    assert campaign['2']['gnss_measurement_covariance_x_m2']['mean'] == 1.0
    assert campaign['1']['rejection_metric_available'] is False
    paired = campaign['2']['matched_to_1x']
    assert paired['pair_count'] == 1
    assert paired['pairs'][0]['mean_error_reduction_m'] == pytest.approx(0.5)
    assert paired['mean_error_better_pair_count'] == 1


def test_healthy_confirmed_freeze_is_reported_as_false_positive(tmp_path) -> None:
    path = tmp_path / 'healthy_freeze.csv'
    _write(path, [{
        'timestamp_sec': '5.0', 'event': 'gnss_diagnostic',
        'profile': 'healthy_calibration_sweep', 'fault_enabled': 'false',
        'fault_mode': 'none', 'gnss_freeze_state': 'CONFIRMED',
        'gnss_trust_state': 'HEALTHY',
    }])
    result = analyze_fusion_csv(path)
    assert result['freeze_detected']
    assert result['freeze_false_positive']
    assert result['severity_response_correct']


def test_legacy_freeze_diagnostic_reason_remains_analyzable(tmp_path) -> None:
    path = tmp_path / 'legacy_freeze.csv'
    _write(path, [
        {
            'timestamp_sec': '1.0', 'event': 'fault_status',
            'profile': 'gnss_freeze', 'fusion_mode': 'protected',
            'seed': '2505', 'fault_enabled': 'true', 'fault_mode': 'freeze',
            'sensor_fault': 'gnss',
        },
        {
            'timestamp_sec': '6.0', 'event': 'gnss_diagnostic',
            'profile': 'gnss_freeze', 'fault_enabled': 'true',
            'fault_mode': 'freeze', 'gnss_trust_state': 'UNTRUSTED',
            'diagnostic_reason': 'GNSS_FREEZE_CONFIRMED',
        },
    ])
    result = analyze_fusion_csv(path)
    assert result['freeze_detected']
    assert result['freeze_detection_latency_sec'] == pytest.approx(5.0)
