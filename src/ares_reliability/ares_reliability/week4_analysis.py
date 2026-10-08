#!/usr/bin/env python3
"""Analyze Week 4 calibration, attribution, timing, and recovery evidence."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics as python_statistics
from typing import Any, Iterable, Optional

from ares_reliability.experiment_matrix import infer_expected_severity
import yaml  # type: ignore[import-untyped]


def _number(value: object) -> Optional[float]:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _percentile(values: list[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def statistics(values: Iterable[float]) -> dict[str, Any]:
    """Return the complete Week 4 descriptive-statistics set."""
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return dict.fromkeys((
            'mean', 'median', 'stddev', 'p90', 'p95', 'p99', 'minimum',
            'maximum')) | {'count': 0}
    return {
        'count': len(finite), 'mean': python_statistics.fmean(finite),
        'median': python_statistics.median(finite),
        'stddev': python_statistics.pstdev(finite),
        'p90': _percentile(finite, 0.90),
        'p95': _percentile(finite, 0.95),
        'p99': _percentile(finite, 0.99),
        'minimum': min(finite), 'maximum': max(finite),
    }


def pearson(first: list[float], second: list[float]) -> Optional[float]:
    """Return Pearson correlation for paired finite samples."""
    pairs = [(x, y) for x, y in zip(first, second)
             if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return None
    xs, ys = zip(*pairs)
    x_mean = python_statistics.fmean(xs)
    y_mean = python_statistics.fmean(ys)
    numerator = sum((x - x_mean) * (y - y_mean) for x, y in pairs)
    denominator = math.sqrt(
        sum((x - x_mean) ** 2 for x in xs) *
        sum((y - y_mean) ** 2 for y in ys))
    return None if denominator == 0.0 else numerator / denominator


def _active(row: dict[str, str]) -> bool:
    return (
        (row.get('fault_enabled', '').lower() == 'true' and
         row.get('fault_mode', 'none') != 'none') or
        row.get('imu_fault_mode', 'none') not in ('', 'none', 'unknown') or
        row.get('wheel_fault_mode', 'none') not in ('', 'none', 'unknown')
    )


def _fault_sensors(rows: list[dict[str, str]]) -> set[str]:
    sensors = set()
    for row in rows:
        if not _active(row):
            continue
        if (row.get('fault_enabled', '').lower() == 'true' and
                row.get('fault_mode', 'none') != 'none'):
            sensors.add('gnss')
        if row.get('imu_fault_mode', 'none') not in ('', 'none', 'unknown'):
            sensors.add('imu')
        if row.get('wheel_fault_mode', 'none') not in ('', 'none', 'unknown'):
            sensors.add('odometry')
    return sensors


def _severity_response_correct(expected: str, degraded: bool,
                               untrusted: bool) -> Optional[bool]:
    if expected == 'HEALTHY':
        return not degraded and not untrusted
    if expected == 'DEGRADED':
        return degraded
    if expected == 'UNTRUSTED':
        return untrusted
    if expected == 'HEALTHY_OR_DEGRADED':
        return not untrusted
    if expected == 'DEGRADED_OR_UNTRUSTED':
        return degraded or untrusted
    return None


def _attribution_correct(actual: str,
                         expected: Optional[str]) -> Optional[bool]:
    if not expected:
        return None
    return actual in expected.split('_OR_')


def _values(rows: list[dict[str, str]], event: str,
            column: str) -> list[float]:
    return [value for row in rows if row.get('event') == event
            for value in [_number(row.get(column))] if value is not None]


def _transition_metrics(
    rows: list[dict[str, str]],
    onset: Optional[float],
    final_time: float,
    state_column: str = 'gnss_trust_state',
) -> dict[str, Any]:
    labeled_starts = [
        timestamp for row in rows
        for timestamp in [_number(row.get('timestamp_sec'))]
        if timestamp is not None and (
            'motion_regime' not in row or
            row.get('motion_regime') not in ('', 'UNKNOWN'))
    ]
    audit_start = min(labeled_starts) if labeled_starts else -math.inf
    transitions = []
    false_degraded = 0
    false_untrusted = 0
    degraded_dwell = 0.0
    degraded_since: Optional[float] = None
    previous_state = 'UNKNOWN'
    for row in rows:
        timestamp = _number(row.get('timestamp_sec'))
        if timestamp is None or timestamp < audit_start:
            continue
        state = row.get(state_column, 'UNKNOWN')
        if state in ('', 'UNKNOWN') or state == previous_state:
            continue
        previous_state = state
        transitions.append((timestamp, state))
        healthy_period = onset is None or timestamp < onset
        if healthy_period and state == 'DEGRADED':
            false_degraded += 1
        if healthy_period and state == 'UNTRUSTED':
            false_untrusted += 1
        if state == 'DEGRADED' and degraded_since is None:
            degraded_since = timestamp
        elif state != 'DEGRADED' and degraded_since is not None:
            degraded_dwell += max(0.0, timestamp - degraded_since)
            degraded_since = None
    if degraded_since is not None:
        degraded_dwell += max(0.0, final_time - degraded_since)
    duration = max(0.0, final_time - (
        _number(rows[0].get('timestamp_sec')) or final_time)) if rows else 0.0
    return {
        'false_degraded_transitions': false_degraded,
        'false_untrusted_transitions': false_untrusted,
        'degraded_dwell_sec': degraded_dwell,
        'transition_count': len(transitions),
        'transition_frequency_per_min': (
            len(transitions) * 60.0 / duration if duration > 0.0 else None),
    }


def analyze_csv(
    path: Path,
    expected_severity: str = 'UNKNOWN',
    expected_attribution: Optional[str] = None,
) -> dict[str, Any]:
    """Analyze one Week 4 scenario CSV without claiming unobserved success."""
    with path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    profile = rows[0].get('profile', path.stem) if rows else path.stem
    active_rows = [row for row in rows if _active(row)]
    fault_sensors = _fault_sensors(active_rows)
    onset = _number(active_rows[0].get('timestamp_sec')) if active_rows else None
    final_time = _number(rows[-1].get('timestamp_sec')) if rows else 0.0
    final_time = final_time or 0.0
    untrusted_at = None
    degraded_at = None
    recovered_at = None
    recovery_onset = None
    last_active = False
    attribution = 'NOT_OBSERVED'
    confidence = None
    fault_attribution_counts: dict[str, int] = {}
    peak_fault_confidence: dict[str, float] = {}
    active_attribution = None
    active_confidence = None
    gate_activations = 0
    gate_releases = 0
    probation_entries = 0
    previous_gate = None
    previous_recovery = None
    for row in rows:
        timestamp = _number(row.get('timestamp_sec'))
        if timestamp is None:
            continue
        active = _active(row)
        if last_active and not active and recovery_onset is None:
            recovery_onset = timestamp
        last_active = active
        state_by_sensor = {
            sensor: row.get(f'{sensor}_trust_state')
            for sensor in ('gnss', 'imu', 'odometry', 'localization')
        }
        monitored = fault_sensors or set(state_by_sensor)
        states = tuple(state_by_sensor[sensor] for sensor in monitored)
        if onset is not None and timestamp >= onset:
            if degraded_at is None and 'DEGRADED' in states:
                degraded_at = timestamp
            if untrusted_at is None and 'UNTRUSTED' in states:
                untrusted_at = timestamp
        if (recovery_onset is not None and timestamp >= recovery_onset and
                recovered_at is None and fault_sensors and
                all(state_by_sensor[sensor] == 'HEALTHY'
                    for sensor in fault_sensors)):
            recovered_at = timestamp
        if row.get('event') == 'system_trust':
            attribution = row.get('system_attribution', attribution)
            confidence = _number(row.get('attribution_confidence'))
            if active:
                active_attribution = attribution
                active_confidence = confidence
                fault_attribution_counts[attribution] = (
                    fault_attribution_counts.get(attribution, 0) + 1)
                if confidence is not None:
                    peak_fault_confidence[attribution] = max(
                        confidence,
                        peak_fault_confidence.get(attribution, -math.inf))
        gated = row.get('gnss_gated', '').lower()
        audit_active = row.get('motion_regime') not in ('', 'UNKNOWN')
        if gated in ('true', 'false'):
            gate = gated == 'true'
            if audit_active and gate and previous_gate is False:
                gate_activations += 1
            if audit_active and not gate and previous_gate is True:
                gate_releases += 1
            previous_gate = gate
        recovery_state = row.get('recovery_state')
        if (audit_active and recovery_state == 'PROBATION' and
                previous_recovery != 'PROBATION'):
            probation_entries += 1
        if recovery_state:
            previous_recovery = recovery_state
    fault_hypotheses = [
        name for name in peak_fault_confidence if name.startswith('LIKELY_')
    ]
    if fault_hypotheses:
        attribution = max(
            fault_hypotheses,
            key=lambda name: (
                peak_fault_confidence[name],
                fault_attribution_counts.get(name, 0),
            ),
        )
        confidence = peak_fault_confidence[attribution]
    elif active_attribution is not None:
        attribution = active_attribution
        confidence = active_confidence
    sensor_statistics = {
        'gnss_residual_m': statistics(_values(
            rows, 'diagnostic', 'instantaneous_residual_m')),
        'gnss_rolling_mean_m': statistics(_values(
            rows, 'diagnostic', 'rolling_mean_residual_m')),
        'imu_residual_radps': statistics(_values(
            rows, 'imu_diagnostic', 'imu_instantaneous_residual_radps')),
        'localization_residual_m': statistics(_values(
            rows, 'localization_diagnostic',
            'localization_instantaneous_residual_m')),
        'speed_mps': statistics([
            value for row in rows
            for value in [_number(row.get('robot_speed_mps'))]
            if value is not None]),
        'yaw_rate_radps': statistics([
            value for row in rows
            for value in [_number(row.get('robot_yaw_rate_radps'))]
            if value is not None]),
        'sync_error_sec': statistics([
            value for row in rows
            for key in ('gnss_sync_error_sec', 'imu_sync_error_sec',
                        'localization_sync_error_sec')
            for value in [_number(row.get(key))] if value is not None]),
    }
    statistics_by_motion_regime = {}
    regimes = sorted({row.get('motion_regime', '') for row in rows
                      if row.get('motion_regime') not in ('', 'UNKNOWN')})
    for regime in regimes:
        regime_rows = [row for row in rows
                       if row.get('motion_regime') == regime]
        statistics_by_motion_regime[regime] = {
            'gnss_residual_m': statistics(_values(
                regime_rows, 'diagnostic', 'instantaneous_residual_m')),
            'gnss_rolling_mean_m': statistics(_values(
                regime_rows, 'diagnostic', 'rolling_mean_residual_m')),
            'imu_residual_radps': statistics(_values(
                regime_rows, 'imu_diagnostic',
                'imu_instantaneous_residual_radps')),
            'localization_residual_m': statistics(_values(
                regime_rows, 'localization_diagnostic',
                'localization_instantaneous_residual_m')),
        }
    correlation_rows = [row for row in rows if row.get('event') == 'diagnostic']
    residual = [_number(row.get('instantaneous_residual_m'))
                for row in correlation_rows]
    correlation = {}
    for name, column in (
            ('linear_speed', 'robot_speed_mps'),
            ('absolute_yaw_rate', 'robot_yaw_rate_radps'),
            ('acceleration', 'robot_acceleration_mps2'),
            ('amcl_covariance_x', 'localization_covariance_x_m2'),
            ('sync_error', 'gnss_sync_error_sec')):
        pairs = [
            (x, y) for row, x in zip(correlation_rows, residual)
            for y in [_number(row.get(column))]
            if x is not None and y is not None]
        if name == 'absolute_yaw_rate':
            pairs = [(x, abs(y)) for x, y in pairs]
        correlation[name] = pearson(
            [pair[0] for pair in pairs], [pair[1] for pair in pairs])
    offset_at_detection = None
    if untrusted_at is not None:
        candidates = [row for row in rows
                      if (_number(row.get('timestamp_sec')) or -math.inf) <=
                      untrusted_at]
        if candidates:
            east = _number(candidates[-1].get('effective_east_offset_m'))
            north = _number(candidates[-1].get('effective_north_offset_m'))
            if east is not None and north is not None:
                offset_at_detection = math.hypot(east, north)
    transition_metrics_by_sensor = {
        sensor: _transition_metrics(
            rows, onset, final_time, f'{sensor}_trust_state')
        for sensor in ('gnss', 'imu', 'localization', 'odometry')
    }
    degraded_detected = degraded_at is not None
    untrusted_detected = untrusted_at is not None
    confidence_peak = (
        max(peak_fault_confidence.values())
        if peak_fault_confidence else confidence)
    return {
        'scenario': profile, 'file': str(path), 'seed': (
            rows[0].get('seed') if rows else None), 'rows': len(rows),
        'attack_onset_sec': onset, 'degraded_at_sec': degraded_at,
        'degraded_detected': degraded_detected,
        'untrusted_at_sec': untrusted_at,
        'untrusted_detected': untrusted_detected,
        'expected_severity': expected_severity,
        'severity_response_correct': _severity_response_correct(
            expected_severity, degraded_detected, untrusted_detected),
        # Backward-compatible aliases. Legacy detection means UNTRUSTED.
        'detected_at_sec': untrusted_at,
        'detection_success': untrusted_detected,
        'degraded_latency_sec': (
            degraded_at - onset if degraded_at is not None and onset is not None
            else None),
        'untrusted_latency_sec': (
            untrusted_at - onset
            if untrusted_at is not None and onset is not None else None),
        'detection_latency_sec': (
            untrusted_at - onset if untrusted_at is not None and onset is not None
            else None),
        'recovery_detected': recovered_at is not None,
        'recovery_onset_sec': recovery_onset,
        'recovery_latency_sec': (
            recovered_at - recovery_onset if recovered_at is not None and
            recovery_onset is not None else None),
        'displacement_at_detection_m': offset_at_detection,
        'attribution_result': attribution,
        'attribution_confidence': confidence,
        'attribution_confidence_peak': confidence_peak,
        'attribution_correct': _attribution_correct(
            attribution, expected_attribution),
        'fault_attribution_counts': fault_attribution_counts,
        'peak_fault_confidence_by_attribution': peak_fault_confidence,
        'sensor_statistics': sensor_statistics,
        'statistics_by_motion_regime': statistics_by_motion_regime,
        'motion_correlations': correlation,
        'transition_metrics': transition_metrics_by_sensor['gnss'],
        'transition_metrics_by_sensor': transition_metrics_by_sensor,
        'recovery_stress': {
            'gate_activations': gate_activations,
            'gate_releases': gate_releases,
            'probation_entries': probation_entries,
            'recovery_oscillations': max(0, probation_entries - 1),
        },
        'timing_rejection_reasons': sorted({
            row.get('gnss_sync_reason', '') for row in rows
            if row.get('gnss_sync_reason') not in ('', 'OK', 'UNKNOWN')}),
    }


def _load_expectations(path: Path) -> dict[str, dict[str, str]]:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    return {
        name: {
            'severity': infer_expected_severity(scenario),
            'attribution': str(scenario['expected_attribution_category']),
        }
        for name, scenario in document['scenarios'].items()
    }


def _distribution(experiments: list[dict[str, Any]],
                  key: str) -> dict[str, Any]:
    return statistics([
        value for experiment in experiments
        for value in [_number(experiment.get(key))] if value is not None])


def main(argv: Optional[list[str]] = None) -> None:
    """Analyze CSV files and write Week 4 summary/calibration JSON."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_files', nargs='+', type=Path)
    parser.add_argument('--output', type=Path,
                        default=Path('results/week4/summary.json'))
    parser.add_argument('--calibration-output', type=Path,
                        default=Path(
                            'results/week4/calibration_summary.json'))
    parser.add_argument('--matrix', type=Path, default=Path(
        'src/ares_reliability/config/week4_fault_matrix.yaml'))
    args = parser.parse_args(argv)
    expectations = _load_expectations(args.matrix)
    experiments = []
    for path in args.csv_files:
        with path.open(encoding='utf-8', newline='') as stream:
            first = next(csv.DictReader(stream), {})
        profile = first.get('profile', path.stem)
        expected = expectations.get(profile, {})
        experiments.append(analyze_csv(
            path,
            expected_severity=expected.get('severity', 'UNKNOWN'),
            expected_attribution=expected.get('attribution'),
        ))
    summary = {
        'schema_version': 2,
        'evidence_only': True,
        'experiments': experiments,
        'degraded_latency_distribution_sec': _distribution(
            experiments, 'degraded_latency_sec'),
        'untrusted_latency_distribution_sec': _distribution(
            experiments, 'untrusted_latency_sec'),
        'severity_response_counts': {
            'correct': sum(
                experiment['severity_response_correct'] is True
                for experiment in experiments),
            'incorrect': sum(
                experiment['severity_response_correct'] is False
                for experiment in experiments),
            'not_evaluated': sum(
                experiment['severity_response_correct'] is None
                for experiment in experiments),
        },
        'attribution_response_counts': {
            'correct': sum(
                experiment['attribution_correct'] is True
                for experiment in experiments),
            'incorrect': sum(
                experiment['attribution_correct'] is False
                for experiment in experiments),
            'not_evaluated': sum(
                experiment['attribution_correct'] is None
                for experiment in experiments),
        },
        # Backward-compatible aggregate. Legacy detection means UNTRUSTED.
        'detection_latency_distribution_sec': _distribution(
            experiments, 'detection_latency_sec'),
        'recovery_latency_distribution_sec': _distribution(
            experiments, 'recovery_latency_sec'),
        'displacement_at_detection_distribution_m': _distribution(
            experiments, 'displacement_at_detection_m'),
    }
    healthy = [experiment for experiment in experiments
               if experiment['scenario'].startswith('healthy_')]
    false_transitions = sum(
        metrics[key]
        for experiment in healthy
        for metrics in experiment['transition_metrics_by_sensor'].values()
        for key in ('false_degraded_transitions',
                    'false_untrusted_transitions')
    )
    calibration = {
        'threshold_status': (
            'healthy_sweep_passed' if false_transitions == 0
            else 'retuning_required'),
        'healthy_false_transition_count': false_transitions,
        'healthy_motion_scenarios': healthy,
        'motion_conditioning_recommendation': (
            'Do not enable motion-conditioned thresholds without significant '
            'and repeatable correlations across multiple healthy regimes.'),
    }
    rendered = json.dumps(summary, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + '\n', encoding='utf-8')
    args.calibration_output.parent.mkdir(parents=True, exist_ok=True)
    args.calibration_output.write_text(
        json.dumps(calibration, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')


if __name__ == '__main__':
    main()
