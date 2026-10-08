#!/usr/bin/env python3
"""Analyze Week 5 active-fusion, recovery, and estimator-health evidence."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
from typing import Any, Callable, Iterable, Optional

from ares_reliability.week4_analysis import pearson, statistics


SUSTAINED_SAMPLE_GAP_TOLERANCE_SEC = 0.25
ERROR_SLOPE_NEAR_ZERO_TOLERANCE_MPS = 0.01
STANDARD_POST_NORMAL_HORIZON_SEC = 60.0


def _number(value: object) -> Optional[float]:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _true(value: object) -> bool:
    return str(value).lower() in ('true', '1', 'yes')


def _freeze_state(row: dict[str, str]) -> str:
    state = row.get('gnss_freeze_state')
    if state:
        return state
    reason = row.get('diagnostic_reason', '')
    return reason.removeprefix('GNSS_FREEZE_') if reason.startswith(
        'GNSS_FREEZE_') else 'UNKNOWN'


def _fault_active(row: dict[str, str]) -> bool:
    return (
        (_true(row.get('fault_enabled')) and
         row.get('fault_mode') not in ('', 'none', 'unknown')) or
        row.get('imu_fault_mode', 'none') not in ('', 'none', 'unknown') or
        row.get('wheel_fault_mode', 'none') not in ('', 'none', 'unknown')
    )


def _first_time(rows: Iterable[dict[str, str]],
                predicate: Callable[[dict[str, str]], bool]) -> Optional[float]:
    for row in rows:
        timestamp = _number(row.get('timestamp_sec'))
        if timestamp is not None and predicate(row):
            return timestamp
    return None


def _value_at_or_after(rows: list[dict[str, str]], timestamp: Optional[float],
                       column: str) -> Optional[float]:
    if timestamp is None:
        return None
    for row in rows:
        row_time = _number(row.get('timestamp_sec'))
        value = _number(row.get(column))
        if row_time is not None and row_time >= timestamp and value is not None:
            return value
    return None


def _last_value_before(rows: list[dict[str, str]], timestamp: Optional[float],
                       column: str) -> Optional[float]:
    if timestamp is None:
        return None
    result = None
    for row in rows:
        row_time = _number(row.get('timestamp_sec'))
        value = _number(row.get(column))
        if row_time is not None and row_time >= timestamp:
            break
        if row_time is not None and value is not None:
            result = value
    return result


def _time_to_threshold(errors: list[tuple[float, float]],
                       recovery_time: Optional[float],
                       threshold: float) -> Optional[float]:
    if recovery_time is None:
        return None
    for timestamp, value in errors:
        if timestamp >= recovery_time and value <= threshold:
            return timestamp - recovery_time
    return None


def _time_to_sustained_threshold(
    errors: list[tuple[float, float]],
    reference_time: Optional[float],
    threshold: float,
    duration_sec: float,
    max_sample_gap_sec: float = SUSTAINED_SAMPLE_GAP_TOLERANCE_SEC,
) -> Optional[float]:
    """Return first threshold entry sustained by continuous sampled evidence."""
    if reference_time is None:
        return None
    below_since: Optional[float] = None
    previous_time: Optional[float] = None
    for timestamp, value in errors:
        if timestamp < reference_time:
            continue
        if (previous_time is not None and
                timestamp - previous_time > max_sample_gap_sec):
            below_since = None
        if value <= threshold:
            if below_since is None:
                below_since = timestamp
            if timestamp - below_since >= duration_sec:
                return below_since - reference_time
        else:
            below_since = None
        previous_time = timestamp
    return None


def _time_fraction_below(
    errors: list[tuple[float, float]],
    start_time: Optional[float],
    end_time: Optional[float],
    threshold: Optional[float],
) -> Optional[float]:
    """Return time-weighted occupancy using each sample until the next sample."""
    if (start_time is None or end_time is None or threshold is None or
            end_time <= start_time):
        return None
    below_duration = 0.0
    observed_duration = 0.0
    for (timestamp, value), (next_time, _) in zip(errors, errors[1:]):
        interval_start = max(timestamp, start_time)
        interval_end = min(next_time, end_time)
        if interval_end <= interval_start:
            continue
        duration = interval_end - interval_start
        observed_duration += duration
        if value <= threshold:
            below_duration += duration
    return (
        below_duration / observed_duration
        if observed_duration > 0.0 else None)


def _window_error_statistics(
    errors: list[tuple[float, float]],
    normal_time: Optional[float],
    start_offset_sec: float,
    end_offset_sec: float,
) -> dict[str, Any]:
    """Summarize errors in one half-open post-NORMAL time window."""
    if normal_time is None:
        return statistics([])
    start = normal_time + start_offset_sec
    end = normal_time + end_offset_sec
    return statistics(
        value for timestamp, value in errors if start <= timestamp < end)


def _error_slope(
    errors: list[tuple[float, float]],
    normal_time: Optional[float],
    start_offset_sec: float,
    end_offset_sec: float,
) -> Optional[float]:
    """Return least-squares error slope in metres per simulated second."""
    if normal_time is None:
        return None
    start = normal_time + start_offset_sec
    end = normal_time + end_offset_sec
    samples = [(timestamp - normal_time, value)
               for timestamp, value in errors if start <= timestamp < end]
    if len(samples) < 2:
        return None
    times = [sample[0] for sample in samples]
    values = [sample[1] for sample in samples]
    time_mean = sum(times) / len(times)
    value_mean = sum(values) / len(values)
    denominator = sum((value - time_mean) ** 2 for value in times)
    if denominator == 0.0:
        return None
    return sum(
        (timestamp - time_mean) * (value - value_mean)
        for timestamp, value in samples) / denominator


def _slope_classification(slope: Optional[float]) -> Optional[str]:
    if slope is None:
        return None
    if slope > ERROR_SLOPE_NEAR_ZERO_TOLERANCE_MPS:
        return 'DRIFTING'
    if slope < -ERROR_SLOPE_NEAR_ZERO_TOLERANCE_MPS:
        return 'CONVERGING'
    return 'STABLE'


def _post_normal_correlations(
    health: list[dict[str, str]],
    normal_time: Optional[float],
    end_time: Optional[float],
) -> dict[str, Optional[float]]:
    """Correlate post-NORMAL error with recorded sensor/estimator signals."""
    columns = (
        'instantaneous_residual_m',
        'rolling_mean_residual_m',
        'imu_rolling_mean_residual_radps',
        'localization_rolling_mean_residual_m',
        'robot_speed_mps',
        'robot_yaw_rate_radps',
        'gnss_sync_error_sec',
        'fused_covariance_trace',
        'gnss_innovation_m',
        'gnss_measurement_covariance_x_m2',
    )
    result: dict[str, Optional[float]] = {}
    for column in columns:
        error_values: list[float] = []
        signal_values: list[float] = []
        for row in health:
            timestamp = _number(row.get('timestamp_sec'))
            error = _number(row.get('fused_reference_error_m'))
            signal = _number(row.get(column))
            if (timestamp is not None and error is not None and
                    signal is not None and normal_time is not None and
                    end_time is not None and
                    normal_time <= timestamp <= end_time):
                error_values.append(error)
                signal_values.append(signal)
        result[column] = pearson(error_values, signal_values)
    return result


def analyze_fusion_csv(path: Path) -> dict[str, Any]:
    """Return evidence-backed fusion, recovery, and health metrics."""
    with path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    first = rows[0] if rows else {}
    health = [row for row in rows if row.get('event') == 'estimator_health']
    active_rows = [row for row in rows if _fault_active(row)]
    onset = _number(active_rows[0].get('timestamp_sec')) if active_rows else None
    recovery_onset = None
    observed_active = False
    for row in rows:
        active = _fault_active(row)
        if active:
            observed_active = True
        elif observed_active:
            recovery_onset = _number(row.get('timestamp_sec'))
            break
    # Reaching UNTRUSTED necessarily satisfies a DEGRADED-or-worse response,
    # even when the sampled trust stream transitions directly to UNTRUSTED.
    degraded_at = _first_time(
        rows, lambda row: onset is not None and
        (_number(row.get('timestamp_sec')) or -math.inf) >= onset and
        row.get('gnss_trust_state') in ('DEGRADED', 'UNTRUSTED'))
    untrusted_at = _first_time(
        rows, lambda row: onset is not None and
        (_number(row.get('timestamp_sec')) or -math.inf) >= onset and
        row.get('gnss_trust_state') == 'UNTRUSTED')
    inflation_at = _first_time(
        rows, lambda row: onset is not None and
        (_number(row.get('timestamp_sec')) or -math.inf) >= onset and
        (_number(row.get('covariance_scale')) or 1.0) > 1.0)
    gated_at = _first_time(
        rows, lambda row: onset is not None and
        (_number(row.get('timestamp_sec')) or -math.inf) >= onset and
        (_true(row.get('gnss_gated')) or
         row.get('gating_state') == 'GATED' or
         row.get('recovery_state') == 'GATED'))
    freeze_confirmed_at = _first_time(
        rows, lambda row: row.get('event') == 'gnss_diagnostic' and
        _freeze_state(row) == 'CONFIRMED')
    freeze_confirmed_after_onset = _first_time(
        rows, lambda row: row.get('event') == 'gnss_diagnostic' and
        _freeze_state(row) == 'CONFIRMED' and onset is not None and
        (_number(row.get('timestamp_sec')) or -math.inf) >= onset)
    freeze_suspected_after_onset = _first_time(
        rows, lambda row: row.get('event') == 'gnss_diagnostic' and
        _freeze_state(row) in ('SUSPECT', 'CONFIRMED') and onset is not None and
        (_number(row.get('timestamp_sec')) or -math.inf) >= onset)
    probation_at = _first_time(
        rows, lambda row: recovery_onset is not None and
        (_number(row.get('timestamp_sec')) or -math.inf) >= recovery_onset and
        row.get('recovery_state') == 'PROBATION')
    normal_at = _first_time(
        rows, lambda row: recovery_onset is not None and
        (_number(row.get('timestamp_sec')) or -math.inf) >= recovery_onset and
        row.get('recovery_state') == 'NORMAL')
    gate_release_at = _first_time(
        rows, lambda row: gated_at is not None and
        (_number(row.get('timestamp_sec')) or -math.inf) >= gated_at and
        row.get('event') == 'gating_state' and
        not _true(row.get('gnss_gated')))
    explicit_reentry_at = _first_time(
        rows, lambda row: gate_release_at is not None and
        (_number(row.get('timestamp_sec')) or -math.inf) >= gate_release_at and
        row.get('event') == 'gnss_reentry' and
        _true(row.get('gnss_forwarded')))
    errors = [
        (timestamp, value) for row in health
        for timestamp in [_number(row.get('timestamp_sec'))]
        for value in [_number(row.get('fused_reference_error_m'))]
        if timestamp is not None and value is not None
    ]
    first_reentry_at = explicit_reentry_at
    reentry_health_row: Optional[dict[str, str]] = None
    if gate_release_at is not None:
        count_before_release = max([
            int(_number(row.get('gnss_measurement_count')) or 0)
            for row in health
            if (_number(row.get('timestamp_sec')) or math.inf) < gate_release_at
        ], default=0)
        reentry_health_row = next((
            row for row in health
            if (_number(row.get('timestamp_sec')) or -math.inf) >=
            gate_release_at and
            int(_number(row.get('gnss_measurement_count')) or 0) >
            count_before_release), None)
        if first_reentry_at is None and reentry_health_row is not None:
            first_reentry_at = _number(
                reentry_health_row.get('timestamp_sec'))
    if reentry_health_row is None and first_reentry_at is not None:
        reentry_health_row = next((
            row for row in health
            if (_number(row.get('timestamp_sec')) or -math.inf) >=
            first_reentry_at), None)
    reentry_health_at = (
        _number(reentry_health_row.get('timestamp_sec'))
        if reentry_health_row is not None else None)
    pre_fault = [value for timestamp, value in errors
                 if onset is None or timestamp < onset]
    during_fault = [
        value for timestamp, value in errors
        if onset is not None and timestamp >= onset and
        (recovery_onset is None or timestamp < recovery_onset)]
    after_gate = [value for timestamp, value in errors
                  if gated_at is not None and timestamp >= gated_at]
    post_recovery = [
        value for timestamp, value in errors
        if recovery_onset is not None and timestamp >= recovery_onset]
    pre_fault_statistics = statistics(pre_fault)
    baseline_band_upper = _number(pre_fault_statistics.get('p95'))
    error_at_fault_clear = _value_at_or_after(
        health, recovery_onset, 'fused_reference_error_m')
    post_recovery_peak = max(post_recovery) if post_recovery else None
    recovery_overshoot = (
        max(0.0, post_recovery_peak - error_at_fault_clear)
        if post_recovery_peak is not None and
        error_at_fault_clear is not None else None)
    recovery_samples = {
        f'error_recovery_plus_{offset_label}': _value_at_or_after(
            health,
            recovery_onset + offset if recovery_onset is not None else None,
            'fused_reference_error_m')
        for offset_label, offset in (
            ('0s', 0.0), ('1s', 1.0), ('2s', 2.0), ('5s', 5.0),
            ('10s', 10.0), ('15s', 15.0), ('20s', 20.0),
            ('30s', 30.0), ('45s', 45.0), ('60s', 60.0))
    }
    post_normal_samples = {
        f'post_normal_error_plus_{offset_label}': _value_at_or_after(
            health,
            normal_at + offset if normal_at is not None else None,
            'fused_reference_error_m')
        for offset_label, offset in (
            ('0s', 0.0), ('1s', 1.0), ('2s', 2.0), ('5s', 5.0),
            ('10s', 10.0), ('15s', 15.0), ('20s', 20.0),
            ('30s', 30.0), ('45s', 45.0), ('60s', 60.0))
    }
    last_error_at = errors[-1][0] if errors else None
    post_normal_duration = (
        last_error_at - normal_at
        if last_error_at is not None and normal_at is not None else None)
    post_normal_end = (
        min(last_error_at, normal_at + STANDARD_POST_NORMAL_HORIZON_SEC)
        if last_error_at is not None and normal_at is not None else None)
    post_normal_complete_60s = bool(
        post_normal_duration is not None and
        post_normal_duration >= (
            STANDARD_POST_NORMAL_HORIZON_SEC -
            SUSTAINED_SAMPLE_GAP_TOLERANCE_SEC))
    post_normal_windows = {
        '0_10s': _window_error_statistics(errors, normal_at, 0.0, 10.0),
        '10_30s': _window_error_statistics(errors, normal_at, 10.0, 30.0),
        '30_60s': _window_error_statistics(errors, normal_at, 30.0, 60.0),
    }
    post_normal_statistics = _window_error_statistics(
        errors, normal_at, 0.0, STANDARD_POST_NORMAL_HORIZON_SEC)
    post_normal_slopes = {
        '10_30s': _error_slope(errors, normal_at, 10.0, 30.0),
        '30_60s': _error_slope(errors, normal_at, 30.0, 60.0),
    }
    post_normal_fraction_below_2m = _time_fraction_below(
        errors, normal_at, post_normal_end, 2.0)
    post_normal_fraction_below_1m = _time_fraction_below(
        errors, normal_at, post_normal_end, 1.0)
    post_normal_fraction_inside_healthy_band = _time_fraction_below(
        errors, normal_at, post_normal_end, baseline_band_upper)
    post_normal_fraction_above_healthy_band = (
        1.0 - post_normal_fraction_inside_healthy_band
        if post_normal_fraction_inside_healthy_band is not None else None)
    sustained_below_2m_5s = _time_to_sustained_threshold(
        errors, recovery_onset, 2.0, 5.0)
    sustained_below_1m_5s = _time_to_sustained_threshold(
        errors, recovery_onset, 1.0, 5.0)
    sustained_below_1m_10s = _time_to_sustained_threshold(
        errors, recovery_onset, 1.0, 10.0)
    sustained_below_0p5m_5s = _time_to_sustained_threshold(
        errors, recovery_onset, 0.5, 5.0)
    sustained_healthy_5s = (
        _time_to_sustained_threshold(
            errors, recovery_onset, baseline_band_upper, 5.0)
        if baseline_band_upper is not None else None)
    sustained_healthy_10s = (
        _time_to_sustained_threshold(
            errors, recovery_onset, baseline_band_upper, 10.0)
        if baseline_band_upper is not None else None)
    post_normal_sustained_below_1m_5s = _time_to_sustained_threshold(
        errors, normal_at, 1.0, 5.0)
    post_normal_sustained_below_1m_10s = _time_to_sustained_threshold(
        errors, normal_at, 1.0, 10.0)
    post_normal_sustained_healthy_5s = (
        _time_to_sustained_threshold(
            errors, normal_at, baseline_band_upper, 5.0)
        if baseline_band_upper is not None else None)
    post_normal_sustained_healthy_10s = (
        _time_to_sustained_threshold(
            errors, normal_at, baseline_band_upper, 10.0)
        if baseline_band_upper is not None else None)
    post_normal_health = [
        row for row in health
        for timestamp in [_number(row.get('timestamp_sec'))]
        if timestamp is not None and normal_at is not None and
        post_normal_end is not None and normal_at <= timestamp <=
        post_normal_end
    ]
    post_normal_numeric_columns = (
        'instantaneous_residual_m', 'rolling_mean_residual_m',
        'imu_rolling_mean_residual_radps',
        'localization_rolling_mean_residual_m', 'robot_speed_mps',
        'robot_yaw_rate_radps', 'gnss_sync_error_sec',
        'fused_covariance_trace', 'gnss_innovation_m',
        'gnss_measurement_covariance_x_m2',
    )
    post_normal_sensor_statistics = {
        column: statistics(
            value for row in post_normal_health
            for value in [_number(row.get(column))]
            if value is not None)
        for column in post_normal_numeric_columns
    }
    post_normal_state_counts = {
        column: dict(Counter(
            row.get(column, 'UNKNOWN') for row in post_normal_health))
        for column in (
            'gnss_trust_state', 'imu_trust_state',
            'localization_trust_state', 'odometry_trust_state',
            'system_attribution', 'recovery_state', 'motion_regime')
    }
    late_median = _number(post_normal_windows['30_60s'].get('median'))
    late_slope = post_normal_slopes['30_60s']
    persistent_recovery_bias = (
        late_median > baseline_band_upper
        if post_normal_complete_60s and late_median is not None and
        baseline_band_upper is not None else None)
    persistent_recovery_drift = (
        late_slope > ERROR_SLOPE_NEAR_ZERO_TOLERANCE_MPS
        if post_normal_complete_60s and late_slope is not None else None)
    if not post_normal_complete_60s:
        recovery_classification = 'INSUFFICIENT_POST_NORMAL_HORIZON'
    elif persistent_recovery_drift:
        recovery_classification = 'PERSISTENT_DRIFT'
    elif persistent_recovery_bias:
        recovery_classification = 'PERSISTENT_BIAS'
    elif post_normal_sustained_healthy_5s is not None:
        recovery_classification = 'SUSTAINED_CONVERGENCE'
    else:
        recovery_classification = 'UNRESOLVED'
    first_reentry_row = next((
        row for row in rows
        if row.get('event') == 'gnss_reentry' and
        (first_reentry_at is None or
         (_number(row.get('timestamp_sec')) or -math.inf) >=
         first_reentry_at - 1.0e-6)), {})
    gate_release_message_age = _number(
        first_reentry_row.get('gnss_proxy_message_age_sec'))
    first_reentry_message_stamp = _number(
        first_reentry_row.get('gnss_message_stamp_sec'))
    first_reentry_generated_before_release = (
        _true(first_reentry_row.get('gnss_generated_before_gate_release'))
        if first_reentry_row else None)
    covariance_during_gate = [
        value for row in health
        for timestamp in [_number(row.get('timestamp_sec'))]
        for value in [_number(row.get('fused_covariance_trace'))]
        if timestamp is not None and value is not None and
        gated_at is not None and timestamp >= gated_at and
        (gate_release_at is None or timestamp < gate_release_at)]
    final_error = errors[-1][1] if errors else None
    rates = [value for row in health
             for value in [_number(row.get(
                 'estimator_publication_rate_hz'))]
             if value is not None]
    freshness = [value for row in health
                 for value in [_number(row.get('estimator_freshness_sec'))]
                 if value is not None]
    covariance = [value for row in health
                  for value in [_number(row.get('fused_covariance_trace'))]
                  if value is not None]
    innovations = [value for row in health
                   for value in [_number(row.get('gnss_innovation_m'))]
                   if value is not None]
    gnss_covariance = [
        value for row in health
        for value in [_number(row.get(
            'gnss_measurement_covariance_x_m2'))]
        if value is not None]
    statuses = Counter(row.get('estimator_status', 'UNKNOWN') for row in health)
    gate_times = [
        timestamp for row in rows
        for timestamp in [_number(row.get('timestamp_sec'))]
        if timestamp is not None and row.get('event') == 'gating_state' and
        _true(row.get('gnss_gated'))]
    fault_sensors: set[str] = {
        sensor for row in active_rows
        for sensor in [row.get('sensor_fault')]
        if sensor
    }
    gnss_fault = bool(fault_sensors & {'gnss', 'multi'})
    attributions = Counter(
        row.get('system_attribution', 'UNKNOWN') for row in rows
        if row.get('event') == 'system_trust' and _fault_active(row))
    scenario = first.get('profile', path.stem)
    expected = (
        'HEALTHY' if not active_rows else
        'DEGRADED_OR_UNTRUSTED'
        if scenario in ('gnss_slow_drift', 'covariance_normal',
                        'covariance_moderate', 'covariance_strong') else
        'UNTRUSTED' if gnss_fault else None
    )
    observed_states = {
        row.get('gnss_trust_state') for row in rows
        if row.get('event') == 'gnss_diagnostic'
    }
    if expected == 'HEALTHY':
        severity_correct = not observed_states.intersection(
            {'DEGRADED', 'UNTRUSTED'})
    elif expected == 'DEGRADED_OR_UNTRUSTED':
        severity_correct = bool(observed_states.intersection(
            {'DEGRADED', 'UNTRUSTED'}))
    elif expected == 'UNTRUSTED':
        severity_correct = untrusted_at is not None
    else:
        severity_correct = None
    expected_attributions = {
        'gnss': 'LIKELY_GNSS_FAULT', 'imu': 'LIKELY_IMU_FAULT',
        'wheel': 'LIKELY_ODOMETRY_FAULT',
    }
    fault_sensor = next(iter(fault_sensors)) if len(fault_sensors) == 1 else ''
    expected_attribution = expected_attributions.get(fault_sensor)
    attribution_correct = (
        attributions.get(expected_attribution, 0) > 0
        if expected_attribution is not None else None)
    freeze_states = Counter(
        _freeze_state(row) for row in rows
        if row.get('event') == 'gnss_diagnostic')
    active_covariance_scales = [
        value for row in active_rows
        for value in [_number(row.get('covariance_scale'))]
        if value is not None]
    covariance_multiplier = max(active_covariance_scales, default=1.0)
    return {
        'file': str(path),
        'scenario': scenario,
        'fusion_mode': first.get('fusion_mode', 'unknown'),
        'seed': first.get('seed'),
        'covariance_multiplier': covariance_multiplier,
        'rows': len(rows),
        'fault_onset_sec': onset,
        'recovery_onset_sec': recovery_onset,
        'degraded_at_sec': degraded_at,
        'degraded_detected': degraded_at is not None,
        'degraded_latency_sec': (
            degraded_at - onset if degraded_at is not None and onset is not None
            else None),
        'untrusted_at_sec': untrusted_at,
        'untrusted_detected': untrusted_at is not None,
        'untrusted_latency_sec': (
            untrusted_at - onset
            if untrusted_at is not None and onset is not None else None),
        'covariance_inflation_at_sec': inflation_at,
        'covariance_inflation_latency_sec': (
            inflation_at - onset
            if inflation_at is not None and onset is not None else None),
        'gated_at_sec': gated_at,
        'gating_latency_sec': (
            gated_at - onset
            if gated_at is not None and onset is not None else None),
        'gate_activated': bool(gate_times),
        'gate_latency_sec': (
            gated_at - onset
            if gated_at is not None and onset is not None else None),
        'severity_response_correct': severity_correct,
        'attribution_correct': attribution_correct,
        'protected_vs_control_improvement': None,
        'freeze_detected': (
            freeze_confirmed_at is not None if onset is None else
            freeze_confirmed_after_onset is not None),
        'freeze_suspected': freeze_suspected_after_onset is not None,
        'freeze_suspicion_latency_sec': (
            freeze_suspected_after_onset - onset
            if freeze_suspected_after_onset is not None and onset is not None
            else None),
        'freeze_detection_latency_sec': (
            freeze_confirmed_after_onset - onset
            if freeze_confirmed_after_onset is not None and onset is not None
            else None),
        'freeze_gate_latency_sec': (
            gated_at - onset
            if freeze_confirmed_at is not None and gated_at is not None and
            onset is not None else None),
        'freeze_confirmation_to_gate_sec': (
            gated_at - freeze_confirmed_after_onset
            if freeze_confirmed_after_onset is not None and
            gated_at is not None else None),
        'freeze_false_positive': (
            freeze_confirmed_at is not None and (
                onset is None or freeze_confirmed_at < onset)),
        'freeze_suspect_sample_count': freeze_states.get('SUSPECT', 0),
        'freeze_confirmed_sample_count': freeze_states.get('CONFIRMED', 0),
        'covariance_scale_during_fault': statistics(active_covariance_scales),
        'fault_clear_sec': recovery_onset,
        'gate_release_sec': gate_release_at,
        'probation_at_sec': probation_at,
        'probation_entry_sec': probation_at,
        'normal_restored_at_sec': normal_at,
        'probation_exit_sec': normal_at,
        'first_gnss_reentry_sec': first_reentry_at,
        'probation_duration_sec': (
            normal_at - probation_at
            if normal_at is not None and probation_at is not None else None),
        'recovery_latency_sec': (
            normal_at - recovery_onset
            if normal_at is not None and recovery_onset is not None else None),
        'position_error_before_fault': pre_fault_statistics,
        'position_error_during_fault': statistics(during_fault),
        'peak_fused_error_m': max(during_fault) if during_fault else None,
        'error_at_degraded_m': _value_at_or_after(
            health, degraded_at, 'fused_reference_error_m'),
        'error_at_untrusted_m': _value_at_or_after(
            health, untrusted_at, 'fused_reference_error_m'),
        'error_at_gating_m': _value_at_or_after(
            health, gated_at, 'fused_reference_error_m'),
        'error_at_fault_clear_m': error_at_fault_clear,
        'error_at_gate_release_m': _value_at_or_after(
            health, gate_release_at, 'fused_reference_error_m'),
        'error_at_probation_start_m': _value_at_or_after(
            health, probation_at, 'fused_reference_error_m'),
        'error_at_probation_end_m': _value_at_or_after(
            health, normal_at, 'fused_reference_error_m'),
        'error_at_first_gnss_reentry_m': _value_at_or_after(
            health, first_reentry_at, 'fused_reference_error_m'),
        **recovery_samples,
        **post_normal_samples,
        'post_recovery_peak_error_m': post_recovery_peak,
        'post_recovery_mean_error_m': statistics(
            post_recovery).get('mean'),
        'post_recovery_median_error_m': statistics(
            post_recovery).get('median'),
        'recovery_overshoot_m': recovery_overshoot,
        'time_to_error_below_2m': _time_to_threshold(
            errors, recovery_onset, 2.0),
        'time_to_error_below_1m': _time_to_threshold(
            errors, recovery_onset, 1.0),
        'time_to_error_below_0p5m': _time_to_threshold(
            errors, recovery_onset, 0.5),
        'time_to_first_below_2m': _time_to_threshold(
            errors, recovery_onset, 2.0),
        'time_to_first_below_1m': _time_to_threshold(
            errors, recovery_onset, 1.0),
        'time_to_first_below_0p5m': _time_to_threshold(
            errors, recovery_onset, 0.5),
        'sustained_threshold_reference': 'fault_clear_sec',
        'sustained_threshold_max_sample_gap_sec':
            SUSTAINED_SAMPLE_GAP_TOLERANCE_SEC,
        'time_to_sustained_below_2m_5s': sustained_below_2m_5s,
        'time_to_sustained_below_1m_5s': sustained_below_1m_5s,
        'time_to_sustained_below_1m_10s': sustained_below_1m_10s,
        'time_to_sustained_below_0p5m_5s': sustained_below_0p5m_5s,
        'time_to_sustained_healthy_band_5s': sustained_healthy_5s,
        'time_to_sustained_healthy_band_10s': sustained_healthy_10s,
        'post_normal_time_to_sustained_below_1m_5s':
            post_normal_sustained_below_1m_5s,
        'post_normal_time_to_sustained_below_1m_10s':
            post_normal_sustained_below_1m_10s,
        'post_normal_time_to_sustained_healthy_band_5s':
            post_normal_sustained_healthy_5s,
        'post_normal_time_to_sustained_healthy_band_10s':
            post_normal_sustained_healthy_10s,
        'healthy_baseline_band_upper_m': baseline_band_upper,
        'healthy_baseline_band_method': 'pre_fault_position_error_p95',
        'time_to_return_to_healthy_baseline_band': (
            _time_to_threshold(errors, recovery_onset, baseline_band_upper)
            if baseline_band_upper is not None else None),
        'post_normal_duration_sec': post_normal_duration,
        'post_normal_standard_horizon_sec':
            STANDARD_POST_NORMAL_HORIZON_SEC,
        'post_normal_complete_60s': post_normal_complete_60s,
        'post_normal_fraction_below_2m': post_normal_fraction_below_2m,
        'post_normal_fraction_below_1m': post_normal_fraction_below_1m,
        'post_normal_fraction_inside_healthy_band':
            post_normal_fraction_inside_healthy_band,
        'post_normal_fraction_above_healthy_band':
            post_normal_fraction_above_healthy_band,
        'post_normal_error_statistics': post_normal_statistics,
        'post_normal_mean_error': post_normal_statistics['mean'],
        'post_normal_p95_error': post_normal_statistics['p95'],
        'post_normal_error_windows': post_normal_windows,
        'post_normal_0_10s_mean_error':
            post_normal_windows['0_10s']['mean'],
        'post_normal_0_10s_median_error':
            post_normal_windows['0_10s']['median'],
        'post_normal_0_10s_p95_error':
            post_normal_windows['0_10s']['p95'],
        'post_normal_0_10s_maximum_error':
            post_normal_windows['0_10s']['maximum'],
        'post_normal_0_10s_minimum_error':
            post_normal_windows['0_10s']['minimum'],
        'post_normal_0_10s_stddev_error':
            post_normal_windows['0_10s']['stddev'],
        'post_normal_10_30s_mean_error':
            post_normal_windows['10_30s']['mean'],
        'post_normal_10_30s_median_error':
            post_normal_windows['10_30s']['median'],
        'post_normal_10_30s_p95_error':
            post_normal_windows['10_30s']['p95'],
        'post_normal_10_30s_maximum_error':
            post_normal_windows['10_30s']['maximum'],
        'post_normal_10_30s_minimum_error':
            post_normal_windows['10_30s']['minimum'],
        'post_normal_10_30s_stddev_error':
            post_normal_windows['10_30s']['stddev'],
        'post_normal_30_60s_mean_error':
            post_normal_windows['30_60s']['mean'],
        'post_normal_30_60s_median_error':
            post_normal_windows['30_60s']['median'],
        'post_normal_30_60s_p95_error':
            post_normal_windows['30_60s']['p95'],
        'post_normal_30_60s_maximum_error':
            post_normal_windows['30_60s']['maximum'],
        'post_normal_30_60s_minimum_error':
            post_normal_windows['30_60s']['minimum'],
        'post_normal_30_60s_stddev_error':
            post_normal_windows['30_60s']['stddev'],
        'error_slope_near_zero_tolerance_mps':
            ERROR_SLOPE_NEAR_ZERO_TOLERANCE_MPS,
        'post_normal_10_30s_error_slope': post_normal_slopes['10_30s'],
        'post_normal_10_30s_error_trend': _slope_classification(
            post_normal_slopes['10_30s']),
        'post_normal_30_60s_error_slope': post_normal_slopes['30_60s'],
        'post_normal_30_60s_error_trend': _slope_classification(
            post_normal_slopes['30_60s']),
        'post_normal_sensor_statistics': post_normal_sensor_statistics,
        'post_normal_error_correlations': _post_normal_correlations(
            health, normal_at, post_normal_end),
        'post_normal_state_counts': post_normal_state_counts,
        'persistent_recovery_bias': persistent_recovery_bias,
        'persistent_recovery_bias_definition': (
            '30-60 s post-NORMAL median exceeds this run pre-fault p95; '
            'requires complete 60 s horizon'),
        'persistent_recovery_drift': persistent_recovery_drift,
        'persistent_recovery_drift_definition': (
            '30-60 s post-NORMAL error slope exceeds reported near-zero '
            'tolerance; requires complete 60 s horizon'),
        'recovery_classification': recovery_classification,
        'max_error_after_gating_m': max(after_gate) if after_gate else None,
        'final_error_m': final_error,
        'reentry_evidence': {
            'message_stamp_sec': first_reentry_message_stamp,
            'message_age_sec': gate_release_message_age,
            'generated_before_gate_release':
                first_reentry_generated_before_release,
            'timing_reason': first_reentry_row.get('gnss_timing_reason'),
            'stale_rejection_count': sum(
                row.get('event') == 'gnss_rejected' and
                row.get('gnss_timing_reason') == 'STALE_MESSAGE'
                for row in rows),
            'pre_release_rejection_count': sum(
                row.get('event') == 'gnss_rejected' and
                row.get('gnss_timing_reason') ==
                'GENERATED_BEFORE_GATE_RELEASE'
                for row in rows),
            'navsat_transform_delay_sec': (
                reentry_health_at - first_reentry_at
                if reentry_health_at is not None and
                first_reentry_at is not None else None),
            'estimator_x_m': (
                _number(reentry_health_row.get('fused_x_m'))
                if reentry_health_row is not None else None),
            'estimator_y_m': (
                _number(reentry_health_row.get('fused_y_m'))
                if reentry_health_row is not None else None),
            'gnss_x_m': (
                _number(reentry_health_row.get('gnss_measurement_x_m'))
                if reentry_health_row is not None else None),
            'gnss_y_m': (
                _number(reentry_health_row.get('gnss_measurement_y_m'))
                if reentry_health_row is not None else None),
            'position_discrepancy_m': (
                _number(reentry_health_row.get('gnss_innovation_m'))
                if reentry_health_row is not None else None),
            'gnss_covariance_x_m2': (
                _number(reentry_health_row.get(
                    'gnss_measurement_covariance_x_m2'))
                if reentry_health_row is not None else None),
            'estimator_covariance_before_reentry': _last_value_before(
                health, first_reentry_at, 'fused_covariance_trace'),
            'estimator_covariance_after_reentry': (
                _number(reentry_health_row.get('fused_covariance_trace'))
                if reentry_health_row is not None else None),
            'pose_jump_at_reentry_m': (
                _number(reentry_health_row.get('estimator_pose_jump_m'))
                if reentry_health_row is not None else None),
        },
        'gating_covariance': {
            'at_fault_onset': _value_at_or_after(
                health, onset, 'fused_covariance_trace'),
            'at_gate_activation': _value_at_or_after(
                health, gated_at, 'fused_covariance_trace'),
            'maximum_during_gated': (
                max(covariance_during_gate)
                if covariance_during_gate else None),
            'immediately_before_reentry': _last_value_before(
                health, first_reentry_at, 'fused_covariance_trace'),
            'after_first_gnss_correction': (
                _number(reentry_health_row.get('fused_covariance_trace'))
                if reentry_health_row is not None else None),
            'after_return_to_normal': _value_at_or_after(
                health, normal_at, 'fused_covariance_trace'),
        },
        'estimator_health': {
            'samples': len(health),
            'status_counts': dict(statuses),
            'publication_rate_hz': statistics(rates),
            'freshness_sec': statistics(freshness),
            'covariance_trace': statistics(covariance),
            'gnss_innovation_m': statistics(innovations),
            'gnss_measurement_covariance_x_m2': statistics(gnss_covariance),
            'invalid_samples': statuses.get('INVALID', 0),
            'stale_samples': statuses.get('STALE', 0),
            'pose_jump_samples': statuses.get('POSE_JUMP', 0),
            'timestamp_regressions': statuses.get('TIME_REGRESSION', 0),
            'restart_count': max(
                [int(_number(row.get('estimator_restart_count')) or 0)
                 for row in health], default=0),
            'uptime_sec': max(
                [_number(row.get('estimator_uptime_sec')) or 0.0
                 for row in health], default=0.0),
        },
        'estimator_restart_count': max(
            [int(_number(row.get('estimator_restart_count')) or 0)
             for row in health], default=0),
        'false_gate_count': sum(
            onset is None or timestamp < onset for timestamp in gate_times),
        'wrong_sensor_gate_count': len(gate_times) if not gnss_fault else 0,
        'gate_count': len(gate_times),
        'active_attribution_counts': dict(attributions),
    }


def _comparison(experiments: list[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[str, dict[str, list[dict[str, Any]]]] = {}
    for experiment in experiments:
        grouped.setdefault(experiment['scenario'], {}).setdefault(
            experiment['fusion_mode'], []).append(experiment)
    result: dict[str, Any] = {}
    for scenario, modes in grouped.items():
        entry: dict[str, Any] = {
            'modes': sorted(modes),
            'trial_counts': {mode: len(trials)
                             for mode, trials in sorted(modes.items())},
            'peak_error_statistics_by_mode': {},
        }
        for mode, trials in modes.items():
            peaks = [_number(trial.get('peak_fused_error_m'))
                     for trial in trials]
            entry['peak_error_statistics_by_mode'][mode] = statistics(
                [peak for peak in peaks if peak is not None])
        protected = modes.get('protected', [])
        unprotected = modes.get('unprotected', [])
        if protected and unprotected:
            protected_stats = entry['peak_error_statistics_by_mode'][
                'protected']
            unprotected_stats = entry['peak_error_statistics_by_mode'][
                'unprotected']
            protected_mean = protected_stats['mean']
            unprotected_mean = unprotected_stats['mean']
            if protected_mean is not None and unprotected_mean is not None:
                reduction = unprotected_mean - protected_mean
                entry['peak_error_reduction_m'] = reduction
                entry['peak_error_reduction_fraction'] = (
                    reduction / unprotected_mean
                    if unprotected_mean != 0.0 else None)
            else:
                entry['peak_error_reduction_m'] = None
                entry['peak_error_reduction_fraction'] = None
            entry['repeat_peak_error_reductions_m'] = [
                unprotected_peak - protected_peak
                for unprotected_trial, protected_trial in zip(
                    unprotected, protected)
                for unprotected_peak in [
                    _number(unprotected_trial.get('peak_fused_error_m'))]
                for protected_peak in [
                    _number(protected_trial.get('peak_fused_error_m'))]
                if unprotected_peak is not None and protected_peak is not None
            ]
            by_seed: dict[str, dict[str, list[dict[str, Any]]]] = {}
            for mode, trials in (('protected', protected),
                                 ('unprotected', unprotected)):
                for trial in trials:
                    by_seed.setdefault(str(trial.get('seed')), {}).setdefault(
                        mode, []).append(trial)
            pairs = []
            for seed, seed_modes in sorted(by_seed.items()):
                for index, (protected_trial, control_trial) in enumerate(zip(
                        seed_modes.get('protected', []),
                        seed_modes.get('unprotected', [])), start=1):
                    protected_peak = _number(
                        protected_trial.get('peak_fused_error_m'))
                    control_peak = _number(
                        control_trial.get('peak_fused_error_m'))
                    improvement = (
                        control_peak - protected_peak
                        if control_peak is not None and
                        protected_peak is not None else None)
                    fraction = (
                        improvement / control_peak
                        if improvement is not None and
                        control_peak is not None and control_peak != 0.0
                        else None)
                    protected_trial['protected_vs_control_improvement'] = {
                        'peak_error_improvement_m': improvement,
                        'peak_error_improvement_fraction': fraction,
                    }
                    pairs.append({
                        'seed': seed,
                        'replicate_index': index,
                        'protected_file': protected_trial.get('file'),
                        'control_file': control_trial.get('file'),
                        'protected_peak_error_m': protected_peak,
                        'unprotected_peak_error_m': control_peak,
                        'protected_fault_mean_error_m': _number(
                            protected_trial.get(
                                'position_error_during_fault', {}).get('mean')),
                        'unprotected_fault_mean_error_m': _number(
                            control_trial.get(
                                'position_error_during_fault', {}).get('mean')),
                        'peak_error_improvement_m': improvement,
                        'peak_error_improvement_fraction': fraction,
                        'protected_error_at_detection_m': _number(
                            protected_trial.get('error_at_untrusted_m')),
                        'unprotected_error_at_detection_m': _number(
                            control_trial.get('error_at_untrusted_m')),
                        'protected_error_at_gate_m': _number(
                            protected_trial.get('error_at_gating_m')),
                        'unprotected_error_at_gate_m': _number(
                            control_trial.get('error_at_gating_m')),
                        'protected_post_gate_peak_error_m': _number(
                            protected_trial.get('max_error_after_gating_m')),
                        'unprotected_post_gate_peak_error_m': _number(
                            control_trial.get('max_error_after_gating_m')),
                        'protected_post_recovery_peak_m': _number(
                            protected_trial.get(
                                'post_recovery_peak_error_m')),
                        'unprotected_post_recovery_peak_m': _number(
                            control_trial.get(
                                'post_recovery_peak_error_m')),
                        'protected_final_error_m': _number(
                            protected_trial.get('final_error_m')),
                        'unprotected_final_error_m': _number(
                            control_trial.get('final_error_m')),
                        'protected_terminal_error_m': _number(
                            protected_trial.get('final_error_m')),
                        'unprotected_terminal_error_m': _number(
                            control_trial.get('final_error_m')),
                        'protected_time_below_1m': _number(
                            protected_trial.get('time_to_error_below_1m')),
                        'unprotected_time_below_1m': _number(
                            control_trial.get('time_to_error_below_1m')),
                        'protected_sustained_below_1m_5s': _number(
                            protected_trial.get(
                                'time_to_sustained_below_1m_5s')),
                        'unprotected_sustained_below_1m_5s': _number(
                            control_trial.get(
                                'time_to_sustained_below_1m_5s')),
                        'protected_post_normal_fraction_below_1m': _number(
                            protected_trial.get(
                                'post_normal_fraction_below_1m')),
                        'unprotected_post_normal_fraction_below_1m': _number(
                            control_trial.get(
                                'post_normal_fraction_below_1m')),
                        'protected_healthy_band_occupancy': _number(
                            protected_trial.get(
                                'post_normal_fraction_inside_healthy_band')),
                        'unprotected_healthy_band_occupancy': _number(
                            control_trial.get(
                                'post_normal_fraction_inside_healthy_band')),
                        'protected_late_error_slope_mps': _number(
                            protected_trial.get(
                                'post_normal_30_60s_error_slope')),
                        'unprotected_late_error_slope_mps': _number(
                            control_trial.get(
                                'post_normal_30_60s_error_slope')),
                        'protected_persistent_recovery_bias':
                            protected_trial.get('persistent_recovery_bias'),
                        'unprotected_persistent_recovery_bias':
                            control_trial.get('persistent_recovery_bias'),
                        'protected_persistent_recovery_drift':
                            protected_trial.get('persistent_recovery_drift'),
                        'unprotected_persistent_recovery_drift':
                            control_trial.get('persistent_recovery_drift'),
                        'protected_recovery_classification':
                            protected_trial.get('recovery_classification'),
                        'unprotected_recovery_classification':
                            control_trial.get('recovery_classification'),
                    })
            improvements: list[float] = [
                float(pair['peak_error_improvement_m'])
                for pair in pairs
                if pair['peak_error_improvement_m'] is not None]
            fractions: list[float] = [
                float(pair['peak_error_improvement_fraction'])
                for pair in pairs
                if pair['peak_error_improvement_fraction'] is not None]
            unique_seeds = {pair['seed'] for pair in pairs}
            entry['matched_pairs'] = pairs
            entry['unique_seed_count'] = len(unique_seeds)
            entry['protected_better_pair_count'] = sum(
                value > 0.0 for value in improvements)
            entry['protected_better_pair_fraction'] = (
                entry['protected_better_pair_count'] / len(improvements)
                if improvements else None)
            entry['peak_error_improvement_m'] = statistics(improvements)
            entry['peak_error_improvement_fraction'] = statistics(fractions)
            entry['protected_better_post_recovery_peak_count'] = sum(
                protected_value < control_value
                for pair in pairs
                for protected_value in [
                    _number(pair.get('protected_post_recovery_peak_m'))]
                for control_value in [
                    _number(pair.get('unprotected_post_recovery_peak_m'))]
                if protected_value is not None and control_value is not None)
            entry['protected_better_terminal_error_count'] = sum(
                protected_value < control_value
                for pair in pairs
                for protected_value in [
                    _number(pair.get('protected_terminal_error_m'))]
                for control_value in [
                    _number(pair.get('unprotected_terminal_error_m'))]
                if protected_value is not None and control_value is not None)
            entry['post_recovery_peak_statistics_by_mode'] = {
                mode: statistics([
                    value for trial in trials
                    for value in [_number(
                        trial.get('post_recovery_peak_error_m'))]
                    if value is not None])
                for mode, trials in modes.items()
            }
            entry['terminal_error_statistics_by_mode'] = {
                mode: statistics([
                    value for trial in trials
                    for value in [_number(trial.get('final_error_m'))]
                    if value is not None])
                for mode, trials in modes.items()
            }
            entry['time_below_1m_statistics_by_mode'] = {
                mode: statistics([
                    value for trial in trials
                    for value in [_number(
                        trial.get('time_to_error_below_1m'))]
                    if value is not None])
                for mode, trials in modes.items()
            }
            entry['sustained_recovery_statistics_by_mode'] = {
                mode: {
                    field: statistics([
                        value for trial in trials
                        for value in [_number(trial.get(field))]
                        if value is not None])
                    for field in (
                        'time_to_sustained_below_1m_5s',
                        'post_normal_fraction_below_1m',
                        'post_normal_fraction_inside_healthy_band',
                        'post_normal_10_30s_error_slope',
                        'post_normal_30_60s_error_slope',
                    )
                }
                for mode, trials in modes.items()
            }
            entry['worst_protected_run'] = max(
                protected,
                key=lambda trial: _number(
                    trial.get('peak_fused_error_m')) or -math.inf)
            entry['worst_protected_run'] = {
                'seed': entry['worst_protected_run'].get('seed'),
                'file': entry['worst_protected_run'].get('file'),
                'peak_error_m': _number(entry['worst_protected_run'].get(
                    'peak_fused_error_m')),
            }
        result[scenario] = entry
    covariance_levels = ('normal', 'moderate', 'strong')
    covariance_sweep: dict[str, Any] = {}
    for level in covariance_levels:
        trials = grouped.get(f'covariance_{level}', {}).get('protected', [])
        covariance_sweep[level] = {
            'trial_count': len(trials),
            'fault_mean_error_m': statistics([
                value for trial in trials
                for value in [_number(trial.get(
                    'position_error_during_fault', {}).get('mean'))]
                if value is not None]),
            'peak_fused_error_m': statistics([
                value for trial in trials
                for value in [_number(trial.get('peak_fused_error_m'))]
                if value is not None]),
            'measurement_covariance_x_m2': statistics([
                value for trial in trials
                for value in [_number(trial.get('estimator_health', {}).get(
                    'gnss_measurement_covariance_x_m2', {}).get('mean'))]
                if value is not None]),
        }
    mean_errors = [covariance_sweep[level]['fault_mean_error_m']['mean']
                   for level in covariance_levels]
    covariance_sweep['mean_fault_error_nonincreasing'] = (
        all(value is not None for value in mean_errors) and
        all(left >= right for left, right in zip(
            mean_errors, mean_errors[1:]))
    )
    result['covariance_sweep'] = covariance_sweep
    covariance_campaign: dict[str, Any] = {}
    factors = (1.0, 2.0, 5.0, 10.0, 25.0, 50.0, 100.0)
    covariance_trials: dict[float, list[dict[str, Any]]] = {}
    for factor in factors:
        trials = [
            experiment for experiment in experiments
            if experiment.get('scenario') == 'covariance_step5' and
            experiment.get('fusion_mode') == 'protected' and
            _number(experiment.get('covariance_multiplier')) == factor
        ]
        covariance_trials[factor] = trials
        covariance_campaign[str(int(factor))] = {
            'trial_count': len(trials),
            'seeds': [trial.get('seed') for trial in trials],
            'covariance_scale': statistics([
                value for trial in trials
                for value in [_number(trial.get(
                    'covariance_scale_during_fault', {}).get('mean'))]
                if value is not None]),
            'estimator_covariance_trace': statistics([
                value for trial in trials
                for value in [_number(trial.get(
                    'estimator_health', {}).get('covariance_trace', {}).get(
                        'mean'))]
                if value is not None]),
            'gnss_measurement_covariance_x_m2': statistics([
                value for trial in trials
                for value in [_number(trial.get('estimator_health', {}).get(
                    'gnss_measurement_covariance_x_m2', {}).get('mean'))]
                if value is not None]),
            'gnss_innovation_m': statistics([
                value for trial in trials
                for value in [_number(trial.get('estimator_health', {}).get(
                    'gnss_innovation_m', {}).get('mean'))]
                if value is not None]),
            'fault_window_mean_error_m': statistics([
                value for trial in trials
                for value in [_number(trial.get(
                    'position_error_during_fault', {}).get('mean'))]
                if value is not None]),
            'fault_window_median_error_m': statistics([
                value for trial in trials
                for value in [_number(trial.get(
                    'position_error_during_fault', {}).get('median'))]
                if value is not None]),
            'peak_fused_error_m': statistics([
                value for trial in trials
                for value in [_number(trial.get('peak_fused_error_m'))]
                if value is not None]),
            'error_at_degraded_m': statistics([
                value for trial in trials
                for value in [_number(trial.get('error_at_degraded_m'))]
                if value is not None]),
            'error_at_untrusted_m': statistics([
                value for trial in trials
                for value in [_number(trial.get('error_at_untrusted_m'))]
                if value is not None]),
            'error_at_gating_m': statistics([
                value for trial in trials
                for value in [_number(trial.get('error_at_gating_m'))]
                if value is not None]),
            'degraded_latency_sec': statistics([
                value for trial in trials
                for value in [_number(trial.get('degraded_latency_sec'))]
                if value is not None]),
            'untrusted_latency_sec': statistics([
                value for trial in trials
                for value in [_number(trial.get('untrusted_latency_sec'))]
                if value is not None]),
            'gate_latency_sec': statistics([
                value for trial in trials
                for value in [_number(trial.get('gate_latency_sec'))]
                if value is not None]),
            'final_recovery_error_m': statistics([
                value for trial in trials
                for value in [_number(trial.get('final_error_m'))]
                if value is not None]),
            'mahalanobis_or_rejection_count': None,
            'rejection_metric_available': False,
        }
    baseline_by_seed: dict[str, list[dict[str, Any]]] = {}
    for trial in covariance_trials[1.0]:
        baseline_by_seed.setdefault(str(trial.get('seed')), []).append(trial)
    for factor in factors:
        paired = []
        factor_by_seed: dict[str, list[dict[str, Any]]] = {}
        for trial in covariance_trials[factor]:
            factor_by_seed.setdefault(str(trial.get('seed')), []).append(trial)
        for seed, baseline_trials in sorted(baseline_by_seed.items()):
            for baseline_trial, treatment_trial in zip(
                    baseline_trials, factor_by_seed.get(seed, [])):
                baseline_mean = _number(baseline_trial.get(
                    'position_error_during_fault', {}).get('mean'))
                treatment_mean = _number(treatment_trial.get(
                    'position_error_during_fault', {}).get('mean'))
                baseline_peak = _number(
                    baseline_trial.get('peak_fused_error_m'))
                treatment_peak = _number(
                    treatment_trial.get('peak_fused_error_m'))
                mean_reduction = (
                    baseline_mean - treatment_mean
                    if baseline_mean is not None and
                    treatment_mean is not None else None)
                peak_reduction = (
                    baseline_peak - treatment_peak
                    if baseline_peak is not None and
                    treatment_peak is not None else None)
                paired.append({
                    'seed': seed,
                    'baseline_file': baseline_trial.get('file'),
                    'treatment_file': treatment_trial.get('file'),
                    'baseline_mean_error_m': baseline_mean,
                    'treatment_mean_error_m': treatment_mean,
                    'mean_error_reduction_m': mean_reduction,
                    'baseline_peak_error_m': baseline_peak,
                    'treatment_peak_error_m': treatment_peak,
                    'peak_error_reduction_m': peak_reduction,
                    'improved_mean_error': (
                        mean_reduction > 0.0
                        if mean_reduction is not None else None),
                    'improved_peak_error': (
                        peak_reduction > 0.0
                        if peak_reduction is not None else None),
                })
        mean_reductions: list[float] = [
            float(pair['mean_error_reduction_m']) for pair in paired
            if pair['mean_error_reduction_m'] is not None]
        peak_reductions: list[float] = [
            float(pair['peak_error_reduction_m']) for pair in paired
            if pair['peak_error_reduction_m'] is not None]
        covariance_campaign[str(int(factor))]['matched_to_1x'] = {
            'pair_count': len(paired),
            'pairs': paired,
            'mean_error_reduction_m': statistics(mean_reductions),
            'peak_error_reduction_m': statistics(peak_reductions),
            'mean_error_better_pair_count': sum(
                pair['improved_mean_error'] is True for pair in paired),
            'peak_error_better_pair_count': sum(
                pair['improved_peak_error'] is True for pair in paired),
        }
    means: list[Optional[float]] = []
    for factor in factors:
        factor_result = covariance_campaign[str(int(factor))]
        error_statistics = factor_result['fault_window_mean_error_m']
        means.append(_number(error_statistics.get('mean')))
    numeric_means = [value for value in means if value is not None]
    covariance_campaign['observed_means_nonincreasing'] = (
        len(numeric_means) == len(means) and
        all(left >= right for left, right in zip(
            numeric_means, numeric_means[1:]))
    )
    result['covariance_campaign'] = covariance_campaign
    confirmed_false_positives = [
        experiment for experiment in experiments
        if experiment.get('freeze_false_positive')]
    result['freeze_false_positive_count'] = len(confirmed_false_positives)
    result['freeze_false_positive_files'] = [
        experiment['file'] for experiment in confirmed_false_positives]
    return result


def _extended_recovery_summary(
    experiments: list[dict[str, Any]],
    comparison: dict[str, Any],
) -> dict[str, Any]:
    """Return the focused sustained-recovery evidence artifact."""
    fields = (
        'file', 'scenario', 'fusion_mode', 'seed', 'fault_clear_sec',
        'normal_restored_at_sec', 'recovery_latency_sec',
        'post_normal_duration_sec', 'post_normal_complete_60s',
        'position_error_before_fault', 'healthy_baseline_band_upper_m',
        'healthy_baseline_band_method', 'error_at_fault_clear_m',
        'error_at_first_gnss_reentry_m', 'post_recovery_peak_error_m',
        'time_to_first_below_2m', 'time_to_first_below_1m',
        'time_to_first_below_0p5m',
        'time_to_sustained_below_2m_5s',
        'time_to_sustained_below_1m_5s',
        'time_to_sustained_below_1m_10s',
        'time_to_sustained_healthy_band_5s',
        'time_to_sustained_healthy_band_10s',
        'post_normal_time_to_sustained_healthy_band_5s',
        'post_normal_time_to_sustained_healthy_band_10s',
        'post_normal_fraction_below_2m',
        'post_normal_fraction_below_1m',
        'post_normal_fraction_inside_healthy_band',
        'post_normal_fraction_above_healthy_band',
        'post_normal_error_statistics', 'post_normal_mean_error',
        'post_normal_p95_error', 'post_normal_error_windows',
        'post_normal_10_30s_error_slope',
        'post_normal_10_30s_error_trend',
        'post_normal_30_60s_error_slope',
        'post_normal_30_60s_error_trend',
        'post_normal_sensor_statistics', 'post_normal_error_correlations',
        'post_normal_state_counts', 'persistent_recovery_bias',
        'persistent_recovery_drift', 'recovery_classification',
        'reentry_evidence', 'estimator_restart_count',
        'wrong_sensor_gate_count', 'gate_count',
    )
    checkpoint_fields = tuple(
        f'post_normal_error_plus_{offset}s'
        for offset in (0, 1, 2, 5, 10, 15, 20, 30, 45, 60))
    focused = [
        {field: experiment.get(field)
         for field in fields + checkpoint_fields}
        for experiment in experiments
        if experiment.get('scenario') == 'gnss_step_5m' and
        str(experiment.get('seed')) in ('2506', '2507', '2508')
    ]
    step_comparison = comparison.get('gnss_step_5m', {})
    protected_2508 = next((
        experiment for experiment in experiments
        if str(experiment.get('seed')) == '2508' and
        experiment.get('fusion_mode') == 'protected'), None)
    control_2508 = next((
        experiment for experiment in experiments
        if str(experiment.get('seed')) == '2508' and
        experiment.get('fusion_mode') == 'unprotected'), None)
    protected_peak = _number(
        protected_2508.get('peak_fused_error_m')
        if protected_2508 is not None else None)
    control_peak = _number(
        control_2508.get('peak_fused_error_m')
        if control_2508 is not None else None)
    protected_post_recovery_peak = _number(
        protected_2508.get('post_recovery_peak_error_m')
        if protected_2508 is not None else None)
    control_post_recovery_peak = _number(
        control_2508.get('post_recovery_peak_error_m')
        if control_2508 is not None else None)
    reentry = (
        protected_2508.get('reentry_evidence', {})
        if protected_2508 is not None else {})
    baseline_band = _number(
        protected_2508.get('healthy_baseline_band_upper_m')
        if protected_2508 is not None else None)
    reentry_jump = _number(reentry.get('pose_jump_at_reentry_m'))
    recovery_states = (
        protected_2508.get('post_normal_state_counts', {}).get(
            'recovery_state', {})
        if protected_2508 is not None else {})
    decision_criteria = {
        'matched_fault_peak_reduced': (
            protected_peak is not None and control_peak is not None and
            protected_peak < control_peak),
        'matched_post_recovery_peak_reduced': (
            protected_post_recovery_peak is not None and
            control_post_recovery_peak is not None and
            protected_post_recovery_peak < control_post_recovery_peak),
        'clean_reentry_timing': (
            reentry.get('timing_reason') == 'ACCEPTED' and
            reentry.get('generated_before_gate_release') is False and
            reentry.get('stale_rejection_count') == 0 and
            reentry.get('pre_release_rejection_count') == 0),
        'no_major_reentry_jump': (
            reentry_jump is not None and baseline_band is not None and
            reentry_jump <= baseline_band),
        'complete_60s_post_normal_horizon': bool(
            protected_2508 is not None and
            protected_2508.get('post_normal_complete_60s')),
        'sustained_healthy_envelope': (
            protected_2508 is not None and
            protected_2508.get('recovery_classification') ==
            'SUSTAINED_CONVERGENCE'),
        'no_persistent_bias': (
            protected_2508 is not None and
            protected_2508.get('persistent_recovery_bias') is False),
        'no_persistent_drift': (
            protected_2508 is not None and
            protected_2508.get('persistent_recovery_drift') is False),
        'no_estimator_restart': (
            protected_2508 is not None and
            protected_2508.get('estimator_restart_count') == 0),
        'no_wrong_sensor_gate': (
            protected_2508 is not None and
            protected_2508.get('wrong_sensor_gate_count') == 0),
        'no_post_normal_recovery_flapping': (
            bool(recovery_states) and set(recovery_states) == {'NORMAL'}),
    }
    week5_decision = (
        'PASS' if all(decision_criteria.values()) else 'BLOCKED')
    return {
        'schema_version': 1,
        'evidence_only': True,
        'sustained_definition': {
            'required_durations_sec': [5.0, 10.0],
            'maximum_sample_gap_sec':
                SUSTAINED_SAMPLE_GAP_TOLERANCE_SEC,
            'reference': 'fault_clear_sec',
        },
        'healthy_band_definition': 'per-run pre-fault position-error p95',
        'trend_definition': {
            'method': 'least-squares error slope versus simulated time',
            'near_zero_tolerance_mps':
                ERROR_SLOPE_NEAR_ZERO_TOLERANCE_MPS,
        },
        'occupancy_definition': (
            'time-weighted, sample held until the next estimator-health '
            'sample, capped at 60 s after NORMAL'),
        'experiments': focused,
        'matched_seed_2508': [
            pair for pair in step_comparison.get('matched_pairs', [])
            if str(pair.get('seed')) == '2508'],
        'decision_criteria': decision_criteria,
        'week5_decision': week5_decision,
        'navigation_readiness': (
            'READY' if week5_decision == 'PASS' else 'NOT_READY'),
        'recovery_logic_changed': False,
    }


def main(argv: Optional[list[str]] = None) -> None:
    """Write Week 5 evidence, A/B comparison, and recovery summaries."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_files', nargs='+', type=Path)
    parser.add_argument('--output', type=Path,
                        default=Path('results/week5/summary.json'))
    parser.add_argument('--fusion-comparison', type=Path, default=Path(
        'results/week5/fusion_comparison.json'))
    parser.add_argument('--recovery-summary', type=Path, default=Path(
        'results/week5/recovery_summary.json'))
    parser.add_argument('--covariance-sweep', type=Path, default=Path(
        'results/week5/covariance_sweep.json'))
    parser.add_argument('--freeze-summary', type=Path, default=Path(
        'results/week5/freeze_summary.json'))
    parser.add_argument('--ab-reproducibility', type=Path, default=Path(
        'results/week5/ab_reproducibility.json'))
    parser.add_argument('--extended-recovery-summary', type=Path, default=Path(
        'results/week5/extended_recovery_summary.json'))
    args = parser.parse_args(argv)
    experiments = [analyze_fusion_csv(path) for path in args.csv_files]
    summary = {
        'schema_version': 1,
        'evidence_only': True,
        'experiments': experiments,
    }
    recovery = {
        'schema_version': 1,
        'experiments': [experiment for experiment in experiments
                        if experiment['fault_onset_sec'] is not None],
    }
    comparison = _comparison(experiments)
    extended_recovery = _extended_recovery_summary(
        experiments, comparison)
    freeze_summary = {
        'schema_version': 1,
        'evidence_only': True,
        'freeze_trials': [experiment for experiment in experiments
                          if experiment['scenario'] == 'gnss_freeze'],
        'healthy_controls': [experiment for experiment in experiments
                             if experiment['scenario'] in (
                                 'healthy_fusion', 'healthy_stationary',
                                 'healthy_calibration_sweep')],
        'confirmed_false_positive_count':
            comparison['freeze_false_positive_count'],
        'false_positive_files':
            comparison['freeze_false_positive_files'],
    }
    ab_reproducibility = {
        'schema_version': 1,
        'evidence_only': True,
        'gnss_step_5m': comparison.get('gnss_step_5m', {}),
        'gnss_slow_drift': comparison.get('gnss_slow_drift', {}),
    }
    covariance_sweep = {
        'schema_version': 1,
        'evidence_only': True,
        'scenario': 'covariance_step5',
        'fault_profile': 'GNSS east step +5 m, 30 s',
        'multipliers': [1, 2, 5, 10, 25, 50, 100],
        'results': comparison['covariance_campaign'],
    }
    for path, payload in (
            (args.output, summary),
            (args.fusion_comparison, comparison),
            (args.recovery_summary, recovery),
            (args.covariance_sweep, covariance_sweep),
            (args.freeze_summary, freeze_summary),
            (args.ab_reproducibility, ab_reproducibility),
            (args.extended_recovery_summary, extended_recovery)):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
