#!/usr/bin/env python3
"""Summarize Week 2 CSV evidence without external analysis dependencies."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
from typing import Any, Iterable, Optional


def _float(row: dict[str, str], key: str) -> Optional[float]:
    value = row.get(key, '')
    if value == '':
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _is_fault_enabled(row: dict[str, str]) -> bool:
    return (
        row.get('fault_enabled', '').lower() == 'true'
        and row.get('fault_mode', 'none') != 'none'
    )


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


def descriptive_statistics(values: Iterable[float]) -> dict[str, Any]:
    """Return the requested descriptive statistics for finite values."""
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {
            'samples': 0,
            'mean_m': None,
            'median_m': None,
            'stddev_m': None,
            'p95_m': None,
            'p90_m': None,
            'p99_m': None,
            'min_m': None,
            'max_m': None,
        }
    return {
        'samples': len(finite),
        'mean_m': statistics.fmean(finite),
        'median_m': statistics.median(finite),
        'stddev_m': statistics.pstdev(finite),
        'p95_m': _percentile(finite, 0.95),
        'p90_m': _percentile(finite, 0.90),
        'p99_m': _percentile(finite, 0.99),
        'min_m': min(finite),
        'max_m': max(finite),
    }


def analyze_csv(path: Path) -> dict[str, Any]:
    """Analyze one single-attack experiment file."""
    with path.open(encoding='utf-8', newline='') as csv_file:
        rows = list(csv.DictReader(csv_file))

    attack_rows = [row for row in rows if _is_fault_enabled(row)]
    attack_onset = _float(attack_rows[0], 'timestamp_sec') if attack_rows else None

    recovery_onset = None
    if attack_onset is not None:
        for row in rows:
            timestamp = _float(row, 'timestamp_sec')
            if timestamp is None or timestamp <= attack_onset:
                continue
            if row.get('event') == 'fault_status' and not _is_fault_enabled(row):
                recovery_onset = timestamp
                break

    healthy_residuals = []
    fault_residuals = []
    peak_rolling_mean = None
    for row in rows:
        if row.get('event') != 'diagnostic':
            continue
        timestamp = _float(row, 'timestamp_sec')
        residual = _float(row, 'instantaneous_residual_m')
        rolling_mean = _float(row, 'rolling_mean_residual_m')
        if residual is None or timestamp is None:
            continue
        if attack_onset is None or timestamp < attack_onset:
            healthy_residuals.append(residual)
        elif recovery_onset is None or timestamp < recovery_onset:
            fault_residuals.append(residual)
            if rolling_mean is not None:
                peak_rolling_mean = (
                    rolling_mean
                    if peak_rolling_mean is None
                    else max(peak_rolling_mean, rolling_mean)
                )

    fault_time = None
    recovery_time = None
    false_transitions = 0
    false_suspect_transitions = 0
    false_fault_transitions = 0
    for row in rows:
        if row.get('event') != 'trust_transition':
            continue
        timestamp = _float(row, 'timestamp_sec')
        if timestamp is None:
            continue
        state = row.get('trust_state')
        if attack_onset is None or timestamp < attack_onset:
            if state not in ('HEALTHY', 'UNKNOWN'):
                false_transitions += 1
            if state == 'SUSPECT':
                false_suspect_transitions += 1
            elif state == 'FAULT':
                false_fault_transitions += 1
        elif (
            fault_time is None
            and state == 'FAULT'
            and (recovery_onset is None or timestamp < recovery_onset)
        ):
            fault_time = timestamp
        elif (
            recovery_onset is not None
            and timestamp >= recovery_onset
            and state == 'HEALTHY'
        ):
            recovery_time = timestamp
            break

    modes = sorted(
        {
            row.get('fault_mode', 'unknown')
            for row in attack_rows
            if row.get('fault_mode')
        }
    )
    dropped_messages = sum(
        1
        for row in rows
        if row.get('event') == 'fault_status'
        and row.get('fault_message_dropped', '').lower() == 'true'
    )

    sensor_columns = {
        'gnss': ('gnss_diagnostic', 'instantaneous_residual_m'),
        'imu': ('imu_diagnostic', 'imu_instantaneous_residual_radps'),
        'localization': (
            'localization_diagnostic',
            'localization_instantaneous_residual_m',
        ),
    }
    sensor_statistics = {}
    for sensor, (event, column) in sensor_columns.items():
        values = [
            value
            for row in rows
            if row.get('event') == event
            for value in [_float(row, column)]
            if value is not None
        ]
        sensor_statistics[sensor] = descriptive_statistics(values)

    return {
        'file': str(path),
        'profile': rows[0].get('profile', 'unknown') if rows else 'unknown',
        'fault_modes': modes,
        'rows': len(rows),
        'attack_onset_sec': attack_onset,
        'recovery_onset_sec': recovery_onset,
        'healthy_residual': descriptive_statistics(healthy_residuals),
        'fault_residual': descriptive_statistics(fault_residuals),
        'peak_rolling_mean_m': peak_rolling_mean,
        'fault_detected': fault_time is not None,
        'detection_latency_sec': (
            fault_time - attack_onset
            if fault_time is not None and attack_onset is not None
            else None
        ),
        'recovered_to_healthy': recovery_time is not None,
        'recovery_latency_sec': (
            recovery_time - recovery_onset
            if recovery_time is not None and recovery_onset is not None
            else None
        ),
        'healthy_false_transitions': false_transitions,
        'healthy_false_suspect_transitions': false_suspect_transitions,
        'healthy_false_fault_transitions': false_fault_transitions,
        'dropped_status_messages': dropped_messages,
        'sensor_statistics': sensor_statistics,
    }


def main(argv: Optional[list[str]] = None) -> None:
    """Analyze one or more Week 2 CSV files and optionally save JSON."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_files', nargs='+', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)

    summary = {'experiments': [analyze_csv(path) for path in args.csv_files]}
    rendered = json.dumps(summary, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
