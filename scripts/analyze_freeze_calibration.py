#!/usr/bin/env python3
"""Calibrate motion-window GNSS freeze evidence from recorded healthy data."""

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Optional

from ares_reliability.gnss_freeze_detector import GnssFreezeDetector
from ares_reliability.week4_analysis import statistics


Sample = tuple[float, tuple[float, float], tuple[float, float], str, bool]


def load_samples(path: Path) -> list[Sample]:
    """Load aligned GNSS/odometry samples emitted by the GNSS monitor."""
    samples = []
    with path.open(encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            if row.get('event') != 'gnss_diagnostic':
                continue
            try:
                values = (
                    float(row['timestamp_sec']),
                    float(row['gnss_local_east_m']),
                    float(row['gnss_local_north_m']),
                    float(row['odometry_x_m']),
                    float(row['odometry_y_m']),
                )
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in values):
                continue
            samples.append((
                values[0], (values[1], values[2]), (values[3], values[4]),
                row.get('motion_regime', 'UNKNOWN'),
                row.get('fault_enabled', '').lower() == 'true',
            ))
    return samples


def replay(samples: list[Sample], window_sec: float,
           minimum_motion_m: float, maximum_gnss_displacement_m: float,
           confirmation_sec: float) -> dict[str, Any]:
    """Replay one detector configuration and summarize state episodes."""
    detector = GnssFreezeDetector(
        window_sec, minimum_motion_m, maximum_gnss_displacement_m,
        confirmation_sec)
    previous_state = 'HEALTHY'
    confirmed_episodes = 0
    suspect_episodes = 0
    first_fault_confirmation: Optional[float] = None
    fault_onset = next((sample[0] for sample in samples if sample[4]), None)
    displacement_by_regime: dict[str, dict[str, list[float]]] = {}
    for timestamp, gnss_xy, odometry_xy, regime, fault_active in samples:
        evaluation = detector.update(timestamp, gnss_xy, odometry_xy)
        if evaluation.state == 'SUSPECT' and previous_state == 'HEALTHY':
            suspect_episodes += 1
        if evaluation.state == 'CONFIRMED' and previous_state != 'CONFIRMED':
            confirmed_episodes += 1
        if (fault_active and evaluation.state == 'CONFIRMED' and
                first_fault_confirmation is None):
            first_fault_confirmation = timestamp
        previous_state = evaluation.state
        if (evaluation.odometry_displacement_m is not None and
                evaluation.gnss_displacement_m is not None):
            regime_values = displacement_by_regime.setdefault(
                regime, {'odometry_m': [], 'gnss_m': []})
            regime_values['odometry_m'].append(
                evaluation.odometry_displacement_m)
            regime_values['gnss_m'].append(evaluation.gnss_displacement_m)
    return {
        'sample_count': len(samples),
        'suspect_episode_count': suspect_episodes,
        'confirmed_episode_count': confirmed_episodes,
        'fault_onset_sec': fault_onset,
        'first_fault_confirmation_sec': first_fault_confirmation,
        'fault_confirmation_latency_sec': (
            first_fault_confirmation - fault_onset
            if first_fault_confirmation is not None and
            fault_onset is not None else None),
        'displacement_by_regime': {
            regime: {
                'odometry_m': statistics(values['odometry_m']),
                'gnss_m': statistics(values['gnss_m']),
            }
            for regime, values in sorted(displacement_by_regime.items())
        },
    }


def main() -> None:
    """Evaluate a threshold grid and write the calibration evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('healthy_csvs', nargs='+', type=Path)
    parser.add_argument('--freeze-csv', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--selected-window-sec', type=float, default=2.0)
    parser.add_argument('--selected-minimum-motion-m', type=float, default=0.2)
    parser.add_argument(
        '--selected-maximum-gnss-displacement-m', type=float, default=0.15)
    parser.add_argument('--selected-confirmation-sec', type=float, default=0.5)
    args = parser.parse_args()

    healthy_samples = {
        str(path): load_samples(path) for path in args.healthy_csvs}
    freeze_samples = load_samples(args.freeze_csv)
    candidates = []
    for window_sec in (1.5, 2.0, 2.5, 3.0, 3.5):
        for minimum_motion_m in (0.10, 0.15, 0.20):
            healthy_results = {
                path: replay(samples, window_sec, minimum_motion_m, 0.15, 0.5)
                for path, samples in healthy_samples.items()
            }
            freeze_result = replay(
                freeze_samples, window_sec, minimum_motion_m, 0.15, 0.5)
            candidates.append({
                'window_sec': window_sec,
                'minimum_motion_m': minimum_motion_m,
                'maximum_gnss_displacement_m': 0.15,
                'confirmation_sec': 0.5,
                'healthy_suspect_episode_count': sum(
                    result['suspect_episode_count']
                    for result in healthy_results.values()),
                'healthy_confirmed_episode_count': sum(
                    result['confirmed_episode_count']
                    for result in healthy_results.values()),
                'freeze_confirmation_latency_sec':
                    freeze_result['fault_confirmation_latency_sec'],
            })

    selected_parameters = {
        'window_sec': args.selected_window_sec,
        'minimum_motion_m': args.selected_minimum_motion_m,
        'maximum_gnss_displacement_m':
            args.selected_maximum_gnss_displacement_m,
        'confirmation_sec': args.selected_confirmation_sec,
    }
    selected_healthy = {
        path: replay(samples, **selected_parameters)
        for path, samples in healthy_samples.items()
    }
    selected_freeze = replay(freeze_samples, **selected_parameters)
    output = {
        'schema_version': 1,
        'evidence_only': True,
        'healthy_sources': list(healthy_samples),
        'freeze_source': str(args.freeze_csv),
        'selected_parameters': selected_parameters,
        'selected_healthy_results': selected_healthy,
        'selected_freeze_result': selected_freeze,
        'candidate_grid': candidates,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
