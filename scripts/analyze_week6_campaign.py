#!/usr/bin/env python3
"""Validate and aggregate one immutable Week 6 matched-seed campaign."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]
EXPECTED_CASES = (
    ('baseline', 'healthy'),
    ('unprotected', 'healthy'),
    ('protected', 'healthy'),
    ('unprotected', 'gnss_step_5m'),
    ('protected', 'gnss_step_5m'),
)
METRICS = (
    'final_goal_error_m',
    'ground_reference_final_goal_error_m',
    'mean_cross_track_error_m',
    'median_cross_track_error_m',
    'p95_cross_track_error_m',
    'max_cross_track_error_m',
    'path_length_ratio',
    'mission_completion_time_sec',
    'mean_localization_error_m',
    'max_localization_error_m',
    'fault_time_max_localization_error_m',
    'post_fault_max_localization_error_m',
    'time_to_resume_stable_navigation_sec',
)
COUNT_METRICS = (
    'nav2_abort_count',
    'planner_failure_count',
    'controller_failure_count',
    'nav_recovery_count',
    'controller_command_gap_count',
    'ares_gate_count',
    'ares_probation_count',
    'tf_error_count',
    'estimator_restart_count',
    'lifecycle_failure_count',
    'costmap_error_count',
)


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def complete_finite_metric_values(
        runs: list[dict[str, Any]], metric: str) -> list[float] | None:
    """Return a complete finite metric vector, or None for invalid evidence."""
    values = [finite_number(run.get(metric)) for run in runs]
    if any(value is None for value in values):
        return None
    return [value for value in values if value is not None]


def safe_int(value: Any, default: int = 0) -> int:
    """Return an integer for a numeric field without crashing on nulls."""
    if isinstance(value, bool) or value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return default


def completed_waypoint_distances(
        runs: list[dict[str, Any]]) -> list[float] | None:
    """Return complete finite waypoint evidence, or None if unavailable."""
    distances: list[float] = []
    for run in runs:
        waypoints = run.get('waypoints')
        if not isinstance(waypoints, list) or not waypoints:
            return None
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                return None
            distance = finite_number(waypoint.get('final_distance_m'))
            if distance is None:
                return None
            distances.append(distance)
    return distances


def distribution(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {'count': 0, 'mean': None, 'median': None,
                'min': None, 'max': None}
    return {
        'count': len(values),
        'mean': statistics.fmean(values),
        'median': statistics.median(values),
        'min': min(values),
        'max': max(values),
    }


def launch_log_evidence(result_path: Path) -> dict[str, int]:
    """Count explicit Nav2 failure evidence missing from diagnostics."""
    text = (result_path.parent / 'launch.log').read_text(
        encoding='utf-8', errors='replace')
    lines = text.splitlines()
    return {
        'planner_failure_count': sum(
            '[planner_server]: GridBasedplugin failed to plan from' in line
            for line in lines),
        'controller_failure_count': sum(
            '[controller_server]: RegulatedPurePursuitController detected '
            'collision ahead!' in line for line in lines),
        'shutdown_sigkill_count': sum(
            'exit code -9' in line for line in lines),
    }


def effective_count(run: dict[str, Any], metric: str) -> int:
    """Prefer explicit launch-log evidence over sparse diagnostics."""
    reported = safe_int(run.get(metric, 0))
    derived = safe_int(run.get('_launch_log_evidence', {}).get(metric, 0))
    return max(reported, derived)


def aggregate_group(runs: list[dict[str, Any]]) -> dict[str, Any]:
    completed = sum(bool(run.get('mission_completed')) for run in runs)
    reached = sum(safe_int(run.get('waypoints_reached', 0)) for run in runs)
    attempted = sum(safe_int(run.get('waypoints_total', 0)) for run in runs)
    controller_gaps = [
        value for run in runs
        if (value := finite_number(
            run.get('max_controller_command_gap_sec', 0.0))) is not None
    ]
    aggregate: dict[str, Any] = {
        'run_count': len(runs),
        'completion_count': completed,
        'completion_rate': completed / len(runs) if runs else 0.0,
        'waypoints_reached': reached,
        'waypoints_attempted': attempted,
        'waypoint_success_fraction': reached / attempted if attempted else 0.0,
        'max_controller_command_gap_sec': max(controller_gaps, default=0.0),
    }
    for metric in COUNT_METRICS:
        aggregate[metric] = sum(effective_count(run, metric) for run in runs)
    aggregate['mission_reported_planner_failure_count'] = sum(
        safe_int(run.get('planner_failure_count', 0)) for run in runs)
    aggregate['mission_reported_controller_failure_count'] = sum(
        safe_int(run.get('controller_failure_count', 0)) for run in runs)
    aggregate['shutdown_sigkill_after_result_count'] = sum(
        safe_int(run.get('_launch_log_evidence', {}).get(
            'shutdown_sigkill_count', 0)) for run in runs)
    for metric in METRICS:
        values = [value for run in runs
                  if (value := finite_number(run.get(metric))) is not None]
        aggregate[metric] = distribution(values)
    return aggregate


def check(identifier: str, passed: bool, evidence: Any) -> dict[str, Any]:
    return {'id': identifier, 'passed': bool(passed), 'evidence': evidence}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign-dir',
                        default='results/week6/final_campaign')
    args = parser.parse_args()
    campaign_dir = WORKSPACE / args.campaign_dir
    metadata_path = campaign_dir / 'campaign_metadata.json'
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    seeds = [int(seed) for seed in metadata['seeds']]

    indexed: dict[tuple[int, str, str], dict[str, Any]] = {}
    run_paths: dict[tuple[int, str, str], str] = {}
    implementation_sets: list[dict[str, str]] = []
    for result_path in sorted(campaign_dir.glob('*/*/result.json')):
        run = json.loads(result_path.read_text(encoding='utf-8'))
        key = (int(run['seed']), str(run['navigation_mode']),
               str(run['scenario']))
        if key in indexed:
            raise ValueError(f'duplicate result for {key}')
        indexed[key] = run
        run_paths[key] = str(result_path.relative_to(WORKSPACE))
        run['_launch_log_evidence'] = launch_log_evidence(result_path)
        run_metadata = json.loads((result_path.parent /
                                   'run_metadata.json').read_text(
                                       encoding='utf-8'))
        implementation_sets.append(run_metadata['implementation_sha256'])

    expected = {
        (seed, mode, scenario)
        for seed in seeds for mode, scenario in EXPECTED_CASES
    }
    missing = sorted(expected - set(indexed))
    unexpected = sorted(set(indexed) - expected)

    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for (_, mode, scenario), run in indexed.items():
        grouped[f'{scenario}/{mode}'].append(run)
    aggregates = {
        label: aggregate_group(sorted(
            runs, key=lambda run: int(run['seed'])))
        for label, runs in sorted(grouped.items())
    }

    matched: list[dict[str, Any]] = []
    for seed in seeds:
        cases: dict[str, Any] = {}
        for mode, scenario in EXPECTED_CASES:
            key = (seed, mode, scenario)
            run = indexed.get(key)
            if run is None:
                continue
            cases[f'{scenario}/{mode}'] = {
                'result_path': run_paths[key],
                'mission_completed': run.get('mission_completed'),
                'waypoints_reached': run.get('waypoints_reached'),
                'waypoints_total': run.get('waypoints_total'),
                **{metric: run.get(metric) for metric in
                   (*METRICS, *COUNT_METRICS,
                    'max_controller_command_gap_sec')},
                'launch_log_evidence': run['_launch_log_evidence'],
                'recovery_events': run.get('recovery_events', []),
            }
        protected = cases.get('gnss_step_5m/protected', {})
        unprotected = cases.get('gnss_step_5m/unprotected', {})
        matched.append({
            'seed': seed,
            'cases': cases,
            'fault_comparison': {
                'protected_completed': protected.get('mission_completed'),
                'unprotected_completed': unprotected.get('mission_completed'),
                'protected_minus_unprotected_max_localization_error_m': (
                    float(protected['max_localization_error_m']) -
                    float(unprotected['max_localization_error_m'])
                    if finite_number(protected.get(
                        'max_localization_error_m')) is not None and
                    finite_number(unprotected.get(
                        'max_localization_error_m')) is not None else None),
            },
        })

    def runs(label: str) -> list[dict[str, Any]]:
        return grouped.get(label, [])

    protected_fault = runs('gnss_step_5m/protected')
    unprotected_fault = runs('gnss_step_5m/unprotected')
    protected_healthy = runs('healthy/protected')
    all_healthy = [run for label, group in grouped.items()
                   if label.startswith('healthy/') for run in group]
    completed_runs = [run for run in indexed.values()
                      if run.get('mission_completed')]
    same_implementation = bool(implementation_sets) and all(
        item == implementation_sets[0] for item in implementation_sets)
    protected_fault_localization_peaks = complete_finite_metric_values(
        protected_fault, 'max_localization_error_m')
    unprotected_fault_localization_peaks = complete_finite_metric_values(
        unprotected_fault, 'max_localization_error_m')
    completed_distances = completed_waypoint_distances(completed_runs)

    acceptance_checks = [
        check('complete_matched_evidence', not missing and not unexpected,
              {'missing': missing, 'unexpected': unexpected,
               'expected_run_count': len(expected),
               'observed_run_count': len(indexed)}),
        check('single_frozen_implementation', same_implementation,
              {'unique_hash_sets': len({
                  json.dumps(item, sort_keys=True)
                  for item in implementation_sets})}),
        check('all_healthy_missions_complete',
              len(all_healthy) == len(seeds) * 3 and
              all(run.get('mission_completed') for run in all_healthy),
              {label: aggregates.get(label, {}).get('completion_rate')
               for label in ('healthy/baseline', 'healthy/unprotected',
                             'healthy/protected')}),
        check('protected_fault_missions_complete_consistently',
              len(protected_fault) == len(seeds) and
              all(run.get('mission_completed') for run in protected_fault),
              {'completed': sum(bool(run.get('mission_completed'))
                                for run in protected_fault),
               'total': len(protected_fault)}),
        check('protected_fault_completion_exceeds_unprotected',
              bool(protected_fault and unprotected_fault) and
              (sum(bool(run.get('mission_completed'))
                   for run in protected_fault) / len(protected_fault)) >
              (sum(bool(run.get('mission_completed'))
                   for run in unprotected_fault) / len(unprotected_fault)),
              {'protected_rate': aggregates.get(
                  'gnss_step_5m/protected', {}).get('completion_rate'),
               'unprotected_rate': aggregates.get(
                  'gnss_step_5m/unprotected', {}).get('completion_rate')}),
        check('protected_fault_localization_peak_improves',
              bool(protected_fault and unprotected_fault) and
              protected_fault_localization_peaks is not None and
              unprotected_fault_localization_peaks is not None and
              statistics.fmean(protected_fault_localization_peaks) <
              statistics.fmean(unprotected_fault_localization_peaks),
              {'protected_mean_of_run_maxima': aggregates.get(
                   'gnss_step_5m/protected', {}).get(
                       'max_localization_error_m', {}).get('mean'),
               'unprotected_mean_of_run_maxima': aggregates.get(
                   'gnss_step_5m/unprotected', {}).get(
                       'max_localization_error_m', {}).get('mean'),
               'protected_complete_metric_count': (
                   len(protected_fault_localization_peaks)
                   if protected_fault_localization_peaks is not None else 0),
               'unprotected_complete_metric_count': (
                   len(unprotected_fault_localization_peaks)
                   if unprotected_fault_localization_peaks is not None
                   else 0)}),
        check('protected_healthy_has_no_false_gating',
              len(protected_healthy) == len(seeds) and all(
                  safe_int(run.get('ares_gate_count', 0)) == 0 and
                  safe_int(run.get('ares_probation_count', 0)) == 0
                  for run in protected_healthy),
              {'gates': sum(safe_int(run.get('ares_gate_count', 0))
                            for run in protected_healthy),
               'probations': sum(safe_int(run.get('ares_probation_count', 0))
                                 for run in protected_healthy)}),
        check('protected_fault_executes_gate_and_probation',
              len(protected_fault) == len(seeds) and all(
                  safe_int(run.get('ares_gate_count', 0)) >= 1 and
                  safe_int(run.get('ares_probation_count', 0)) >= 1
                  for run in protected_fault),
              [{'seed': run.get('seed'),
                'gates': run.get('ares_gate_count'),
                'probations': run.get('ares_probation_count')}
               for run in protected_fault]),
        check('protected_runs_have_no_systematic_stack_failures',
              all(all(effective_count(run, metric) == 0 for metric in (
                  'tf_error_count', 'estimator_restart_count',
                  'lifecycle_failure_count', 'planner_failure_count',
                  'controller_failure_count', 'costmap_error_count'))
                  for run in (*protected_healthy, *protected_fault)),
              [{metric: sum(effective_count(run, metric) for run in
                            (*protected_healthy, *protected_fault))}
               for metric in ('tf_error_count', 'estimator_restart_count',
                              'lifecycle_failure_count',
                              'planner_failure_count',
                              'controller_failure_count',
                              'costmap_error_count')]),
        check('frozen_goal_threshold_satisfied',
              bool(completed_runs) and completed_distances is not None and
              all(distance <= 0.30 for distance in completed_distances),
              {'threshold_m': 0.30,
               'max_completed_waypoint_error_m': max(
                   completed_distances or [],
                   default=None)}),
    ]
    verdict = 'PASS' if all(
        item['passed'] for item in acceptance_checks) else 'BLOCKED'

    summary = {
        'schema_version': 1,
        'campaign_dir': str(campaign_dir.relative_to(WORKSPACE)),
        'seeds': seeds,
        'run_count': len(indexed),
        'aggregates': aggregates,
        'implementation_sha256': (
            implementation_sets[0] if same_implementation else None),
        'verdict': verdict,
    }
    acceptance = {
        'schema_version': 1,
        'criteria_frozen_before_campaign_analysis': True,
        'checks': acceptance_checks,
        'week6': verdict,
        'navigation_reliability': (
            'READY' if verdict == 'PASS' else 'NOT_READY'),
    }
    (campaign_dir / 'summary.json').write_text(
        json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    (campaign_dir / 'matched_seed_comparison.json').write_text(
        json.dumps({'schema_version': 1, 'matched_seeds': matched},
                   indent=2) + '\n', encoding='utf-8')
    (campaign_dir / 'acceptance.json').write_text(
        json.dumps(acceptance, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({
        'campaign_dir': str(campaign_dir),
        'seeds': seeds,
        'run_count': len(indexed),
        'verdict': verdict,
        'failed_checks': [item['id'] for item in acceptance_checks
                          if not item['passed']],
    }, indent=2))


if __name__ == '__main__':
    main()
