"""Regression tests for robust Week 6 campaign analyzer execution."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from typing import Callable


WORKSPACE = Path(__file__).resolve().parents[3]
ANALYZER_PATH = WORKSPACE / 'scripts' / 'analyze_week6_campaign.py'
SPEC = importlib.util.spec_from_file_location(
    'analyze_week6_campaign', ANALYZER_PATH)
assert SPEC is not None and SPEC.loader is not None
ANALYZER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYZER)


def _result(mode: str, scenario: str) -> dict[str, object]:
    protected_fault = mode == 'protected' and scenario == 'gnss_step_5m'
    unprotected_fault = mode == 'unprotected' and scenario == 'gnss_step_5m'
    completed = not unprotected_fault
    result: dict[str, object] = {
        'schema_version': 1,
        'navigation_mode': mode,
        'scenario': scenario,
        'seed': 2530,
        'mission_completed': completed,
        'waypoints_reached': 5 if completed else 1,
        'waypoints_total': 5,
        'waypoints': [{'final_distance_m': 0.20}],
        'max_localization_error_m': (
            0.50 if protected_fault else 5.0 if unprotected_fault else 0.25),
        'ares_gate_count': 1 if protected_fault else 0,
        'ares_probation_count': 1 if protected_fault else 0,
        'max_controller_command_gap_sec': 0.05,
        'runtime_quality': {'controller_rate_miss_count': 0},
    }
    for metric in ANALYZER.METRICS:
        result.setdefault(metric, 1.0)
    for metric in ANALYZER.COUNT_METRICS:
        result.setdefault(metric, 0)
    return result


def _accepted_results() -> dict[tuple[str, str], dict[str, object]]:
    return {
        (mode, scenario): _result(mode, scenario)
        for mode, scenario in ANALYZER.EXPECTED_CASES
    }


def _run_analyzer(
        tmp_path: Path, monkeypatch,
        mutate: Callable[[dict[tuple[str, str], dict[str, object]]], None]
        | None = None) -> dict[str, object]:
    campaign = tmp_path / 'campaign'
    campaign.mkdir()
    (campaign / 'campaign_metadata.json').write_text(
        json.dumps({'schema_version': 1, 'seeds': [2530]}) + '\n',
        encoding='utf-8')
    results = _accepted_results()
    if mutate is not None:
        mutate(results)
    for (mode, scenario), result in results.items():
        run_dir = campaign / scenario / f'{scenario}_{mode}_s2530'
        run_dir.mkdir(parents=True)
        (run_dir / 'result.json').write_text(
            json.dumps(result) + '\n', encoding='utf-8')
        (run_dir / 'launch.log').write_text('', encoding='utf-8')
        (run_dir / 'run_metadata.json').write_text(
            json.dumps({'implementation_sha256': {'frozen': 'same'}}) + '\n',
            encoding='utf-8')
    monkeypatch.setattr(ANALYZER, 'WORKSPACE', tmp_path)
    monkeypatch.setattr(
        sys, 'argv', ['analyze_week6_campaign.py',
                      '--campaign-dir', 'campaign'])
    ANALYZER.main()
    return json.loads(
        (campaign / 'acceptance.json').read_text(encoding='utf-8'))


def test_fully_successful_result_schema_completes_with_pass(
        tmp_path, monkeypatch) -> None:
    acceptance = _run_analyzer(tmp_path, monkeypatch)
    assert acceptance['week6'] == 'PASS'


def test_scientific_navigation_failure_completes_with_blocked(
        tmp_path, monkeypatch) -> None:
    def mutate(results):
        results[('unprotected', 'healthy')]['mission_completed'] = False

    acceptance = _run_analyzer(tmp_path, monkeypatch, mutate)
    assert acceptance['week6'] == 'BLOCKED'


def test_startup_readiness_failure_schema_completes_with_blocked(
        tmp_path, monkeypatch) -> None:
    def mutate(results):
        results[('baseline', 'healthy')] = {
            'schema_version': 1,
            'navigation_mode': 'baseline',
            'scenario': 'healthy',
            'seed': 2530,
            'mission_completed': False,
            'mission_failure_reasons': ['runner_error: readiness failed'],
            'startup_diagnostics': {'readiness_barrier': {'ready': False}},
        }

    acceptance = _run_analyzer(tmp_path, monkeypatch, mutate)
    assert acceptance['week6'] == 'BLOCKED'


def test_lifecycle_failure_schema_completes_with_blocked(
        tmp_path, monkeypatch) -> None:
    def mutate(results):
        result = results[('protected', 'gnss_step_5m')]
        result['lifecycle_failure_count'] = 1
        result['mission_failure_reasons'] = ['lifecycle activation failed']

    acceptance = _run_analyzer(tmp_path, monkeypatch, mutate)
    assert acceptance['week6'] == 'BLOCKED'


def test_null_waypoint_metrics_complete_with_blocked(
        tmp_path, monkeypatch) -> None:
    def mutate(results):
        result = results[('protected', 'gnss_step_5m')]
        result['waypoints'] = [{'final_distance_m': None}]

    acceptance = _run_analyzer(tmp_path, monkeypatch, mutate)
    assert acceptance['week6'] == 'BLOCKED'
    threshold = next(
        check for check in acceptance['checks']
        if check['id'] == 'frozen_goal_threshold_satisfied')
    assert threshold['passed'] is False


def test_missing_optional_runtime_metrics_do_not_crash_or_change_verdict(
        tmp_path, monkeypatch) -> None:
    def mutate(results):
        for result in results.values():
            for key in ('mission_completion_time_sec',
                        'max_controller_command_gap_sec',
                        'runtime_quality'):
                result.pop(key, None)

    acceptance = _run_analyzer(tmp_path, monkeypatch, mutate)
    assert acceptance['week6'] == 'PASS'


def test_complete_finite_metric_values_rejects_nonfinite_values() -> None:
    for invalid in (None, float('nan'), float('inf')):
        runs = [{'max_localization_error_m': invalid}]
        assert ANALYZER.complete_finite_metric_values(
            runs, 'max_localization_error_m') is None
