#!/usr/bin/env python3
"""Run gated Week 6 runtime handoff validations without changing criteria."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]
RUNNER = WORKSPACE / 'scripts' / 'run_week6_navigation.py'
ROOT = WORKSPACE / 'results/week6/post_v7_runtime_diagnostics'
STRESS_ROOT = ROOT / 'teardown_stress_v2_5x'
MIXED_ROOT = ROOT / 'mixed_mode_handoff_v2_5x'
MINI_ROOT = ROOT / 'mini_campaign_seed2530_v2'
STRESS_MODES = ('protected',) * 5
MIXED_MODES = (
    'baseline', 'unprotected', 'protected', 'unprotected', 'protected')
MINI_CASES = (
    ('baseline', 'healthy'),
    ('unprotected', 'healthy'),
    ('protected', 'healthy'),
    ('unprotected', 'gnss_step_5m'),
    ('protected', 'gnss_step_5m'),
)


def _load_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    temporary.replace(path)


def _launch_readiness_passed(run_directory: Path) -> bool:
    from run_week6_navigation import inspect_startup_log

    path = run_directory / 'launch.log'
    if not path.is_file():
        return False
    return bool(inspect_startup_log(
        path.read_text(encoding='utf-8', errors='replace'))['passed'])


def _invoke(
        run_directory: Path, *, mode: str, scenario: str,
        startup_only: bool) -> dict[str, Any]:
    if run_directory.exists() and any(run_directory.iterdir()):
        raise FileExistsError(
            f'{run_directory} already contains evidence; refusing rerun')
    run_directory.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    command = [
        sys.executable, str(RUNNER),
        '--navigation-mode', mode,
        '--scenario', scenario,
        '--seed', '2530',
        '--run-directory', str(run_directory.relative_to(WORKSPACE)),
    ]
    if startup_only:
        command.append('--startup-only')
    completed = subprocess.run(
        command, cwd=WORKSPACE, env=environment,
        capture_output=True, text=True, check=False)
    (run_directory / 'orchestrator_stdout.log').write_text(
        completed.stdout + completed.stderr, encoding='utf-8')
    isolation = _load_json(run_directory / 'runtime_isolation.json')
    teardown = _load_json(run_directory / 'post_run_teardown.json')
    readiness = _load_json(run_directory / 'endpoint_readiness.json')
    result = _load_json(run_directory / 'result.json')
    if readiness is not None:
        startup = readiness
    else:
        from run_week6_navigation import inspect_startup_log

        launch_log = run_directory / 'launch.log'
        startup = inspect_startup_log(
            launch_log.read_text(encoding='utf-8', errors='replace')
            if launch_log.is_file() else '')
    barrier = startup.get('readiness_barrier', {})
    lifecycle = startup.get('lifecycle_active_confirmations', {})
    before = (isolation or {}).get('checks_before_cleanup', {})
    before_topics = before.get('topic_publishers', {})
    teardown = teardown or {}
    readiness_passed = (
        bool(readiness and readiness.get('passed'))
        if startup_only else _launch_readiness_passed(run_directory))
    case = {
        'run_directory': str(run_directory.relative_to(WORKSPACE)),
        'navigation_mode': mode,
        'scenario': scenario,
        'startup_only': startup_only,
        'runner_exit_code': completed.returncode,
        'world_exclusivity_ready': bool(
            isolation and isolation.get('world_exclusivity_ready')),
        'endpoint_readiness_passed': readiness_passed,
        'mission_completed': (
            result.get('mission_completed') if result else None),
        'canonical_result_exists': (run_directory / 'result.json').is_file(),
        'post_run_teardown_ready': bool(
            teardown and teardown.get('teardown_ready')),
        'teardown_report': teardown,
        'isolation_report': isolation,
        'pre_run_convergence_duration_sec': (
            (isolation or {}).get('pre_run_convergence_duration_sec')),
        'stale_resources_initially_detected': {
            'owned_processes': (isolation or {}).get(
                'detected_stale_processes', []),
            'critical_ros_nodes': (isolation or {}).get(
                'detected_stale_ros_nodes', []),
            'world_present': (
                before.get('gazebo_ares_world_services') or []),
            'clock_publishers': before_topics.get(
                '/clock', {}).get('publisher_count'),
            'raw_clock_publishers': before_topics.get(
                '/ares/week6/raw_clock', {}).get('publisher_count'),
        },
        'pre_run_cleanup_actions': (isolation or {}).get(
            'cleanup_actions', []),
        'pre_run_first_clean_snapshot_utc': (isolation or {}).get(
            'first_clean_snapshot_utc'),
        'pre_run_second_clean_snapshot_utc': (isolation or {}).get(
            'second_clean_snapshot_utc'),
        'post_run_cleanup_actions': teardown.get('cleanup_actions', []),
        'first_clean_snapshot_utc': teardown.get(
            'first_clean_snapshot_utc'),
        'second_clean_snapshot_utc': teardown.get(
            'second_clean_snapshot_utc'),
        'total_teardown_duration_sec': teardown.get(
            'total_teardown_duration_sec'),
        'owned_survivors_after_sigint': teardown.get(
            'survivors_after_sigint', []),
        'owned_survivors_after_sigterm': teardown.get(
            'survivors_after_sigterm', []),
        'owned_survivors_after_sigkill': teardown.get(
            'survivors_after_sigkill', []),
        'clock_publisher_samples': teardown.get(
            'clock_publisher_counts_over_time', []),
        'raw_clock_publisher_samples': teardown.get(
            'raw_clock_publisher_counts_over_time', []),
        'remaining_critical_nodes': teardown.get(
            'remaining_critical_nodes', []),
        'world_present_after_teardown': teardown.get('world_present'),
        'behavior_action_discovery_failures': [
            action for action in ('/spin', '/backup', '/drive_on_heading', '/wait')
            if not barrier.get('actions', {}).get(action, False)
        ],
        'lifecycle_startup_failures': [
            node for node, active in lifecycle.items() if not active
        ],
    }
    return case


def _publisher_series_converged(case: dict[str, Any], field: str) -> bool:
    samples = case.get(field, [])
    return (
        bool(case.get('post_run_teardown_ready')) and
        case.get('first_clean_snapshot_utc') is not None and
        case.get('second_clean_snapshot_utc') is not None and
        len(samples) >= 2 and
        all(sample.get('publisher_count') == 0 for sample in samples[-2:]))


def _runtime_clean(case: dict[str, Any]) -> bool:
    return (
        case.get('post_run_teardown_ready', False) and
        case.get('world_present_after_teardown') is False and
        not case.get('remaining_critical_nodes') and
        _publisher_series_converged(case, 'clock_publisher_samples') and
        _publisher_series_converged(case, 'raw_clock_publisher_samples'))


def _case_passed(case: dict[str, Any], *, startup_only: bool) -> bool:
    return (
        case.get('world_exclusivity_ready', False) and
        case.get('endpoint_readiness_passed', False) and
        case.get('post_run_teardown_ready', False) and
        (case.get('runtime_clean', False) if startup_only else True))


def run_teardown_stress() -> dict[str, Any]:
    report_path = STRESS_ROOT / 'stress_report.json'
    if STRESS_ROOT.exists():
        previous = _load_json(report_path)
        if previous is None:
            raise FileExistsError(
                f'{STRESS_ROOT} contains evidence without a resumable report')
        cases = list(previous.get('cases', []))
        if any(not _case_passed(case, startup_only=True) for case in cases):
            for case in cases:
                case['runtime_clean_before_classifier_fix'] = case.get(
                    'runtime_clean')
                case['runtime_clean'] = _runtime_clean(case)
        if any(not _case_passed(case, startup_only=True) for case in cases):
            raise RuntimeError(
                'an existing stress cycle failed; refusing to rerun it')
        if any(case.get('cycle') != index
               for index, case in enumerate(cases, start=1)):
            raise RuntimeError('existing stress cycle order is invalid')
    else:
        cases = []

    for index in range(len(cases) + 1, len(STRESS_MODES) + 1):
        mode = STRESS_MODES[index - 1]
        case = _invoke(
            STRESS_ROOT / f'cycle_{index:02d}' / mode,
            mode=mode, scenario='healthy', startup_only=True)
        case['cycle'] = index
        case['runtime_clean'] = _runtime_clean(case)
        cases.append(case)
        interim = {
            'schema_version': 1,
            'phase': 'teardown_stress_v2_5x',
            'classification': 'two consecutive final clean snapshots',
            'cases': cases,
            'pass_count': sum(
                _case_passed(item, startup_only=True)
                for item in cases),
        }
        _write_json(report_path, interim)
        if not _case_passed(case, startup_only=True):
            break
    passed = sum(
        _case_passed(item, startup_only=True)
        for item in cases)
    report = {
        'schema_version': 1,
        'phase': 'teardown_stress_v2_5x',
        'expected_cycles': 5,
        'pass_count': passed,
        'pass': passed == 5 and len(cases) == 5,
        'stale_world_failures': sum(
            item.get('world_present_after_teardown') is not False
            for item in cases),
        'stale_clock_failures': sum(
            not _publisher_series_converged(
                item, 'clock_publisher_samples')
            for item in cases),
        'stale_raw_clock_failures': sum(
            not _publisher_series_converged(
                item, 'raw_clock_publisher_samples')
            for item in cases),
        'stale_node_failures': sum(
            bool(item.get('remaining_critical_nodes'))
            for item in cases),
        'endpoint_readiness_failures': sum(
            not item['endpoint_readiness_passed'] for item in cases),
        'behavior_action_discovery_failures': sum(
            len(item['behavior_action_discovery_failures']) for item in cases),
        'lifecycle_startup_failures': sum(
            bool(item['lifecycle_startup_failures']) or
            not item['endpoint_readiness_passed'] for item in cases),
        'transient_clock_endpoint_observations': sum(
            any(sample.get('publisher_count') not in (0, None)
                for sample in item.get('clock_publisher_samples', []))
            for item in cases),
        'transient_raw_clock_endpoint_observations': sum(
            any(sample.get('publisher_count') not in (0, None)
                for sample in item.get('raw_clock_publisher_samples', []))
            for item in cases),
        'cases': cases,
    }
    _write_json(report_path, report)
    return report


def run_mixed_handoff() -> dict[str, Any]:
    stress = _load_json(STRESS_ROOT / 'stress_report.json')
    if not stress or not stress.get('pass'):
        raise RuntimeError('teardown stress is not 5/5; mixed handoff blocked')
    if MIXED_ROOT.exists():
        raise FileExistsError(
            f'{MIXED_ROOT} exists; refusing to overwrite handoff evidence')
    cases: list[dict[str, Any]] = []
    for index, mode in enumerate(MIXED_MODES, start=1):
        case = _invoke(
            MIXED_ROOT / f'cycle_{index:02d}' / mode,
            mode=mode, scenario='healthy', startup_only=True)
        previous_teardown = (
            cases[-1]['post_run_teardown_ready'] if cases else True)
        case['cycle'] = index
        case['previous_teardown_ready'] = previous_teardown
        case['next_pre_run_isolation_ready'] = case[
            'world_exclusivity_ready']
        case['transition_passed'] = (
            previous_teardown and
            case['next_pre_run_isolation_ready'] and
            case['endpoint_readiness_passed'] and
            case['post_run_teardown_ready'] and
            _runtime_clean(case))
        cases.append(case)
        _write_json(MIXED_ROOT / 'handoff_report.json', {
            'schema_version': 1,
            'phase': 'mixed_mode_handoff_5x',
            'cases': cases,
            'pass_count': sum(item['transition_passed'] for item in cases),
        })
        if not case['transition_passed']:
            break
    passed = sum(item['transition_passed'] for item in cases)
    report = {
        'schema_version': 1,
        'phase': 'mixed_mode_handoff_5x',
        'expected_cycles': 5,
        'pass_count': passed,
        'pass': passed == 5 and len(cases) == 5,
        'world_exclusivity_failures': sum(
            not item['world_exclusivity_ready'] for item in cases),
        'stale_clock_failures': sum(
            not _publisher_series_converged(
                item, 'clock_publisher_samples') for item in cases),
        'stale_raw_clock_failures': sum(
            not _publisher_series_converged(
                item, 'raw_clock_publisher_samples') for item in cases),
        'endpoint_failures': sum(
            not item['endpoint_readiness_passed'] for item in cases),
        'behavior_action_discovery_failures': sum(
            len(item['behavior_action_discovery_failures']) for item in cases),
        'lifecycle_startup_failures': sum(
            bool(item['lifecycle_startup_failures']) or
            not item['endpoint_readiness_passed'] for item in cases),
        'cases': cases,
    }
    _write_json(MIXED_ROOT / 'handoff_report.json', report)
    return report


def run_mini_campaign() -> dict[str, Any]:
    mixed = _load_json(MIXED_ROOT / 'handoff_report.json')
    if not mixed or not mixed.get('pass'):
        raise RuntimeError('mixed-mode handoff is not 5/5; mini campaign blocked')
    if MINI_ROOT.exists():
        raise FileExistsError(
            f'{MINI_ROOT} exists; refusing to overwrite mini-campaign evidence')
    cases: list[dict[str, Any]] = []
    for index, (mode, scenario) in enumerate(MINI_CASES, start=1):
        case = _invoke(
            MINI_ROOT / f'case_{index:02d}' /
            f'{scenario}_{mode}_s2530',
            mode=mode, scenario=scenario, startup_only=False)
        previous_teardown = (
            cases[-1]['post_run_teardown_ready'] if cases else True)
        case['case'] = index
        case['previous_teardown_ready'] = previous_teardown
        case['transition_passed'] = (
            previous_teardown and
            case['world_exclusivity_ready'] and
            case['endpoint_readiness_passed'] and
            case['canonical_result_exists'] and
            case['post_run_teardown_ready'] and
            _runtime_clean(case))
        cases.append(case)
        _write_json(MINI_ROOT / 'mini_campaign_report.json', {
            'schema_version': 1,
            'phase': 'mini_campaign_seed2530',
            'cases': cases,
            'launches_passed': sum(
                item['transition_passed'] for item in cases),
            'inter_run_teardowns_passed': sum(
                item['post_run_teardown_ready']
                for item in cases[:-1]),
        })
        if not case['transition_passed']:
            break
    launches_passed = sum(item['transition_passed'] for item in cases)
    teardown_count = sum(
        bool(item['post_run_teardown_ready']) for item in cases[:-1])
    report = {
        'schema_version': 1,
        'phase': 'mini_campaign_seed2530',
        'expected_runs': 5,
        'launches_passed': launches_passed,
        'inter_run_teardowns_passed': teardown_count,
        'pass': (
            launches_passed == 5 and len(cases) == 5 and
            teardown_count == 4),
        'world_exclusivity_failures': sum(
            not item['world_exclusivity_ready'] for item in cases),
        'stale_clock_failures': sum(
            not _publisher_series_converged(
                item, 'clock_publisher_samples') for item in cases),
        'stale_raw_clock_failures': sum(
            not _publisher_series_converged(
                item, 'raw_clock_publisher_samples') for item in cases),
        'endpoint_readiness_failures': sum(
            not item['endpoint_readiness_passed'] for item in cases),
        'behavior_action_discovery_failures': sum(
            len(item['behavior_action_discovery_failures']) for item in cases),
        'lifecycle_startup_failures': sum(
            bool(item['lifecycle_startup_failures']) or
            not item['endpoint_readiness_passed'] for item in cases),
        'duplicate_world_errors': sum(
            item['teardown_report'].get('world_present') is True
            for item in cases if item.get('teardown_report')),
        'cases': cases,
    }
    _write_json(MINI_ROOT / 'mini_campaign_report.json', report)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('stress', 'mixed', 'mini'))
    phase = parser.parse_args().phase
    if phase == 'stress':
        report = run_teardown_stress()
    elif phase == 'mixed':
        report = run_mixed_handoff()
    else:
        report = run_mini_campaign()
    print(json.dumps({
        'phase': phase,
        'pass': report['pass'],
        'pass_count': report.get(
            'pass_count', report.get('launches_passed')),
        'expected_count': report.get(
            'expected_cycles', report.get('expected_runs')),
    }, sort_keys=True), flush=True)
    raise SystemExit(0 if report['pass'] else 1)


if __name__ == '__main__':
    main()
