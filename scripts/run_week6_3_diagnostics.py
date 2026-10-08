#!/usr/bin/env python3
"""Run gated Week 6.3 diagnostics without campaign edits."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]
RUNNER = WORKSPACE / 'scripts/run_week6_navigation.py'
ROOT = WORKSPACE / 'results/week6/week6_3_diagnostics'
DDS_ROOT = ROOT / 'dds_startup_stress'
COLLISION_ROOT = ROOT / 'healthy_protected_s2531_collision_diagnostic'
MATCHED_ROOT = ROOT / 'matched_healthy_s2531'

DDS_CASES = (
    ('baseline', 2530),
    ('unprotected', 2530),
    ('protected', 2530),
    ('unprotected', 2531),
    ('protected', 2531),
    ('baseline', 2531),
    ('protected', 2532),
    ('unprotected', 2532),
    ('protected', 2530),
    ('unprotected', 2530),
)
MATCHED_CASES = (
    ('baseline', 'healthy', 2531),
    ('unprotected', 'healthy', 2531),
    ('protected', 'healthy', 2531),
)
REQUIRED_PLANNER_ACTIONS = (
    '/compute_path_to_pose',
    '/compute_path_through_poses',
)
REQUIRED_BEHAVIOR_ACTIONS = (
    '/spin', '/backup', '/drive_on_heading', '/wait',
)
REQUIRED_SERVICES = (
    '/is_path_valid',
    '/global_costmap/clear_entirely_global_costmap',
    '/local_costmap/clear_entirely_local_costmap',
)


def _load(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _write(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(
        json.dumps(data, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    temp.replace(path)


def _run_case(
        run_directory: Path, *, mode: str, scenario: str, seed: int,
        startup_only: bool, collision_diagnostics: bool = False) \
        -> dict[str, Any]:
    if run_directory.exists() and any(run_directory.iterdir()):
        raise FileExistsError(
            f'{run_directory} already has evidence; refusing to rerun')
    run_directory.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    command = [
        sys.executable, str(RUNNER),
        '--navigation-mode', mode,
        '--scenario', scenario,
        '--seed', str(seed),
        '--run-directory', str(run_directory.relative_to(WORKSPACE)),
    ]
    if startup_only:
        command.append('--startup-only')
    if collision_diagnostics:
        command.append('--collision-diagnostics')
    completed = subprocess.run(
        command, cwd=WORKSPACE, env=environment,
        capture_output=True, text=True, check=False)
    (run_directory / 'orchestrator_stdout.log').write_text(
        completed.stdout + completed.stderr, encoding='utf-8')

    startup = _load(run_directory / 'endpoint_readiness.json')
    isolation = _load(run_directory / 'runtime_isolation.json')
    teardown = _load(run_directory / 'post_run_teardown.json')
    result = _load(run_directory / 'result.json')
    barrier = (startup or {}).get('readiness_barrier') or {}
    actions = barrier.get('actions', {})
    services = barrier.get('services', {})
    lifecycle = barrier.get('lifecycle_states', {})
    confirmations = (startup or {}).get(
        'lifecycle_active_confirmations', {})
    action_graph = barrier.get('action_graph_diagnostics', {})
    initial = (isolation or {}).get('checks_before_cleanup', {})
    initial_topics = initial.get('topic_publishers', {})
    initial_graph = {
        'owned_processes': initial.get('owned_processes', []),
        'critical_ros_nodes': initial.get('critical_ros_nodes', []),
        'world_services': initial.get('gazebo_ares_world_services', []),
        'clock_publishers': initial_topics.get(
            '/clock', {}).get('publisher_count'),
        'raw_clock_publishers': initial_topics.get(
            '/ares/week6/raw_clock', {}).get('publisher_count'),
    }
    planner_missing = [
        name for name in REQUIRED_PLANNER_ACTIONS
        if not actions.get(name, False)
    ]
    behavior_missing = [
        name for name in REQUIRED_BEHAVIOR_ACTIONS
        if not actions.get(name, False)
    ]
    controller_missing = (
        [] if actions.get('/follow_path', False) else ['/follow_path'])
    service_missing = [
        name for name in REQUIRED_SERVICES
        if not services.get(name, False)
    ]
    required_lifecycle = [
        name for name in lifecycle if name != 'bt_navigator'
    ]
    lifecycle_missing = [
        name for name in required_lifecycle
        if lifecycle.get(name) != 'active' or
        (name in confirmations and confirmations.get(name) is not True)
    ]
    bt_active = confirmations.get('bt_navigator') is True
    startup_passed = bool(
        startup and startup.get('passed') and
        barrier.get('ready') and not planner_missing and
        not behavior_missing and not service_missing and
        not lifecycle_missing and bt_active)
    preflight_passed = bool(
        isolation and isolation.get('world_exclusivity_ready'))
    teardown_passed = bool(teardown and teardown.get('teardown_ready'))
    return {
        'run_directory': str(run_directory.relative_to(WORKSPACE)),
        'navigation_mode': mode,
        'scenario': scenario,
        'seed': seed,
        'startup_only': startup_only,
        'runner_exit_code': completed.returncode,
        'pre_run_isolation_passed': preflight_passed,
        'pre_run_convergence_duration_sec': (
            (isolation or {}).get('pre_run_convergence_duration_sec')),
        'initial_stale_resources': initial_graph,
        'planner_action_discovery_failures': planner_missing,
        'controller_action_discovery_failures': controller_missing,
        'costmap_service_discovery_failures': [
            name for name in service_missing if 'costmap' in name
        ],
        'planner_service_discovery_failures': [
            name for name in service_missing if name == '/is_path_valid'
        ],
        'behavior_action_discovery_failures': behavior_missing,
        'lifecycle_startup_failures': lifecycle_missing,
        'bt_navigator_active': bt_active,
        'startup_readiness_passed': startup_passed,
        'post_run_teardown_passed': teardown_passed,
        'first_clean_snapshot_utc': (teardown or {}).get(
            'first_clean_snapshot_utc'),
        'second_clean_snapshot_utc': (teardown or {}).get(
            'second_clean_snapshot_utc'),
        'total_teardown_duration_sec': (teardown or {}).get(
            'total_teardown_duration_sec'),
        'clock_publisher_samples': (teardown or {}).get(
            'clock_publisher_counts_over_time', []),
        'raw_clock_publisher_samples': (teardown or {}).get(
            'raw_clock_publisher_counts_over_time', []),
        'world_present_after_teardown': (teardown or {}).get(
            'world_present'),
        'remaining_critical_nodes': (teardown or {}).get(
            'remaining_critical_nodes', []),
        'controller_parameter_diagnostics': barrier.get(
            'controller_parameter_diagnostics', {}),
        'relevant_topic_types': barrier.get('relevant_topic_types', {}),
        'action_graph_diagnostics': action_graph,
        'mission_completed': (result or {}).get('mission_completed'),
        'waypoints_reached': (result or {}).get('waypoints_reached'),
        'waypoints_total': (result or {}).get('waypoints_total'),
        'collision_events_path': (
            str((run_directory / 'collision_events.json').relative_to(
                WORKSPACE))
            if (run_directory / 'collision_events.json').is_file() else None),
    }


def _dds_case_passed(case: dict[str, Any]) -> bool:
    return all((
        case.get('pre_run_isolation_passed'),
        case.get('startup_readiness_passed'),
        case.get('post_run_teardown_passed'),
        not case.get('planner_action_discovery_failures'),
        not case.get('controller_action_discovery_failures'),
        not case.get('costmap_service_discovery_failures'),
        not case.get('behavior_action_discovery_failures'),
        not case.get('lifecycle_startup_failures'),
        case.get('bt_navigator_active'),
    ))


def _refresh_case_classification(case: dict[str, Any]) -> None:
    """Reclassify persisted readiness using post-activation runner evidence."""
    run_directory = WORKSPACE / case['run_directory']
    startup = _load(run_directory / 'endpoint_readiness.json') or {}
    isolation = _load(run_directory / 'runtime_isolation.json') or {}
    teardown = _load(run_directory / 'post_run_teardown.json') or {}
    barrier = startup.get('readiness_barrier') or {}
    actions = barrier.get('actions', {})
    services = barrier.get('services', {})
    lifecycle_states = barrier.get('lifecycle_states', {})
    confirmations = startup.get('lifecycle_active_confirmations', {})
    required_lifecycle = [
        name for name in lifecycle_states if name != 'bt_navigator'
    ]
    lifecycle_missing = [
        name for name in required_lifecycle
        if lifecycle_states.get(name) != 'active' or
        (name in confirmations and confirmations.get(name) is not True)
    ]
    bt_active = confirmations.get('bt_navigator') is True
    case['planner_action_discovery_failures'] = [
        name for name in REQUIRED_PLANNER_ACTIONS
        if not actions.get(name, False)
    ]
    case['controller_action_discovery_failures'] = (
        [] if actions.get('/follow_path', False) else ['/follow_path'])
    case['costmap_service_discovery_failures'] = [
        name for name in REQUIRED_SERVICES
        if 'costmap' in name and not services.get(name, False)
    ]
    case['planner_service_discovery_failures'] = [
        name for name in REQUIRED_SERVICES
        if name == '/is_path_valid' and not services.get(name, False)
    ]
    case['behavior_action_discovery_failures'] = [
        name for name in REQUIRED_BEHAVIOR_ACTIONS
        if not actions.get(name, False)
    ]
    case['lifecycle_startup_failures'] = lifecycle_missing
    case['bt_navigator_active'] = bt_active
    case['bt_navigator_state_before_activation'] = lifecycle_states.get(
        'bt_navigator')
    case['startup_readiness_passed'] = bool(
        startup.get('passed') and barrier.get('ready') and
        not case['planner_action_discovery_failures'] and
        not case['controller_action_discovery_failures'] and
        not case['costmap_service_discovery_failures'] and
        not case['planner_service_discovery_failures'] and
        not case['behavior_action_discovery_failures'] and
        not lifecycle_missing and bt_active)
    case['pre_run_isolation_passed'] = bool(
        isolation.get('world_exclusivity_ready'))
    case['post_run_teardown_passed'] = bool(teardown.get('teardown_ready'))
    case['controller_parameter_diagnostics'] = barrier.get(
        'controller_parameter_diagnostics', {})
    case['relevant_topic_types'] = barrier.get('relevant_topic_types', {})
    case['passed'] = _dds_case_passed(case)


def run_dds_startup_stress() -> dict[str, Any]:
    report_path = DDS_ROOT / 'dds_startup_stress_report.json'
    if DDS_ROOT.exists():
        existing = _load(report_path)
        if existing is None:
            raise FileExistsError(
                f'{DDS_ROOT} exists without a readable report')
        cases = list(existing.get('cases', []))
        if any(case.get('cycle') != index
               for index, case in enumerate(cases, start=1)):
            raise RuntimeError('existing DDS stress order is invalid')
        if any(not case.get('post_run_teardown_passed') for case in cases):
            raise RuntimeError(
                'previous cycle teardown failed; cannot safely continue')
        for case in cases:
            case.setdefault('pre_reclassification_passed', case.get('passed'))
            case.setdefault(
                'pre_reclassification_startup_passed',
                case.get('startup_readiness_passed'))
            _refresh_case_classification(case)
    else:
        cases = []

    for index in range(len(cases) + 1, len(DDS_CASES) + 1):
        mode, seed = DDS_CASES[index - 1]
        case = _run_case(
            DDS_ROOT / f'cycle_{index:02d}' /
            f'healthy_{mode}_s{seed}',
            mode=mode, scenario='healthy', seed=seed, startup_only=True)
        case['cycle'] = index
        startup = _load(
            WORKSPACE / case['run_directory'] /
            'endpoint_readiness.json') or {}
        case['bt_navigator_state_before_activation'] = (
            (startup.get('readiness_barrier') or {}).get(
                'lifecycle_states', {}).get('bt_navigator'))
        _refresh_case_classification(case)
        cases.append(case)
        _write(report_path, {
            'schema_version': 1,
            'phase': 'dds_startup_stress',
            'expected_cycles': 10,
            'attempted_cycles': len(cases),
            'pass_count': sum(item.get('passed', False) for item in cases),
            'cases': cases,
        })
        if not case['post_run_teardown_passed']:
            break

    summary = {
        'schema_version': 1,
        'phase': 'dds_startup_stress',
        'expected_cycles': 10,
        'attempted_cycles': len(cases),
        'pass_count': sum(item.get('passed', False) for item in cases),
        'pass': (
            len(cases) == 10 and
            all(item.get('passed', False) for item in cases)),
        'planner_action_discovery_failures': sum(
            len(item.get('planner_action_discovery_failures', []))
            for item in cases),
        'controller_action_discovery_failures': sum(
            len(item.get('controller_action_discovery_failures', []))
            for item in cases),
        'planner_service_discovery_failures': sum(
            len(item.get('planner_service_discovery_failures', []))
            for item in cases),
        'costmap_service_discovery_failures': sum(
            len(item.get('costmap_service_discovery_failures', []))
            for item in cases),
        'behavior_action_discovery_failures': sum(
            len(item.get('behavior_action_discovery_failures', []))
            for item in cases),
        'lifecycle_startup_failures': sum(
            len(item.get('lifecycle_startup_failures', [])) +
            int(not item.get('bt_navigator_active', False))
            for item in cases),
        'world_isolation_failures': sum(
            not item.get('pre_run_isolation_passed', False)
            for item in cases),
        'post_run_teardown_failures': sum(
            not item.get('post_run_teardown_passed', False)
            for item in cases),
        'cases': cases,
    }
    _write(report_path, summary)
    return summary


def _dds_gate_passed() -> bool:
    report = _load(
        DDS_ROOT.parent / 'dds_startup_stress_v2' /
        'dds_startup_stress_v2_report.json')
    return bool(
        report and report.get('pass') and
        report.get('attempted_cycles') == 10 and
        all(report.get(field) == 0 for field in (
            'endpoint_discovery_failures',
            'genuine_lifecycle_failures',
            'world_isolation_failures',
            'teardown_failures')))


def run_collision_diagnostic() -> dict[str, Any]:
    if not _dds_gate_passed():
        raise RuntimeError(
            'DDS startup stress is not 10/10; navigation diagnostic blocked')
    report_path = COLLISION_ROOT / 'collision_diagnostic_report.json'
    if COLLISION_ROOT.exists():
        raise FileExistsError(
            f'{COLLISION_ROOT} exists; refusing to rerun collision diagnostic')
    case = _run_case(
        COLLISION_ROOT, mode='protected', scenario='healthy', seed=2531,
        startup_only=False, collision_diagnostics=True)
    report = {
        'schema_version': 1,
        'phase': 'healthy_protected_s2531_collision_diagnostic',
        'case': case,
        'mission_complete': case.get('mission_completed') is True,
        'collision_events': _load(COLLISION_ROOT / 'collision_events.json'),
    }
    _write(report_path, report)
    return report


def run_matched_healthy() -> dict[str, Any]:
    collision_report = _load(
        COLLISION_ROOT / 'collision_diagnostic_report.json')
    if not collision_report or not collision_report.get('mission_complete'):
        raise RuntimeError(
            'protected seed2531 diagnostic did not complete; matched '
            'healthy cases are blocked')
    report_path = MATCHED_ROOT / 'matched_healthy_report.json'
    if MATCHED_ROOT.exists():
        raise FileExistsError(
            f'{MATCHED_ROOT} exists; refusing to rerun matched diagnostics')
    cases = []
    for index, (mode, scenario, seed) in enumerate(MATCHED_CASES, start=1):
        case = _run_case(
            MATCHED_ROOT / f'case_{index:02d}' /
            f'{scenario}_{mode}_s{seed}',
            mode=mode, scenario=scenario, seed=seed,
            startup_only=False, collision_diagnostics=True)
        case['case'] = index
        cases.append(case)
        _write(report_path, {
            'schema_version': 1,
            'phase': 'matched_healthy_s2531',
            'expected_runs': 3,
            'attempted_runs': len(cases),
            'cases': cases,
        })
        if not case.get('post_run_teardown_passed'):
            break
    report = {
        'schema_version': 1,
        'phase': 'matched_healthy_s2531',
        'expected_runs': 3,
        'attempted_runs': len(cases),
        'complete': len(cases) == 3 and all(
            item.get('mission_completed') is not None for item in cases),
        'cases': cases,
    }
    _write(report_path, report)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'phase', choices=('dds-startup-stress', 'collision', 'matched'))
    phase = parser.parse_args().phase
    if phase == 'dds-startup-stress':
        report = run_dds_startup_stress()
        print(json.dumps({
            'phase': phase,
            'pass': report['pass'],
            'pass_count': report['pass_count'],
            'expected_count': 10,
        }, sort_keys=True), flush=True)
        raise SystemExit(0 if report['pass'] else 1)
    if phase == 'collision':
        report = run_collision_diagnostic()
        print(json.dumps({
            'phase': phase,
            'mission_complete': report['mission_complete'],
        }, sort_keys=True), flush=True)
        raise SystemExit(
            0 if report['mission_complete'] else 1)
    report = run_matched_healthy()
    print(json.dumps({
        'phase': phase,
        'complete': report['complete'],
        'attempted_count': report['attempted_runs'],
    }, sort_keys=True), flush=True)
    raise SystemExit(0 if report['complete'] else 1)


if __name__ == '__main__':
    main()
