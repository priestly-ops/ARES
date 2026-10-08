#!/usr/bin/env python3
"""Run the immutable Week 6.3 lifecycle-race smoke and startup stress v2."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]
DIAGNOSTICS_PATH = WORKSPACE / 'scripts/run_week6_3_diagnostics.py'
SPEC = importlib.util.spec_from_file_location(
    'week6_3_existing_diagnostics', DIAGNOSTICS_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f'cannot import {DIAGNOSTICS_PATH}')
EXISTING = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXISTING)

ROOT = WORKSPACE / 'results/week6/week6_3_diagnostics'
FIX_ROOT = ROOT / 'lifecycle_race_fix'
SMOKE_ROOT = FIX_ROOT / 'smoke_protected_s2530'
V2_ROOT = ROOT / 'dds_startup_stress_v2'
V2_REPORT = V2_ROOT / 'dds_startup_stress_v2_report.json'
V2_CASES = EXISTING.DDS_CASES


def _load(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _write(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    temporary.replace(path)


def _lifecycle_counts(run_directory: Path) -> dict[str, Any]:
    evidence = _load(
        run_directory / 'lifecycle_transition_evidence.json') or {}
    summary = evidence.get('summary', {})
    operations = evidence.get('operations', [])
    return {
        'lifecycle_evidence_present': bool(operations),
        'lifecycle_operation_count': len(operations),
        'lifecycle_orchestration_passed': bool(
            operations and summary.get('success')),
        'lifecycle_service_response_losses': int(
            summary.get('service_response_losses', 0)),
        'transition_event_recoveries': int(
            summary.get('transition_event_recoveries', 0)),
        'get_state_recoveries': int(
            summary.get('get_state_recoveries', 0)),
        'genuine_lifecycle_failures': int(
            summary.get('genuine_lifecycle_failures', 0)),
    }


def _endpoint_failure_count(case: dict[str, Any]) -> int:
    return sum(len(case.get(field, [])) for field in (
        'planner_action_discovery_failures',
        'controller_action_discovery_failures',
        'planner_service_discovery_failures',
        'costmap_service_discovery_failures',
        'behavior_action_discovery_failures',
    ))


def _enrich(case: dict[str, Any]) -> dict[str, Any]:
    run_directory = WORKSPACE / case['run_directory']
    case.update(_lifecycle_counts(run_directory))
    case['endpoint_discovery_failure_count'] = _endpoint_failure_count(case)
    case['passed'] = bool(
        EXISTING._dds_case_passed(case) and
        case['lifecycle_orchestration_passed'] and
        case['genuine_lifecycle_failures'] == 0)
    return case


def run_smoke(*, attempt: int = 1) -> dict[str, Any]:
    if attempt < 1:
        raise ValueError('smoke attempt must be positive')
    run_directory = (
        SMOKE_ROOT if attempt == 1 else
        FIX_ROOT / f'smoke_protected_s2530_attempt_{attempt:02d}')
    report_path = (
        FIX_ROOT / 'smoke_report.json' if attempt == 1 else
        FIX_ROOT / f'smoke_attempt_{attempt:02d}_report.json')
    if run_directory.exists() and any(run_directory.iterdir()):
        raise FileExistsError(
            f'{run_directory} already contains immutable smoke evidence')
    case = EXISTING._run_case(
        run_directory,
        mode='protected', scenario='healthy', seed=2530,
        startup_only=True)
    case = _enrich(case)
    report = {
        'schema_version': 1,
        'phase': 'smoke_protected_s2530',
        'no_navigation_goal': True,
        'passed': case['passed'],
        'case': case,
    }
    _write(report_path, report)
    return report


def _existing_smoke_passed() -> bool:
    report_paths = [FIX_ROOT / 'smoke_report.json']
    report_paths.extend(sorted(FIX_ROOT.glob('smoke_attempt_*_report.json')))
    return any(
        bool((report := _load(path)) and report.get('passed'))
        for path in report_paths)


def _v2_summary(cases: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        'schema_version': 1,
        'phase': 'dds_startup_stress_v2',
        'expected_cycles': 10,
        'attempted_cycles': len(cases),
        'pass_count': sum(case.get('passed', False) for case in cases),
        'pass': len(cases) == 10 and all(
            case.get('passed', False) for case in cases),
        'lifecycle_service_response_losses': sum(
            case.get('lifecycle_service_response_losses', 0)
            for case in cases),
        'transition_event_recoveries': sum(
            case.get('transition_event_recoveries', 0)
            for case in cases),
        'get_state_recoveries': sum(
            case.get('get_state_recoveries', 0)
            for case in cases),
        'genuine_lifecycle_failures': sum(
            case.get('genuine_lifecycle_failures', 0)
            for case in cases),
        'endpoint_discovery_failures': sum(
            case.get('endpoint_discovery_failure_count', 0)
            for case in cases),
        'world_isolation_failures': sum(
            not case.get('pre_run_isolation_passed', False)
            for case in cases),
        'teardown_failures': sum(
            not case.get('post_run_teardown_passed', False)
            for case in cases),
        'cases': cases,
    }


def run_stress_v2() -> dict[str, Any]:
    if not _existing_smoke_passed():
        raise RuntimeError(
            'protected seed2530 smoke did not pass; stress v2 is blocked')
    if V2_ROOT.exists():
        raise FileExistsError(
            f'{V2_ROOT} exists; refusing to rerun immutable stress v2')
    cases: list[dict[str, Any]] = []
    for index, (mode, seed) in enumerate(V2_CASES, start=1):
        run_directory = (
            V2_ROOT / f'cycle_{index:02d}' /
            f'healthy_{mode}_s{seed}')
        case = EXISTING._run_case(
            run_directory,
            mode=mode, scenario='healthy', seed=seed,
            startup_only=True)
        case['cycle'] = index
        case = _enrich(case)
        cases.append(case)
        _write(V2_REPORT, _v2_summary(cases))
        if not case.get('post_run_teardown_passed'):
            break
    report = _v2_summary(cases)
    _write(V2_REPORT, report)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('smoke', 'stress-v2', 'all'))
    parser.add_argument('--smoke-attempt', type=int, default=1)
    args = parser.parse_args()
    smoke: dict[str, Any] | None = None
    if args.phase in ('smoke', 'all'):
        smoke = run_smoke(attempt=args.smoke_attempt)
        print(json.dumps({
            'phase': 'smoke',
            'passed': smoke['passed'],
        }, sort_keys=True), flush=True)
        if not smoke['passed']:
            raise SystemExit(1)
    if args.phase in ('stress-v2', 'all'):
        stress = run_stress_v2()
        print(json.dumps({
            'phase': 'stress-v2',
            'pass_count': stress['pass_count'],
            'expected_count': 10,
            'passed': stress['pass'],
        }, sort_keys=True), flush=True)
        raise SystemExit(0 if stress['pass'] else 1)


if __name__ == '__main__':
    main()
