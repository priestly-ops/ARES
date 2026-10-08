#!/usr/bin/env python3
"""Run isolated Week 6 runtime probes without starting any simulation."""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
OUTPUT = WORKSPACE / (
    'results/week6/week6_3_diagnostics/'
    'runtime_isolation_probe_stress')
ISOLATION_PATH = WORKSPACE / 'scripts/week6_runtime_isolation.py'
SPEC = importlib.util.spec_from_file_location(
    'week6_runtime_isolation', ISOLATION_PATH)
assert SPEC is not None and SPEC.loader is not None
ISOLATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ISOLATION)


def _controlled_cases(environment: dict[str, str]) -> dict[str, object]:
    def clean(command, **_kwargs):
        if command[:3] == ['ros2', 'node', 'list']:
            return subprocess.CompletedProcess(command, 0, '', '')
        if command[:3] == ['ros2', 'topic', 'info']:
            return subprocess.CompletedProcess(
                command, 1, '', f"Unknown topic '{command[3]}'")
        if command[:3] == ['gz', 'service', '-l']:
            return subprocess.CompletedProcess(command, 0, '', '')
        raise AssertionError(f'unexpected probe command {command!r}')

    clean_snapshot = ISOLATION._snapshot(
        run_command=clean, environment=environment)

    def contaminated(command, **kwargs):
        if command[:3] == ['ros2', 'node', 'list']:
            return subprocess.CompletedProcess(
                command, 0, '/planner_server\n', '')
        return clean(command, **kwargs)

    contaminated_snapshot = ISOLATION._snapshot(
        run_command=contaminated, environment=environment)

    def broken(command, **_kwargs):
        raise subprocess.TimeoutExpired(command, timeout=8.0)

    uncertain_snapshot = ISOLATION._snapshot(
        run_command=broken, environment=environment)
    return {
        'clean_case': ISOLATION._snapshot_classification(clean_snapshot),
        'controlled_stale_week6_node_case':
            ISOLATION._snapshot_classification(contaminated_snapshot),
        'intentionally_broken_probe_case':
            ISOLATION._snapshot_classification(uncertain_snapshot),
        'evidence': {
            'clean': ISOLATION._jsonable_snapshot(clean_snapshot),
            'controlled_contamination':
                ISOLATION._jsonable_snapshot(contaminated_snapshot),
            'probe_failure': ISOLATION._jsonable_snapshot(
                uncertain_snapshot),
        },
    }


def main() -> int:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    if any(OUTPUT.iterdir()):
        raise FileExistsError(
            f'refusing to overwrite existing stress evidence in {OUTPUT}')
    environment = os.environ.copy()
    environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    iterations = []
    for index in range(1, 21):
        started = time.monotonic()
        snapshot = ISOLATION._snapshot(
            run_command=subprocess.run, environment=environment)
        iterations.append({
            'iteration': index,
            'started_utc': datetime.now(timezone.utc).isoformat(),
            'elapsed_sec': time.monotonic() - started,
            'classification': ISOLATION._snapshot_classification(snapshot),
            'gazebo_probe_status': snapshot['gazebo_world_probe']['ok'],
            'ros_node_probe_status': snapshot['ros_node_probe']['ok'],
            'clock_probe_status': snapshot['topic_publishers']['/clock'][
                'status'],
            'raw_clock_probe_status': snapshot['topic_publishers'][
                '/ares/week6/raw_clock']['status'],
            'process_ownership_status': (
                'CONTAMINATION_CONFIRMED'
                if snapshot['owned_processes'] else 'CLEAN_CONFIRMED'),
            'errors': [
                probe.get('probe_error') or probe.get('stderr')
                for probe in (
                    snapshot['gazebo_world_probe'],
                    snapshot['ros_node_probe'],
                    *snapshot['topic_publishers'].values())
                if not probe.get('ok') and
                (probe.get('probe_error') or probe.get('stderr'))],
            'snapshot': ISOLATION._jsonable_snapshot(snapshot),
        })

    classifications = [entry['classification'] for entry in iterations]
    controlled = _controlled_cases(environment)
    report = {
        'schema_version': 1,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'probe_environment': {
            key: environment[key] for key in (
                'RMW_IMPLEMENTATION', 'ROS_AUTOMATIC_DISCOVERY_RANGE')},
        'mission_processes_started': False,
        'iteration_count': len(iterations),
        'clean_confirmed_count': classifications.count('CLEAN_CONFIRMED'),
        'probe_uncertain_count': classifications.count('PROBE_UNCERTAIN'),
        'contamination_confirmed_count': classifications.count(
            'CONTAMINATION_CONFIRMED'),
        'iterations': iterations,
        'controlled_cases': controlled,
    }
    (OUTPUT / 'probe_stress.json').write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({
        key: report[key] for key in (
            'iteration_count', 'clean_confirmed_count',
            'probe_uncertain_count', 'contamination_confirmed_count',
            'controlled_cases', 'probe_environment')
    }, indent=2))
    return 0 if report['clean_confirmed_count'] == 20 else 1


if __name__ == '__main__':
    raise SystemExit(main())
