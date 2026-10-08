#!/usr/bin/env python3
"""Verify isolation detects and safely releases a harness-owned stale PID."""

from __future__ import annotations

import importlib.util
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
OUTPUT = WORKSPACE / (
    'results/week6/week6_3_diagnostics/'
    'runtime_isolation_probe_stress/controlled_contamination.json')
SPEC = importlib.util.spec_from_file_location(
    'week6_runtime_isolation',
    WORKSPACE / 'scripts/week6_runtime_isolation.py')
assert SPEC is not None and SPEC.loader is not None
ISOLATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ISOLATION)


def _clean_probe(command, **_kwargs):
    if command[:3] == ['ros2', 'node', 'list']:
        return subprocess.CompletedProcess(command, 0, '', '')
    if command[:3] == ['ros2', 'topic', 'info']:
        return subprocess.CompletedProcess(
            command, 1, '', f"Unknown topic '{command[3]}'")
    if command[:3] == ['gz', 'service', '-l']:
        return subprocess.CompletedProcess(command, 0, '', '')
    raise AssertionError(f'unexpected probe command {command!r}')


def main() -> int:
    if OUTPUT.exists():
        raise FileExistsError(f'refusing to overwrite {OUTPUT}')
    run_id = f'isolation-controlled-{os.getpid()}-{time.time_ns()}'
    child_environment = os.environ.copy()
    child_environment['ARES_WEEK6_RUN_ID'] = run_id
    process = subprocess.Popen(
        [sys.executable, '-c', 'import time; time.sleep(60)'],
        env=child_environment, start_new_session=True)
    try:
        time.sleep(0.15)
        processes = ISOLATION._read_processes()
        owned, reasons = ISOLATION._owned_processes(processes)
        detected = process.pid in owned
        record = {
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'controlled_resource_pid': process.pid,
            'run_id_marker': run_id,
            'process_detected_as_week6_owned': detected,
            'ownership_reason': reasons.get(process.pid),
            'classification': (
                'CONTAMINATION_CONFIRMED' if detected else
                'PROBE_UNCERTAIN'),
            'cleanup': 'SIGTERM to harness-created PID only',
        }
        if detected:
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=3.0)
        OUTPUT.write_text(
            json.dumps(record, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
        print(json.dumps(record, indent=2))
        return 0 if detected else 1
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3.0)


if __name__ == '__main__':
    raise SystemExit(main())
