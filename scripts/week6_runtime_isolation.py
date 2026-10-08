#!/usr/bin/env python3
"""Establish exclusive ownership of the ARES Week 6 runtime before launch."""

from __future__ import annotations

import json
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable


WEEK6_LAUNCH_SIGNATURE = (
    'ros2', 'launch', 'ares_reliability', 'week6_navigation.launch.py')
CRITICAL_ROS_NODES = {
    '/amcl',
    '/behavior_server',
    '/bt_navigator',
    '/cmd_vel_adapter',
    '/controller_server',
    '/ekf_filter_node',
    '/ekf_trust_fusion_node',
    '/estimator_health_monitor',
    '/gnss_fault_injector',
    '/gnss_trusted_proxy',
    '/gnss_unprotected_normalizer',
    '/lifecycle_manager_week6_bt',
    '/lifecycle_manager_week6_upstream',
    '/localization_consistency_monitor',
    '/map_server',
    '/navsat_trust_transform',
    '/planner_server',
    '/recovery_manager',
    '/robot_state_publisher',
    '/ros_gz_bridge',
    '/trust_engine',
    '/week6_clock_boundary',
    '/week6_nav2_lifecycle_orchestrator_bt',
    '/week6_nav2_lifecycle_orchestrator_upstream',
    '/week6_nav2_startup_barrier',
}
TOPICS_TO_CHECK = ('/clock', '/ares/week6/raw_clock')
WORLD_SERVICE_PATTERN = re.compile(r'(?:^|/)world/ares_world(?:/|$)')

CommandRunner = Callable[..., subprocess.CompletedProcess[str]]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _process_identity(process: dict) -> dict[str, object]:
    return {
        'pid': process['pid'],
        'ppid': process['ppid'],
        'pgid': process['pgid'],
        'start_time_ticks': process['start_time_ticks'],
        'command': process['command'],
    }


class RunProcessTracker:
    """Track the dedicated launch session and inherited run-id processes."""

    def __init__(self, launch_pid: int, run_id: str) -> None:
        self.launch_pid = launch_pid
        self.run_id = run_id
        self._processes: dict[tuple[int, str], dict] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._sample_loop,
            name=f'week6-process-tracker-{launch_pid}',
            daemon=True,
        )
        self._thread.start()

    def _sample(self) -> None:
        processes = _read_processes()
        matching = {
            pid: process for pid, process in processes.items()
            if process['pgid'] == self.launch_pid or
            process['week6_run_id'] == self.run_id
        }
        roots = {
            pid for pid, process in matching.items()
            if pid == self.launch_pid or
            process['week6_run_id'] == self.run_id
        }
        changed = True
        while changed:
            changed = False
            for pid, process in processes.items():
                if (pid not in matching and
                        process['ppid'] in roots and
                        (process['pgid'] == self.launch_pid or
                         process['week6_run_id'] == self.run_id)):
                    matching[pid] = process
                    roots.add(pid)
                    changed = True
        with self._lock:
            for process in matching.values():
                key = (process['pid'], process['start_time_ticks'])
                self._processes[key] = process

    def _sample_loop(self) -> None:
        while not self._stop.is_set():
            self._sample()
            self._stop.wait(0.1)
        self._sample()

    def finish(self) -> list[dict[str, object]]:
        self._stop.set()
        self._thread.join(timeout=2.0)
        with self._lock:
            return [
                _process_identity(process)
                for _, process in sorted(self._processes.items())
            ]


def _tracked_survivors(
        tracked: list[dict[str, object]]) -> list[dict[str, object]]:
    current = _read_processes()
    return [
        record for record in tracked
        if record['pid'] in current and
        current[record['pid']]['start_time_ticks'] ==
        record['start_time_ticks']
    ]


def _wait_tracked_exit(
        tracked: list[dict[str, object]],
        process: subprocess.Popen | None,
        timeout_sec: float) -> list[dict[str, object]]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        survivors = _tracked_survivors(tracked)
        if process is not None:
            process.poll()
        if not survivors:
            return []
        time.sleep(0.2)
    return _tracked_survivors(tracked)


def _signal_survivors(
        survivors: list[dict[str, object]], sig: int,
        actions: list[dict[str, object]]) -> None:
    current = _read_processes()
    for record in survivors:
        pid = int(record['pid'])
        process = current.get(pid)
        if (process is None or
                process['start_time_ticks'] != record['start_time_ticks']):
            continue
        try:
            os.kill(pid, sig)
            actions.append({
                'signal': signal.Signals(sig).name,
                'pid': pid,
                'command': record['command'],
            })
        except ProcessLookupError:
            pass


def post_run_teardown(
        *, report_path: Path, run_name: str,
        result_path: Path | None,
        launch: subprocess.Popen | None,
        tracked_processes: list[dict[str, object]],
        environment: dict[str, str] | None = None,
        release_timeout_sec: float = 45.0,
        run_command: CommandRunner = subprocess.run) -> dict[str, object]:
    """Stop only tracked run PIDs and verify the global ARES runtime is clear."""
    started = time.monotonic()
    tracked_by_identity = {
        (int(record['pid']), str(record['start_time_ticks'])): record
        for record in tracked_processes
    }
    if launch is not None:
        current = _read_processes()
        for pid, process in current.items():
            if process['pgid'] == launch.pid:
                tracked_by_identity.setdefault(
                    (pid, str(process['start_time_ticks'])),
                    _process_identity(process))
    tracked_processes = list(tracked_by_identity.values())
    report: dict[str, object] = {
        'schema_version': 1,
        'run_name': run_name,
        'result_written_utc': (
            datetime.fromtimestamp(
                result_path.stat().st_mtime, timezone.utc).isoformat()
            if result_path is not None and result_path.exists() else None),
        'teardown_started_utc': utc_now(),
        'launch_process_pid': launch.pid if launch is not None else None,
        'launch_process_exit_code': (
            launch.poll() if launch is not None else None),
        'launch_process_exit_utc': None,
        'tracked_child_pids': tracked_processes,
        'survivors_after_sigint': [],
        'survivors_after_sigterm': [],
        'survivors_after_sigkill': [],
        'cleanup_actions': [],
        'clock_publisher_counts_over_time': [],
        'raw_clock_publisher_counts_over_time': [],
        'first_clean_snapshot_utc': None,
        'second_clean_snapshot_utc': None,
        'remaining_critical_nodes': [],
        'world_present': None,
        'total_teardown_duration_sec': None,
        'teardown_ready': False,
        'classification': 'PROBE_UNCERTAIN',
        'errors': [],
    }
    actions: list[dict[str, object]] = report['cleanup_actions']

    current = _tracked_survivors(tracked_processes)
    _signal_survivors(current, signal.SIGINT, actions)
    survivors = _wait_tracked_exit(tracked_processes, launch, 25.0)
    report['survivors_after_sigint'] = survivors

    if survivors:
        _signal_survivors(survivors, signal.SIGTERM, actions)
        survivors = _wait_tracked_exit(tracked_processes, launch, 8.0)
    report['survivors_after_sigterm'] = survivors

    if survivors:
        _signal_survivors(survivors, signal.SIGKILL, actions)
        survivors = _wait_tracked_exit(tracked_processes, launch, 5.0)
    report['survivors_after_sigkill'] = survivors

    if launch is not None:
        if launch.poll() is not None:
            report['launch_process_exit_code'] = launch.returncode
            report['launch_process_exit_utc'] = utc_now()
        try:
            launch.wait(timeout=0.1)
        except subprocess.TimeoutExpired:
            report['errors'].append(
                'launch parent remained alive after owned-process escalation')
        if launch.poll() is not None:
            report['launch_process_exit_code'] = launch.returncode
            report['launch_process_exit_utc'] = utc_now()

    stable_clean_samples = 0
    deadline = time.monotonic() + release_timeout_sec
    last_snapshot: dict[str, object] | None = None
    while time.monotonic() < deadline:
        snapshot = _snapshot(
            run_command=run_command, environment=environment)
        last_snapshot = snapshot
        clock_count = _publisher_count(snapshot, '/clock')
        raw_count = _publisher_count(
            snapshot, '/ares/week6/raw_clock')
        sample_time = utc_now()
        report['clock_publisher_counts_over_time'].append({
            'observed_utc': sample_time,
            'publisher_count': clock_count,
        })
        report['raw_clock_publisher_counts_over_time'].append({
            'observed_utc': sample_time,
            'publisher_count': raw_count,
        })
        report['remaining_critical_nodes'] = snapshot['critical_ros_nodes']
        report['world_present'] = (
            bool(snapshot['gazebo_ares_world_services'])
            if snapshot['gazebo_world_probe']['ok'] else None)
        current_tracked = _tracked_survivors(tracked_processes)
        no_run_processes = not current_tracked
        no_stale_ares_processes = not snapshot['owned_processes']
        world_probe_ok = bool(snapshot['gazebo_world_probe']['ok'])
        graph_probe_ok = bool(snapshot['ros_node_probe']['ok'])
        publishers_known = all(
            entry['known']
            for entry in snapshot['topic_publishers'].values())
        sample_clean = (
            no_run_processes and no_stale_ares_processes and
            not snapshot['critical_ros_nodes'] and
            world_probe_ok and
            not snapshot['gazebo_ares_world_services'] and
            graph_probe_ok and publishers_known and
            clock_count == 0 and raw_count == 0)
        stable_clean_samples = stable_clean_samples + 1 if sample_clean else 0
        if stable_clean_samples == 1:
            report['first_clean_snapshot_utc'] = sample_time
        elif stable_clean_samples == 2:
            report['second_clean_snapshot_utc'] = sample_time
        else:
            report['first_clean_snapshot_utc'] = None
            report['second_clean_snapshot_utc'] = None
        if stable_clean_samples >= 2:
            break
        time.sleep(0.25)

    if last_snapshot is None:
        report['errors'].append('post-run cleanup produced no verification sample')
    else:
        if not last_snapshot['ros_node_probe']['ok']:
            report['errors'].append('ROS node graph probe failed')
        if not last_snapshot['gazebo_world_probe']['ok']:
            report['errors'].append('Gazebo world probe failed')
        for topic, probe in last_snapshot['topic_publishers'].items():
            if not probe['known']:
                report['errors'].append(
                    f'publisher count for {topic} is unknown')
        if last_snapshot['owned_processes']:
            report['errors'].append(
                'ARES-owned runtime processes remain after cleanup')
        if last_snapshot['critical_ros_nodes']:
            report['errors'].append(
                'critical ARES ROS nodes remain after cleanup')
        if last_snapshot['gazebo_ares_world_services']:
            report['errors'].append('ares_world remains registered')
        if (_publisher_count(last_snapshot, '/clock') is not None and
                _publisher_count(last_snapshot, '/clock') > 0):
            report['errors'].append('/clock publisher remains')
        raw_count = _publisher_count(
            last_snapshot, '/ares/week6/raw_clock')
        if raw_count is not None and raw_count > 0:
            report['errors'].append('/ares/week6/raw_clock publisher remains')
    if _tracked_survivors(tracked_processes):
        report['errors'].append('tracked run PIDs remain after cleanup')
    if launch is not None and launch.poll() is None:
        report['errors'].append('launch parent remains alive')
    if stable_clean_samples < 2:
        report['errors'].append(
            'runtime release did not produce two consecutive clean samples')

    if last_snapshot is not None:
        report['classification'] = _snapshot_classification(last_snapshot)
    if (report['classification'] == 'CLEAN_CONFIRMED' and
            stable_clean_samples >= 2 and
            not _tracked_survivors(tracked_processes) and
            (launch is None or launch.poll() is not None)):
        report['teardown_ready'] = not report['errors']
    else:
        report['teardown_ready'] = False
    if report['teardown_ready']:
        report['classification'] = 'CLEAN_CONFIRMED'
    report['teardown_finished_utc'] = utc_now()
    report['total_teardown_duration_sec'] = time.monotonic() - started
    _write_report(report_path, report)
    if report['teardown_ready']:
        print('POST_RUN_TEARDOWN_READY: YES', flush=True)
    else:
        print('POST_RUN_TEARDOWN_READY: NO', flush=True)
    return report


def _command(
        command: list[str], *, timeout: float,
        run_command: CommandRunner,
        environment: dict[str, str] | None = None) -> dict[str, object]:
    started_utc = utc_now()
    started = time.monotonic()
    relevant_environment = {
        key: (environment or os.environ).get(key)
        for key in ('RMW_IMPLEMENTATION', 'ROS_AUTOMATIC_DISCOVERY_RANGE')
    }
    try:
        result = run_command(
            command, capture_output=True, text=True, timeout=timeout,
            check=False, env=environment)
        record = {
            'command': command,
            'returncode': result.returncode,
            'stdout': result.stdout,
            'stderr': result.stderr,
            'ok': result.returncode == 0,
            'started_utc': started_utc,
            'ended_utc': utc_now(),
            'elapsed_sec': time.monotonic() - started,
            'timeout_sec': timeout,
            'environment': relevant_environment,
        }
        diagnostic = f"{result.stdout}\n{result.stderr}".lower()
        if any(marker in diagnostic for marker in (
                'error creating socket', 'operation not permitted',
                'transport failed to register')):
            record['ok'] = False
            record['probe_error'] = 'middleware transport initialization failed'
        return record
    except (OSError, subprocess.TimeoutExpired) as error:
        return {
            'command': command,
            'returncode': None,
            'stdout': '',
            'stderr': str(error),
            'ok': False,
            'started_utc': started_utc,
            'ended_utc': utc_now(),
            'elapsed_sec': time.monotonic() - started,
            'timeout_sec': timeout,
            'environment': relevant_environment,
            'probe_error': type(error).__name__,
        }


def _read_processes(proc_root: Path = Path('/proc')) -> dict[int, dict]:
    processes: dict[int, dict] = {}
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            raw = (entry / 'cmdline').read_bytes()
            cmdline = tuple(
                part.decode(errors='replace')
                for part in raw.split(b'\0') if part)
            status = (entry / 'status').read_text(encoding='utf-8')
            ppid_match = re.search(r'^PPid:\s+(\d+)$', status, re.MULTILINE)
            if ppid_match is None:
                continue
            stat = (entry / 'stat').read_text(encoding='utf-8')
            stat_fields = stat[stat.rfind(')') + 2:].split()
            if len(stat_fields) <= 19:
                continue
            if stat_fields[0] in ('Z', 'X'):
                continue
            environment = (entry / 'environ').read_bytes().split(b'\0')
            env = {
                key.decode(errors='replace'): value.decode(errors='replace')
                for item in environment if b'=' in item
                for key, value in (item.split(b'=', 1),)
            }
            processes[pid] = {
                'pid': pid,
                'ppid': int(ppid_match.group(1)),
                'pgid': os.getpgid(pid),
                'start_time_ticks': stat_fields[19],
                'cmdline': cmdline,
                'command': ' '.join(cmdline),
                'week6_run_id': env.get('ARES_WEEK6_RUN_ID'),
            }
        except (OSError, ProcessLookupError, PermissionError):
            continue
    return processes


def _is_week6_launch(process: dict) -> bool:
    command = process['cmdline']
    if not command:
        return False
    joined = ' '.join(command)
    return all(part in joined for part in WEEK6_LAUNCH_SIGNATURE)


def _ares_world_or_bridge_reason(process: dict) -> str | None:
    command = process['command']
    arguments = process['cmdline']
    if ('gz' in arguments and 'sim' in arguments and
            any('ares_baseline_' in arg for arg in arguments)):
        for argument in arguments:
            path = Path(argument)
            if not path.is_file() or 'ares_baseline_' not in str(path):
                continue
            try:
                contents = path.read_text(
                    encoding='utf-8', errors='replace')
            except OSError:
                continue
            if re.search(
                    r'<world\b[^>]*\bname=[\'"]ares_world[\'"]',
                    contents):
                return 'Gazebo sim process loading the ARES ares_world SDF'
    if ('parameter_bridge' in command and
            'ares_baseline_' in command):
        for argument in arguments:
            path = Path(argument)
            if not path.is_file() or 'ares_baseline_' not in str(path):
                continue
            try:
                contents = path.read_text(
                    encoding='utf-8', errors='replace')
            except OSError:
                continue
            if '/ares/week6/raw_clock' in contents:
                return 'ros_gz_bridge using the Week 6 raw-clock bridge config'
    return None


def _owned_processes(processes: dict[int, dict]) -> tuple[dict[int, dict], dict]:
    current_process_chain: set[int] = set()
    current_pid = os.getpid()
    while current_pid in processes and current_pid not in current_process_chain:
        current_process_chain.add(current_pid)
        current_pid = processes[current_pid]['ppid']
    roots = {
        pid for pid, process in processes.items()
        if pid not in current_process_chain and _is_week6_launch(process)
    }
    reasons: dict[int, str] = {}
    for pid, process in processes.items():
        if pid not in current_process_chain and process['week6_run_id']:
            reasons[pid] = 'ARES_WEEK6_RUN_ID environment marker'
        process_reason = _ares_world_or_bridge_reason(process)
        if process_reason is not None:
            reasons[pid] = process_reason

    for root in roots:
        reasons[root] = 'Week 6 launch command signature'
        descendants = {root}
        changed = True
        while changed:
            changed = False
            for pid, process in processes.items():
                if pid not in descendants and process['ppid'] in descendants:
                    descendants.add(pid)
                    changed = True
        for pid in descendants:
            reasons.setdefault(pid, f'descendant of Week 6 launch PID {root}')
    return ({pid: processes[pid] for pid in reasons if pid in processes},
            reasons)


def _topic_publisher_count(
        topic: str, *, run_command: CommandRunner,
        environment: dict[str, str] | None = None) -> dict[str, object]:
    probe = _command(
        ['ros2', 'topic', 'info', topic, '--verbose'],
        timeout=8.0, run_command=run_command, environment=environment)
    output = f"{probe['stdout']}\n{probe['stderr']}"
    match = re.search(r'Publisher count:\s*(\d+)', output)
    if match is not None:
        probe['publisher_count'] = int(match.group(1))
        probe['known'] = probe['ok']
    elif (probe.get('probe_error') is None and
          ('Unknown topic' in output or 'topic not found' in output.lower())):
        probe['publisher_count'] = 0
        probe['known'] = True
    else:
        probe['publisher_count'] = None
        probe['known'] = False
    probe['status'] = (
        'CLEAN_CONFIRMED' if probe['known'] and
        probe['publisher_count'] == 0 else
        'CONTAMINATION_CONFIRMED' if probe['known'] and
        probe['publisher_count'] > 0 else 'PROBE_UNCERTAIN')
    return probe


def _snapshot(
        *, run_command: CommandRunner,
        proc_root: Path = Path('/proc'),
        environment: dict[str, str] | None = None) -> dict[str, object]:
    processes = _read_processes(proc_root)
    owned, ownership_reasons = _owned_processes(processes)

    node_probe = _command(
        ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '1'],
        timeout=8.0, run_command=run_command, environment=environment)
    nodes = sorted({
        line.strip() for line in str(node_probe['stdout']).splitlines()
        if line.strip()
    })

    topic_probes = {
        topic: _topic_publisher_count(
            topic, run_command=run_command, environment=environment)
        for topic in TOPICS_TO_CHECK
    }

    gz_path = shutil.which('gz')
    world_probe = (
        _command(['gz', 'service', '-l'], timeout=8.0,
                 run_command=run_command, environment=environment)
        if gz_path else {
            'command': ['gz', 'service', '-l'],
            'returncode': None,
            'stdout': '',
            'stderr': 'gz executable is not available on PATH',
            'ok': False,
        })
    service_names = str(world_probe['stdout']).splitlines()
    matching_world_services = [
        line.strip() for line in service_names
        if WORLD_SERVICE_PATTERN.search(line.strip())
    ]
    return {
        'processes': processes,
        'owned_processes': owned,
        'ownership_reasons': ownership_reasons,
        'ros_node_probe': node_probe,
        'ros_nodes': nodes,
        'critical_ros_nodes': sorted(
            set(nodes).intersection(CRITICAL_ROS_NODES)),
        'topic_publishers': topic_probes,
        'gazebo_world_probe': world_probe,
        'gazebo_ares_world_services': matching_world_services,
        'duplicate_world_visible': bool(matching_world_services),
    }


def _snapshot_classification(snapshot: dict) -> str:
    """Separate observed contamination from inability to complete probes."""
    contaminated = bool(
        snapshot['owned_processes'] or snapshot['critical_ros_nodes'] or
        snapshot['gazebo_ares_world_services'] or any(
            probe.get('publisher_count', 0) > 0
            for probe in snapshot['topic_publishers'].values()
            if probe.get('known')))
    if contaminated:
        return 'CONTAMINATION_CONFIRMED'
    probes_succeeded = (
        snapshot['ros_node_probe']['ok'] and
        snapshot['gazebo_world_probe']['ok'] and all(
            probe['known'] for probe in snapshot['topic_publishers'].values()))
    return 'CLEAN_CONFIRMED' if probes_succeeded else 'PROBE_UNCERTAIN'


def _write_report(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(path.suffix + '.tmp')
    temporary_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    temporary_path.replace(path)


def _terminate_owned(
        owned: dict[int, dict], reasons: dict[int, str],
        actions: list[dict], *, wait_sec: float = 20.0) -> None:
    if not owned:
        return
    roots = [
        process for process in owned.values()
        if _is_week6_launch(process)
    ]
    signalled: set[int] = set()
    for process in roots:
        pgid = process['pgid']
        current = _read_processes().get(process['pid'])
        if (current is None or
                current['start_time_ticks'] != process['start_time_ticks']):
            continue
        try:
            if pgid == process['pid'] and pgid != os.getpgrp():
                os.killpg(pgid, signal.SIGINT)
                signalled.add(process['pid'])
                actions.append({
                    'action': 'SIGINT_process_group',
                    'pid': process['pid'],
                    'process_group': pgid,
                })
            else:
                os.kill(process['pid'], signal.SIGINT)
                signalled.add(process['pid'])
                actions.append({
                    'action': 'SIGINT_process',
                    'pid': process['pid'],
                    'reason': (
                        'Week 6 launch process; shared process groups '
                        'are never signalled'),
                })
        except ProcessLookupError:
            pass
    for pid, process in owned.items():
        if pid in signalled:
            continue
        current = _read_processes().get(pid)
        if (current is None or
                current['start_time_ticks'] != process['start_time_ticks']):
            continue
        try:
            os.kill(pid, signal.SIGINT)
            actions.append({
                'action': 'SIGINT_process',
                'pid': pid,
                'reason': reasons.get(pid, 'identified Week 6 process'),
            })
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + wait_sec
    while time.monotonic() < deadline:
        current_processes = _read_processes()
        remaining = {
            pid for pid, process in owned.items()
            if pid in current_processes and
            current_processes[pid]['start_time_ticks'] ==
            process['start_time_ticks']
        }
        if not remaining:
            return
        time.sleep(0.2)

    current_processes = _read_processes()
    remaining = {
        pid for pid, process in owned.items()
        if pid in current_processes and
        current_processes[pid]['start_time_ticks'] ==
        process['start_time_ticks']
    }
    for pid in sorted(remaining):
        current = _read_processes().get(pid)
        if (current is None or
                current['start_time_ticks'] != owned[pid]['start_time_ticks']):
            continue
        try:
            os.kill(pid, signal.SIGTERM)
            actions.append({'action': 'SIGTERM_process', 'pid': pid})
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        current_processes = _read_processes()
        remaining = {
            pid for pid, process in owned.items()
            if pid in current_processes and
            current_processes[pid]['start_time_ticks'] ==
            process['start_time_ticks']
        }
        if not remaining:
            return
        time.sleep(0.2)

    current_processes = _read_processes()
    remaining = {
        pid for pid, process in owned.items()
        if pid in current_processes and
        current_processes[pid]['start_time_ticks'] ==
        process['start_time_ticks']
    }
    for pid in sorted(remaining):
        current = _read_processes().get(pid)
        if (current is None or
                current['start_time_ticks'] != owned[pid]['start_time_ticks']):
            continue
        try:
            os.kill(pid, signal.SIGKILL)
            actions.append({'action': 'SIGKILL_process', 'pid': pid})
        except ProcessLookupError:
            pass


def establish_world_exclusivity(
        report_path: Path, *, run_command: CommandRunner = subprocess.run,
        proc_root: Path = Path('/proc'),
        cleanup_timeout_sec: float = 20.0,
        environment: dict[str, str] | None = None) -> dict[str, object]:
    """Clean only attributable Week 6 processes and fail closed otherwise."""
    report: dict[str, object] = {
        'schema_version': 1,
        'required_marker': 'WORLD_EXCLUSIVITY_READY: YES',
        'checks_before_cleanup': {},
        'classification_before_cleanup': 'PROBE_UNCERTAIN',
        'classification_after_cleanup': 'PROBE_UNCERTAIN',
        'detected_stale_processes': [],
        'detected_stale_ros_nodes': [],
        'duplicate_world_status': 'unknown',
        'clock_publisher_count': None,
        'raw_clock_publisher_count': None,
        'cleanup_actions': [],
        'cleanup_observations': [],
        'pre_run_convergence_duration_sec': None,
        'first_clean_snapshot_utc': None,
        'second_clean_snapshot_utc': None,
        'checks_after_cleanup': {},
        'world_exclusivity_ready': False,
        'errors': [],
    }
    convergence_started = time.monotonic()
    before = _snapshot(
        run_command=run_command, proc_root=proc_root,
        environment=environment)
    report['checks_before_cleanup'] = _jsonable_snapshot(before)
    report['classification_before_cleanup'] = _snapshot_classification(before)
    owned = before['owned_processes']
    reasons = before['ownership_reasons']
    report['detected_stale_processes'] = [
        {
            'pid': pid,
            'command': process['command'],
            'ownership_reason': reasons.get(pid),
        }
        for pid, process in sorted(owned.items())
    ]
    report['detected_stale_ros_nodes'] = before['critical_ros_nodes']
    report['duplicate_world_status'] = (
        'visible' if before['duplicate_world_visible'] else
        'not_visible' if before['gazebo_world_probe']['ok'] else 'unknown')

    actions: list[dict] = report['cleanup_actions']
    if owned:
        _terminate_owned(
            owned, reasons, actions, wait_sec=cleanup_timeout_sec)

    cleanup_observations: list[dict[str, object]] = []
    report['cleanup_observations'] = cleanup_observations
    deadline = time.monotonic() + cleanup_timeout_sec
    consecutive_clean = 0
    after = before
    while True:
        after = _snapshot(
            run_command=run_command, proc_root=proc_root,
            environment=environment)
        observation = {
            'observed_utc': utc_now(),
            'owned_process_count': len(after['owned_processes']),
            'critical_ros_nodes': list(after['critical_ros_nodes']),
            'duplicate_world_visible': after['duplicate_world_visible'],
            'ros_node_probe_ok': after['ros_node_probe']['ok'],
            'gazebo_world_probe_ok': after['gazebo_world_probe']['ok'],
            'clock_publisher_count': _publisher_count(after, '/clock'),
            'raw_clock_publisher_count': _publisher_count(
                after, '/ares/week6/raw_clock'),
            'classification': _snapshot_classification(after),
        }
        cleanup_observations.append(observation)
        clean = (
            not after['owned_processes'] and
            not after['critical_ros_nodes'] and
            not after['duplicate_world_visible'] and
            after['ros_node_probe']['ok'] and
            after['gazebo_world_probe']['ok'] and
            _publisher_count(after, '/clock') == 0 and
            _publisher_count(after, '/ares/week6/raw_clock') == 0 and
            all(
                probe['known']
                for probe in after['topic_publishers'].values())
        )
        consecutive_clean = consecutive_clean + 1 if clean else 0
        if consecutive_clean == 1:
            report['first_clean_snapshot_utc'] = observation['observed_utc']
        elif consecutive_clean == 2:
            report['second_clean_snapshot_utc'] = observation['observed_utc']
        else:
            report['first_clean_snapshot_utc'] = None
            report['second_clean_snapshot_utc'] = None
        if consecutive_clean >= 2 or time.monotonic() >= deadline:
            break
        time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))

    report['pre_run_convergence_duration_sec'] = (
        time.monotonic() - convergence_started)
    report['checks_after_cleanup'] = _jsonable_snapshot(after)
    report['classification_after_cleanup'] = _snapshot_classification(after)
    report['clock_publisher_count'] = _publisher_count(
        after, '/clock')
    report['raw_clock_publisher_count'] = _publisher_count(
        after, '/ares/week6/raw_clock')
    report['duplicate_world_status'] = (
        'visible' if after['duplicate_world_visible'] else
        'not_visible' if after['gazebo_world_probe']['ok'] else 'unknown')

    for check_name, snapshot in (('before', before), ('after', after)):
        if not snapshot['ros_node_probe']['ok']:
            report['errors'].append(
                f'ROS node graph probe failed {check_name} cleanup')
        if not snapshot['gazebo_world_probe']['ok']:
            report['errors'].append(
                f'Gazebo world probe failed {check_name} cleanup')
        for topic, probe in snapshot['topic_publishers'].items():
            if not probe['known']:
                report['errors'].append(
                    f'publisher count for {topic} unknown {check_name} cleanup')

    stale_after = after['owned_processes']
    if stale_after:
        report['errors'].append(
            'attributable Week 6 processes remain after bounded cleanup')
    if after['critical_ros_nodes']:
        report['errors'].append(
            'critical ROS nodes remain; unrelated processes were not killed')
    if after['duplicate_world_visible']:
        report['errors'].append(
            'Gazebo reports an existing world named ares_world')
    if _publisher_count(after, '/clock') != 0:
        report['errors'].append('a /clock publisher remains')
    if _publisher_count(after, '/ares/week6/raw_clock') != 0:
        report['errors'].append('a Week 6 raw-clock publisher remains')

    report['world_exclusivity_ready'] = not report['errors']
    _write_report(report_path, report)
    if report['world_exclusivity_ready']:
        print('WORLD_EXCLUSIVITY_READY: YES', flush=True)
    else:
        print('WORLD_EXCLUSIVITY_READY: NO', flush=True)
        raise RuntimeError(
            'Week 6 runtime isolation failed; see '
            f'{report_path}: {", ".join(report["errors"])}')
    return report


def _publisher_count(snapshot: dict, topic: str) -> int | None:
    return snapshot['topic_publishers'][topic].get('publisher_count')


def _jsonable_snapshot(snapshot: dict) -> dict[str, object]:
    return {
        'owned_processes': [
            {
                'pid': pid,
                'command': process['command'],
                'ownership_reason': snapshot['ownership_reasons'].get(pid),
            }
            for pid, process in sorted(snapshot['owned_processes'].items())
        ],
        'critical_ros_nodes': snapshot['critical_ros_nodes'],
        'ros_node_probe': {
            key: value for key, value in snapshot['ros_node_probe'].items()
        } | {'nodes': snapshot['ros_nodes']},
        'topic_publishers': {
            topic: dict(probe)
            for topic, probe in snapshot['topic_publishers'].items()
        },
        'gazebo_world_probe': dict(snapshot['gazebo_world_probe']),
        'gazebo_ares_world_services': snapshot[
            'gazebo_ares_world_services'],
        'duplicate_world_visible': snapshot['duplicate_world_visible'],
    }


if __name__ == '__main__':
    raise SystemExit(
        'Call establish_world_exclusivity from the Week 6 runner.')
