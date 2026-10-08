#!/usr/bin/env python3
"""Launch and record one isolated ARES Week 6 navigation run."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

from ares_reliability.navigation_metrics import count_controller_rate_misses

from week6_runtime_isolation import (
    RunProcessTracker,
    establish_world_exclusivity,
    post_run_teardown,
    utc_now,
)


WORKSPACE = Path(__file__).resolve().parents[1]
WEEK6_RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
WEEK6_AUTOMATIC_DISCOVERY_RANGE = 'LOCALHOST'
WEEK6_UNIQUE_NETWORK_FLOWS_SETTING = 'DISABLED'
WEEK6_UNIQUE_NETWORK_FLOWS_ENV = (
    'RMW_FASTRTPS_ROS_DISCOVERY_INFO_UNIQUE_NETWORK_FLOWS')
WEEK6_CLOCK_RATE_HZ = 100.0
WEEK6_RAW_CLOCK_TOPIC = '/ares/week6/raw_clock'
COLLISION_DIAGNOSTIC_BAG_TOPICS = (
    '/lookahead_collision_arc',
    '/local_costmap/costmap_raw',
    '/ares/scan',
    '/cmd_vel',
    '/ares/cmd_vel',
    '/odometry/trust_fused',
    '/plan',
    '/transformed_global_plan',
    '/local_plan',
    '/global_plan',
    '/map',
    '/tf',
    '/tf_static',
    '/clock',
    '/rosout',
    '/follow_path/_action/status',
    '/navigate_to_pose/_action/status',
    '/navigate_through_poses/_action/status',
)

IMPLEMENTATION_FILES = (
    'scripts/run_week6_navigation.py',
    'scripts/week6_runtime_isolation.py',
    'src/ares_reliability/ares_reliability/week6_navigation_mission.py',
    'src/ares_reliability/ares_reliability/week6_nav2_startup_barrier.py',
    'src/ares_reliability/ares_reliability/'
    'week6_nav2_lifecycle_orchestrator.py',
    'src/ares_reliability/ares_reliability/week6_collision_diagnostics.py',
    'src/ares_reliability/ares_reliability/navigation_metrics.py',
    'src/ares_reliability/ares_reliability/consistency_monitor.py',
    'src/ares_reliability/ares_reliability/gnss_trusted_proxy.py',
    'src/ares_reliability/ares_reliability/recovery_manager.py',
    'src/ares_reliability/ares_reliability/gnss_fault_injector.py',
    'src/ares_reliability/config/week4_trust.yaml',
    'src/ares_reliability/config/week5_fault_matrix.yaml',
    'src/ares_reliability/config/week6_mission.yaml',
    'src/ares_reliability/config/nav2_week6_navigation.yaml',
    'src/ares_reliability/config/navsat_week6_navigation.yaml',
    'src/ares_reliability/config/ekf_week6_navigation.yaml',
    'src/ares_reliability/package.xml',
    'src/ares_reliability/setup.py',
    'src/ares_reliability/launch/week6_navigation.launch.py',
    'src/ares_reliability/config/nav2_week6_navigation.yaml',
    'scripts/run_week6_campaign.py',
    'scripts/analyze_week6_campaign.py',
    'src/ares_simulation/launch/ares_baseline.launch.py',
    'src/ares_simulation/CMakeLists.txt',
    'src/ares_simulation/package.xml',
    'src/ares_simulation/include/ares_simulation/week6_clock_boundary.hpp',
    'src/ares_simulation/src/week6_clock_boundary.cpp',
    'src/ares_simulation/maps/ares_warehouse.yaml',
    'src/ares_simulation/maps/ares_warehouse.pgm',
    'src/ares_simulation/models/ares_jackal/model.sdf',
    'src/ares_jackal_description/urdf/jackal.urdf.xacro',
)


def implementation_hashes() -> dict[str, str]:
    """Return immutable fingerprints for files that define one Week 6 run."""
    return {
        relative: hashlib.sha256(
            (WORKSPACE / relative).read_bytes()).hexdigest()
        for relative in IMPLEMENTATION_FILES
    }


def week6_runtime_environment(
        base_environment: dict[str, str],
        run_directory: Path,
        run_id: str) -> dict[str, str]:
    """Apply only supported Week 6 runtime settings to child processes."""
    environment = base_environment.copy()
    environment.pop(WEEK6_UNIQUE_NETWORK_FLOWS_ENV, None)
    environment.update({
        'RMW_IMPLEMENTATION': WEEK6_RMW_IMPLEMENTATION,
        'ROS_AUTOMATIC_DISCOVERY_RANGE':
            WEEK6_AUTOMATIC_DISCOVERY_RANGE,
        'GALLIUM_DRIVER': 'llvmpipe',
        'ROS_LOG_DIR': str(run_directory / 'ros_logs'),
        'ARES_WEEK6_RUN_ID': run_id,
        'ARES_WEEK6_LIFECYCLE_EVIDENCE_PATH': str(
            run_directory / 'lifecycle_transition_evidence.json'),
    })
    return environment


def stop_collision_monitor(
        process: subprocess.Popen | None,
        log_handle: object | None) -> dict[str, object]:
    """Stop the owned telemetry node and report any escalation explicitly."""
    actions: list[str] = []
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        actions.append('SIGINT')
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            actions.append('SIGTERM')
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                actions.append('SIGKILL')
                process.wait(timeout=3.0)
    if log_handle is not None:
        log_handle.close()
    return {
        'exit_code': None if process is None else process.returncode,
        'cleanup_signals': actions,
        'stopped': process is None or process.poll() is not None,
    }


def controller_thread_scheduling_snapshot() -> dict[str, object]:
    """Inspect controller thread policy while the live launch is available."""
    evidence: dict[str, object] = {
        'process_found': False,
        'threads': [],
        'errors': [],
    }
    proc_root = Path('/proc')
    for process_path in proc_root.iterdir():
        if not process_path.name.isdigit():
            continue
        try:
            command = (process_path / 'cmdline').read_bytes().replace(
                b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if 'nav2_controller/controller_server' not in command:
            continue
        evidence['process_found'] = True
        evidence['pid'] = int(process_path.name)
        task_root = process_path / 'task'
        for task_path in task_root.iterdir():
            if not task_path.name.isdigit():
                continue
            tid = int(task_path.name)
            try:
                policy = os.sched_getscheduler(tid)
                policy_name = {
                    os.SCHED_OTHER: 'SCHED_OTHER',
                    os.SCHED_FIFO: 'SCHED_FIFO',
                    os.SCHED_RR: 'SCHED_RR',
                }.get(policy, str(policy))
                evidence['threads'].append({
                    'tid': tid,
                    'policy': policy_name,
                    'priority': os.sched_getparam(tid).sched_priority,
                })
            except OSError as error:
                evidence['errors'].append({
                    'tid': tid,
                    'error': str(error),
                })
        break
    return evidence


def add_runtime_quality(result_path: Path, launch_log_path: Path) -> None:
    """Attach non-acceptance controller timing evidence to one result."""
    if not result_path.exists():
        return
    result = json.loads(result_path.read_text(encoding='utf-8'))
    launch_log = launch_log_path.read_text(
        encoding='utf-8', errors='replace')
    miss_count = count_controller_rate_misses(launch_log)
    runtime = result.setdefault('runtime_quality', {})
    runtime['controller_rate_miss_count'] = miss_count
    result['controller_rate_miss_count'] = miss_count
    startup_diagnostics = inspect_startup_log(launch_log)
    barrier = startup_diagnostics.get('readiness_barrier')
    if isinstance(barrier, dict) and not barrier.get('ready', False):
        result['startup_diagnostics'] = startup_diagnostics
        result['failure_class'] = 'ORCHESTRATION_RUNTIME'
        result['scientific_mission_failure'] = False
    result_path.write_text(
        json.dumps(result, indent=2, allow_nan=False) + '\n',
        encoding='utf-8')


def inspect_startup_log(log_text: str) -> dict[str, object]:
    """Evaluate endpoint, lifecycle, and BT construction evidence in a log."""
    readiness: dict[str, object] | None = None
    for line in log_text.splitlines():
        match = re.search(r'WEEK6_STARTUP_READINESS_JSON=(\{.*\})', line)
        if match is not None:
            readiness = json.loads(match.group(1))
    required_nodes = (
        'map_server', 'planner_server', 'controller_server',
        'behavior_server', 'bt_navigator',
    )
    active_confirmations = {
        name: f'Server {name} connected with bond' in log_text
        for name in required_nodes
    }
    required_trees = (
        'NavigateToPoseWReplanningAndRecovery',
        'NavigateThroughPosesWReplanningAndRecovery',
    )
    created_trees = {
        tree: f'Created BT from ID: {tree}' in log_text
        for tree in required_trees
    }
    endpoints_ready = bool(readiness and readiness.get('ready'))
    missing_actions = [
        name for name, ready in
        (readiness or {}).get('actions', {}).items() if not ready
    ]
    missing_services = [
        name for name, ready in
        (readiness or {}).get('services', {}).items() if not ready
    ]
    missing_lifecycle_nodes = [
        name for name, state in
        (readiness or {}).get('lifecycle_states', {}).items()
        if name != 'bt_navigator' and state != 'active'
    ]
    failure_classes: list[str] = []
    if any(name in missing_actions for name in (
            '/compute_path_to_pose', '/compute_path_through_poses')):
        failure_classes.append('PLANNER_ACTION_DISCOVERY_FAILURE')
    if '/is_path_valid' in missing_services:
        failure_classes.append('PLANNER_SERVICE_DISCOVERY_FAILURE')
    if any(name in missing_services for name in (
            '/global_costmap/clear_entirely_global_costmap',
            '/local_costmap/clear_entirely_local_costmap')):
        failure_classes.append('COSTMAP_SERVICE_DISCOVERY_FAILURE')
    if '/follow_path' in missing_actions:
        failure_classes.append('CONTROLLER_ACTION_DISCOVERY_FAILURE')
    if any(name in missing_actions for name in (
            '/spin', '/backup', '/drive_on_heading', '/wait')):
        failure_classes.append('BEHAVIOR_ACTION_DISCOVERY_FAILURE')
    if missing_lifecycle_nodes:
        failure_classes.append('LIFECYCLE_ACTIVATION_FAILURE')
    states = (readiness or {}).get('lifecycle_states', {})
    missing_actions_from_provider = bool(
        {'/compute_path_to_pose', '/compute_path_through_poses',
         '/is_path_valid',
         '/global_costmap/clear_entirely_global_costmap',
         '/local_costmap/clear_entirely_local_costmap'} &
        (set(missing_actions) | set(missing_services)))
    if (states.get('bt_navigator') not in (None, 'active') and
            not missing_actions_from_provider and
            not failure_classes):
        failure_classes.append('BT_NAVIGATOR_ACTIVATION_FAILURE')
    return {
        'passed': (
            endpoints_ready and
            all(active_confirmations.values()) and
            all(created_trees.values())),
        'readiness_barrier': readiness,
        'lifecycle_active_confirmations': active_confirmations,
        'bt_trees_created': created_trees,
        'missing_lifecycle_confirmations': [
            name for name, confirmed in active_confirmations.items()
            if not confirmed
        ],
        'missing_bt_trees': [
            tree for tree, created in created_trees.items() if not created
        ],
        'failure_classes': failure_classes,
        'failure_is_orchestration_runtime': bool(failure_classes),
    }


def run_startup_only(launch: subprocess.Popen,
                     launch_log_path: Path,
                     timeout_wall_sec: float) -> dict[str, object]:
    """Wait for a single fresh launch to confirm Week 6 startup readiness."""
    deadline = time.monotonic() + timeout_wall_sec
    latest = inspect_startup_log('')
    while time.monotonic() < deadline:
        if launch_log_path.exists():
            latest = inspect_startup_log(launch_log_path.read_text(
                encoding='utf-8', errors='replace'))
        if latest['passed']:
            latest['launch_exit_code_before_shutdown'] = launch.poll()
            latest['startup_timeout_wall_sec'] = timeout_wall_sec
            latest['validation_elapsed_wall_sec'] = (
                timeout_wall_sec - max(0.0, deadline - time.monotonic()))
            return latest
        if launch.poll() is not None:
            break
        time.sleep(0.1)

    if launch_log_path.exists():
        latest = inspect_startup_log(launch_log_path.read_text(
            encoding='utf-8', errors='replace'))
    latest['passed'] = False
    latest['launch_exit_code_before_shutdown'] = launch.poll()
    latest['startup_timeout_wall_sec'] = timeout_wall_sec
    latest['validation_elapsed_wall_sec'] = (
        timeout_wall_sec - max(0.0, deadline - time.monotonic()))
    if launch.poll() is None:
        latest['failure'] = (
            'startup validation did not observe every required lifecycle '
            'confirmation and both BTs before the wall-time deadline')
    else:
        latest['failure'] = (
            'launch exited before startup validation completed')
    return latest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation-mode',
                        choices=('baseline', 'unprotected', 'protected'),
                        required=True)
    parser.add_argument('--scenario',
                        choices=('healthy', 'gnss_step_5m'), required=True)
    parser.add_argument('--seed', type=int, default=2506)
    parser.add_argument('--results-dir', default='results/week6')
    parser.add_argument('--run-directory')
    parser.add_argument('--startup-only', action='store_true')
    parser.add_argument('--startup-timeout-wall-sec', type=float, default=90.0)
    parser.add_argument('--launch-ready-wall-sec', type=float, default=15.0)
    parser.add_argument('--collision-diagnostics', action='store_true')
    args = parser.parse_args()
    if args.navigation_mode == 'baseline' and args.scenario != 'healthy':
        parser.error('baseline mode currently supports only healthy missions')
    if args.startup_only and args.collision_diagnostics:
        parser.error(
            '--collision-diagnostics is only for goal-sending diagnostic runs')
    if args.seed < 0:
        parser.error('--seed must be non-negative')

    run_name = f'{args.scenario}_{args.navigation_mode}_s{args.seed}'
    category = 'healthy' if args.scenario == 'healthy' else args.scenario
    run_directory = (
        WORKSPACE / args.run_directory
        if args.run_directory else
        WORKSPACE / args.results_dir / category / run_name)
    run_directory.mkdir(parents=True, exist_ok=True)
    result_path = run_directory / 'result.json'
    startup_result_path = run_directory / 'endpoint_readiness.json'
    run_metadata_path = run_directory / 'run_metadata.json'
    runtime_isolation_path = run_directory / 'runtime_isolation.json'
    teardown_report_path = run_directory / 'post_run_teardown.json'
    collision_events_path = run_directory / 'collision_events.json'
    if (result_path.exists() or
            (args.collision_diagnostics and collision_events_path.exists()) or
            (args.startup_only and startup_result_path.exists()) or
            runtime_isolation_path.exists() or
            teardown_report_path.exists() or
            ((args.startup_only or args.run_directory) and
             run_metadata_path.exists())):
        raise FileExistsError(
            f'output already exists in {run_directory}; '
            'preserve immutable run evidence')
    if args.startup_only and args.startup_timeout_wall_sec <= 0.0:
        parser.error('--startup-timeout-wall-sec must be positive')
    environment = week6_runtime_environment(
        os.environ, run_directory, str(uuid.uuid4()))
    Path(environment['ROS_LOG_DIR']).mkdir(parents=True, exist_ok=True)
    matrix_scenario = (
        'healthy_fusion' if args.scenario == 'healthy'
        else args.scenario)
    launch_command = [
        'ros2', 'launch', 'ares_reliability',
        'week6_navigation.launch.py',
        f'navigation_mode:={args.navigation_mode}',
        f'scenario:={matrix_scenario}',
        f'seed:={args.seed}', 'rviz:=false',
    ]
    mission_command = [
        'ros2', 'run', 'ares_reliability', 'week6_navigation_mission',
        '--navigation-mode', args.navigation_mode,
        '--scenario', args.scenario,
        '--seed', str(args.seed),
        '--output', str(result_path),
    ]
    metadata = {
        'run_name': run_name,
        'rmw_implementation': WEEK6_RMW_IMPLEMENTATION,
        'clock_boundary': {
            'raw_topic': WEEK6_RAW_CLOCK_TOPIC,
            'consumer_topic': '/clock',
            'maximum_rate_hz': WEEK6_CLOCK_RATE_HZ,
        },
        'launch_command': launch_command,
        'mission_command': mission_command,
        'environment': {
            name: environment[name] for name in
            ('RMW_IMPLEMENTATION', 'GALLIUM_DRIVER', 'ROS_LOG_DIR',
             'ARES_WEEK6_RUN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
             'ARES_WEEK6_LIFECYCLE_EVIDENCE_PATH')},
        'fastdds_discovery_configuration': {
            'ROS_AUTOMATIC_DISCOVERY_RANGE': {
                'requested': WEEK6_AUTOMATIC_DISCOVERY_RANGE,
                'applied': environment[
                    'ROS_AUTOMATIC_DISCOVERY_RANGE'],
                'supported_by_installed_rcl': True,
            },
            'collision_diagnostics_enabled': args.collision_diagnostics,
            WEEK6_UNIQUE_NETWORK_FLOWS_ENV: {
                'requested': WEEK6_UNIQUE_NETWORK_FLOWS_SETTING,
                'applied': None,
                'supported_by_installed_rmw_fastrtps': False,
                'reason': (
                    'The installed rmw_fastrtps shared library exposes '
                    'fastdds.unique_network_flows as an XML profile '
                    'property but does not read the requested environment '
                    'variable. ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST is '
                    'the supported local-only discovery control.'),
            },
        },
        'runtime_isolation_report': str(
            runtime_isolation_path.relative_to(WORKSPACE)),
        'implementation_sha256': implementation_hashes(),
    }
    run_metadata_path.write_text(
        json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    launch_log_path = run_directory / 'launch.log'
    mission_log_path = run_directory / 'mission.log'
    try:
        establish_world_exclusivity(
            runtime_isolation_path, environment=environment)
    except RuntimeError as error:
        launch_log_path.write_text('', encoding='utf-8')
        mission_log_path.write_text('', encoding='utf-8')
        result_path.write_text(
            json.dumps({
                'schema_version': 1,
                'run_name': run_name,
                'navigation_mode': args.navigation_mode,
                'scenario': args.scenario,
                'seed': args.seed,
                'mission_completed': False,
                'mission_failure_reasons': [
                    f'runner_error: world isolation failed: {error}',
                ],
                'failure_class': 'ORCHESTRATION_RUNTIME',
                'scientific_mission_failure': False,
                'startup_diagnostics': {
                    'passed': False,
                    'failure_phase': 'runtime_isolation',
                },
            }, indent=2) + '\n',
            encoding='utf-8')
        add_runtime_quality(result_path, launch_log_path)
        teardown = post_run_teardown(
            report_path=teardown_report_path,
            run_name=run_name,
            result_path=result_path,
            launch=None,
            tracked_processes=[],
            environment=environment,
        )
        print(result_path)
        raise SystemExit(1)
    launch_log = launch_log_path.open(
        'w', encoding='utf-8')
    mission_log = mission_log_path.open(
        'w', encoding='utf-8')
    launch = subprocess.Popen(
        launch_command, cwd=WORKSPACE, env=environment,
        stdout=launch_log, stderr=subprocess.STDOUT,
        start_new_session=True, text=True)
    tracker = RunProcessTracker(launch.pid, environment['ARES_WEEK6_RUN_ID'])
    collision_monitor: subprocess.Popen | None = None
    collision_monitor_log = None
    diagnostic_bag: subprocess.Popen | None = None
    diagnostic_bag_log = None
    if args.collision_diagnostics:
        collision_monitor_log = (run_directory / 'collision_monitor.log').open(
            'w', encoding='utf-8')
        collision_monitor = subprocess.Popen(
            [
                sys.executable,
                str(WORKSPACE / 'src/ares_reliability/ares_reliability/'
                    'week6_collision_diagnostics.py'),
                '--output', str(collision_events_path),
                '--run-name', run_name,
                '--result', str(result_path),
                '--launch-log', str(launch_log_path),
            ],
            cwd=WORKSPACE, env=environment,
            stdout=collision_monitor_log, stderr=subprocess.STDOUT,
            start_new_session=True, text=True)
        diagnostic_bag_log = (run_directory / 'diagnostic_bag.log').open(
            'w', encoding='utf-8')
        diagnostic_bag = subprocess.Popen(
            [
                'ros2', 'bag', 'record',
                '--storage', 'sqlite3',
                '--output', str(run_directory / 'diagnostic_bag'),
                '--include-hidden-topics',
                '--include-unpublished-topics',
                '--disable-keyboard-controls',
                '--max-cache-size', '104857600',
                '--topics', *COLLISION_DIAGNOSTIC_BAG_TOPICS,
            ],
            cwd=WORKSPACE, env=environment,
            stdout=diagnostic_bag_log, stderr=subprocess.STDOUT,
            start_new_session=True, text=True)
    launch_started_utc = utc_now()
    if args.startup_only:
        try:
            startup_result = run_startup_only(
                launch, launch_log_path, args.startup_timeout_wall_sec)
            startup_result['controller_thread_scheduling'] = (
                controller_thread_scheduling_snapshot())
        finally:
            mission_log.close()
        tracked_processes = tracker.finish()
        teardown = post_run_teardown(
            report_path=teardown_report_path,
            run_name=run_name,
            result_path=None,
            launch=launch,
            tracked_processes=tracked_processes,
            environment=environment,
        )
        launch_log.close()
        startup_result['run_name'] = run_name
        startup_result['seed'] = args.seed
        startup_result['navigation_mode'] = args.navigation_mode
        startup_result['scenario'] = args.scenario
        startup_result['launch_started_utc'] = launch_started_utc
        startup_result['post_run_teardown_ready'] = (
            teardown['teardown_ready'])
        startup_result_path.write_text(
            json.dumps(startup_result, indent=2) + '\n',
            encoding='utf-8')
        print(startup_result_path)
        raise SystemExit(
            0 if startup_result['passed'] and
            teardown['teardown_ready'] else 1)

    return_code = 1
    launch_failure: str | None = None
    try:
        deadline = time.monotonic() + args.launch_ready_wall_sec
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise RuntimeError(
                    f'launch exited early with code {launch.returncode}')
            time.sleep(0.25)
        completed = subprocess.run(
            mission_command, cwd=WORKSPACE, env=environment,
            stdout=mission_log, stderr=subprocess.STDOUT,
            text=True, check=False)
        return_code = completed.returncode
    except RuntimeError as error:
        launch_failure = str(error)
    finally:
        launch_log.flush()
        mission_log.close()
    if launch_failure is not None and not result_path.exists():
        launch_text = launch_log_path.read_text(
            encoding='utf-8', errors='replace')
        result_path.write_text(
            json.dumps({
                'schema_version': 1,
                'run_name': run_name,
                'navigation_mode': args.navigation_mode,
                'scenario': args.scenario,
                'seed': args.seed,
                'mission_completed': False,
                'mission_failure_reasons': [
                    f'runner_error: {launch_failure}',
                ],
                'failure_class': 'ORCHESTRATION_RUNTIME',
                'scientific_mission_failure': False,
                'startup_diagnostics': inspect_startup_log(launch_text),
            }, indent=2) + '\n',
            encoding='utf-8')
        return_code = 1
    if args.collision_diagnostics:
        monitor_status = stop_collision_monitor(
            collision_monitor, collision_monitor_log)
        monitor_status['report_path'] = str(
            collision_events_path.relative_to(WORKSPACE))
        monitor_status['capture_passed'] = (
            monitor_status['stopped'] and
            monitor_status['exit_code'] == 0 and
            collision_events_path.is_file())
        (run_directory / 'collision_diagnostics_status.json').write_text(
            json.dumps(monitor_status, indent=2) + '\n',
            encoding='utf-8')
        if not monitor_status['capture_passed']:
            return_code = 1
        bag_status = stop_collision_monitor(
            diagnostic_bag, diagnostic_bag_log)
        bag_metadata_path = (
            run_directory / 'diagnostic_bag' / 'metadata.yaml')
        bag_status['bag_directory'] = str(
            (run_directory / 'diagnostic_bag').relative_to(WORKSPACE))
        bag_status['metadata_path'] = str(
            bag_metadata_path.relative_to(WORKSPACE))
        bag_status['capture_passed'] = bool(
            bag_status['stopped'] and
            bag_status['exit_code'] == 0 and
            bag_metadata_path.is_file())
        (run_directory / 'diagnostic_bag_status.json').write_text(
            json.dumps(bag_status, indent=2) + '\n',
            encoding='utf-8')
        if not bag_status['capture_passed']:
            return_code = 1
    tracked_processes = tracker.finish()
    teardown = post_run_teardown(
        report_path=teardown_report_path,
        run_name=run_name,
        result_path=result_path,
        launch=launch,
        tracked_processes=tracked_processes,
        environment=environment,
    )
    launch_log.close()
    add_runtime_quality(result_path, launch_log_path)
    print(result_path)
    raise SystemExit(
        return_code if teardown['teardown_ready'] else 1)


if __name__ == '__main__':
    main()
