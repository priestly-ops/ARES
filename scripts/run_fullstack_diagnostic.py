#!/usr/bin/env python3
"""Instrument one fresh full-stack health preflight without changing ARES."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import sys
import time


parser = argparse.ArgumentParser()
parser.add_argument('--name', required=True)
parser.add_argument('--runs', type=int, default=1,
                    help='Number of fresh trials to run under one captured host configuration')
parser.add_argument('--cpus', help='CPU list for the trial process tree, e.g. 2-11')
parser.add_argument('--isolate-agent', action='store_true',
                    help='Temporarily pin the headroom proxy to CPUs 0-1')
parser.add_argument('--mission', action='store_true',
                    help='Run the fixed three-goal mission instead of health-only')
parser.add_argument('--gallium-driver', choices=('d3d12', 'llvmpipe'),
                    help='Override only the Mesa Gallium driver for this trial')
parser.add_argument('--fastdds-builtin-transports', choices=('DEFAULT', 'UDPv4'),
                    help='Select a Fast DDS built-in transport set for this trial')
parser.add_argument('--capture-core', action='store_true',
                    help='Enable core dumps for the trial launch tree in its evidence directory')
parser.add_argument('--bridge-keep-rmw-loaded', action='store_true',
                    help='Keep the selected Fast DDS RMW mapped in parameter_bridge through exit')
args = parser.parse_args()
if args.runs < 1:
    parser.error('--runs must be positive')

root = Path(__file__).resolve().parents[1]
out = root / 'results' / 'week1' / 'diagnostics' / args.name
out.mkdir(parents=True, exist_ok=False)
trial_rel = Path('results') / 'week1' / 'diagnostics' / args.name / 'trial'


def sourced_runtime_environment():
    command = (
        'source /opt/ros/lyrical/setup.bash && '
        f'source {root / "install/setup.bash"} && env -0'
    )
    raw = subprocess.check_output(['bash', '-lc', command])
    environment = {}
    for item in raw.split(b'\0'):
        if b'=' in item:
            key, value = item.split(b'=', 1)
            environment[key.decode(errors='replace')] = value.decode(errors='replace')
    return environment


runtime_environment = sourced_runtime_environment()
if args.gallium_driver:
    runtime_environment['GALLIUM_DRIVER'] = args.gallium_driver
if args.fastdds_builtin_transports:
    runtime_environment['FASTDDS_BUILTIN_TRANSPORTS'] = args.fastdds_builtin_transports


def command_output(command, env=None):
    result = subprocess.run(
        command, cwd=root, text=True, capture_output=True, env=env
    )
    return {'command': command, 'returncode': result.returncode,
            'stdout': result.stdout, 'stderr': result.stderr}


def read(path):
    try:
        return Path(path).read_text(errors='replace')
    except (OSError, PermissionError):
        return None


def relevant_environment(pid=None):
    allowed = {
        'DISPLAY', 'WAYLAND_DISPLAY', 'XDG_RUNTIME_DIR', 'PULSE_SERVER',
        'ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'GZ_PARTITION',
        'GZ_SIM_RESOURCE_PATH', 'LIBGL_ALWAYS_SOFTWARE',
        'MESA_LOADER_DRIVER_OVERRIDE', 'GALLIUM_DRIVER',
        'FASTDDS_BUILTIN_TRANSPORTS',
        '__GLX_VENDOR_LIBRARY_NAME', 'LD_LIBRARY_PATH', 'PATH',
    }
    if pid is None:
        source = os.environ
    else:
        raw = Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
        source = {}
        for item in raw:
            if b'=' in item:
                key, value = item.split(b'=', 1)
                source[key.decode(errors='replace')] = value.decode(errors='replace')
    return {key: source[key] for key in sorted(allowed) if key in source}


def matching_processes():
    patterns = (
        'gz sim', 'gz-sim', 'parameter_bridge', 'robot_state_publisher',
        'ekf_node', 'map_server', 'amcl', 'planner_server',
        'controller_server', 'behavior_server', 'bt_navigator',
        'lifecycle_manager', 'cmd_vel_adapter', 'ros2 bag', 'health_check',
        'headroom.cli proxy', 'headroom wrap codex', '/codex',
    )
    found = []
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            cmdline = (item / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if cmdline and any(pattern in cmdline for pattern in patterns):
                status = read(item / 'status') or ''
                fields = {}
                for line in status.splitlines():
                    if ':' in line:
                        key, value = line.split(':', 1)
                        if key in ('Name', 'State', 'Threads', 'VmRSS', 'VmSize',
                                   'Cpus_allowed_list', 'voluntary_ctxt_switches',
                                   'nonvoluntary_ctxt_switches'):
                            fields[key] = value.strip()
                found.append({'pid': int(item.name), 'cmdline': cmdline,
                              'status': fields,
                              'cpu_ticks': {
                                  'utime': int((item / 'stat').read_text().split()[13]),
                                  'stime': int((item / 'stat').read_text().split()[14]),
                              },
                              'environment': relevant_environment(int(item.name))})
        except (OSError, PermissionError, ProcessLookupError):
            continue
    return found


def pressure_snapshot():
    return {name: read(f'/proc/pressure/{name}') for name in ('cpu', 'memory', 'io')}


def frequency_snapshot():
    values = {}
    for path in sorted(Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq')):
        values[path.parent.parent.name] = read(path)
    return values


def thermal_snapshot():
    values = {}
    for path in sorted(Path('/sys/class/thermal').glob('thermal_zone*/temp')):
        values[path.parent.name] = read(path)
    return values


metadata = {
    'name': args.name,
    'started_wall': time.time(),
    'cpus': args.cpus,
    'isolate_agent': args.isolate_agent,
    'gallium_driver_override': args.gallium_driver,
    'fastdds_builtin_transports': args.fastdds_builtin_transports,
    'capture_core': args.capture_core,
    'bridge_keep_rmw_loaded': args.bridge_keep_rmw_loaded,
    'wrapper_affinity': sorted(os.sched_getaffinity(0)),
    'uname': command_output(['uname', '-a']),
    'lscpu': command_output(['lscpu']),
    'gazebo': command_output(['gz', 'sim', '--versions']),
    'renderer': command_output(['glxinfo', '-B'], env=runtime_environment),
    'packages': command_output([
        'dpkg-query', '-W', '-f=${Package} ${Version}\n',
        'gz-sim10-cli', 'gz-sim10-server', 'libgl1-mesa-dri',
        'mesa-libgallium', 'mesa-utils']),
    'environment': relevant_environment(),
    'runtime_environment': {
        key: runtime_environment[key]
        for key in sorted(runtime_environment)
        if key in {
            'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'DISPLAY',
            'FASTDDS_BUILTIN_TRANSPORTS', 'GALLIUM_DRIVER',
            'GZ_SIM_RESOURCE_PATH', 'LD_LIBRARY_PATH',
            'PATH', 'ROS_DISTRO', 'WAYLAND_DISPLAY', 'XDG_RUNTIME_DIR'
        }
    },
    'wslconfig': read('/mnt/c/Users/rajpr/.wslconfig'),
    'cgroup_cpu_max': read('/sys/fs/cgroup/cpu.max'),
    'meminfo_before': read('/proc/meminfo'),
    'pressure_before': pressure_snapshot(),
    'frequency_before': frequency_snapshot(),
    'thermal_before': thermal_snapshot(),
    'processes_before': matching_processes(),
    'dmesg_before': command_output(['dmesg', '--ctime']),
}
for path in (
    root / 'src/ares_simulation/models/ares_jackal/model.sdf',
    root / 'src/ares_benchmark/config/mission.yaml',
    root / 'src/ares_simulation/config/bridge_baseline.yaml',
    root / 'src/ares_simulation/launch/ares_baseline.launch.py',
    root / 'scripts/run_healthy_trials.py',
):
    metadata.setdefault('file_sha256', {})[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
(out / 'host_before.json').write_text(json.dumps(metadata, indent=2) + '\n')

agent_affinities = {}
if args.isolate_agent:
    for process in matching_processes():
        if 'headroom.cli proxy --port' not in process['cmdline']:
            continue
        pid = process['pid']
        try:
            original = sorted(os.sched_getaffinity(pid))
            os.sched_setaffinity(pid, {0, 1})
            agent_affinities[pid] = original
        except (OSError, PermissionError, ProcessLookupError) as exc:
            agent_affinities[pid] = {'error': repr(exc)}

command = [sys.executable, str(root / 'scripts/run_healthy_trials.py'),
           '--runs', str(args.runs), '--output', str(trial_rel)]
if not args.mission:
    command.append('--health-only')
if args.capture_core:
    command.extend(['--core-dir', str(out / 'cores')])
if args.bridge_keep_rmw_loaded:
    command.append('--bridge-keep-rmw-loaded')
if args.cpus:
    command = ['taskset', '-c', args.cpus] + command

with (out / 'driver.log').open('w') as driver, (out / 'samples.jsonl').open('w') as samples:
    process = subprocess.Popen(command, cwd=root, stdout=driver,
                               stderr=subprocess.STDOUT, start_new_session=True,
                               env=runtime_environment)
    try:
        while process.poll() is None:
            ps = command_output(['ps', '-eo',
                'pid=,ppid=,psr=,ni=,cls=,pri=,stat=,pcpu=,pmem=,rss=,vsz=,nlwp=,etimes=,comm=,args=',
                '--sort=-pcpu'])
            lines = ps['stdout'].splitlines()
            sample = {
                'wall': time.time(),
                'monotonic': time.monotonic(),
                'loadavg': read('/proc/loadavg'),
                'proc_stat': read('/proc/stat'),
                'pressure': pressure_snapshot(),
                'frequency': frequency_snapshot(),
                'thermal': thermal_snapshot(),
                'meminfo': read('/proc/meminfo'),
                'processes': matching_processes(),
                'ps_top': lines[:30],
                'ps_relevant': [line for line in lines if any(token in line for token in (
                    'gz-sim', 'gz sim', 'parameter_bridg', 'robot_state_pub',
                    'ekf_node', 'map_server', 'amcl', 'planner_server',
                    'controller_serv', 'behavior_server', 'bt_navigator',
                    'lifecycle_manag', 'cmd_vel_adapter', 'ros2 bag',
                    'health_check', 'headroom', 'codex'))],
            }
            samples.write(json.dumps(sample) + '\n')
            samples.flush()
            time.sleep(1.0)
    finally:
        for pid, original in agent_affinities.items():
            if isinstance(original, list):
                try:
                    os.sched_setaffinity(pid, set(original))
                except (OSError, PermissionError, ProcessLookupError):
                    pass

trial_metrics = None
metric_files = sorted((root / trial_rel).glob('run_*/metrics.json'))
trial_results = []
for metric_file in metric_files:
    if metric_file.stat().st_size:
        result = json.loads(metric_file.read_text())
        result['metrics_path'] = str(metric_file.relative_to(root))
        trial_results.append(result)
if trial_results:
    trial_metrics = trial_results[0]
effective_rtfs = [
    result['effective_rtf'] for result in trial_results
    if result.get('effective_rtf') is not None
]
all_runs_present = len(trial_results) == args.runs
passing_runs = sum(bool(result.get('success')) for result in trial_results)
clean_shutdowns = sum(
    bool(result.get('clean_child_shutdown')) for result in trial_results
)
child_exits = [
    {'run': index, **child}
    for index, result in enumerate(trial_results, start=1)
    for child in result.get('launch_child_exit_codes', [])
]
summary = {
    'name': args.name,
    'driver_exit_code': process.returncode,
    'runs_requested': args.runs,
    'runs_completed': len(trial_results),
    'cpus': args.cpus,
    'isolate_agent': args.isolate_agent,
    'workload': 'mission+bag' if args.mission else 'health-only+bag',
    'agent_affinities': agent_affinities,
    'metrics_path': (
        str(metric_files[0].relative_to(root))
        if args.runs == 1 and metric_files else None
    ),
    'metrics_paths': [str(path.relative_to(root)) for path in metric_files],
    'passing_runs': passing_runs,
    'clean_shutdowns': clean_shutdowns,
    'effective_rtf': trial_metrics.get('effective_rtf') if args.runs == 1 and trial_metrics else None,
    'effective_rtf_min': min(effective_rtfs) if effective_rtfs else None,
    'effective_rtf_mean': statistics.mean(effective_rtfs) if effective_rtfs else None,
    'effective_rtf_median': statistics.median(effective_rtfs) if effective_rtfs else None,
    'effective_rtf_max': max(effective_rtfs) if effective_rtfs else None,
    'run_results': trial_results,
    'measurement_sim_delta_s': (
        max(value.get('last', 0.0) for value in trial_metrics.get('sensors', {}).values())
        - min(value.get('first', 0.0) for value in trial_metrics.get('sensors', {}).values())
        if args.runs == 1 and trial_metrics and trial_metrics.get('sensors') else None
    ),
    'functional_checks': trial_metrics.get('checks') if args.runs == 1 and trial_metrics else None,
    'sensors': trial_metrics.get('sensors') if args.runs == 1 and trial_metrics else None,
    'process_exit_codes': trial_metrics.get('process_exit_codes') if args.runs == 1 and trial_metrics else None,
    'launch_child_exit_codes': child_exits,
    'clean_child_shutdown': (
        bool(trial_metrics.get('clean_child_shutdown'))
        if args.runs == 1 and trial_metrics
        else all_runs_present and clean_shutdowns == args.runs
    ),
    'success': (
        process.returncode == 0
        and all_runs_present
        and passing_runs == args.runs
    ),
    'reason': (
        trial_metrics.get('reason') if args.runs == 1 and trial_metrics
        else None if all_runs_present and passing_runs == args.runs
        else 'One or more trials failed or did not produce metrics'
    ),
    'meminfo_after': read('/proc/meminfo'),
    'pressure_after': pressure_snapshot(),
    'frequency_after': frequency_snapshot(),
    'thermal_after': thermal_snapshot(),
    'processes_after': matching_processes(),
    'dmesg_after': command_output(['dmesg', '--ctime']),
}
(out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({key: summary[key] for key in (
    'name', 'driver_exit_code', 'effective_rtf', 'clean_child_shutdown',
    'success', 'reason')}), flush=True)
raise SystemExit(process.returncode)
