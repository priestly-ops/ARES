#!/usr/bin/env python3
"""Reproduce Gazebo / parameter_bridge shutdown behavior outside mission acceptance."""
import argparse
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import yaml


parser = argparse.ArgumentParser()
parser.add_argument('--name', required=True)
parser.add_argument('--first', required=True, choices=('bridge', 'gazebo'))
parser.add_argument('--signal', required=True,
                    choices=('SIGINT', 'SIGTERM', 'SERVER_STOP',
                             'WORLD_PAUSE_STOP', 'REMOVE_MODEL_STOP',
                             'BRIDGE_STOP_SERVER'))
parser.add_argument('--gazebo-only', action='store_true')
parser.add_argument('--remove-lidar', action='store_true',
                    help='Remove LiDAR only in the generated diagnostic world')
parser.add_argument('--software', action='store_true')
parser.add_argument('--subscriber-driven-lidar', action='store_true',
                    help='Set LiDAR always_on=false only in diagnostic world')
parser.add_argument('--render-engine', choices=('ogre', 'ogre2'))
parser.add_argument('--output-root', default='results/week1/shutdown_diagnostics')
args = parser.parse_args()

root = Path(__file__).resolve().parents[1]
out = root / args.output_root / args.name
out.mkdir(parents=True, exist_ok=False)
core_limit_before = resource.getrlimit(resource.RLIMIT_CORE)
resource.setrlimit(resource.RLIMIT_CORE, (resource.RLIM_INFINITY, resource.RLIM_INFINITY))
core_limit_after = resource.getrlimit(resource.RLIMIT_CORE)
selected_signal = getattr(signal, args.signal) if args.signal.startswith('SIG') else None
if args.signal in ('SERVER_STOP', 'WORLD_PAUSE_STOP', 'REMOVE_MODEL_STOP') and args.first != 'gazebo':
    parser.error(args.signal + ' requires --first gazebo')
if args.gazebo_only and args.first != 'gazebo':
    parser.error('--gazebo-only requires --first gazebo')
if args.signal == 'BRIDGE_STOP_SERVER' and args.first != 'bridge':
    parser.error('BRIDGE_STOP_SERVER requires --first bridge')


def read_dmesg():
    result = subprocess.run(['dmesg', '--ctime'], text=True, capture_output=True)
    return result.stdout.splitlines(), result.stderr, result.returncode


def check_clean():
    rows = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True).splitlines()
    conflicts = [row for row in rows if any(token in row for token in (
        'gz sim -', 'gz-sim-main', '/ros_gz_bridge/parameter_bridge'))]
    if conflicts:
        raise RuntimeError('Contaminated baseline: ' + repr(conflicts))


def stop_owned(process, sig=signal.SIGINT, timeout=10):
    if process is None or process.poll() is not None:
        return
    try:
        os.kill(process.pid, sig)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


check_clean()
before_dmesg, before_dmesg_stderr, before_dmesg_rc = read_dmesg()

env = os.environ.copy()
env.update(
    ROS_DOMAIN_ID=str(100 + os.getpid() % 100),
    GZ_PARTITION='ares_shutdown_' + str(os.getpid()),
    ROS_LOG_DIR=str(out / 'ros_logs'),
)
if args.software:
    env['LIBGL_ALWAYS_SOFTWARE'] = '1'

processes = {}
logs = {}
temp = tempfile.TemporaryDirectory(prefix='ares_shutdown_')
directory = Path(temp.name)
try:
    world_tree = ET.parse(root / 'src/ares_simulation/worlds/ares_test_world.sdf')
    world = world_tree.getroot().find('world')
    if args.render_engine:
        world.find("plugin[@name='gz::sim::systems::Sensors']/render_engine").text = args.render_engine
    for actor in world.findall('actor'):
        world.remove(actor)
    include = world.find('include')
    model = ET.parse(root / 'src/ares_simulation/models/ares_jackal/model.sdf').getroot().find('model')
    model.find('pose').text = include.findtext('pose')
    world.remove(include)
    camera_link = model.find("link[@name='camera_link']")
    for sensor_element in camera_link.findall('sensor'):
        camera_link.remove(sensor_element)
    if args.remove_lidar:
        lidar_link = model.find("link[@name='lidar_link']")
        for sensor_element in lidar_link.findall('sensor'):
            lidar_link.remove(sensor_element)
    if args.subscriber_driven_lidar:
        model.find(".//sensor[@name='ares_lidar']/always_on").text = 'false'
    world.append(model)
    world_path = directory / 'world.sdf'
    world_tree.write(world_path, encoding='unicode')
    bridge_path = directory / 'bridge.yaml'
    bridge = yaml.safe_load((root / 'src/ares_simulation/config/bridge_baseline.yaml').read_text())
    bridge_path.write_text(yaml.safe_dump(bridge))

    def start(name, command):
        log = (out / (name + '.log')).open('w')
        logs[name] = log
        process = subprocess.Popen(command, cwd=out if name == 'gazebo' else root,
                                   env=env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes[name] = process
        return process

    gazebo = start('gazebo', ['gz', 'sim', '-s', '-r', '--seed', '42', str(world_path)])
    time.sleep(3)
    bridge_process = subscriber = None
    if not args.gazebo_only:
        bridge_process = start('bridge', [
            '/opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge',
            '--ros-args', '-r', '__node:=ros_gz_bridge',
            '-p', 'config_file:=' + str(bridge_path),
            '-p', 'use_sim_time:=true'])
        time.sleep(3)
        subscriber = start('scan_subscriber', [
            'ros2', 'topic', 'hz', '/ares/scan', '--window', '20'])
        time.sleep(8)
    else:
        time.sleep(8)

    startup = {
        name: {'pid': process.pid, 'poll': process.poll()}
        for name, process in processes.items()
    }
    (out / 'startup.json').write_text(json.dumps(startup, indent=2) + '\n')
    for name, process in processes.items():
        try:
            (out / (name + '.maps')).write_text(
                Path(f'/proc/{process.pid}/maps').read_text(errors='replace')
            )
        except (OSError, PermissionError, ProcessLookupError) as exc:
            (out / (name + '.maps')).write_text(f'UNAVAILABLE: {exc!r}\n')
    if gazebo.poll() is not None or (bridge_process is not None and bridge_process.poll() is not None):
        raise RuntimeError('Gazebo or bridge exited before shutdown test: ' + repr(startup))

    first = bridge_process if args.first == 'bridge' else gazebo
    second = None if args.gazebo_only else (gazebo if args.first == 'bridge' else bridge_process)
    timeline = []
    first_name = args.first
    second_name = None if args.gazebo_only else ('gazebo' if args.first == 'bridge' else 'bridge')
    if args.signal in ('SERVER_STOP', 'WORLD_PAUSE_STOP', 'REMOVE_MODEL_STOP'):
        if args.signal == 'WORLD_PAUSE_STOP':
            pause = subprocess.run([
                'gz', 'service', '-s', '/world/ares_world/control',
                '--reqtype', 'gz.msgs.WorldControl',
                '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
                '--req', 'pause: true'], cwd=root, env=env, text=True,
                capture_output=True)
            timeline.append({'event': 'world_pause', 'process': first_name,
                             'returncode': pause.returncode,
                             'stdout': pause.stdout, 'stderr': pause.stderr,
                             'wall': time.time()})
            time.sleep(5)
        if args.signal == 'REMOVE_MODEL_STOP':
            remove = subprocess.run([
                'gz', 'service', '-s', '/world/ares_world/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
                '--req', 'name: "ares_jackal" type: MODEL'], cwd=root,
                env=env, text=True, capture_output=True)
            timeline.append({'event': 'remove_model', 'process': first_name,
                             'returncode': remove.returncode,
                             'stdout': remove.stdout, 'stderr': remove.stderr,
                             'wall': time.time()})
            time.sleep(5)
        control = subprocess.run([
            'gz', 'service', '-s', '/server_control',
            '--reqtype', 'gz.msgs.ServerControl',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
            '--req', 'stop: true'], cwd=root, env=env, text=True,
            capture_output=True)
        timeline.append({'event': 'server_control', 'process': first_name,
                         'returncode': control.returncode,
                         'stdout': control.stdout, 'stderr': control.stderr,
                         'wall': time.time()})
    else:
        first_signal = signal.SIGINT if args.signal == 'BRIDGE_STOP_SERVER' else selected_signal
        os.kill(first.pid, first_signal)
        timeline.append({'event': 'signal', 'process': first_name,
                         'signal': 'SIGINT' if args.signal == 'BRIDGE_STOP_SERVER' else args.signal,
                         'wall': time.time()})
    try:
        first.wait(timeout=12)
    except subprocess.TimeoutExpired:
        timeline.append({'event': 'timeout', 'process': first_name, 'wall': time.time()})
    timeline.append({'event': 'exit', 'process': first_name,
                     'exit_code': first.poll(), 'wall': time.time()})

    time.sleep(2)
    if second is not None and second.poll() is None:
        if args.signal == 'BRIDGE_STOP_SERVER':
            time.sleep(5)
            control = subprocess.run([
                'gz', 'service', '-s', '/server_control',
                '--reqtype', 'gz.msgs.ServerControl',
                '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
                '--req', 'stop: true'], cwd=root, env=env, text=True,
                capture_output=True)
            timeline.append({'event': 'server_control', 'process': second_name,
                             'returncode': control.returncode,
                             'stdout': control.stdout, 'stderr': control.stderr,
                             'wall': time.time()})
        else:
            second_signal = signal.SIGINT if args.signal in ('SERVER_STOP', 'WORLD_PAUSE_STOP', 'REMOVE_MODEL_STOP') else selected_signal
            os.kill(second.pid, second_signal)
            timeline.append({'event': 'signal', 'process': second_name,
                             'signal': 'SIGINT' if args.signal in ('SERVER_STOP', 'WORLD_PAUSE_STOP', 'REMOVE_MODEL_STOP') else args.signal,
                             'wall': time.time()})
        try:
            second.wait(timeout=12)
        except subprocess.TimeoutExpired:
            timeline.append({'event': 'timeout', 'process': second_name,
                             'wall': time.time()})
    if second is not None:
        timeline.append({'event': 'exit', 'process': second_name,
                         'exit_code': second.poll(), 'wall': time.time()})
finally:
    for name in ('scan_subscriber', 'bridge', 'gazebo'):
        stop_owned(processes.get(name))
    for log in logs.values():
        log.close()
    temp.cleanup()

after_dmesg, after_dmesg_stderr, after_dmesg_rc = read_dmesg()
new_dmesg = after_dmesg[len(before_dmesg):] if after_dmesg[:len(before_dmesg)] == before_dmesg else []
summary = {
    'name': args.name,
    'first': args.first,
    'signal': args.signal,
    'ros_domain_id': env['ROS_DOMAIN_ID'],
    'gz_partition': env['GZ_PARTITION'],
    'core_pattern': Path('/proc/sys/kernel/core_pattern').read_text(errors='replace').strip(),
    'core_limit_before': list(core_limit_before),
    'core_limit_after': [
        'unlimited' if value == resource.RLIM_INFINITY else value
        for value in core_limit_after
    ],
    'startup': startup,
    'timeline': timeline,
    'exit_codes': {name: process.returncode for name, process in processes.items()},
    'dmesg_before_returncode': before_dmesg_rc,
    'dmesg_before_stderr': before_dmesg_stderr,
    'dmesg_after_returncode': after_dmesg_rc,
    'dmesg_after_stderr': after_dmesg_stderr,
    'dmesg_new': new_dmesg,
    'logs': {name: str((out / (name + '.log')).relative_to(root)) for name in logs},
    'core_files': [
        {'path': str(path.relative_to(root)), 'size': path.stat().st_size}
        for path in sorted(out.glob('core*')) if path.is_file()
    ],
}
(out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
check_clean()
print(json.dumps({key: summary[key] for key in (
    'name', 'first', 'signal', 'exit_codes', 'dmesg_new')}), flush=True)
