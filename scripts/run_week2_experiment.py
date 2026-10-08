#!/usr/bin/env python3
"""Run one deterministic Week 2 scenario against an existing ARES baseline."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
PROFILE_PATH = (
    WORKSPACE
    / 'src'
    / 'ares_reliability'
    / 'config'
    / 'fault_profiles.yaml'
)


def _parameter_text(value: Any) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def _ros_command(arguments: list[str], environment: dict[str, str]) -> str:
    result = subprocess.run(
        ['ros2', *arguments],
        cwd=WORKSPACE,
        env=environment,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return result.stdout


def _set_profile(
    profile: dict[str, Any],
    environment: dict[str, str],
) -> None:
    # Keep enabled false until all mode-specific values are installed.
    _ros_command(
        ['param', 'set', '/gnss_fault_injector', 'enabled', 'false'],
        environment,
    )
    for name, value in profile.items():
        if name == 'enabled':
            continue
        _ros_command(
            [
                'param',
                'set',
                '/gnss_fault_injector',
                name,
                _parameter_text(value),
            ],
            environment,
        )
    _ros_command(
        [
            'param',
            'set',
            '/gnss_fault_injector',
            'enabled',
            _parameter_text(profile['enabled']),
        ],
        environment,
    )


def main() -> None:
    """Launch reliability healthy, inject one scenario, then measure recovery."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'scenario',
        choices=(
            'healthy',
            'east_1m',
            'east_3m',
            'east_5m',
            'east_10m',
            'slow_east_drift',
            'north_step',
            'dropout',
        ),
    )
    parser.add_argument('--baseline-sec', type=float, default=15.0)
    parser.add_argument('--attack-sec', type=float, default=25.0)
    parser.add_argument('--recovery-sec', type=float, default=15.0)
    parser.add_argument('--results-dir', default='results/week2')
    args = parser.parse_args()

    configuration = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    profiles = configuration['profiles']
    environment = os.environ.copy()
    environment['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    environment['GALLIUM_DRIVER'] = 'llvmpipe'

    result_name = f'{args.scenario}_{time.strftime("%Y%m%d_%H%M%S")}'
    command = [
        'ros2',
        'launch',
        'ares_reliability',
        'reliability_test.launch.py',
        'profile:=healthy',
        'record:=true',
        f'experiment_label:={args.scenario}',
        f'result_name:={result_name}',
        f'results_dir:={args.results_dir}',
    ]
    launch_process = subprocess.Popen(
        command,
        cwd=WORKSPACE,
        env=environment,
        start_new_session=True,
    )

    try:
        time.sleep(3.0)
        if launch_process.poll() is not None:
            raise RuntimeError('reliability launch exited during startup')

        nodes = _ros_command(['node', 'list', '--no-daemon'], environment)
        required_nodes = (
            '/gnss_fault_injector',
            '/consistency_monitor',
            '/week2_experiment_recorder',
        )
        missing = [name for name in required_nodes if name not in nodes]
        if missing:
            raise RuntimeError('missing reliability nodes: ' + ', '.join(missing))

        services = _ros_command(['service', 'list'], environment)
        required_service = '/gnss_fault_injector/set_parameters'
        if required_service not in services:
            raise RuntimeError(
                f'missing parameter service: {required_service}'
            )

        print(f'Healthy reference period: {args.baseline_sec:.1f} s')
        time.sleep(args.baseline_sec)

        if args.scenario == 'healthy':
            print(f'Healthy observation period: {args.attack_sec:.1f} s')
            time.sleep(args.attack_sec)
        else:
            print(
                f'Injecting {args.scenario} for {args.attack_sec:.1f} s '
                'without restarting the monitor'
            )
            _set_profile(profiles[args.scenario], environment)
            time.sleep(args.attack_sec)
            print(f'Removing fault; recovery period: {args.recovery_sec:.1f} s')
            _set_profile(profiles['healthy'], environment)
            time.sleep(args.recovery_sec)
    finally:
        if launch_process.poll() is None:
            os.killpg(launch_process.pid, signal.SIGINT)
            try:
                launch_process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                launch_process.terminate()
                launch_process.wait(timeout=10.0)

    csv_path = Path(args.results_dir) / f'{result_name}.csv'
    print(f'Result: {csv_path}')


if __name__ == '__main__':
    main()
