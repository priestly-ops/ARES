#!/usr/bin/env python3
"""Run one reproducible Week 5 active-fusion experiment."""

import argparse
import json
import math
import os
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any

import rclpy

from ares_reliability.experiment_matrix import (
    injector_profiles,
    load_fault_matrix,
)
from run_week4_experiment import ExperimentController, _stop
from std_msgs.msg import String


WORKSPACE = Path(__file__).resolve().parents[1]
MATRIX_PATH = (
    WORKSPACE / 'src/ares_reliability/config/week5_fault_matrix.yaml')
FUSION_MODES = ('odom_imu_only', 'unprotected', 'protected')


class Week5ExperimentController(ExperimentController):
    """Observe recovery state while retaining the Week 4 motion driver."""

    def __init__(self) -> None:
        super().__init__()
        self.recovery_state: str | None = None
        self.node.create_subscription(
            String, '/ares/recovery_state', self._recovery_callback, 10)

    def _recovery_callback(self, message: String) -> None:
        self.recovery_state = message.data

    def reset_recovery_observation(self) -> None:
        """Require a fresh recovery-state sample after fault clear."""
        self.recovery_state = None

    def wait_for_recovery_state(
        self,
        desired_state: str,
        timeout_sec: float,
        motion: dict[str, Any],
    ) -> float:
        """Wait in simulation time for a freshly observed recovery state."""
        start_sec = self.current_sec
        elapsed = 0.0
        while elapsed < timeout_sec:
            self.wait(min(0.5, timeout_sec - elapsed), motion)
            if self.recovery_state == desired_state:
                if self.current_sec is None:
                    raise RuntimeError('recovery state arrived without /clock')
                return self.current_sec
            if self.current_sec is not None:
                if start_sec is None or self.current_sec < start_sec:
                    start_sec = self.current_sec
                elapsed = self.current_sec - start_sec
            else:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        raise TimeoutError(
            f'recovery state did not reach {desired_state!r} within '
            f'{timeout_sec:.3f} simulated seconds')


def main() -> None:
    """Execute a matrix scenario against one isolated fusion mode."""
    matrix = load_fault_matrix(MATRIX_PATH)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=tuple(matrix['scenarios']))
    parser.add_argument('--fusion-mode', choices=FUSION_MODES,
                        default='protected')
    parser.add_argument('--results-dir', default='results/week5')
    parser.add_argument('--launch-baseline', action='store_true')
    parser.add_argument('--stage', choices=('ekf', 'localization', 'nav2'),
                        default='localization')
    parser.add_argument('--baseline-ready-sec', type=float, default=18.0)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--covariance-multiplier', type=float)
    parser.add_argument('--recovery-sec', type=float)
    parser.add_argument('--post-normal-sec', type=float)
    parser.add_argument('--normal-timeout-sec', type=float, default=120.0)
    args = parser.parse_args()
    scenario = matrix['scenarios'][args.scenario]
    if args.recovery_sec is not None:
        if not math.isfinite(args.recovery_sec) or args.recovery_sec < 0.0:
            parser.error('--recovery-sec must be finite and non-negative')
        matrix['defaults']['recovery_sec'] = args.recovery_sec
    if (args.post_normal_sec is not None and
            (not math.isfinite(args.post_normal_sec) or
             args.post_normal_sec < 0.0)):
        parser.error('--post-normal-sec must be finite and non-negative')
    if (not math.isfinite(args.normal_timeout_sec) or
            args.normal_timeout_sec <= 0.0):
        parser.error('--normal-timeout-sec must be finite and positive')
    matrix_path = MATRIX_PATH
    temporary_matrix = None
    if args.seed is not None:
        if args.seed < 0:
            parser.error('--seed must be non-negative')
        matrix['defaults']['seed'] = args.seed
        scenario['seed'] = args.seed
    if args.covariance_multiplier is not None:
        multiplier = args.covariance_multiplier
        if not math.isfinite(multiplier) or multiplier < 1.0:
            parser.error('--covariance-multiplier must be finite and >= 1')
        recovery = scenario.setdefault('recovery_parameters', {})
        for parameter in (
                'degraded_covariance_factor',
                'untrusted_covariance_factor',
                'probation_high_covariance_factor',
                'probation_covariance_factor'):
            recovery[parameter] = multiplier
    if args.seed is not None or args.covariance_multiplier is not None:
        temporary = tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', suffix='.yaml', delete=False)
        with temporary:
            json.dump(matrix, temporary)
        temporary_matrix = Path(temporary.name)
        matrix_path = temporary_matrix
    environment = os.environ.copy()
    environment['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    environment['GALLIUM_DRIVER'] = 'llvmpipe'
    log_directory = WORKSPACE / 'results/week5/ros_logs'
    log_directory.mkdir(parents=True, exist_ok=True)
    environment['ROS_LOG_DIR'] = str(log_directory)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['ROS_LOG_DIR'] = str(log_directory)
    baseline_process = None
    reliability_process = None
    controller = None
    seed_suffix = f'_s{args.seed}' if args.seed is not None else ''
    covariance_suffix = (
        f'_c{args.covariance_multiplier:g}'
        if args.covariance_multiplier is not None else '')
    post_normal_suffix = (
        f'_pn{args.post_normal_sec:g}'
        if args.post_normal_sec is not None else '')
    result_name = (
        f'{args.scenario}_{args.fusion_mode}_'
        f'{time.strftime("%Y%m%d_%H%M%S")}'
        f'{seed_suffix}{covariance_suffix}{post_normal_suffix}')
    activate = bool(scenario.get('activate_on_start', False))
    healthy = {
        sensor: {'enabled': False, 'mode': 'none'}
        for sensor in ('gnss', 'imu', 'wheel')
    }
    try:
        if args.launch_baseline:
            baseline_process = subprocess.Popen(
                ['ros2', 'launch', 'ares_simulation',
                 'ares_baseline.launch.py', f'stage:={args.stage}',
                 'rviz:=false', 'camera:=false',
                 f'seed:={matrix["defaults"]["seed"]}'],
                cwd=WORKSPACE, env=environment, start_new_session=True)
            time.sleep(args.baseline_ready_sec)
        reliability_process = subprocess.Popen(
            ['ros2', 'launch', 'ares_reliability',
             'week5_trust_fusion.launch.py',
             f'scenario:={args.scenario}',
             f'fusion_mode:={args.fusion_mode}',
             f'activate_on_start:={str(activate).lower()}',
             'record:=true', f'result_name:={result_name}',
             f'results_dir:={args.results_dir}',
             f'matrix_file:={matrix_path}'],
            cwd=WORKSPACE, env=environment, start_new_session=True)
        time.sleep(8.0)
        controller = Week5ExperimentController()
        motion = dict(scenario.get('motion', {}))
        recovery_motion: dict[str, Any] = {'type': 'stationary'}
        if not activate:
            controller.set_profiles(healthy)
            controller.wait(float(scenario['start_time']), motion)
            profiles = injector_profiles(scenario)
            cycles = int(scenario.get('cycles', 1))
            for cycle in range(cycles):
                controller.set_profiles(profiles)
                controller.wait(float(scenario['duration']), motion)
                if (cycle + 1 == cycles and
                        args.post_normal_sec is not None):
                    controller.reset_recovery_observation()
                controller.set_profiles(healthy)
                if cycle + 1 < cycles:
                    controller.wait(float(scenario.get(
                        'recovery_between_cycles_sec', 15.0)),
                        recovery_motion)
        else:
            controller.wait(float(scenario['duration']), motion)
            if args.post_normal_sec is not None:
                controller.reset_recovery_observation()
            controller.set_profiles(healthy)
        if args.post_normal_sec is None:
            controller.wait(float(matrix['defaults']['recovery_sec']),
                            recovery_motion)
        else:
            normal_observed_at = controller.wait_for_recovery_state(
                'NORMAL', args.normal_timeout_sec, recovery_motion)
            print(
                f'Observed recovery NORMAL at sim time '
                f'{normal_observed_at:.3f}; holding for '
                f'{args.post_normal_sec:.3f}s',
                flush=True,
            )
            controller.wait(args.post_normal_sec, recovery_motion)
    finally:
        if controller is not None:
            controller.close()
        _stop(reliability_process)
        _stop(baseline_process)
        if temporary_matrix is not None:
            temporary_matrix.unlink(missing_ok=True)
    csv_path = Path(args.results_dir) / f'{result_name}.csv'
    print(f'Result: {csv_path}')


if __name__ == '__main__':
    main()
