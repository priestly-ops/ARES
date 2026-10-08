#!/usr/bin/env python3
"""Run reproducible Week 3 monitoring scenarios against ARES simulation."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any, Optional

import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rosgraph_msgs.msg import Clock
import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
PROFILE_PATH = WORKSPACE / 'src/ares_reliability/config/fault_profiles.yaml'


SCENARIOS = {
    'healthy_stationary': ('healthy', None),
    'healthy_straight': ('healthy', (0.15, 0.0)),
    'healthy_left_turn': ('healthy', (0.08, 0.35)),
    'healthy_right_turn': ('healthy', (0.08, -0.35)),
    'healthy_mixed': ('healthy', (0.12, 0.20)),
    'gnss_east_5m': ('east_5m', (0.10, 0.0)),
    'gnss_north_5m': ('north_step', (0.10, 0.0)),
    'gnss_slow_drift': ('slow_east_drift', (0.10, 0.0)),
    'gnss_dropout': ('dropout', (0.10, 0.0)),
    'gnss_fault_recovery': ('east_5m', (0.10, 0.15)),
    'localization_healthy': ('healthy', (0.10, 0.15)),
}


class SimulationClock:
    """Wait for experiment phases using Gazebo time instead of wall time."""

    def __init__(self) -> None:
        rclpy.init(args=None)
        self.node = rclpy.create_node('week3_experiment_clock')
        self.current_sec: Optional[float] = None
        self.subscription = self.node.create_subscription(
            Clock, '/clock', self._clock_callback, 10
        )
        self.fault_parameters = AsyncParameterClient(
            self.node, '/gnss_fault_injector'
        )
        if not self.fault_parameters.wait_for_services(timeout_sec=5.0):
            self.node.destroy_node()
            rclpy.shutdown()
            raise TimeoutError('GNSS fault injector parameter services absent')

    def _clock_callback(self, message: Clock) -> None:
        seconds = float(message.clock.sec)
        nanoseconds = float(message.clock.nanosec) / 1_000_000_000.0
        self.current_sec = seconds + nanoseconds

    def wait(self, duration_sec: float) -> None:
        """Wait for a non-negative duration measured on ``/clock``."""
        if duration_sec < 0.0:
            raise ValueError('experiment phase durations must be non-negative')
        if duration_sec == 0.0:
            return
        start_sec: Optional[float] = None
        wall_deadline = time.monotonic() + max(30.0, duration_sec * 30.0)
        while time.monotonic() < wall_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.25)
            if self.current_sec is None:
                continue
            if start_sec is None or self.current_sec < start_sec:
                start_sec = self.current_sec
            if self.current_sec - start_sec >= duration_sec:
                return
        raise TimeoutError(
            f'/clock did not advance {duration_sec:.3f} seconds before timeout'
        )

    def set_profile(self, profile: dict[str, Any]) -> None:
        """Apply one complete fault profile in a single atomic request."""
        parameters = [
            Parameter(name, value=value) for name, value in profile.items()
        ]
        future = self.fault_parameters.set_parameters_atomically(parameters)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if not future.done():
            raise TimeoutError('atomic GNSS fault profile update timed out')
        response = future.result()
        if response is None or not response.result.successful:
            reason = 'no service response'
            if response is not None:
                reason = response.result.reason
            raise RuntimeError(f'GNSS fault profile rejected: {reason}')

    def close(self) -> None:
        """Release the temporary clock subscriber."""
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def ros(arguments: list[str], environment: dict[str, str]) -> str:
    """Run one non-interactive ROS command and return output."""
    result = subprocess.run(
        ['ros2', *arguments], cwd=WORKSPACE, env=environment, check=True,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return result.stdout


def start_motion(linear: float, angular: float,
                 environment: dict[str, str]) -> subprocess.Popen:
    """Publish a bounded test velocity directly to the simulated robot."""
    value = f'{{linear: {{x: {linear}}}, angular: {{z: {angular}}}}}'
    return subprocess.Popen(
        ['ros2', 'topic', 'pub', '-r', '10', '/ares/cmd_vel',
         'geometry_msgs/msg/Twist', value], cwd=WORKSPACE, env=environment,
        start_new_session=True, stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT)


def stop(process: Optional[subprocess.Popen]) -> None:
    """Stop an owned process group without affecting unrelated ROS nodes."""
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5.0)


def main() -> None:
    """Launch, validate interfaces, execute a fault, and record recovery."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=tuple(SCENARIOS))
    parser.add_argument(
        '--baseline-sec', type=float, default=15.0,
        help='baseline duration in simulated seconds')
    parser.add_argument(
        '--scenario-sec', type=float, default=25.0,
        help='fault/healthy scenario duration in simulated seconds')
    parser.add_argument(
        '--recovery-sec', type=float, default=15.0,
        help='post-fault recovery duration in simulated seconds')
    parser.add_argument('--results-dir', default='results/week3')
    parser.add_argument('--launch-baseline', action='store_true')
    parser.add_argument('--stage', choices=('ekf', 'localization', 'nav2'),
                        default='localization')
    args = parser.parse_args()
    profiles = yaml.safe_load(PROFILE_PATH.read_text(
        encoding='utf-8'))['profiles']
    profile_name, motion = SCENARIOS[args.scenario]
    environment = os.environ.copy()
    environment['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    environment['GALLIUM_DRIVER'] = 'llvmpipe'
    os.environ['RMW_IMPLEMENTATION'] = environment['RMW_IMPLEMENTATION']
    baseline_process = None
    reliability_process = None
    motion_process = None
    simulation_clock = None
    result_name = f'{args.scenario}_{time.strftime("%Y%m%d_%H%M%S")}'
    try:
        if args.launch_baseline:
            baseline_process = subprocess.Popen(
                ['ros2', 'launch', 'ares_simulation',
                 'ares_baseline.launch.py', f'stage:={args.stage}',
                 'rviz:=false', 'camera:=false'], cwd=WORKSPACE,
                env=environment, start_new_session=True)
            time.sleep(18.0)
        reliability_process = subprocess.Popen(
            ['ros2', 'launch', 'ares_reliability',
             'week3_reliability.launch.py', 'profile:=healthy',
             f'result_profile:={profile_name}',
             'record:=true', f'result_name:={result_name}',
             f'results_dir:={args.results_dir}'], cwd=WORKSPACE,
            env=environment, start_new_session=True)
        time.sleep(4.0)
        nodes = ros(['node', 'list', '--no-daemon'], environment)
        required = ('/gnss_fault_injector', '/consistency_monitor',
                    '/imu_consistency_monitor', '/trust_engine',
                    '/recovery_manager', '/gnss_trusted_proxy',
                    '/week3_experiment_recorder')
        missing = [node for node in required if node not in nodes]
        if missing:
            raise RuntimeError('missing Week 3 nodes: ' + ', '.join(missing))
        topic_details = ros(
            ['topic', 'list', '-t', '--no-daemon'], environment
        )
        for topic in ('/ares/gps', '/ares/imu', '/ares/odom',
                      '/odometry/filtered'):
            if topic not in topic_details:
                raise RuntimeError(f'required runtime topic absent: {topic}')
        if motion is not None:
            motion_process = start_motion(*motion, environment)
        simulation_clock = SimulationClock()
        simulation_clock.wait(args.baseline_sec)
        if profile_name != 'healthy':
            simulation_clock.set_profile(profiles[profile_name])
        simulation_clock.wait(args.scenario_sec)
        if profile_name != 'healthy':
            simulation_clock.set_profile(profiles['healthy'])
        simulation_clock.wait(args.recovery_sec)
    finally:
        stop(motion_process)
        if simulation_clock is not None:
            simulation_clock.close()
        stop(reliability_process)
        stop(baseline_process)
    csv_path = Path(args.results_dir) / f'{result_name}.csv'
    print(f'Result: {csv_path}')
    print('Analyze: ros2 run ares_reliability analyze_week3_results '
          f'{csv_path}')


if __name__ == '__main__':
    main()
