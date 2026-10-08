#!/usr/bin/env python3
"""Run one matrix-defined Week 4 ARES experiment in simulated time."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any, Optional

from ares_reliability.experiment_matrix import (
    bounded_motion_command,
    calibration_sweep_motion,
    injector_profiles,
    load_fault_matrix,
)
from geometry_msgs.msg import Twist
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


WORKSPACE = Path(__file__).resolve().parents[1]
MATRIX_PATH = (
    WORKSPACE / 'src/ares_reliability/config/week4_fault_matrix.yaml')


class ExperimentController:
    """Drive motion and injector parameters against the Gazebo clock."""

    def __init__(self) -> None:
        rclpy.init(args=None)
        self.node = rclpy.create_node('week4_experiment_controller')
        self.current_sec: Optional[float] = None
        self.node.create_subscription(Clock, '/clock', self._clock_callback, 10)
        self.motion_pub = self.node.create_publisher(
            Twist, '/ares/cmd_vel', 10)
        self.regime_pub = self.node.create_publisher(
            String, '/ares/experiment/motion_regime', 10)
        self.clients = {
            sensor: AsyncParameterClient(self.node, node_name)
            for sensor, node_name in {
                'gnss': '/gnss_fault_injector',
                'imu': '/imu_fault_injector',
                'wheel': '/wheel_fault_injector',
            }.items()
        }
        for sensor, client in self.clients.items():
            if not client.wait_for_services(timeout_sec=10.0):
                raise TimeoutError(f'{sensor} injector parameter service absent')

    def _clock_callback(self, message: Clock) -> None:
        self.current_sec = (float(message.clock.sec) +
                            float(message.clock.nanosec) / 1.0e9)

    def set_profiles(self,
                     profiles: dict[str, dict[str, Any]]) -> None:
        """Apply all sensor profiles, atomically per injector."""
        for sensor, profile in profiles.items():
            parameters = [
                Parameter(name, value=value) for name, value in profile.items()
            ]
            future = self.clients[sensor].set_parameters_atomically(parameters)
            rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=5.0)
            response = future.result() if future.done() else None
            if response is None or not response.result.successful:
                reason = ('no service response' if response is None else
                          response.result.reason)
                raise RuntimeError(
                    f'{sensor} injector rejected profile: {reason}')

    @staticmethod
    def _motion_command(
        motion: dict[str, Any],
        elapsed: float,
    ) -> tuple[Twist, str]:
        command = Twist()
        kind = str(motion.get('type', 'stationary'))
        linear = float(motion.get('linear_x', 0.0))
        angular = float(motion.get('angular_z', 0.0))
        regime = kind
        if kind == 'stop_start' and int(elapsed / 10.0) % 2 == 1:
            linear = 0.0
        elif kind == 'accel_decel':
            phase = (elapsed % 20.0) / 10.0
            linear *= phase if phase <= 1.0 else 2.0 - phase
        elif kind == 'figure8':
            angular *= 1.0 if int(elapsed / 15.0) % 2 == 0 else -1.0
        elif kind == 'mixed':
            phase = int(elapsed / 15.0) % 4
            linear = (0.0, linear, linear * 0.5, linear)[phase]
            angular = (0.0, 0.0, angular, -angular)[phase]
            regime = ('stationary', 'straight_slow', 'gentle_left',
                      'gentle_right')[phase]
        elif kind == 'calibration_sweep':
            linear, angular, regime = calibration_sweep_motion(elapsed)
        if 'reverse_period_sec' in motion:
            linear, angular = bounded_motion_command(
                linear,
                angular,
                elapsed,
                float(motion['reverse_period_sec']),
            )
        command.linear.x = linear
        command.angular.z = angular
        return command, regime

    def wait(self, duration_sec: float, motion: dict[str, Any]) -> None:
        """Wait in simulation time while publishing the selected motion."""
        if duration_sec <= 0.0:
            return
        start_sec: Optional[float] = None
        last_publish = -1.0
        deadline = time.monotonic() + max(60.0, duration_sec * 30.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.current_sec is None:
                continue
            if start_sec is None or self.current_sec < start_sec:
                start_sec = self.current_sec
            elapsed = self.current_sec - start_sec
            if elapsed - last_publish >= 0.1:
                command, regime = self._motion_command(motion, elapsed)
                self.motion_pub.publish(command)
                label = String()
                label.data = regime
                self.regime_pub.publish(label)
                last_publish = elapsed
            if elapsed >= duration_sec:
                return
        raise TimeoutError(
            f'/clock did not advance {duration_sec:.3f}s before timeout')

    def close(self) -> None:
        """Stop motion and close the temporary ROS node."""
        self.motion_pub.publish(Twist())
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _stop(process: Optional[subprocess.Popen]) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5.0)


def main() -> None:
    """Execute one selected scenario and print its evidence path."""
    matrix = load_fault_matrix(MATRIX_PATH)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=tuple(matrix['scenarios']))
    parser.add_argument('--results-dir', default='results/week4')
    parser.add_argument('--launch-baseline', action='store_true')
    parser.add_argument('--stage', choices=('ekf', 'localization', 'nav2'),
                        default='localization')
    args = parser.parse_args()
    scenario = matrix['scenarios'][args.scenario]
    if not scenario.get('automated', True):
        raise RuntimeError(str(scenario.get('blocker', 'scenario is manual')))
    environment = os.environ.copy()
    environment['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    environment['GALLIUM_DRIVER'] = 'llvmpipe'
    log_directory = WORKSPACE / 'results/week4/ros_logs'
    log_directory.mkdir(parents=True, exist_ok=True)
    environment['ROS_LOG_DIR'] = str(log_directory)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['ROS_LOG_DIR'] = str(log_directory)
    baseline_process = None
    reliability_process = None
    controller = None
    result_name = f'{args.scenario}_{time.strftime("%Y%m%d_%H%M%S")}'
    activate = bool(scenario.get('activate_on_start', False))
    healthy = {
        sensor: {'enabled': False, 'mode': 'none'}
        for sensor in ('gnss', 'imu', 'wheel')}
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
             'week4_reliability.launch.py', f'scenario:={args.scenario}',
             f'activate_on_start:={str(activate).lower()}', 'record:=true',
             f'result_name:={result_name}',
             f'results_dir:={args.results_dir}'], cwd=WORKSPACE,
            env=environment, start_new_session=True)
        time.sleep(5.0)
        controller = ExperimentController()
        motion = dict(scenario.get('motion', {}))
        if not activate:
            controller.set_profiles(healthy)
            controller.wait(float(scenario['start_time']), motion)
            profiles = injector_profiles(scenario)
            cycles = int(scenario.get('cycles', 1))
            for cycle in range(cycles):
                controller.set_profiles(profiles)
                controller.wait(float(scenario['duration']), motion)
                controller.set_profiles(healthy)
                if cycle + 1 < cycles:
                    controller.wait(float(scenario.get(
                        'recovery_between_cycles_sec', 15.0)),
                        {'type': 'stationary'})
        else:
            controller.wait(float(scenario['duration']), motion)
            controller.set_profiles(healthy)
        # Recovery is measured at rest so a trajectory does not restart and
        # contaminate regime labels or mask a cleared fault with fresh motion.
        controller.wait(float(matrix['defaults']['recovery_sec']),
                        {'type': 'stationary'})
    finally:
        if controller is not None:
            controller.close()
        _stop(reliability_process)
        _stop(baseline_process)
    csv_path = Path(args.results_dir) / f'{result_name}.csv'
    print(f'Result: {csv_path}')
    print('Analyze: ros2 run ares_reliability analyze_week4_results '
          f'{csv_path} --output results/week4/summary.json')


if __name__ == '__main__':
    main()
