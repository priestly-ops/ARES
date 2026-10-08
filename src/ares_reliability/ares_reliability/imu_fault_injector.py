#!/usr/bin/env python3
"""Publish an operational IMU stream with deterministic message-level faults."""

import copy
import json
import math
from typing import Any, Optional

from ares_reliability.fault_models import IMU_FAULT_MODES, inject_scalar
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class ImuFaultInjector(Node):
    """Apply yaw-rate faults between the raw and operational IMU topics."""

    def __init__(self) -> None:
        super().__init__('imu_fault_injector')
        defaults = {
            'enabled': False, 'mode': 'none', 'input_topic': '/ares/imu',
            'output_topic': '/ares/imu_operational', 'bias_radps': 0.0,
            'drift_rate_radps2': 0.0, 'scale_factor': 1.0,
            'additional_noise_stddev_radps': 0.0, 'spike_radps': 1.0,
            'spike_every_n': 10, 'dropout_every_n': 1, 'seed': 0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.create_subscription(Imu, input_topic, self.imu_callback, 50)
        self.publisher = self.create_publisher(Imu, output_topic, 50)
        self.status_publisher = self.create_publisher(
            String, '/ares/imu_fault_status', 20)
        self._signature: Optional[tuple[Any, ...]] = None
        self._started_sec = 0.0
        self._sample_index = 0
        self._frozen_rate: Optional[float] = None
        self.add_on_set_parameters_callback(self._validate_parameters)

    def _validate_parameters(self,
                             parameters: list[Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if (parameter.name == 'mode' and
                    parameter.value not in IMU_FAULT_MODES):
                return SetParametersResult(
                    successful=False,
                    reason='unsupported IMU fault mode')
            if parameter.name in ('spike_every_n', 'dropout_every_n') and (
                    not isinstance(parameter.value, int) or
                    parameter.value < 1):
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} must be an integer >= 1')
        return SetParametersResult(successful=True)

    def _configuration(self) -> tuple[Any, ...]:
        names = (
            'enabled', 'mode', 'bias_radps', 'drift_rate_radps2',
            'scale_factor', 'additional_noise_stddev_radps', 'spike_radps',
            'spike_every_n', 'dropout_every_n', 'seed')
        return tuple(self.get_parameter(name).value for name in names)

    def _elapsed(self, signature: tuple[Any, ...], now_sec: float) -> float:
        if signature != self._signature or now_sec < self._started_sec:
            self._signature = signature
            self._started_sec = now_sec
            self._sample_index = 0
            self._frozen_rate = None
        return max(0.0, now_sec - self._started_sec)

    def imu_callback(self, message: Imu) -> None:
        """Inject one yaw-rate sample and preserve all other IMU fields."""
        config = self._configuration()
        enabled = bool(config[0])
        mode = str(config[1]) if enabled else 'none'
        now_sec = self.get_clock().now().nanoseconds / 1.0e9
        elapsed = self._elapsed(config, now_sec)
        index = self._sample_index
        self._sample_index += 1
        if mode == 'dropout' and (
                (index + 1) % int(config[8]) == 0):
            self._publish_status(mode, elapsed, True, None)
            return
        raw_rate = float(message.angular_velocity.z)
        if not math.isfinite(raw_rate):
            self._publish_status(mode, elapsed, True, None)
            return
        if mode == 'freeze' and self._frozen_rate is None:
            self._frozen_rate = raw_rate
        injected = inject_scalar(
            raw_rate, mode, elapsed, index, seed=int(config[9]),
            bias=float(config[2]), drift_rate=float(config[3]),
            scale=float(config[4]), noise_stddev=float(config[5]),
            spike_value=float(config[6]), spike_every_n=int(config[7]),
            frozen_value=self._frozen_rate)
        if injected is None:
            self._publish_status(mode, elapsed, True, None)
            return
        output = copy.deepcopy(message)
        output.angular_velocity.z = injected
        self.publisher.publish(output)
        self._publish_status(mode, elapsed, False, injected)

    def _publish_status(self, mode: str, elapsed: float, dropped: bool,
                        yaw_rate: Optional[float]) -> None:
        status = String()
        status.data = json.dumps({
            'enabled': bool(self.get_parameter('enabled').value),
            'mode': mode, 'elapsed_sec': elapsed, 'dropped': dropped,
            'output_yaw_rate_radps': yaw_rate,
            'seed': int(self.get_parameter('seed').value),
        }, separators=(',', ':'), sort_keys=True)
        self.status_publisher.publish(status)


def main(args: Optional[list[str]] = None) -> None:
    """Run the IMU fault injector node."""
    rclpy.init(args=args)
    node = ImuFaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
