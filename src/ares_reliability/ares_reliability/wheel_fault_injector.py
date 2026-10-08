#!/usr/bin/env python3
"""Publish deterministic message-level wheel-odometry faults."""

import copy
import json
import math
from typing import Any, Optional

from ares_reliability.fault_models import (
    WHEEL_FAULT_MODES,
    WheelFaultModel,
    WheelSample,
)
from ares_reliability.sensor_consistency import PoseYawRateEstimator
from ares_reliability.time_sync import stamp_to_seconds
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


def _yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _set_planar_yaw(message: Odometry, yaw: float) -> None:
    q = message.pose.pose.orientation
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)


class WheelFaultInjector(Node):
    """Corrupt odometry messages without modifying simulator ground truth."""

    def __init__(self) -> None:
        super().__init__('wheel_fault_injector')
        defaults = {
            'enabled': False, 'mode': 'none', 'input_topic': '/ares/odom',
            'output_topic': '/ares/odom_operational', 'linear_scale': 1.0,
            'angular_scale': 1.0, 'yaw_bias_radps': 0.0,
            'slip_bias_mps': 0.0, 'dropout_every_n': 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.create_subscription(Odometry, input_topic, self.odom_callback, 50)
        self.publisher = self.create_publisher(Odometry, output_topic, 50)
        self.status_publisher = self.create_publisher(
            String, '/ares/wheel_fault_status', 20)
        self.model = WheelFaultModel()
        self.yaw_rate_estimator = PoseYawRateEstimator()
        self._signature: Optional[tuple[Any, ...]] = None
        self._started_sec = 0.0
        self._sample_index = 0
        self.add_on_set_parameters_callback(self._validate_parameters)

    def _validate_parameters(self,
                             parameters: list[Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if (parameter.name == 'mode' and
                    parameter.value not in WHEEL_FAULT_MODES):
                return SetParametersResult(
                    successful=False, reason='unsupported wheel fault mode')
            if parameter.name == 'dropout_every_n' and (
                    not isinstance(parameter.value, int) or
                    parameter.value < 1):
                return SetParametersResult(
                    successful=False,
                    reason='dropout_every_n must be an integer >= 1')
        return SetParametersResult(successful=True)

    def _configuration(self) -> tuple[Any, ...]:
        names = (
            'enabled', 'mode', 'linear_scale', 'angular_scale',
            'yaw_bias_radps', 'slip_bias_mps', 'dropout_every_n')
        return tuple(self.get_parameter(name).value for name in names)

    def _elapsed(self, signature: tuple[Any, ...], now_sec: float) -> float:
        if signature != self._signature or now_sec < self._started_sec:
            self._signature = signature
            self._started_sec = now_sec
            self._sample_index = 0
            self.model.reset()
            self.yaw_rate_estimator.reset()
        return max(0.0, now_sec - self._started_sec)

    def odom_callback(self, message: Odometry) -> None:
        """Apply one operational wheel-odometry corruption."""
        config = self._configuration()
        enabled = bool(config[0])
        mode = str(config[1]) if enabled else 'none'
        now_sec = self.get_clock().now().nanoseconds / 1.0e9
        elapsed = self._elapsed(config, now_sec)
        index = self._sample_index
        self._sample_index += 1
        if mode == 'dropout' and (
                (index + 1) % int(config[6]) != 0):
            mode = 'none'
        yaw = _yaw(message)
        pose_yaw_rate = self.yaw_rate_estimator.update(
            stamp_to_seconds(message.header.stamp), yaw)
        source_yaw_rate = (message.twist.twist.angular.z
                           if pose_yaw_rate is None else pose_yaw_rate)
        sample = WheelSample(
            message.pose.pose.position.x, message.pose.pose.position.y,
            yaw, message.twist.twist.linear.x, source_yaw_rate)
        injected = self.model.apply(
            sample, mode, linear_scale=float(config[2]),
            angular_scale=float(config[3]), yaw_bias=float(config[4]),
            slip_bias_mps=float(config[5]), elapsed_sec=elapsed)
        if injected is None:
            self._publish_status(mode, elapsed, True)
            return
        output = copy.deepcopy(message)
        output.pose.pose.position.x = injected.x
        output.pose.pose.position.y = injected.y
        _set_planar_yaw(output, injected.yaw)
        output.twist.twist.linear.x = injected.linear_x
        output.twist.twist.angular.z = injected.angular_z
        self.publisher.publish(output)
        self._publish_status(mode, elapsed, False)

    def _publish_status(self, mode: str, elapsed: float,
                        dropped: bool) -> None:
        status = String()
        status.data = json.dumps({
            'enabled': bool(self.get_parameter('enabled').value),
            'mode': mode, 'elapsed_sec': elapsed, 'dropped': dropped,
            'injection_level': 'message',
        }, separators=(',', ':'), sort_keys=True)
        self.status_publisher.publish(status)


def main(args: Optional[list[str]] = None) -> None:
    """Run the wheel-odometry fault injector node."""
    rclpy.init(args=args)
    node = WheelFaultInjector()
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
