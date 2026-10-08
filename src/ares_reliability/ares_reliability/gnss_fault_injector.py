#!/usr/bin/env python3
"""Inject deterministic GNSS faults without modifying the raw bridge topic."""

import copy
import json
import math
from typing import Any, Optional

from ares_reliability.core import (
    apply_meter_offsets,
    effective_offsets,
    VALID_FAULT_MODES,
)
from ares_reliability.fault_models import (
    DelayQueue,
    deterministic_uniform,
    gnss_stochastic_offsets,
)
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


def _set_stamp(message: NavSatFix, timestamp_sec: float) -> None:
    """Set a ROS stamp from finite non-negative seconds."""
    timestamp = max(0.0, timestamp_sec)
    seconds = int(timestamp)
    nanoseconds = int(round((timestamp - seconds) * 1.0e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds = 0
    message.header.stamp.sec = seconds
    message.header.stamp.nanosec = nanoseconds


def _stamp_seconds(message: NavSatFix) -> float:
    return (float(message.header.stamp.sec) +
            float(message.header.stamp.nanosec) / 1.0e9)


class GnssFaultInjector(Node):
    """Publish fault-injected ``/ares/gps`` from untouched raw GNSS."""

    def __init__(self) -> None:
        super().__init__('gnss_fault_injector')
        defaults = {
            'enabled': False, 'mode': 'none', 'east_offset_m': 0.0,
            'north_offset_m': 0.0, 'altitude_offset_m': 0.0,
            'drift_rate_east_mps': 0.0, 'drift_rate_north_mps': 0.0,
            'dropout_every_n': 1, 'additional_noise_stddev_m': 0.0,
            'seed': 0, 'freeze_timestamp': False, 'delay_sec': 0.0,
            'timestamp_jitter_sec': 0.0, 'out_of_order_every_n': 10,
            'spike_every_n': 10, 'spike_east_m': 5.0,
            'spike_north_m': 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.subscription = self.create_subscription(
            NavSatFix, '/ares/gps_raw', self.gps_callback, 20)
        self.publisher = self.create_publisher(NavSatFix, '/ares/gps', 20)
        self.status_publisher = self.create_publisher(
            String, '/ares/gnss_fault_status', 20)
        self._attack_signature: Optional[tuple[Any, ...]] = None
        self._attack_started_sec = 0.0
        self._sample_index = 0
        self._frozen_coordinates: Optional[tuple[float, float, float]] = None
        self._frozen_stamp_sec: Optional[float] = None
        self._held_message: Optional[NavSatFix] = None
        self._delay_queue: DelayQueue[NavSatFix] = DelayQueue()
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.create_timer(0.01, self._drain_delay_queue)
        self.get_logger().info(
            'GNSS fault injector started: /ares/gps_raw -> /ares/gps')

    def _validate_parameters(self,
                             parameters: list[Parameter]) -> SetParametersResult:
        """Reject invalid dynamic settings without destabilizing the node."""
        count_names = {
            'dropout_every_n', 'out_of_order_every_n', 'spike_every_n'}
        nonnegative_names = {
            'additional_noise_stddev_m', 'delay_sec',
            'timestamp_jitter_sec'}
        numeric_names = {
            'east_offset_m', 'north_offset_m', 'altitude_offset_m',
            'drift_rate_east_mps', 'drift_rate_north_mps', 'spike_east_m',
            'spike_north_m', *nonnegative_names,
        }
        for parameter in parameters:
            if (parameter.name == 'mode' and
                    parameter.value not in VALID_FAULT_MODES):
                return SetParametersResult(
                    successful=False,
                    reason='mode must be one of: ' +
                    ', '.join(VALID_FAULT_MODES))
            if parameter.name in count_names and (
                    not isinstance(parameter.value, int) or
                    parameter.value < 1):
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} must be an integer >= 1')
            if parameter.name in numeric_names:
                if (not isinstance(parameter.value, (float, int)) or
                        not math.isfinite(float(parameter.value))):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be finite numeric')
                if (parameter.name in nonnegative_names and
                        float(parameter.value) < 0.0):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be non-negative')
        return SetParametersResult(successful=True)

    def _configuration(self) -> tuple[Any, ...]:
        names = (
            'enabled', 'mode', 'east_offset_m', 'north_offset_m',
            'altitude_offset_m', 'drift_rate_east_mps',
            'drift_rate_north_mps', 'dropout_every_n',
            'additional_noise_stddev_m', 'seed', 'freeze_timestamp',
            'delay_sec', 'timestamp_jitter_sec', 'out_of_order_every_n',
            'spike_every_n', 'spike_east_m', 'spike_north_m')
        return tuple(self.get_parameter(name).value for name in names)

    def _elapsed_for(self, signature: tuple[Any, ...], now_sec: float) -> float:
        if (signature != self._attack_signature or
                now_sec < self._attack_started_sec):
            self._attack_signature = signature
            self._attack_started_sec = now_sec
            self._sample_index = 0
            self._frozen_coordinates = None
            self._frozen_stamp_sec = None
            self._held_message = None
            self._delay_queue.clear()
        return max(0.0, now_sec - self._attack_started_sec)

    def _publish_status(self, enabled: bool, mode: str, elapsed_sec: float,
                        offsets: tuple[float, float, float], dropped: bool,
                        diagnostic_reason: str) -> None:
        status = String()
        status.data = json.dumps({
            'enabled': enabled, 'mode': mode, 'elapsed_sec': elapsed_sec,
            'east_offset_m': offsets[0], 'north_offset_m': offsets[1],
            'altitude_offset_m': offsets[2], 'dropped': dropped,
            'seed': int(self.get_parameter('seed').value),
            'diagnostic_reason': diagnostic_reason,
            'queued_messages': len(self._delay_queue),
        }, separators=(',', ':'), sort_keys=True)
        self.status_publisher.publish(status)

    def _drain_delay_queue(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1.0e9
        for message in self._delay_queue.pop_due(now_sec):
            self.publisher.publish(message)

    def gps_callback(self, msg: NavSatFix) -> None:
        """Apply the selected deterministic fault once per raw fix."""
        config = self._configuration()
        enabled = bool(config[0])
        configured_mode = str(config[1])
        mode = configured_mode if enabled else 'none'
        now_sec = self.get_clock().now().nanoseconds / 1.0e9
        elapsed_sec = self._elapsed_for(config, now_sec)
        sample_index = self._sample_index
        self._sample_index += 1
        offsets = effective_offsets(
            mode, elapsed_sec, float(config[2]), float(config[3]),
            float(config[4]), float(config[5]), float(config[6]))
        stochastic = gnss_stochastic_offsets(
            mode, int(config[9]), sample_index, float(config[8]),
            int(config[14]), float(config[15]), float(config[16]))
        offsets = (offsets[0] + stochastic[0],
                   offsets[1] + stochastic[1], offsets[2])
        dropped = mode == 'dropout' and (
            (sample_index + 1) % int(config[7]) == 0)
        if dropped:
            self._publish_status(
                enabled, mode, elapsed_sec, offsets, True, 'DROPOUT')
            return
        output = copy.deepcopy(msg)
        coordinates = apply_meter_offsets(
            msg.latitude, msg.longitude, msg.altitude, *offsets)
        if mode == 'freeze':
            if self._frozen_coordinates is None:
                self._frozen_coordinates = coordinates
            coordinates = self._frozen_coordinates
        output.latitude, output.longitude, output.altitude = coordinates
        source_stamp = _stamp_seconds(output)
        if mode == 'timestamp_jitter':
            source_stamp += deterministic_uniform(
                int(config[9]), sample_index, float(config[12]))
            _set_stamp(output, source_stamp)
        elif mode == 'zero_timestamp':
            _set_stamp(output, 0.0)
        elif mode == 'backwards_timestamp':
            if sample_index > 0 and sample_index % int(config[13]) == 0:
                source_stamp -= max(0.001, float(config[12]) or 0.1)
            _set_stamp(output, source_stamp)
        elif (mode == 'frozen_timestamp' or
              (mode == 'freeze' and bool(config[10]))):
            if self._frozen_stamp_sec is None:
                self._frozen_stamp_sec = source_stamp
            _set_stamp(output, self._frozen_stamp_sec)
        if mode == 'delay':
            self._delay_queue.push(now_sec, float(config[11]), output)
            self._publish_status(
                enabled, mode, elapsed_sec, offsets, False, 'DELAY_QUEUED')
            return
        if mode == 'out_of_order':
            every_n = int(config[13])
            if self._held_message is not None:
                self.publisher.publish(output)
                self.publisher.publish(self._held_message)
                self._held_message = None
                reason = 'OUT_OF_ORDER_RELEASED'
            elif (sample_index + 1) % every_n == 0:
                self._held_message = output
                reason = 'OUT_OF_ORDER_HELD'
            else:
                self.publisher.publish(output)
                reason = 'PASSTHROUGH'
            self._publish_status(
                enabled, mode, elapsed_sec, offsets, False, reason)
            return
        self.publisher.publish(output)
        self._publish_status(
            enabled, mode, elapsed_sec, offsets, False, 'PUBLISHED')


def main(args: Optional[list[str]] = None) -> None:
    """Run the GNSS fault injector node."""
    rclpy.init(args=args)
    node = GnssFaultInjector()
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
