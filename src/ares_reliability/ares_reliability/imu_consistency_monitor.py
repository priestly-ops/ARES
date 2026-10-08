#!/usr/bin/env python3
"""IMU yaw-rate consistency against independent raw wheel odometry."""

import json
import math
from typing import Optional

from ares_reliability.diagnostics import diagnostic_level, MonitorDiagnostics
from ares_reliability.sensor_consistency import yaw_rate_residual
from ares_reliability.time_sync import (
    CacheTimestampAligner,
    interpolate_scalar,
    stamp_to_seconds,
    SyncResult,
)
from ares_reliability.trust_model import HystereticTrustEvaluator
import message_filters
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String


def _odom_yaw_rate(message: Odometry) -> float:
    return message.twist.twist.angular.z


class ImuConsistencyMonitor(Node):
    """Monitor planar rotation without using circular filtered odometry."""

    def __init__(self) -> None:
        super().__init__('imu_consistency_monitor')
        self.imu_topic = str(self.declare_parameter(
            'imu_topic', '/ares/imu').value)
        self.odometry_topic = str(self.declare_parameter(
            'odometry_topic', '/ares/odom').value)
        if self.odometry_topic == '/odometry/filtered':
            raise ValueError('IMU evidence must not use filtered odometry')
        self.odometry_yaw_rate_scale = float(self.declare_parameter(
            'odometry_yaw_rate_scale', 1.0).value)
        if (not math.isfinite(self.odometry_yaw_rate_scale) or
                self.odometry_yaw_rate_scale <= 0.0):
            raise ValueError('odometry_yaw_rate_scale must be positive')
        self.max_sync_error = float(self.declare_parameter(
            'max_sync_error_sec', 0.05).value)
        history_duration = float(self.declare_parameter(
            'history_duration_sec', 2.0).value)
        expected_rate = float(self.declare_parameter(
            'expected_odometry_rate_hz', 50.0).value)
        self.interpolate_odometry = bool(self.declare_parameter(
            'interpolate_odometry', True).value)
        self.timeout = float(self.declare_parameter(
            'imu_timeout_sec', 0.25).value)
        self.startup_grace = float(self.declare_parameter(
            'startup_grace_sec', 1.0).value)
        self.trust = HystereticTrustEvaluator(
            window_size=int(self.declare_parameter('window_size', 20).value),
            degraded_enter=float(self.declare_parameter(
                'degraded_enter_radps', 0.15).value),
            untrusted_enter=float(self.declare_parameter(
                'untrusted_enter_radps', 0.40).value),
            degraded_exit=float(self.declare_parameter(
                'degraded_exit_radps', 0.25).value),
            healthy_exit=float(self.declare_parameter(
                'healthy_exit_radps', 0.10).value),
            degrade_persistence=int(self.declare_parameter(
                'degrade_persistence', 10).value),
            untrusted_persistence=int(self.declare_parameter(
                'untrusted_persistence', 10).value),
            recovery_persistence=int(self.declare_parameter(
                'recovery_persistence', 30).value),
        )
        cache_size = max(5, math.ceil(history_duration * expected_rate) + 2)
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, self.odometry_topic, 50)
        self.odom_cache = message_filters.Cache(self.odom_sub, cache_size,
                                                allow_headerless=False)
        self.aligner = CacheTimestampAligner(
            self.odom_cache, _odom_yaw_rate, self.max_sync_error,
            history_duration, interpolate_scalar,
            clock_mismatch_tolerance_sec=max(5.0, history_duration * 2.0))
        self.start_sec = self.get_clock().now().nanoseconds / 1.0e9
        self.last_imu_receive_sec: Optional[float] = None
        self.last_imu_covariance: Optional[float] = None

        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 50)
        self.residual_pub = self.create_publisher(
            Float64, '/ares/residual/imu_odom', 10)
        self.rolling_pub = self.create_publisher(
            Float64, '/ares/residual/imu_odom_rolling', 10)
        self.trust_pub = self.create_publisher(
            String, '/ares/trust/imu', 10)
        self.evidence_pub = self.create_publisher(
            String, '/ares/evidence/imu', 10)
        self.diagnostics = MonitorDiagnostics(
            self, 'ARES IMU-wheel consistency', '/ares/diagnostics/imu')
        self.create_timer(0.1, self.check_timeout)
        self.get_logger().info(
            f'IMU monitor: {self.imu_topic} vs raw {self.odometry_topic}; '
            f'message_filters.Cache={cache_size}')

    def imu_callback(self, message: Imu) -> None:
        """Evaluate one IMU yaw-rate sample at its header timestamp."""
        now = self.get_clock().now().nanoseconds / 1.0e9
        self.last_imu_receive_sec = now
        imu_rate = message.angular_velocity.z
        if not math.isfinite(imu_rate):
            self._publish(None, None, 'non-finite IMU yaw rate')
            return
        variance = message.angular_velocity_covariance[8]
        self.last_imu_covariance = variance if (
            math.isfinite(variance) and variance > 0.0
        ) else None
        source_stamp = stamp_to_seconds(message.header.stamp)
        reset_before = self.aligner.reset_count
        sync = self.aligner.lookup(source_stamp, clock_sec=now,
                                   interpolate=self.interpolate_odometry)
        if self.aligner.reset_count != reset_before:
            self.trust.reset()
        if not sync.accepted:
            self._publish(None, sync, sync.reason.value)
            return
        odom_rate = sync.value
        assert odom_rate is not None
        if not math.isfinite(odom_rate):
            self._publish(None, sync, 'non-finite wheel yaw rate')
            return
        residual = yaw_rate_residual(
            imu_rate, odom_rate, self.odometry_yaw_rate_scale)
        evaluation = self.trust.add_residual(residual)
        output = Float64()
        output.data = residual
        self.residual_pub.publish(output)
        output = Float64()
        output.data = evaluation.rolling_mean or 0.0
        self.rolling_pub.publish(output)
        self._publish(residual, sync, 'aligned yaw-rate residual', imu_rate,
                      odom_rate)

    def check_timeout(self) -> None:
        """Publish UNAVAILABLE before first data, then UNTRUSTED on dropout."""
        now = self.get_clock().now().nanoseconds / 1.0e9
        if self.last_imu_receive_sec is None:
            if now - self.start_sec >= self.startup_grace:
                self._publish(None, None, 'IMU_NOT_AVAILABLE')
            return
        age = now - self.last_imu_receive_sec
        if age < 0.0:
            self.last_imu_receive_sec = None
            self.trust.reset()
            return
        if age < self.timeout:
            return
        self.trust.evaluate_explicit_state(
            'UNTRUSTED', reason='IMU stream timeout')
        self._publish(None, None, 'IMU_TIMEOUT')

    def _publish(self, residual: Optional[float],
                 sync: Optional[SyncResult], reason: str,
                 imu_rate: Optional[float] = None,
                 odom_rate: Optional[float] = None) -> None:
        unavailable = reason in (
            'IMU_NOT_AVAILABLE', 'ZERO_STAMP', 'CLOCK_MISMATCH',
            'NO_HISTORY', 'TOO_OLD', 'TOO_FAR',
            'non-finite IMU yaw rate', 'non-finite wheel yaw rate')
        if unavailable:
            self.trust.mark_unavailable(reason)
        state = self.trust.current_state
        trust = String()
        trust.data = state
        self.trust_pub.publish(trust)
        payload = {
            'sensor': 'imu', 'status': state, 'trust_state': state,
            'reason': reason, 'test_id': 'imu_wheel',
            'dependencies': ['imu', 'odometry'], 'correlated': False,
            'evidence_state': {'HEALTHY': 'CONSISTENT',
                               'DEGRADED': 'DEGRADED',
                               'UNTRUSTED': 'INCONSISTENT'}.get(
                                   state, 'UNAVAILABLE'),
            'residual': residual, 'rolling_mean': self.trust.rolling_mean,
            'imu_yaw_rate_radps': imu_rate,
            'odom_yaw_rate_radps': odom_rate,
            'odometry_yaw_rate_scale': self.odometry_yaw_rate_scale,
            'scaled_odom_yaw_rate_radps': (
                None if odom_rate is None else
                self.odometry_yaw_rate_scale * odom_rate),
            'imu_yaw_rate_variance': self.last_imu_covariance,
            'normalized_residual': None,
            'stale': sync is None or not sync.accepted,
            'sync_reason': reason if sync is None else sync.reason.value,
            'source_timestamp_sec': (
                None if sync is None else sync.source_timestamp_sec),
            'matched_timestamp_sec': (
                None if sync is None else sync.matched_timestamp_sec),
            'signed_time_difference_sec': (
                None if sync is None else sync.signed_time_difference_sec),
            'sync_error_sec': (
                None if sync is None else sync.absolute_sync_error_sec),
            'interpolation_mode': (
                'NONE' if sync is None else sync.interpolation_mode),
            'oldest_buffered_timestamp_sec': (
                None if sync is None else sync.oldest_timestamp_sec),
            'newest_buffered_timestamp_sec': (
                None if sync is None else sync.newest_timestamp_sec),
        }
        evidence = String()
        evidence.data = json.dumps(
            payload, separators=(',', ':'), sort_keys=True)
        self.evidence_pub.publish(evidence)
        self.diagnostics.publish(payload, diagnostic_level(state), reason)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ImuConsistencyMonitor()
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
