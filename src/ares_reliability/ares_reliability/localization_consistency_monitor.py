#!/usr/bin/env python3
"""Timestamped AMCL relative-motion integrity monitor using TF2."""

import json
import math
from typing import Optional, Tuple

from ares_reliability.diagnostics import diagnostic_level, MonitorDiagnostics
from ares_reliability.time_sync import (
    CacheTimestampAligner,
    interpolate_pose2d,
    shortest_angle_difference,
    stamp_to_seconds,
    SyncResult,
)
from ares_reliability.trust_model import HystereticTrustEvaluator
from geometry_msgs.msg import PoseWithCovarianceStamped
import message_filters
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformException, TransformListener


Pose2D = Tuple[float, float, float]


def _yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def _odom_pose(message: Odometry) -> Pose2D:
    pose = message.pose.pose
    q = pose.orientation
    return (pose.position.x, pose.position.y, _yaw(q.x, q.y, q.z, q.w))


class LocalizationConsistencyMonitor(Node):
    """Compare AMCL and wheel relative motion without treating them independent."""

    def __init__(self) -> None:
        super().__init__('localization_consistency_monitor')
        self.localization_topic = str(self.declare_parameter(
            'localization_topic', '/amcl_pose').value)
        self.odometry_topic = str(self.declare_parameter(
            'odometry_topic', '/ares/odom').value)
        self.map_frame = str(self.declare_parameter(
            'map_frame', 'map').value)
        self.odom_frame = str(self.declare_parameter(
            'odom_frame', 'odom').value)
        self.max_sync_error = float(self.declare_parameter(
            'max_sync_error_sec', 0.20).value)
        history_duration = float(self.declare_parameter(
            'history_duration_sec', 5.0).value)
        expected_rate = float(self.declare_parameter(
            'expected_odometry_rate_hz', 50.0).value)
        self.timeout = float(self.declare_parameter(
            'localization_timeout_sec', 2.0).value)
        self.startup_grace = float(self.declare_parameter(
            'startup_grace_sec', 2.0).value)
        self.tf_timeout = float(self.declare_parameter(
            'tf_timeout_sec', 0.1).value)
        self.trust = HystereticTrustEvaluator(
            window_size=int(self.declare_parameter('window_size', 5).value),
            degraded_enter=float(self.declare_parameter(
                'degraded_enter_m', 0.20).value),
            untrusted_enter=float(self.declare_parameter(
                'untrusted_enter_m', 0.75).value),
            degraded_exit=float(self.declare_parameter(
                'degraded_exit_m', 0.40).value),
            healthy_exit=float(self.declare_parameter(
                'healthy_exit_m', 0.10).value),
            degrade_persistence=int(self.declare_parameter(
                'degrade_persistence', 5).value),
            untrusted_persistence=int(self.declare_parameter(
                'untrusted_persistence', 5).value),
            recovery_persistence=int(self.declare_parameter(
                'recovery_persistence', 10).value),
        )
        cache_size = max(5, math.ceil(history_duration * expected_rate) + 2)
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, self.odometry_topic, 50)
        self.odom_cache = message_filters.Cache(self.odom_sub, cache_size,
                                                allow_headerless=False)
        self.aligner = CacheTimestampAligner(
            self.odom_cache, _odom_pose, self.max_sync_error,
            history_duration, interpolate_pose2d,
            clock_mismatch_tolerance_sec=max(5.0, history_duration * 2.0))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.previous_localization: Optional[Pose2D] = None
        self.previous_odometry: Optional[Pose2D] = None
        self.previous_map_odom_yaw: Optional[float] = None
        self.last_localization_receive: Optional[float] = None
        self.start_sec = self.get_clock().now().nanoseconds / 1.0e9

        self.create_subscription(PoseWithCovarianceStamped,
                                 self.localization_topic,
                                 self.localization_callback, 10)
        self.residual_pub = self.create_publisher(
            Float64, '/ares/residual/localization_odom', 10)
        self.rolling_pub = self.create_publisher(
            Float64, '/ares/residual/localization_odom_rolling', 10)
        self.trust_pub = self.create_publisher(
            String, '/ares/trust/localization', 10)
        self.evidence_pub = self.create_publisher(
            String, '/ares/evidence/localization', 10)
        self.diagnostics = MonitorDiagnostics(
            self, 'ARES AMCL-motion consistency',
            '/ares/diagnostics/localization')
        self.create_timer(0.5, self.check_timeout)

    def localization_callback(self,
                              message: PoseWithCovarianceStamped) -> None:
        """Evaluate a timestamped AMCL interval in compatible map axes."""
        now = self.get_clock().now().nanoseconds / 1.0e9
        self.last_localization_receive = now
        pose = message.pose.pose
        q = pose.orientation
        localization = (pose.position.x, pose.position.y,
                        _yaw(q.x, q.y, q.z, q.w))
        if any(not math.isfinite(value) for value in localization):
            self._publish(None, None, 'non-finite AMCL pose')
            return
        stamp_sec = stamp_to_seconds(message.header.stamp)
        reset_before = self.aligner.reset_count
        sync = self.aligner.lookup(stamp_sec, clock_sec=now,
                                   interpolate=True)
        if self.aligner.reset_count != reset_before:
            self._reset_interval()
            self.trust.reset()
        if not sync.accepted:
            self._publish(None, sync, sync.reason.value)
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException as error:
            self._publish(None, sync, f'TF_UNAVAILABLE: {error}')
            return
        tq = transform.transform.rotation
        map_odom_yaw = _yaw(tq.x, tq.y, tq.z, tq.w)
        odometry = sync.value
        assert odometry is not None
        covariance = list(message.pose.covariance)
        variances = (covariance[0], covariance[7], covariance[35])
        covariance_valid = all(math.isfinite(value) and value > 0.0
                               for value in variances)
        if (self.previous_localization is None or
                self.previous_odometry is None or
                self.previous_map_odom_yaw is None):
            self.previous_localization = localization
            self.previous_odometry = odometry
            self.previous_map_odom_yaw = map_odom_yaw
            self.trust.evaluate_explicit_state('HEALTHY')
            self._publish(None, sync, 'relative-motion reference initialized',
                          covariance_valid=covariance_valid,
                          covariance=variances)
            return
        loc_delta = (localization[0] - self.previous_localization[0],
                     localization[1] - self.previous_localization[1])
        odom_delta = (odometry[0] - self.previous_odometry[0],
                      odometry[1] - self.previous_odometry[1])
        angle = self.previous_map_odom_yaw
        odom_map_delta = (
            math.cos(angle) * odom_delta[0] -
            math.sin(angle) * odom_delta[1],
            math.sin(angle) * odom_delta[0] +
            math.cos(angle) * odom_delta[1],
        )
        translation_residual = math.hypot(
            loc_delta[0] - odom_map_delta[0],
            loc_delta[1] - odom_map_delta[1])
        loc_yaw_delta = shortest_angle_difference(
            localization[2], self.previous_localization[2])
        odom_yaw_delta = shortest_angle_difference(
            odometry[2], self.previous_odometry[2])
        yaw_residual = abs(shortest_angle_difference(
            loc_yaw_delta, odom_yaw_delta))
        self.previous_localization = localization
        self.previous_odometry = odometry
        self.previous_map_odom_yaw = map_odom_yaw
        evaluation = self.trust.add_residual(translation_residual)
        output = Float64()
        output.data = translation_residual
        self.residual_pub.publish(output)
        output = Float64()
        output.data = evaluation.rolling_mean or 0.0
        self.rolling_pub.publish(output)
        self._publish(translation_residual, sync, 'relative-motion residual',
                      yaw_residual, covariance_valid, variances)

    def check_timeout(self) -> None:
        """AMCL absence is UNAVAILABLE, never UNTRUSTED."""
        now = self.get_clock().now().nanoseconds / 1.0e9
        if self.last_localization_receive is None:
            if now - self.start_sec >= self.startup_grace:
                self._publish(None, None, 'AMCL_NOT_LAUNCHED')
            return
        age = now - self.last_localization_receive
        if age < 0.0:
            self.last_localization_receive = None
            self._reset_interval()
            self.trust.reset()
        elif age >= self.timeout:
            self._publish(None, None, 'AMCL_STALE')

    def _reset_interval(self) -> None:
        self.previous_localization = None
        self.previous_odometry = None
        self.previous_map_odom_yaw = None

    def _publish(self, residual: Optional[float],
                 sync: Optional[SyncResult], reason: str,
                 yaw_residual: Optional[float] = None,
                 covariance_valid: Optional[bool] = None,
                 covariance: Optional[tuple] = None) -> None:
        unavailable = (residual is None and reason not in (
            'relative-motion reference initialized',))
        if unavailable:
            self.trust.mark_unavailable(reason)
        state = self.trust.current_state
        trust = String()
        trust.data = state
        self.trust_pub.publish(trust)
        payload = {
            'sensor': 'localization', 'status': state,
            'trust_state': state, 'reason': reason,
            'test_id': 'localization_motion',
            'dependencies': ['localization', 'odometry'],
            'correlated': True, 'independence_weight': 0.5,
            'evidence_state': {'HEALTHY': 'CONSISTENT',
                               'DEGRADED': 'DEGRADED',
                               'UNTRUSTED': 'INCONSISTENT'}.get(
                                   state, 'UNAVAILABLE'),
            'residual': residual, 'rolling_mean': self.trust.rolling_mean,
            'yaw_residual_rad': yaw_residual,
            'amcl_covariance_valid': covariance_valid,
            'amcl_variances': covariance,
            'tf_timestamped': reason != 'TF_UNAVAILABLE',
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
        }
        evidence = String()
        evidence.data = json.dumps(
            payload, separators=(',', ':'), sort_keys=True)
        self.evidence_pub.publish(evidence)
        self.diagnostics.publish(payload, diagnostic_level(state), reason)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LocalizationConsistencyMonitor()
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
