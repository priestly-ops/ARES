#!/usr/bin/env python3
"""Timestamp-aligned GNSS versus filtered-motion consistency monitor."""

import json
import math
from collections import deque
from typing import Any, Deque, Optional, Tuple

from ares_reliability.core import RollingTrustEvaluator, TrustEvaluation
from ares_reliability.diagnostics import diagnostic_level, MonitorDiagnostics
from ares_reliability.gnss_freeze_detector import (
    freeze_trust_candidate,
    FreezeEvaluation,
    GnssFreezeDetector,
)
from ares_reliability.sensor_consistency import (
    geodetic_distance_m,
    rotate_enu,
    vector_displacement_residual,
)
from ares_reliability.time_sync import (
    CacheTimestampAligner,
    interpolate_pair,
    SourceTimestampValidator,
    stamp_to_seconds,
    SyncReason,
    SyncResult,
)
from ares_reliability.trust_model import (
    HystereticTrustEvaluator,
    SensorTrustEvaluation,
)
from diagnostic_msgs.msg import DiagnosticStatus
import message_filters
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Float64MultiArray, String

EARTH_RADIUS_M = 6371000.0


def _odom_xy(message: Odometry) -> Tuple[float, float]:
    return (message.pose.pose.position.x, message.pose.pose.position.y)


class ConsistencyMonitor(Node):
    """Preserve Week 2 outputs while adding timestamp-aware GNSS trust."""

    def __init__(self) -> None:
        super().__init__('consistency_monitor')
        window_size = int(self.declare_parameter('window_size', 5).value)
        healthy_threshold = float(
            self.declare_parameter('healthy_threshold_m', 1.5).value)
        fault_threshold = float(
            self.declare_parameter('fault_threshold_m', 3.0).value)
        self.fault_threshold_m = fault_threshold
        self.step_fault_latch_enabled = bool(self.declare_parameter(
            'step_fault_latch_enabled', False).value)
        self.step_fault_release_persistence = int(self.declare_parameter(
            'step_fault_release_persistence', 5).value)
        persistence = int(
            self.declare_parameter('required_consecutive', 5).value)
        self.gnss_topic = str(
            self.declare_parameter('gnss_topic', '/ares/gps').value)
        self.odometry_topic = str(self.declare_parameter(
            'odometry_topic', '/odometry/filtered').value)
        self.gnss_timeout_sec = float(
            self.declare_parameter('gnss_timeout_sec', 1.0).value)
        self.dropout_period = float(self.declare_parameter(
            'dropout_check_period_sec', 0.2).value)
        self.max_sync_error_sec = float(self.declare_parameter(
            'max_sync_error_sec', 0.15).value)
        history_duration = float(self.declare_parameter(
            'history_duration_sec', 5.0).value)
        self.interpolate_odometry = bool(self.declare_parameter(
            'interpolate_odometry', False).value)
        expected_odom_rate = float(self.declare_parameter(
            'expected_odometry_rate_hz', 50.0).value)
        self.log_every = int(self.declare_parameter(
            'diagnostic_log_every_n', 10).value)
        self.initialization_mode = str(self.declare_parameter(
            'initialization_mode', 'relative').value)
        if self.initialization_mode not in ('relative', 'configured_anchor'):
            raise ValueError(
                'initialization_mode must be relative or configured_anchor')
        self.trusted_reference = (
            float(self.declare_parameter(
                'trusted_reference_latitude', 39.7392).value),
            float(self.declare_parameter(
                'trusted_reference_longitude', -104.9903).value),
        )
        self.max_initial_reference_error_m = float(self.declare_parameter(
            'max_initial_reference_error_m', 2.5).value)
        self.gnss_to_odom_yaw_deg = float(self.declare_parameter(
            'gnss_to_odom_yaw_deg', 0.0).value)

        self.residual_mode = str(self.declare_parameter(
            'residual_mode', 'startup_relative').value)
        if self.residual_mode not in ('startup_relative', 'window_vector'):
            raise ValueError(
                'residual_mode must be startup_relative or window_vector')

        self.residual_window_sec = float(self.declare_parameter(
            'residual_window_sec', 2.0).value)
        if self.residual_window_sec <= 0.0:
            raise ValueError('residual_window_sec must be positive')

        self.residual_samples: Deque[
            Tuple[float, Tuple[float, float], Tuple[float, float]]
        ] = deque()
        self.step_fault_anchor: Optional[
            Tuple[Tuple[float, float], Tuple[float, float]]
        ] = None
        self.step_fault_latch_confirmed = False
        self.step_fault_release_count = 0
        self.step_fault_vector = None

        self.trust = RollingTrustEvaluator(
            window_size, healthy_threshold, fault_threshold, persistence)
        self.healthy_exit_m = float(self.declare_parameter(
            'healthy_exit_m', 1.2).value)
        self.sensor_trust = HystereticTrustEvaluator(
            window_size=window_size,
            degraded_enter=healthy_threshold,
            untrusted_enter=fault_threshold,
            degraded_exit=float(self.declare_parameter(
                'degraded_exit_m', 2.0).value),
            healthy_exit=self.healthy_exit_m,
            degrade_persistence=int(self.declare_parameter(
                'degrade_persistence', persistence).value),
            untrusted_persistence=int(self.declare_parameter(
                'untrusted_persistence', persistence).value),
            recovery_persistence=int(self.declare_parameter(
                'recovery_persistence', 10).value),
        )
        self.freeze_detection_enabled = bool(self.declare_parameter(
            'freeze_detection_enabled', False).value)
        self.freeze_detector = GnssFreezeDetector(
            window_sec=float(self.declare_parameter(
                'freeze_window_sec', 5.0).value),
            minimum_motion_m=float(self.declare_parameter(
                'freeze_minimum_motion_m', 0.5).value),
            maximum_gnss_displacement_m=float(self.declare_parameter(
                'freeze_maximum_gnss_displacement_m', 0.15).value),
            confirmation_sec=float(self.declare_parameter(
                'freeze_confirmation_sec', 0.5).value),
        )
        cache_size = max(5, math.ceil(history_duration * expected_odom_rate) + 2)
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, self.odometry_topic, 20)
        self.odom_cache = message_filters.Cache(
            self.odom_sub, cache_size=cache_size, allow_headerless=False)
        self.aligner = CacheTimestampAligner(
            self.odom_cache, _odom_xy, self.max_sync_error_sec,
            history_duration, interpolate_pair,
            clock_mismatch_tolerance_sec=max(5.0, history_duration * 2.0))
        self.timestamp_validator = SourceTimestampValidator(
            max_equal_stamps=1)
        self.gps_ref: Optional[Tuple[float, float]] = None
        self.odom_ref: Optional[Tuple[float, float]] = None
        self.last_gps_receive_sec: Optional[float] = None
        self.dropout_active = False
        self.diagnostic_count = 0

        self.create_subscription(NavSatFix, self.gnss_topic,
                                 self.gps_callback, 10)
        self.residual_pub = self.create_publisher(
            Float64, '/ares/gnss_odom_residual', 10)
        self.rolling_pub = self.create_publisher(
            Float64, '/ares/gnss_odom_rolling_mean', 10)
        self.legacy_trust_pub = self.create_publisher(
            String, '/ares/trust_state', 10)
        self.sensor_trust_pub = self.create_publisher(
            String, '/ares/trust/gnss', 10)
        self.week3_residual_pub = self.create_publisher(
            Float64, '/ares/residual/gnss_odom', 10)
        self.week3_rolling_pub = self.create_publisher(
            Float64, '/ares/residual/gnss_odom_rolling', 10)
        self.legacy_diagnostics_pub = self.create_publisher(
            Float64MultiArray, '/ares/gnss_odom_diagnostics', 10)
        self.evidence_pub = self.create_publisher(
            String, '/ares/evidence/gnss', 10)
        self.diagnostics = MonitorDiagnostics(
            self, 'ARES GNSS consistency', '/ares/diagnostics/gnss')
        self.create_timer(self.dropout_period, self.check_gnss_timeout)
        self.get_logger().info(
            f'GNSS monitor: {self.gnss_topic} vs {self.odometry_topic}; '
            f'message_filters.Cache={cache_size}, max_sync='
            f'{self.max_sync_error_sec:.3f}s')

    def gps_to_local(self, latitude: float,
                     longitude: float) -> Tuple[float, float]:
        """Convert a fix to the Week 2 local tangent approximation."""
        if self.gps_ref is None:
            raise RuntimeError('GNSS reference has not been initialized')
        lat0, lon0 = self.gps_ref
        north = EARTH_RADIUS_M * math.radians(latitude - lat0)
        east = (EARTH_RADIUS_M * math.cos(math.radians(lat0)) *
                math.radians(longitude - lon0))
        return rotate_enu(east, north, self.gnss_to_odom_yaw_deg)

    def gps_callback(self, message: NavSatFix) -> None:
        """Evaluate one operational GNSS fix; raw validation truth is unused."""
        now = self.get_clock().now().nanoseconds / 1.0e9
        self.last_gps_receive_sec = now
        self.dropout_active = False
        if not all(math.isfinite(value) for value in
                   (message.latitude, message.longitude)):
            self._publish_unavailable('non-finite GNSS position', None)
            return
        source_stamp = stamp_to_seconds(message.header.stamp)
        stamp_reason = self.timestamp_validator.observe(source_stamp)
        if stamp_reason != SyncReason.OK:
            self._publish_unavailable(stamp_reason.value, None)
            return
        reset_before = self.aligner.reset_count
        sync = self.aligner.lookup(source_stamp, clock_sec=now,
                                   interpolate=self.interpolate_odometry)
        if self.aligner.reset_count != reset_before:
            self._reset_references()
        if not sync.accepted:
            self._publish_unavailable(sync.reason.value, sync)
            return
        aligned_odom = sync.value
        assert aligned_odom is not None
        if self.gps_ref is None:
            reference = (message.latitude, message.longitude)
            if self.initialization_mode == 'configured_anchor':
                initial_error = geodetic_distance_m(
                    self.trusted_reference, reference)
                if initial_error > self.max_initial_reference_error_m:
                    legacy = self.trust.evaluate_explicit_state('FAULT')
                    sensor = self.sensor_trust.force_state(
                        'UNTRUSTED', rolling_mean=initial_error,
                        reason='configured anchor rejected initial GNSS')
                    self._handle_legacy_transition(legacy)
                    self._handle_sensor_transition(sensor)
                    self._publish_states()
                    self._publish_diagnostic(
                        initial_error, sync,
                        'INITIAL_REFERENCE_MISMATCH',
                        test_id='gnss_initialization',
                        dependencies=['gnss', 'configured_anchor'])
                    return
                reference = self.trusted_reference
            self.gps_ref = reference
            self.odom_ref = aligned_odom
            self.sensor_trust.evaluate_explicit_state('HEALTHY')
            self._publish_diagnostic(None, sync, 'reference initialized')
            return
        gps_xy = self.gps_to_local(message.latitude, message.longitude)
        assert self.odom_ref is not None
        odom_xy = (aligned_odom[0] - self.odom_ref[0],
                   aligned_odom[1] - self.odom_ref[1])
        if self.residual_mode == 'startup_relative':
            residual = math.hypot(
                gps_xy[0] - odom_xy[0],
                gps_xy[1] - odom_xy[1],
            )
        else:
            self.residual_samples.append(
                (source_stamp, gps_xy, odom_xy))
            cutoff = source_stamp - self.residual_window_sec
            while (
                len(self.residual_samples) > 2
                and self.residual_samples[1][0] <= cutoff
            ):
                self.residual_samples.popleft()

            oldest = self.residual_samples[0]
            if source_stamp - oldest[0] < self.residual_window_sec:
                window_residual = 0.0
            else:
                window_residual = vector_displacement_residual(
                    oldest[1],
                    gps_xy,
                    oldest[2],
                    odom_xy,
                )

            if (
                self.step_fault_latch_enabled
                and self.step_fault_anchor is None
                and window_residual >= self.fault_threshold_m
            ):
                self.step_fault_anchor = (oldest[1], oldest[2])
                self.step_fault_latch_confirmed = False
                self.step_fault_release_count = 0

                gps_dx = gps_xy[0] - oldest[1][0]
                gps_dy = gps_xy[1] - oldest[1][1]
                odom_dx = odom_xy[0] - oldest[2][0]
                odom_dy = odom_xy[1] - oldest[2][1]
                self.step_fault_vector = (
                    gps_dx - odom_dx,
                    gps_dy - odom_dy,
                )

                self.get_logger().warning(
                    'GNSS STEP LATCH: armed at '
                    f'{window_residual:.3f}m residual')

            if self.step_fault_anchor is not None:
                residual = vector_displacement_residual(
                    self.step_fault_anchor[0],
                    gps_xy,
                    self.step_fault_anchor[1],
                    odom_xy,
                )

                if (
                    self.step_fault_latch_confirmed
                    and self.step_fault_vector is not None
                ):
                    gps_dx = gps_xy[0] - oldest[1][0]
                    gps_dy = gps_xy[1] - oldest[1][1]
                    odom_dx = odom_xy[0] - oldest[2][0]
                    odom_dy = odom_xy[1] - oldest[2][1]

                    window_vector = (
                        gps_dx - odom_dx,
                        gps_dy - odom_dy,
                    )

                    step_x, step_y = self.step_fault_vector
                    win_x, win_y = window_vector

                    step_mag = (step_x * step_x + step_y * step_y) ** 0.5
                    win_mag = (win_x * win_x + win_y * win_y) ** 0.5

                    reverse_edge = False
                    if (
                        step_mag > 1e-6
                        and win_mag >= 0.8 * self.fault_threshold_m
                    ):
                        cosine = (
                            step_x * win_x + step_y * win_y
                        ) / (step_mag * win_mag)
                        reverse_edge = cosine <= -0.7

                    if reverse_edge:
                        self.step_fault_release_count += 1
                    else:
                        self.step_fault_release_count = 0

                    if (
                        self.step_fault_release_count
                        >= self.step_fault_release_persistence
                    ):
                        self.get_logger().info(
                            'GNSS STEP LATCH: reverse edge detected; '
                            f'residual={window_residual:.3f}m')
                        self.step_fault_anchor = None
                        self.step_fault_latch_confirmed = False
                        self.step_fault_release_count = 0
                        self.step_fault_vector = None

                        # Remove the reverse transition from the new
                        # reference window so it cannot immediately re-arm.
                        self.residual_samples.clear()
                        self.residual_samples.append(
                            (source_stamp, gps_xy, odom_xy))

                        # Recovery itself still goes through the existing
                        # hysteretic trust evaluator.
                        residual = 0.0
                else:
                    self.step_fault_release_count = 0
            else:
                residual = window_residual

        freeze = (
            self.freeze_detector.update(source_stamp, gps_xy, odom_xy)
            if self.freeze_detection_enabled else
            FreezeEvaluation('HEALTHY', None, None, None)
        )
        legacy = self.trust.add_residual(residual)
        freeze_candidate = freeze_trust_candidate(
            freeze.state, self.sensor_trust.current_state)
        if freeze_candidate is not None:
            sensor = self.sensor_trust.evaluate_explicit_state(
                freeze_candidate, residual, 'GNSS_FREEZE_CONFIRMED')
        else:
            sensor = self.sensor_trust.add_residual(residual)

        if self.step_fault_anchor is not None:
            if sensor.public_state == 'UNTRUSTED':
                self.step_fault_latch_confirmed = True
            elif (
                not self.step_fault_latch_confirmed
                and residual <= self.healthy_exit_m
            ):
                self.get_logger().info(
                    'GNSS STEP LATCH: disarmed before confirmation')
                self.step_fault_anchor = None
                self.step_fault_latch_confirmed = False
                self.step_fault_release_count = 0
                self.step_fault_vector = None
            elif (
                self.step_fault_latch_confirmed
                and sensor.public_state == 'HEALTHY'
            ):
                self.get_logger().info(
                    'GNSS STEP LATCH: cleared after recovery')
                self.step_fault_anchor = None
                self.step_fault_latch_confirmed = False
                self.step_fault_release_count = 0
                self.step_fault_vector = None

        reason = ('GNSS_FREEZE_' + freeze.state
                  if freeze.state != 'HEALTHY' else 'aligned residual')
        self._publish_numeric(
            gps_xy, odom_xy, residual, legacy, sensor, sync, reason, freeze)

    def _reset_references(self) -> None:
        self.gps_ref = None
        self.odom_ref = None
        self.trust.reset()
        self.sensor_trust.reset()
        self.timestamp_validator.reset()
        self.freeze_detector.reset()
        self.residual_samples.clear()
        self.step_fault_anchor = None
        self.step_fault_latch_confirmed = False
        self.step_fault_release_count = 0
        self.step_fault_vector = None

    def check_gnss_timeout(self) -> None:
        """Treat established-stream dropout as GNSS freshness evidence."""
        if self.last_gps_receive_sec is None:
            return
        now = self.get_clock().now().nanoseconds / 1.0e9
        age = now - self.last_gps_receive_sec
        if age < 0.0:
            self._reset_references()
            self.last_gps_receive_sec = None
            self._publish_unavailable('CLOCK_MISMATCH', None)
            return
        if age < self.gnss_timeout_sec:
            return
        self.dropout_active = True
        legacy = self.trust.evaluate_explicit_state('FAULT')
        sensor = self.sensor_trust.evaluate_explicit_state(
            'UNTRUSTED', reason='GNSS stream timeout')
        self._handle_legacy_transition(legacy)
        self._handle_sensor_transition(sensor)
        self._publish_states()
        payload = self._base_payload(None, None, 'GNSS_TIMEOUT')
        payload.update({'age_sec': age, 'test_id': 'gnss_freshness',
                        'dependencies': ['gnss'], 'evidence_state':
                        'INCONSISTENT'})
        self._publish_payload(payload, DiagnosticStatus.ERROR)

    def _publish_unavailable(self, reason: str,
                             sync: Optional[SyncResult]) -> None:
        evaluation = self.sensor_trust.mark_unavailable(reason)
        self._handle_sensor_transition(evaluation)
        self._publish_states()
        self._publish_diagnostic(None, sync, reason)

    def _publish_numeric(self, gps_xy: Tuple[float, float],
                         odom_xy: Tuple[float, float], residual: float,
                         legacy: TrustEvaluation,
                         sensor: SensorTrustEvaluation,
                         sync: SyncResult,
                         diagnostic_reason: str,
                         freeze: FreezeEvaluation) -> None:
        for publisher in (self.residual_pub, self.week3_residual_pub):
            message = Float64()
            message.data = residual
            publisher.publish(message)
        assert legacy.rolling_mean_m is not None
        message = Float64()
        message.data = legacy.rolling_mean_m
        self.rolling_pub.publish(message)
        assert sensor.rolling_mean is not None
        message = Float64()
        message.data = sensor.rolling_mean
        self.week3_rolling_pub.publish(message)
        diagnostics = Float64MultiArray()
        diagnostics.data = [*gps_xy, *odom_xy, residual,
                            legacy.rolling_mean_m,
                            sync.absolute_sync_error_sec or 0.0, 0.0]
        self.legacy_diagnostics_pub.publish(diagnostics)
        self._handle_legacy_transition(legacy)
        self._handle_sensor_transition(sensor)
        self._publish_states()
        self._publish_diagnostic(
            residual, sync, diagnostic_reason,
            additional_payload={
                'gnss_freeze_state': freeze.state,
                'gnss_freeze_odometry_displacement_m':
                    freeze.odometry_displacement_m,
                'gnss_freeze_position_displacement_m':
                    freeze.gnss_displacement_m,
                'gnss_freeze_detected_at_sec': freeze.detected_at_sec,
            })
        self.diagnostic_count += 1
        if self.diagnostic_count % self.log_every == 0:
            self.get_logger().info(
                f'GNSS residual={residual:.3f}m rolling='
                f'{sensor.rolling_mean:.3f}m state={sensor.public_state}')

    def _base_payload(self, residual: Optional[float],
                      sync: Optional[SyncResult], reason: str) -> dict:
        payload = {
            'sensor': 'gnss', 'status': self.sensor_trust.current_state,
            'trust_state': self.sensor_trust.current_state,
            'reason': reason, 'residual': residual,
            'rolling_mean': self.sensor_trust.rolling_mean,
            'stale': sync is None or not sync.accepted,
            'source_timestamp_sec': None,
            'matched_timestamp_sec': None,
            'signed_time_difference_sec': None,
            'sync_error_sec': None,
            'interpolation_mode': 'NONE',
            'oldest_buffered_timestamp_sec': None,
            'newest_buffered_timestamp_sec': None,
            'sync_reason': reason,
        }
        if sync is not None:
            payload.update({
                'source_timestamp_sec': sync.source_timestamp_sec,
                'matched_timestamp_sec': sync.matched_timestamp_sec,
                'signed_time_difference_sec':
                    sync.signed_time_difference_sec,
                'sync_error_sec': sync.absolute_sync_error_sec,
                'interpolation_mode': sync.interpolation_mode,
                'oldest_buffered_timestamp_sec': sync.oldest_timestamp_sec,
                'newest_buffered_timestamp_sec': sync.newest_timestamp_sec,
                'sync_reason': sync.reason.value,
            })
        return payload

    def _publish_diagnostic(
        self,
        residual: Optional[float],
        sync: Optional[SyncResult],
        reason: str,
        test_id: str = 'gnss_motion',
        dependencies: Optional[list[str]] = None,
        additional_payload: Optional[dict] = None,
    ) -> None:
        payload = self._base_payload(residual, sync, reason)
        payload.update({
            'test_id': test_id,
            'dependencies': dependencies or ['gnss', 'odometry'],
            'correlated': False,
            'evidence_state': self._evidence_state(),
        })
        if additional_payload:
            payload.update(additional_payload)
        self._publish_payload(payload,
                              diagnostic_level(self.sensor_trust.current_state))

    def _publish_payload(self, payload: dict, level: Any) -> None:
        evidence = String()
        evidence.data = json.dumps(payload, separators=(',', ':'),
                                   sort_keys=True)
        self.evidence_pub.publish(evidence)
        self.diagnostics.publish(payload, level, str(payload['reason']))

    def _evidence_state(self) -> str:
        return {'HEALTHY': 'CONSISTENT', 'DEGRADED': 'DEGRADED',
                'UNTRUSTED': 'INCONSISTENT'}.get(
                    self.sensor_trust.current_state, 'UNAVAILABLE')

    def _publish_states(self) -> None:
        legacy = String()
        legacy.data = self.trust.current_state
        self.legacy_trust_pub.publish(legacy)
        sensor = String()
        sensor.data = self.sensor_trust.current_state
        self.sensor_trust_pub.publish(sensor)

    def _handle_legacy_transition(self, evaluation: TrustEvaluation) -> None:
        if evaluation.transition:
            self.get_logger().warning(
                f'LEGACY TRUST: {evaluation.transition[0]} -> '
                f'{evaluation.transition[1]}')

    def _handle_sensor_transition(self,
                                  evaluation: SensorTrustEvaluation) -> None:
        if evaluation.transition:
            self.get_logger().info(
                f'GNSS TRUST: {evaluation.transition[0]} -> '
                f'{evaluation.transition[1]}')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ConsistencyMonitor()
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
