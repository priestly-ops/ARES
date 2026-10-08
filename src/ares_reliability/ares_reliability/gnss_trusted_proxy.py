#!/usr/bin/env python3
"""Apply recovery commands to operational GNSS on a separate output topic."""

import json
import math
from collections import deque
from typing import Deque, Optional, Tuple

from ares_reliability.recovery_manager import RecoveryDecision
from ares_reliability.sensor_consistency import (
    EARTH_RADIUS_M,
    rotate_enu,
    vector_displacement_residual,
)
from ares_reliability.time_sync import (
    CacheTimestampAligner,
    interpolate_pair,
)
import message_filters
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


def validate_reentry_timing(
    message_stamp_sec: float,
    current_time_sec: float,
    gate_release_sec: Optional[float],
    max_message_age_sec: float,
    future_tolerance_sec: float,
) -> tuple[bool, float, str, bool]:
    """Reject stale, future-dated, or pre-release fixes deterministically."""
    age_sec = current_time_sec - message_stamp_sec
    generated_before_release = (
        gate_release_sec is not None and
        message_stamp_sec < gate_release_sec)
    if not all(math.isfinite(value) for value in (
            message_stamp_sec, current_time_sec, max_message_age_sec,
            future_tolerance_sec)):
        return False, age_sec, 'NONFINITE_TIMESTAMP', generated_before_release
    if message_stamp_sec <= 0.0:
        return False, age_sec, 'MISSING_TIMESTAMP', generated_before_release
    if generated_before_release:
        return False, age_sec, 'GENERATED_BEFORE_GATE_RELEASE', True
    if age_sec > max_message_age_sec:
        return False, age_sec, 'STALE_MESSAGE', generated_before_release
    if age_sec < -future_tolerance_sec:
        return False, age_sec, 'FUTURE_MESSAGE', generated_before_release
    return True, age_sec, 'ACCEPTED', generated_before_release


def inflated_fix(message: NavSatFix, factor: float,
                 unknown_variance_m2: float = 1.0) -> NavSatFix:
    """Copy a fix and inflate valid or explicitly provisioned covariance."""
    if factor < 1.0 or not math.isfinite(factor):
        raise ValueError('covariance factor must be finite and at least one')
    if unknown_variance_m2 <= 0.0 or not math.isfinite(unknown_variance_m2):
        raise ValueError('unknown variance must be finite and positive')
    output = NavSatFix()
    output.header = message.header
    output.status = message.status
    output.latitude = message.latitude
    output.longitude = message.longitude
    output.altitude = message.altitude
    covariance = list(message.position_covariance)
    input_known = (
        message.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
        and all(math.isfinite(covariance[index]) and covariance[index] > 0.0
                for index in (0, 4, 8)))
    for index in (0, 4, 8):
        base = covariance[index] if input_known else unknown_variance_m2
        covariance[index] = base * factor

    output.position_covariance_type = (
        message.position_covariance_type if input_known else
        NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN)
    output.position_covariance = covariance
    return output



def _odom_xy(message: Odometry) -> Tuple[float, float]:
    """Return planar operational-odometry position."""
    return (
        float(message.pose.pose.position.x),
        float(message.pose.pose.position.y),
    )


def prefusion_guard_accept(residual_m: float, threshold_m: float) -> bool:
    """Return whether one pre-fusion displacement residual is admissible."""
    if not math.isfinite(residual_m):
        return False
    if not math.isfinite(threshold_m) or threshold_m <= 0.0:
        raise ValueError('pre-fusion threshold must be finite and positive')
    return residual_m < threshold_m


class GnssTrustedProxy(Node):
    """Forward, deweight, gate, and probation GNSS without touching raw truth."""

    def __init__(self) -> None:
        super().__init__('gnss_trusted_proxy')
        self.input_topic = str(self.declare_parameter(
            'input_topic', '/ares/gps').value)
        self.output_topic = str(self.declare_parameter(
            'output_topic', '/ares/gps_trusted').value)
        self.unknown_variance = float(self.declare_parameter(
            'unknown_variance_m2', 1.0).value)
        initial_factor = float(self.declare_parameter(
            'initial_covariance_factor', 1.0).value)
        initial_forward = bool(self.declare_parameter(
            'initial_forward', True).value)
        initial_state = str(self.declare_parameter(
            'initial_state', 'NORMAL').value)
        self.command_topic = str(self.declare_parameter(
            'command_topic', '/ares/recovery/gnss_policy').value)
        self.status_topic = str(self.declare_parameter(
            'status_topic', '/ares/gnss_gating_state').value)
        self.max_message_age = float(self.declare_parameter(
            'max_message_age_sec', 0.5).value)
        self.future_tolerance = float(self.declare_parameter(
            'future_tolerance_sec', 0.05).value)

        # Optional fast pre-fusion containment. Disabled by default so the
        # frozen Week 3/4/5 behavior is unchanged. Week 6 protected mode may
        # opt in explicitly.
        self.prefusion_guard_enabled = bool(self.declare_parameter(
            'prefusion_guard_enabled', False).value)
        self.prefusion_odometry_topic = str(self.declare_parameter(
            'prefusion_odometry_topic', '/ares/odom_operational').value)
        self.prefusion_threshold_m = float(self.declare_parameter(
            'prefusion_threshold_m', 3.0).value)
        self.prefusion_window_sec = float(self.declare_parameter(
            'prefusion_window_sec', 2.0).value)
        self.prefusion_max_sync_error_sec = float(self.declare_parameter(
            'prefusion_max_sync_error_sec', 0.15).value)
        self.prefusion_history_duration_sec = float(self.declare_parameter(
            'prefusion_history_duration_sec', 5.0).value)
        self.prefusion_expected_odometry_rate_hz = float(
            self.declare_parameter(
                'prefusion_expected_odometry_rate_hz', 50.0).value)
        self.prefusion_gnss_to_odom_yaw_deg = float(
            self.declare_parameter(
                'prefusion_gnss_to_odom_yaw_deg', -90.0).value)

        if self.max_message_age <= 0.0:
            raise ValueError('max_message_age_sec must be positive')
        if self.future_tolerance < 0.0:
            raise ValueError('future_tolerance_sec must be non-negative')
        if self.prefusion_threshold_m <= 0.0:
            raise ValueError('prefusion_threshold_m must be positive')
        if self.prefusion_window_sec <= 0.0:
            raise ValueError('prefusion_window_sec must be positive')
        if self.prefusion_max_sync_error_sec < 0.0:
            raise ValueError(
                'prefusion_max_sync_error_sec must be non-negative')
        if self.prefusion_history_duration_sec <= 0.0:
            raise ValueError(
                'prefusion_history_duration_sec must be positive')
        if self.prefusion_expected_odometry_rate_hz <= 0.0:
            raise ValueError(
                'prefusion_expected_odometry_rate_hz must be positive')
        self.decision = RecoveryDecision(
            initial_forward, initial_factor, initial_state,
            'configured startup policy')
        self.gate_release_sec: Optional[float] = None
        self.reentry_pending = not initial_forward
        self.accepted_measurement_count = 0
        self.rejected_measurement_count = 0

        self.prefusion_gps_ref: Optional[Tuple[float, float]] = None
        self.prefusion_odom_ref: Optional[Tuple[float, float]] = None
        self.prefusion_samples: Deque[
            Tuple[float, Tuple[float, float], Tuple[float, float]]
        ] = deque()
        self.prefusion_rejected_count = 0
        self.prefusion_sync_unavailable_count = 0
        self.prefusion_last_residual_m: Optional[float] = None
        self.prefusion_last_reason = 'DISABLED'

        self.prefusion_odom_sub = None
        self.prefusion_odom_cache = None
        self.prefusion_aligner = None
        if self.prefusion_guard_enabled:
            cache_size = max(
                5,
                math.ceil(
                    self.prefusion_history_duration_sec *
                    self.prefusion_expected_odometry_rate_hz
                ) + 2,
            )
            self.prefusion_odom_sub = message_filters.Subscriber(
                self, Odometry, self.prefusion_odometry_topic, 20)
            self.prefusion_odom_cache = message_filters.Cache(
                self.prefusion_odom_sub,
                cache_size=cache_size,
                allow_headerless=False,
            )
            self.prefusion_aligner = CacheTimestampAligner(
                self.prefusion_odom_cache,
                _odom_xy,
                self.prefusion_max_sync_error_sec,
                self.prefusion_history_duration_sec,
                interpolate_pair,
                clock_mismatch_tolerance_sec=max(
                    5.0, self.prefusion_history_duration_sec * 2.0),
            )

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, self.command_topic,
                                 self.command_callback, qos)
        self.create_subscription(NavSatFix, self.input_topic,
                                 self.gnss_callback, 10)
        self.output_pub = self.create_publisher(
            NavSatFix, self.output_topic, 10)
        self.status_pub = self.create_publisher(
            String, self.status_topic, 10)

    def _reset_prefusion_guard(self) -> None:
        """Discard the pre-fusion displacement reference/history."""
        self.prefusion_gps_ref = None
        self.prefusion_odom_ref = None
        self.prefusion_samples.clear()
        self.prefusion_last_residual_m = None
        self.prefusion_last_reason = 'RESET'

    def _prefusion_local_xy(
        self,
        latitude: float,
        longitude: float,
    ) -> Tuple[float, float]:
        if self.prefusion_gps_ref is None:
            raise RuntimeError('pre-fusion GNSS reference is not initialized')
        lat0, lon0 = self.prefusion_gps_ref
        north = EARTH_RADIUS_M * math.radians(latitude - lat0)
        east = (
            EARTH_RADIUS_M *
            math.cos(math.radians(lat0)) *
            math.radians(longitude - lon0)
        )
        return rotate_enu(
            east,
            north,
            self.prefusion_gnss_to_odom_yaw_deg,
        )

    def _prefusion_check(
        self,
        message: NavSatFix,
        source_stamp: float,
        current_time_sec: float,
    ) -> tuple[bool, str, Optional[float], Optional[float]]:
        """Reject a clearly impossible GNSS displacement before fusion."""
        if not self.prefusion_guard_enabled:
            return True, 'DISABLED', None, None

        if not all(math.isfinite(value) for value in (
                message.latitude, message.longitude)):
            self.prefusion_rejected_count += 1
            self.prefusion_last_reason = 'PREFUSION_NONFINITE_GNSS'
            self.prefusion_last_residual_m = None
            return False, self.prefusion_last_reason, None, None

        assert self.prefusion_aligner is not None
        reset_before = self.prefusion_aligner.reset_count
        sync = self.prefusion_aligner.lookup(
            source_stamp,
            clock_sec=current_time_sec,
            interpolate=False,
        )
        if self.prefusion_aligner.reset_count != reset_before:
            self._reset_prefusion_guard()

        if not sync.accepted:
            # The fast guard is supplementary. Lack of a synchronized wheel
            # sample must not create a new source of healthy false gating.
            self.prefusion_sync_unavailable_count += 1
            self.prefusion_last_reason = (
                f'PREFUSION_SYNC_{sync.reason.value}')
            self.prefusion_last_residual_m = None
            return (
                True,
                self.prefusion_last_reason,
                None,
                sync.absolute_sync_error_sec,
            )

        aligned_odom = sync.value
        assert aligned_odom is not None

        if self.prefusion_gps_ref is None:
            self.prefusion_gps_ref = (
                float(message.latitude),
                float(message.longitude),
            )
            self.prefusion_odom_ref = aligned_odom
            self.prefusion_samples.clear()
            self.prefusion_samples.append((
                source_stamp,
                (0.0, 0.0),
                (0.0, 0.0),
            ))
            self.prefusion_last_residual_m = 0.0
            self.prefusion_last_reason = 'PREFUSION_INITIALIZED'
            return (
                True,
                self.prefusion_last_reason,
                0.0,
                sync.absolute_sync_error_sec,
            )

        gps_xy = self._prefusion_local_xy(
            float(message.latitude),
            float(message.longitude),
        )
        assert self.prefusion_odom_ref is not None
        odom_xy = (
            aligned_odom[0] - self.prefusion_odom_ref[0],
            aligned_odom[1] - self.prefusion_odom_ref[1],
        )

        cutoff = source_stamp - self.prefusion_window_sec
        while (
            len(self.prefusion_samples) > 1 and
            self.prefusion_samples[1][0] <= cutoff
        ):
            self.prefusion_samples.popleft()

        oldest = self.prefusion_samples[0]
        if source_stamp - oldest[0] < self.prefusion_window_sec:
            self.prefusion_samples.append(
                (source_stamp, gps_xy, odom_xy))
            self.prefusion_last_residual_m = 0.0
            self.prefusion_last_reason = 'PREFUSION_WARMUP'
            return (
                True,
                self.prefusion_last_reason,
                0.0,
                sync.absolute_sync_error_sec,
            )

        residual = vector_displacement_residual(
            oldest[1],
            gps_xy,
            oldest[2],
            odom_xy,
        )
        self.prefusion_last_residual_m = residual

        if not prefusion_guard_accept(
                residual, self.prefusion_threshold_m):
            # Deliberately do not advance accepted history with a rejected
            # measurement. Persistent step corruption therefore remains
            # inconsistent with the last trusted trajectory until it clears
            # or the normal recovery manager gates the stream.
            self.prefusion_rejected_count += 1
            self.prefusion_last_reason = 'PREFUSION_RESIDUAL_REJECTED'
            self.get_logger().warning(
                'GNSS PRE-FUSION REJECT: '
                f'residual={residual:.3f}m '
                f'threshold={self.prefusion_threshold_m:.3f}m')
            return (
                False,
                self.prefusion_last_reason,
                residual,
                sync.absolute_sync_error_sec,
            )

        self.prefusion_samples.append((source_stamp, gps_xy, odom_xy))
        self.prefusion_last_reason = 'PREFUSION_ACCEPTED'
        return (
            True,
            self.prefusion_last_reason,
            residual,
            sync.absolute_sync_error_sec,
        )

    def command_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            decision = RecoveryDecision(
                bool(payload['forward']), float(payload['covariance_factor']),
                str(payload['state']), str(payload['reason']))
            if not self.decision.forward and decision.forward:
                self.gate_release_sec = (
                    self.get_clock().now().nanoseconds / 1.0e9)
                self.reentry_pending = True
                self._reset_prefusion_guard()
            elif self.decision.forward and not decision.forward:
                self.gate_release_sec = None
                self.reentry_pending = True
            self.decision = decision
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f'Ignoring invalid recovery command: {error}')

    def gnss_callback(self, message: NavSatFix) -> None:
        current_time_sec = self.get_clock().now().nanoseconds / 1.0e9
        message_stamp_sec = (
            float(message.header.stamp.sec) +
            float(message.header.stamp.nanosec) / 1.0e9)
        accepted, age_sec, timing_reason, before_release = (
            validate_reentry_timing(
                message_stamp_sec, current_time_sec, self.gate_release_sec,
                self.max_message_age, self.future_tolerance))
        forwarded = False
        first_reentry = False
        prefusion_accepted = True
        prefusion_reason = 'NOT_EVALUATED'
        prefusion_residual_m = None
        prefusion_sync_error_sec = None

        if self.decision.forward and accepted:
            (
                prefusion_accepted,
                prefusion_reason,
                prefusion_residual_m,
                prefusion_sync_error_sec,
            ) = self._prefusion_check(
                message,
                message_stamp_sec,
                current_time_sec,
            )

        if self.decision.forward and accepted and prefusion_accepted:
            output = inflated_fix(message, self.decision.covariance_factor,
                                  self.unknown_variance)
            self.output_pub.publish(output)
            forwarded = True
            self.accepted_measurement_count += 1
            if self.reentry_pending:
                first_reentry = True
                self.reentry_pending = False
        elif self.decision.forward:
            self.rejected_measurement_count += 1
            if accepted and not prefusion_accepted:
                timing_reason = prefusion_reason
        else:
            timing_reason = 'GATED'
        status = String()
        status.data = json.dumps({
            'state': self.decision.state,
            'gated': not self.decision.forward,
            'forwarded': forwarded,
            'first_reentry': first_reentry,
            'covariance_factor': self.decision.covariance_factor,
            'reason': self.decision.reason,
            'timing_reason': timing_reason,
            'message_stamp_sec': message_stamp_sec,
            'current_time_sec': current_time_sec,
            'message_age_sec': age_sec,
            'gate_release_sec': self.gate_release_sec,
            'generated_before_gate_release': before_release,
            'accepted_measurement_count': self.accepted_measurement_count,
            'rejected_measurement_count': self.rejected_measurement_count,
            'prefusion_guard_enabled': self.prefusion_guard_enabled,
            'prefusion_threshold_m': self.prefusion_threshold_m,
            'prefusion_window_sec': self.prefusion_window_sec,
            'prefusion_accepted': prefusion_accepted,
            'prefusion_reason': prefusion_reason,
            'prefusion_residual_m': prefusion_residual_m,
            'prefusion_sync_error_sec': prefusion_sync_error_sec,
            'prefusion_rejected_count': self.prefusion_rejected_count,
            'prefusion_sync_unavailable_count':
                self.prefusion_sync_unavailable_count,
            'prefusion_odometry_topic': self.prefusion_odometry_topic,
            'input_topic': self.input_topic,
            'output_topic': self.output_topic,
            'command_topic': self.command_topic,
            'production_ekf_connected': False,
        }, separators=(',', ':'), sort_keys=True)
        self.status_pub.publish(status)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GnssTrustedProxy()
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
