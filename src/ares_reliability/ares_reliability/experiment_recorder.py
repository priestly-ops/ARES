#!/usr/bin/env python3
"""Record compatible Week 2 and extended Week 3 reliability CSV files."""

import csv
import json
from pathlib import Path
from typing import Any, Optional, TextIO

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


CSV_FIELDS = (
    'timestamp_sec',
    'event',
    'profile',
    'motion_regime',
    'seed',
    'sensor_fault',
    'fault_mode',
    'fault_enabled',
    'fault_message_dropped',
    'imu_fault_mode',
    'wheel_fault_mode',
    'diagnostic_reason',
    'effective_east_offset_m',
    'effective_north_offset_m',
    'gnss_local_east_m',
    'gnss_local_north_m',
    'odometry_x_m',
    'odometry_y_m',
    'robot_speed_mps',
    'robot_yaw_rate_radps',
    'robot_acceleration_mps2',
    'localization_covariance_x_m2',
    'localization_covariance_y_m2',
    'localization_covariance_yaw_rad2',
    'gnss_message_freshness_sec',
    'imu_message_freshness_sec',
    'localization_message_freshness_sec',
    'instantaneous_residual_m',
    'rolling_mean_residual_m',
    'trust_state',
    'gnss_trust_state',
    'gnss_sync_error_sec',
    'gnss_sync_reason',
    'gnss_stale',
    'imu_instantaneous_residual_radps',
    'imu_rolling_mean_residual_radps',
    'imu_trust_state',
    'imu_sync_error_sec',
    'imu_sync_reason',
    'imu_stale',
    'localization_instantaneous_residual_m',
    'localization_rolling_mean_residual_m',
    'localization_trust_state',
    'localization_sync_error_sec',
    'localization_sync_reason',
    'localization_stale',
    'odometry_trust_state',
    'system_consistency',
    'system_attribution',
    'attribution_confidence',
    'attribution_reason',
    'supporting_evidence',
    'unavailable_evidence',
    'recovery_state',
    'gating_state',
    'gnss_gated',
    'covariance_scale',
    'gating_reason',
    'gnss_forwarded',
    'gnss_first_reentry',
    'gnss_timing_reason',
    'gnss_message_stamp_sec',
    'gnss_proxy_time_sec',
    'gnss_proxy_message_age_sec',
    'gnss_gate_release_sec',
    'gnss_generated_before_gate_release',
    'gnss_proxy_accepted_count',
    'gnss_proxy_rejected_count',
    'fusion_mode',
    'fused_x_m',
    'fused_y_m',
    'fused_reference_error_m',
    'fused_covariance_trace',
    'fused_covariance_max',
    'estimator_status',
    'estimator_finite',
    'estimator_stale',
    'estimator_publication_rate_hz',
    'estimator_freshness_sec',
    'estimator_pose_jump_m',
    'estimator_timestamp_regression',
    'estimator_restart_count',
    'estimator_uptime_sec',
    'gnss_measurement_x_m',
    'gnss_measurement_y_m',
    'gnss_measurement_covariance_x_m2',
    'gnss_measurement_covariance_y_m2',
    'gnss_innovation_m',
    'gnss_measurement_count',
    'gnss_freeze_state',
    'gnss_freeze_odometry_displacement_m',
    'gnss_freeze_position_displacement_m',
    'gnss_freeze_detected_at_sec',
)


class ExperimentRecorder(Node):
    """Combine monitor diagnostics, fault status, and trust state into CSV."""

    def __init__(self) -> None:
        super().__init__('week2_experiment_recorder')
        output_directory = str(
            self.declare_parameter('output_directory', 'results/week2').value
        )
        result_name = str(
            self.declare_parameter('result_name', 'week2_experiment').value
        )
        self.profile = str(self.declare_parameter('profile', 'healthy').value)
        self.fusion_mode = str(self.declare_parameter(
            'fusion_mode', 'not_applicable').value)
        configured_seed = int(self.declare_parameter(
            'experiment_seed', -1).value)
        self.motion_regime = 'UNKNOWN'
        self.flush_every_row = bool(
            self.declare_parameter('flush_every_row', True).value
        )

        safe_name = Path(result_name).name
        if not safe_name.endswith('.csv'):
            safe_name += '.csv'
        output_path = Path(output_directory).expanduser().resolve()
        output_path.mkdir(parents=True, exist_ok=True)
        self.csv_path = output_path / safe_name
        self.csv_file: TextIO = self.csv_path.open(
            'w',
            encoding='utf-8',
            newline='',
        )
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()

        self.fault_mode = 'unknown'
        self.seed: Optional[int] = (
            configured_seed if configured_seed >= 0 else None)
        self.sensor_fault = 'none'
        self.imu_fault_mode = 'none'
        self.wheel_fault_mode = 'none'
        self.diagnostic_reason = 'UNKNOWN'
        self.gnss_freeze_state = 'UNKNOWN'
        self.gnss_freeze_odometry_displacement_m: Optional[float] = None
        self.gnss_freeze_position_displacement_m: Optional[float] = None
        self.gnss_freeze_detected_at_sec: Optional[float] = None
        self.fault_enabled = False
        self.fault_message_dropped = False
        self.effective_east_offset_m: Optional[float] = None
        self.effective_north_offset_m: Optional[float] = None
        self.gnss_local_east_m: Optional[float] = None
        self.gnss_local_north_m: Optional[float] = None
        self.odometry_x_m: Optional[float] = None
        self.odometry_y_m: Optional[float] = None
        self.robot_speed_mps: Optional[float] = None
        self.robot_yaw_rate_radps: Optional[float] = None
        self.robot_acceleration_mps2: Optional[float] = None
        self.localization_covariance_x_m2: Optional[float] = None
        self.localization_covariance_y_m2: Optional[float] = None
        self.localization_covariance_yaw_rad2: Optional[float] = None
        self.last_motion_timestamp_sec: Optional[float] = None
        self.last_motion_speed_mps: Optional[float] = None
        self.last_gnss_diagnostic_sec: Optional[float] = None
        self.last_imu_diagnostic_sec: Optional[float] = None
        self.last_localization_diagnostic_sec: Optional[float] = None
        self.instantaneous_residual_m: Optional[float] = None
        self.rolling_mean_residual_m: Optional[float] = None
        self.trust_state = 'UNKNOWN'
        self.gnss_trust_state = 'UNKNOWN'
        self.gnss_sync_error_sec: Optional[float] = None
        self.gnss_sync_reason = 'UNKNOWN'
        self.gnss_stale: Optional[bool] = None
        self.imu_instantaneous_residual_radps: Optional[float] = None
        self.imu_rolling_mean_residual_radps: Optional[float] = None
        self.imu_trust_state = 'UNKNOWN'
        self.imu_sync_error_sec: Optional[float] = None
        self.imu_sync_reason = 'UNKNOWN'
        self.imu_stale: Optional[bool] = None
        self.localization_instantaneous_residual_m: Optional[float] = None
        self.localization_rolling_mean_residual_m: Optional[float] = None
        self.localization_trust_state = 'UNKNOWN'
        self.localization_sync_error_sec: Optional[float] = None
        self.localization_sync_reason = 'UNKNOWN'
        self.localization_stale: Optional[bool] = None
        self.odometry_trust_state = 'UNAVAILABLE'
        self.system_consistency = 'UNKNOWN'
        self.system_attribution = 'UNKNOWN'
        self.attribution_confidence = 'UNKNOWN'
        self.attribution_reason = 'UNKNOWN'
        self.supporting_evidence = '[]'
        self.unavailable_evidence = '[]'
        self.recovery_state = 'UNKNOWN'
        self.gating_state = 'UNKNOWN'
        self.gnss_gated: Optional[bool] = None
        self.covariance_scale: Optional[float] = None
        self.gating_reason = 'UNKNOWN'
        self.gnss_forwarded: Optional[bool] = None
        self.gnss_first_reentry: Optional[bool] = None
        self.gnss_timing_reason = 'UNKNOWN'
        self.gnss_message_stamp_sec: Optional[float] = None
        self.gnss_proxy_time_sec: Optional[float] = None
        self.gnss_proxy_message_age_sec: Optional[float] = None
        self.gnss_gate_release_sec: Optional[float] = None
        self.gnss_generated_before_gate_release: Optional[bool] = None
        self.gnss_proxy_accepted_count = 0
        self.gnss_proxy_rejected_count = 0
        self.fused_x_m: Optional[float] = None
        self.fused_y_m: Optional[float] = None
        self.fused_reference_error_m: Optional[float] = None
        self.fused_covariance_trace: Optional[float] = None
        self.fused_covariance_max: Optional[float] = None
        self.estimator_status = 'UNAVAILABLE'
        self.estimator_finite: Optional[bool] = None
        self.estimator_stale: Optional[bool] = None
        self.estimator_publication_rate_hz: Optional[float] = None
        self.estimator_freshness_sec: Optional[float] = None
        self.estimator_pose_jump_m: Optional[float] = None
        self.estimator_timestamp_regression: Optional[bool] = None
        self.estimator_restart_count = 0
        self.estimator_uptime_sec: Optional[float] = None
        self.gnss_measurement_x_m: Optional[float] = None
        self.gnss_measurement_y_m: Optional[float] = None
        self.gnss_measurement_covariance_x_m2: Optional[float] = None
        self.gnss_measurement_covariance_y_m2: Optional[float] = None
        self.gnss_innovation_m: Optional[float] = None
        self.gnss_measurement_count = 0
        self.last_system_signature: Optional[tuple[Any, ...]] = None
        self.last_gating_signature: Optional[tuple[Any, ...]] = None

        self.diagnostics_sub = self.create_subscription(
            Float64MultiArray,
            '/ares/gnss_odom_diagnostics',
            self.diagnostics_callback,
            10,
        )
        self.fault_sub = self.create_subscription(
            String,
            '/ares/gnss_fault_status',
            self.fault_status_callback,
            10,
        )
        self.trust_sub = self.create_subscription(
            String,
            '/ares/trust_state',
            self.trust_callback,
            10,
        )
        self.create_subscription(
            String,
            '/ares/diagnostics/gnss',
            self.gnss_diagnostics_callback,
            10,
        )
        self.create_subscription(
            String, '/ares/imu_fault_status', self.imu_fault_callback, 20)
        self.create_subscription(
            String, '/ares/wheel_fault_status', self.wheel_fault_callback, 20)
        self.create_subscription(
            Odometry, '/ares/odom_operational', self.motion_callback, 50)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.localization_pose_callback, 10)
        self.create_subscription(
            String, '/ares/experiment/motion_regime',
            self.motion_regime_callback, 10)
        self.create_subscription(
            String,
            '/ares/diagnostics/imu',
            self.imu_diagnostics_callback,
            20,
        )
        self.create_subscription(
            String,
            '/ares/diagnostics/localization',
            self.localization_diagnostics_callback,
            10,
        )
        self.create_subscription(
            String, '/ares/trust/gnss', self.gnss_trust_callback, 10
        )
        self.create_subscription(
            String, '/ares/trust/imu', self.imu_trust_callback, 20
        )
        self.create_subscription(
            String,
            '/ares/trust/localization',
            self.localization_trust_callback,
            10,
        )
        self.create_subscription(
            String, '/ares/trust/odometry', self.odometry_trust_callback, 10
        )
        self.create_subscription(
            String,
            '/ares/system_trust_state',
            self.system_trust_callback,
            10,
        )
        self.create_subscription(
            String,
            '/ares/recovery_state',
            self.recovery_callback,
            10,
        )
        self.create_subscription(
            String,
            '/ares/gnss_gating_state',
            self.gating_callback,
            10,
        )
        self.create_subscription(
            String,
            '/ares/estimator_health',
            self.estimator_health_callback,
            20,
        )
        self.get_logger().info(f'Recording reliability data to {self.csv_path}')

    @staticmethod
    def _csv_value(value: Any) -> Any:
        return '' if value is None else value

    def _write_row(self, event: str) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1.0e9

        def freshness(last: Optional[float]) -> Any:
            return '' if last is None else max(0.0, now_sec - last)

        row = {
            'timestamp_sec': now_sec,
            'event': event,
            'profile': self.profile,
            'motion_regime': self.motion_regime,
            'seed': self._csv_value(self.seed),
            'sensor_fault': self.sensor_fault,
            'fault_mode': self.fault_mode,
            'fault_enabled': str(self.fault_enabled).lower(),
            'fault_message_dropped': str(
                self.fault_message_dropped
            ).lower(),
            'imu_fault_mode': self.imu_fault_mode,
            'wheel_fault_mode': self.wheel_fault_mode,
            'diagnostic_reason': self.diagnostic_reason,
            'effective_east_offset_m': self._csv_value(
                self.effective_east_offset_m
            ),
            'effective_north_offset_m': self._csv_value(
                self.effective_north_offset_m
            ),
            'gnss_local_east_m': self._csv_value(self.gnss_local_east_m),
            'gnss_local_north_m': self._csv_value(self.gnss_local_north_m),
            'odometry_x_m': self._csv_value(self.odometry_x_m),
            'odometry_y_m': self._csv_value(self.odometry_y_m),
            'robot_speed_mps': self._csv_value(self.robot_speed_mps),
            'robot_yaw_rate_radps': self._csv_value(
                self.robot_yaw_rate_radps),
            'robot_acceleration_mps2': self._csv_value(
                self.robot_acceleration_mps2),
            'localization_covariance_x_m2': self._csv_value(
                self.localization_covariance_x_m2),
            'localization_covariance_y_m2': self._csv_value(
                self.localization_covariance_y_m2),
            'localization_covariance_yaw_rad2': self._csv_value(
                self.localization_covariance_yaw_rad2),
            'gnss_message_freshness_sec': freshness(
                self.last_gnss_diagnostic_sec),
            'imu_message_freshness_sec': freshness(
                self.last_imu_diagnostic_sec),
            'localization_message_freshness_sec': freshness(
                self.last_localization_diagnostic_sec),
            'instantaneous_residual_m': self._csv_value(
                self.instantaneous_residual_m
            ),
            'rolling_mean_residual_m': self._csv_value(
                self.rolling_mean_residual_m
            ),
            'trust_state': self.trust_state,
            'gnss_trust_state': self.gnss_trust_state,
            'gnss_sync_error_sec': self._csv_value(
                self.gnss_sync_error_sec
            ),
            'gnss_sync_reason': self.gnss_sync_reason,
            'gnss_stale': self._csv_value(self.gnss_stale),
            'imu_instantaneous_residual_radps': self._csv_value(
                self.imu_instantaneous_residual_radps
            ),
            'imu_rolling_mean_residual_radps': self._csv_value(
                self.imu_rolling_mean_residual_radps
            ),
            'imu_trust_state': self.imu_trust_state,
            'imu_sync_error_sec': self._csv_value(self.imu_sync_error_sec),
            'imu_sync_reason': self.imu_sync_reason,
            'imu_stale': self._csv_value(self.imu_stale),
            'localization_instantaneous_residual_m': self._csv_value(
                self.localization_instantaneous_residual_m
            ),
            'localization_rolling_mean_residual_m': self._csv_value(
                self.localization_rolling_mean_residual_m
            ),
            'localization_trust_state': self.localization_trust_state,
            'localization_sync_error_sec': self._csv_value(
                self.localization_sync_error_sec
            ),
            'localization_sync_reason': self.localization_sync_reason,
            'localization_stale': self._csv_value(self.localization_stale),
            'odometry_trust_state': self.odometry_trust_state,
            'system_consistency': self.system_consistency,
            'system_attribution': self.system_attribution,
            'attribution_confidence': self.attribution_confidence,
            'attribution_reason': self.attribution_reason,
            'supporting_evidence': self.supporting_evidence,
            'unavailable_evidence': self.unavailable_evidence,
            'recovery_state': self.recovery_state,
            'gating_state': self.gating_state,
            'gnss_gated': self._csv_value(self.gnss_gated),
            'covariance_scale': self._csv_value(self.covariance_scale),
            'gating_reason': self.gating_reason,
            'gnss_forwarded': self._csv_value(self.gnss_forwarded),
            'gnss_first_reentry': self._csv_value(self.gnss_first_reentry),
            'gnss_timing_reason': self.gnss_timing_reason,
            'gnss_message_stamp_sec': self._csv_value(
                self.gnss_message_stamp_sec),
            'gnss_proxy_time_sec': self._csv_value(
                self.gnss_proxy_time_sec),
            'gnss_proxy_message_age_sec': self._csv_value(
                self.gnss_proxy_message_age_sec),
            'gnss_gate_release_sec': self._csv_value(
                self.gnss_gate_release_sec),
            'gnss_generated_before_gate_release': self._csv_value(
                self.gnss_generated_before_gate_release),
            'gnss_proxy_accepted_count': self.gnss_proxy_accepted_count,
            'gnss_proxy_rejected_count': self.gnss_proxy_rejected_count,
            'fusion_mode': self.fusion_mode,
            'fused_x_m': self._csv_value(self.fused_x_m),
            'fused_y_m': self._csv_value(self.fused_y_m),
            'fused_reference_error_m': self._csv_value(
                self.fused_reference_error_m),
            'fused_covariance_trace': self._csv_value(
                self.fused_covariance_trace),
            'fused_covariance_max': self._csv_value(
                self.fused_covariance_max),
            'estimator_status': self.estimator_status,
            'estimator_finite': self._csv_value(self.estimator_finite),
            'estimator_stale': self._csv_value(self.estimator_stale),
            'estimator_publication_rate_hz': self._csv_value(
                self.estimator_publication_rate_hz),
            'estimator_freshness_sec': self._csv_value(
                self.estimator_freshness_sec),
            'estimator_pose_jump_m': self._csv_value(
                self.estimator_pose_jump_m),
            'estimator_timestamp_regression': self._csv_value(
                self.estimator_timestamp_regression),
            'estimator_restart_count': self.estimator_restart_count,
            'estimator_uptime_sec': self._csv_value(
                self.estimator_uptime_sec),
            'gnss_measurement_x_m': self._csv_value(
                self.gnss_measurement_x_m),
            'gnss_measurement_y_m': self._csv_value(
                self.gnss_measurement_y_m),
            'gnss_measurement_covariance_x_m2': self._csv_value(
                self.gnss_measurement_covariance_x_m2),
            'gnss_measurement_covariance_y_m2': self._csv_value(
                self.gnss_measurement_covariance_y_m2),
            'gnss_innovation_m': self._csv_value(self.gnss_innovation_m),
            'gnss_measurement_count': self.gnss_measurement_count,
            'gnss_freeze_state': self.gnss_freeze_state,
            'gnss_freeze_odometry_displacement_m': self._csv_value(
                self.gnss_freeze_odometry_displacement_m),
            'gnss_freeze_position_displacement_m': self._csv_value(
                self.gnss_freeze_position_displacement_m),
            'gnss_freeze_detected_at_sec': self._csv_value(
                self.gnss_freeze_detected_at_sec),
        }
        self.writer.writerow(row)
        if self.flush_every_row:
            self.csv_file.flush()

    def diagnostics_callback(self, msg: Float64MultiArray) -> None:
        """Record an exact monitor residual/rolling-mean evaluation."""
        if len(msg.data) < 6:
            self.get_logger().warning('Ignoring incomplete diagnostics message')
            return
        (
            self.gnss_local_east_m,
            self.gnss_local_north_m,
            self.odometry_x_m,
            self.odometry_y_m,
            self.instantaneous_residual_m,
            self.rolling_mean_residual_m,
        ) = msg.data[:6]
        self._write_row('diagnostic')

    def fault_status_callback(self, msg: String) -> None:
        """Record injector state, including messages deliberately dropped."""
        try:
            status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('Ignoring malformed GNSS fault status')
            return
        self.fault_mode = str(status.get('mode', 'unknown'))
        self.sensor_fault = 'gnss' if status.get('enabled', False) else 'none'
        if self.seed is None:
            self.seed = status.get('seed')
        self.diagnostic_reason = str(
            status.get('diagnostic_reason', 'UNKNOWN'))
        self.fault_enabled = bool(status.get('enabled', False))
        self.fault_message_dropped = bool(status.get('dropped', False))
        self.effective_east_offset_m = status.get('east_offset_m')
        self.effective_north_offset_m = status.get('north_offset_m')
        self._write_row('fault_status')

    def trust_callback(self, msg: String) -> None:
        """Record public trust-state publications and transitions."""
        changed = msg.data != self.trust_state
        self.trust_state = msg.data
        self._write_row('trust_transition' if changed else 'trust_state')

    @staticmethod
    def _json_payload(msg: String) -> Optional[dict[str, Any]]:
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return None
        return payload if isinstance(payload, dict) else None

    def gnss_diagnostics_callback(self, msg: String) -> None:
        """Record timestamp alignment and Week 3 GNSS trust diagnostics."""
        payload = self._json_payload(msg)
        if payload is None:
            return
        self.last_gnss_diagnostic_sec = (
            self.get_clock().now().nanoseconds / 1.0e9)
        self.diagnostic_reason = str(payload.get('reason', 'UNKNOWN'))
        self.gnss_sync_error_sec = payload.get('sync_error_sec')
        self.gnss_sync_reason = str(payload.get('sync_reason', 'UNKNOWN'))
        self.gnss_stale = payload.get('stale')
        self.gnss_trust_state = str(payload.get('trust_state', 'UNKNOWN'))
        self.gnss_freeze_state = str(
            payload.get('gnss_freeze_state', 'UNKNOWN'))
        self.gnss_freeze_odometry_displacement_m = payload.get(
            'gnss_freeze_odometry_displacement_m')
        self.gnss_freeze_position_displacement_m = payload.get(
            'gnss_freeze_position_displacement_m')
        self.gnss_freeze_detected_at_sec = payload.get(
            'gnss_freeze_detected_at_sec')
        self._write_row('gnss_diagnostic')

    def imu_diagnostics_callback(self, msg: String) -> None:
        """Record IMU/odometry residual diagnostics."""
        payload = self._json_payload(msg)
        if payload is None:
            return
        self.last_imu_diagnostic_sec = (
            self.get_clock().now().nanoseconds / 1.0e9)
        self.diagnostic_reason = str(payload.get('reason', 'UNKNOWN'))
        self.imu_instantaneous_residual_radps = payload.get('residual')
        self.imu_rolling_mean_residual_radps = payload.get('rolling_mean')
        self.imu_sync_error_sec = payload.get('sync_error_sec')
        self.imu_sync_reason = str(payload.get('sync_reason', 'UNKNOWN'))
        self.imu_stale = payload.get('stale')
        self.imu_trust_state = str(payload.get('trust_state', 'UNKNOWN'))
        self._write_row('imu_diagnostic')

    def localization_diagnostics_callback(self, msg: String) -> None:
        """Record localization/odometry relative-displacement diagnostics."""
        payload = self._json_payload(msg)
        if payload is None:
            return
        self.last_localization_diagnostic_sec = (
            self.get_clock().now().nanoseconds / 1.0e9)
        self.diagnostic_reason = str(payload.get('reason', 'UNKNOWN'))
        self.localization_instantaneous_residual_m = payload.get('residual')
        self.localization_rolling_mean_residual_m = payload.get(
            'rolling_mean'
        )
        self.localization_sync_error_sec = payload.get('sync_error_sec')
        self.localization_sync_reason = str(
            payload.get('sync_reason', 'UNKNOWN')
        )
        self.localization_stale = payload.get('stale')
        self.localization_trust_state = str(
            payload.get('trust_state', 'UNKNOWN')
        )
        self._write_row('localization_diagnostic')

    def gnss_trust_callback(self, msg: String) -> None:
        """Record per-GNSS trust transitions."""
        changed = msg.data != self.gnss_trust_state
        self.gnss_trust_state = msg.data
        if changed:
            self._write_row('gnss_trust_transition')

    def imu_trust_callback(self, msg: String) -> None:
        """Record per-IMU trust transitions."""
        changed = msg.data != self.imu_trust_state
        self.imu_trust_state = msg.data
        if changed:
            self._write_row('imu_trust_transition')

    def localization_trust_callback(self, msg: String) -> None:
        """Record per-localization trust transitions."""
        changed = msg.data != self.localization_trust_state
        self.localization_trust_state = msg.data
        if changed:
            self._write_row('localization_trust_transition')

    def system_trust_callback(self, msg: String) -> None:
        """Record system consistency and cautious attribution."""
        payload = self._json_payload(msg)
        if payload is None:
            return
        self.system_consistency = str(payload.get('consistency', 'UNKNOWN'))
        self.system_attribution = str(payload.get('attribution', 'UNKNOWN'))
        confidence = payload.get('confidence', 'UNKNOWN')
        self.attribution_confidence = str(confidence)
        self.attribution_reason = str(payload.get('reason', 'UNKNOWN'))
        self.supporting_evidence = json.dumps(
            payload.get('supporting_evidence', [])
        )
        self.unavailable_evidence = json.dumps(
            payload.get('unavailable_evidence', [])
        )
        signature = (
            self.system_consistency, self.system_attribution,
            self.attribution_confidence, self.supporting_evidence,
            self.unavailable_evidence)
        if signature != self.last_system_signature:
            self.last_system_signature = signature
            self._write_row('system_trust')

    def recovery_callback(self, msg: String) -> None:
        """Record staged recovery state."""
        if msg.data != self.recovery_state:
            self.recovery_state = msg.data
            self._write_row('recovery_state')

    def odometry_trust_callback(self, msg: String) -> None:
        """Record inferred wheel/motion trust separately."""
        if msg.data != self.odometry_trust_state:
            self.odometry_trust_state = msg.data
            self._write_row('odometry_trust_state')

    def gating_callback(self, msg: String) -> None:
        """Record forwarding and covariance/gating status."""
        payload = self._json_payload(msg)
        if payload is None:
            return
        self.gating_state = str(payload.get('state', 'UNKNOWN'))
        self.gnss_gated = payload.get('gated')
        self.covariance_scale = payload.get('covariance_factor')
        self.gating_reason = str(payload.get('reason', 'UNKNOWN'))
        self.gnss_forwarded = payload.get('forwarded')
        self.gnss_first_reentry = payload.get('first_reentry')
        self.gnss_timing_reason = str(
            payload.get('timing_reason', 'UNKNOWN'))
        self.gnss_message_stamp_sec = payload.get('message_stamp_sec')
        self.gnss_proxy_time_sec = payload.get('current_time_sec')
        self.gnss_proxy_message_age_sec = payload.get('message_age_sec')
        self.gnss_gate_release_sec = payload.get('gate_release_sec')
        self.gnss_generated_before_gate_release = payload.get(
            'generated_before_gate_release')
        self.gnss_proxy_accepted_count = int(payload.get(
            'accepted_measurement_count', self.gnss_proxy_accepted_count))
        self.gnss_proxy_rejected_count = int(payload.get(
            'rejected_measurement_count', self.gnss_proxy_rejected_count))
        signature = (
            self.gating_state, self.gnss_gated, self.covariance_scale,
            self.gating_reason)
        if self.gnss_first_reentry:
            self._write_row('gnss_reentry')
        elif (self.gnss_forwarded is False and not self.gnss_gated and
              self.gnss_timing_reason not in ('UNKNOWN', 'GATED')):
            self._write_row('gnss_rejected')
        if signature != self.last_gating_signature:
            self.last_gating_signature = signature
            self._write_row('gating_state')

    def estimator_health_callback(self, msg: String) -> None:
        """Record experimental EKF output and health separately from trust."""
        payload = self._json_payload(msg)
        if payload is None:
            return
        self.fused_x_m = payload.get('x_m')
        self.fused_y_m = payload.get('y_m')
        self.fused_reference_error_m = payload.get('reference_error_m')
        self.fused_covariance_trace = payload.get('covariance_trace')
        self.fused_covariance_max = payload.get('covariance_max')
        self.estimator_status = str(payload.get('status', 'UNKNOWN'))
        self.estimator_finite = payload.get('finite')
        self.estimator_stale = payload.get('stale')
        self.estimator_publication_rate_hz = payload.get(
            'publication_rate_hz')
        self.estimator_freshness_sec = payload.get('freshness_sec')
        self.estimator_pose_jump_m = payload.get('pose_jump_m')
        self.estimator_timestamp_regression = payload.get(
            'timestamp_regression')
        self.estimator_restart_count = int(payload.get('restart_count', 0))
        self.estimator_uptime_sec = payload.get('uptime_sec')
        self.gnss_measurement_x_m = payload.get('gnss_measurement_x_m')
        self.gnss_measurement_y_m = payload.get('gnss_measurement_y_m')
        self.gnss_measurement_covariance_x_m2 = payload.get(
            'gnss_covariance_x_m2')
        self.gnss_measurement_covariance_y_m2 = payload.get(
            'gnss_covariance_y_m2')
        self.gnss_innovation_m = payload.get('gnss_innovation_m')
        self.gnss_measurement_count = int(payload.get(
            'gnss_measurement_count', 0))
        self._write_row('estimator_health')

    def imu_fault_callback(self, msg: String) -> None:
        """Record the current IMU injection state."""
        payload = self._json_payload(msg)
        if payload is None:
            return
        previous = self.imu_fault_mode
        self.imu_fault_mode = str(payload.get('mode', 'unknown'))
        if payload.get('enabled', False):
            self.sensor_fault = 'imu' if self.sensor_fault == 'none' else 'multi'
        if self.seed is None:
            self.seed = payload.get('seed')
        if self.imu_fault_mode != previous or payload.get('dropped', False):
            self._write_row('imu_fault_status')

    def wheel_fault_callback(self, msg: String) -> None:
        """Record the message-level wheel injection state."""
        payload = self._json_payload(msg)
        if payload is None:
            return
        previous = self.wheel_fault_mode
        self.wheel_fault_mode = str(payload.get('mode', 'unknown'))
        if payload.get('enabled', False):
            self.sensor_fault = (
                'wheel' if self.sensor_fault == 'none' else 'multi')
        if self.wheel_fault_mode != previous or payload.get('dropped', False):
            self._write_row('wheel_fault_status')

    def motion_callback(self, msg: Odometry) -> None:
        """Track speed, yaw rate, and finite-difference acceleration."""
        now = self.get_clock().now().nanoseconds / 1.0e9
        speed = float(msg.twist.twist.linear.x)
        self.robot_yaw_rate_radps = float(msg.twist.twist.angular.z)
        if (self.last_motion_timestamp_sec is not None and
                self.last_motion_speed_mps is not None and
                now > self.last_motion_timestamp_sec):
            self.robot_acceleration_mps2 = (
                (speed - self.last_motion_speed_mps) /
                (now - self.last_motion_timestamp_sec))
        self.robot_speed_mps = speed
        self.last_motion_speed_mps = speed
        self.last_motion_timestamp_sec = now

    def localization_pose_callback(
        self,
        msg: PoseWithCovarianceStamped,
    ) -> None:
        """Track the AMCL covariance used in motion calibration."""
        covariance = msg.pose.covariance
        self.localization_covariance_x_m2 = float(covariance[0])
        self.localization_covariance_y_m2 = float(covariance[7])
        self.localization_covariance_yaw_rad2 = float(covariance[35])

    def motion_regime_callback(self, msg: String) -> None:
        """Record the runner-supplied healthy-motion regime label."""
        self.motion_regime = msg.data

    def destroy_node(self) -> None:
        """Flush and close the CSV before destroying the ROS node."""
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """Run the Week 2 experiment recorder node."""
    rclpy.init(args=args)
    node = ExperimentRecorder()
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
