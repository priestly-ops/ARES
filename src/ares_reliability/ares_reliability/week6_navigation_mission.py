#!/usr/bin/env python3
"""Run and record one deterministic ARES Week 6 waypoint mission."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from ament_index_python.packages import get_package_share_directory


from ares_reliability.navigation_metrics import (
    ClockRuntimeMetrics,
    classify_mission,
    controller_command_gaps,
    cross_track_errors,
    path_length,
    summarize_errors,
    waypoint_summary,
)


from diagnostic_msgs.msg import DiagnosticArray


from geometry_msgs.msg import TwistStamped


from lifecycle_msgs.srv import GetState


from nav2_msgs.action import NavigateToPose


from nav_msgs.msg import Odometry, Path as NavPath


from rcl_interfaces.msg import Parameter as ParameterMessage


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time


from rosgraph_msgs.msg import Clock


from std_msgs.msg import String


from tf2_msgs.msg import TFMessage


from tf2_ros import Buffer, TransformException, TransformListener


import yaml  # type: ignore[import-untyped]


def stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1.0e9


def quaternion_yaw(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z +
               quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y +
                     quaternion.z * quaternion.z),
    )


def angle_difference(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


def parameter_values(names: list[str], values: list[Any]) -> dict[str, Any]:
    """Associate GetParameters values with their requested parameter names."""
    if len(names) != len(values):
        raise ValueError('parameter response length does not match request')
    return {
        name: Parameter.from_parameter_msg(
            ParameterMessage(name=name, value=value)).value
        for name, value in zip(names, values, strict=True)
    }


class Week6Mission(Node):
    """Sequential goal client plus synchronized mission evidence recorder."""

    def __init__(self, config: dict[str, Any], mode: str,
                 scenario: str, seed: int) -> None:
        super().__init__('week6_navigation_mission')
        self.config = config
        self.mode = mode
        self.scenario = scenario
        self.seed = seed
        self.created_wall = time.monotonic()
        self.sim_time: Optional[float] = None
        self.clock_runtime = ClockRuntimeMetrics()
        self.clock_observations: list[tuple[float, float]] = []
        self.clock_boundary_status: dict[str, Any] = {}
        self.mission_start_sim: Optional[float] = None
        self.mission_end_sim: Optional[float] = None
        self.goal_active = False
        self.active_goal_index: Optional[int] = None
        self.active_window_start: Optional[float] = None
        self.active_windows: list[tuple[float, float]] = []
        self.waypoints: list[dict[str, Any]] = []
        self.latest_feedback: dict[str, Any] = {}
        self.latest_reference: Optional[tuple[float, float, float]] = None
        self.latest_wheel_odometry: Optional[tuple[float, float, float]] = None
        self.latest_localization: Optional[tuple[float, float, float]] = None
        self.latest_localization_stamp_sec: Optional[float] = None
        self.latest_localization_received_wall: Optional[float] = None
        self.odometry_pose_samples: dict[str, dict[str, Any]] = {}
        self.goal_checker_configuration: dict[str, Any] = {
            'query_status': 'not_attempted',
        }
        self.goal_diagnostic_context: Optional[dict[str, Any]] = None
        self.goal_completion_diagnostics: list[dict[str, Any]] = []
        self.readiness_localization_history: list[
            tuple[float, float, float, float]
        ] = []
        self.readiness_evidence: dict[str, Any] = {}
        self.actual_path: list[tuple[float, float]] = []
        self.wheel_odometry_path: list[tuple[float, float]] = []
        self.localization_path: list[tuple[float, float]] = []
        self.localization_samples: list[dict[str, Any]] = []
        self.commands: list[tuple[float, float, float]] = []
        self.current_plan: list[tuple[float, float]] = []
        self.initial_plans: dict[int, list[tuple[float, float]]] = {}
        self.replan_counts: dict[int, int] = {}
        self.cross_track_samples: list[dict[str, Any]] = []
        self.fault_active = False
        self.fault_started = False
        self.fault_cleared = False
        self.fault_events: list[dict[str, Any]] = []
        self.fault_status: dict[str, Any] = {}
        self.trust = {
            'gnss': 'UNAVAILABLE', 'imu': 'UNAVAILABLE',
            'odometry': 'UNAVAILABLE', 'system': 'UNAVAILABLE',
        }
        self.attribution = 'INSUFFICIENT_EVIDENCE'
        self.recovery_state = 'UNAVAILABLE'
        self.recovery_events: list[dict[str, Any]] = []
        self.gate_count = 0
        self.probation_count = 0
        self.estimator_restart_count = 0
        self.estimator_covariance: list[dict[str, float]] = []
        self.gnss_innovations: list[dict[str, Any]] = []
        self.tf_error_count = 0
        self.tf_age_max_sec = 0.0
        self.tf_timestamp_regressions = 0
        self.tf_last_stamps: dict[str, float] = {}
        self.tf_authorities: dict[str, set[str]] = {}
        self.diagnostic_errors: set[tuple[str, str]] = set()
        self.planner_failure_count = 0
        self.controller_failure_count = 0
        self.costmap_error_count = 0
        self.action_abort_count = 0
        self.failure_reason: Optional[str] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.gnss_parameters = AsyncParameterClient(
            self, '/gnss_fault_injector')
        self.goal_checker_parameters = AsyncParameterClient(
            self, '/controller_server')
        self.lifecycle_names = [
            'map_server', 'planner_server', 'controller_server',
            'behavior_server', 'bt_navigator',
        ]
        if mode == 'baseline':
            self.lifecycle_names.append('amcl')
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in self.lifecycle_names
        }

        self.create_subscription(
            Clock, '/clock', self._clock_callback, qos_profile_sensor_data)
        self.create_subscription(
            String, '/ares/week6/clock_boundary_status',
            self._clock_boundary_status_callback, 10)
        self.create_subscription(
            Odometry, '/ares/odom', self._wheel_odometry_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/ares/ground_truth',
            self._ground_truth_callback, 50)
        localization_topic = (
            '/odometry/filtered' if mode == 'baseline'
            else '/odometry/trust_fused')
        self.create_subscription(
            Odometry, localization_topic, self._localization_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odometry/filtered',
            lambda message: self._odometry_diagnostic_callback(
                'odometry_filtered', message),
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odometry/trust_fused',
            lambda message: self._odometry_diagnostic_callback(
                'odometry_trust_fused', message),
            qos_profile_sensor_data)
        self.create_subscription(
            NavPath, '/plan', self._plan_callback, 10)
        self.create_subscription(
            TwistStamped, '/cmd_vel', self._command_callback, 20)
        self.create_subscription(
            TFMessage, '/tf', self._tf_callback, 100)
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self._diagnostics_callback, 20)
        for name in ('gnss', 'imu', 'odometry'):
            self.create_subscription(
                String, f'/ares/trust/{name}',
                lambda message, sensor=name: self._trust_callback(
                    sensor, message), 10)
        self.create_subscription(
            String, '/ares/system_trust_state',
            self._system_trust_callback, 10)
        self.create_subscription(
            String, '/ares/attribution', self._attribution_callback, 10)
        self.create_subscription(
            String, '/ares/recovery_state', self._recovery_callback, 10)
        self.create_subscription(
            String, '/ares/gnss_fault_status', self._fault_callback, 10)
        self.create_subscription(
            String, '/ares/estimator_health', self._health_callback, 20)
        self.create_timer(0.1, self._sample_pose)

    def _clock_callback(self, message: Clock) -> None:
        self.sim_time = stamp_seconds(message.clock)
        wall_time = time.monotonic()
        self.clock_runtime.observe(self.sim_time, wall_time)
        self.clock_observations.append((self.sim_time, wall_time))

    def _clock_boundary_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict):
            self.clock_boundary_status = payload

    def _wheel_odometry_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.latest_wheel_odometry = (
            float(pose.position.x), float(pose.position.y),
            quaternion_yaw(pose.orientation))
        if self.mission_start_sim is not None:
            self.wheel_odometry_path.append((
                float(pose.position.x),
                float(pose.position.y),
            ))

    def _ground_truth_callback(self, message: Odometry) -> None:
        origin_x, origin_y, origin_yaw = (
            float(value) for value in
            self.config['ground_truth_world_origin'])
        pose = message.pose.pose
        world_x = float(pose.position.x)
        world_y = float(pose.position.y)
        world_yaw = quaternion_yaw(pose.orientation)
        dx = world_x - origin_x
        dy = world_y - origin_y
        # Rotate world displacement by -origin_yaw into mission-map axes.
        cosine = math.cos(origin_yaw)
        sine = math.sin(origin_yaw)
        self.latest_reference = (
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            angle_difference(world_yaw, origin_yaw),
        )

    def _localization_callback(self, message: Odometry) -> None:
        covariance = message.pose.covariance
        self.estimator_covariance.append({
            'sim_time_sec': stamp_seconds(message.header.stamp),
            'x_m2': float(covariance[0]),
            'y_m2': float(covariance[7]),
            'yaw_rad2': float(covariance[35]),
        })

    def _odometry_diagnostic_callback(
            self, source: str, message: Odometry) -> None:
        pose = message.pose.pose
        self.odometry_pose_samples[source] = {
            'frame_id': message.header.frame_id,
            'stamp': message.header.stamp,
            'stamp_sec': stamp_seconds(message.header.stamp),
            'received_sim_time_sec': self.sim_time,
            'received_wall_monotonic_sec': time.monotonic(),
            'pose': (
                float(pose.position.x),
                float(pose.position.y),
                quaternion_yaw(pose.orientation),
            ),
        }

    def _pose_report(self, sample: dict[str, Any],
                     goal: tuple[float, float, float]) -> dict[str, Any]:
        now_wall = time.monotonic()
        stamp_sec = float(sample['stamp_sec'])
        pose = sample['pose']
        frame_id = str(sample['frame_id'])
        transformed_pose: Optional[tuple[float, float, float]] = None
        transform_info: dict[str, Any] = {'status': 'not_needed'}

        if frame_id == self.config['map_frame']:
            transformed_pose = pose
        elif frame_id:
            try:
                source_time = (
                    Time() if stamp_sec == 0.0 else
                    Time.from_msg(sample['stamp']))
                transform = self.tf_buffer.lookup_transform(
                    self.config['map_frame'], frame_id, source_time)
                transform_stamp = stamp_seconds(transform.header.stamp)
                transform_pose = transform.transform
                transform_yaw = quaternion_yaw(transform_pose.rotation)
                cosine = math.cos(transform_yaw)
                sine = math.sin(transform_yaw)
                transformed_pose = (
                    float(transform_pose.translation.x) +
                    cosine * pose[0] - sine * pose[1],
                    float(transform_pose.translation.y) +
                    sine * pose[0] + cosine * pose[1],
                    angle_difference(pose[2] + transform_yaw, 0.0),
                )
                transform_info = {
                    'status': (
                        'transformed_at_odometry_stamp'
                        if stamp_sec != 0.0 else
                        'transformed_using_latest_tf_zero_source_stamp'),
                    'target_frame': self.config['map_frame'],
                    'transform_stamp_sec': transform_stamp,
                    'transform_age_at_pose_sample_sec': (
                        stamp_sec - transform_stamp),
                }
            except TransformException as error:
                transform_info = {
                    'status': 'transform_unavailable',
                    'error': f'{type(error).__name__}: {error}',
                }

        distance: Optional[float] = None
        yaw_error: Optional[float] = None
        if transformed_pose is not None:
            distance = math.hypot(
                transformed_pose[0] - goal[0],
                transformed_pose[1] - goal[1])
            yaw_error = abs(angle_difference(transformed_pose[2], goal[2]))

        age_sim = (
            None if self.sim_time is None or stamp_sec == 0.0 else
            self.sim_time - stamp_sec)
        received_sim = sample['received_sim_time_sec']
        return {
            'frame_id': frame_id,
            'pose_in_source_frame': list(pose),
            'pose_in_goal_frame': (
                list(transformed_pose)
                if transformed_pose is not None else None),
            'goal_frame': self.config['map_frame'],
            'source_stamp_sec': stamp_sec,
            'received_sim_time_sec': received_sim,
            'age_at_evaluator_sim_time_sec': age_sim,
            'age_since_receipt_wall_sec': (
                now_wall - sample['received_wall_monotonic_sec']),
            'age_at_receipt_sim_sec': (
                None if received_sim is None or stamp_sec == 0.0 else
                received_sim - stamp_sec),
            'distance_to_goal_m': distance,
            'yaw_error_rad': yaw_error,
            'transform': transform_info,
        }

    def _tf_pose_report(
            self, goal: tuple[float, float, float]
            ) -> Optional[dict[str, Any]]:
        if (self.latest_localization is None or
                self.latest_localization_stamp_sec is None):
            return None
        pose = self.latest_localization
        return {
            'frame_id': self.config['map_frame'],
            'child_frame_id': self.config['base_frame'],
            'pose_in_goal_frame': list(pose),
            'source_stamp_sec': self.latest_localization_stamp_sec,
            'age_at_evaluator_sim_time_sec': (
                None if self.sim_time is None or
                self.latest_localization_stamp_sec == 0.0 else
                self.sim_time - self.latest_localization_stamp_sec),
            'age_since_receipt_wall_sec': (
                None if self.latest_localization_received_wall is None else
                time.monotonic() - self.latest_localization_received_wall),
            'distance_to_goal_m': math.hypot(
                pose[0] - goal[0], pose[1] - goal[1]),
            'yaw_error_rad': abs(angle_difference(pose[2], goal[2])),
        }

    def _tf_pose_at_action_result(
            self, goal: tuple[float, float, float],
            action_ros_time_sec: float) -> dict[str, Any]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.config['map_frame'], self.config['base_frame'], Time())
        except TransformException as error:
            return {
                'status': 'transform_unavailable',
                'error': f'{type(error).__name__}: {error}',
            }

        stamp_sec = stamp_seconds(transform.header.stamp)
        pose = transform.transform
        position = (
            float(pose.translation.x),
            float(pose.translation.y),
            quaternion_yaw(pose.rotation),
        )
        return {
            'status': 'available',
            'frame_id': transform.header.frame_id,
            'child_frame_id': transform.child_frame_id,
            'pose_map_base': list(position),
            'source_stamp_sec': stamp_sec,
            'transform_age_at_action_result_sec': (
                None if stamp_sec == 0.0 else action_ros_time_sec - stamp_sec),
            'distance_to_goal_m': math.hypot(
                position[0] - goal[0], position[1] - goal[1]),
            'yaw_error_rad': abs(angle_difference(position[2], goal[2])),
        }

    def _rtf_interval(self, start_wall: float,
                      end_wall: float) -> dict[str, Any]:
        observations = [
            (sim_time, wall_time)
            for sim_time, wall_time in self.clock_observations
            if start_wall <= wall_time <= end_wall
        ]
        if len(observations) < 2:
            return {
                'status': 'insufficient_clock_samples',
                'sample_count': len(observations),
                'wall_duration_sec': max(0.0, end_wall - start_wall),
                'rtf': None,
            }
        sim_duration = observations[-1][0] - observations[0][0]
        wall_duration = observations[-1][1] - observations[0][1]
        return {
            'status': 'measured',
            'sample_count': len(observations),
            'first_sim_time_sec': observations[0][0],
            'last_sim_time_sec': observations[-1][0],
            'wall_duration_sec': wall_duration,
            'sim_duration_sec': sim_duration,
            'rtf': (
                None if wall_duration <= 0.0 else
                max(0.0, sim_duration / wall_duration)),
        }

    def _goal_pose_trace_sample(self, transform: Any,
                                goal: tuple[float, float, float]
                                ) -> dict[str, Any]:
        tf_pose = transform.transform
        pose = (
            float(tf_pose.translation.x),
            float(tf_pose.translation.y),
            quaternion_yaw(tf_pose.rotation),
        )
        goal_context = self.goal_diagnostic_context
        odometry: dict[str, Any] = {}
        for source, sample in self.odometry_pose_samples.items():
            odometry[source] = {
                'frame_id': sample['frame_id'],
                'pose_in_source_frame': list(sample['pose']),
                'source_stamp_sec': sample['stamp_sec'],
                'age_at_trace_sim_time_sec': (
                    None if self.sim_time is None or
                    sample['stamp_sec'] == 0.0 else
                    self.sim_time - sample['stamp_sec']),
            }
        return {
            'goal_index': (
                None if goal_context is None else
                goal_context['goal_index']),
            'sim_time_sec': self.sim_time,
            'wall_monotonic_sec': time.monotonic(),
            'tf_stamp_sec': stamp_seconds(transform.header.stamp),
            'tf_pose_map_base': list(pose),
            'tf_distance_to_goal_m': math.hypot(
                pose[0] - goal[0], pose[1] - goal[1]),
            'tf_yaw_error_rad': abs(angle_difference(pose[2], goal[2])),
            'odometry': odometry,
        }

    def _goal_completion_diagnostics(
            self, goal: tuple[float, float, float],
            status: int) -> dict[str, Any]:
        action_wall = time.monotonic()
        action_ros = stamp_seconds(self.get_clock().now().to_msg())
        evaluator_wall = time.monotonic()
        evaluator_ros = stamp_seconds(self.get_clock().now().to_msg())
        odometry: dict[str, Any] = {}
        for source in ('odometry_filtered', 'odometry_trust_fused'):
            sample = self.odometry_pose_samples.get(source)
            odometry[source] = (
                self._pose_report(sample, goal)
                if sample is not None else
                {'status': 'no_sample_received'})
        tf_report = self._tf_pose_report(goal)
        goal_context = self.goal_diagnostic_context or {}
        tf_at_result = self._tf_pose_at_action_result(goal, action_ros)
        leg_start_wall = float(goal_context.get(
            'goal_start_wall_monotonic_sec', action_wall))
        final_window_start = max(leg_start_wall, action_wall - 5.0)
        filtered = self.odometry_pose_samples.get('odometry_filtered')
        trust_fused = self.odometry_pose_samples.get(
            'odometry_trust_fused')
        return {
            'goal': {
                'waypoint_index': goal_context.get('goal_index'),
                'x_m': goal[0],
                'y_m': goal[1],
                'yaw_rad': goal[2],
            },
            'action_status': status,
            'action_result_observed': {
                'ros_time_sec': action_ros,
                'sim_clock_time_sec': self.sim_time,
                'wall_monotonic_sec': action_wall,
                'wall_utc': datetime.now(timezone.utc).isoformat(),
                'server_completion_timestamp_available': False,
            },
            'action_to_evaluator_delta_wall_sec': (
                evaluator_wall - action_wall),
            'action_to_evaluator_delta_ros_sec': (
                evaluator_ros - action_ros),
            'tf_stamp_to_evaluator_ros_delta_sec': (
                None if tf_report is None else
                evaluator_ros - float(tf_report['source_stamp_sec'])),
            'filtered_to_trust_fused_stamp_delta_sec': (
                None if filtered is None or trust_fused is None else
                float(filtered['stamp_sec']) -
                float(trust_fused['stamp_sec'])),
            'mission_evaluator_sample': {
                'ros_time_sec': evaluator_ros,
                'sim_clock_time_sec': self.sim_time,
                'wall_monotonic_sec': evaluator_wall,
                'pose_source': (
                    'latest TF lookup map_frame -> base_frame from '
                    'Week6Mission._sample_pose'),
                'pose_in_goal_frame': (
                    list(self.latest_localization)
                    if self.latest_localization is not None else None),
                'source_stamp_sec': self.latest_localization_stamp_sec,
                'distance_to_goal_m': (
                    None if self.latest_localization is None else
                    math.hypot(
                        self.latest_localization[0] - goal[0],
                        self.latest_localization[1] - goal[1])),
                'yaw_error_rad': (
                    None if self.latest_localization is None else
                    abs(angle_difference(
                        self.latest_localization[2], goal[2]))),
                'age_at_evaluator_sim_time_sec': (
                    None if self.sim_time is None or
                    self.latest_localization_stamp_sec is None or
                    self.latest_localization_stamp_sec == 0.0 else
                    self.sim_time - self.latest_localization_stamp_sec),
                'age_since_receipt_wall_sec': (
                    None if self.latest_localization_received_wall is None
                    else evaluator_wall -
                    self.latest_localization_received_wall),
            },
            'pose_sources': {
                **odometry,
                'tf_map_base': tf_report,
                'tf_map_base_at_action_result': tf_at_result,
            },
            'tf_age_at_evaluator_sim_time_sec': (
                None if tf_report is None else
                tf_report['age_at_evaluator_sim_time_sec']),
            'tf_age_at_action_result_sec': tf_at_result.get(
                'transform_age_at_action_result_sec'),
            'odometry_age_at_action_result_sec': {
                source: odometry[source].get(
                    'age_at_evaluator_sim_time_sec')
                for source in ('odometry_filtered',
                               'odometry_trust_fused')
            },
            'rtf': {
                'waypoint_leg': self._rtf_interval(
                    leg_start_wall, action_wall),
                'final_5_wall_seconds_before_action_result':
                    self._rtf_interval(final_window_start, action_wall),
            },
            'controller_goal_checker': dict(
                self.goal_checker_configuration),
            'pose_trace_around_completion': (
                [] if self.goal_diagnostic_context is None else
                self.goal_diagnostic_context['pose_trace']),
            'diagnostic_note': (
                'The NavigateToPose result has no server-side completion '
                'timestamp; action_result_observed is its client receipt.'),
        }

    def _query_goal_checker_configuration(self) -> dict[str, Any]:
        names = [
            'goal_checker.xy_goal_tolerance',
            'goal_checker.yaw_goal_tolerance',
            'goal_checker.stateful',
        ]
        if not self.goal_checker_parameters.wait_for_services(
                timeout_sec=0.5):
            return {
                'query_status': 'parameter_service_unavailable',
                'node': '/controller_server',
                'parameter_names': names,
            }
        future = self.goal_checker_parameters.get_parameters(names)
        try:
            response = self._wait_future(future, 1.0)
        except (RuntimeError, TimeoutError) as error:
            return {
                'query_status': 'parameter_query_failed',
                'node': '/controller_server',
                'parameter_names': names,
                'error': f'{type(error).__name__}: {error}',
            }
        return {
            'query_status': 'queried',
            'node': '/controller_server',
            'parameters': parameter_values(names, response.values),
        }

    def _plan_callback(self, message: NavPath) -> None:
        if not self.goal_active or self.active_goal_index is None:
            return
        points = [(float(item.pose.position.x),
                   float(item.pose.position.y)) for item in message.poses]
        if len(points) < 2:
            return
        self.current_plan = points
        index = self.active_goal_index
        if index not in self.initial_plans:
            self.initial_plans[index] = points
            self.replan_counts[index] = 0
        else:
            self.replan_counts[index] = self.replan_counts.get(index, 0) + 1

    def _command_callback(self, message: TwistStamped) -> None:
        if self.sim_time is None:
            return
        self.commands.append((
            self.sim_time, float(message.twist.linear.x),
            float(message.twist.angular.z)))

    def _tf_callback(self, message: TFMessage, info: Any) -> None:
        publisher = 'unavailable'
        try:
            publisher = bytes(info['publisher_gid']['data']).hex()
        except (KeyError, TypeError):
            pass
        for transform in message.transforms:
            child = transform.child_frame_id
            parent = transform.header.frame_id
            key = f'{parent}->{child}'
            current = stamp_seconds(transform.header.stamp)
            previous = self.tf_last_stamps.get(key)
            if previous is not None and current < previous:
                self.tf_timestamp_regressions += 1
            self.tf_last_stamps[key] = current
            self.tf_authorities.setdefault(child, set()).add(publisher)

    def _diagnostics_callback(self, message: DiagnosticArray) -> None:
        for status in message.status:
            level = (
                status.level[0]
                if isinstance(status.level, (bytes, bytearray))
                else int(status.level))
            if level < 2:
                continue
            key = (status.name, status.message)
            if key in self.diagnostic_errors:
                continue
            self.diagnostic_errors.add(key)
            rendered = f'{status.name} {status.message}'.lower()
            if 'planner' in rendered:
                self.planner_failure_count += 1
            if 'controller' in rendered:
                self.controller_failure_count += 1
            if 'costmap' in rendered:
                self.costmap_error_count += 1

    def _trust_callback(self, sensor: str, message: String) -> None:
        self.trust[sensor] = message.data

    def _system_trust_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.trust['system'] = str(payload.get(
                'system_state', payload.get('status', 'UNAVAILABLE')))
        except (json.JSONDecodeError, TypeError):
            self.trust['system'] = message.data

    def _attribution_callback(self, message: String) -> None:
        try:
            self.attribution = str(json.loads(message.data).get(
                'hypothesis', 'INSUFFICIENT_EVIDENCE'))
        except (json.JSONDecodeError, TypeError):
            self.attribution = 'INSUFFICIENT_EVIDENCE'

    def _recovery_callback(self, message: String) -> None:
        state = message.data
        if state == self.recovery_state:
            return
        self.recovery_state = state
        event = {'sim_time_sec': self.sim_time, 'state': state}
        self.recovery_events.append(event)
        if state == 'GATED':
            self.gate_count += 1
        elif state == 'PROBATION':
            self.probation_count += 1

    def _fault_callback(self, message: String) -> None:
        try:
            self.fault_status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.fault_status = {'malformed': message.data}

    def _health_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.estimator_restart_count = max(
            self.estimator_restart_count,
            int(payload.get('restart_count', 0)))
        innovation = payload.get('gnss_innovation_m')
        if innovation is not None and math.isfinite(float(innovation)):
            self.gnss_innovations.append({
                'sim_time_sec': self.sim_time,
                'innovation_m': float(innovation),
            })

    def _sample_pose(self) -> None:
        if self.sim_time is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.config['map_frame'], self.config['base_frame'], Time())
        except Exception:
            if self.goal_active:
                self.tf_error_count += 1
            return
        pose = transform.transform
        localization = (
            float(pose.translation.x), float(pose.translation.y),
            quaternion_yaw(pose.rotation))
        self.latest_localization = localization
        self.latest_localization_stamp_sec = stamp_seconds(
            transform.header.stamp)
        self.latest_localization_received_wall = time.monotonic()

        if self.goal_diagnostic_context is not None:
            context = self.goal_diagnostic_context
            completion_sim = context.get('completion_sim_time_sec')
            if (self.goal_active or completion_sim is None or
                    (self.sim_time is not None and
                     self.sim_time - completion_sim <= 2.0)):
                goal_values = context['goal']
                goal = (
                    float(goal_values[0]),
                    float(goal_values[1]),
                    float(goal_values[2]),
                )
                context['pose_trace'].append(
                    self._goal_pose_trace_sample(transform, goal))

        if self.mission_start_sim is None and self.sim_time is not None:
            self.readiness_localization_history.append(
                (self.sim_time, localization[0], localization[1],
                 localization[2])
            )
            cutoff = self.sim_time - 10.0
            self.readiness_localization_history = [
                sample for sample in self.readiness_localization_history
                if sample[0] >= cutoff
            ]

        if self.mission_start_sim is None:
            return
        if self.latest_reference is None:
            return
        actual = self.latest_reference
        self.actual_path.append((actual[0], actual[1]))
        self.localization_path.append((localization[0], localization[1]))
        relative = (
            None if self.mission_start_sim is None else
            self.sim_time - self.mission_start_sim)
        position_error = math.hypot(
            localization[0] - actual[0], localization[1] - actual[1])
        yaw_error = abs(angle_difference(localization[2], actual[2]))
        sample = {
            'sim_time_sec': self.sim_time,
            'mission_time_sec': relative,
            'position_error_m': position_error,
            'yaw_error_rad': yaw_error,
            'fault_active': self.fault_active,
        }
        self.localization_samples.append(sample)
        if self.goal_active and self.current_plan:
            distance = cross_track_errors(
                [(actual[0], actual[1])], self.current_plan)[0]
            self.cross_track_samples.append({
                'sim_time_sec': self.sim_time,
                'mission_time_sec': relative or 0.0,
                'error_m': distance,
                'fault_active': self.fault_active,
                'waypoint_index': self.active_goal_index,
            })
        transform_stamp = stamp_seconds(transform.header.stamp)
        if transform_stamp > 0.0:
            self.tf_age_max_sec = max(
                self.tf_age_max_sec,
                max(0.0, self.sim_time - transform_stamp))

    def _spin_once(self, timeout_sec: float = 0.05) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        self._update_fault_schedule()

    def _wait_future(self, future: Any, timeout_wall_sec: float) -> Any:
        deadline = time.monotonic() + timeout_wall_sec
        while not future.done() and time.monotonic() < deadline:
            self._spin_once()
        if not future.done():
            raise TimeoutError('ROS operation timed out')
        return future.result()

    def _wait_sim(self, duration_sec: float) -> None:
        start = self.sim_time
        deadline = time.monotonic() + max(30.0, duration_sec * 20.0)
        while (start is None or self.sim_time is None or
               self.sim_time - start < duration_sec):
            if time.monotonic() >= deadline:
                raise TimeoutError('simulation-time wait timed out')
            self._spin_once()
            if start is None and self.sim_time is not None:
                start = self.sim_time

    def _set_gnss_parameters(self, values: dict[str, Any]) -> None:
        future = self.gnss_parameters.set_parameters_atomically([
            Parameter(name, value=value) for name, value in values.items()
        ])
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        response = future.result() if future.done() else None
        if response is None or not response.result.successful:
            reason = ('no response' if response is None else
                      response.result.reason)
            raise RuntimeError(f'GNSS injector rejected profile: {reason}')

    def _update_fault_schedule(self) -> None:
        if (self.scenario == 'healthy' or self.mode == 'baseline' or
                self.mission_start_sim is None or self.sim_time is None):
            return
        fault = self.config['faults'][self.scenario]
        elapsed = self.sim_time - self.mission_start_sim
        onset = float(fault['onset_sim_sec'])
        duration = float(fault['duration_sim_sec'])
        if not self.fault_started and elapsed >= onset:
            self.fault_started = True
            self._set_gnss_parameters(dict(fault['parameters']))
            self.fault_active = True
            self.fault_events.append({
                'event': 'FAULT_ON', 'sim_time_sec': self.sim_time,
                'mission_time_sec': elapsed,
            })
        elif (self.fault_started and not self.fault_cleared and
              elapsed >= onset + duration):
            self._clear_fault(elapsed)

    def _clear_fault(self, elapsed: Optional[float] = None) -> None:
        self._set_gnss_parameters({
            'enabled': False, 'mode': 'none', 'east_offset_m': 0.0,
        })
        self.fault_active = False
        self.fault_cleared = True
        self.fault_events.append({
            'event': 'FAULT_OFF', 'sim_time_sec': self.sim_time,
            'mission_time_sec': elapsed,
        })

    def _lifecycle_states(self) -> dict[str, str]:
        result: dict[str, str] = {}
        for name, client in self.lifecycle_clients.items():
            if not client.wait_for_service(timeout_sec=1.0):
                result[name] = 'SERVICE_UNAVAILABLE'
                continue
            try:
                response = self._wait_future(
                    client.call_async(GetState.Request()), 3.0)
                result[name] = response.current_state.label
            except Exception as error:
                result[name] = f'ERROR: {error}'
        return result

    def wait_until_ready(self) -> dict[str, str]:
        timeout = float(self.config['readiness_timeout_wall_sec'])
        settle_sim_sec = float(
            self.config.get('readiness_settle_sim_sec', 0.0))
        minimum_sim_time_sec = float(
            self.config.get('readiness_min_sim_time_sec', 0.0))
        minimum_samples = int(
            self.config.get('readiness_min_localization_samples', 1))
        minimum_history_sec = float(
            self.config.get('readiness_min_history_sim_sec', 0.0))
        deadline = time.monotonic() + timeout
        states: dict[str, str] = {}
        ready_since_sim: Optional[float] = None

        while time.monotonic() < deadline:
            self._spin_once(0.1)
            if (self.sim_time is None or self.latest_localization is None or
                    not self.action.server_is_ready()):
                ready_since_sim = None
                continue

            states = self._lifecycle_states()
            if not states or not all(
                    value == 'active' for value in states.values()):
                ready_since_sim = None
                continue

            if self.scenario != 'healthy' and self.mode != 'baseline':
                if not self.gnss_parameters.wait_for_services(
                        timeout_sec=1.0):
                    ready_since_sim = None
                    continue

            if ready_since_sim is None:
                ready_since_sim = self.sim_time

            if self.sim_time - ready_since_sim < settle_sim_sec:
                continue

            if self.sim_time < minimum_sim_time_sec:
                continue

            samples = list(self.readiness_localization_history)
            if len(samples) < minimum_samples:
                continue
            history_duration = samples[-1][0] - samples[0][0]
            if history_duration < minimum_history_sec:
                continue

            max_span_m = 0.0
            for index, first in enumerate(samples):
                for second in samples[index + 1:]:
                    max_span_m = max(
                        max_span_m,
                        math.hypot(second[1] - first[1],
                                   second[2] - first[2]))
            covariance = (
                dict(self.estimator_covariance[-1])
                if self.estimator_covariance else None)
            self.goal_checker_configuration = (
                self._query_goal_checker_configuration())
            reference = self.latest_reference
            localization = self.latest_localization
            self.readiness_evidence = {
                'mission_start_sim_time_sec': self.sim_time,
                'localization_at_mission_start': list(localization),
                'ground_reference_at_mission_start': (
                    list(reference) if reference is not None else None),
                'localization_error_at_mission_start_m': (
                    math.hypot(localization[0] - reference[0],
                               localization[1] - reference[1])
                    if reference is not None else None),
                'readiness_history_duration_sim_sec': history_duration,
                'readiness_localization_sample_count': len(samples),
                'readiness_localization_span_m': max_span_m,
                'readiness_covariance': covariance,
                'readiness_ground_truth_used': False,
            }

            return states

        raise TimeoutError(
            f'Week 6 readiness timed out; lifecycle states={states}')

    def _feedback_callback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        self.latest_feedback = {
            'distance_remaining_m': float(feedback.distance_remaining),
            'navigation_time_sec': stamp_seconds(feedback.navigation_time),
            'number_of_recoveries': int(feedback.number_of_recoveries),
        }

    def _close_active_window(self) -> None:
        if self.active_window_start is not None and self.sim_time is not None:
            self.active_windows.append(
                (self.active_window_start, self.sim_time))
        self.active_window_start = None
        self.goal_active = False

    def run(self) -> dict[str, Any]:
        pre_states = self.wait_until_ready()
        self.mission_start_sim = self.sim_time
        mission_start_wall = time.monotonic()
        for index, waypoint in enumerate(self.config['waypoints']):
            x, y, target_yaw = (float(value) for value in waypoint)
            self.active_goal_index = index
            self.current_plan = []
            self.latest_feedback = {}
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = self.config['map_frame']
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(target_yaw / 2.0)
            goal_tuple = (x, y, target_yaw)
            self.goal_diagnostic_context = {
                'goal_index': index,
                'goal': list(goal_tuple),
                'pose_trace': [],
                'goal_start_wall_monotonic_sec': time.monotonic(),
            }
            sent_at = self.sim_time
            handle = self._wait_future(
                self.action.send_goal_async(
                    goal, feedback_callback=self._feedback_callback), 10.0)
            if not handle.accepted:
                raise RuntimeError(f'waypoint {index + 1} rejected')
            self.goal_active = True
            self.active_window_start = self.sim_time
            result = self._wait_future(
                handle.get_result_async(),
                float(self.config['goal_timeout_wall_sec']))
            self._close_active_window()
            status = int(result.status)
            if status == 6:
                self.action_abort_count += 1
            localization = self.latest_localization
            reference = self.latest_reference
            if localization is None or reference is None:
                raise RuntimeError('pose unavailable at waypoint result')
            localized_distance = math.hypot(
                localization[0] - x, localization[1] - y)
            actual_distance = math.hypot(
                reference[0] - x, reference[1] - y)
            yaw_error = abs(angle_difference(localization[2], target_yaw))
            reached = (
                status == 4 and
                localized_distance <=
                float(self.config['waypoint_xy_tolerance_m']) and
                yaw_error <=
                float(self.config['waypoint_yaw_tolerance_rad']))
            completion_diagnostics = self._goal_completion_diagnostics(
                goal_tuple, status)
            self.goal_completion_diagnostics.append(
                completion_diagnostics)
            if self.goal_diagnostic_context is not None:
                self.goal_diagnostic_context[
                    'completion_sim_time_sec'] = self.sim_time
            record = {
                'waypoint_index': index,
                'goal': [x, y, target_yaw],
                'action_status': status,
                'reached': reached,
                'arrival_sim_time_sec': self.sim_time,
                'duration_sim_sec': (
                    None if sent_at is None or self.sim_time is None else
                    self.sim_time - sent_at),
                'final_distance_m': localized_distance,
                'ground_reference_distance_m': actual_distance,
                'yaw_error_rad': yaw_error,
                'fault_active_at_arrival': self.fault_active,
                'gnss_trust_at_arrival': self.trust['gnss'],
                'recovery_state_at_arrival': self.recovery_state,
                'replan_count': self.replan_counts.get(index, 0),
                'nav_recovery_count': int(self.latest_feedback.get(
                    'number_of_recoveries', 0)),
                'completion_diagnostics': completion_diagnostics,
            }
            self.waypoints.append(record)
            self.get_logger().info(json.dumps(record, sort_keys=True))
            if not reached:
                break
            self._wait_sim(float(self.config['waypoint_dwell_sim_sec']))
        self.mission_end_sim = self.sim_time
        if self.fault_active:
            elapsed = (
                None if self.mission_start_sim is None or self.sim_time is None
                else self.sim_time - self.mission_start_sim)
            self._clear_fault(elapsed)
        self._wait_sim(float(
            self.config['post_mission_observation_sim_sec']))
        post_states = self._lifecycle_states()
        return self._result(
            pre_states, post_states, time.monotonic() - mission_start_wall)

    def _result(self, pre_states: dict[str, str],
                post_states: dict[str, str],
                duration_wall_sec: float) -> dict[str, Any]:
        fault = self.config['faults'][self.scenario]
        onset = fault.get('onset_sim_sec')
        duration = float(fault.get('duration_sim_sec', 0.0))
        localization_errors = [
            float(item['position_error_m'])
            for item in self.localization_samples]
        fault_errors = [
            float(item['position_error_m'])
            for item in self.localization_samples
            if item['fault_active']]
        post_fault_errors = [
            float(item['position_error_m'])
            for item in self.localization_samples
            if (onset is not None and item['mission_time_sec'] is not None and
                item['mission_time_sec'] >= float(onset) + duration)]
        cte = [float(item['error_m'])
               for item in self.cross_track_samples]
        fault_cte = [float(item['error_m'])
                     for item in self.cross_track_samples
                     if item['fault_active']]
        planned_length = sum(
            path_length(self.initial_plans[index])
            for index in sorted(self.initial_plans))
        actual_length = path_length(self.actual_path)
        waypoint_metrics = waypoint_summary(self.waypoints)
        final_ground_reference_error = (
            self.waypoints[-1].get('ground_reference_distance_m')
            if self.waypoints else None)
        lifecycle_failure_count = sum(
            value != 'active' for value in post_states.values())
        classification = classify_mission(
            self.waypoints,
            controller_failed=self.controller_failure_count > 0,
            tf_failure_count=self.tf_error_count,
            estimator_restart_count=self.estimator_restart_count,
        )
        gap_metrics = controller_command_gaps(
            self.commands, self.active_windows,
            float(self.config['controller_gap_threshold_sec']))
        clear_time = next((event['sim_time_sec'] for event in self.fault_events
                           if event['event'] == 'FAULT_OFF'), None)
        normal_time = next(
            (event['sim_time_sec'] for event in self.recovery_events
             if (event['state'] == 'NORMAL' and clear_time is not None
                 and event['sim_time_sec'] is not None
                 and event['sim_time_sec'] >= clear_time)),
            None)
        resume = (
            normal_time - clear_time
            if normal_time is not None and clear_time is not None else None)
        cte_summary = summarize_errors(cte)
        localization_summary = summarize_errors(localization_errors)
        result = {
            'schema_version': 1,
            'mission_id': self.config['mission_id'],
            'navigation_mode': self.mode,
            'scenario': self.scenario,
            'fault_profile': fault['profile'],
            'seed': self.seed,
            **self.readiness_evidence,
            **classification,
            **waypoint_metrics,
            'ground_reference_final_goal_error_m':
                final_ground_reference_error,
            'mission_completion_time_sec': (
                None if self.mission_start_sim is None or
                self.mission_end_sim is None else
                self.mission_end_sim - self.mission_start_sim),
            'mission_wall_time_sec': duration_wall_sec,
            'actual_path_length_m': actual_length,
            'planned_path_length_m': planned_length,
            'path_length_ratio': (
                actual_length / planned_length if planned_length > 0.0 else None),
            'mean_cross_track_error_m': cte_summary['mean'],
            'median_cross_track_error_m': cte_summary['median'],
            'p95_cross_track_error_m': cte_summary['p95'],
            'max_cross_track_error_m': cte_summary['max'],
            'fault_time_mean_cross_track_error_m':
                summarize_errors(fault_cte)['mean'],
            'fault_time_max_cross_track_error_m':
                summarize_errors(fault_cte)['max'],
            'max_localization_error_m': localization_summary['max'],
            'mean_localization_error_m': localization_summary['mean'],
            'fault_time_max_localization_error_m':
                summarize_errors(fault_errors)['max'],
            'post_fault_max_localization_error_m':
                summarize_errors(post_fault_errors)['max'],
            **gap_metrics,
            'runtime_quality': {
                **self.clock_runtime.summary(),
                'clock_boundary_status': dict(self.clock_boundary_status),
                # Filled from the retained launch log by the outer runner.
                'controller_rate_miss_count': None,
            },
            'planner_failure_count': self.planner_failure_count,
            'controller_failure_count': self.controller_failure_count,
            'nav_recovery_count': sum(int(item['nav_recovery_count'])
                                      for item in self.waypoints),
            'nav2_abort_count': self.action_abort_count,
            'tf_error_count': self.tf_error_count,
            'tf_timestamp_regression_count': self.tf_timestamp_regressions,
            'tf_max_age_sec': self.tf_age_max_sec,
            'estimator_restart_count': self.estimator_restart_count,
            'ares_gate_count': self.gate_count,
            'ares_probation_count': self.probation_count,
            'time_to_resume_stable_navigation_sec': resume,
            'lifecycle_failure_count': lifecycle_failure_count,
            'costmap_error_count': self.costmap_error_count,
            'collision_count': None,
            'waypoints': self.waypoints,
            'goal_completion_diagnostics':
                self.goal_completion_diagnostics,
            'lifecycle_pre': pre_states,
            'lifecycle_post': post_states,
            'fault_events': self.fault_events,
            'recovery_events': self.recovery_events,
            'final_trust': dict(self.trust),
            'final_attribution': self.attribution,
            'final_recovery_state': self.recovery_state,
            'dynamic_tf_authority_count': {
                child: len(authorities)
                for child, authorities in self.tf_authorities.items()},
            'diagnostic_errors': [
                {'name': name, 'message': message}
                for name, message in sorted(self.diagnostic_errors)],
            'max_abs_linear_command_mps': max(
                (abs(item[1]) for item in self.commands), default=None),
            'max_abs_angular_command_radps': max(
                (abs(item[2]) for item in self.commands), default=None),
            'raw': {
                'actual_path_xy': self.actual_path,
                'wheel_odometry_path_xy': self.wheel_odometry_path,
                'localization_path_xy': self.localization_path,
                'localization_samples': self.localization_samples,
                'cross_track_samples': self.cross_track_samples,
                'commands': self.commands,
                'initial_plans': {
                    str(index): points
                    for index, points in self.initial_plans.items()},
                'estimator_covariance': self.estimator_covariance,
                'gnss_innovations': self.gnss_innovations,
            },
            'measurement_notes': {
                'ground_reference': (
                    'Gazebo /world/ares_world/pose/info model pose, bridged '
                    'by an opt-in OdometryPublisher as /ares/ground_truth and '
                    'transformed '
                    'from the fixed world spawn into mission-map axes'),
                'cross_track': (
                    'raw odometry path to the active Nav2 planned path'),
                'runtime_quality': (
                    'diagnostic only; RTF is simulation-clock advance '
                    'divided by wall advance over one-second windows; a '
                    'severe collapse is RTF below 0.50 and does not change '
                    'mission acceptance'),
            },
            'configuration': self.config,
            'controller_goal_checker_configuration':
                self.goal_checker_configuration,
        }
        if lifecycle_failure_count:
            result['mission_completed'] = False
            result['mission_failure_reasons'].append('lifecycle_failure')
        return result


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    default_config = str(Path(get_package_share_directory(
        'ares_reliability')) / 'config' / 'week6_mission.yaml')
    parser.add_argument('--config', default=default_config)
    parser.add_argument('--navigation-mode',
                        choices=('baseline', 'unprotected', 'protected'),
                        required=True)
    parser.add_argument('--scenario',
                        choices=('healthy', 'gnss_step_5m'), required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--output', required=True)
    parsed, ros_args = parser.parse_known_args(args)
    output = Path(parsed.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(output)
    config = yaml.safe_load(Path(parsed.config).read_text(encoding='utf-8'))
    rclpy.init(args=ros_args)
    node = Week6Mission(
        config, parsed.navigation_mode, parsed.scenario, parsed.seed)
    result: dict[str, Any] = {
        'schema_version': 1,
        'navigation_mode': parsed.navigation_mode,
        'scenario': parsed.scenario,
        'seed': parsed.seed,
        'mission_completed': False,
    }
    try:
        result = node.run()
    except Exception as error:
        node.failure_reason = str(error)
        if node.mission_start_sim is not None:
            node.mission_end_sim = node.sim_time
            node._close_active_window()
            try:
                post_states = node._lifecycle_states()
            except Exception:
                post_states = {}
            result = node._result(
                {}, post_states, time.monotonic() - node.created_wall)
            result['mission_completed'] = False
            result['mission_failure_reasons'].append(
                f'runner_error: {error}')
        else:
            result['mission_failure_reasons'] = [f'runner_error: {error}']
    finally:
        output.write_text(
            json.dumps(result, indent=2, allow_nan=False) + '\n',
            encoding='utf-8')
        node.get_logger().info(
            f'Week 6 result written to {output}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result.get('mission_completed') else 1)


if __name__ == '__main__':
    main()
