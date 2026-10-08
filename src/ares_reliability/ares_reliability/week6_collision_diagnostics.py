"""Capture sensor, path, command, TF, costmap, and RPP collision evidence."""

from __future__ import annotations

import argparse
import json
import math
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from action_msgs.msg import GoalStatusArray


from geometry_msgs.msg import PointStamped, Twist, TwistStamped


from nav2_msgs.msg import Costmap


from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath


from rcl_interfaces.msg import Log


import rclpy
from rclpy.serialization import deserialize_message
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.exceptions import InvalidTopicNameException
from rclpy.validate_full_topic_name import validate_full_topic_name


from rosgraph_msgs.msg import Clock


from sensor_msgs.msg import LaserScan


from tf2_msgs.msg import TFMessage


COLLISION_ERROR = 'RegulatedPurePursuitController detected collision ahead!'
ROBOT_RADIUS_M = 0.30
DIAGNOSTIC_TOPIC_TYPES = {
    '/curvature_lookahead_point': ('geometry_msgs/msg/PointStamped',),
    '/global_plan': ('nav_msgs/msg/Path',),
    '/local_costmap/costmap': ('nav_msgs/msg/OccupancyGrid',),
    '/local_costmap/costmap_raw': ('nav2_msgs/msg/Costmap',),
    '/local_costmap/obstacle_layer': ('nav_msgs/msg/OccupancyGrid',),
    '/local_costmap/obstacle_layer_raw': ('nav2_msgs/msg/Costmap',),
    '/local_plan': ('nav_msgs/msg/Path',),
    '/lookahead_collision_arc': ('nav_msgs/msg/Path',),
    '/lookahead_point': ('geometry_msgs/msg/PointStamped',),
    '/plan': ('nav_msgs/msg/Path',),
    '/transformed_global_plan': ('nav_msgs/msg/Path',),
}
WAYPOINTS = (
    (2.0, 0.0, 0.0),
    (6.0, -2.0, 0.0),
    (6.0, 2.0, 1.5707963267948966),
    (3.0, 2.0, 3.141592653589793),
    (2.0, 0.0, -1.5707963267948966),
)


def _stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _quaternion_yaw(rotation: Any) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))


class Week6CollisionDiagnostics(Node):
    """Keep bounded latest-sample context and snapshot it on RPP log events."""

    def __init__(self, output_path: Path, run_name: str) -> None:
        super().__init__('week6_collision_diagnostics')
        self.output_path = output_path
        self.run_name = run_name
        self.started_wall = time.monotonic()
        self.latest_ros_time: float | None = None
        self.latest: dict[str, dict[str, Any]] = {}
        self._diagnostic_subscriptions = []
        self.subscribed_topics: set[str] = set()
        self.topic_discovery_skips: list[dict[str, Any]] = []
        self._recorded_topic_discovery_skips: set[
            tuple[str, str]] = set()
        self.subscription_failures: list[dict[str, str]] = []
        self.disabled_topics: list[str] = []
        self.message_conversion_failures: list[dict[str, str]] = []
        self.events: list[dict[str, Any]] = []
        self.max_command_gap_sec = 0.0
        self.last_command_wall: float | None = None
        self.max_costmap_interarrival_sec = 0.0
        self.last_costmap_wall: float | None = None
        self.tf_poses: dict[str, dict[str, Any]] = {}
        self.transforms: dict[tuple[str, str], dict[str, Any]] = {}
        self.topic_inventory: dict[str, list[str]] = {}
        self.stream_timing: dict[str, dict[str, Any]] = {}
        self.costmap_update_timestamps: list[dict[str, Any]] = []
        self.sim_wall_timeline: list[dict[str, Any]] = []
        self.relevant_log_events: list[dict[str, Any]] = []
        self.local_costmap_topic: str | None = None
        qos = QoSProfile(
            depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._subscribe('/rosout', Log, self._log_callback, qos)
        self._subscribe('/clock', Clock, self._clock_callback, qos)
        self._subscribe('/ares/scan', LaserScan, self._scan_callback, qos)
        self._subscribe(
            '/odometry/trust_fused', Odometry, self._odom_callback, qos)
        self._subscribe(
            '/cmd_vel', TwistStamped, self._stamped_cmd_callback, qos)
        self._subscribe('/ares/cmd_vel', Twist, self._cmd_callback, qos)
        self._subscribe('/tf', TFMessage, self._tf_callback, qos)
        self._subscribe('/map', OccupancyGrid, self._map_callback, qos)
        self._subscribe(
            '/follow_path/_action/status', GoalStatusArray,
            self._action_status_callback, qos)
        static_tf_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._subscribe(
            '/tf_static', TFMessage, self._tf_callback, static_tf_qos)
        self._discover_diagnostic_topics(qos)
        self.create_timer(
            0.5, lambda: self._discover_diagnostic_topics(qos))
        self.create_timer(0.5, self._record_sim_wall_pair)

    def _subscribe(self, topic: str, message_type: Any,
                   callback: Any, qos: QoSProfile,
                   *, optional: bool = False) -> bool:
        try:
            validate_full_topic_name(topic)
        except InvalidTopicNameException as error:
            self._record_topic_discovery_skip(
                topic, (), 'invalid_topic_name', str(error))
            return False
        if topic in self.subscribed_topics:
            return False
        subscription_callback = (
            self._raw_diagnostic_callback(topic, message_type, callback)
            if optional else callback)
        try:
            subscription = self.create_subscription(
                message_type, topic, subscription_callback, qos,
                raw=optional)
        except Exception as error:
            failure = {
                'topic': topic,
                'message_type': (
                    f'{message_type.__module__}.{message_type.__name__}'),
                'error': f'{type(error).__name__}: {error}',
            }
            self.subscription_failures.append(failure)
            self.disabled_topics.append(topic)
            self.get_logger().error(
                f"Skipping optional diagnostic topic {topic}: "
                f"{failure['error']}")
            return False
        self._diagnostic_subscriptions.append(subscription)
        self.subscribed_topics.add(topic)
        return True

    def _raw_diagnostic_callback(
            self, topic: str, message_type: Any, callback: Any) -> Any:
        def receive(serialized_message: bytes) -> None:
            try:
                message = deserialize_message(
                    serialized_message, message_type)
            except Exception as error:
                failure = {
                    'topic': topic,
                    'message_type': (
                        f'{message_type.__module__}.{message_type.__name__}'),
                    'error': f'{type(error).__name__}: {error}',
                }
                self.message_conversion_failures.append(failure)
                self.disabled_topics.append(topic)
                self._disable_diagnostic_topic(topic)
                self.get_logger().error(
                    f"Disabled optional diagnostic topic {topic} after "
                    f"message conversion failure: {failure['error']}")
                return
            callback(message)
        return receive

    def _disable_diagnostic_topic(self, topic: str) -> None:
        for subscription in tuple(self._diagnostic_subscriptions):
            if subscription.topic_name != topic:
                continue
            self.destroy_subscription(subscription)
            self._diagnostic_subscriptions.remove(subscription)
            self.subscribed_topics.discard(topic)
            return

    def _record_topic_discovery_skip(
            self, topic: str, types: tuple[str, ...] | list[str],
            reason: str, detail: str | None = None) -> None:
        key = (topic, reason)
        if key in self._recorded_topic_discovery_skips:
            return
        self._recorded_topic_discovery_skips.add(key)
        record: dict[str, Any] = {
            'topic': topic,
            'types': list(types),
            'reason': reason,
        }
        if detail is not None:
            record['detail'] = detail
        self.topic_discovery_skips.append(record)

    def _remember(self, key: str, message: Any,
                  stamp: float | None = None) -> None:
        now = time.monotonic()
        self.latest[key] = {
            'message': message,
            'wall_monotonic_sec': now,
            'ros_stamp_sec': stamp,
        }
        timing = self.stream_timing.setdefault(key, {
            'sample_count': 0,
            'first_wall_monotonic_sec': now,
            'last_wall_monotonic_sec': None,
            'first_ros_stamp_sec': stamp,
            'last_ros_stamp_sec': None,
            'max_wall_interarrival_sec': 0.0,
        })
        previous = timing['last_wall_monotonic_sec']
        timing['sample_count'] += 1
        timing['last_wall_monotonic_sec'] = now
        timing['last_ros_stamp_sec'] = stamp
        if previous is not None:
            timing['max_wall_interarrival_sec'] = max(
                timing['max_wall_interarrival_sec'], now - previous)

    def _record_sim_wall_pair(self) -> None:
        self.sim_wall_timeline.append({
            'simulation_time_sec': self.latest_ros_time,
            'wall_monotonic_sec': time.monotonic(),
            'wall_utc': datetime.now(timezone.utc).isoformat(),
        })

    def _discover_diagnostic_topics(self, qos: QoSProfile) -> None:
        known = {
            '/curvature_lookahead_point': PointStamped,
            '/global_plan': NavPath,
            '/local_costmap/costmap': (
                OccupancyGrid),
            '/local_costmap/costmap_raw': (
                Costmap),
            '/local_costmap/obstacle_layer': (
                OccupancyGrid),
            '/local_costmap/obstacle_layer_raw': (
                Costmap),
            '/local_plan': NavPath,
            '/lookahead_collision_arc': NavPath,
            '/lookahead_point': PointStamped,
            '/plan': NavPath,
            '/transformed_global_plan': NavPath,
        }
        for topic, types in self.get_topic_names_and_types():
            try:
                validate_full_topic_name(topic)
            except InvalidTopicNameException as error:
                self._record_topic_discovery_skip(
                    topic, types, 'invalid_topic_name', str(error))
                continue
            selected = known.get(topic)
            if selected is None:
                self._record_topic_discovery_skip(
                    topic, types, 'not_allowlisted')
                continue
            message_type = selected
            expected_type = DIAGNOSTIC_TOPIC_TYPES[topic][0]
            if expected_type not in types:
                self._record_topic_discovery_skip(
                    topic, types, 'unexpected_message_type',
                    f'expected {expected_type}')
                continue
            if message_type is Costmap:
                callback = self._costmap_callback_for(topic)
            elif message_type is OccupancyGrid:
                callback = self._occupancy_callback_for(topic)
            elif message_type is NavPath:
                callback = self._path_callback_for(topic)
            else:
                callback = self._point_callback_for(topic)
            if self._subscribe(
                    topic, message_type, callback, qos, optional=True):
                self.topic_inventory[topic] = list(types)
                if message_type in (Costmap, OccupancyGrid):
                    self.local_costmap_topic = topic

    def _clock_callback(self, message: Clock) -> None:
        self.latest_ros_time = _stamp_seconds(message.clock)

    def _scan_callback(self, message: LaserScan) -> None:
        self._remember(
            'scan', message, _stamp_seconds(message.header.stamp))

    def _odom_callback(self, message: Odometry) -> None:
        self._remember(
            'odometry', message, _stamp_seconds(message.header.stamp))

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._remember('static_map', message, _stamp_seconds(
            message.header.stamp))

    def _action_status_callback(self, message: GoalStatusArray) -> None:
        self._remember('follow_path_status', message)

    def _stamped_cmd_callback(self, message: TwistStamped) -> None:
        self._record_command_gap()
        self._remember(
            'cmd_vel', message, _stamp_seconds(message.header.stamp))

    def _cmd_callback(self, message: Twist) -> None:
        self._record_command_gap()
        self._remember('ares_cmd_vel', message)

    def _record_command_gap(self) -> None:
        now = time.monotonic()
        if self.last_command_wall is not None:
            self.max_command_gap_sec = max(
                self.max_command_gap_sec, now - self.last_command_wall)
        self.last_command_wall = now

    def _costmap_callback(self, message: Costmap) -> None:
        self._record_costmap_update()
        self._remember(
            'local_costmap', message, _stamp_seconds(message.header.stamp))

    def _costmap_callback_for(self, topic: str) -> Any:
        def callback(message: Costmap) -> None:
            self._costmap_callback(message)
            self.local_costmap_topic = topic
        return callback

    def _occupancy_callback(self, message: OccupancyGrid) -> None:
        self._record_costmap_update()
        self._remember(
            'local_costmap', message, _stamp_seconds(message.header.stamp))

    def _occupancy_callback_for(self, topic: str) -> Any:
        def callback(message: OccupancyGrid) -> None:
            self._occupancy_callback(message)
            self.local_costmap_topic = topic
        return callback

    def _record_costmap_update(self) -> None:
        now = time.monotonic()
        if self.last_costmap_wall is not None:
            self.max_costmap_interarrival_sec = max(
                self.max_costmap_interarrival_sec,
                now - self.last_costmap_wall)
        self.last_costmap_wall = now
        self.costmap_update_timestamps.append({
            'simulation_time_sec': self.latest_ros_time,
            'wall_monotonic_sec': now,
            'wall_utc': datetime.now(timezone.utc).isoformat(),
        })

    def _path_callback(self, message: NavPath) -> None:
        self._remember(
            'path', message, _stamp_seconds(message.header.stamp))

    def _path_callback_for(self, topic: str) -> Any:
        def callback(message: NavPath) -> None:
            self._remember(
                topic, message, _stamp_seconds(message.header.stamp))
        return callback

    def _point_callback(self, message: PointStamped) -> None:
        self._remember(
            'lookahead_point', message, _stamp_seconds(message.header.stamp))

    def _point_callback_for(self, topic: str) -> Any:
        def callback(message: PointStamped) -> None:
            self._remember(
                topic, message, _stamp_seconds(message.header.stamp))
        return callback

    def _tf_callback(self, message: TFMessage) -> None:
        now = time.monotonic()
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')
            translation = transform.transform.translation
            self.transforms[(parent, child)] = {
                'parent_frame': parent,
                'child_frame': child,
                'x': translation.x,
                'y': translation.y,
                'yaw_rad': _quaternion_yaw(transform.transform.rotation),
                'stamp_sec': _stamp_seconds(transform.header.stamp),
                'wall_monotonic_sec': now,
            }
            if transform.child_frame_id not in (
                    'base_footprint', 'base_link'):
                continue
            self.tf_poses[transform.header.frame_id] = {
                'frame_id': transform.header.frame_id,
                'child_frame_id': transform.child_frame_id,
                'x': translation.x,
                'y': translation.y,
                'yaw_rad': _quaternion_yaw(transform.transform.rotation),
                'stamp_sec': _stamp_seconds(transform.header.stamp),
                'wall_monotonic_sec': now,
            }

    def _log_callback(self, message: Log) -> None:
        interesting = any(token in message.msg for token in (
            COLLISION_ERROR,
            'Control loop missed its desired rate',
            '[follow_path] [ActionServer] Aborting handle',
            'Received request to clear entirely',
            'Clearing all layers in costmap',
            'Creating new path',
            'Begin computing',
            'Recovery',
            'recovery',
        ))
        if interesting:
            self.relevant_log_events.append({
                'log_timestamp_sec': _stamp_seconds(message.stamp),
                'simulation_time_sec': self.latest_ros_time,
                'wall_monotonic_sec': time.monotonic(),
                'wall_utc': datetime.now(timezone.utc).isoformat(),
                'logger': message.name,
                'level': int(message.level),
                'message': message.msg,
            })
        if COLLISION_ERROR not in message.msg:
            return
        costmap = self._costmap_snapshot()
        colliding_sample = None
        first_non_free = None
        if costmap is not None:
            colliding_sample = next((
                sample for sample in costmap['arc_center_cell_samples']
                if sample['footprint_collision']), None)
            first_non_free = next((
                sample for sample in costmap['arc_center_cell_samples']
                if sample['footprint_max_cost'] > 0), None)
        scan = self._scan_snapshot(colliding_sample)
        paths = self._path_snapshots(colliding_sample)
        self.events.append({
            'event_index': len(self.events) + 1,
            'simulation_timestamp_sec': self.latest_ros_time,
            'controller_log_timestamp_sec': _stamp_seconds(message.stamp),
            'timestamp_wall_utc': datetime.now(timezone.utc).isoformat(),
            'timestamp_wall_monotonic_sec': time.monotonic(),
            'controller_error_text': message.msg,
            'logger': message.name,
            'robot_pose': self._robot_pose(),
            'commands': self._commands(),
            'arc': self._arc_snapshot(),
            'local_costmap': costmap,
            'first_colliding_arc_sample': colliding_sample,
            'first_non_free_arc_sample': first_non_free,
            'collision_point_static_map': self._static_map_sample(
                colliding_sample),
            'scan': scan,
            'odometry': self._odometry_snapshot(),
            'tf_base_poses': self.tf_poses.copy(),
            'paths': paths,
            'current_waypoint': self._infer_current_waypoint(),
            'controller_timing_at_event': {
                'max_command_gap_sec': self.max_command_gap_sec,
                'max_costmap_interarrival_sec':
                    self.max_costmap_interarrival_sec,
            },
            'nav2_recovered': None,
            'follow_path_aborted': None,
            'recovery_invoked': None,
            'subsequent_replan_observed': None,
            'same_waypoint_ultimately_succeeded': None,
            'classification': 'F',
            'classification_name': 'INSUFFICIENT_EVIDENCE',
            'classification_evidence': [],
            'mission_eventually_completed': None,
        })

    def _sample(self, key: str) -> dict[str, Any] | None:
        value = self.latest.get(key)
        if value is None:
            return None
        age = None
        if (self.latest_ros_time is not None and
                value.get('ros_stamp_sec') is not None):
            age = self.latest_ros_time - value['ros_stamp_sec']
        return {
            'age_ros_sec': age,
            'age_wall_sec': time.monotonic() - value['wall_monotonic_sec'],
            'message': value['message'],
        }

    @staticmethod
    def _apply_transform(
            point: tuple[float, float], transform: dict[str, Any],
            inverse: bool = False) -> tuple[float, float]:
        """Transform a 2-D point across one parent/child TF edge."""
        x, y = point
        tx = float(transform['x'])
        ty = float(transform['y'])
        yaw = float(transform['yaw_rad'])
        if inverse:
            dx, dy = x - tx, y - ty
            return (
                math.cos(yaw) * dx + math.sin(yaw) * dy,
                -math.sin(yaw) * dx + math.cos(yaw) * dy,
            )
        return (
            math.cos(yaw) * x - math.sin(yaw) * y + tx,
            math.sin(yaw) * x + math.cos(yaw) * y + ty,
        )

    def _transform_point(
            self, x: float, y: float, source_frame: str,
            target_frame: str) -> tuple[float, float] | None:
        """Transform a point using the latest captured planar TF graph."""
        source = source_frame.lstrip('/')
        target = target_frame.lstrip('/')
        if source == target:
            return (x, y)
        frontier: list[tuple[str, tuple[float, float]]] = [
            (source, (x, y))]
        visited = {source}
        while frontier:
            frame, point = frontier.pop(0)
            for (parent, child), transform in self.transforms.items():
                if frame == child and parent not in visited:
                    moved = self._apply_transform(point, transform)
                    if parent == target:
                        return moved
                    visited.add(parent)
                    frontier.append((parent, moved))
                if frame == parent and child not in visited:
                    moved = self._apply_transform(
                        point, transform, inverse=True)
                    if child == target:
                        return moved
                    visited.add(child)
                    frontier.append((child, moved))
        return None

    def _robot_pose(self) -> dict[str, Any] | None:
        sample = self._sample('odometry')
        if sample is None:
            return None
        odom = sample['message']
        pose = odom.pose.pose
        return {
            'source': 'odometry/trust_fused',
            'frame_id': odom.header.frame_id,
            'child_frame_id': odom.child_frame_id,
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw_rad': _quaternion_yaw(pose.orientation),
            'age_ros_sec': sample['age_ros_sec'],
            'age_wall_sec': sample['age_wall_sec'],
        }

    def _commands(self) -> dict[str, Any]:
        commands: dict[str, Any] = {}
        for key in ('cmd_vel', 'ares_cmd_vel'):
            sample = self._sample(key)
            if sample is None:
                continue
            message = sample['message']
            twist = (
                message.twist
                if isinstance(message, TwistStamped) else message)
            commands[key] = {
                'linear_x_mps': twist.linear.x,
                'angular_z_radps': twist.angular.z,
                'age_ros_sec': sample['age_ros_sec'],
                'age_wall_sec': sample['age_wall_sec'],
            }
        return commands

    def _arc_snapshot(self) -> dict[str, Any]:
        arc_samples = {
            name: data for name, data in self.latest.items()
            if 'arc' in name.lower()
        }
        result: dict[str, Any] = {
            'published_collision_arc_topics': [
                name for name in self.topic_inventory
                if 'arc' in name.lower()
            ],
            'samples': {},
            'lookahead_point': None,
        }
        for name, value in arc_samples.items():
            message = value['message']
            if isinstance(message, NavPath):
                result['samples'][name] = {
                    'frame_id': message.header.frame_id,
                    'sample_count': len(message.poses),
                    'age_ros_sec': (
                        None if self.latest_ros_time is None or
                        value['ros_stamp_sec'] is None else
                        self.latest_ros_time - value['ros_stamp_sec']),
                    'age_wall_sec': (
                        time.monotonic() - value['wall_monotonic_sec']),
                    'poses': [
                        {
                            'x': item.pose.position.x,
                            'y': item.pose.position.y,
                        }
                        for item in message.poses
                    ],
                }
            elif isinstance(message, PointStamped):
                result['samples'][name] = {
                    'frame_id': message.header.frame_id,
                    'sample_count': 1,
                    'age_ros_sec': (
                        None if self.latest_ros_time is None or
                        value['ros_stamp_sec'] is None else
                        self.latest_ros_time - value['ros_stamp_sec']),
                    'age_wall_sec': (
                        time.monotonic() - value['wall_monotonic_sec']),
                    'poses': [{
                        'x': message.point.x,
                        'y': message.point.y,
                    }],
                }
        point = self.latest.get('lookahead_point')
        if point is None:
            point = next((
                value for name, value in self.latest.items()
                if 'lookahead_point' in name.lower()), None)
        if point is not None and isinstance(
                point['message'], PointStamped):
            message = point['message']
            result['lookahead_point'] = {
                'frame_id': message.header.frame_id,
                'x': message.point.x,
                'y': message.point.y,
            }
        result['sample_count'] = max(
            (item['sample_count'] for item in result['samples'].values()),
            default=0)
        return result

    def _costmap_snapshot(self) -> dict[str, Any] | None:
        sample = self._sample('local_costmap')
        if sample is None:
            return None
        message = sample['message']
        if isinstance(message, Costmap):
            metadata = message.metadata
            resolution = float(metadata.resolution)
            size_x, size_y = int(metadata.size_x), int(metadata.size_y)
            origin = metadata.origin
            costs = message.data
            update_stamp = _stamp_seconds(metadata.update_time)
        else:
            metadata = message.info
            resolution = float(metadata.resolution)
            size_x, size_y = int(metadata.width), int(metadata.height)
            origin = metadata.origin
            costs = message.data
            update_stamp = sample['ros_stamp_sec']
        cells = self._arc_cost_samples(
            costs, size_x, size_y, resolution, origin,
            message.header.frame_id)
        return {
            'topic': self.local_costmap_topic,
            'frame_id': message.header.frame_id,
            'size_x': size_x,
            'size_y': size_y,
            'resolution_m': resolution,
            'age_ros_sec': sample['age_ros_sec'],
            'age_wall_sec': sample['age_wall_sec'],
            'update_age_ros_sec': (
                None if self.latest_ros_time is None or update_stamp is None
                else self.latest_ros_time - update_stamp),
            'arc_center_cell_samples': cells,
            'first_non_free_arc_sample': next((
                item for item in cells
                if item['footprint_max_cost'] > 0), None),
            'first_footprint_collision_arc_sample': next((
                item for item in cells
                if item['footprint_collision']), None),
        }

    @staticmethod
    def _cost_type(cost: int) -> str:
        if cost == 255:
            return 'UNKNOWN'
        if cost == 254:
            return 'LETHAL_OBSTACLE'
        if cost == 253:
            return 'INSCRIBED_INFLATED_OBSTACLE'
        if cost > 0:
            return 'INFLATED_COST'
        return 'FREE'

    def _arc_cost_samples(self, costs: Any, size_x: int, size_y: int,
                          resolution: float, origin: Any,
                          costmap_frame: str) -> list[dict[str, Any]]:
        arc_data = self._arc_snapshot()['samples']
        samples: list[dict[str, Any]] = []
        if resolution <= 0.0:
            return samples
        for topic, arc in arc_data.items():
            prior_point: tuple[float, float] | None = None
            cumulative_distance = 0.0
            for sample_index, point in enumerate(arc['poses']):
                transformed = self._transform_point(
                    point['x'], point['y'], arc['frame_id'], costmap_frame)
                if transformed is None:
                    continue
                world_x, world_y = transformed
                if prior_point is not None:
                    cumulative_distance += math.hypot(
                        world_x - prior_point[0], world_y - prior_point[1])
                prior_point = (world_x, world_y)
                dx = world_x - origin.position.x
                dy = world_y - origin.position.y
                yaw = _quaternion_yaw(origin.orientation)
                local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
                local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
                cell_x = int(local_x / resolution)
                cell_y = int(local_y / resolution)
                if (cell_x < 0 or cell_y < 0 or
                        cell_x >= size_x or cell_y >= size_y):
                    continue
                cost = int(costs[cell_y * size_x + cell_x])
                radius_cells = int(math.ceil(ROBOT_RADIUS_M / resolution))
                footprint_cells: list[dict[str, Any]] = []
                for footprint_y in range(
                        max(0, cell_y - radius_cells),
                        min(size_y, cell_y + radius_cells + 1)):
                    for footprint_x in range(
                            max(0, cell_x - radius_cells),
                            min(size_x, cell_x + radius_cells + 1)):
                        center_local_x = (footprint_x + 0.5) * resolution
                        center_local_y = (footprint_y + 0.5) * resolution
                        center_world_x = (
                            math.cos(yaw) * center_local_x -
                            math.sin(yaw) * center_local_y +
                            origin.position.x)
                        center_world_y = (
                            math.sin(yaw) * center_local_x +
                            math.cos(yaw) * center_local_y +
                            origin.position.y)
                        distance = math.hypot(
                            center_world_x - world_x,
                            center_world_y - world_y)
                        if distance > ROBOT_RADIUS_M:
                            continue
                        footprint_cost = int(
                            costs[footprint_y * size_x + footprint_x])
                        footprint_cells.append({
                            'cell_x': footprint_x,
                            'cell_y': footprint_y,
                            'cost': footprint_cost,
                            'cost_type': self._cost_type(footprint_cost),
                            'distance_from_arc_pose_m': distance,
                        })
                max_footprint = max(
                    footprint_cells, key=lambda item: item['cost'],
                    default={
                        'cell_x': cell_x,
                        'cell_y': cell_y,
                        'cost': cost,
                        'cost_type': self._cost_type(cost),
                        'distance_from_arc_pose_m': 0.0,
                    })
                commands = self._commands()
                command = commands.get('cmd_vel') or commands.get(
                    'ares_cmd_vel') or {}
                linear_speed = abs(float(command.get(
                    'linear_x_mps') or 0.0))
                samples.append({
                    'arc_topic': topic,
                    'arc_sample_index': sample_index,
                    'arc_frame_id': arc['frame_id'],
                    'costmap_frame_id': costmap_frame,
                    'point_x': world_x,
                    'point_y': world_y,
                    'cell_x': cell_x,
                    'cell_y': cell_y,
                    'cost': cost,
                    'cost_type': self._cost_type(cost),
                    'lethal_center_cell': cost in (253, 254),
                    'footprint_radius_m': ROBOT_RADIUS_M,
                    'footprint_max_cost': max_footprint['cost'],
                    'footprint_max_cost_type': max_footprint['cost_type'],
                    'footprint_max_cost_cell_x': max_footprint['cell_x'],
                    'footprint_max_cost_cell_y': max_footprint['cell_y'],
                    'footprint_collision': max_footprint['cost'] in (
                        253, 254, 255),
                    'distance_along_arc_m': cumulative_distance,
                    'projected_duration_sec': (
                        cumulative_distance / linear_speed
                        if linear_speed > 1e-6 else None),
                    'interpretation': (
                        'circular footprint approximation using configured '
                        'robot radius 0.30 m; retained alongside center cost'),
                })
        return samples

    def _scan_snapshot(
            self, collision_sample: dict[str, Any] | None = None) \
            -> dict[str, Any] | None:
        sample = self._sample('scan')
        if sample is None:
            return None
        message = sample['message']
        valid = [
            (float(value), index)
            for index, value in enumerate(message.ranges)
            if math.isfinite(value) and message.range_min <= value <=
            message.range_max
        ]
        minimum, index = min(valid, default=(None, None))
        snapshot: dict[str, Any] = {
            'topic': '/ares/scan',
            'frame_id': message.header.frame_id,
            'sample_count': len(message.ranges),
            'minimum_range_m': minimum,
            'minimum_range_angle_rad': (
                None if index is None else
                message.angle_min + index * message.angle_increment),
            'age_ros_sec': sample['age_ros_sec'],
            'age_wall_sec': sample['age_wall_sec'],
            'nearest_endpoint_to_collision': None,
        }
        if collision_sample is not None:
            collision_frame = collision_sample['costmap_frame_id']
            collision_xy = (
                collision_sample['point_x'], collision_sample['point_y'])
            endpoint_candidates: list[dict[str, Any]] = []
            for range_m, scan_index in valid:
                angle = (
                    message.angle_min +
                    scan_index * message.angle_increment)
                scan_x = range_m * math.cos(angle)
                scan_y = range_m * math.sin(angle)
                endpoint = self._transform_point(
                    scan_x, scan_y, message.header.frame_id,
                    collision_frame)
                if endpoint is None:
                    continue
                endpoint_candidates.append({
                    'scan_index': scan_index,
                    'range_m': range_m,
                    'angle_rad': angle,
                    'x': endpoint[0],
                    'y': endpoint[1],
                    'distance_to_collision_point_m': math.hypot(
                        endpoint[0] - collision_xy[0],
                        endpoint[1] - collision_xy[1]),
                })
            snapshot['nearest_endpoint_to_collision'] = min(
                endpoint_candidates,
                key=lambda item: item['distance_to_collision_point_m'],
                default=None)
        return snapshot

    def _static_map_sample(
            self, collision_sample: dict[str, Any] | None) \
            -> dict[str, Any] | None:
        sample = self._sample('static_map')
        if sample is None or collision_sample is None:
            return None
        message = sample['message']
        point = self._transform_point(
            collision_sample['point_x'], collision_sample['point_y'],
            collision_sample['costmap_frame_id'], message.header.frame_id)
        if point is None or message.info.resolution <= 0.0:
            return None
        origin = message.info.origin
        yaw = _quaternion_yaw(origin.orientation)
        dx = point[0] - origin.position.x
        dy = point[1] - origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        cell_x = int(local_x / message.info.resolution)
        cell_y = int(local_y / message.info.resolution)
        if (cell_x < 0 or cell_y < 0 or
                cell_x >= message.info.width or
                cell_y >= message.info.height):
            return None
        value = int(message.data[cell_y * message.info.width + cell_x])
        return {
            'topic': '/map',
            'frame_id': message.header.frame_id,
            'cell_x': cell_x,
            'cell_y': cell_y,
            'occupancy_value': value,
            'occupied': value >= 65,
            'unknown': value < 0,
            'age_ros_sec': sample['age_ros_sec'],
            'age_wall_sec': sample['age_wall_sec'],
        }

    def _odometry_snapshot(self) -> dict[str, Any] | None:
        pose = self._robot_pose()
        if pose is None:
            return None
        sample = self._sample('odometry')
        message = sample['message']
        twist = message.twist.twist
        return {
            **pose,
            'linear_velocity_x_mps': twist.linear.x,
            'angular_velocity_z_radps': twist.angular.z,
        }

    def _path_snapshots(
            self, collision_sample: dict[str, Any] | None = None) \
            -> dict[str, Any]:
        paths: dict[str, Any] = {}
        robot = self._robot_pose()
        for name, value in self.latest.items():
            message = value['message']
            if not isinstance(message, NavPath):
                continue
            robot_in_path = None
            if robot is not None:
                robot_in_path = self._transform_point(
                    robot['x'], robot['y'], robot['frame_id'],
                    message.header.frame_id)
            collision_in_path = None
            if collision_sample is not None:
                collision_in_path = self._transform_point(
                    collision_sample['point_x'],
                    collision_sample['point_y'],
                    collision_sample['costmap_frame_id'],
                    message.header.frame_id)
            pose_xy = [
                (item.pose.position.x, item.pose.position.y)
                for item in message.poses]
            paths[name] = {
                'frame_id': message.header.frame_id,
                'age_ros_sec': (
                    None if self.latest_ros_time is None or
                    value['ros_stamp_sec'] is None else
                    self.latest_ros_time - value['ros_stamp_sec']),
                'poses': [
                    {
                        'x': item.pose.position.x,
                        'y': item.pose.position.y,
                        'yaw_rad': _quaternion_yaw(item.pose.orientation),
                    }
                    for item in message.poses
                ],
                'robot_lateral_distance_m': (
                    min((math.hypot(
                        robot_in_path[0] - x,
                        robot_in_path[1] - y) for x, y in pose_xy),
                        default=None)
                    if robot_in_path is not None else None),
                'collision_point_lateral_distance_m': (
                    min((math.hypot(
                        collision_in_path[0] - x,
                        collision_in_path[1] - y) for x, y in pose_xy),
                        default=None)
                    if collision_in_path is not None else None),
            }
        return paths

    def _infer_current_waypoint(self) -> dict[str, Any] | None:
        candidates = [
            (name, value['message'])
            for name, value in self.latest.items()
            if isinstance(value['message'], NavPath) and
            ('plan' in name.lower() or 'path' in name.lower())
        ]
        if not candidates:
            return None
        _name, path = max(candidates, key=lambda item: len(item[1].poses))
        if not path.poses:
            return None
        goal = path.poses[-1].pose.position
        index, waypoint = min(
            enumerate(WAYPOINTS),
            key=lambda item: math.hypot(
                item[1][0] - goal.x, item[1][1] - goal.y))
        return {
            'waypoint_index': index,
            'goal_xy_m': [waypoint[0], waypoint[1]],
            'inference': (
                'nearest configured mission waypoint to active plan '
                'endpoint; no ground truth used'),
        }

    def write_report(self, result_path: Path, launch_log_path: Path) -> None:
        mission_result = (
            json.loads(result_path.read_text(encoding='utf-8'))
            if result_path.is_file() else {})
        launch_text = (
            launch_log_path.read_text(encoding='utf-8', errors='replace')
            if launch_log_path.is_file() else '')
        collision_log_count = sum(
            COLLISION_ERROR in line and '[ERROR]' in line
            for line in launch_text.splitlines())
        controller_rate_lines = [
            line for line in launch_text.splitlines()
            if 'Control loop missed its desired rate' in line
        ]
        timestamp_pattern = re.compile(r'\[(\d+\.\d+)\]')

        def timestamped_lines(token: str) -> list[dict[str, Any]]:
            records = []
            for line in launch_text.splitlines():
                if token not in line:
                    continue
                match = timestamp_pattern.search(line)
                records.append({
                    'timestamp_sec': (
                        float(match.group(1)) if match else None),
                    'line': line,
                })
            return records

        abort_records = timestamped_lines(
            '[follow_path] [ActionServer] Aborting handle')
        miss_records = timestamped_lines(
            'Control loop missed its desired rate')
        clear_records = timestamped_lines(
            'Received request to clear entirely')
        replan_records = [
            *timestamped_lines('Computing path goal'),
            *timestamped_lines('Passing new path to controller'),
            *timestamped_lines('begin computing control effort'),
        ]
        costmap_wait_durations = []
        for record in miss_records:
            match = re.search(
                r'Waited\s+([0-9.]+)s for costmap update',
                record['line'])
            if match:
                costmap_wait_durations.append({
                    **record,
                    'wait_duration_sec': float(match.group(1)),
                })
        recovery_events = mission_result.get('recovery_events', [])
        successful_waypoints = {
            item.get('goal', {}).get('waypoint_index')
            for item in mission_result.get(
                'goal_completion_diagnostics', [])
            if item.get('action_status') == 4
        }
        for event in self.events:
            event_time = event['controller_log_timestamp_sec']
            next_event_time = next((
                other['controller_log_timestamp_sec']
                for other in self.events
                if other['event_index'] == event['event_index'] + 1),
                float('inf'))

            def after(records: list[dict[str, Any]], horizon: float) \
                    -> list[dict[str, Any]]:
                return [
                    record for record in records
                    if record['timestamp_sec'] is not None and
                    event_time <= record['timestamp_sec'] <= min(
                        event_time + horizon, next_event_time)
                ]

            aborts = after(abort_records, 1.0)
            clears = after(clear_records, 2.0)
            replans = after(replan_records, 15.0)
            nearest_miss = min(
                (record for record in miss_records
                 if record['timestamp_sec'] is not None),
                key=lambda record: abs(
                    record['timestamp_sec'] - event_time),
                default=None)
            miss_delta = (
                None if nearest_miss is None else
                nearest_miss['timestamp_sec'] - event_time)
            miss_correlated = (
                miss_delta is not None and abs(miss_delta) <= 1.0)
            waypoint_index = (
                (event.get('current_waypoint') or {}).get(
                    'waypoint_index'))
            same_waypoint_succeeded = waypoint_index in successful_waypoints
            event['follow_path_aborted'] = bool(aborts)
            event['follow_path_abort_evidence'] = aborts
            event['recovery_invoked'] = bool(clears)
            event['recovery_evidence'] = clears
            event['subsequent_replan_observed'] = bool(replans)
            event['subsequent_replan_evidence'] = replans
            event['same_waypoint_ultimately_succeeded'] = (
                same_waypoint_succeeded)
            event['recovery_succeeded'] = bool(
                clears and same_waypoint_succeeded)
            event['nav2_recovered'] = event['recovery_succeeded']
            event['mission_eventually_completed'] = mission_result.get(
                'mission_completed')
            event['controller_timing_at_event'].update({
                'controller_rate_miss_count_total': len(controller_rate_lines),
                'controller_rate_miss_log_lines': controller_rate_lines,
                'collision_error_log_count_total': collision_log_count,
                'nearest_controller_rate_miss_delta_sec': miss_delta,
                'correlated_with_controller_rate_miss': miss_correlated,
                'nearest_controller_rate_miss': nearest_miss,
            })
            collision = event.get('first_colliding_arc_sample') or {}
            scan = event.get('scan') or {}
            event['calculated_checks'] = {
                'robot_to_collision_point_distance_m':
                    collision.get('distance_along_arc_m'),
                'robot_to_nearest_lidar_obstacle_distance_m':
                    scan.get('minimum_range_m'),
                'collision_arc_projected_duration_sec':
                    collision.get('projected_duration_sec'),
                'commanded_linear_velocity_mps': (
                    (event.get('commands', {}).get('cmd_vel') or
                     event.get('commands', {}).get('ares_cmd_vel') or
                     {}).get('linear_x_mps')),
                'commanded_angular_velocity_radps': (
                    (event.get('commands', {}).get('cmd_vel') or
                     event.get('commands', {}).get('ares_cmd_vel') or
                     {}).get('angular_z_radps')),
                'local_costmap_age_sec': (
                    (event.get('local_costmap') or {}).get(
                        'update_age_ros_sec')),
                'scan_age_sec': scan.get('age_ros_sec'),
                'odometry_age_sec': (
                    (event.get('odometry') or {}).get('age_ros_sec')),
                'lateral_distance_from_planned_path_m': min((
                    path['robot_lateral_distance_m']
                    for path in event.get('paths', {}).values()
                    if path.get('robot_lateral_distance_m') is not None
                ), default=None),
                'robot_radius_m': ROBOT_RADIUS_M,
                'lidar_clearance_beyond_robot_radius_m': (
                    None if scan.get('minimum_range_m') is None else
                    scan['minimum_range_m'] - ROBOT_RADIUS_M),
                'collision_cell_cost': collision.get(
                    'footprint_max_cost'),
                'collision_cell_cost_type': collision.get(
                    'footprint_max_cost_type'),
                'collision_mechanism': (
                    'CENTER_CELL' if collision.get(
                        'lethal_center_cell') else
                    'FOOTPRINT' if collision else None),
            }
        payload = {
            'schema_version': 1,
            'run_name': self.run_name,
            'started_wall_utc': datetime.fromtimestamp(
                time.time() - (time.monotonic() - self.started_wall),
                timezone.utc).isoformat(),
            'captured_collision_events': len(self.events),
            'controller_collision_error_log_count': collision_log_count,
            'controller_rate_miss_count': len(controller_rate_lines),
            'controller_rate_miss_log_lines': controller_rate_lines,
            'controller_rate_miss_records': miss_records,
            'costmap_wait_durations': costmap_wait_durations,
            'follow_path_abort_count': len(abort_records),
            'follow_path_abort_records': abort_records,
            'planner_failure_count': mission_result.get(
                'planner_failure_count'),
            'controller_failure_count': mission_result.get(
                'controller_failure_count'),
            'nav2_abort_count': mission_result.get('nav2_abort_count'),
            'max_controller_command_gap_sec': self.max_command_gap_sec,
            'max_local_costmap_interarrival_sec':
                self.max_costmap_interarrival_sec,
            'diagnostic_topics_discovered': self.topic_inventory,
            'diagnostic_topic_discovery_skips': (
                self.topic_discovery_skips),
            'subscription_failures': self.subscription_failures,
            'disabled_topics': sorted(set(self.disabled_topics)),
            'message_conversion_failures':
                self.message_conversion_failures,
            'collision_arc_topic_discovered': any(
                'arc' in name.lower()
                for name in self.topic_inventory),
            'rpp_visualization_topics': [
                name for name in self.topic_inventory
                if 'lookahead' in name.lower() or 'arc' in name.lower()
            ],
            'collision_events': self.events,
            'events': self.events,
            'mission_completed': mission_result.get('mission_completed'),
            'mission_waypoints_reached': mission_result.get(
                'waypoints_reached'),
            'mission_waypoints_total': mission_result.get(
                'waypoints_total'),
            'final_goal_error_m': mission_result.get('final_goal_error_m'),
            'nav_recovery_count': mission_result.get('nav_recovery_count'),
            'mission_recovery_state_events': recovery_events,
            'costmap_update_timestamps': self.costmap_update_timestamps,
            'sim_wall_timeline': self.sim_wall_timeline,
            'stream_timing': self.stream_timing,
            'relevant_log_events': self.relevant_log_events,
            'controller_miss_correlation': {
                'collision_events_correlated_with_controller_misses': any(
                    event['controller_timing_at_event'].get(
                        'correlated_with_controller_rate_miss')
                    for event in self.events),
                'definition': (
                    'absolute controller warning timestamp delta <= 1.0 s'),
            },
            'rtf_metrics': {
                **mission_result.get('runtime_quality', {}),
                'mission_completion_time_sec': mission_result.get(
                    'mission_completion_time_sec'),
                'mission_wall_time_sec': mission_result.get(
                    'mission_wall_time_sec'),
            },
        }
        self.output_path.write_text(
            json.dumps(payload, indent=2, allow_nan=False) + '\n',
            encoding='utf-8')


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run-name', required=True)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--launch-log', type=Path, required=True)
    parsed = parser.parse_args(args)
    rclpy.init()
    node = Week6CollisionDiagnostics(parsed.output, parsed.run_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_report(parsed.result, parsed.launch_log)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
