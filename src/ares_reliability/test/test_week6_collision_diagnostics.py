"""Tests for the standalone Week 6 collision diagnostics monitor."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.serialization import serialize_message

from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path as NavPath
from ares_reliability.week6_collision_diagnostics import (
    DIAGNOSTIC_TOPIC_TYPES,
    Week6CollisionDiagnostics,
)


MONITOR_PATH = (
    Path(__file__).resolve().parents[1] / 'ares_reliability' /
    'week6_collision_diagnostics.py'
)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    was_initialized = rclpy.ok()
    if not was_initialized:
        rclpy.init()
    yield
    if not was_initialized and rclpy.ok():
        rclpy.shutdown()


def _monitor(tmp_path: Path) -> Week6CollisionDiagnostics:
    return Week6CollisionDiagnostics(
        tmp_path / 'collision_events.json', 'unit_test')


def test_node_construction_succeeds(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(Node, 'get_topic_names_and_types', lambda _self: [])
    node = _monitor(tmp_path)
    try:
        assert node.get_name() == 'week6_collision_diagnostics'
    finally:
        node.destroy_node()


def test_subscriptions_are_created_without_property_collision(
        tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(Node, 'get_topic_names_and_types', lambda _self: [])
    node = _monitor(tmp_path)
    try:
        assert len(node._diagnostic_subscriptions) == 10
        assert len(node.subscribed_topics) == len(node._diagnostic_subscriptions)
        assert len(list(node.subscriptions)) == len(
            node._diagnostic_subscriptions)
    finally:
        node.destroy_node()


def test_monitor_spins_briefly_without_publishers(
        tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(Node, 'get_topic_names_and_types', lambda _self: [])
    node = _monitor(tmp_path)
    try:
        rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()


def test_missing_optional_topics_do_not_crash_startup(
        tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(Node, 'get_topic_names_and_types', lambda _self: [])
    node = _monitor(tmp_path)
    try:
        assert node.topic_inventory == {}
        assert node.local_costmap_topic is None
    finally:
        node.destroy_node()


def test_intended_diagnostic_topic_is_accepted(tmp_path, monkeypatch) -> None:
    topic = '/lookahead_collision_arc'
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, list(DIAGNOSTIC_TOPIC_TYPES[topic]))])
    node = _monitor(tmp_path)
    try:
        assert topic in node.subscribed_topics
        assert topic in node.topic_inventory
    finally:
        node.destroy_node()


def test_allowlisted_path_topic_uses_guarded_raw_delivery(
        tmp_path, monkeypatch) -> None:
    topic = '/plan'
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, list(DIAGNOSTIC_TOPIC_TYPES[topic]))])
    node = _monitor(tmp_path)
    try:
        subscription = next(
            sub for sub in node._diagnostic_subscriptions
            if sub.topic_name == topic)
        assert subscription.raw is True
        callback = node._raw_diagnostic_callback(
            topic, NavPath, node._path_callback_for(topic))
        callback(serialize_message(NavPath()))
        assert topic in node.latest
    finally:
        node.destroy_node()


def test_lookahead_collision_arc_uses_path_type(
        tmp_path, monkeypatch) -> None:
    topic = '/lookahead_collision_arc'
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, list(DIAGNOSTIC_TOPIC_TYPES[topic]))])
    node = _monitor(tmp_path)
    try:
        subscription = next(
            sub for sub in node._diagnostic_subscriptions
            if sub.topic_name == topic)
        assert subscription.msg_type is NavPath
        callback = node._raw_diagnostic_callback(
            topic, NavPath, node._path_callback_for(topic))
        callback(serialize_message(NavPath()))
        assert topic in node.latest
    finally:
        node.destroy_node()


def test_costmap_message_type_is_converted_inside_safe_callback(
        tmp_path, monkeypatch) -> None:
    topic = '/local_costmap/costmap_raw'
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, list(DIAGNOSTIC_TOPIC_TYPES[topic]))])
    node = _monitor(tmp_path)
    try:
        callback = node._raw_diagnostic_callback(
            topic, Costmap, node._costmap_callback_for(topic))
        callback(serialize_message(Costmap()))
        assert node.stream_timing['local_costmap']['sample_count'] == 1
        assert node.message_conversion_failures == []
    finally:
        node.destroy_node()


def test_bad_optional_message_conversion_disables_only_that_topic(
        tmp_path, monkeypatch) -> None:
    topic = '/plan'
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, list(DIAGNOSTIC_TOPIC_TYPES[topic]))])
    node = _monitor(tmp_path)
    try:
        callback = node._raw_diagnostic_callback(
            topic, NavPath, node._path_callback_for(topic))
        callback(b'not a serialized ROS message')
        assert topic in node.disabled_topics
        assert any(item['topic'] == topic
                   for item in node.message_conversion_failures)
        assert topic not in node.subscribed_topics
    finally:
        node.destroy_node()


def test_failed_optional_subscription_does_not_terminate_monitor(
        tmp_path, monkeypatch) -> None:
    topic = '/lookahead_collision_arc'
    original_create_subscription = Node.create_subscription

    def fail_selected_subscription(
            self, message_type, selected_topic, callback,
            qos_profile, **kwargs):
        if selected_topic == topic:
            raise RuntimeError('simulated optional subscription failure')
        return original_create_subscription(
            self, message_type, selected_topic, callback, qos_profile,
            **kwargs)

    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, list(DIAGNOSTIC_TOPIC_TYPES[topic]))])
    monkeypatch.setattr(
        Node, 'create_subscription', fail_selected_subscription)
    node = _monitor(tmp_path)
    try:
        rclpy.spin_once(node, timeout_sec=0.05)
        assert topic in node.disabled_topics
        assert node.subscription_failures[0]['topic'] == topic
        assert node.get_name() == 'week6_collision_diagnostics'
    finally:
        node.destroy_node()


@pytest.mark.parametrize('topic', [
    '/local_costmap/obstacle_layer_raw/_buf_cpu',
    '/local_costmap/obstacle_layer_raw/_buf_cpu/_buf_cpu',
])
def test_internal_buffer_topics_are_rejected(
        tmp_path, monkeypatch, topic) -> None:
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, ['nav2_msgs/msg/Costmap'])])
    node = _monitor(tmp_path)
    try:
        assert topic not in node.subscribed_topics
        assert any(item['topic'] == topic and
                   item['reason'] == 'not_allowlisted'
                   for item in node.topic_discovery_skips)
    finally:
        node.destroy_node()


def test_overly_long_topic_is_recorded_without_crashing(
        tmp_path, monkeypatch) -> None:
    topic = '/' + 'unexpected_' * 30
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, ['nav2_msgs/msg/Costmap'])])
    node = _monitor(tmp_path)
    try:
        assert topic not in node.subscribed_topics
        assert any(item['topic'] == topic and
                   item['reason'] == 'invalid_topic_name'
                   for item in node.topic_discovery_skips)
    finally:
        node.destroy_node()


def test_duplicate_topic_is_subscribed_only_once(tmp_path, monkeypatch) -> None:
    topic = '/lookahead_collision_arc'
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, list(DIAGNOSTIC_TOPIC_TYPES[topic]))])
    node = _monitor(tmp_path)
    try:
        initial_count = len(node._diagnostic_subscriptions)
        node._discover_diagnostic_topics(QoSProfile(depth=1))
        assert len(node._diagnostic_subscriptions) == initial_count
        assert topic in node.subscribed_topics
    finally:
        node.destroy_node()


def test_unknown_unrelated_topic_is_ignored_and_recorded(
        tmp_path, monkeypatch) -> None:
    topic = '/unrelated_debug_output'
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, ['std_msgs/msg/String'])])
    node = _monitor(tmp_path)
    try:
        assert topic not in node.subscribed_topics
        assert any(item['topic'] == topic and
                   item['reason'] == 'not_allowlisted'
                   for item in node.topic_discovery_skips)
    finally:
        node.destroy_node()


def test_discovery_does_not_expand_synthetic_descendants(
        tmp_path, monkeypatch) -> None:
    topics = [
        ('/lookahead_collision_arc', ['nav_msgs/msg/Path']),
        ('/lookahead_collision_arc/_buf_cpu', ['nav_msgs/msg/Path']),
        ('/local_costmap/costmap_raw/_buf_cpu/_buf_cpu',
         ['nav2_msgs/msg/Costmap']),
    ]
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types', lambda _self: topics)
    node = _monitor(tmp_path)
    try:
        initial_count = len(node._diagnostic_subscriptions)
        node._discover_diagnostic_topics(QoSProfile(depth=1))
        assert len(node._diagnostic_subscriptions) == initial_count
        assert node.subscribed_topics == {
            '/rosout', '/clock', '/ares/scan', '/odometry/trust_fused',
            '/cmd_vel', '/ares/cmd_vel', '/tf', '/map',
            '/follow_path/_action/status', '/tf_static',
            '/lookahead_collision_arc',
        }
    finally:
        node.destroy_node()


def test_monitor_spins_after_invalid_topic_appears(
        tmp_path, monkeypatch) -> None:
    topic = '/' + 'invalid_' * 35
    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, ['nav_msgs/msg/Path'])])
    node = _monitor(tmp_path)
    try:
        rclpy.spin_once(node, timeout_sec=0.65)
        assert any(item['topic'] == topic and
                   item['reason'] == 'invalid_topic_name'
                   for item in node.topic_discovery_skips)
    finally:
        node.destroy_node()


def test_empty_event_report_writes_expected_schema(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(Node, 'get_topic_names_and_types', lambda _self: [])
    node = _monitor(tmp_path)
    try:
        node.write_report(tmp_path / 'missing-result.json',
                          tmp_path / 'missing-launch.log')
        report = json.loads(
            (tmp_path / 'collision_events.json').read_text(encoding='utf-8'))
        assert report['schema_version'] == 1
        assert report['collision_events'] == []
        assert report['events'] == []
    finally:
        node.destroy_node()


def test_report_writes_after_partial_diagnostic_coverage(
        tmp_path, monkeypatch) -> None:
    topic = '/plan'
    original_create_subscription = Node.create_subscription

    def fail_selected_subscription(
            self, message_type, selected_topic, callback,
            qos_profile, **kwargs):
        if selected_topic == topic:
            raise RuntimeError('simulated optional subscription failure')
        return original_create_subscription(
            self, message_type, selected_topic, callback, qos_profile,
            **kwargs)

    monkeypatch.setattr(
        Node, 'get_topic_names_and_types',
        lambda _self: [(topic, list(DIAGNOSTIC_TOPIC_TYPES[topic]))])
    monkeypatch.setattr(
        Node, 'create_subscription', fail_selected_subscription)
    node = _monitor(tmp_path)
    try:
        node.write_report(tmp_path / 'missing-result.json',
                          tmp_path / 'missing-launch.log')
        report = json.loads(
            (tmp_path / 'collision_events.json').read_text(encoding='utf-8'))
        assert report['subscription_failures'][0]['topic'] == topic
        assert topic in report['disabled_topics']
        assert report['collision_events'] == []
        assert report['events'] == []
    finally:
        node.destroy_node()


def test_collision_event_schema_retains_legacy_events_key(
        tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(Node, 'get_topic_names_and_types', lambda _self: [])
    node = _monitor(tmp_path)
    event = {
        'event_index': 1,
        'controller_log_timestamp_sec': 1.0,
        'controller_timing_at_event': {},
    }
    node.events.append(event)
    try:
        node.write_report(tmp_path / 'missing-result.json',
                          tmp_path / 'missing-launch.log')
        report = json.loads(
            (tmp_path / 'collision_events.json').read_text(encoding='utf-8'))
        assert report['captured_collision_events'] == 1
        assert report['collision_events'] == report['events']
        assert report['events'][0]['event_index'] == 1
    finally:
        node.destroy_node()


def test_repeated_report_write_is_safe(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(Node, 'get_topic_names_and_types', lambda _self: [])
    node = _monitor(tmp_path)
    try:
        result_path = tmp_path / 'missing-result.json'
        launch_log_path = tmp_path / 'missing-launch.log'
        node.write_report(result_path, launch_log_path)
        node.write_report(result_path, launch_log_path)
        report = json.loads(
            (tmp_path / 'collision_events.json').read_text(encoding='utf-8'))
        assert report['captured_collision_events'] == 0
        assert report['collision_events'] == []
    finally:
        node.destroy_node()


def test_shutdown_writes_report_successfully(tmp_path) -> None:
    output_path = tmp_path / 'collision_events.json'
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = '231'
    environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    environment.pop('ROS_LOG_DIR', None)
    process = subprocess.Popen(
        [
            sys.executable, str(MONITOR_PATH),
            '--output', str(output_path),
            '--run-name', 'shutdown_test',
            '--result', str(tmp_path / 'missing-result.json'),
            '--launch-log', str(tmp_path / 'missing-launch.log'),
        ],
        cwd=MONITOR_PATH.parents[3],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        time.sleep(3.0)
        assert process.poll() is None, 'monitor exited before shutdown signal'
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=10)
        assert process.returncode == 0, output
        assert 'Traceback' not in output
        report = json.loads(output_path.read_text(encoding='utf-8'))
        assert report['collision_events'] == []
        assert report['events'] == []
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
