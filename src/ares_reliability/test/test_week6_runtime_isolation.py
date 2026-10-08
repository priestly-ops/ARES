"""Tests for fail-closed Week 6 process and world exclusivity checks."""

from __future__ import annotations

import importlib.util
import subprocess
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
ISOLATION_PATH = WORKSPACE / 'scripts' / 'week6_runtime_isolation.py'
SPEC = importlib.util.spec_from_file_location(
    'week6_runtime_isolation', ISOLATION_PATH)
assert SPEC is not None and SPEC.loader is not None
ISOLATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ISOLATION)


def _clean_probe(command, **_kwargs):
    if command[:3] == ['ros2', 'node', 'list']:
        return subprocess.CompletedProcess(command, 0, '', '')
    if command[:3] == ['ros2', 'topic', 'info']:
        return subprocess.CompletedProcess(
            command, 1, '', f"Unknown topic '{command[3]}'")
    if command[:3] == ['gz', 'service', '-l']:
        return subprocess.CompletedProcess(command, 0, '', '')
    raise AssertionError(f'unexpected probe command {command!r}')


def test_isolation_writes_machine_readable_ready_record(
        tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(ISOLATION.shutil, 'which', lambda _name: '/usr/bin/gz')
    proc_root = tmp_path / 'proc'
    proc_root.mkdir()
    record_path = tmp_path / 'runtime_isolation.json'

    report = ISOLATION.establish_world_exclusivity(
        record_path, run_command=_clean_probe, proc_root=proc_root)

    assert report['world_exclusivity_ready'] is True
    assert report['clock_publisher_count'] == 0
    assert report['raw_clock_publisher_count'] == 0
    assert report['duplicate_world_status'] == 'not_visible'
    assert record_path.is_file()
    assert 'WORLD_EXCLUSIVITY_READY: YES' in report['required_marker']


def test_isolation_fails_closed_without_killing_unrelated_ros_nodes(
        tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(ISOLATION.shutil, 'which', lambda _name: '/usr/bin/gz')
    proc_root = tmp_path / 'proc'
    proc_root.mkdir()
    record_path = tmp_path / 'runtime_isolation.json'

    def unrelated_node_probe(command, **_kwargs):
        if command[:3] == ['ros2', 'node', 'list']:
            return subprocess.CompletedProcess(
                command, 0, '/planner_server\n', '')
        return _clean_probe(command, **_kwargs)

    try:
        ISOLATION.establish_world_exclusivity(
            record_path, run_command=unrelated_node_probe,
            proc_root=proc_root)
    except RuntimeError as error:
        assert 'critical ROS nodes remain' in str(error)
    else:
        raise AssertionError('stale critical ROS node did not fail the gate')

    import json

    report = json.loads(record_path.read_text(encoding='utf-8'))
    assert report['world_exclusivity_ready'] is False
    assert report['detected_stale_ros_nodes'] == ['/planner_server']
    assert report['cleanup_actions'] == []


def test_week6_launch_signature_requires_the_specific_launch_file() -> None:
    assert ISOLATION._is_week6_launch({
        'cmdline': (
            'ros2', 'launch', 'ares_reliability',
            'week6_navigation.launch.py',
        ),
    })
    assert not ISOLATION._is_week6_launch({
        'cmdline': (
            'ros2', 'launch', 'ares_reliability',
            'week5_trust_fusion.launch.py',
        ),
    })


def test_orphaned_gazebo_world_is_identified_from_ares_sdf(
        tmp_path) -> None:
    directory = tmp_path / 'ares_baseline_stale'
    directory.mkdir()
    world_path = directory / 'world.sdf'
    world_path.write_text('<sdf><world name="ares_world"/></sdf>')

    reason = ISOLATION._ares_world_or_bridge_reason({
        'command': f'gz sim {world_path}',
        'cmdline': ('gz', 'sim', str(world_path)),
    })

    assert reason == (
        'Gazebo sim process loading the ARES ares_world SDF')


def test_week6_raw_clock_bridge_is_identified_from_its_config(
        tmp_path) -> None:
    directory = tmp_path / 'ares_baseline_stale'
    directory.mkdir()
    bridge_path = directory / 'bridge.yaml'
    bridge_path.write_text(
        '- ros_topic_name: /ares/week6/raw_clock\n')

    reason = ISOLATION._ares_world_or_bridge_reason({
        'command': f'parameter_bridge {bridge_path}',
        'cmdline': (
            'parameter_bridge', '--config_file', str(bridge_path)),
    })

    assert reason == (
        'ros_gz_bridge using the Week 6 raw-clock bridge config')


def test_topic_absence_is_unknown_when_fastdds_transport_failed() -> None:
    command = ['ros2', 'topic', 'info', '/clock', '--verbose']

    def failed_transport(_command, **_kwargs):
        return subprocess.CompletedProcess(
            command, 1, "Unknown topic '/clock'",
            'Error creating socket: Operation not permitted')

    probe = ISOLATION._topic_publisher_count(
        '/clock', run_command=failed_transport,
        environment={
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        })

    assert probe['publisher_count'] is None
    assert probe['known'] is False
    assert probe['status'] == 'PROBE_UNCERTAIN'
    assert probe['environment'] == {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }


def test_snapshot_classification_separates_clean_contamination_and_uncertain(
        tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(ISOLATION.shutil, 'which', lambda _name: '/usr/bin/gz')
    proc_root = tmp_path / 'proc'
    proc_root.mkdir()

    def clean(command, **_kwargs):
        if command[:3] == ['ros2', 'node', 'list']:
            return subprocess.CompletedProcess(command, 0, '', '')
        if command[:3] == ['ros2', 'topic', 'info']:
            return subprocess.CompletedProcess(
                command, 1, '', f"Unknown topic '{command[3]}'")
        return subprocess.CompletedProcess(command, 0, '', '')

    snapshot = ISOLATION._snapshot(
        run_command=clean, proc_root=proc_root)
    assert ISOLATION._snapshot_classification(snapshot) == 'CLEAN_CONFIRMED'

    def stale_node(command, **kwargs):
        if command[:3] == ['ros2', 'node', 'list']:
            return subprocess.CompletedProcess(
                command, 0, '/planner_server\n', '')
        return clean(command, **kwargs)

    snapshot = ISOLATION._snapshot(
        run_command=stale_node, proc_root=proc_root)
    assert (ISOLATION._snapshot_classification(snapshot) ==
            'CONTAMINATION_CONFIRMED')

    def broken(_command, **_kwargs):
        raise subprocess.TimeoutExpired('probe', timeout=8.0)

    snapshot = ISOLATION._snapshot(
        run_command=broken, proc_root=proc_root)
    assert ISOLATION._snapshot_classification(snapshot) == 'PROBE_UNCERTAIN'
