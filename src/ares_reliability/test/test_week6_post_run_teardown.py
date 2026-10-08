"""Tests for the Week 6 post-run teardown barrier."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys


WORKSPACE = Path(__file__).resolve().parents[3]
ISOLATION_PATH = WORKSPACE / 'scripts' / 'week6_runtime_isolation.py'
SPEC = importlib.util.spec_from_file_location(
    'week6_runtime_isolation_teardown_test', ISOLATION_PATH)
assert SPEC is not None and SPEC.loader is not None
ISOLATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ISOLATION)
VALIDATION_PATH = WORKSPACE / 'scripts' / 'validate_week6_handoffs.py'
VALIDATION_SPEC = importlib.util.spec_from_file_location(
    'validate_week6_handoffs_test', VALIDATION_PATH)
assert VALIDATION_SPEC is not None and VALIDATION_SPEC.loader is not None
VALIDATION = importlib.util.module_from_spec(VALIDATION_SPEC)
VALIDATION_SPEC.loader.exec_module(VALIDATION)


def _snapshot(*, publishers=(0, 0), nodes=(), world=(), owned=()):
    counts = {
        '/clock': publishers[0],
        '/ares/week6/raw_clock': publishers[1],
    }
    return {
        'owned_processes': {
            pid: {'pid': pid, 'command': 'owned process'}
            for pid in owned
        },
        'ownership_reasons': {},
        'critical_ros_nodes': list(nodes),
        'ros_nodes': list(nodes),
        'ros_node_probe': {'ok': True},
        'gazebo_world_probe': {'ok': True},
        'gazebo_ares_world_services': list(world),
        'duplicate_world_visible': bool(world),
        'topic_publishers': {
            topic: {
                'known': True,
                'publisher_count': count,
            }
            for topic, count in counts.items()
        },
    }


def test_post_run_teardown_requires_two_clean_observations(
        tmp_path, monkeypatch, capsys) -> None:
    observed = []

    def snapshot(**_kwargs):
        item = _snapshot()
        observed.append(item)
        return item

    monkeypatch.setattr(ISOLATION, '_snapshot', snapshot)
    report_path = tmp_path / 'post_run_teardown.json'

    report = ISOLATION.post_run_teardown(
        report_path=report_path,
        run_name='healthy_baseline_s2530',
        result_path=None,
        launch=None,
        tracked_processes=[],
        release_timeout_sec=1.0,
    )

    assert report['teardown_ready'] is True
    assert len(observed) == 2
    assert report_path.is_file()
    assert json.loads(report_path.read_text())['teardown_ready'] is True
    assert 'POST_RUN_TEARDOWN_READY: YES' in capsys.readouterr().out


def test_post_run_teardown_fails_if_raw_clock_publisher_remains(
        tmp_path, monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        ISOLATION, '_snapshot',
        lambda **_kwargs: _snapshot(publishers=(0, 1)))
    report_path = tmp_path / 'post_run_teardown.json'

    report = ISOLATION.post_run_teardown(
        report_path=report_path,
        run_name='healthy_unprotected_s2530',
        result_path=None,
        launch=None,
        tracked_processes=[],
        release_timeout_sec=0.01,
    )

    assert report['teardown_ready'] is False
    assert '/ares/week6/raw_clock publisher remains' in report['errors']
    assert report['raw_clock_publisher_counts_over_time']
    assert 'POST_RUN_TEARDOWN_READY: NO' in capsys.readouterr().out


def test_post_run_teardown_labels_unknown_publishers_as_uncertain(
        tmp_path, monkeypatch, capsys) -> None:
    unknown = _snapshot()
    for probe in unknown['topic_publishers'].values():
        probe.update({'known': False, 'publisher_count': None})
    unknown['gazebo_world_probe'] = {'ok': False}
    unknown['ros_node_probe'] = {'ok': False}
    monkeypatch.setattr(ISOLATION, '_snapshot', lambda **_kwargs: unknown)

    report = ISOLATION.post_run_teardown(
        report_path=tmp_path / 'post_run_teardown.json',
        run_name='probe_uncertain',
        result_path=None,
        launch=None,
        tracked_processes=[],
        release_timeout_sec=0.01,
    )

    assert report['teardown_ready'] is False
    assert report['classification'] == 'PROBE_UNCERTAIN'
    assert 'publisher count for /clock is unknown' in report['errors']
    assert '/clock publisher remains' not in report['errors']
    assert 'POST_RUN_TEARDOWN_READY: NO' in capsys.readouterr().out


def test_campaign_checks_teardown_before_advancing_to_next_run() -> None:
    source = (WORKSPACE / 'scripts' /
              'run_week6_campaign.py').read_text(encoding='utf-8')
    assert "expected.parent / 'post_run_teardown.json'" in source
    assert "if not teardown.get('teardown_ready'):" in source
    assert "if not isolation.get('world_exclusivity_ready'):" in source
    assert source.index("if not teardown.get('teardown_ready'):") < source.index(
        "f'END {label}")


def test_campaign_fail_closed_preserves_current_case_and_never_launches_next(
        tmp_path, monkeypatch) -> None:
    campaign_path = WORKSPACE / 'scripts' / 'run_week6_campaign.py'
    spec = importlib.util.spec_from_file_location(
        'week6_campaign_teardown_test', campaign_path)
    assert spec is not None and spec.loader is not None
    campaign = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(campaign)

    calls = []

    def fake_run(command, **_kwargs):
        calls.append(command)
        mode = command[command.index('--navigation-mode') + 1]
        scenario = command[command.index('--scenario') + 1]
        seed = int(command[command.index('--seed') + 1])
        results_dir = Path(command[command.index('--results-dir') + 1])
        result = campaign.result_path(results_dir, mode, scenario, seed)
        result.parent.mkdir(parents=True, exist_ok=True)
        result.write_text(json.dumps({
            'mission_completed': False,
            'waypoints_reached': 0,
            'waypoints_total': 5,
        }))
        (result.parent / 'post_run_teardown.json').write_text(
            json.dumps({'teardown_ready': False}))
        (result.parent / 'runtime_isolation.json').write_text(
            json.dumps({'world_exclusivity_ready': True}))
        return subprocess.CompletedProcess(command, 1)

    monkeypatch.setattr(campaign.subprocess, 'run', fake_run)
    monkeypatch.setattr(
        sys, 'argv',
        ['run_week6_campaign.py', '--seeds', '2530',
         '--results-dir', str(tmp_path)])

    try:
        campaign.main()
    except RuntimeError as error:
        assert 'POST_RUN_TEARDOWN_READY: NO' in str(error)
    else:
        raise AssertionError('campaign must stop after teardown failure')

    case_n = campaign.result_path(tmp_path, 'baseline', 'healthy', 2530)
    case_n_plus_1 = campaign.result_path(
        tmp_path, 'unprotected', 'healthy', 2530)
    assert len(calls) == 1
    assert case_n.is_file()
    assert (case_n.parent / 'post_run_teardown.json').is_file()
    assert not case_n_plus_1.parent.exists()
    assert not case_n_plus_1.exists()


def test_world_exclusivity_waits_for_publisher_discovery_to_clear(
        tmp_path, monkeypatch, capsys) -> None:
    snapshots = [
        _snapshot(publishers=(0, 0)),
        _snapshot(publishers=(0, 1)),
        _snapshot(publishers=(0, 0)),
        _snapshot(publishers=(0, 0)),
    ]
    observed = []

    def snapshot(**_kwargs):
        item = snapshots[len(observed)]
        observed.append(item)
        return item

    monkeypatch.setattr(ISOLATION, '_snapshot', snapshot)
    report = ISOLATION.establish_world_exclusivity(
        tmp_path / 'runtime_isolation.json',
        cleanup_timeout_sec=1.0,
    )

    assert report['world_exclusivity_ready'] is True
    assert len(observed) == 4
    assert [
        item['raw_clock_publisher_count']
        for item in report['cleanup_observations']
    ] == [1, 0, 0]
    assert 'WORLD_EXCLUSIVITY_READY: YES' in capsys.readouterr().out


def test_world_exclusivity_fails_closed_if_publisher_never_clears(
        tmp_path, monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        ISOLATION, '_snapshot',
        lambda **_kwargs: _snapshot(publishers=(0, 1)))

    try:
        ISOLATION.establish_world_exclusivity(
            tmp_path / 'runtime_isolation.json',
            cleanup_timeout_sec=0.01,
        )
    except RuntimeError as error:
        assert 'raw-clock publisher remains' in str(error)
    else:
        raise AssertionError('isolation must fail while publisher remains')

    assert 'WORLD_EXCLUSIVITY_READY: NO' in capsys.readouterr().out


def test_transient_dds_publishers_converge_after_two_clean_snapshots() -> None:
    case = {
        'post_run_teardown_ready': True,
        'first_clean_snapshot_utc': '2026-10-07T02:00:20Z',
        'second_clean_snapshot_utc': '2026-10-07T02:00:26Z',
        'world_present_after_teardown': False,
        'remaining_critical_nodes': [],
        'clock_publisher_samples': [
            {'publisher_count': 0},
            {'publisher_count': 0},
        ],
        'raw_clock_publisher_samples': [
            {'publisher_count': 1},
            {'publisher_count': 1},
            {'publisher_count': 0},
            {'publisher_count': 0},
        ],
    }

    assert VALIDATION._runtime_clean(case) is True
    assert VALIDATION._publisher_series_converged(
        case, 'raw_clock_publisher_samples') is True


def test_run_tracker_records_process_identity_without_group_wide_cleanup() \
        -> None:
    source = ISOLATION_PATH.read_text(encoding='utf-8')
    assert 'class RunProcessTracker' in source
    assert "'start_time_ticks': stat_fields[19]" in source
    assert 'os.kill(pid, sig)' in source
    teardown = source.split('def post_run_teardown(', 1)[1].split(
        '\ndef _terminate_owned(', 1)[0]
    assert '_signal_survivors' in teardown
    assert '_terminate_owned(' not in teardown
