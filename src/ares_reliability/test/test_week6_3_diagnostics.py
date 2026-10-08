"""Tests for the gated Week 6.3 runtime validation."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
VALIDATOR_PATH = WORKSPACE / 'scripts' / 'run_week6_3_diagnostics.py'
SPEC = importlib.util.spec_from_file_location(
    'week6_3_diagnostics_test', VALIDATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


def _passing_case() -> dict:
    return {
        'pre_run_isolation_passed': True,
        'startup_readiness_passed': True,
        'post_run_teardown_passed': True,
        'planner_action_discovery_failures': [],
        'costmap_service_discovery_failures': [],
        'behavior_action_discovery_failures': [],
        'lifecycle_startup_failures': [],
        'bt_navigator_active': True,
    }


def test_dds_case_requires_every_provider_and_teardown_gate() -> None:
    assert VALIDATOR._dds_case_passed(_passing_case()) is True
    case = _passing_case()
    case['planner_action_discovery_failures'] = [
        '/compute_path_to_pose']
    assert VALIDATOR._dds_case_passed(case) is False
    case = _passing_case()
    case['post_run_teardown_passed'] = False
    assert VALIDATOR._dds_case_passed(case) is False


def test_navigation_diagnostic_is_blocked_without_ten_of_ten_startups(
        tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(VALIDATOR, 'DDS_ROOT', tmp_path / 'dds')
    monkeypatch.setattr(
        VALIDATOR, 'COLLISION_ROOT', tmp_path / 'collision')
    (tmp_path / 'dds').mkdir()
    (tmp_path / 'dds' / 'dds_startup_stress_report.json').write_text(
        '{"pass": false, "attempted_cycles": 10}', encoding='utf-8')

    try:
        VALIDATOR.run_collision_diagnostic()
    except RuntimeError as error:
        assert 'not 10/10' in str(error)
    else:
        raise AssertionError('collision diagnostic must remain gated')

    assert not (tmp_path / 'collision').exists()


def test_completed_case_directory_is_never_rerun(tmp_path) -> None:
    run_directory = tmp_path / 'already-attempted'
    run_directory.mkdir()
    (run_directory / 'endpoint_readiness.json').write_text('{}')
    try:
        VALIDATOR._run_case(
            run_directory, mode='protected', scenario='healthy', seed=2531,
            startup_only=True)
    except FileExistsError:
        pass
    else:
        raise AssertionError('completed diagnostics must not be rerun')


def test_bt_pre_activation_unconfigured_state_is_not_startup_failure(
        tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(VALIDATOR, 'WORKSPACE', tmp_path)
    run_directory = tmp_path / 'run'
    run_directory.mkdir()
    actions = dict.fromkeys((
        *VALIDATOR.REQUIRED_PLANNER_ACTIONS,
        '/follow_path',
        *VALIDATOR.REQUIRED_BEHAVIOR_ACTIONS,
    ), True)
    services = dict.fromkeys(VALIDATOR.REQUIRED_SERVICES, True)
    lifecycle_nodes = (
        'map_server', 'planner_server', 'controller_server',
        'behavior_server')
    lifecycle_states = dict.fromkeys((*lifecycle_nodes, 'amcl'), 'active')
    lifecycle_states['bt_navigator'] = 'unconfigured'
    lifecycle_confirmations = dict.fromkeys(lifecycle_nodes, True)
    lifecycle_confirmations['bt_navigator'] = True
    (run_directory / 'endpoint_readiness.json').write_text(
        json.dumps({
            'passed': True,
            'required_lifecycle_nodes': list(lifecycle_nodes),
            'lifecycle_active_confirmations': lifecycle_confirmations,
            'readiness_barrier': {
                'ready': True,
                'actions': actions,
                'services': services,
                'lifecycle_states': lifecycle_states,
            },
        }), encoding='utf-8')
    (run_directory / 'runtime_isolation.json').write_text(
        '{"world_exclusivity_ready": true}', encoding='utf-8')
    (run_directory / 'post_run_teardown.json').write_text(
        '{"teardown_ready": true}', encoding='utf-8')
    case = {
        'run_directory': 'run',
        'passed': False,
        'pre_run_isolation_passed': True,
        'post_run_teardown_passed': True,
    }

    VALIDATOR._refresh_case_classification(case)

    assert case['startup_readiness_passed'] is True
    assert case['bt_navigator_active'] is True
    assert case['bt_navigator_state_before_activation'] == 'unconfigured'
    assert case['passed'] is True
