"""Regression tests for the immutable Week 6 campaign runner matrix."""

from __future__ import annotations

import importlib.util
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
RUNNER_PATH = WORKSPACE / 'scripts' / 'run_week6_campaign.py'
SPEC = importlib.util.spec_from_file_location(
    'run_week6_campaign', RUNNER_PATH)
assert SPEC is not None and SPEC.loader is not None
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def test_campaign_runner_preserves_required_per_seed_order() -> None:
    assert RUNNER.RUN_MATRIX == (
        ('baseline', 'healthy'),
        ('unprotected', 'healthy'),
        ('protected', 'healthy'),
        ('unprotected', 'gnss_step_5m'),
        ('protected', 'gnss_step_5m'),
    )


def test_campaign_runner_uses_canonical_result_path(tmp_path) -> None:
    assert RUNNER.result_path(
        tmp_path, 'protected', 'gnss_step_5m', 2530) == (
            tmp_path / 'gnss_step_5m' /
            'gnss_step_5m_protected_s2530' / 'result.json')


def test_campaign_runner_skips_existing_results_only_on_resume() -> None:
    source = RUNNER_PATH.read_text(encoding='utf-8')
    assert 'if expected.exists():' in source
    assert 'if not args.resume:' in source
    assert 'SKIP {label}' in source


def test_isolation_failure_is_preserved_as_a_canonical_failed_run() -> None:
    source = (WORKSPACE / 'scripts' /
              'run_week6_navigation.py').read_text(encoding='utf-8')
    assert 'except RuntimeError as error:' in source
    assert "'failure_phase': 'runtime_isolation'" in source
    assert "f'runner_error: world isolation failed: {error}'" in source
    assert source.index('establish_world_exclusivity(') < source.index(
        'subprocess.Popen(')


def test_campaign_does_not_rerun_or_overwrite_existing_canonical_results() \
        -> None:
    source = RUNNER_PATH.read_text(encoding='utf-8')
    assert 'if expected.exists():' in source
    assert 'raise FileExistsError(expected)' in source
