#!/usr/bin/env python3
"""Run the frozen Week 6 matched-seed navigation campaign sequentially."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
RUNNER = WORKSPACE / 'scripts' / 'run_week6_navigation.py'
RUN_MATRIX = (
    ('baseline', 'healthy'),
    ('unprotected', 'healthy'),
    ('protected', 'healthy'),
    ('unprotected', 'gnss_step_5m'),
    ('protected', 'gnss_step_5m'),
)


def result_path(results_dir: Path, mode: str, scenario: str,
                seed: int) -> Path:
    return (results_dir / scenario /
            f'{scenario}_{mode}_s{seed}' / 'result.json')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', nargs='+', type=int,
                        default=[2530, 2531, 2532])
    parser.add_argument('--results-dir',
                        default='results/week6/final_campaign')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    if len(set(args.seeds)) != len(args.seeds) or any(
            seed < 0 for seed in args.seeds):
        parser.error('seeds must be unique non-negative integers')

    relative_results = Path(args.results_dir)
    results_dir = WORKSPACE / relative_results
    results_dir.mkdir(parents=True, exist_ok=True)
    campaign_metadata_path = results_dir / 'campaign_metadata.json'
    campaign_metadata = {
        'schema_version': 1,
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'seeds': args.seeds,
        'run_order_per_seed': [
            {'navigation_mode': mode, 'scenario': scenario}
            for mode, scenario in RUN_MATRIX
        ],
        'runner': str(RUNNER.relative_to(WORKSPACE)),
    }
    if campaign_metadata_path.exists():
        existing = json.loads(campaign_metadata_path.read_text(
            encoding='utf-8'))
        if existing.get('seeds') != args.seeds:
            raise RuntimeError(
                'existing campaign metadata has different seeds')
        if not args.resume:
            raise FileExistsError(
                f'{campaign_metadata_path} exists; use --resume')
    else:
        campaign_metadata_path.write_text(
            json.dumps(campaign_metadata, indent=2) + '\n',
            encoding='utf-8')

    missing: list[str] = []
    run_transitions: list[dict[str, object]] = []
    for seed in args.seeds:
        for mode, scenario in RUN_MATRIX:
            expected = result_path(results_dir, mode, scenario, seed)
            label = f'{scenario}/{mode}/seed={seed}'
            if expected.exists():
                if not args.resume:
                    raise FileExistsError(expected)
                print(f'SKIP {label}: {expected}', flush=True)
                continue
            print(f'START {label}', flush=True)
            completed = subprocess.run([
                sys.executable, str(RUNNER),
                '--navigation-mode', mode,
                '--scenario', scenario,
                '--seed', str(seed),
                '--results-dir', str(relative_results),
            ], cwd=WORKSPACE, check=False)
            if not expected.exists():
                missing.append(label)
                print(
                    f'MISSING {label}: runner exit={completed.returncode}',
                    flush=True)
                continue
            result = json.loads(expected.read_text(encoding='utf-8'))
            teardown_path = expected.parent / 'post_run_teardown.json'
            isolation_path = expected.parent / 'runtime_isolation.json'
            if not teardown_path.exists():
                raise RuntimeError(
                    f'{label}: missing post-run teardown report; stopping '
                    'before the next scientific run')
            teardown = json.loads(teardown_path.read_text(encoding='utf-8'))
            if not teardown.get('teardown_ready'):
                raise RuntimeError(
                    f'{label}: POST_RUN_TEARDOWN_READY: NO; stopping '
                    'before the next scientific run')
            if not isolation_path.exists():
                raise RuntimeError(
                    f'{label}: missing pre-run isolation report; stopping '
                    'before the next scientific run')
            isolation = json.loads(isolation_path.read_text(encoding='utf-8'))
            if not isolation.get('world_exclusivity_ready'):
                raise RuntimeError(
                    f'{label}: WORLD_EXCLUSIVITY_READY: NO; stopping '
                    'the campaign before the next scientific run')
            print('POST_RUN_TEARDOWN_READY: YES', flush=True)
            run_transitions.append({
                'label': label,
                'runner_exit_code': completed.returncode,
                'result_path': str(expected.relative_to(WORKSPACE)),
                'runtime_isolation_ready': True,
                'post_run_teardown_ready': True,
                'post_run_teardown_path': str(
                    teardown_path.relative_to(WORKSPACE)),
            })
            print(
                f'END {label}: runner exit={completed.returncode} '
                f'completed={result.get("mission_completed")} '
                f'waypoints={result.get("waypoints_reached")}/'
                f'{result.get("waypoints_total")}',
                flush=True)

    if missing:
        raise SystemExit(
            'campaign missing result evidence: ' + ', '.join(missing))
    campaign_metadata['run_transitions'] = run_transitions
    campaign_metadata['completed_utc'] = datetime.now(
        timezone.utc).isoformat()
    campaign_metadata_path.write_text(
        json.dumps(campaign_metadata, indent=2) + '\n',
        encoding='utf-8')
    print(f'CAMPAIGN_COMPLETE {results_dir}', flush=True)


if __name__ == '__main__':
    main()
