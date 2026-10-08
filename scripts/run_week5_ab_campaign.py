#!/usr/bin/env python3
"""Run sequential matched protected/control Week 5 fusion pairs."""

import argparse
from pathlib import Path
import subprocess
import sys


WORKSPACE = Path(__file__).resolve().parents[1]
RUNNER = WORKSPACE / 'scripts' / 'run_week5_experiment.py'


def main() -> None:
    """Run both fusion modes for each seed without concurrent ROS graphs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=('gnss_step_5m', 'gnss_slow_drift'))
    parser.add_argument('--seeds', nargs='+', type=int, required=True)
    parser.add_argument('--results-dir', required=True)
    parser.add_argument('--baseline-ready-sec', type=float, default=18.0)
    args = parser.parse_args()
    if any(seed < 0 for seed in args.seeds):
        parser.error('seeds must be non-negative')

    for seed in args.seeds:
        for mode in ('unprotected', 'protected'):
            command = [
                sys.executable, str(RUNNER), args.scenario,
                '--fusion-mode', mode,
                '--launch-baseline',
                '--seed', str(seed),
                '--results-dir', args.results_dir,
                '--baseline-ready-sec', str(args.baseline_ready_sec),
            ]
            print(f'Running seed={seed} mode={mode}', flush=True)
            subprocess.run(command, cwd=WORKSPACE, check=True)


if __name__ == '__main__':
    main()
