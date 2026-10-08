#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash
source install/setup.bash
exec python3 scripts/run_healthy_trials.py "$@"
