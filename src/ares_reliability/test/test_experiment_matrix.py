"""Tests for the Week 4 machine-readable experiment matrix."""

import math
from pathlib import Path

from ares_reliability.experiment_matrix import (
    bounded_motion_command,
    CALIBRATION_PHASE_SEC,
    CALIBRATION_REGIMES,
    calibration_sweep_motion,
    injector_profiles,
    load_fault_matrix,
)


MATRIX = Path(__file__).parents[1] / 'config' / 'week4_fault_matrix.yaml'


def test_week4_matrix_is_complete_and_seeded() -> None:
    matrix = load_fault_matrix(MATRIX)
    assert len(matrix['scenarios']) >= 30
    assert all('seed' in scenario
               for scenario in matrix['scenarios'].values())


def test_combined_profile_activates_only_declared_sensors() -> None:
    matrix = load_fault_matrix(MATRIX)
    profiles = injector_profiles(
        matrix['scenarios']['gnss_plus_wheel_fault'])
    assert profiles['gnss']['enabled']
    assert profiles['wheel']['enabled']
    assert not profiles['imu']['enabled']


def test_healthy_profile_keeps_all_injectors_passthrough() -> None:
    matrix = load_fault_matrix(MATRIX)
    profiles = injector_profiles(matrix['scenarios']['healthy_figure8'])
    assert all(profile == {'enabled': False, 'mode': 'none'}
               for profile in profiles.values())


def test_calibration_sweep_stays_clear_of_world_obstacle() -> None:
    """Keep the open-loop sweep inside the clear area around the spawn."""
    step = 0.01
    x = 0.0
    y = 0.0
    yaw = 0.0
    maximum_radius = 0.0
    duration = CALIBRATION_PHASE_SEC * len(CALIBRATION_REGIMES)
    for index in range(int(duration / step)):
        linear, angular, _ = calibration_sweep_motion(index * step)
        x += linear * math.cos(yaw) * step
        y += linear * math.sin(yaw) * step
        yaw += angular * step
        maximum_radius = max(maximum_radius, math.hypot(x, y))
    assert maximum_radius < 2.0
    assert math.hypot(x, y) < 0.02
    assert abs(yaw) < 0.02


def test_calibration_sweep_covers_every_regime() -> None:
    observed = {
        calibration_sweep_motion(
            (index + 0.5) * CALIBRATION_PHASE_SEC)[2]
        for index in range(len(CALIBRATION_REGIMES))
    }
    assert observed == set(CALIBRATION_REGIMES)


def test_bounded_motion_retraces_fault_trajectory() -> None:
    step = 0.01
    x = 0.0
    y = 0.0
    yaw = 0.0
    maximum_radius = 0.0
    for index in range(int(45.0 / step)):
        linear, angular = bounded_motion_command(
            0.10, 0.15, index * step, 7.5)
        x += linear * math.cos(yaw) * step
        y += linear * math.sin(yaw) * step
        yaw += angular * step
        maximum_radius = max(maximum_radius, math.hypot(x, y))
    assert maximum_radius < 1.0
    assert math.hypot(x, y) < 0.01
    assert abs(yaw) < 0.01


def test_bounded_motion_rejects_invalid_period() -> None:
    try:
        bounded_motion_command(0.1, 0.15, 0.0, 0.0)
    except ValueError as error:
        assert 'reverse_period_sec' in str(error)
    else:
        raise AssertionError('zero reverse period should be rejected')
