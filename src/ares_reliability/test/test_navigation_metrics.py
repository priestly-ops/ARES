"""Week 6 navigation-metric and mission-classification tests."""

import math

import pytest

from ares_reliability.navigation_metrics import (
    ClockRuntimeMetrics,
    classify_mission,
    count_controller_rate_misses,
    controller_command_gaps,
    count_fault_cycles,
    cross_track_errors,
    path_length,
    validate_ab_match,
    waypoint_summary,
)


def test_path_length_calculation() -> None:
    assert path_length([(0, 0), (3, 4), (6, 4)]) == 8.0
    assert path_length([]) == 0.0


def test_cross_track_error() -> None:
    errors = cross_track_errors(
        [(1, 2), (4, -3)], [(0, 0), (5, 0)])
    assert errors == pytest.approx([2.0, 3.0])


def test_missing_path_data() -> None:
    with pytest.raises(ValueError, match='planned path'):
        cross_track_errors([(1, 1)], [])


def test_waypoint_completion_parsing() -> None:
    result = waypoint_summary([
        {'reached': True, 'final_distance_m': 0.1},
        {'reached': False, 'final_distance_m': 1.2},
    ])
    assert result['waypoints_reached'] == 1
    assert result['waypoint_success_fraction'] == 0.5
    assert result['final_goal_error_m'] == 1.2


def test_incomplete_mission_classification() -> None:
    result = classify_mission([
        {'reached': True, 'action_status': 4},
        {'reached': False, 'action_status': 6},
    ])
    assert not result['mission_completed']
    assert 'incomplete_waypoints' in result['mission_failure_reasons']
    assert 'nav2_goal_aborted' in result['mission_failure_reasons']


def test_system_health_makes_goal_reach_insufficient() -> None:
    result = classify_mission(
        [{'reached': True, 'action_status': 4}], tf_failure_count=1)
    assert not result['mission_completed']
    assert result['mission_failure_reasons'] == ['tf_failure']


def test_controller_gap_calculation_excludes_dwell() -> None:
    result = controller_command_gaps(
        [(0.0, 0.2, 0.0), (0.1, 0.2, 0.0),
         (0.8, 0.0, 0.0), (1.1, 0.2, 0.0), (1.2, 0.2, 0.0)],
        [(0.0, 0.2), (1.0, 1.2)],
        0.25,
    )
    assert result['controller_command_gap_count'] == 0
    assert result['max_controller_command_gap_sec'] == 0.0


def test_controller_gap_is_counted_during_active_goal() -> None:
    result = controller_command_gaps(
        [(0.0, 0.2, 0.0), (0.1, 0.2, 0.0), (0.7, 0.2, 0.0)],
        [(0.0, 0.7)],
        0.25,
    )
    assert result['controller_command_gap_count'] == 1
    assert math.isclose(result['max_controller_command_gap_sec'], 0.6)


def test_ab_matching() -> None:
    common = {
        'scenario': 'gnss_step_5m', 'seed': 2506,
        'mission_id': 'week6_v1', 'fault_profile': 'gnss_step_5m',
    }
    validate_ab_match(
        {**common, 'navigation_mode': 'protected'},
        {**common, 'navigation_mode': 'unprotected'},
    )
    with pytest.raises(ValueError, match='seed'):
        validate_ab_match(
            {**common, 'navigation_mode': 'protected'},
            {**common, 'navigation_mode': 'unprotected', 'seed': 2507},
        )


def test_repeated_fault_cycle_accounting() -> None:
    assert count_fault_cycles(
        [False, True, True, False, True, False, True, False]) == 3


def test_clock_runtime_metrics_rate_periods_and_rtf_distribution() -> None:
    metrics = ClockRuntimeMetrics(
        rtf_window_wall_sec=1.0, severe_rtf_threshold=0.5)
    for index in range(301):
        wall = index * 0.01
        # First two wall-time windows run at RTF 1.0; the third runs at 0.25.
        sim = wall if wall <= 2.0 else 2.0 + (wall - 2.0) * 0.25
        metrics.observe(sim, wall)
    summary = metrics.summary()
    assert summary['clock_mean_hz'] == pytest.approx(100.0)
    assert summary['clock_min_period_sec'] == pytest.approx(0.01)
    assert summary['clock_max_period_sec'] == pytest.approx(0.01)
    assert summary['rtf_sample_count'] == 3
    assert summary['rtf_median'] == pytest.approx(1.0)
    assert summary['rtf_p10'] == pytest.approx(0.4)
    assert summary['rtf_p90'] == pytest.approx(1.0)
    assert summary['severe_rtf_collapse_count'] == 1


def test_clock_runtime_metrics_records_backward_jump() -> None:
    metrics = ClockRuntimeMetrics()
    metrics.observe(10.0, 1.0)
    metrics.observe(9.0, 2.0)
    assert metrics.summary()['clock_backward_jump_count'] == 1


def test_controller_rate_miss_warning_count_is_diagnostic() -> None:
    log = '\n'.join((
        '[WARN] Control loop missed its desired rate of 10.0000Hz',
        '[INFO] controller active',
        '[WARN] Controller loop missed the requested rate',
    ))
    assert count_controller_rate_misses(log) == 2
