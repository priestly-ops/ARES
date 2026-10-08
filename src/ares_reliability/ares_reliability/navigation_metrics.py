"""Deterministic, ROS-independent mission metrics for ARES Week 6."""

from __future__ import annotations

import math
import re
from typing import Any, Iterable, Sequence


Point = tuple[float, float]
TimedCommand = tuple[float, float, float]
TimeWindow = tuple[float, float]


class ClockRuntimeMetrics:
    """Accumulate diagnostic clock-rate and one-second-window RTF evidence."""

    def __init__(self, *, rtf_window_wall_sec: float = 1.0,
                 severe_rtf_threshold: float = 0.50) -> None:
        if (not math.isfinite(rtf_window_wall_sec) or
                rtf_window_wall_sec <= 0.0):
            raise ValueError('RTF window must be finite and positive')
        if (not math.isfinite(severe_rtf_threshold) or
                severe_rtf_threshold < 0.0):
            raise ValueError(
                'severe RTF threshold must be finite and nonnegative')
        self.rtf_window_wall_sec = rtf_window_wall_sec
        self.severe_rtf_threshold = severe_rtf_threshold
        self.sample_count = 0
        self.first_wall_sec: float | None = None
        self.last_wall_sec: float | None = None
        self.last_sim_sec: float | None = None
        self.minimum_period_sec: float | None = None
        self.maximum_period_sec: float | None = None
        self.window_wall_sec: float | None = None
        self.window_sim_sec: float | None = None
        self.rtf_samples: list[float] = []
        self.backward_jump_count = 0

    def observe(self, sim_sec: float, wall_sec: float) -> None:
        """Observe one genuine delivered Clock sample."""
        sim_sec = float(sim_sec)
        wall_sec = float(wall_sec)
        if not (math.isfinite(sim_sec) and math.isfinite(wall_sec)):
            raise ValueError('clock observations must be finite')
        if self.last_wall_sec is not None:
            period = wall_sec - self.last_wall_sec
            if period <= 0.0:
                raise ValueError('wall clock must advance monotonically')
            self.minimum_period_sec = (
                period if self.minimum_period_sec is None else
                min(self.minimum_period_sec, period))
            self.maximum_period_sec = (
                period if self.maximum_period_sec is None else
                max(self.maximum_period_sec, period))
        if self.first_wall_sec is None:
            self.first_wall_sec = wall_sec
        self.sample_count += 1

        if self.last_sim_sec is not None and sim_sec < self.last_sim_sec:
            self.backward_jump_count += 1
            self.window_wall_sec = wall_sec
            self.window_sim_sec = sim_sec
        elif self.window_wall_sec is None:
            self.window_wall_sec = wall_sec
            self.window_sim_sec = sim_sec
        else:
            elapsed_wall = wall_sec - self.window_wall_sec
            if elapsed_wall >= self.rtf_window_wall_sec:
                assert self.window_sim_sec is not None
                elapsed_sim = sim_sec - self.window_sim_sec
                self.rtf_samples.append(max(0.0, elapsed_sim / elapsed_wall))
                self.window_wall_sec = wall_sec
                self.window_sim_sec = sim_sec

        self.last_wall_sec = wall_sec
        self.last_sim_sec = sim_sec

    def summary(self) -> dict[str, Any]:
        """Return non-acceptance runtime-quality fields."""
        span = (
            None if self.first_wall_sec is None or self.last_wall_sec is None
            else self.last_wall_sec - self.first_wall_sec)
        mean_hz = (
            (self.sample_count - 1) / span
            if span is not None and span > 0.0 and self.sample_count > 1
            else None)
        return {
            'clock_sample_count': self.sample_count,
            'clock_mean_hz': mean_hz,
            'clock_min_period_sec': self.minimum_period_sec,
            'clock_max_period_sec': self.maximum_period_sec,
            'clock_backward_jump_count': self.backward_jump_count,
            'rtf_window_wall_sec': self.rtf_window_wall_sec,
            'rtf_sample_count': len(self.rtf_samples),
            'rtf_median': percentile(self.rtf_samples, 0.50),
            'rtf_p10': percentile(self.rtf_samples, 0.10),
            'rtf_p90': percentile(self.rtf_samples, 0.90),
            'severe_rtf_threshold': self.severe_rtf_threshold,
            'severe_rtf_collapse_count': sum(
                value < self.severe_rtf_threshold
                for value in self.rtf_samples),
        }


def _finite_point(value: Sequence[float]) -> Point:
    if len(value) < 2:
        raise ValueError('a path point requires x and y')
    point = (float(value[0]), float(value[1]))
    if not all(math.isfinite(component) for component in point):
        raise ValueError('path points must be finite')
    return point


def path_length(points: Sequence[Sequence[float]]) -> float:
    """Return planar polyline length; an empty or singleton path is zero."""
    parsed = [_finite_point(point) for point in points]
    return sum(math.dist(first, second)
               for first, second in zip(parsed, parsed[1:]))


def point_to_segment_distance(point: Sequence[float],
                              start: Sequence[float],
                              end: Sequence[float]) -> float:
    """Return the minimum planar distance from a point to one segment."""
    px, py = _finite_point(point)
    ax, ay = _finite_point(start)
    bx, by = _finite_point(end)
    dx = bx - ax
    dy = by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(px - ax, py - ay)
    fraction = max(0.0, min(
        1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    return math.hypot(px - (ax + fraction * dx),
                      py - (ay + fraction * dy))


def cross_track_errors(actual_path: Sequence[Sequence[float]],
                       planned_path: Sequence[Sequence[float]]) -> list[float]:
    """Measure each actual point against the nearest planned segment."""
    actual = [_finite_point(point) for point in actual_path]
    planned = [_finite_point(point) for point in planned_path]
    if not actual:
        return []
    if not planned:
        raise ValueError('planned path is unavailable')
    if len(planned) == 1:
        return [math.dist(point, planned[0]) for point in actual]
    segments = list(zip(planned, planned[1:]))
    return [min(point_to_segment_distance(point, start, end)
                for start, end in segments) for point in actual]


def percentile(values: Iterable[float], quantile: float) -> float | None:
    """Return a linearly interpolated quantile in [0, 1]."""
    if not 0.0 <= quantile <= 1.0:
        raise ValueError('quantile must be in [0, 1]')
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if not all(math.isfinite(value) for value in ordered):
        raise ValueError('percentile values must be finite')
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def summarize_errors(values: Sequence[float]) -> dict[str, float | None]:
    """Return the common Week 6 distribution fields."""
    parsed = [float(value) for value in values]
    if not parsed:
        return {'mean': None, 'median': None, 'p95': None, 'max': None}
    if not all(math.isfinite(value) and value >= 0.0 for value in parsed):
        raise ValueError('errors must be finite and non-negative')
    return {
        'mean': sum(parsed) / len(parsed),
        'median': percentile(parsed, 0.5),
        'p95': percentile(parsed, 0.95),
        'max': max(parsed),
    }


def controller_command_gaps(
    samples: Sequence[TimedCommand],
    active_windows: Sequence[TimeWindow],
    gap_threshold_sec: float,
) -> dict[str, float | int]:
    """
    Count missing command intervals only while a goal is active.

    Samples are `(sim_time, linear_x, angular_z)`. Command values are retained
    in the schema so callers cannot accidentally pass bare receipt times.
    Legitimate time outside active goal windows, including waypoint dwell, is
    excluded.
    """
    if not math.isfinite(gap_threshold_sec) or gap_threshold_sec <= 0.0:
        raise ValueError('gap threshold must be finite and positive')
    windows = sorted((float(start), float(end))
                     for start, end in active_windows)
    for start, end in windows:
        if not (math.isfinite(start) and math.isfinite(end) and end >= start):
            raise ValueError('active windows must be finite and ordered')
    times = sorted(float(sample[0]) for sample in samples)
    if not all(math.isfinite(value) for value in times):
        raise ValueError('command timestamps must be finite')
    gaps: list[float] = []
    for window_start, window_end in windows:
        within = [value for value in times
                  if window_start <= value <= window_end]
        boundaries = [window_start, *within, window_end]
        gaps.extend(second - first
                    for first, second in zip(boundaries, boundaries[1:])
                    if second - first > gap_threshold_sec)
    return {
        'controller_command_gap_count': len(gaps),
        'max_controller_command_gap_sec': max(gaps, default=0.0),
    }


def count_controller_rate_misses(log_text: str) -> int:
    """Count Nav2 missed-rate warnings without changing acceptance."""
    pattern = re.compile(r'\bcontrol(?:ler)? loop missed .*\brate\b', re.I)
    return sum(bool(pattern.search(line)) for line in log_text.splitlines())


def waypoint_summary(waypoints: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Summarize sequential NavigateToPose results."""
    reached = sum(bool(item.get('reached', item.get('success', False)))
                  for item in waypoints)
    total = len(waypoints)
    return {
        'waypoints_total': total,
        'waypoints_reached': reached,
        'waypoint_success_fraction': reached / total if total else 0.0,
        'final_goal_error_m': (
            waypoints[-1].get('final_distance_m',
                              waypoints[-1].get('error_m'))
            if waypoints else None),
    }


def classify_mission(
    waypoints: Sequence[dict[str, Any]],
    *,
    stuck: bool = False,
    corridor_exceeded: bool = False,
    catastrophic_localization: bool = False,
    controller_failed: bool = False,
    tf_failure_count: int = 0,
    estimator_restart_count: int = 0,
    recovery_loop_failure: bool = False,
    collision_count: int = 0,
) -> dict[str, Any]:
    """Classify mission completion using action and system health evidence."""
    reasons: list[str] = []
    if not waypoints or not all(bool(item.get(
            'reached', item.get('success', False))) for item in waypoints):
        reasons.append('incomplete_waypoints')
    if any(int(item.get('action_status', 0)) == 6 for item in waypoints):
        reasons.append('nav2_goal_aborted')
    flags = {
        'robot_stuck': stuck,
        'corridor_exceeded': corridor_exceeded,
        'catastrophic_localization': catastrophic_localization,
        'controller_failed': controller_failed,
        'tf_failure': tf_failure_count > 0,
        'estimator_restart': estimator_restart_count > 0,
        'recovery_loop_failure': recovery_loop_failure,
        'collision': collision_count > 0,
    }
    reasons.extend(name for name, active in flags.items() if active)
    return {
        'mission_completed': not reasons,
        'mission_failure_reasons': reasons,
    }


def count_fault_cycles(active_samples: Sequence[bool]) -> int:
    """Count inactive-to-active fault transitions."""
    count = 0
    previous = False
    for active in active_samples:
        current = bool(active)
        if current and not previous:
            count += 1
        previous = current
    return count


def validate_ab_match(protected: dict[str, Any],
                      unprotected: dict[str, Any]) -> None:
    """Reject an A/B pair unless all controlled identifiers match."""
    expected_modes = {
        str(protected.get('navigation_mode')),
        str(unprotected.get('navigation_mode')),
    }
    if expected_modes != {'protected', 'unprotected'}:
        raise ValueError('A/B pair must contain protected and unprotected')
    for field in ('scenario', 'seed', 'mission_id', 'fault_profile'):
        if protected.get(field) != unprotected.get(field):
            raise ValueError(f'A/B mismatch in {field}')
