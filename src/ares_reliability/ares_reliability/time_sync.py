"""Thin, deterministic timestamp semantics around ROS ``message_filters.Cache``."""

from bisect import bisect_left
from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Callable, Generic, Optional, TypeVar

T = TypeVar('T')


class SyncReason(str, Enum):
    """Outcome of a timestamp lookup."""

    OK = 'OK'
    NO_HISTORY = 'NO_HISTORY'
    ZERO_STAMP = 'ZERO_STAMP'
    CLOCK_MISMATCH = 'CLOCK_MISMATCH'
    TOO_OLD = 'TOO_OLD'
    TOO_FAR = 'TOO_FAR'
    OUT_OF_ORDER = 'OUT_OF_ORDER'
    FROZEN_STAMP = 'FROZEN_STAMP'


@dataclass(frozen=True)
class SyncResult(Generic[T]):
    """Timestamp-alignment value and complete diagnostic context."""

    value: Optional[T]
    reason: SyncReason
    source_timestamp_sec: float
    matched_timestamp_sec: Optional[float]
    signed_time_difference_sec: Optional[float]
    absolute_sync_error_sec: Optional[float]
    interpolation_mode: str
    oldest_timestamp_sec: Optional[float]
    newest_timestamp_sec: Optional[float]
    reset_count: int

    @property
    def accepted(self) -> bool:
        """Return whether a valid comparison value is available."""
        return self.reason == SyncReason.OK and self.value is not None

    @property
    def stale(self) -> bool:
        """Compatibility property for previous monitor implementations."""
        return not self.accepted

    @property
    def sync_error_sec(self) -> Optional[float]:
        """Compatibility alias for absolute synchronization error."""
        return self.absolute_sync_error_sec


class SourceTimestampValidator:
    """Reject non-monotonic source stamps before spatial comparison."""

    def __init__(self, max_equal_stamps: int = 1) -> None:
        if max_equal_stamps < 0:
            raise ValueError('max_equal_stamps must be non-negative')
        self.max_equal_stamps = max_equal_stamps
        self.last_stamp_sec: Optional[float] = None
        self.equal_count = 0

    def reset(self) -> None:
        """Forget timestamp history after a simulation clock reset."""
        self.last_stamp_sec = None
        self.equal_count = 0

    def observe(self, stamp_sec: float) -> SyncReason:
        """Classify a stamp without accepting regressions or long freezes."""
        if not math.isfinite(stamp_sec):
            raise ValueError('timestamp must be finite')
        if stamp_sec == 0.0:
            return SyncReason.ZERO_STAMP
        if self.last_stamp_sec is None:
            self.last_stamp_sec = stamp_sec
            return SyncReason.OK
        if stamp_sec < self.last_stamp_sec - 1.0e-12:
            return SyncReason.OUT_OF_ORDER
        if math.isclose(stamp_sec, self.last_stamp_sec,
                        rel_tol=0.0, abs_tol=1.0e-12):
            self.equal_count += 1
            if self.equal_count > self.max_equal_stamps:
                return SyncReason.FROZEN_STAMP
            return SyncReason.OK
        self.last_stamp_sec = stamp_sec
        self.equal_count = 0
        return SyncReason.OK


def stamp_to_seconds(stamp: object) -> float:
    """Convert ROS builtin time fields to finite seconds."""
    sec = int(getattr(stamp, 'sec'))
    nanosec = int(getattr(stamp, 'nanosec'))
    if nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError('nanosec must be in [0, 1000000000)')
    result = float(sec) + float(nanosec) / 1.0e9
    if not math.isfinite(result):
        raise ValueError('timestamp must be finite')
    return result


def shortest_angle_difference(after: float, before: float) -> float:
    """Return the signed shortest rotation from ``before`` to ``after``."""
    return math.atan2(math.sin(after - before), math.cos(after - before))


def interpolate_scalar(before: float, after: float, fraction: float) -> float:
    """Linearly interpolate a scalar."""
    return before + (after - before) * fraction


def interpolate_pair(before: tuple[float, float], after: tuple[float, float],
                     fraction: float) -> tuple[float, float]:
    """Linearly interpolate a planar pair."""
    return (
        interpolate_scalar(before[0], after[0], fraction),
        interpolate_scalar(before[1], after[1], fraction),
    )


def interpolate_pose2d(before: tuple[float, float, float],
                       after: tuple[float, float, float],
                       fraction: float) -> tuple[float, float, float]:
    """Interpolate position linearly and yaw along the shortest arc."""
    return (
        interpolate_scalar(before[0], after[0], fraction),
        interpolate_scalar(before[1], after[1], fraction),
        before[2] + shortest_angle_difference(after[2], before[2]) * fraction,
    )


class CacheTimestampAligner(Generic[T]):
    """
    Add ARES rejection semantics to ``message_filters.Cache``.

    ROS owns storage and subscription. ARES only selects deterministic nearest
    data (earlier on ties), optionally interpolates values, and explains
    rejection.
    """

    def __init__(self, cache: Any, value_extractor: Callable[[Any], T],
                 max_sync_error_sec: float, history_duration_sec: float,
                 interpolator: Optional[Callable[[T, T, float], T]] = None,
                 clock_mismatch_tolerance_sec: float = 5.0) -> None:
        for name, value, allow_zero in (
            ('max_sync_error_sec', max_sync_error_sec, True),
            ('history_duration_sec', history_duration_sec, False),
            ('clock_mismatch_tolerance_sec', clock_mismatch_tolerance_sec,
             False),
        ):
            if not math.isfinite(value) or value < 0.0 or (
                    not allow_zero and value == 0.0):
                raise ValueError(f'{name} has an invalid value')
        self.cache = cache
        self.value_extractor = value_extractor
        self.max_sync_error_sec = max_sync_error_sec
        self.history_duration_sec = history_duration_sec
        self.interpolator = interpolator
        self.clock_mismatch_tolerance_sec = clock_mismatch_tolerance_sec
        self.last_clock_sec: Optional[float] = None
        self.reset_count = 0

    def clear(self) -> None:
        """Clear the wrapped cache after a clock discontinuity."""
        self.cache.cache_msgs.clear()
        self.cache.cache_times.clear()

    def observe_clock(self, clock_sec: float) -> bool:
        """Reset buffered history when simulation time jumps backwards."""
        if not math.isfinite(clock_sec):
            raise ValueError('clock_sec must be finite')
        jumped = self.last_clock_sec is not None and (
            clock_sec < self.last_clock_sec - 1.0e-9
        )
        if jumped:
            self.clear()
            self.reset_count += 1
        self.last_clock_sec = clock_sec
        return jumped

    def _snapshot(self) -> list[tuple[float, Any]]:
        snapshot = [(float(stamp.nanoseconds) / 1.0e9, message)
                    for stamp, message in zip(self.cache.cache_times,
                                              self.cache.cache_msgs)]
        snapshot.sort(key=lambda item: item[0])
        if not snapshot:
            return snapshot
        cutoff = snapshot[-1][0] - self.history_duration_sec
        return [item for item in snapshot if item[0] >= cutoff]

    def lookup(self, source_timestamp_sec: float, *,
               clock_sec: Optional[float] = None,
               interpolate: bool = False) -> SyncResult[T]:
        """Find or interpolate the cached value at a source timestamp."""
        if not math.isfinite(source_timestamp_sec):
            raise ValueError('source timestamp must be finite')
        if source_timestamp_sec == 0.0:
            return self._rejected(SyncReason.ZERO_STAMP,
                                  source_timestamp_sec)
        if clock_sec is not None:
            self.observe_clock(clock_sec)
            if clock_sec > 0.0 and abs(clock_sec - source_timestamp_sec) > (
                    self.clock_mismatch_tolerance_sec):
                return self._rejected(SyncReason.CLOCK_MISMATCH,
                                      source_timestamp_sec)
        samples = self._snapshot()
        if not samples:
            return self._rejected(SyncReason.NO_HISTORY,
                                  source_timestamp_sec)
        oldest, newest = samples[0][0], samples[-1][0]
        if source_timestamp_sec < oldest - self.max_sync_error_sec:
            return self._rejected(SyncReason.TOO_OLD, source_timestamp_sec,
                                  oldest, newest)
        timestamps = [sample[0] for sample in samples]
        index = bisect_left(timestamps, source_timestamp_sec)
        if index < len(samples) and math.isclose(
                timestamps[index], source_timestamp_sec, abs_tol=1.0e-12):
            return self._accepted(self.value_extractor(samples[index][1]),
                                  source_timestamp_sec, samples[index][0],
                                  'EXACT', 0.0, oldest, newest)
        if (interpolate and self.interpolator is not None and
                0 < index < len(samples)):
            before, after = samples[index - 1], samples[index]
            before_error = source_timestamp_sec - before[0]
            after_error = after[0] - source_timestamp_sec
            maximum_error = max(before_error, after_error)
            if maximum_error <= self.max_sync_error_sec:
                fraction = before_error / (after[0] - before[0])
                value = self.interpolator(
                    self.value_extractor(before[1]),
                    self.value_extractor(after[1]), fraction)
                return self._accepted(value, source_timestamp_sec,
                                      source_timestamp_sec, 'LINEAR',
                                      maximum_error, oldest, newest)
        candidates = []
        if index > 0:
            candidates.append(samples[index - 1])
        if index < len(samples):
            candidates.append(samples[index])
        selected = candidates[0]
        for candidate in candidates[1:]:
            candidate_error = abs(candidate[0] - source_timestamp_sec)
            selected_error = abs(selected[0] - source_timestamp_sec)
            if candidate_error < selected_error and not math.isclose(
                    candidate_error, selected_error,
                    rel_tol=0.0, abs_tol=1.0e-12):
                selected = candidate
            elif math.isclose(candidate_error, selected_error,
                              rel_tol=0.0, abs_tol=1.0e-12):
                selected = min(selected, candidate, key=lambda item: item[0])
        signed_error = selected[0] - source_timestamp_sec
        absolute_error = abs(signed_error)
        if absolute_error > self.max_sync_error_sec:
            return self._rejected(SyncReason.TOO_FAR, source_timestamp_sec,
                                  oldest, newest, selected[0], signed_error,
                                  absolute_error)
        return self._accepted(self.value_extractor(selected[1]),
                              source_timestamp_sec, selected[0], 'NEAREST',
                              absolute_error, oldest, newest)

    def _accepted(self, value: T, source: float, matched: float, mode: str,
                  absolute: float, oldest: float,
                  newest: float) -> SyncResult[T]:
        return SyncResult(value, SyncReason.OK, source, matched,
                          matched - source, absolute, mode, oldest, newest,
                          self.reset_count)

    def _rejected(self, reason: SyncReason, source: float,
                  oldest: Optional[float] = None,
                  newest: Optional[float] = None,
                  matched: Optional[float] = None,
                  signed: Optional[float] = None,
                  absolute: Optional[float] = None) -> SyncResult[T]:
        return SyncResult(None, reason, source, matched, signed, absolute,
                          'NONE', oldest, newest, self.reset_count)
