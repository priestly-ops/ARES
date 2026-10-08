"""Unit tests for message_filters-backed timestamp alignment."""

from ares_reliability.time_sync import (
    CacheTimestampAligner,
    interpolate_scalar,
    SourceTimestampValidator,
    SyncReason,
)
import message_filters
import pytest
from sensor_msgs.msg import NavSatFix


def message(timestamp: float, value: float) -> NavSatFix:
    output = NavSatFix()
    output.header.stamp.sec = int(timestamp)
    output.header.stamp.nanosec = int(round((timestamp - int(timestamp)) * 1e9))
    output.latitude = value
    return output


def aligner(samples=(), maximum: float = 0.25):
    source = message_filters.SimpleFilter()
    cache = message_filters.Cache(source, cache_size=20)
    result = CacheTimestampAligner(
        cache,
        lambda item: item.latitude,
        maximum,
        5.0,
        interpolate_scalar,
        clock_mismatch_tolerance_sec=100.0,
    )
    for timestamp, value in samples:
        source.signalMessage(message(timestamp, value))
    return result


def test_exact_timestamp_match() -> None:
    result = aligner([(10.0, 3.0)]).lookup(10.0)
    assert result.accepted
    assert result.interpolation_mode == 'EXACT'
    assert result.value == 3.0
    assert result.absolute_sync_error_sec == 0.0


def test_nearest_tie_is_deterministically_earlier() -> None:
    result = aligner([(1.0, 1.0), (1.2, 2.0)]).lookup(1.1)
    assert result.value == 1.0
    assert result.matched_timestamp_sec == 1.0
    assert result.signed_time_difference_sec == pytest.approx(-0.1)


def test_too_far_and_too_old_are_distinct() -> None:
    matcher = aligner([(2.0, 2.0)], maximum=0.1)
    assert matcher.lookup(3.0).reason == SyncReason.TOO_FAR
    assert matcher.lookup(1.0).reason == SyncReason.TOO_OLD


def test_zero_stamp_is_rejected() -> None:
    assert aligner([(1.0, 1.0)]).lookup(0.0).reason == SyncReason.ZERO_STAMP


def test_no_history_is_rejected() -> None:
    assert aligner().lookup(1.0).reason == SyncReason.NO_HISTORY


def test_linear_interpolation() -> None:
    result = aligner([(2.0, 10.0), (2.2, 20.0)]).lookup(
        2.1, interpolate=True
    )
    assert result.accepted
    assert result.interpolation_mode == 'LINEAR'
    assert result.value == pytest.approx(15.0)


def test_clock_mismatch_is_rejected() -> None:
    matcher = aligner([(2.0, 2.0)])
    matcher.clock_mismatch_tolerance_sec = 1.0
    assert matcher.lookup(2.0, clock_sec=10.0).reason == (
        SyncReason.CLOCK_MISMATCH
    )


def test_backward_clock_jump_clears_cache() -> None:
    matcher = aligner([(2.0, 2.0)])
    assert not matcher.observe_clock(3.0)
    assert matcher.observe_clock(1.0)
    assert matcher.reset_count == 1
    assert matcher.lookup(2.0).reason == SyncReason.NO_HISTORY


def test_source_timestamp_validator_rejects_zero_and_out_of_order() -> None:
    validator = SourceTimestampValidator()
    assert validator.observe(0.0) == SyncReason.ZERO_STAMP
    assert validator.observe(2.0) == SyncReason.OK
    assert validator.observe(1.9) == SyncReason.OUT_OF_ORDER


def test_source_timestamp_validator_detects_frozen_stamp() -> None:
    validator = SourceTimestampValidator(max_equal_stamps=1)
    assert validator.observe(2.0) == SyncReason.OK
    assert validator.observe(2.0) == SyncReason.OK
    assert validator.observe(2.0) == SyncReason.FROZEN_STAMP
    validator.reset()
    assert validator.observe(1.0) == SyncReason.OK
