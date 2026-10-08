"""Unit tests for deterministic Week 2 reliability logic."""

import math

from ares_reliability.core import (
    classify_trust_state,
    effective_offsets,
    injected_coordinates,
    METERS_PER_DEGREE_LATITUDE,
    meters_to_latitude_degrees,
    meters_to_longitude_degrees,
    RollingTrustEvaluator,
)
import pytest


TEST_LATITUDE = 39.73913
TEST_LONGITUDE = -104.99030
TEST_ALTITUDE = 1610.0


def test_meter_to_latitude_conversion() -> None:
    degrees = meters_to_latitude_degrees(1.0)
    assert degrees * METERS_PER_DEGREE_LATITUDE == pytest.approx(1.0)


def test_meter_to_longitude_conversion() -> None:
    degrees = meters_to_longitude_degrees(1.0, TEST_LATITUDE)
    reconstructed = (
        degrees
        * METERS_PER_DEGREE_LATITUDE
        * math.cos(math.radians(TEST_LATITUDE))
    )
    assert reconstructed == pytest.approx(1.0, abs=1.0e-9)


def test_one_meter_east_injection() -> None:
    latitude, longitude, altitude = injected_coordinates(
        True,
        'step',
        TEST_LATITUDE,
        TEST_LONGITUDE,
        TEST_ALTITUDE,
        east_offset_m=1.0,
    )
    measured_east = (
        (longitude - TEST_LONGITUDE)
        * METERS_PER_DEGREE_LATITUDE
        * math.cos(math.radians(TEST_LATITUDE))
    )
    assert latitude == TEST_LATITUDE
    assert altitude == TEST_ALTITUDE
    assert measured_east == pytest.approx(1.0, abs=1.0e-6)


def test_north_offset() -> None:
    latitude, longitude, _ = injected_coordinates(
        True,
        'step',
        TEST_LATITUDE,
        TEST_LONGITUDE,
        TEST_ALTITUDE,
        north_offset_m=3.0,
    )
    measured_north = (
        latitude - TEST_LATITUDE
    ) * METERS_PER_DEGREE_LATITUDE
    assert longitude == TEST_LONGITUDE
    assert measured_north == pytest.approx(3.0, abs=1.0e-6)


def test_disabled_injector_is_exact_passthrough() -> None:
    output = injected_coordinates(
        False,
        'drift',
        TEST_LATITUDE,
        TEST_LONGITUDE,
        TEST_ALTITUDE,
        elapsed_sec=50.0,
        east_offset_m=5.0,
        north_offset_m=7.0,
        altitude_offset_m=11.0,
        drift_rate_east_mps=2.0,
        drift_rate_north_mps=3.0,
    )
    assert output == (TEST_LATITUDE, TEST_LONGITUDE, TEST_ALTITUDE)


@pytest.mark.parametrize(
    ('residual', 'expected'),
    [
        (0.0, 'HEALTHY'),
        (1.499999, 'HEALTHY'),
        (1.5, 'SUSPECT'),
        (2.999999, 'SUSPECT'),
        (3.0, 'FAULT'),
        (10.0, 'FAULT'),
    ],
)
def test_trust_threshold_classification(
    residual: float,
    expected: str,
) -> None:
    assert classify_trust_state(residual, 1.5, 3.0) == expected


def test_persistence_requires_consecutive_candidates() -> None:
    evaluator = RollingTrustEvaluator(required_consecutive=3)
    first = evaluator.evaluate_explicit_state('FAULT')
    second = evaluator.evaluate_explicit_state('FAULT')
    third = evaluator.evaluate_explicit_state('FAULT')
    assert first.public_state == 'HEALTHY'
    assert second.public_state == 'HEALTHY'
    assert third.public_state == 'FAULT'
    assert third.transition == ('HEALTHY', 'FAULT')

    evaluator.evaluate_explicit_state('HEALTHY')
    evaluator.evaluate_explicit_state('SUSPECT')
    assert evaluator.current_state == 'FAULT'
    assert evaluator.candidate_count == 1
    assert evaluator.candidate_state == 'SUSPECT'


def test_rolling_window_discards_oldest_sample() -> None:
    evaluator = RollingTrustEvaluator(
        window_size=3,
        healthy_threshold_m=10.0,
        fault_threshold_m=20.0,
    )
    evaluator.add_residual(1.0)
    evaluator.add_residual(2.0)
    third = evaluator.add_residual(6.0)
    fourth = evaluator.add_residual(10.0)
    assert third.rolling_mean_m == pytest.approx(3.0)
    assert fourth.rolling_mean_m == pytest.approx(6.0)
    assert list(evaluator.residuals) == [2.0, 6.0, 10.0]


def test_drift_progression() -> None:
    initial = effective_offsets(
        'drift',
        0.0,
        0.5,
        -0.25,
        1.0,
        0.2,
        0.1,
    )
    later = effective_offsets(
        'drift',
        10.0,
        0.5,
        -0.25,
        1.0,
        0.2,
        0.1,
    )
    assert initial == pytest.approx((0.5, -0.25, 1.0))
    assert later == pytest.approx((2.5, 0.75, 1.0))
