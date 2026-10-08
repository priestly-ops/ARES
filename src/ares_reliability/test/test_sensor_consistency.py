"""Unit tests for IMU and localization residual primitives."""

import math

from ares_reliability.sensor_consistency import (
    displacement_residual,
    geodetic_distance_m,
    vector_displacement_residual,
    PoseYawRateEstimator,
    rotate_enu,
    yaw_rate_residual,
)
import pytest


def test_zero_yaw_rate_residual() -> None:
    assert yaw_rate_residual(0.4, 0.4) == 0.0


def test_known_yaw_rate_residual() -> None:
    assert yaw_rate_residual(0.7, 0.2) == pytest.approx(0.5)


def test_calibrated_yaw_rate_residual() -> None:
    assert yaw_rate_residual(
        0.30, 0.50, odometry_scale=0.60) == pytest.approx(0.0)


def test_yaw_rate_residual_rejects_invalid_scale() -> None:
    with pytest.raises(ValueError, match='scale must be positive'):
        yaw_rate_residual(0.30, 0.50, odometry_scale=0.0)


def test_equal_displacement_in_rotated_frames() -> None:
    residual = displacement_residual(
        (0.0, 0.0),
        (1.0, 0.0),
        (4.0, 8.0),
        (4.0, 9.0),
    )
    assert residual == 0.0


def test_known_displacement_residual() -> None:
    residual = displacement_residual(
        (0.0, 0.0),
        (2.0, 0.0),
        (0.0, 0.0),
        (0.5, 0.0),
    )
    assert residual == pytest.approx(1.5)


def test_geodetic_distance_is_meter_scaled() -> None:
    one_meter_north = 1.0 / 111320.0
    assert geodetic_distance_m(
        (39.7392, -104.9903),
        (39.7392 + one_meter_north, -104.9903),
    ) == pytest.approx(1.0, rel=0.01)


def test_gazebo_enu_rotation_maps_north_to_positive_odom_x() -> None:
    assert rotate_enu(2.0, 3.0, -90.0) == pytest.approx((3.0, -2.0))


def test_pose_yaw_rate_handles_angle_wrap() -> None:
    estimator = PoseYawRateEstimator()
    assert estimator.update(1.0, math.radians(179.0)) is None
    rate = estimator.update(1.1, math.radians(-179.0))
    assert rate == pytest.approx(math.radians(20.0))


def test_pose_yaw_rate_rejects_backwards_and_stale_intervals() -> None:
    estimator = PoseYawRateEstimator(maximum_interval_sec=0.2)
    assert estimator.update(1.0, 0.0) is None
    assert estimator.update(0.9, 0.1) is None
    assert estimator.update(2.0, 0.2) is None

def test_vector_displacement_residual_detects_direction_disagreement() -> None:
    residual = vector_displacement_residual(
        (0.0, 0.0),
        (1.0, 0.0),
        (0.0, 0.0),
        (0.0, 1.0),
    )
    assert residual == pytest.approx(math.sqrt(2.0))


def test_vector_displacement_residual_zero_for_matching_motion() -> None:
    residual = vector_displacement_residual(
        (2.0, -1.0),
        (3.5, 0.5),
        (10.0, 4.0),
        (11.5, 5.5),
    )
    assert residual == pytest.approx(0.0)

