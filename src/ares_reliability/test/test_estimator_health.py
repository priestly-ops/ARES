"""Tests for experimental-estimator health evidence."""

import math

from ares_reliability.estimator_health_monitor import odometry_health
from nav_msgs.msg import Odometry
import pytest


def odometry(x: float, y: float, stamp: float = 1.0) -> Odometry:
    message = Odometry()
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = int((stamp - int(stamp)) * 1.0e9)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.w = 1.0
    for index in (0, 7, 14, 21, 28, 35):
        message.pose.covariance[index] = 0.5
    return message


def test_healthy_odometry_records_covariance_and_reference_error() -> None:
    result = odometry_health(
        odometry(3.0, 4.0), reference_position=(0.0, 0.0))
    assert result['status'] == 'HEALTHY'
    assert result['finite']
    assert result['covariance_trace'] == pytest.approx(3.0)
    assert result['reference_error_m'] == pytest.approx(5.0)


def test_nonfinite_output_is_estimator_failure_not_sensor_fault() -> None:
    result = odometry_health(odometry(math.nan, 0.0))
    assert result['status'] == 'INVALID'
    assert not result['finite']


def test_pose_jump_and_timestamp_regression_are_explicit() -> None:
    jump = odometry_health(
        odometry(3.0, 0.0, 2.0), previous_stamp=1.0,
        previous_position=(0.0, 0.0), jump_threshold_m=2.0)
    assert jump['status'] == 'POSE_JUMP'
    assert jump['pose_jump_m'] == pytest.approx(3.0)
    regression = odometry_health(
        odometry(0.0, 0.0, 0.5), previous_stamp=1.0)
    assert regression['status'] == 'TIME_REGRESSION'
    assert regression['timestamp_regression']
