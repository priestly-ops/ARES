"""Pure residual calculations shared by Week 3 sensor monitors."""

import math
from typing import Optional, Tuple


EARTH_RADIUS_M = 6371000.0


def rotate_enu(east: float, north: float,
               yaw_degrees: float) -> Tuple[float, float]:
    """Rotate an ENU displacement into a configured odometry frame."""
    if any(not math.isfinite(value)
           for value in (east, north, yaw_degrees)):
        raise ValueError('ENU displacement and rotation must be finite')
    angle = math.radians(yaw_degrees)
    return (
        math.cos(angle) * east - math.sin(angle) * north,
        math.sin(angle) * east + math.cos(angle) * north,
    )


class PoseYawRateEstimator:
    """Estimate wheel yaw rate from stamped odometry orientation changes."""

    def __init__(self, maximum_interval_sec: float = 0.2) -> None:
        if maximum_interval_sec <= 0.0:
            raise ValueError('maximum_interval_sec must be positive')
        self.maximum_interval_sec = maximum_interval_sec
        self.previous: Optional[Tuple[float, float]] = None

    def reset(self) -> None:
        """Discard the previous pose sample."""
        self.previous = None

    def update(self, timestamp_sec: float, yaw_rad: float) -> Optional[float]:
        """Return a finite-difference rate, or ``None`` until valid."""
        if not math.isfinite(timestamp_sec) or not math.isfinite(yaw_rad):
            raise ValueError('timestamp and yaw must be finite')
        previous = self.previous
        self.previous = (timestamp_sec, yaw_rad)
        if previous is None:
            return None
        delta_time = timestamp_sec - previous[0]
        if delta_time <= 0.0 or delta_time > self.maximum_interval_sec:
            return None
        delta_yaw = math.atan2(
            math.sin(yaw_rad - previous[1]),
            math.cos(yaw_rad - previous[1]),
        )
        return delta_yaw / delta_time


def geodetic_distance_m(first: Tuple[float, float],
                        second: Tuple[float, float]) -> float:
    """Return a local tangent-plane distance between geodetic coordinates."""
    values = (*first, *second)
    if any(not math.isfinite(value) for value in values):
        raise ValueError('geodetic coordinates must be finite')
    mean_latitude = math.radians((first[0] + second[0]) / 2.0)
    north = EARTH_RADIUS_M * math.radians(second[0] - first[0])
    east = (EARTH_RADIUS_M * math.cos(mean_latitude) *
            math.radians(second[1] - first[1]))
    return math.hypot(east, north)


def yaw_rate_residual(imu_yaw_rate: float, odom_yaw_rate: float,
                      odometry_scale: float = 1.0) -> float:
    """Return calibrated absolute yaw-rate disagreement in rad/s."""
    if any(not math.isfinite(value) for value in (
            imu_yaw_rate, odom_yaw_rate, odometry_scale)):
        raise ValueError('yaw rates and odometry scale must be finite')
    if odometry_scale <= 0.0:
        raise ValueError('odometry scale must be positive')
    return abs(imu_yaw_rate - odometry_scale * odom_yaw_rate)


def displacement_residual(
    previous_localization: Tuple[float, float],
    localization: Tuple[float, float],
    previous_odometry: Tuple[float, float],
    odometry: Tuple[float, float],
) -> float:
    """Compare frame-invariant displacement magnitudes over one interval."""
    values = (
        *previous_localization,
        *localization,
        *previous_odometry,
        *odometry,
    )
    if any(not math.isfinite(value) for value in values):
        raise ValueError('positions must be finite')
    localization_distance = math.hypot(
        localization[0] - previous_localization[0],
        localization[1] - previous_localization[1],
    )
    odometry_distance = math.hypot(
        odometry[0] - previous_odometry[0],
        odometry[1] - previous_odometry[1],
    )
    return abs(localization_distance - odometry_distance)

def vector_displacement_residual(
    previous_localization: Tuple[float, float],
    localization: Tuple[float, float],
    previous_odometry: Tuple[float, float],
    odometry: Tuple[float, float],
) -> float:
    """Compare displacement vectors over one interval."""
    values = (
        *previous_localization,
        *localization,
        *previous_odometry,
        *odometry,
    )
    if any(not math.isfinite(value) for value in values):
        raise ValueError('positions must be finite')

    localization_delta = (
        localization[0] - previous_localization[0],
        localization[1] - previous_localization[1],
    )
    odometry_delta = (
        odometry[0] - previous_odometry[0],
        odometry[1] - previous_odometry[1],
    )

    return math.hypot(
        localization_delta[0] - odometry_delta[0],
        localization_delta[1] - odometry_delta[1],
    )

