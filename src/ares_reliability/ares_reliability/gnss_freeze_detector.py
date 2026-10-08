"""Detect fresh GNSS streams whose position stops during robot motion."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Optional, Tuple


Vector2 = Tuple[float, float]
Sample = Tuple[float, Vector2, Vector2]


@dataclass(frozen=True)
class FreezeEvaluation:
    """Freeze evidence and aligned displacement for one GNSS sample."""

    state: str
    odometry_displacement_m: Optional[float]
    gnss_displacement_m: Optional[float]
    detected_at_sec: Optional[float]


class GnssFreezeDetector:
    """Compare fresh GNSS and independent odometry over a time window."""

    def __init__(self, window_sec: float = 3.5,
                 minimum_motion_m: float = 0.2,
                 maximum_gnss_displacement_m: float = 0.15,
                 confirmation_sec: float = 0.5) -> None:
        values = (window_sec, minimum_motion_m,
                  maximum_gnss_displacement_m, confirmation_sec)
        if any(not math.isfinite(value) for value in values):
            raise ValueError('freeze detector parameters must be finite')
        if window_sec <= 0.0 or confirmation_sec <= 0.0:
            raise ValueError('freeze windows must be positive')
        if minimum_motion_m <= 0.0 or maximum_gnss_displacement_m < 0.0:
            raise ValueError('freeze displacement thresholds are invalid')
        self.window_sec = window_sec
        self.minimum_motion_m = minimum_motion_m
        self.maximum_gnss_displacement_m = maximum_gnss_displacement_m
        self.confirmation_sec = confirmation_sec
        self.samples: Deque[Sample] = deque()
        self.suspect_since_sec: Optional[float] = None
        self.detected_at_sec: Optional[float] = None

    def reset(self) -> None:
        """Clear motion history after a clock or reference reset."""
        self.samples.clear()
        self.suspect_since_sec = None
        self.detected_at_sec = None

    def update(self, timestamp_sec: float, gnss_xy: Vector2,
               odometry_xy: Vector2) -> FreezeEvaluation:
        """Return SUSPECT/CONFIRMED only for moving, fresh, frozen GNSS."""
        values = (timestamp_sec, *gnss_xy, *odometry_xy)
        if any(not math.isfinite(value) for value in values):
            raise ValueError('freeze detector samples must be finite')
        if self.samples and timestamp_sec <= self.samples[-1][0]:
            self.reset()
            return FreezeEvaluation('HEALTHY', None, None, None)

        self.samples.append((timestamp_sec, gnss_xy, odometry_xy))
        cutoff = timestamp_sec - self.window_sec
        while len(self.samples) > 2 and self.samples[1][0] <= cutoff:
            self.samples.popleft()
        oldest = self.samples[0]
        if timestamp_sec - oldest[0] < self.window_sec:
            return FreezeEvaluation('HEALTHY', None, None, None)

        gnss_displacement = self._distance(gnss_xy, oldest[1])
        odometry_displacement = self._distance(odometry_xy, oldest[2])
        freeze_candidate = (
            odometry_displacement >= self.minimum_motion_m and
            gnss_displacement <= self.maximum_gnss_displacement_m
        )
        if not freeze_candidate:
            recovered = self.detected_at_sec is not None
            self.suspect_since_sec = None
            self.detected_at_sec = None
            return FreezeEvaluation(
                'RECOVERED' if recovered else 'HEALTHY',
                odometry_displacement, gnss_displacement, None)

        if self.suspect_since_sec is None:
            self.suspect_since_sec = timestamp_sec
        state = 'SUSPECT'
        if timestamp_sec - self.suspect_since_sec >= self.confirmation_sec:
            state = 'CONFIRMED'
            if self.detected_at_sec is None:
                self.detected_at_sec = self.suspect_since_sec
        return FreezeEvaluation(
            state, odometry_displacement, gnss_displacement,
            self.detected_at_sec)

    @staticmethod
    def _distance(first: Vector2, second: Vector2) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])


def freeze_trust_candidate(freeze_state: str,
                           current_trust_state: str) -> Optional[str]:
    """Map confirmed freeze evidence through the existing trust states."""
    if freeze_state != 'CONFIRMED':
        return None
    return ('UNTRUSTED' if current_trust_state in ('DEGRADED', 'UNTRUSTED')
            else 'DEGRADED')
