"""Pure reliability logic shared by the ROS nodes and unit tests."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Optional, Tuple


METERS_PER_DEGREE_LATITUDE = 111320.0
VALID_FAULT_MODES = (
    'none', 'step', 'drift', 'dropout', 'noise', 'freeze', 'delay',
    'timestamp_jitter', 'out_of_order', 'spike', 'plausible_spoof',
    'zero_timestamp', 'backwards_timestamp', 'frozen_timestamp',
)
VALID_TRUST_STATES = ('HEALTHY', 'SUSPECT', 'FAULT')


def meters_to_latitude_degrees(north_m: float) -> float:
    """Convert a north displacement in meters to latitude degrees."""
    return north_m / METERS_PER_DEGREE_LATITUDE


def meters_to_longitude_degrees(east_m: float, latitude_deg: float) -> float:
    """Convert an east displacement in meters to longitude degrees."""
    meters_per_degree = (
        METERS_PER_DEGREE_LATITUDE * math.cos(math.radians(latitude_deg))
    )
    if abs(meters_per_degree) < 1.0e-9:
        raise ValueError('longitude conversion is undefined at the poles')
    return east_m / meters_per_degree


def apply_meter_offsets(
    latitude: float,
    longitude: float,
    altitude: float,
    east_m: float,
    north_m: float,
    altitude_m: float,
) -> Tuple[float, float, float]:
    """Apply local meter offsets to a geodetic coordinate."""
    return (
        latitude + meters_to_latitude_degrees(north_m),
        longitude + meters_to_longitude_degrees(east_m, latitude),
        altitude + altitude_m,
    )


def effective_offsets(
    mode: str,
    elapsed_sec: float,
    east_offset_m: float,
    north_offset_m: float,
    altitude_offset_m: float,
    drift_rate_east_mps: float,
    drift_rate_north_mps: float,
) -> Tuple[float, float, float]:
    """Return deterministic offsets for a fault mode and elapsed time."""
    if mode in (
        'none', 'dropout', 'noise', 'freeze', 'delay', 'timestamp_jitter',
        'out_of_order', 'spike', 'zero_timestamp', 'backwards_timestamp',
        'frozen_timestamp',
    ):
        return 0.0, 0.0, 0.0
    if mode == 'step':
        return east_offset_m, north_offset_m, altitude_offset_m
    if mode == 'drift':
        elapsed = max(0.0, elapsed_sec)
        return (
            east_offset_m + drift_rate_east_mps * elapsed,
            north_offset_m + drift_rate_north_mps * elapsed,
            altitude_offset_m,
        )
    if mode == 'plausible_spoof':
        from ares_reliability.fault_models import plausible_spoof_offset
        return plausible_spoof_offset(elapsed_sec), 0.0, 0.0
    raise ValueError(f'unsupported fault mode: {mode}')


def injected_coordinates(
    enabled: bool,
    mode: str,
    latitude: float,
    longitude: float,
    altitude: float,
    elapsed_sec: float = 0.0,
    east_offset_m: float = 0.0,
    north_offset_m: float = 0.0,
    altitude_offset_m: float = 0.0,
    drift_rate_east_mps: float = 0.0,
    drift_rate_north_mps: float = 0.0,
) -> Tuple[float, float, float]:
    """Calculate the output coordinate for an enabled or disabled injector."""
    active_mode = mode if enabled else 'none'
    offsets = effective_offsets(
        active_mode,
        elapsed_sec,
        east_offset_m,
        north_offset_m,
        altitude_offset_m,
        drift_rate_east_mps,
        drift_rate_north_mps,
    )
    return apply_meter_offsets(
        latitude,
        longitude,
        altitude,
        offsets[0],
        offsets[1],
        offsets[2],
    )


def classify_trust_state(
    rolling_mean_m: float,
    healthy_threshold_m: float,
    fault_threshold_m: float,
) -> str:
    """Classify a rolling residual using the experimental thresholds."""
    if rolling_mean_m < healthy_threshold_m:
        return 'HEALTHY'
    if rolling_mean_m < fault_threshold_m:
        return 'SUSPECT'
    return 'FAULT'


@dataclass(frozen=True)
class TrustEvaluation:
    """Result of one residual or explicit-state evaluation."""

    rolling_mean_m: Optional[float]
    candidate_state: str
    public_state: str
    transition: Optional[Tuple[str, str]]


class RollingTrustEvaluator:
    """Maintain the rolling residual and debounced public trust state."""

    def __init__(
        self,
        window_size: int = 5,
        healthy_threshold_m: float = 1.5,
        fault_threshold_m: float = 3.0,
        required_consecutive: int = 5,
    ) -> None:
        if window_size < 1:
            raise ValueError('window_size must be at least 1')
        if healthy_threshold_m < 0.0:
            raise ValueError('healthy_threshold_m must be non-negative')
        if fault_threshold_m <= healthy_threshold_m:
            raise ValueError(
                'fault_threshold_m must be greater than healthy_threshold_m'
            )
        if required_consecutive < 1:
            raise ValueError('required_consecutive must be at least 1')

        self.healthy_threshold_m = healthy_threshold_m
        self.fault_threshold_m = fault_threshold_m
        self.required_consecutive = required_consecutive
        self.residuals: Deque[float] = deque(maxlen=window_size)
        self.current_state = 'HEALTHY'
        self.candidate_state: Optional[str] = None
        self.candidate_count = 0

    @property
    def rolling_mean_m(self) -> Optional[float]:
        """Return the current rolling mean, or None before the first sample."""
        if not self.residuals:
            return None
        return sum(self.residuals) / len(self.residuals)

    def add_residual(self, residual_m: float) -> TrustEvaluation:
        """Add one residual and evaluate the debounced public state."""
        if not math.isfinite(residual_m) or residual_m < 0.0:
            raise ValueError('residual_m must be finite and non-negative')
        self.residuals.append(residual_m)
        rolling_mean = self.rolling_mean_m
        assert rolling_mean is not None
        candidate = classify_trust_state(
            rolling_mean,
            self.healthy_threshold_m,
            self.fault_threshold_m,
        )
        transition = self._update_public_state(candidate)
        return TrustEvaluation(
            rolling_mean,
            candidate,
            self.current_state,
            transition,
        )

    def reset(self) -> None:
        """Clear time-series state after a simulation clock discontinuity."""
        self.residuals.clear()
        self.current_state = 'HEALTHY'
        self.candidate_state = None
        self.candidate_count = 0

    def evaluate_explicit_state(self, candidate: str) -> TrustEvaluation:
        """Apply the same debounce to a non-residual condition such as dropout."""
        if candidate not in VALID_TRUST_STATES:
            raise ValueError(f'unsupported trust state: {candidate}')
        transition = self._update_public_state(candidate)
        return TrustEvaluation(
            self.rolling_mean_m,
            candidate,
            self.current_state,
            transition,
        )

    def _update_public_state(
        self,
        candidate: str,
    ) -> Optional[Tuple[str, str]]:
        if candidate == self.current_state:
            self.candidate_state = None
            self.candidate_count = 0
            return None

        if candidate == self.candidate_state:
            self.candidate_count += 1
        else:
            self.candidate_state = candidate
            self.candidate_count = 1

        if self.candidate_count < self.required_consecutive:
            return None

        previous = self.current_state
        self.current_state = candidate
        self.candidate_state = None
        self.candidate_count = 0
        return previous, self.current_state
