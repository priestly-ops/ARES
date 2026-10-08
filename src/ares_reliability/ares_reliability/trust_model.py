"""Per-source trust with asymmetric hysteresis and unavailable semantics."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Optional, Tuple

SENSOR_TRUST_STATES = (
    'HEALTHY', 'DEGRADED', 'UNTRUSTED', 'UNAVAILABLE'
)


@dataclass(frozen=True)
class SensorTrustEvaluation:
    """One consistency evaluation and availability-aware public state."""

    rolling_mean: Optional[float]
    candidate_state: str
    public_state: str
    transition: Optional[Tuple[str, str]]
    reason: str


class HystereticTrustEvaluator:
    """Classify rolling residuals with asymmetric entry and recovery."""

    def __init__(self, window_size: int, degraded_enter: float,
                 untrusted_enter: float, degraded_exit: float,
                 healthy_exit: float, degrade_persistence: int,
                 untrusted_persistence: int,
                 recovery_persistence: int) -> None:
        if window_size < 1:
            raise ValueError('window_size must be at least 1')
        thresholds = (degraded_enter, untrusted_enter, degraded_exit,
                      healthy_exit)
        if any(not math.isfinite(value) or value < 0.0
               for value in thresholds):
            raise ValueError('thresholds must be finite and non-negative')
        if untrusted_enter <= degraded_enter:
            raise ValueError('untrusted_enter must exceed degraded_enter')
        if degraded_exit >= untrusted_enter:
            raise ValueError('degraded_exit must be below untrusted_enter')
        if healthy_exit >= degraded_enter:
            raise ValueError('healthy_exit must be below degraded_enter')
        if min(degrade_persistence, untrusted_persistence,
               recovery_persistence) < 1:
            raise ValueError('persistence values must be at least 1')
        self.residuals: Deque[float] = deque(maxlen=window_size)
        self.degraded_enter = degraded_enter
        self.untrusted_enter = untrusted_enter
        self.degraded_exit = degraded_exit
        self.healthy_exit = healthy_exit
        self.degrade_persistence = degrade_persistence
        self.untrusted_persistence = untrusted_persistence
        self.recovery_persistence = recovery_persistence
        self.consistency_state = 'HEALTHY'
        self.current_state = 'UNAVAILABLE'
        self.candidate_state: Optional[str] = None
        self.candidate_count = 0

    @property
    def rolling_mean(self) -> Optional[float]:
        """Return the current rolling mean."""
        if not self.residuals:
            return None
        return sum(self.residuals) / len(self.residuals)

    def reset(self) -> None:
        """Reset evidence after a simulation clock discontinuity."""
        self.residuals.clear()
        self.consistency_state = 'HEALTHY'
        self.current_state = 'UNAVAILABLE'
        self.candidate_state = None
        self.candidate_count = 0

    def mark_unavailable(self, reason: str) -> SensorTrustEvaluation:
        """Expose missing evidence without treating it as inconsistency."""
        previous = self.current_state
        self.current_state = 'UNAVAILABLE'
        self.candidate_state = None
        self.candidate_count = 0
        transition = None if previous == 'UNAVAILABLE' else (
            previous, 'UNAVAILABLE'
        )
        return SensorTrustEvaluation(self.rolling_mean, 'UNAVAILABLE',
                                     'UNAVAILABLE', transition, reason)

    def add_residual(self, residual: float) -> SensorTrustEvaluation:
        """Add one residual and update latent consistency state."""
        if not math.isfinite(residual) or residual < 0.0:
            raise ValueError('residual must be finite and non-negative')
        self.residuals.append(residual)
        mean = self.rolling_mean
        assert mean is not None
        return self.evaluate_explicit_state(self._classify(mean), mean)

    def evaluate_explicit_state(
        self,
        candidate: str,
        rolling_mean: Optional[float] = None,
        reason: str = 'residual classification',
    ) -> SensorTrustEvaluation:
        """Apply persistence to explicit consistency/freshness evidence."""
        if candidate == 'UNAVAILABLE':
            return self.mark_unavailable(reason)
        if candidate not in SENSOR_TRUST_STATES:
            raise ValueError(f'unsupported trust state: {candidate}')
        if self.current_state == 'UNAVAILABLE':
            self.current_state = self.consistency_state
        transition = self._update(candidate)
        self.current_state = self.consistency_state
        return SensorTrustEvaluation(
            self.rolling_mean if rolling_mean is None else rolling_mean,
            candidate, self.current_state, transition, reason)

    def force_state(
        self,
        state: str,
        rolling_mean: Optional[float] = None,
        reason: str = 'authoritative evidence',
    ) -> SensorTrustEvaluation:
        """Apply authoritative evidence without residual persistence delay."""
        if state not in ('HEALTHY', 'DEGRADED', 'UNTRUSTED'):
            raise ValueError(f'unsupported forced trust state: {state}')
        previous = self.current_state
        self.consistency_state = state
        self.current_state = state
        self.candidate_state = None
        self.candidate_count = 0
        transition = None if previous == state else (previous, state)
        return SensorTrustEvaluation(
            self.rolling_mean if rolling_mean is None else rolling_mean,
            state, state, transition, reason)

    def _classify(self, mean: float) -> str:
        if self.consistency_state == 'HEALTHY':
            if mean >= self.untrusted_enter:
                return 'UNTRUSTED'
            if mean >= self.degraded_enter:
                return 'DEGRADED'
            return 'HEALTHY'
        if self.consistency_state == 'DEGRADED':
            if mean >= self.untrusted_enter:
                return 'UNTRUSTED'
            if mean <= self.healthy_exit:
                return 'HEALTHY'
            return 'DEGRADED'
        if mean <= self.degraded_exit:
            return 'DEGRADED'
        return 'UNTRUSTED'

    def _required(self, candidate: str) -> int:
        if candidate == 'UNTRUSTED':
            return self.untrusted_persistence
        if self.consistency_state == 'HEALTHY' and candidate == 'DEGRADED':
            return self.degrade_persistence
        return self.recovery_persistence

    def _update(self, candidate: str) -> Optional[Tuple[str, str]]:
        if candidate == self.consistency_state:
            self.candidate_state = None
            self.candidate_count = 0
            return None
        if candidate == self.candidate_state:
            self.candidate_count += 1
        else:
            self.candidate_state = candidate
            self.candidate_count = 1
        if self.candidate_count < self._required(candidate):
            return None
        previous = self.consistency_state
        self.consistency_state = candidate
        self.candidate_state = None
        self.candidate_count = 0
        return previous, candidate
