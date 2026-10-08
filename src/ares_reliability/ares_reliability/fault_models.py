"""Deterministic, message-independent fault primitives for Week 4 tests."""

from dataclasses import dataclass
import heapq
import math
import random
from typing import Generic, Optional, TypeVar


T = TypeVar('T')

GNSS_FAULT_MODES = (
    'none', 'step', 'drift', 'dropout', 'noise', 'freeze', 'delay',
    'timestamp_jitter', 'out_of_order', 'spike', 'plausible_spoof',
)
IMU_FAULT_MODES = (
    'none', 'bias', 'drift', 'dropout', 'freeze', 'spike', 'noise', 'scale',
)
WHEEL_FAULT_MODES = (
    'none', 'linear_scale', 'angular_scale', 'yaw_bias', 'dropout',
    'freeze', 'slip',
)


def deterministic_gaussian(seed: int, sample_index: int,
                           standard_deviation: float) -> float:
    """Return a reproducible independent Gaussian sample."""
    if sample_index < 0:
        raise ValueError('sample_index must be non-negative')
    if not math.isfinite(standard_deviation) or standard_deviation < 0.0:
        raise ValueError('standard_deviation must be finite and non-negative')
    generator = random.Random(f'{seed}:{sample_index}')
    return generator.gauss(0.0, standard_deviation)


def deterministic_uniform(seed: int, sample_index: int,
                          magnitude: float) -> float:
    """Return reproducible bounded symmetric jitter."""
    if sample_index < 0:
        raise ValueError('sample_index must be non-negative')
    if not math.isfinite(magnitude) or magnitude < 0.0:
        raise ValueError('magnitude must be finite and non-negative')
    generator = random.Random(f'jitter:{seed}:{sample_index}')
    return generator.uniform(-magnitude, magnitude)


def plausible_spoof_offset(
    elapsed_sec: float,
    slow_rate_mps: float = 0.05,
    accelerated_rate_mps: float = 0.10,
) -> float:
    """Return the continuous east offset for the canonical smooth spoof."""
    elapsed = max(0.0, elapsed_sec)
    if elapsed <= 10.0:
        return 0.0
    if elapsed <= 30.0:
        return (elapsed - 10.0) * slow_rate_mps
    offset_at_30 = 20.0 * slow_rate_mps
    if elapsed <= 45.0:
        return offset_at_30 + (elapsed - 30.0) * accelerated_rate_mps
    offset_at_45 = offset_at_30 + 15.0 * accelerated_rate_mps
    if elapsed <= 55.0:
        return offset_at_45
    recovery_duration = 10.0
    if elapsed <= 65.0:
        fraction = (elapsed - 55.0) / recovery_duration
        return offset_at_45 * (1.0 - fraction)
    return 0.0


def gnss_stochastic_offsets(mode: str, seed: int, sample_index: int,
                            noise_stddev_m: float, spike_every_n: int,
                            spike_east_m: float,
                            spike_north_m: float) -> tuple[float, float]:
    """Return deterministic stochastic/noise or periodic spike offsets."""
    if mode == 'noise':
        return (
            deterministic_gaussian(seed, sample_index * 2, noise_stddev_m),
            deterministic_gaussian(seed, sample_index * 2 + 1,
                                   noise_stddev_m),
        )
    if mode == 'spike':
        if spike_every_n < 1:
            raise ValueError('spike_every_n must be at least one')
        if (sample_index + 1) % spike_every_n == 0:
            return spike_east_m, spike_north_m
    return 0.0, 0.0


def inject_scalar(value: float, mode: str, elapsed_sec: float,
                  sample_index: int, seed: int = 0, bias: float = 0.0,
                  drift_rate: float = 0.0, scale: float = 1.0,
                  noise_stddev: float = 0.0, spike_value: float = 0.0,
                  spike_every_n: int = 10,
                  frozen_value: Optional[float] = None) -> Optional[float]:
    """Apply one deterministic scalar sensor fault; ``None`` means dropout."""
    if not math.isfinite(value):
        raise ValueError('input value must be finite')
    if mode == 'none':
        return value
    if mode == 'dropout':
        return None
    if mode == 'bias':
        return value + bias
    if mode == 'drift':
        return value + bias + drift_rate * max(0.0, elapsed_sec)
    if mode == 'scale':
        return value * scale
    if mode == 'noise':
        return value + deterministic_gaussian(seed, sample_index,
                                              noise_stddev)
    if mode == 'freeze':
        return value if frozen_value is None else frozen_value
    if mode == 'spike':
        if spike_every_n < 1:
            raise ValueError('spike_every_n must be at least one')
        if (sample_index + 1) % spike_every_n == 0:
            return value + spike_value
        return value
    raise ValueError(f'unsupported scalar fault mode: {mode}')


@dataclass(frozen=True)
class WheelSample:
    """Planar wheel-odometry fields altered by the Week 4 injector."""

    x: float
    y: float
    yaw: float
    linear_x: float
    angular_z: float


class WheelFaultModel:
    """Corrupt wheel messages relative to an activation anchor."""

    def __init__(self) -> None:
        self.anchor: Optional[WheelSample] = None
        self.frozen: Optional[WheelSample] = None

    def reset(self) -> None:
        """Forget activation anchors when the configured fault changes."""
        self.anchor = None
        self.frozen = None

    def apply(self, sample: WheelSample, mode: str,
              linear_scale: float = 1.0, angular_scale: float = 1.0,
              yaw_bias: float = 0.0, slip_bias_mps: float = 0.0,
              elapsed_sec: float = 0.0) -> Optional[WheelSample]:
        """Return a corrupted sample while avoiding activation discontinuities."""
        if mode == 'dropout':
            return None
        if mode == 'none':
            return sample
        if self.anchor is None:
            self.anchor = sample
        if mode == 'freeze':
            if self.frozen is None:
                self.frozen = sample
            return self.frozen
        anchor = self.anchor
        if mode == 'linear_scale':
            return WheelSample(
                anchor.x + (sample.x - anchor.x) * linear_scale,
                anchor.y + (sample.y - anchor.y) * linear_scale,
                sample.yaw, sample.linear_x * linear_scale, sample.angular_z)
        if mode == 'angular_scale':
            return WheelSample(
                sample.x, sample.y,
                anchor.yaw + (sample.yaw - anchor.yaw) * angular_scale,
                sample.linear_x, sample.angular_z * angular_scale)
        if mode == 'yaw_bias':
            return WheelSample(
                sample.x, sample.y, sample.yaw + yaw_bias * elapsed_sec,
                sample.linear_x, sample.angular_z + yaw_bias)
        if mode == 'slip':
            distance = slip_bias_mps * max(0.0, elapsed_sec)
            return WheelSample(
                sample.x + distance * math.cos(sample.yaw),
                sample.y + distance * math.sin(sample.yaw), sample.yaw,
                sample.linear_x + slip_bias_mps, sample.angular_z)
        raise ValueError(f'unsupported wheel fault mode: {mode}')


class DelayQueue(Generic[T]):
    """Stable simulated-time delay queue with deterministic tie ordering."""

    def __init__(self) -> None:
        self._queue: list[tuple[float, int, T]] = []
        self._sequence = 0

    def clear(self) -> None:
        """Discard queued messages after a reset or configuration change."""
        self._queue.clear()
        self._sequence = 0

    def push(self, now_sec: float, delay_sec: float, value: T) -> None:
        """Queue a value for release at ``now_sec + delay_sec``."""
        if delay_sec < 0.0 or not math.isfinite(delay_sec):
            raise ValueError('delay_sec must be finite and non-negative')
        heapq.heappush(
            self._queue, (now_sec + delay_sec, self._sequence, value))
        self._sequence += 1

    def pop_due(self, now_sec: float) -> list[T]:
        """Return all due values in stable release order."""
        values = []
        while self._queue and self._queue[0][0] <= now_sec:
            values.append(heapq.heappop(self._queue)[2])
        return values

    def __len__(self) -> int:
        return len(self._queue)
