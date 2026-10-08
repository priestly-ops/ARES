"""Unit tests for deterministic Week 4 fault primitives."""

from ares_reliability.fault_models import (
    DelayQueue,
    deterministic_gaussian,
    deterministic_uniform,
    gnss_stochastic_offsets,
    inject_scalar,
    plausible_spoof_offset,
    WheelFaultModel,
    WheelSample,
)
import pytest


def test_noise_is_deterministic_by_seed_and_index() -> None:
    first = [deterministic_gaussian(42, index, 3.0) for index in range(5)]
    second = [deterministic_gaussian(42, index, 3.0) for index in range(5)]
    assert first == second
    assert first != [deterministic_gaussian(43, index, 3.0)
                     for index in range(5)]


def test_gnss_noise_axes_and_jitter_are_bounded() -> None:
    assert gnss_stochastic_offsets('noise', 7, 2, 1.0, 5, 0.0, 0.0) == (
        gnss_stochastic_offsets('noise', 7, 2, 1.0, 5, 0.0, 0.0)
    )
    assert abs(deterministic_uniform(7, 3, 0.1)) <= 0.1


def test_spike_is_periodic() -> None:
    offsets = [gnss_stochastic_offsets(
        'spike', 0, index, 0.0, 3, 5.0, -2.0) for index in range(4)]
    assert offsets == [(0.0, 0.0), (0.0, 0.0), (5.0, -2.0), (0.0, 0.0)]


def test_delay_queue_respects_delay_and_stable_order() -> None:
    queue: DelayQueue[str] = DelayQueue()
    queue.push(1.0, 0.1, 'first')
    queue.push(1.0, 0.1, 'second')
    assert queue.pop_due(1.099) == []
    assert queue.pop_due(1.1) == ['first', 'second']


def test_imu_bias_drift_dropout_freeze_and_spike() -> None:
    assert inject_scalar(0.2, 'bias', 1.0, 0, bias=0.1) == pytest.approx(0.3)
    assert inject_scalar(0.2, 'drift', 5.0, 0,
                         drift_rate=0.05) == pytest.approx(0.45)
    assert inject_scalar(0.2, 'dropout', 0.0, 0) is None
    assert inject_scalar(0.2, 'freeze', 0.0, 0,
                         frozen_value=0.1) == 0.1
    assert inject_scalar(0.2, 'spike', 0.0, 1, spike_value=2.0,
                         spike_every_n=2) == pytest.approx(2.2)


def test_wheel_linear_scale_uses_activation_anchor() -> None:
    model = WheelFaultModel()
    anchor = WheelSample(10.0, 3.0, 0.2, 1.0, 0.1)
    anchored = model.apply(anchor, 'linear_scale', linear_scale=1.1)
    assert anchored is not None
    assert anchored.x == 10.0
    changed = model.apply(
        WheelSample(12.0, 4.0, 0.2, 1.0, 0.1),
        'linear_scale', linear_scale=1.1)
    assert changed is not None
    assert changed.x == pytest.approx(12.2)
    assert changed.y == pytest.approx(4.1)
    assert changed.linear_x == pytest.approx(1.1)


def test_wheel_bias_dropout_and_freeze() -> None:
    sample = WheelSample(1.0, 2.0, 0.1, 0.5, 0.2)
    model = WheelFaultModel()
    biased = model.apply(sample, 'yaw_bias', yaw_bias=0.1, elapsed_sec=2.0)
    assert biased is not None
    assert biased.angular_z == pytest.approx(0.3)
    assert model.apply(sample, 'dropout') is None
    model.reset()
    frozen = model.apply(sample, 'freeze')
    assert model.apply(WheelSample(4.0, 5.0, 0.3, 1.0, 0.4),
                       'freeze') == frozen


def test_plausible_spoof_is_continuous_and_recovers() -> None:
    assert plausible_spoof_offset(10.0) == 0.0
    assert plausible_spoof_offset(30.0) == pytest.approx(1.0)
    assert plausible_spoof_offset(45.0) == pytest.approx(2.5)
    assert plausible_spoof_offset(55.0) == pytest.approx(2.5)
    assert plausible_spoof_offset(60.0) == pytest.approx(1.25)
    assert plausible_spoof_offset(65.0) == 0.0
