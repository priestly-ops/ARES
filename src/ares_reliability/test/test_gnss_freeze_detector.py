"""Tests for motion-window GNSS freeze detection."""

from ares_reliability.gnss_freeze_detector import (
    freeze_trust_candidate,
    GnssFreezeDetector,
)


def detector() -> GnssFreezeDetector:
    return GnssFreezeDetector(
        window_sec=3.5,
        minimum_motion_m=0.2,
        maximum_gnss_displacement_m=0.15,
        confirmation_sec=0.5,
    )


def feed(detector_instance, timestamp, gnss_x, odometry_x):
    return detector_instance.update(
        timestamp, (gnss_x, 0.0), (odometry_x, 0.0))


def test_stationary_robot_does_not_trigger_on_gnss_noise() -> None:
    instance = detector()
    states = [feed(instance, index * 0.1, (index % 3) * 0.01, 0.0).state
              for index in range(120)]
    assert 'CONFIRMED' not in states


def test_moving_robot_with_healthy_gnss_does_not_trigger() -> None:
    instance = detector()
    states = [feed(instance, index * 0.1, index * 0.08, index * 0.08).state
              for index in range(120)]
    assert 'CONFIRMED' not in states


def test_moving_robot_with_frozen_gnss_confirms_after_persistence() -> None:
    instance = detector()
    states = []
    for index in range(75):
        states.append(feed(instance, index * 0.1, 0.0, index * 0.08).state)
    assert states.index('SUSPECT') < states.index('CONFIRMED')
    assert 'CONFIRMED' in states


def test_stop_start_motion_resets_candidate_without_false_freeze() -> None:
    instance = detector()
    states = []
    for index in range(60):
        displacement = max(0.0, (index - 20) * 0.08)
        states.append(feed(
            instance, index * 0.1, displacement, displacement).state)
    assert 'CONFIRMED' not in states


def test_brief_stationary_position_does_not_pass_persistence() -> None:
    instance = detector()
    states = [feed(instance, index * 0.1, index * 0.08, index * 0.08).state
              for index in range(41)]
    states.extend(feed(instance, index * 0.1, 3.2, index * 0.08).state
                  for index in range(41, 77))
    states.extend(feed(instance, index * 0.1, index * 0.08, index * 0.08).state
                  for index in range(77, 130))
    assert 'SUSPECT' in states
    assert 'CONFIRMED' not in states


def test_recovered_gnss_clears_freeze_candidate() -> None:
    instance = detector()
    for index in range(120):
        feed(instance, index * 0.1, 0.0, index * 0.08)
    assert feed(instance, 12.0, 0.0, 9.6).state == 'CONFIRMED'
    recovered = [feed(instance, index * 0.1, index * 0.08, index * 0.08)
                 for index in range(121, 190)]
    assert recovered[0].state == 'RECOVERED'
    assert recovered[-1].state == 'HEALTHY'


def test_suspect_freeze_does_not_change_trust_state() -> None:
    assert freeze_trust_candidate('SUSPECT', 'HEALTHY') is None


def test_confirmed_freeze_uses_existing_hysteretic_trust_states() -> None:
    assert freeze_trust_candidate('CONFIRMED', 'HEALTHY') == 'DEGRADED'
    assert freeze_trust_candidate('CONFIRMED', 'DEGRADED') == 'UNTRUSTED'
