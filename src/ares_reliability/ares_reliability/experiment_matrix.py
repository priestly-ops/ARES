"""Validation and sensor-profile extraction for the Week 4 matrix."""

from pathlib import Path
from typing import Any

import yaml  # type: ignore[import-untyped]


SENSORS = ('gnss', 'imu', 'wheel')
EXPECTED_SEVERITIES = (
    'HEALTHY', 'DEGRADED', 'UNTRUSTED', 'HEALTHY_OR_DEGRADED',
    'DEGRADED_OR_UNTRUSTED',
)

CALIBRATION_PHASE_SEC = 18.0
CALIBRATION_REGIMES = (
    'stationary', 'straight_slow', 'straight_fast', 'gentle_left',
    'gentle_right', 'sharp_left', 'sharp_right', 'stop_start',
    'accel_decel', 'figure8',
)


def bounded_motion_command(
    linear: float,
    angular: float,
    elapsed: float,
    reverse_period_sec: float,
) -> tuple[float, float]:
    """Reverse a command periodically so an open-loop run retraces its path."""
    if elapsed < 0.0:
        raise ValueError('bounded-motion elapsed time must be non-negative')
    if reverse_period_sec <= 0.0:
        raise ValueError('reverse_period_sec must be positive')
    direction = -1.0 if int(elapsed / reverse_period_sec) % 2 else 1.0
    return direction * linear, direction * angular


def calibration_sweep_motion(elapsed: float) -> tuple[float, float, str]:
    """Return a bounded calibration command and its motion-regime label."""
    if elapsed < 0.0:
        raise ValueError('calibration elapsed time must be non-negative')
    phase = min(
        len(CALIBRATION_REGIMES) - 1,
        int(elapsed / CALIBRATION_PHASE_SEC),
    )
    regime = CALIBRATION_REGIMES[phase]
    within = elapsed - phase * CALIBRATION_PHASE_SEC
    half = CALIBRATION_PHASE_SEC / 2.0
    reverse = within >= half
    direction = -1.0 if reverse else 1.0

    if regime == 'stationary':
        return 0.0, 0.0, regime
    if regime == 'straight_slow':
        return direction * 0.08, 0.0, regime
    if regime == 'straight_fast':
        return direction * 0.20, 0.0, regime
    if regime == 'gentle_left':
        return direction * 0.10, direction * 0.15, regime
    if regime == 'gentle_right':
        return direction * 0.10, direction * -0.15, regime
    if regime == 'sharp_left':
        return direction * 0.06, direction * 0.50, regime
    if regime == 'sharp_right':
        return direction * 0.06, direction * -0.50, regime
    if regime == 'stop_start':
        quarter = int(within / (CALIBRATION_PHASE_SEC / 4.0))
        linear = (0.15, 0.0, 0.0, -0.15)[min(3, quarter)]
        return linear, 0.0, regime
    if regime == 'accel_decel':
        mirrored = within if not reverse else CALIBRATION_PHASE_SEC - within
        normalized = mirrored / half
        magnitude = 0.20 * (1.0 - abs(2.0 * normalized - 1.0))
        return direction * magnitude, 0.0, regime

    quarter = int(within / (CALIBRATION_PHASE_SEC / 4.0))
    linear = (0.10, 0.10, -0.10, -0.10)[min(3, quarter)]
    angular = (0.30, -0.30, 0.30, -0.30)[min(3, quarter)]
    return linear, angular, regime


def infer_expected_severity(scenario: dict[str, Any]) -> str:
    """Return an explicit or conservative expected trust response."""
    configured = scenario.get('expected_severity')
    if configured is not None:
        severity = str(configured)
        if severity not in EXPECTED_SEVERITIES:
            raise ValueError(f'unsupported expected severity: {severity}')
        return severity
    if scenario.get('sensor') == 'none':
        return 'HEALTHY'
    category = str(scenario.get('expected_attribution_category', ''))
    if category.startswith('SYSTEM_HEALTHY_OR_'):
        return 'HEALTHY_OR_DEGRADED'
    if scenario.get('fault_mode') in ('dropout', 'freeze', 'frozen_timestamp'):
        return 'UNTRUSTED'
    return 'DEGRADED'


def load_fault_matrix(path: Path) -> dict[str, Any]:
    """Load and validate a machine-readable Week 4 experiment matrix."""
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('version') != 1:
        raise ValueError('Week 4 matrix version must be 1')
    defaults = document.get('defaults')
    scenarios = document.get('scenarios')
    if not isinstance(defaults, dict) or not isinstance(scenarios, dict):
        raise ValueError('matrix requires defaults and scenarios mappings')
    for name, scenario in scenarios.items():
        if not isinstance(scenario, dict):
            raise ValueError(f'scenario {name} must be a mapping')
        for required in (
                'sensor', 'fault_mode', 'start_time', 'duration',
                'expected_evidence_pattern',
                'expected_attribution_category'):
            if required not in scenario:
                raise ValueError(f'scenario {name} missing {required}')
        if float(scenario['start_time']) < 0.0:
            raise ValueError(f'scenario {name} has negative start_time')
        if float(scenario['duration']) <= 0.0:
            raise ValueError(f'scenario {name} duration must be positive')
        scenario.setdefault('seed', defaults.get('seed', 0))
        scenario.setdefault('expected_severity',
                            infer_expected_severity(scenario))
    return document


def injector_profiles(scenario: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Return complete active profiles keyed by injector sensor name."""
    profiles = {
        sensor: {'enabled': False, 'mode': 'none'} for sensor in SENSORS}
    sensor = str(scenario['sensor'])
    if sensor in SENSORS:
        profile = dict(scenario.get('parameters', {}))
        profile.update({'enabled': True, 'mode': scenario['fault_mode']})
        if sensor in ('gnss', 'imu'):
            profile.setdefault('seed', int(scenario.get('seed', 0)))
        profiles[sensor] = profile
    elif sensor == 'multi':
        for name, configured in scenario.get('faults', {}).items():
            if name not in SENSORS:
                raise ValueError(f'unsupported injector sensor: {name}')
            profile = dict(configured)
            profile['enabled'] = True
            if name in ('gnss', 'imu'):
                profile.setdefault('seed', int(scenario.get('seed', 0)))
            profiles[name] = profile
    elif sensor != 'none' and scenario.get('automated', True):
        raise ValueError(f'unsupported automated sensor: {sensor}')
    return profiles
