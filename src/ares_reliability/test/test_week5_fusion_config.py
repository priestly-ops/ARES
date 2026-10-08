"""Static safety tests for the isolated Week 5 fusion configuration."""

import math
from pathlib import Path

from ares_reliability.experiment_matrix import load_fault_matrix
import pytest
import yaml  # type: ignore[import-untyped]


PACKAGE = Path(__file__).parents[1]


def _parameters(name: str, node: str) -> dict:
    document = yaml.safe_load(
        (PACKAGE / 'config' / name).read_text(encoding='utf-8'))
    return document[node]['ros__parameters']


def test_experimental_ekf_is_isolated_from_production_tf() -> None:
    params = _parameters('ekf_trust_fusion.yaml', 'ekf_trust_fusion_node')
    assert params['publish_tf'] is False
    assert params['world_frame'] == 'odom'
    assert params['odom0'] == '/ares/odom_operational'
    assert params['imu0'] == '/ares/imu_operational'
    assert params['odom1'] == '/odometry/gps_trusted'
    assert params['odom0_config'] == [
        False, False, False, False, False, False,
        True, True, False, False, False, False,
        False, False, False,
    ]
    assert params['imu0_config'][11] is True
    assert sum(params['imu0_config']) == 1
    assert params['odom1_config'][:2] == [True, True]
    assert sum(params['odom1_config']) == 2


def test_navsat_transform_uses_supported_conservative_options() -> None:
    params = _parameters('navsat_trust.yaml', 'navsat_trust_transform')
    assert params['broadcast_utm_transform'] is False
    assert params['publish_filtered_gps'] is False
    assert params['use_odometry_yaw'] is False
    assert params['wait_for_datum'] is False
    assert params['yaw_offset'] == pytest.approx(math.pi / 2.0)


def test_launch_keeps_production_output_unmodified() -> None:
    launch = (PACKAGE / 'launch' / 'week5_trust_fusion.launch.py').read_text(
        encoding='utf-8')
    assert '/odometry/trust_fused' in launch
    assert "'/odometry/filtered'" not in launch
    assert "'/ares/gps_trusted'" in launch
    assert "'/ares/gps'" in launch
    assert "FUSION_MODES = ('odom_imu_only', 'unprotected', 'protected')" in launch


def test_week5_matrix_covers_required_active_fusion_scenarios() -> None:
    matrix = load_fault_matrix(PACKAGE / 'config' / 'week5_fault_matrix.yaml')
    required = {
        'healthy_fusion', 'gnss_step_1m', 'gnss_step_3m', 'gnss_step_5m',
        'gnss_step_10m', 'gnss_slow_drift', 'gnss_dropout', 'gnss_freeze',
        'gnss_fault_recovery', 'repeated_fault_recovery',
        'preinit_gnss_5m', 'covariance_normal', 'covariance_moderate',
        'covariance_strong', 'covariance_step5',
    }
    assert required <= set(matrix['scenarios'])
    preinit = matrix['scenarios']['preinit_gnss_5m']
    assert preinit['activate_on_start']
    assert preinit['trusted_initialization'] == 'configured_anchor'
    assert matrix['scenarios']['repeated_fault_recovery']['cycles'] >= 2
    covariance = matrix['scenarios']['covariance_step5']
    assert covariance['parameters']['east_offset_m'] == 5.0
    assert covariance['recovery_parameters'][
        'untrusted_observations_before_gate'] >= 100000


def test_freeze_detector_is_opted_in_only_for_week4_week5() -> None:
    week4 = yaml.safe_load(
        (PACKAGE / 'config' / 'week4_trust.yaml').read_text(
            encoding='utf-8'))
    week3 = yaml.safe_load(
        (PACKAGE / 'config' / 'week3_trust.yaml').read_text(
            encoding='utf-8'))
    assert week4['gnss_monitor']['freeze_detection_enabled'] is True
    assert week4['gnss_monitor']['freeze_window_sec'] == 2.0
    assert week4['gnss_monitor']['freeze_minimum_motion_m'] == 0.2
    assert week4['gnss_monitor'][
        'freeze_maximum_gnss_displacement_m'] == 0.15
    assert week4['gnss_monitor']['freeze_confirmation_sec'] == 0.5
    assert 'freeze_detection_enabled' not in week3['gnss_monitor']
