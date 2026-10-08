"""Launch the ARES Week 4 adversarial reliability experiment layer."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml  # type: ignore[import-untyped]


def _fault_parameters(scenario: dict, sensor: str,
                      activate: bool) -> dict:
    if not activate:
        return {'enabled': False, 'mode': 'none'}
    if scenario.get('sensor') == sensor:
        result = dict(scenario.get('parameters', {}))
        result.update({'enabled': True, 'mode': scenario['fault_mode']})
        if sensor in ('gnss', 'imu'):
            result.setdefault('seed', scenario.get('seed', 2404))
        return result
    combined = scenario.get('faults', {}).get(sensor)
    if combined is not None:
        result = dict(combined)
        result['enabled'] = True
        if sensor in ('gnss', 'imu'):
            result.setdefault('seed', scenario.get('seed', 2404))
        return result
    return {'enabled': False, 'mode': 'none'}


def _setup(context):
    package_share = Path(get_package_share_directory('ares_reliability'))
    trust = yaml.safe_load(
        (package_share / 'config' / 'week4_trust.yaml').read_text(
            encoding='utf-8'))
    configured_matrix = LaunchConfiguration('matrix_file').perform(context)
    matrix_path = (
        Path(configured_matrix) if configured_matrix else
        package_share / 'config' / 'week4_fault_matrix.yaml')
    matrix = yaml.safe_load(matrix_path.read_text(encoding='utf-8'))
    scenario_name = LaunchConfiguration('scenario').perform(context)
    scenarios = matrix['scenarios']
    if scenario_name not in scenarios:
        raise ValueError(f'Unknown Week 4 scenario: {scenario_name}')
    scenario = dict(scenarios[scenario_name])
    scenario.setdefault('seed', matrix['defaults']['seed'])
    activate = (
        LaunchConfiguration('activate_on_start').perform(context).lower() ==
        'true' or bool(scenario.get('activate_on_start', False)))
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true')
    common = {'use_sim_time': use_sim_time}
    gnss_monitor = dict(trust['gnss_monitor'])
    gnss_monitor.update({
        'healthy_threshold_m': float(
            LaunchConfiguration('gnss_healthy_threshold_m').perform(context)),
        'fault_threshold_m': float(
            LaunchConfiguration('gnss_fault_threshold_m').perform(context)),
    })
    gnss_monitor['residual_mode'] = LaunchConfiguration(
        'gnss_residual_mode').perform(context)
    gnss_monitor['residual_window_sec'] = float(
        LaunchConfiguration('gnss_residual_window_sec').perform(context))
    gnss_monitor['degrade_persistence'] = int(
        LaunchConfiguration('gnss_degrade_persistence').perform(context))
    gnss_monitor['untrusted_persistence'] = int(
        LaunchConfiguration('gnss_untrusted_persistence').perform(context))
    gnss_monitor['recovery_persistence'] = int(
        LaunchConfiguration('gnss_recovery_persistence').perform(context))
    gnss_monitor['step_fault_latch_enabled'] = (
        LaunchConfiguration('gnss_step_fault_latch_enabled')
        .perform(context).lower() == 'true')
    recovery_manager = dict(trust['recovery_manager'])
    recovery_manager.update(scenario.get('recovery_parameters', {}))
    recovery_manager['probation_after_ungated_degraded'] = (
        LaunchConfiguration(
            'probation_after_ungated_degraded'
        ).perform(context).lower() == 'true')
    gnss_proxy = dict(trust['gnss_trusted_proxy'])
    gnss_proxy.update(scenario.get('gnss_proxy_parameters', {}))
    gnss_proxy.update({
        'prefusion_guard_enabled': (
            LaunchConfiguration('prefusion_guard_enabled')
            .perform(context).lower() == 'true'),
        'prefusion_odometry_topic': LaunchConfiguration(
            'prefusion_odometry_topic').perform(context),
        'prefusion_threshold_m': float(
            LaunchConfiguration('prefusion_threshold_m').perform(context)),
        'prefusion_window_sec': float(
            LaunchConfiguration('prefusion_window_sec').perform(context)),
        'prefusion_max_sync_error_sec': float(
            LaunchConfiguration(
                'prefusion_max_sync_error_sec').perform(context)),
        'prefusion_history_duration_sec': float(
            LaunchConfiguration(
                'prefusion_history_duration_sec').perform(context)),
        'prefusion_expected_odometry_rate_hz': float(
            LaunchConfiguration(
                'prefusion_expected_odometry_rate_hz').perform(context)),
        'prefusion_gnss_to_odom_yaw_deg': float(
            LaunchConfiguration(
                'prefusion_gnss_to_odom_yaw_deg').perform(context)),
    })
    if scenario.get('trusted_initialization') == 'configured_anchor':
        gnss_monitor['initialization_mode'] = 'configured_anchor'
        recovery_manager['startup_forward'] = False
        gnss_proxy.update({
            'initial_forward': False,
            'initial_state': 'GATED',
        })
    actions = [
        Node(
            package='ares_reliability', executable='gnss_fault_injector',
            name='gnss_fault_injector',
            parameters=[_fault_parameters(scenario, 'gnss', activate), common],
            output='screen'),
        Node(
            package='ares_reliability', executable='imu_fault_injector',
            name='imu_fault_injector',
            parameters=[_fault_parameters(scenario, 'imu', activate), common],
            output='screen'),
        Node(
            package='ares_reliability', executable='wheel_fault_injector',
            name='wheel_fault_injector',
            parameters=[_fault_parameters(scenario, 'wheel', activate), common],
            output='screen'),
        Node(
            package='ares_reliability', executable='consistency_monitor',
            name='consistency_monitor', parameters=[gnss_monitor, common],
            output='screen'),
        Node(
            package='ares_reliability', executable='imu_consistency_monitor',
            name='imu_consistency_monitor',
            parameters=[trust['imu_monitor'], common], output='screen'),
        Node(
            package='ares_reliability',
            executable='localization_consistency_monitor',
            name='localization_consistency_monitor',
            parameters=[trust['localization_monitor'], common],
            output='screen'),
        Node(
            package='ares_reliability', executable='trust_engine',
            name='trust_engine', parameters=[common], output='screen'),
        Node(
            package='ares_reliability', executable='recovery_manager',
            name='recovery_manager',
            parameters=[recovery_manager, common], output='screen'),
        Node(
            package='ares_reliability', executable='gnss_trusted_proxy',
            name='gnss_trusted_proxy',
            parameters=[gnss_proxy, common], output='screen'),
    ]
    if LaunchConfiguration('record').perform(context).lower() == 'true':
        actions.append(Node(
            package='ares_reliability', executable='experiment_recorder',
            name='week4_experiment_recorder',
            parameters=[{
                'output_directory': LaunchConfiguration(
                    'results_dir').perform(context),
                'result_name': LaunchConfiguration(
                    'result_name').perform(context),
                'profile': scenario_name, 'use_sim_time': use_sim_time,
                'fusion_mode': LaunchConfiguration(
                    'fusion_mode').perform(context),
                'experiment_seed': int(scenario['seed']),
            }], output='screen'))
    return actions


def generate_launch_description():
    """Return the Week 4 reliability-only launch description."""
    return LaunchDescription([
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        # The frozen Week 4 default remains CycloneDDS. Week 6 passes Fast DDS
        # explicitly so its included reliability nodes share one RMW.
        DeclareLaunchArgument(
            'rmw_implementation', default_value='rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('scenario', default_value='healthy_stationary'),
        DeclareLaunchArgument('activate_on_start', default_value='false'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('result_name', default_value='week4_experiment'),
        DeclareLaunchArgument('results_dir', default_value='results/week4'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fusion_mode', default_value='not_applicable'),
        DeclareLaunchArgument('matrix_file', default_value=''),
        DeclareLaunchArgument(
            'gnss_degrade_persistence', default_value='5'),
        DeclareLaunchArgument(
            'gnss_untrusted_persistence', default_value='5'),
        DeclareLaunchArgument(
            'gnss_recovery_persistence', default_value='10'),
        DeclareLaunchArgument(
            'gnss_step_fault_latch_enabled', default_value='false'),
        DeclareLaunchArgument(
            'gnss_healthy_threshold_m', default_value='1.5'),
        DeclareLaunchArgument(
            'gnss_fault_threshold_m', default_value='3.0'),
        DeclareLaunchArgument(
            'probation_after_ungated_degraded', default_value='true'),
        DeclareLaunchArgument(
            'prefusion_guard_enabled', default_value='false'),
        DeclareLaunchArgument(
            'prefusion_odometry_topic',
            default_value='/ares/odom_operational'),
        DeclareLaunchArgument(
            'prefusion_threshold_m', default_value='3.0'),
        DeclareLaunchArgument(
            'prefusion_window_sec', default_value='2.0'),
        DeclareLaunchArgument(
            'prefusion_max_sync_error_sec', default_value='0.15'),
        DeclareLaunchArgument(
            'prefusion_history_duration_sec', default_value='5.0'),
        DeclareLaunchArgument(
            'prefusion_expected_odometry_rate_hz', default_value='50.0'),
        DeclareLaunchArgument(
            'prefusion_gnss_to_odom_yaw_deg', default_value='-90.0'),
        OpaqueFunction(function=_setup),
    ])
