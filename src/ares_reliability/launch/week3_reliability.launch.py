"""Launch ARES Week 3 multi-sensor trust and staged GNSS recovery."""

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


def _setup(context):
    package_share = Path(get_package_share_directory('ares_reliability'))
    profiles = yaml.safe_load(
        (package_share / 'config' / 'fault_profiles.yaml').read_text(
            encoding='utf-8'
        )
    )['profiles']
    trust = yaml.safe_load(
        (package_share / 'config' / 'week3_trust.yaml').read_text(
            encoding='utf-8'
        )
    )
    profile_name = LaunchConfiguration('profile').perform(context)
    if profile_name not in profiles:
        raise ValueError(f'Unknown fault profile: {profile_name}')
    result_profile = LaunchConfiguration('result_profile').perform(context)
    if not result_profile:
        result_profile = profile_name
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    )
    common = {'use_sim_time': use_sim_time}
    actions = [
        Node(
            package='ares_reliability',
            executable='gnss_fault_injector',
            name='gnss_fault_injector',
            parameters=[profiles[profile_name], common],
            output='screen',
        ),
        Node(
            package='ares_reliability',
            executable='consistency_monitor',
            name='consistency_monitor',
            parameters=[trust['gnss_monitor'], common],
            output='screen',
        ),
        Node(
            package='ares_reliability',
            executable='imu_consistency_monitor',
            name='imu_consistency_monitor',
            parameters=[trust['imu_monitor'], common],
            output='screen',
        ),
        Node(
            package='ares_reliability',
            executable='localization_consistency_monitor',
            name='localization_consistency_monitor',
            parameters=[trust['localization_monitor'], common],
            output='screen',
        ),
        Node(
            package='ares_reliability',
            executable='trust_engine',
            name='trust_engine',
            parameters=[common],
            output='screen',
        ),
        Node(
            package='ares_reliability',
            executable='recovery_manager',
            name='recovery_manager',
            parameters=[trust['recovery_manager'], common],
            output='screen',
        ),
        Node(
            package='ares_reliability',
            executable='gnss_trusted_proxy',
            name='gnss_trusted_proxy',
            parameters=[trust['gnss_trusted_proxy'], common],
            output='screen',
        ),
    ]
    if LaunchConfiguration('record').perform(context).lower() == 'true':
        actions.append(
            Node(
                package='ares_reliability',
                executable='experiment_recorder',
                name='week3_experiment_recorder',
                parameters=[
                    {
                        'output_directory': LaunchConfiguration(
                            'results_dir'
                        ).perform(context),
                        'result_name': LaunchConfiguration(
                            'result_name'
                        ).perform(context),
                        'profile': result_profile,
                        'use_sim_time': use_sim_time,
                    }
                ],
                output='screen',
            )
        )
    return actions


def generate_launch_description():
    """Return the Week 3 reliability-only launch description."""
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                'RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'
            ),
            DeclareLaunchArgument('profile', default_value='healthy'),
            DeclareLaunchArgument('result_profile', default_value=''),
            DeclareLaunchArgument('record', default_value='true'),
            DeclareLaunchArgument(
                'result_name', default_value='week3_experiment'
            ),
            DeclareLaunchArgument(
                'results_dir', default_value='results/week3'
            ),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            OpaqueFunction(function=_setup),
        ]
    )
