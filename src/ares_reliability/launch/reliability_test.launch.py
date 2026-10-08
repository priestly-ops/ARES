"""Launch the ARES Week 2 injector, monitor, and optional CSV recorder."""

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
    config_path = package_share / 'config' / 'fault_profiles.yaml'
    configuration = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    profile_name = LaunchConfiguration('profile').perform(context)
    profiles = configuration['profiles']
    if profile_name not in profiles:
        choices = ', '.join(sorted(profiles))
        raise ValueError(
            f'Unknown fault profile {profile_name!r}; choose one of: {choices}'
        )

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
            parameters=[configuration['monitor'], common],
            output='screen',
        ),
    ]

    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    if record:
        experiment_label = LaunchConfiguration(
            'experiment_label'
        ).perform(context)
        if not experiment_label:
            experiment_label = profile_name
        actions.append(
            Node(
                package='ares_reliability',
                executable='experiment_recorder',
                name='week2_experiment_recorder',
                parameters=[
                    {
                        'output_directory': LaunchConfiguration(
                            'results_dir'
                        ).perform(context),
                        'result_name': LaunchConfiguration(
                            'result_name'
                        ).perform(context),
                        'profile': experiment_label,
                        'use_sim_time': use_sim_time,
                    }
                ],
                output='screen',
            )
        )
    return actions


def generate_launch_description():
    """Return the reliability-only launch description."""
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                'RMW_IMPLEMENTATION',
                'rmw_cyclonedds_cpp',
            ),
            DeclareLaunchArgument('profile', default_value='healthy'),
            DeclareLaunchArgument('record', default_value='true'),
            DeclareLaunchArgument(
                'result_name',
                default_value='week2_experiment',
            ),
            DeclareLaunchArgument('experiment_label', default_value=''),
            DeclareLaunchArgument(
                'results_dir',
                default_value='results/week2',
            ),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            OpaqueFunction(function=_setup),
        ]
    )
