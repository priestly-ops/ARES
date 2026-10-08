"""Launch the isolated Week 5 trust-aware GNSS fusion experiment."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FUSION_MODES = ('odom_imu_only', 'unprotected', 'protected')


def _fusion_nodes(context):
    package_share = Path(get_package_share_directory('ares_reliability'))
    mode = LaunchConfiguration('fusion_mode').perform(context)
    if mode not in FUSION_MODES:
        raise ValueError(
            f'Unknown fusion mode {mode!r}; expected one of {FUSION_MODES}')
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true')
    common = {'use_sim_time': use_sim_time}
    actions = [Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_trust_fusion_node',
        parameters=[
            str(package_share / 'config' / 'ekf_trust_fusion.yaml'),
            common,
        ],
        remappings=[('odometry/filtered', '/odometry/trust_fused')],
        output='screen',
    )]
    if mode != 'odom_imu_only':
        gps_topic = (
            '/ares/gps_trusted' if mode == 'protected' else '/ares/gps')
        actions.append(Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_trust_transform',
            parameters=[
                str(package_share / 'config' / 'navsat_trust.yaml'),
                common,
            ],
            remappings=[
                ('imu', '/ares/imu_operational'),
                ('gps/fix', gps_topic),
                ('odometry/filtered', '/odometry/trust_fused'),
                ('odometry/gps', '/odometry/gps_trusted'),
                ('gps/filtered', '/ares/gps_trust_filtered'),
            ],
            output='screen',
        ))
    return actions


def generate_launch_description():
    """Return the separate reliability and experimental fusion graph."""
    package_share = Path(get_package_share_directory('ares_reliability'))
    reliability_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            package_share / 'launch' / 'week4_reliability.launch.py')),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in (
                'scenario', 'activate_on_start', 'record', 'result_name',
                'results_dir', 'use_sim_time', 'fusion_mode', 'matrix_file')
        }.items(),
    )
    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('scenario', default_value='healthy_stationary'),
        DeclareLaunchArgument('activate_on_start', default_value='false'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('result_name', default_value='week5_experiment'),
        DeclareLaunchArgument('results_dir', default_value='results/week5'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fusion_mode', default_value='protected'),
        DeclareLaunchArgument(
            'matrix_file',
            default_value=str(
                package_share / 'config' / 'week5_fault_matrix.yaml')),
        reliability_launch,
        OpaqueFunction(function=_fusion_nodes),
        Node(
            package='ares_reliability',
            executable='estimator_health_monitor',
            name='estimator_health_monitor',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
    ])
