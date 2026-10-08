"""Opt-in Week 6 navigation launch with isolated experimental TF authority."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogError,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


from launch_ros.actions import Node


import xacro


NAVIGATION_MODES = ('baseline', 'unprotected', 'protected')
WEEK6_RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
WEEK6_AUTOMATIC_DISCOVERY_RANGE = 'LOCALHOST'
WEEK6_RAW_CLOCK_TOPIC = '/ares/week6/raw_clock'
WEEK6_CLOCK_RATE_HZ = 100.0
MODE_LIFECYCLE_NODE_ORDER = {
    'baseline': [
        'map_server', 'amcl', 'planner_server', 'controller_server',
        'behavior_server',
    ],
    'unprotected': [
        'map_server', 'planner_server', 'controller_server',
        'behavior_server',
    ],
    'protected': [
        'map_server', 'planner_server', 'controller_server',
        'behavior_server',
    ],
}


def _include_baseline(stage: str, context, nav2_parameters: str | None = None):
    simulation_share = Path(
        get_package_share_directory('ares_simulation'))
    launch_arguments = {
        'stage': stage,
        'rviz': LaunchConfiguration('rviz').perform(context),
        'camera': 'false',
        'ground_truth': 'true',
        'seed': LaunchConfiguration('seed').perform(context),
        'map': LaunchConfiguration('map').perform(context),
        'clock_ros_topic': WEEK6_RAW_CLOCK_TOPIC,
    }
    if nav2_parameters is not None:
        launch_arguments['nav2_params'] = nav2_parameters
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            simulation_share / 'launch' / 'ares_baseline.launch.py')),
        launch_arguments=launch_arguments.items(),
    )


def _clock_boundary():
    return Node(
        package='ares_simulation',
        executable='week6_clock_boundary',
        name='week6_clock_boundary',
        parameters=[{
            'input_topic': WEEK6_RAW_CLOCK_TOPIC,
            'output_topic': '/clock',
            'output_rate_hz': WEEK6_CLOCK_RATE_HZ,
        }],
        output='screen',
    )


def _lifecycle_orchestrator(
        *, name: str, phase: str, navigation_mode: str,
        node_names: list[str]):
    arguments = [
        '--navigation-mode', navigation_mode,
        '--phase', phase,
    ]
    for node_name in node_names:
        arguments.extend(['--node', node_name])
    return Node(
        package='ares_reliability',
        executable='week6_nav2_lifecycle_orchestrator',
        name=name,
        arguments=arguments,
        output='screen',
    )


def _week6_startup_barrier_actions(*, navigation_mode: str):
    upstream_orchestrator = _lifecycle_orchestrator(
        name='week6_nav2_lifecycle_orchestrator_upstream',
        phase='upstream',
        navigation_mode=navigation_mode,
        node_names=MODE_LIFECYCLE_NODE_ORDER[navigation_mode],
    )
    barrier_arguments = [
        '--timeout-wall-sec', '60.0',
        '--navigation-mode', navigation_mode,
    ]
    startup_barrier = Node(
        package='ares_reliability',
        executable='week6_nav2_startup_barrier',
        name='week6_nav2_startup_barrier',
        arguments=barrier_arguments,
        output='screen',
    )
    bt_orchestrator = _lifecycle_orchestrator(
        name='week6_nav2_lifecycle_orchestrator_bt',
        phase='bt',
        navigation_mode=navigation_mode,
        node_names=['bt_navigator'],
    )

    def _upstream_orchestrator_exited(event, _context):
        if event.returncode == 0:
            confirmations = [
                LogInfo(msg=f'Server {node_name} connected with bond')
                for node_name in MODE_LIFECYCLE_NODE_ORDER[navigation_mode]
            ]
            return confirmations + [
                LogInfo(msg=(
                    'Week 6 upstream lifecycle states are confirmed; '
                    'starting endpoint readiness barrier')),
                startup_barrier,
            ]
        return [
            LogError(msg=(
                'Week 6 upstream lifecycle orchestration failed; '
                'endpoint barrier and navigation goals remain blocked')),
            Shutdown(reason='Week 6 upstream lifecycle orchestration failed'),
        ]

    def _startup_barrier_exited(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg=(
                    'Week 6 Nav2 startup endpoints are discoverable; '
                    'starting bt_navigator lifecycle activation')),
                bt_orchestrator,
            ]
        return [
            LogError(msg=(
                'Week 6 Nav2 startup readiness barrier failed; '
                'bt_navigator will remain inactive')),
            Shutdown(reason='Week 6 Nav2 startup readiness barrier failed'),
        ]

    def _bt_orchestrator_exited(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg='Server bt_navigator connected with bond'),
                LogInfo(msg=(
                    'Week 6 bt_navigator lifecycle state is confirmed '
                    'active')),
            ]
        return [
            LogError(msg=(
                'Week 6 bt_navigator lifecycle orchestration failed; '
                'navigation goals remain blocked')),
            Shutdown(reason='Week 6 bt_navigator lifecycle failed'),
        ]

    return [
        RegisterEventHandler(OnProcessExit(
            target_action=upstream_orchestrator,
            on_exit=_upstream_orchestrator_exited,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=startup_barrier,
            on_exit=_startup_barrier_exited,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=bt_orchestrator,
            on_exit=_bt_orchestrator_exited,
        )),
        TimerAction(period=3.0, actions=[upstream_orchestrator]),
    ]


def _setup(context):
    mode = LaunchConfiguration('navigation_mode').perform(context)
    if mode not in NAVIGATION_MODES:
        raise ValueError(
            f'Unknown navigation_mode {mode!r}; expected {NAVIGATION_MODES}')
    reliability_share = Path(
        get_package_share_directory('ares_reliability'))
    nav2_parameters = str(
        reliability_share / 'config' / 'nav2_week6_navigation.yaml')
    if mode == 'baseline':
        actions = [
            _include_baseline(
                'nav2_week6_upstream', context, nav2_parameters),
            _clock_boundary(),
        ]
        actions.extend(_week6_startup_barrier_actions(
            navigation_mode=mode))
        return actions

    description_share = Path(
        get_package_share_directory('ares_jackal_description'))
    # Week 6 uses a stateless SimpleGoalChecker so XY and yaw must be true at
    # the same instant. The production Week 4/5 file remains untouched.
    robot_description = xacro.process_file(str(
        description_share / 'urdf' / 'jackal.urdf.xacro')).toxml()
    use_sim_time = True

    # The baseline launch's sensors stage provides the identical world and
    # bridge, while deliberately stopping before the production EKF and AMCL.
    actions = [_include_baseline('sensors', context), _clock_boundary()]
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
        output='screen',
    ))

    # Reuse the frozen Week 5 reliability graph and matrix. Only the GNSS
    # path selected below differs between protected and unprotected modes.
    week4_arguments = {
        'scenario': LaunchConfiguration('scenario').perform(context),
        'activate_on_start': 'false',
        'record': 'false',
        'fusion_mode': mode,
        'matrix_file': LaunchConfiguration('matrix_file').perform(context),
        'use_sim_time': 'true',
        'rmw_implementation': WEEK6_RMW_IMPLEMENTATION,
        'gnss_residual_mode': 'window_vector',
        'gnss_residual_window_sec': '2.0',
        'gnss_degrade_persistence': '2',
        'gnss_untrusted_persistence': '2',
        'gnss_recovery_persistence': '10',
        'gnss_step_fault_latch_enabled': 'true',
        'gnss_healthy_threshold_m': '1.5',
        'gnss_fault_threshold_m': '4.0',
        'probation_after_ungated_degraded': 'false',
        # Fast containment is intentionally Week-6 protected-only.
        'prefusion_guard_enabled': (
            'true' if mode == 'protected' else 'false'),
        'prefusion_odometry_topic': '/ares/odom_operational',
        'prefusion_threshold_m': '4.0',
        'prefusion_window_sec': '2.0',
        'prefusion_max_sync_error_sec': '0.15',
        'prefusion_history_duration_sec': '5.0',
        'prefusion_expected_odometry_rate_hz': '50.0',
        'prefusion_gnss_to_odom_yaw_deg': '-90.0',
    }
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            reliability_share / 'launch' / 'week4_reliability.launch.py')),
        launch_arguments=week4_arguments.items(),
    ))

    # Keep protected-vs-unprotected GNSS measurement semantics identical.
    # Both paths receive the same covariance normalization. Only protected
    # mode is controlled by the ARES recovery policy.
    if mode == 'unprotected':
        actions.append(Node(
            package='ares_reliability',
            executable='gnss_trusted_proxy',
            name='gnss_unprotected_normalizer',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/ares/gps',
                'output_topic': '/ares/gps_unprotected_normalized',
                'unknown_variance_m2': 1.0,
                'initial_covariance_factor': 1.0,
                'initial_forward': True,
                'initial_state': 'NORMAL',
                # Deliberately disconnected from the real recovery policy.
                'command_topic': '/ares/week6/unprotected_unused_policy',
                'status_topic': '/ares/week6/unprotected_gnss_state',
            }],
            output='screen',
        ))

    gps_topic = (
        '/ares/gps_trusted'
        if mode == 'protected'
        else '/ares/gps_unprotected_normalized'
    )
    actions.extend([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_trust_fusion_node',
            parameters=[
                str(reliability_share / 'config' /
                    'ekf_week6_navigation.yaml'),
                {
                    'use_sim_time': use_sim_time,
                    # Week 5 remains message-only. This explicit Week 6 launch
                    # is isolated from the production TF authority.
                    'publish_tf': True,
                },
            ],
            remappings=[
                ('odometry/filtered', '/odometry/trust_fused'),
            ],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_trust_transform',
            parameters=[
                str(reliability_share / 'config' /
                    'navsat_week6_navigation.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('imu', '/ares/imu_operational'),
                ('gps/fix', gps_topic),
                ('odometry/filtered', '/odometry/trust_fused'),
                ('odometry/gps', '/odometry/gps_trusted'),
                ('gps/filtered', '/ares/gps_trust_filtered'),
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='week6_map_to_odom',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='ares_reliability',
            executable='estimator_health_monitor',
            name='estimator_health_monitor',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])

    map_path = LaunchConfiguration('map').perform(context)
    actions.extend([
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server',
            parameters=[nav2_parameters, {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_path,
            }],
            output='screen',
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', parameters=[nav2_parameters],
            output='screen',
        ),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server',
            parameters=[nav2_parameters],
            # Lyrical's node-specific YAML value takes precedence over a
            # later launch parameter file. An explicit topic remap preserves
            # the production parameter set while selecting experimental odom.
            remappings=[
                ('/odometry/filtered', '/odometry/trust_fused'),
            ],
            output='screen',
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', parameters=[nav2_parameters],
            output='screen',
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_parameters],
            remappings=[
                ('/odometry/filtered', '/odometry/trust_fused'),
            ],
            output='screen',
        ),
        Node(
            package='ares_localization', executable='cmd_vel_adapter.py',
            name='cmd_vel_adapter',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])

    actions.extend(_week6_startup_barrier_actions(
        navigation_mode=mode))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2',
            parameters=[{'use_sim_time': use_sim_time}], output='screen'))
    return actions


def generate_launch_description():
    simulation_share = Path(
        get_package_share_directory('ares_simulation'))
    reliability_share = Path(
        get_package_share_directory('ares_reliability'))
    return LaunchDescription([
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION', WEEK6_RMW_IMPLEMENTATION),
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE',
            WEEK6_AUTOMATIC_DISCOVERY_RANGE),
        DeclareLaunchArgument('navigation_mode', default_value='protected'),
        DeclareLaunchArgument('scenario', default_value='healthy_fusion'),
        DeclareLaunchArgument('seed', default_value='2506'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'map', default_value=str(
                simulation_share / 'maps' / 'ares_warehouse.yaml')),
        DeclareLaunchArgument(
            'matrix_file', default_value=str(
                reliability_share / 'config' /
                'week5_fault_matrix.yaml')),
        OpaqueFunction(function=_setup),
    ])
