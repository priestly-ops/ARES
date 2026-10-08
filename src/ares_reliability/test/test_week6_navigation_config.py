"""Safety checks for the Week 6-only navigation estimator delta."""

from pathlib import Path

import yaml  # type: ignore[import-untyped]


PACKAGE = Path(__file__).parents[1]
LOCALIZATION_PACKAGE = PACKAGE.parent / 'ares_localization'
SIMULATION_PACKAGE = PACKAGE.parent / 'ares_simulation'
WORKSPACE = PACKAGE.parents[1]


def _parameters(filename: str) -> dict:
    document = yaml.safe_load((PACKAGE / 'config' / filename).read_text(
        encoding='utf-8'))
    return document['ekf_trust_fusion_node']['ros__parameters']


def test_week6_relative_yaw_fix_is_isolated_from_frozen_week5() -> None:
    """Only the documented IMU yaw observation may differ from Week 5."""
    week5 = _parameters('ekf_trust_fusion.yaml')
    week6 = _parameters('ekf_week6_navigation.yaml')
    assert week5['imu0_config'][5] is False
    assert week5['imu0_relative'] is False
    assert week6['imu0_config'][5] is True
    assert week6['imu0_relative'] is True
    for key in set(week5) - {'imu0_config', 'imu0_relative'}:
        assert week6[key] == week5[key]


def test_week6_modes_share_one_estimator_configuration() -> None:
    launch = (PACKAGE / 'launch' / 'week6_navigation.launch.py').read_text(
        encoding='utf-8')
    assert 'ekf_week6_navigation.yaml' in launch
    assert launch.count('ekf_week6_navigation.yaml') == 1
    assert "'/ares/gps_trusted'" in launch
    assert "'/ares/gps_unprotected_normalized'" in launch
    assert "if mode == 'protected'" in launch
    assert "'/ares/week6/unprotected_unused_policy'" in launch
    assert "'/ares/week6/unprotected_gnss_state'" in launch
    assert launch.count("('/odometry/filtered', '/odometry/trust_fused')") == 2


def test_fixed_datum_is_week6_only() -> None:
    week5 = yaml.safe_load((
        PACKAGE / 'config' / 'navsat_trust.yaml'
    ).read_text(encoding='utf-8'))['navsat_trust_transform']['ros__parameters']
    week6 = yaml.safe_load((
        PACKAGE / 'config' / 'navsat_week6_navigation.yaml'
    ).read_text(encoding='utf-8'))['navsat_trust_transform']['ros__parameters']
    assert week5['wait_for_datum'] is False
    assert 'datum' not in week5
    assert week6['wait_for_datum'] is True
    assert week6['datum'] == [39.7391356028, -104.9903, 0.0]


def test_week6_nav2_deltas_are_goal_checking_and_bt_discovery_wait() -> None:
    production = yaml.safe_load((
        LOCALIZATION_PACKAGE / 'config' / 'nav2_params.yaml'
    ).read_text(encoding='utf-8'))
    week6 = yaml.safe_load((
        PACKAGE / 'config' / 'nav2_week6_navigation.yaml'
    ).read_text(encoding='utf-8'))
    assert production['controller_server']['ros__parameters'][
        'goal_checker']['xy_goal_tolerance'] == 0.20
    assert production['controller_server']['ros__parameters'][
        'goal_checker']['stateful'] is True
    assert week6['controller_server']['ros__parameters'][
        'goal_checker']['xy_goal_tolerance'] == 0.20
    assert week6['controller_server']['ros__parameters'][
        'goal_checker']['stateful'] is False
    assert 'wait_for_service_timeout' not in (
        production['bt_navigator']['ros__parameters'])
    assert week6['bt_navigator']['ros__parameters'][
        'wait_for_service_timeout'] == 5000
    week6['controller_server']['ros__parameters'][
        'goal_checker']['stateful'] = True
    week6['bt_navigator']['ros__parameters'].pop(
        'wait_for_service_timeout')
    behavior_parameters = week6['behavior_server']['ros__parameters']
    assert behavior_parameters['behavior_plugins'] == [
        'spin', 'backup', 'drive_on_heading', 'wait']
    assert {
        name: behavior_parameters[name]['plugin']
        for name in behavior_parameters['behavior_plugins']
    } == {
        'spin': 'nav2_behaviors::Spin',
        'backup': 'nav2_behaviors::BackUp',
        'drive_on_heading': 'nav2_behaviors::DriveOnHeading',
        'wait': 'nav2_behaviors::Wait',
    }
    for name in behavior_parameters['behavior_plugins']:
        behavior_parameters.pop(name)
    behavior_parameters.pop('behavior_plugins')
    week6['controller_server']['ros__parameters']['FollowPath'].pop(
        'inflation_cost_scaling_factor')
    assert week6 == production


def test_week6_baseline_uses_override_without_changing_default() -> None:
    week6_launch = (PACKAGE / 'launch' /
                    'week6_navigation.launch.py').read_text(encoding='utf-8')
    baseline_launch = (SIMULATION_PACKAGE / 'launch' /
                       'ares_baseline.launch.py').read_text(encoding='utf-8')
    assert (
        "'nav2_week6_upstream', context, nav2_parameters"
        in week6_launch)
    assert "params = arg('nav2_params')" in baseline_launch
    assert "'nav2_week6_upstream'" in baseline_launch
    assert (
        "if stage == 'nav2' or executable != 'bt_navigator':"
        in baseline_launch)
    assert "lifecycle_manager_baseline" in baseline_launch
    assert (
        "default_value=str(loc / 'config/nav2_params.yaml')"
        in baseline_launch)


def test_week6_clock_boundary_is_opt_in_and_has_one_clock_owner() -> None:
    week6_launch = (PACKAGE / 'launch' /
                    'week6_navigation.launch.py').read_text(encoding='utf-8')
    baseline_launch = (SIMULATION_PACKAGE / 'launch' /
                       'ares_baseline.launch.py').read_text(encoding='utf-8')
    assert "WEEK6_RAW_CLOCK_TOPIC = '/ares/week6/raw_clock'" in week6_launch
    assert 'WEEK6_CLOCK_RATE_HZ = 100.0' in week6_launch
    assert week6_launch.count("executable='week6_clock_boundary'") == 1
    assert "'output_topic': '/clock'" in week6_launch
    assert "'clock_ros_topic': WEEK6_RAW_CLOCK_TOPIC" in week6_launch
    declaration = (
        'DeclareLaunchArgument(' +
        "'clock_ros_topic', default_value='/clock')")
    assert declaration in baseline_launch
    assert "mapping['ros_topic_name'] = clock_ros_topic" in baseline_launch


def test_week6_fastdds_is_explicit_without_changing_week4_default() -> None:
    week6_launch = (PACKAGE / 'launch' /
                    'week6_navigation.launch.py').read_text(encoding='utf-8')
    week4_launch = (PACKAGE / 'launch' /
                    'week4_reliability.launch.py').read_text(encoding='utf-8')
    runner = (WORKSPACE / 'scripts' /
              'run_week6_navigation.py').read_text(encoding='utf-8')
    assert "WEEK6_RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'" in week6_launch
    assert "'rmw_implementation': WEEK6_RMW_IMPLEMENTATION" in week6_launch
    assert 'rmw_cyclonedds_cpp' not in week6_launch
    assert "WEEK6_RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'" in runner
    assert "'rmw_implementation': WEEK6_RMW_IMPLEMENTATION" in runner
    assert "default_value='rmw_cyclonedds_cpp'" in week4_launch
    assert "WEEK6_AUTOMATIC_DISCOVERY_RANGE = 'LOCALHOST'" in runner
    assert "'ROS_AUTOMATIC_DISCOVERY_RANGE':" in runner
    assert "SetEnvironmentVariable(\n            'ROS_AUTOMATIC_DISCOVERY_RANGE'" in week6_launch
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE' not in week4_launch


def test_week6_process_environment_is_localhost_and_week6_scoped(
        monkeypatch) -> None:
    import importlib.util

    runner_path = WORKSPACE / 'scripts' / 'run_week6_navigation.py'
    monkeypatch.syspath_prepend(str(runner_path.parent))
    spec = importlib.util.spec_from_file_location(
        'run_week6_navigation_environment_test', runner_path)
    assert spec is not None and spec.loader is not None
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)

    base = {
        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET',
        runner.WEEK6_UNIQUE_NETWORK_FLOWS_ENV: 'ENABLED',
        'KEEP_ME': 'unchanged',
    }
    environment = runner.week6_runtime_environment(
        base, Path('/tmp/week6-test'), 'test-run')

    assert environment['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert runner.WEEK6_UNIQUE_NETWORK_FLOWS_ENV not in environment
    assert environment['KEEP_ME'] == 'unchanged'
    assert base['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SUBNET'
    assert runner.WEEK6_AUTOMATIC_DISCOVERY_RANGE == 'LOCALHOST'


def test_week6_rpp_inflation_scaling_matches_costmap_without_other_tuning() \
        -> None:
    import yaml

    config = yaml.safe_load(
        (PACKAGE / 'config' / 'nav2_week6_navigation.yaml').read_text(
            encoding='utf-8'))
    rpp = config['controller_server']['ros__parameters']['FollowPath']
    global_inflation = config['global_costmap']['global_costmap'][
        'ros__parameters']['inflation_layer']['cost_scaling_factor']
    local_inflation = config['local_costmap']['local_costmap'][
        'ros__parameters']['inflation_layer']['cost_scaling_factor']
    assert rpp['inflation_cost_scaling_factor'] == global_inflation == 2.0
    assert rpp['inflation_cost_scaling_factor'] == local_inflation == 2.0
    assert rpp['use_collision_detection'] is True


def test_startup_failure_classification_separates_planner_and_costmap() -> None:
    import importlib.util
    import json

    runner_path = WORKSPACE / 'scripts' / 'run_week6_navigation.py'
    import sys
    sys.path.insert(0, str(runner_path.parent))
    spec = importlib.util.spec_from_file_location(
        'run_week6_navigation_failure_class_test', runner_path)
    assert spec is not None and spec.loader is not None
    runner = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(runner)
    finally:
        sys.path.remove(str(runner_path.parent))

    readiness = {
        'ready': False,
        'actions': {
            '/compute_path_to_pose': False,
            '/compute_path_through_poses': False,
            '/follow_path': True,
            '/spin': True,
            '/backup': True,
            '/drive_on_heading': True,
            '/wait': True,
        },
        'services': {
            '/is_path_valid': True,
            '/global_costmap/clear_entirely_global_costmap': False,
            '/local_costmap/clear_entirely_local_costmap': False,
        },
        'lifecycle_states': {
            'map_server': 'active',
            'planner_server': 'active',
            'controller_server': 'active',
            'behavior_server': 'active',
            'bt_navigator': 'unconfigured',
        },
    }
    diagnostic = runner.inspect_startup_log(
        'WEEK6_STARTUP_READINESS_JSON=' +
        json.dumps(readiness) + '\n')

    assert diagnostic['failure_classes'] == [
        'PLANNER_ACTION_DISCOVERY_FAILURE',
        'COSTMAP_SERVICE_DISCOVERY_FAILURE',
    ]
    assert diagnostic['failure_is_orchestration_runtime'] is True


def test_week6_nav2_bt_activation_waits_for_lyrical_endpoints() -> None:
    launch = (PACKAGE / 'launch' /
              'week6_navigation.launch.py').read_text(encoding='utf-8')
    baseline_launch = (SIMULATION_PACKAGE / 'launch' /
                       'ares_baseline.launch.py').read_text(encoding='utf-8')
    barrier = (PACKAGE / 'ares_reliability' /
               'week6_nav2_startup_barrier.py').read_text(encoding='utf-8')
    assert 'MODE_LIFECYCLE_NODE_ORDER' in launch
    assert "'map_server', 'planner_server', 'controller_server'" in launch
    assert "node_names=['bt_navigator']" in launch
    assert "phase='upstream'" in launch
    assert "phase='bt'" in launch
    assert 'week6_nav2_lifecycle_orchestrator' in launch
    assert 'nav2_lifecycle_manager' not in launch
    assert "('nav2_bt_navigator', 'bt_navigator')" in baseline_launch
    assert "executable != 'bt_navigator'" in baseline_launch
    assert "if stage != 'nav2_week6_upstream':" in baseline_launch
    assert 'week6_nav2_startup_barrier' in launch
    for endpoint in (
            '/compute_path_to_pose',
            '/compute_path_through_poses',
            '/follow_path',
            '/spin',
            '/wait',
            '/backup',
            '/drive_on_heading',
            '/is_path_valid',
            '/global_costmap/clear_entirely_global_costmap',
            '/local_costmap/clear_entirely_local_costmap'):
        assert endpoint in barrier
    assert 'checked_before_bt_navigator_activation' in barrier
    assert '--navigation-mode' in barrier
    assert 'MODE_READINESS_MANIFESTS' in barrier
    assert "navigation_mode=mode" in launch
    assert 'first_discovery_wall_monotonic_sec' in barrier
    assert 'first_discovery_elapsed_wall_sec' in barrier
    assert 'WEEK6_NAV2_ENDPOINTS_READY=' in barrier


def test_week6_completion_diagnostics_include_pose_timing_and_rtf() -> None:
    source = (PACKAGE / 'ares_reliability' /
              'week6_navigation_mission.py').read_text(encoding='utf-8')
    for field in (
            "'goal_completion_diagnostics'",
            "'tf_map_base_at_action_result'",
            "'filtered_to_trust_fused_stamp_delta_sec'",
            "'tf_stamp_to_evaluator_ros_delta_sec'",
            "'waypoint_leg'",
            "'final_5_wall_seconds_before_action_result'",
            "'transform_age_at_action_result_sec'",
            "'odometry_age_at_action_result_sec'"):
        assert field in source


def test_week6_goal_diagnostics_preserve_tf_acceptance_source() -> None:
    source = (PACKAGE / 'ares_reliability' /
              'week6_navigation_mission.py').read_text(encoding='utf-8')
    assert 'self.tf_buffer.lookup_transform(' in source
    assert 'Time())' in source
    assert 'map_frame' in source
    assert 'base_frame' in source
    assert "'/odometry/filtered'" in source
    assert "'/odometry/trust_fused'" in source
    assert "'completion_diagnostics': completion_diagnostics" in source
    assert "'server_completion_timestamp_available': False" in source


def test_goal_checker_parameter_diagnostics_keep_response_names() -> None:
    from ares_reliability.week6_navigation_mission import parameter_values
    from rclpy.parameter import Parameter

    xy_tolerance = Parameter(
        'goal_checker.xy_goal_tolerance', value=0.20).get_parameter_value()
    stateful = Parameter(
        'goal_checker.stateful', value=False).get_parameter_value()
    response_values = [
        xy_tolerance,
        stateful,
    ]
    assert parameter_values(
        ['goal_checker.xy_goal_tolerance', 'goal_checker.stateful'],
        response_values,
    ) == {
        'goal_checker.xy_goal_tolerance': 0.20,
        'goal_checker.stateful': False,
    }


def test_week6_readiness_is_fixed_history_not_ground_truth_selection() -> None:
    mission = yaml.safe_load((
        PACKAGE / 'config' / 'week6_mission.yaml'
    ).read_text(encoding='utf-8'))
    source = (PACKAGE / 'ares_reliability' /
              'week6_navigation_mission.py').read_text(encoding='utf-8')
    assert mission['readiness_min_sim_time_sec'] == 20.0
    assert mission['readiness_min_localization_samples'] == 50
    assert mission['readiness_min_history_sim_sec'] == 5.0
    assert 'readiness_stability_max_drift_m' not in mission
    readiness_source = source.split('def wait_until_ready', 1)[1].split(
        'def _feedback_callback', 1)[0]
    assert 'latest_reference is None' not in readiness_source
    assert "'readiness_ground_truth_used': False" in readiness_source


def test_frozen_week6_mission_tolerance_is_unchanged() -> None:
    mission = yaml.safe_load((
        PACKAGE / 'config' / 'week6_mission.yaml'
    ).read_text(encoding='utf-8'))
    assert mission['waypoint_xy_tolerance_m'] == 0.30


def test_unknown_gnss_covariance_is_provisioned_at_factor_one() -> None:
    from sensor_msgs.msg import NavSatFix
    from ares_reliability.gnss_trusted_proxy import inflated_fix

    message = NavSatFix()
    message.position_covariance = [0.0] * 9
    message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

    output = inflated_fix(
        message,
        factor=1.0,
        unknown_variance_m2=1.0,
    )

    assert output.position_covariance[0] == 1.0
    assert output.position_covariance[4] == 1.0
    assert output.position_covariance[8] == 1.0
    assert (
        output.position_covariance_type
        == NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    )
