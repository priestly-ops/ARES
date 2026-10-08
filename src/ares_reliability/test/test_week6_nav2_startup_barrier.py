"""Tests for Week 6 Nav2 startup endpoint coverage."""

from pathlib import Path

from ares_reliability.week6_nav2_startup_barrier import (
    ACTION_ENDPOINTS,
    BASE_UPSTREAM_LIFECYCLE_NODES,
    BEHAVIOR_ACTIONS,
    BEHAVIOR_PLUGIN_TYPES,
    MODE_READINESS_MANIFESTS,
    SERVICE_ENDPOINTS,
    _failure_classes,
)


from nav2_msgs.action import (
    BackUp,
    ComputePathThroughPoses,
    ComputePathToPose,
    DriveOnHeading,
    FollowPath,
    Spin,
    Wait,
)
from nav2_msgs.srv import ClearEntireCostmap, IsPathValid


def test_required_bt_and_behavior_actions_are_discovered() -> None:
    assert dict(ACTION_ENDPOINTS) == {
        '/compute_path_to_pose': ComputePathToPose,
        '/compute_path_through_poses': ComputePathThroughPoses,
        '/follow_path': FollowPath,
        '/spin': Spin,
        '/wait': Wait,
        '/backup': BackUp,
        '/drive_on_heading': DriveOnHeading,
    }


def test_frozen_bt_costmap_services_are_discovered() -> None:
    assert dict(SERVICE_ENDPOINTS) == {
        '/is_path_valid': IsPathValid,
        '/global_costmap/clear_entirely_global_costmap':
            ClearEntireCostmap,
        '/local_costmap/clear_entirely_local_costmap':
            ClearEntireCostmap,
    }


def test_mode_manifests_preserve_frozen_bt_endpoint_coverage() -> None:
    for mode in ('baseline', 'unprotected', 'protected'):
        manifest = MODE_READINESS_MANIFESTS[mode]
        assert manifest['action_endpoints'] == ACTION_ENDPOINTS
        assert manifest['service_endpoints'] == SERVICE_ENDPOINTS


def test_mode_manifests_match_lifecycle_topology() -> None:
    assert MODE_READINESS_MANIFESTS['baseline'][
        'required_lifecycle_nodes'] == (
            *BASE_UPSTREAM_LIFECYCLE_NODES, 'amcl')
    for mode in ('unprotected', 'protected'):
        assert MODE_READINESS_MANIFESTS[mode][
            'required_lifecycle_nodes'] == BASE_UPSTREAM_LIFECYCLE_NODES


def test_behavior_plugins_and_actions_have_matching_nav2_types() -> None:
    assert BEHAVIOR_ACTIONS == {
        '/spin': 'spin',
        '/backup': 'backup',
        '/drive_on_heading': 'drive_on_heading',
        '/wait': 'wait',
    }
    assert BEHAVIOR_PLUGIN_TYPES == {
        'spin': 'nav2_behaviors::Spin',
        'backup': 'nav2_behaviors::BackUp',
        'drive_on_heading': 'nav2_behaviors::DriveOnHeading',
        'wait': 'nav2_behaviors::Wait',
    }
    assert dict(ACTION_ENDPOINTS) == {
        '/compute_path_to_pose': ComputePathToPose,
        '/compute_path_through_poses': ComputePathThroughPoses,
        '/follow_path': FollowPath,
        '/spin': Spin,
        '/wait': Wait,
        '/backup': BackUp,
        '/drive_on_heading': DriveOnHeading,
    }


def test_barrier_sequences_behavior_actions_before_planner_services() -> None:
    source = (Path(__file__).parents[1] /
              'ares_reliability' / 'week6_nav2_startup_barrier.py')
    text = source.read_text(encoding='utf-8')
    assert 'behavior_actions_ready_before_planner_services' in text
    assert 'behavior_server_plugins' in text
    assert 'first_discovery_ros_time_sec' in text
    assert 'action_graph_diagnostics' in text


def test_provider_failure_classes_are_distinct_and_bt_remains_gated() -> None:
    class Barrier:
        required_lifecycle_nodes = (
            'map_server', 'planner_server', 'controller_server',
            'behavior_server')

    result = {
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

    assert _failure_classes(result, Barrier()) == [
        'PLANNER_ACTION_DISCOVERY_FAILURE',
        'COSTMAP_SERVICE_DISCOVERY_FAILURE',
    ]


def test_live_controller_parameter_diagnostics_query_scaling_and_realtime() \
        -> None:
    source = (Path(__file__).parents[1] /
              'ares_reliability' / 'week6_nav2_startup_barrier.py')
    text = source.read_text(encoding='utf-8')
    assert "'/controller_server/get_parameters'" in text
    assert "'FollowPath.inflation_cost_scaling_factor'" in text
    assert "'use_realtime_priority'" in text
    assert "'controller_parameter_diagnostics'" in text
