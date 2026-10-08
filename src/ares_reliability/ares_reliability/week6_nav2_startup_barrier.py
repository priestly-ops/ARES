"""Wait for Week 6 Nav2 endpoints before allowing BT navigator activation."""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState


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
from rosgraph_msgs.msg import Clock


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


BASE_UPSTREAM_LIFECYCLE_NODES = (
    'map_server',
    'planner_server',
    'controller_server',
    'behavior_server',
)

ACTION_ENDPOINTS = (
    ('/compute_path_to_pose', ComputePathToPose),
    ('/compute_path_through_poses', ComputePathThroughPoses),
    ('/follow_path', FollowPath),
    ('/spin', Spin),
    ('/wait', Wait),
    ('/backup', BackUp),
    ('/drive_on_heading', DriveOnHeading),
)

SERVICE_ENDPOINTS = (
    ('/is_path_valid', IsPathValid),
    ('/global_costmap/clear_entirely_global_costmap', ClearEntireCostmap),
    ('/local_costmap/clear_entirely_local_costmap', ClearEntireCostmap),
)

BEHAVIOR_PLUGIN_TYPES = {
    'spin': 'nav2_behaviors::Spin',
    'backup': 'nav2_behaviors::BackUp',
    'drive_on_heading': 'nav2_behaviors::DriveOnHeading',
    'wait': 'nav2_behaviors::Wait',
}
BEHAVIOR_ACTIONS = {
    '/spin': 'spin',
    '/backup': 'backup',
    '/drive_on_heading': 'drive_on_heading',
    '/wait': 'wait',
}

MODE_READINESS_MANIFESTS = {
    mode: {
        'required_lifecycle_nodes': (
            BASE_UPSTREAM_LIFECYCLE_NODES +
            (('amcl',) if mode == 'baseline' else ())),
        'action_endpoints': ACTION_ENDPOINTS,
        'service_endpoints': SERVICE_ENDPOINTS,
    }
    for mode in ('baseline', 'unprotected', 'protected')
}


class Nav2StartupBarrier(Node):
    """Report whether upstream lifecycle and default BT endpoints are ready."""

    def __init__(self, navigation_mode: str = 'protected') -> None:
        super().__init__('week6_nav2_startup_barrier')
        try:
            manifest = MODE_READINESS_MANIFESTS[navigation_mode]
        except KeyError as error:
            raise ValueError(
                f'unknown navigation mode {navigation_mode!r}') from error
        self.navigation_mode = navigation_mode
        self.required_lifecycle_nodes = manifest['required_lifecycle_nodes']
        self.required_action_endpoints = manifest['action_endpoints']
        self.required_service_endpoints = manifest['service_endpoints']
        self.action_clients = {
            name: ActionClient(self, action_type, name)
            for name, action_type in self.required_action_endpoints
        }
        self.service_clients = {
            name: self.create_client(service_type, name)
            for name, service_type in self.required_service_endpoints
        }
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in (*self.required_lifecycle_nodes, 'bt_navigator')
        }
        self.behavior_parameters_client = self.create_client(
            GetParameters, '/behavior_server/get_parameters')
        self.controller_parameters_client = self.create_client(
            GetParameters, '/controller_server/get_parameters')
        self.started_wall_monotonic_sec = time.monotonic()
        self.first_discovery: dict[str, float] = {}
        self.first_discovery_ros_time: dict[str, float | None] = {}
        self.latest_ros_time_sec: float | None = None
        self.behavior_plugin_evidence: dict[str, Any] | None = None
        self.controller_parameter_evidence: dict[str, Any] | None = None
        self.last_behavior_parameter_query = 0.0
        self.last_controller_parameter_query = 0.0
        self.readiness_phases: dict[str, dict[str, float | None]] = {
            'upstream_lifecycle_active': {'wall_monotonic_sec': None},
            'behavior_actions_ready': {'wall_monotonic_sec': None},
            'planner_controller_services_ready': {
                'wall_monotonic_sec': None,
            },
        }
        self.create_subscription(
            Clock, '/clock', self._clock_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT))

    def _clock_callback(self, message: Clock) -> None:
        self.latest_ros_time_sec = (
            float(message.clock.sec) +
            float(message.clock.nanosec) / 1_000_000_000.0)

    def _query_behavior_plugins(self, lifecycle_state: str) -> None:
        now = time.monotonic()
        if (now - self.last_behavior_parameter_query < 1.0 or
                not self.behavior_parameters_client.service_is_ready()):
            return
        self.last_behavior_parameter_query = now
        request = GetParameters.Request()
        parameter_names = ['behavior_plugins'] + [
            f'{name}.plugin' for name in BEHAVIOR_PLUGIN_TYPES
        ]
        request.names = parameter_names
        future = self.behavior_parameters_client.call_async(request)
        deadline = now + 0.5
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None:
            return
        values = future.result().values
        if len(values) != len(parameter_names):
            return
        configured_names = (
            list(values[0].string_array_value)
            if values[0].type == ParameterType.PARAMETER_STRING_ARRAY
            else [])
        plugins: dict[str, dict[str, Any]] = {}
        for index, (plugin_name, expected_type) in enumerate(
                BEHAVIOR_PLUGIN_TYPES.items(), start=1):
            value = values[index]
            actual_type = (
                value.string_value
                if value.type == ParameterType.PARAMETER_STRING else None)
            configured = (
                plugin_name in configured_names and
                actual_type == expected_type)
            plugins[plugin_name] = {
                'configured': configured,
                'listed_in_behavior_plugins': plugin_name in configured_names,
                'configured_plugin_type': actual_type,
                'expected_plugin_type': expected_type,
                'activated': configured and lifecycle_state == 'active',
            }
        self.behavior_plugin_evidence = {
            'lifecycle_state': lifecycle_state,
            'plugins': plugins,
            'parameter_query_succeeded': True,
        }

    def _query_controller_parameters(self) -> None:
        now = time.monotonic()
        if (now - self.last_controller_parameter_query < 1.0 or
                not self.controller_parameters_client.service_is_ready()):
            return
        self.last_controller_parameter_query = now
        names = (
            'use_realtime_priority',
            'FollowPath.inflation_cost_scaling_factor',
            'FollowPath.use_cost_regulated_linear_velocity_scaling',
        )
        request = GetParameters.Request()
        request.names = list(names)
        future = self.controller_parameters_client.call_async(request)
        deadline = now + 0.5
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None:
            return
        values = future.result().values
        if len(values) != len(names):
            return
        parameter_values: dict[str, Any] = {}
        for name, value in zip(names, values):
            if value.type == ParameterType.PARAMETER_BOOL:
                parsed_value: Any = value.bool_value
            elif value.type == ParameterType.PARAMETER_DOUBLE:
                parsed_value = value.double_value
            elif value.type == ParameterType.PARAMETER_INTEGER:
                parsed_value = value.integer_value
            else:
                parsed_value = None
            parameter_values[name] = {
                'type': value.type,
                'value': parsed_value,
            }
        self.controller_parameter_evidence = {
            'parameter_query_succeeded': True,
            'values': parameter_values,
        }

    def _relevant_topic_types(self) -> dict[str, list[str]]:
        patterns = (
            'lookahead', 'arc', 'costmap', 'scan', 'cmd_vel', 'odometry',
            'plan', 'rosout',
        )
        return {
            name: sorted(types)
            for name, types in self.get_topic_names_and_types()
            if any(pattern in name.lower() for pattern in patterns)
        }

    def _action_graph_evidence(self) -> dict[str, Any]:
        topics = self.get_topic_names_and_types()
        topic_types = {
            name: types for name, types in topics
        }
        service_types = {
            name: types
            for name, types in self.get_service_names_and_types()
        }
        action_evidence: dict[str, Any] = {}
        for endpoint, action_type in self.required_action_endpoints:
            expected_base = (
                f'nav2_msgs/action/{action_type.__name__}')
            expected_send_goal_type = f'{expected_base}_SendGoal'
            exact_topics = {
                name: types for name, types in topic_types.items()
                if name.startswith(endpoint + '/_action/')
            }
            exact_services = {
                name: types for name, types in service_types.items()
                if name.startswith(endpoint + '/_action/')
            }
            candidate_names = sorted({
                name.rsplit('/_action/', 1)[0]
                for name in (*topic_types, *service_types)
                if '/_action/' in name and
                name.rsplit('/_action/', 1)[0].endswith(
                    '/' + endpoint.rsplit('/', 1)[-1])
            })
            action_evidence[endpoint] = {
                'action_type': expected_base,
                'expected_send_goal_type': expected_send_goal_type,
                'discovered_topics': exact_topics,
                'discovered_services': exact_services,
                'candidate_action_names': candidate_names,
                'send_goal_type_matches': (
                    expected_send_goal_type in
                    exact_services.get(
                        endpoint + '/_action/send_goal', [])),
            }
        return action_evidence

    def inspect(self) -> dict[str, Any]:
        observed_wall = time.monotonic()
        lifecycle_states: dict[str, str] = {}
        for name, client in self.lifecycle_clients.items():
            if not client.service_is_ready():
                lifecycle_states[name] = 'SERVICE_UNAVAILABLE'
                continue
            future = client.call_async(GetState.Request())
            deadline = time.monotonic() + 0.5
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not future.done():
                lifecycle_states[name] = 'STATE_QUERY_TIMEOUT'
                continue
            response = future.result()
            lifecycle_states[name] = (
                response.current_state.label if response is not None
                else 'STATE_QUERY_FAILED')
        behavior_lifecycle_state = lifecycle_states.get(
            'behavior_server', 'SERVICE_UNAVAILABLE')
        self._query_behavior_plugins(behavior_lifecycle_state)
        self._query_controller_parameters()

        actions = {
            name: client.server_is_ready()
            for name, client in self.action_clients.items()
        }
        if self.behavior_plugin_evidence is not None:
            for endpoint, plugin_name in BEHAVIOR_ACTIONS.items():
                plugin = self.behavior_plugin_evidence[
                    'plugins'][plugin_name]
                plugin['activated'] = (
                    plugin['configured'] and
                    behavior_lifecycle_state == 'active' and
                    actions.get(endpoint, False))
        services = {
            name: client.service_is_ready()
            for name, client in self.service_clients.items()
        }
        endpoint_states = {
            **{
                name: (ready, 'action')
                for name, ready in actions.items()
            },
            **{
                name: (ready, 'service')
                for name, ready in services.items()
            },
        }
        for name, (ready, _kind) in endpoint_states.items():
            if ready:
                self.first_discovery.setdefault(
                    name, observed_wall)
                if (name not in self.first_discovery_ros_time or
                        (self.first_discovery_ros_time[name] is None and
                         self.latest_ros_time_sec is not None)):
                    self.first_discovery_ros_time[name] = (
                        self.latest_ros_time_sec)
        action_graph = self._action_graph_evidence()
        behavior_actions_ready = all(
            actions.get(endpoint, False)
            for endpoint in BEHAVIOR_ACTIONS)
        behavior_plugins_ready = bool(
            self.behavior_plugin_evidence and
            all(
                plugin['configured'] and plugin['activated']
                for plugin in self.behavior_plugin_evidence[
                    'plugins'].values()))
        if (all(
                lifecycle_states.get(name) == 'active'
                for name in self.required_lifecycle_nodes) and
                self.readiness_phases[
                    'upstream_lifecycle_active'
                ]['wall_monotonic_sec'] is None):
            self.readiness_phases[
                'upstream_lifecycle_active'
            ]['wall_monotonic_sec'] = observed_wall
        behavior_phase_was_ready = (
            self.readiness_phases['behavior_actions_ready'][
                'wall_monotonic_sec'] is not None)
        if (behavior_actions_ready and behavior_plugins_ready and
                self.readiness_phases[
                    'behavior_actions_ready'
                ]['wall_monotonic_sec'] is None):
            self.readiness_phases[
                'behavior_actions_ready'
            ]['wall_monotonic_sec'] = observed_wall
        planner_controller_ready = (
            all(
                ready for endpoint, ready in actions.items()
                if endpoint not in BEHAVIOR_ACTIONS) and
            all(services.values()))
        if (behavior_phase_was_ready and planner_controller_ready and
                self.readiness_phases[
                    'planner_controller_services_ready'
                ]['wall_monotonic_sec'] is None):
            self.readiness_phases[
                'planner_controller_services_ready'
            ]['wall_monotonic_sec'] = observed_wall
        behavior_endpoint_diagnostics = {
            endpoint: {
                'behavior_server_lifecycle_state':
                    behavior_lifecycle_state,
                'endpoint_discovered': actions.get(endpoint, False),
                'first_discovery_wall_monotonic_sec':
                    self.first_discovery.get(endpoint),
                'first_discovery_ros_time_sec':
                    self.first_discovery_ros_time.get(endpoint),
                'plugin_configured': (
                    None if self.behavior_plugin_evidence is None else
                    self.behavior_plugin_evidence['plugins'][
                        plugin_name]['configured']),
                'plugin_activated': (
                    None if self.behavior_plugin_evidence is None else
                    self.behavior_plugin_evidence['plugins'][
                        plugin_name]['activated']),
                'action_graph': action_graph[endpoint],
            }
            for endpoint, plugin_name in BEHAVIOR_ACTIONS.items()
        }
        return {
            'navigation_mode': self.navigation_mode,
            'required_action_endpoints': [
                name for name, _action_type in self.required_action_endpoints
            ],
            'required_service_endpoints': [
                name for name, _service_type in self.required_service_endpoints
            ],
            'lifecycle_states': lifecycle_states,
            'actions': actions,
            'services': services,
            'behavior_server_plugins':
                self.behavior_plugin_evidence or {
                    'lifecycle_state': behavior_lifecycle_state,
                    'parameter_query_succeeded': False,
                    'plugins': {
                        name: {
                            'configured': None,
                            'activated': None,
                            'expected_plugin_type': plugin_type,
                        }
                        for name, plugin_type in
                        BEHAVIOR_PLUGIN_TYPES.items()
                    },
                },
            'behavior_action_diagnostics':
                behavior_endpoint_diagnostics,
            'controller_parameter_diagnostics':
                self.controller_parameter_evidence or {
                    'parameter_query_succeeded': False,
                    'values': {},
                },
            'action_graph_diagnostics': action_graph,
            'relevant_topic_types': self._relevant_topic_types(),
            'readiness_phases': self.readiness_phases,
            'endpoint_discovery': {
                name: {
                    'kind': kind,
                    'discovered': ready,
                    'first_discovery_wall_monotonic_sec': (
                        self.first_discovery.get(name)),
                    'first_discovery_elapsed_wall_sec': (
                        None if name not in self.first_discovery else
                        self.first_discovery[name] -
                        self.started_wall_monotonic_sec),
                    'first_discovery_ros_time_sec':
                        self.first_discovery_ros_time.get(name),
                }
                for name, (ready, kind) in endpoint_states.items()
            },
        }


def wait_for_readiness(timeout_wall_sec: float,
                       navigation_mode: str = 'protected') -> dict[str, Any]:
    rclpy.init()
    node = Nav2StartupBarrier(navigation_mode=navigation_mode)
    start = time.monotonic()
    deadline = start + timeout_wall_sec
    latest: dict[str, Any] = {}
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            latest = node.inspect()
            upstream_active = all(
                latest['lifecycle_states'].get(name) == 'active'
                for name in node.required_lifecycle_nodes)
            bt_not_activated = latest['lifecycle_states'].get(
                'bt_navigator') in ('unconfigured', 'inactive')
            behavior_plugins = latest['behavior_server_plugins']['plugins']
            behavior_phase_ready = (
                all(
                    latest['actions'].get(endpoint, False)
                    for endpoint in BEHAVIOR_ACTIONS) and
                all(
                    plugin.get('configured') is True and
                    plugin.get('activated') is True
                    for plugin in behavior_plugins.values()))
            other_endpoints_ready = (
                all(
                    ready for endpoint, ready in latest['actions'].items()
                    if endpoint not in BEHAVIOR_ACTIONS) and
                all(latest['services'].values()))
            if (upstream_active and bt_not_activated and
                    behavior_phase_ready and other_endpoints_ready and
                    latest['readiness_phases'][
                        'planner_controller_services_ready'
                    ]['wall_monotonic_sec'] is not None):
                result = {
                    **latest,
                    'ready': True,
                    'timeout_wall_sec': timeout_wall_sec,
                    'elapsed_wall_sec': time.monotonic() - start,
                    'checked_before_bt_navigator_activation': True,
                    'behavior_actions_ready_before_planner_services': (
                        latest['readiness_phases'][
                            'behavior_actions_ready'
                        ]['wall_monotonic_sec'] is not None and
                        latest['readiness_phases'][
                            'planner_controller_services_ready'
                        ]['wall_monotonic_sec'] is not None and
                        latest['readiness_phases'][
                            'behavior_actions_ready'
                        ]['wall_monotonic_sec'] <=
                        latest['readiness_phases'][
                            'planner_controller_services_ready'
                        ]['wall_monotonic_sec']),
                    'required_lifecycle_nodes':
                        list(node.required_lifecycle_nodes),
                }
                node.get_logger().info(
                    'WEEK6_NAV2_ENDPOINTS_READY=' +
                    json.dumps(result, sort_keys=True))
                return result
        return {
            **latest,
            'ready': False,
            'timeout_wall_sec': timeout_wall_sec,
            'elapsed_wall_sec': time.monotonic() - start,
            'checked_before_bt_navigator_activation': (
                latest.get('lifecycle_states', {}).get('bt_navigator')
                in ('unconfigured', 'inactive')),
            'missing_actions': [
                name for name, ready in latest.get('actions', {}).items()
                if not ready
            ],
            'missing_services': [
                name for name, ready in latest.get('services', {}).items()
                if not ready
            ],
            'inactive_upstream_nodes': [
                name for name in node.required_lifecycle_nodes
                if latest.get('lifecycle_states', {}).get(name) != 'active'
            ],
            'behavior_actions_ready': all(
                latest.get('actions', {}).get(endpoint, False)
                for endpoint in BEHAVIOR_ACTIONS),
            'behavior_plugins_ready': all(
                plugin.get('configured') is True and
                plugin.get('activated') is True
                for plugin in latest.get(
                    'behavior_server_plugins', {}).get(
                        'plugins', {}).values()),
            'failure_phase': _failed_phase(latest, node),
            'failure_classes': _failure_classes(latest, node),
            'required_lifecycle_nodes':
                list(node.required_lifecycle_nodes),
        }
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-wall-sec', type=float, default=60.0)
    parser.add_argument(
        '--navigation-mode',
        choices=tuple(MODE_READINESS_MANIFESTS),
        default='protected')
    parsed, _ros_args = parser.parse_known_args(args)
    if parsed.timeout_wall_sec <= 0.0:
        parser.error('--timeout-wall-sec must be positive')

    result = wait_for_readiness(
        parsed.timeout_wall_sec, navigation_mode=parsed.navigation_mode)
    print(
        'WEEK6_STARTUP_READINESS_JSON=' +
        json.dumps(result, sort_keys=True),
        flush=True,
    )
    raise SystemExit(0 if result['ready'] else 1)


def _failed_phase(result: dict[str, Any], node: Nav2StartupBarrier) -> str:
    if not all(
            result.get('lifecycle_states', {}).get(name) == 'active'
            for name in node.required_lifecycle_nodes):
        return 'upstream_lifecycle_activation'
    if not result.get('behavior_actions_ready'):
        return 'behavior_action_registration_or_discovery'
    if not result.get('behavior_plugins_ready'):
        return 'behavior_plugin_configuration_or_activation'
    return 'planner_controller_services'


def _failure_classes(result: dict[str, Any],
                     node: Nav2StartupBarrier) -> list[str]:
    missing_actions = [
        name for name, ready in result.get('actions', {}).items()
        if not ready
    ]
    missing_services = [
        name for name, ready in result.get('services', {}).items()
        if not ready
    ]
    classes: list[str] = []
    if any(name in missing_actions for name in (
            '/compute_path_to_pose', '/compute_path_through_poses')):
        classes.append('PLANNER_ACTION_DISCOVERY_FAILURE')
    if '/is_path_valid' in missing_services:
        classes.append('PLANNER_SERVICE_DISCOVERY_FAILURE')
    if any(name in missing_services for name in (
            '/global_costmap/clear_entirely_global_costmap',
            '/local_costmap/clear_entirely_local_costmap')):
        classes.append('COSTMAP_SERVICE_DISCOVERY_FAILURE')
    if '/follow_path' in missing_actions:
        classes.append('CONTROLLER_ACTION_DISCOVERY_FAILURE')
    if any(name in missing_actions for name in (
            '/spin', '/backup', '/drive_on_heading', '/wait')):
        classes.append('BEHAVIOR_ACTION_DISCOVERY_FAILURE')
    lifecycle_states = result.get('lifecycle_states', {})
    if any(lifecycle_states.get(name) != 'active'
           for name in node.required_lifecycle_nodes):
        classes.append('LIFECYCLE_ACTIVATION_FAILURE')
    bt_state = lifecycle_states.get('bt_navigator')
    upstream_ready = all(
        lifecycle_states.get(name) == 'active'
        for name in node.required_lifecycle_nodes)
    all_endpoints_ready = not (
        missing_actions or missing_services)
    if (bt_state != 'active' and upstream_ready and all_endpoints_ready):
        classes.append('BT_NAVIGATOR_ACTIVATION_FAILURE')
    return classes


if __name__ == '__main__':
    main()
