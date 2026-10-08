"""Week 6-only lifecycle orchestration resilient to lost service replies."""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Protocol

from lifecycle_msgs.msg import State, Transition, TransitionEvent
from lifecycle_msgs.srv import ChangeState, GetState

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


CONFIGURE = {
    'name': 'CONFIGURE',
    'transition_id': Transition.TRANSITION_CONFIGURE,
    'source_state_id': State.PRIMARY_STATE_UNCONFIGURED,
    'source_state': 'unconfigured',
    'target_state_id': State.PRIMARY_STATE_INACTIVE,
    'target_state': 'inactive',
}
ACTIVATE = {
    'name': 'ACTIVATE',
    'transition_id': Transition.TRANSITION_ACTIVATE,
    'source_state_id': State.PRIMARY_STATE_INACTIVE,
    'source_state': 'inactive',
    'target_state_id': State.PRIMARY_STATE_ACTIVE,
    'target_state': 'active',
}
PRIMARY_STATE_IDS = {
    State.PRIMARY_STATE_UNCONFIGURED,
    State.PRIMARY_STATE_INACTIVE,
    State.PRIMARY_STATE_ACTIVE,
    State.PRIMARY_STATE_FINALIZED,
}
LOST_RESPONSE_STATUSES = {'TIMEOUT', 'ERROR'}


@dataclass(frozen=True)
class ServiceCallOutcome:
    """Bounded lifecycle service-call result."""

    status: str
    success: bool | None = None
    failure_text: str | None = None


class LifecycleBackend(Protocol):
    """Minimal backend used by both ROS and deterministic unit tests."""

    def monotonic(self) -> float: ...

    def wall_time(self) -> str: ...

    def sleep(self, duration_sec: float) -> None: ...

    def subscribe_transition_events(self, node_name: str) -> None: ...

    def wait_change_state(self, node_name: str, timeout_sec: float) -> bool:
        ...

    def wait_get_state(self, node_name: str, timeout_sec: float) -> bool:
        ...

    def get_state(
            self, node_name: str,
            timeout_sec: float) -> tuple[int, str]: ...

    def change_state(
            self, node_name: str, transition_id: int,
            timeout_sec: float) -> ServiceCallOutcome: ...

    def take_transition_events(self, node_name: str) -> list[dict[str, Any]]:
        ...


def _latest_primary_event(
        events: list[dict[str, Any]]) -> dict[str, Any] | None:
    primary = [
        event for event in events
        if event.get('goal_state_id') in PRIMARY_STATE_IDS
    ]
    return primary[-1] if primary else None


class RobustLifecycleOrchestrator:
    """Apply lifecycle transitions with positive, independent confirmation."""

    def __init__(
            self, backend: LifecycleBackend, *,
            service_availability_timeout_sec: float = 20.0,
            response_timeout_sec: float = 5.0,
            state_query_timeout_sec: float = 1.0,
            initial_state_deadline_sec: float = 3.0,
            operation_deadline_sec: float = 12.0,
            retry_interval_sec: float = 0.1) -> None:
        self.backend = backend
        self.service_availability_timeout_sec = (
            service_availability_timeout_sec)
        self.response_timeout_sec = response_timeout_sec
        self.state_query_timeout_sec = state_query_timeout_sec
        self.initial_state_deadline_sec = initial_state_deadline_sec
        self.operation_deadline_sec = operation_deadline_sec
        self.retry_interval_sec = retry_interval_sec

    def _new_evidence(
            self, node_name: str,
            transition: dict[str, Any]) -> dict[str, Any]:
        return {
            'node': node_name,
            'requested_transition': transition['name'],
            'transition_id': transition['transition_id'],
            'target_state': transition['target_state'],
            'request_time': None,
            'service_response_status': 'NOT_SENT',
            'response_timeout_or_failure_text': None,
            'transition_event_observations': [],
            'get_state_observations': [],
            'final_confirmed_state': None,
            'confirmation_method': None,
            'elapsed_wall_time_sec': None,
            'retries': 0,
            'success': False,
        }

    def _finish(
            self, evidence: dict[str, Any], started: float, *,
            success: bool, final_state: str | None = None,
            method: str | None = None,
            failure: str | None = None) -> dict[str, Any]:
        evidence['success'] = success
        evidence['final_confirmed_state'] = final_state
        evidence['confirmation_method'] = method
        evidence['elapsed_wall_time_sec'] = round(
            self.backend.monotonic() - started, 6)
        if failure is not None:
            evidence['failure'] = failure
        return evidence

    def _observe_state(
            self, node_name: str, evidence: dict[str, Any], *,
            phase: str, deadline: float,
            target_state_id: int | None = None) -> tuple[int, str] | None:
        attempts = 0
        last_state: tuple[int, str] | None = None
        while self.backend.monotonic() < deadline:
            attempts += 1
            remaining = max(0.0, deadline - self.backend.monotonic())
            try:
                state_id, label = self.backend.get_state(
                    node_name,
                    min(self.state_query_timeout_sec, remaining))
                last_state = (state_id, label)
                evidence['get_state_observations'].append({
                    'phase': phase,
                    'observation_time': self.backend.wall_time(),
                    'state_id': state_id,
                    'state': label,
                    'status': 'RESPONSE',
                })
                if target_state_id is None or state_id == target_state_id:
                    break
            except (RuntimeError, TimeoutError) as error:
                evidence['get_state_observations'].append({
                    'phase': phase,
                    'observation_time': self.backend.wall_time(),
                    'state_id': None,
                    'state': None,
                    'status': 'TIMEOUT_OR_ERROR',
                    'error': str(error),
                })
            if self.backend.monotonic() < deadline:
                self.backend.sleep(self.retry_interval_sec)
        if phase == 'post_request':
            evidence['retries'] = max(0, attempts - 1)
        return last_state

    def transition(
            self, node_name: str,
            transition: dict[str, Any]) -> dict[str, Any]:
        """Request a transition and require positive target-state evidence."""
        started = self.backend.monotonic()
        evidence = self._new_evidence(node_name, transition)
        self.backend.subscribe_transition_events(node_name)

        if not self.backend.wait_change_state(
                node_name, self.service_availability_timeout_sec):
            return self._finish(
                evidence, started, success=False,
                failure='change_state service unavailable')
        if not self.backend.wait_get_state(
                node_name, self.service_availability_timeout_sec):
            return self._finish(
                evidence, started, success=False,
                failure='get_state service unavailable')

        initial = self._observe_state(
            node_name, evidence, phase='pre_request',
            deadline=(self.backend.monotonic() +
                      self.initial_state_deadline_sec))
        if initial is None:
            return self._finish(
                evidence, started, success=False,
                failure='initial lifecycle state could not be confirmed')
        initial_id, initial_label = initial
        if initial_id == transition['target_state_id']:
            evidence['service_response_status'] = (
                'NOT_SENT_ALREADY_AT_TARGET')
            return self._finish(
                evidence, started, success=True, final_state=initial_label,
                method='GET_STATE_ALREADY_AT_TARGET')
        if initial_id != transition['source_state_id']:
            evidence['service_response_status'] = (
                'NOT_SENT_INVALID_SOURCE_STATE')
            return self._finish(
                evidence, started, success=False, final_state=initial_label,
                failure=(
                    f"expected source state {transition['source_state']}; "
                    f'confirmed {initial_label}'))

        pre_request_events = self.backend.take_transition_events(node_name)
        for event in pre_request_events:
            event['phase'] = 'pre_request'
        evidence['transition_event_observations'].extend(
            pre_request_events)
        evidence['request_time'] = self.backend.wall_time()
        outcome = self.backend.change_state(
            node_name, transition['transition_id'],
            self.response_timeout_sec)
        evidence['service_response_status'] = outcome.status
        evidence['response_timeout_or_failure_text'] = outcome.failure_text
        post_request_events = self.backend.take_transition_events(node_name)
        for event in post_request_events:
            event['phase'] = 'post_request'
        evidence['transition_event_observations'].extend(
            post_request_events)
        latest_event = _latest_primary_event(post_request_events)
        event_state_id = (
            latest_event.get('goal_state_id')
            if latest_event is not None else None)
        operation_deadline = (
            self.backend.monotonic() + self.operation_deadline_sec)

        if outcome.status == 'RESPONSE' and outcome.success is False:
            state = self._observe_state(
                node_name, evidence, phase='post_request',
                deadline=min(
                    operation_deadline,
                    self.backend.monotonic() +
                    self.state_query_timeout_sec))
            late_events = self.backend.take_transition_events(node_name)
            for event in late_events:
                event['phase'] = 'post_request'
            evidence['transition_event_observations'].extend(late_events)
            final_label = state[1] if state is not None else None
            return self._finish(
                evidence, started, success=False,
                final_state=final_label,
                failure='change_state returned success=false')

        if outcome.status == 'RESPONSE' and outcome.success is True:
            state = self._observe_state(
                node_name, evidence, phase='post_request',
                deadline=operation_deadline,
                target_state_id=transition['target_state_id'])
            late_events = self.backend.take_transition_events(node_name)
            for event in late_events:
                event['phase'] = 'post_request'
            evidence['transition_event_observations'].extend(late_events)
            if late_events:
                post_request_events.extend(late_events)
                latest_event = _latest_primary_event(post_request_events)
                event_state_id = (
                    latest_event.get('goal_state_id')
                    if latest_event is not None else None)
            if state is not None and state[0] == transition['target_state_id']:
                if (event_state_id is not None and
                        event_state_id != transition['target_state_id']):
                    return self._finish(
                        evidence, started, success=False,
                        final_state=state[1],
                        failure='contradictory transition event and state')
                return self._finish(
                    evidence, started, success=True,
                    final_state=state[1],
                    method='SERVICE_RESPONSE_AND_STATE')
            return self._finish(
                evidence, started, success=False,
                final_state=state[1] if state is not None else None,
                failure='service succeeded but target state was not confirmed')

        if outcome.status not in LOST_RESPONSE_STATUSES:
            return self._finish(
                evidence, started, success=False,
                failure=f'unsupported service outcome {outcome.status}')

        if event_state_id == transition['target_state_id']:
            state = self._observe_state(
                node_name, evidence, phase='post_request',
                deadline=min(
                    operation_deadline,
                    self.backend.monotonic() +
                    self.state_query_timeout_sec))
            late_events = self.backend.take_transition_events(node_name)
            for event in late_events:
                event['phase'] = 'post_request'
            evidence['transition_event_observations'].extend(late_events)
            if late_events:
                post_request_events.extend(late_events)
                latest_event = _latest_primary_event(post_request_events)
                event_state_id = (
                    latest_event.get('goal_state_id')
                    if latest_event is not None else None)
            if event_state_id != transition['target_state_id']:
                return self._finish(
                    evidence, started, success=False,
                    final_state=state[1] if state is not None else None,
                    failure='contradictory transition event evidence')
            if (state is not None and state[0] in PRIMARY_STATE_IDS and
                    state[0] != transition['target_state_id']):
                return self._finish(
                    evidence, started, success=False,
                    final_state=state[1],
                    failure='contradictory transition event and state')
            return self._finish(
                evidence, started, success=True,
                final_state=transition['target_state'],
                method='TRANSITION_EVENT_AFTER_LOST_RESPONSE')

        state = self._observe_state(
            node_name, evidence, phase='post_request',
            deadline=operation_deadline,
            target_state_id=transition['target_state_id'])
        late_events = self.backend.take_transition_events(node_name)
        for event in late_events:
            event['phase'] = 'post_request'
        evidence['transition_event_observations'].extend(late_events)
        if late_events:
            post_request_events.extend(late_events)
            latest_event = _latest_primary_event(post_request_events)
            event_state_id = (
                latest_event.get('goal_state_id')
                if latest_event is not None else None)
        if state is not None and state[0] == transition['target_state_id']:
            if (event_state_id is not None and
                    event_state_id != transition['target_state_id']):
                return self._finish(
                    evidence, started, success=False,
                    final_state=state[1],
                    failure='contradictory transition event and state')
            return self._finish(
                evidence, started, success=True, final_state=state[1],
                method='GET_STATE_AFTER_LOST_RESPONSE')
        if event_state_id == transition['target_state_id']:
            if (state is not None and state[0] in PRIMARY_STATE_IDS and
                    state[0] != transition['target_state_id']):
                return self._finish(
                    evidence, started, success=False,
                    final_state=state[1],
                    failure='contradictory transition event and state')
            return self._finish(
                evidence, started, success=True,
                final_state=transition['target_state'],
                method='TRANSITION_EVENT_AFTER_LOST_RESPONSE')
        return self._finish(
            evidence, started, success=False,
            final_state=state[1] if state is not None else None,
            failure='lost response and target state could not be confirmed')


class RosLifecycleBackend(Node):
    """ROS implementation of the lifecycle orchestration backend."""

    def __init__(self, node_names: list[str]) -> None:
        super().__init__('week6_nav2_lifecycle_orchestrator')
        self.change_clients = {
            name: self.create_client(
                ChangeState, f'/{name}/change_state')
            for name in node_names
        }
        self.state_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in node_names
        }
        self.events: dict[str, list[dict[str, Any]]] = {
            name: [] for name in node_names
        }
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.event_subscriptions = {
            name: self.create_subscription(
                TransitionEvent, f'/{name}/transition_event',
                self._event_callback(name), qos)
            for name in node_names
        }

    def _event_callback(self, node_name: str):
        def callback(message: TransitionEvent) -> None:
            self.events[node_name].append({
                'observation_time': self.wall_time(),
                'transition_id': int(message.transition.id),
                'transition_label': message.transition.label,
                'start_state_id': int(message.start_state.id),
                'start_state': message.start_state.label,
                'goal_state_id': int(message.goal_state.id),
                'goal_state': message.goal_state.label,
            })
        return callback

    def monotonic(self) -> float:
        return time.monotonic()

    def wall_time(self) -> str:
        return datetime.now(timezone.utc).isoformat()

    def sleep(self, duration_sec: float) -> None:
        deadline = self.monotonic() + duration_sec
        while self.monotonic() < deadline:
            rclpy.spin_once(
                self, timeout_sec=min(0.05, deadline - self.monotonic()))

    def subscribe_transition_events(self, node_name: str) -> None:
        if node_name not in self.event_subscriptions:
            raise RuntimeError(
                f'no transition-event subscription for {node_name}')
        rclpy.spin_once(self, timeout_sec=0.0)

    def wait_change_state(self, node_name: str, timeout_sec: float) -> bool:
        return self.change_clients[node_name].wait_for_service(
            timeout_sec=timeout_sec)

    def wait_get_state(self, node_name: str, timeout_sec: float) -> bool:
        return self.state_clients[node_name].wait_for_service(
            timeout_sec=timeout_sec)

    def _wait_future(self, future: Any, timeout_sec: float) -> bool:
        deadline = self.monotonic() + timeout_sec
        while not future.done() and self.monotonic() < deadline:
            rclpy.spin_once(
                self, timeout_sec=min(0.05, deadline - self.monotonic()))
        return future.done()

    def get_state(
            self, node_name: str,
            timeout_sec: float) -> tuple[int, str]:
        client = self.state_clients[node_name]
        future = client.call_async(GetState.Request())
        if not self._wait_future(future, timeout_sec):
            client.remove_pending_request(future)
            raise TimeoutError(
                f'/{node_name}/get_state response timed out after '
                f'{timeout_sec:.3f}s')
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(str(exception))
        response = future.result()
        if response is None:
            raise RuntimeError('get_state completed without a response')
        return int(response.current_state.id), response.current_state.label

    def change_state(
            self, node_name: str, transition_id: int,
            timeout_sec: float) -> ServiceCallOutcome:
        client = self.change_clients[node_name]
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        if not self._wait_future(future, timeout_sec):
            client.remove_pending_request(future)
            return ServiceCallOutcome(
                status='TIMEOUT',
                failure_text=(
                    f'/{node_name}/change_state response timed out after '
                    f'{timeout_sec:.3f}s'))
        exception = future.exception()
        if exception is not None:
            return ServiceCallOutcome(
                status='ERROR', failure_text=str(exception))
        response = future.result()
        if response is None:
            return ServiceCallOutcome(
                status='ERROR',
                failure_text='change_state completed without a response')
        return ServiceCallOutcome(
            status='RESPONSE', success=bool(response.success),
            failure_text=(
                None if response.success
                else 'change_state returned success=false'))

    def take_transition_events(self, node_name: str) -> list[dict[str, Any]]:
        rclpy.spin_once(self, timeout_sec=0.0)
        events = self.events[node_name]
        self.events[node_name] = []
        return events


def lifecycle_interfaces(node_names: list[str]) -> dict[str, Any]:
    """Describe every lifecycle interface used by the orchestrator."""
    return {
        name: {
            'change_state_service': f'/{name}/change_state',
            'get_state_service': f'/{name}/get_state',
            'transition_event_topic': f'/{name}/transition_event',
        }
        for name in node_names
    }


def write_evidence(
        path: Path, *, run_id: str | None, navigation_mode: str,
        phase: str, node_names: list[str],
        operations: list[dict[str, Any]]) -> None:
    """Atomically append one non-overlapping orchestration phase."""
    if path.is_file():
        document = json.loads(path.read_text(encoding='utf-8'))
        if document.get('run_id') != run_id:
            raise RuntimeError('refusing to append lifecycle evidence run ID')
    else:
        document = {
            'schema_version': 1,
            'run_id': run_id,
            'navigation_mode': navigation_mode,
            'interfaces': {},
            'phases': [],
            'operations': [],
        }
    document['interfaces'].update(lifecycle_interfaces(node_names))
    document['phases'] = [
        item for item in document['phases'] if item.get('phase') != phase
    ]
    document['phases'].append({
        'phase': phase,
        'nodes': node_names,
        'success': all(item.get('success') for item in operations),
    })
    document['operations'] = [
        item for item in document['operations']
        if item.get('phase') != phase
    ]
    for operation in operations:
        operation['phase'] = phase
    document['operations'].extend(operations)
    all_operations = document['operations']
    document['summary'] = {
        'success': all(item.get('success') for item in all_operations),
        'operation_count': len(all_operations),
        'service_response_losses': sum(
            item.get('service_response_status') in LOST_RESPONSE_STATUSES
            for item in all_operations),
        'transition_event_recoveries': sum(
            item.get('confirmation_method') ==
            'TRANSITION_EVENT_AFTER_LOST_RESPONSE'
            for item in all_operations),
        'get_state_recoveries': sum(
            item.get('confirmation_method') ==
            'GET_STATE_AFTER_LOST_RESPONSE'
            for item in all_operations),
        'genuine_lifecycle_failures': sum(
            not item.get('success') for item in all_operations),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    temporary.replace(path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--navigation-mode',
        choices=('baseline', 'unprotected', 'protected'), required=True)
    parser.add_argument('--phase', choices=('upstream', 'bt'), required=True)
    parser.add_argument('--node', action='append', required=True,
                        dest='node_names')
    parser.add_argument('--evidence')
    args = parser.parse_args(remove_ros_args()[1:])
    evidence_value = args.evidence or os.environ.get(
        'ARES_WEEK6_LIFECYCLE_EVIDENCE_PATH')
    if not evidence_value:
        parser.error(
            '--evidence or ARES_WEEK6_LIFECYCLE_EVIDENCE_PATH is required')

    rclpy.init()
    backend = RosLifecycleBackend(args.node_names)
    orchestrator = RobustLifecycleOrchestrator(backend)
    operations: list[dict[str, Any]] = []
    try:
        for transition in (CONFIGURE, ACTIVATE):
            for node_name in args.node_names:
                result = orchestrator.transition(node_name, transition)
                operations.append(result)
                backend.get_logger().info(
                    'WEEK6_LIFECYCLE_TRANSITION_JSON=' +
                    json.dumps(result, sort_keys=True))
                write_evidence(
                    Path(evidence_value),
                    run_id=os.environ.get('ARES_WEEK6_RUN_ID'),
                    navigation_mode=args.navigation_mode,
                    phase=args.phase,
                    node_names=args.node_names,
                    operations=operations)
                if not result['success']:
                    backend.get_logger().error(
                        f"{transition['name']} failed for {node_name}: "
                        f"{result.get('failure')}")
                    raise SystemExit(1)
    finally:
        backend.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
