"""Tests for lost-response-safe Week 6 lifecycle orchestration."""

from __future__ import annotations

from collections import deque
from typing import Any

from ares_reliability.week6_nav2_lifecycle_orchestrator import (
    ACTIVATE,
    CONFIGURE,
    RobustLifecycleOrchestrator,
    ServiceCallOutcome,
)

from lifecycle_msgs.msg import State


def _event(state_id: int, state: str) -> dict[str, Any]:
    return {
        'observation_time': '2026-10-07T00:00:00+00:00',
        'transition_id': 10,
        'transition_label': 'callback_success',
        'start_state_id': 10,
        'start_state': 'transitioning',
        'goal_state_id': state_id,
        'goal_state': state,
    }


class FakeBackend:
    """Deterministic lifecycle backend for evidence reconciliation tests."""

    def __init__(
            self, states: list[tuple[int, str] | Exception],
            outcome: ServiceCallOutcome,
            events: list[dict[str, Any]] | None = None) -> None:
        self.states = deque(states)
        self.outcome = outcome
        self.change_events = list(events or [])
        self.events: list[dict[str, Any]] = []
        self.now = 0.0
        self.change_calls = 0
        self.subscribed = False

    def monotonic(self) -> float:
        return self.now

    def wall_time(self) -> str:
        return f'monotonic:{self.now:.3f}'

    def sleep(self, duration_sec: float) -> None:
        self.now += duration_sec

    def subscribe_transition_events(self, node_name: str) -> None:
        self.subscribed = True

    def wait_change_state(self, node_name: str, timeout_sec: float) -> bool:
        return True

    def wait_get_state(self, node_name: str, timeout_sec: float) -> bool:
        return True

    def get_state(
            self, node_name: str,
            timeout_sec: float) -> tuple[int, str]:
        self.now += min(timeout_sec, 0.01)
        if not self.states:
            self.now += timeout_sec
            raise TimeoutError('simulated missing get_state response')
        value = self.states.popleft()
        if isinstance(value, Exception):
            raise value
        return value

    def change_state(
            self, node_name: str, transition_id: int,
            timeout_sec: float) -> ServiceCallOutcome:
        assert self.subscribed
        self.change_calls += 1
        self.now += min(timeout_sec, 0.01)
        self.events.extend(self.change_events)
        return self.outcome

    def take_transition_events(self, node_name: str) -> list[dict[str, Any]]:
        result = self.events
        self.events = []
        return result


def _run(
        transition: dict[str, Any],
        states: list[tuple[int, str] | Exception],
        outcome: ServiceCallOutcome,
        events: list[dict[str, Any]] | None = None):
    backend = FakeBackend(states, outcome, events)
    orchestrator = RobustLifecycleOrchestrator(
        backend,
        state_query_timeout_sec=0.01,
        initial_state_deadline_sec=0.05,
        operation_deadline_sec=0.05,
        retry_interval_sec=0.001,
    )
    return backend, orchestrator.transition('test_node', transition)


def test_normal_configure_response_and_inactive_state() -> None:
    backend, result = _run(
        CONFIGURE,
        [(State.PRIMARY_STATE_UNCONFIGURED, 'unconfigured'),
         (State.PRIMARY_STATE_INACTIVE, 'inactive')],
        ServiceCallOutcome(status='RESPONSE', success=True),
    )
    assert backend.change_calls == 1
    assert result['success'] is True
    assert result['confirmation_method'] == 'SERVICE_RESPONSE_AND_STATE'
    assert result['final_confirmed_state'] == 'inactive'


def test_normal_activate_response_and_active_state() -> None:
    _, result = _run(
        ACTIVATE,
        [(State.PRIMARY_STATE_INACTIVE, 'inactive'),
         (State.PRIMARY_STATE_ACTIVE, 'active')],
        ServiceCallOutcome(status='RESPONSE', success=True),
    )
    assert result['success'] is True
    assert result['confirmation_method'] == 'SERVICE_RESPONSE_AND_STATE'
    assert result['final_confirmed_state'] == 'active'


def test_configure_response_lost_but_event_confirms_inactive() -> None:
    _, result = _run(
        CONFIGURE,
        [(State.PRIMARY_STATE_UNCONFIGURED, 'unconfigured'),
         TimeoutError('get_state response also lost')],
        ServiceCallOutcome(
            status='TIMEOUT', failure_text='change_state response lost'),
        [_event(State.PRIMARY_STATE_INACTIVE, 'inactive')],
    )
    assert result['success'] is True
    assert result['confirmation_method'] == (
        'TRANSITION_EVENT_AFTER_LOST_RESPONSE')
    assert result['service_response_status'] == 'TIMEOUT'


def test_activate_response_lost_but_event_confirms_active() -> None:
    _, result = _run(
        ACTIVATE,
        [(State.PRIMARY_STATE_INACTIVE, 'inactive'),
         TimeoutError('get_state response also lost')],
        ServiceCallOutcome(
            status='TIMEOUT', failure_text='change_state response lost'),
        [_event(State.PRIMARY_STATE_ACTIVE, 'active')],
    )
    assert result['success'] is True
    assert result['confirmation_method'] == (
        'TRANSITION_EVENT_AFTER_LOST_RESPONSE')


def test_lost_response_missing_event_get_state_reports_target() -> None:
    _, result = _run(
        CONFIGURE,
        [(State.PRIMARY_STATE_UNCONFIGURED, 'unconfigured'),
         (State.PRIMARY_STATE_INACTIVE, 'inactive')],
        ServiceCallOutcome(
            status='TIMEOUT', failure_text='change_state response lost'),
    )
    assert result['success'] is True
    assert result['confirmation_method'] == 'GET_STATE_AFTER_LOST_RESPONSE'


def test_lost_response_without_target_evidence_fails_closed() -> None:
    _, result = _run(
        CONFIGURE,
        [(State.PRIMARY_STATE_UNCONFIGURED, 'unconfigured')],
        ServiceCallOutcome(
            status='TIMEOUT', failure_text='change_state response lost'),
    )
    assert result['success'] is False
    assert result['confirmation_method'] is None
    assert 'could not be confirmed' in result['failure']


def test_already_at_target_is_accepted_without_duplicate_transition() -> None:
    backend, result = _run(
        CONFIGURE,
        [(State.PRIMARY_STATE_INACTIVE, 'inactive')],
        ServiceCallOutcome(status='RESPONSE', success=True),
    )
    assert backend.change_calls == 0
    assert result['success'] is True
    assert result['service_response_status'] == (
        'NOT_SENT_ALREADY_AT_TARGET')
    assert result['confirmation_method'] == 'GET_STATE_ALREADY_AT_TARGET'


def test_contradictory_event_and_state_evidence_fails_closed() -> None:
    _, result = _run(
        ACTIVATE,
        [(State.PRIMARY_STATE_INACTIVE, 'inactive'),
         (State.PRIMARY_STATE_INACTIVE, 'inactive')],
        ServiceCallOutcome(
            status='TIMEOUT', failure_text='change_state response lost'),
        [_event(State.PRIMARY_STATE_ACTIVE, 'active')],
    )
    assert result['success'] is False
    assert result['confirmation_method'] is None
    assert result['failure'] == 'contradictory transition event and state'
