"""Structured ROS diagnostics with a JSON compatibility topic."""

import json
from typing import Any

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from std_msgs.msg import String


class MonitorDiagnostics:
    """Publish standard diagnostics while retaining inspectable JSON output."""

    def __init__(self, node: Any, name: str, compatibility_topic: str) -> None:
        self.name = name
        self.payload: dict[str, Any] = {
            'status': 'UNAVAILABLE',
            'reason': 'not evaluated',
        }
        self.level: Any = DiagnosticStatus.STALE
        self.summary = 'not evaluated'
        self.publisher = node.create_publisher(
            String, compatibility_topic, 10
        )
        self.updater = Updater(node)
        self.updater.setHardwareID('ares-supervisory-layer')
        self.updater.add(name, self._diagnostic_task)

    def publish(self, payload: dict[str, Any], level: Any,
                summary: str) -> None:
        """Publish one structured and compatibility diagnostic snapshot."""
        self.payload = payload
        self.level = level
        self.summary = summary
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'),
                                  sort_keys=True)
        self.publisher.publish(message)
        self.updater.force_update()

    def _diagnostic_task(self, status: Any) -> Any:
        status.summary(self.level, self.summary)
        for key, value in sorted(self.payload.items()):
            status.add(key, 'null' if value is None else str(value))
        return status


def diagnostic_level(trust_state: str) -> Any:
    """Map ARES trust/availability state to ROS diagnostic level."""
    if trust_state == 'HEALTHY':
        return DiagnosticStatus.OK
    if trust_state == 'DEGRADED':
        return DiagnosticStatus.WARN
    if trust_state == 'UNTRUSTED':
        return DiagnosticStatus.ERROR
    return DiagnosticStatus.STALE
