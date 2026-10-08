#!/usr/bin/env python3
"""Central dependency-aware ARES attribution engine."""

import json
from typing import Dict, Optional

from ares_reliability.evidence_graph import EvidenceDependencyGraph
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TrustEngine(Node):
    """Combine provenance-carrying tests without majority voting."""

    def __init__(self) -> None:
        super().__init__('trust_engine')
        self.graph = EvidenceDependencyGraph()
        self.source_states: Dict[str, str] = {
            'gnss': 'UNAVAILABLE', 'imu': 'UNAVAILABLE',
            'odometry': 'UNAVAILABLE', 'localization': 'UNAVAILABLE',
        }
        for sensor in ('gnss', 'imu', 'localization'):
            self.create_subscription(
                String, f'/ares/trust/{sensor}',
                lambda message, name=sensor: self._state_callback(
                    name, message), 10)
            self.create_subscription(
                String, f'/ares/evidence/{sensor}',
                self._evidence_callback, 10)
        self.system_pub = self.create_publisher(
            String, '/ares/system_trust_state', 10)
        self.attribution_pub = self.create_publisher(
            String, '/ares/attribution', 10)
        self.odom_pub = self.create_publisher(
            String, '/ares/trust/odometry', 10)
        self.graph_pub = self.create_publisher(
            String, '/ares/evidence_graph', 10)
        self.create_timer(1.0, self._publish)

    def _state_callback(self, sensor: str, message: String) -> None:
        if message.data not in (
                'HEALTHY', 'DEGRADED', 'UNTRUSTED', 'UNAVAILABLE'):
            self.get_logger().warning(
                f'Ignoring invalid {sensor} trust state: {message.data}')
            return
        self.source_states[sensor] = message.data
        self._publish()

    def _evidence_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.graph.update_payload(payload)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f'Ignoring malformed evidence: {error}')
            return
        self._publish()

    def _publish(self) -> None:
        assessment = self.graph.assess()
        hypothesis = assessment.hypothesis
        if hypothesis == 'LIKELY_ODOMETRY_FAULT':
            self.source_states['odometry'] = (
                'UNTRUSTED' if assessment.confidence >= 0.8 else 'DEGRADED')
        else:
            odom_tests = [
                item for item in self.graph.evidence.values()
                if 'odometry' in item.dependencies and
                item.state != 'UNAVAILABLE'
            ]
            if len(odom_tests) >= 2 and all(
                    item.state == 'CONSISTENT' for item in odom_tests):
                self.source_states['odometry'] = 'HEALTHY'
            elif odom_tests:
                self.source_states['odometry'] = 'DEGRADED'
            else:
                self.source_states['odometry'] = 'UNAVAILABLE'
        odom = String()
        odom.data = self.source_states['odometry']
        self.odom_pub.publish(odom)
        payload = assessment.as_dict()
        payload['sensor_states'] = dict(self.source_states)
        payload['statement'] = (
            'Hypothesis expresses consistency evidence, not malicious intent.')
        rendered = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        system = String()
        system.data = rendered
        self.system_pub.publish(system)
        attribution = String()
        attribution.data = rendered
        self.attribution_pub.publish(attribution)
        graph = String()
        graph.data = json.dumps({
            name: {
                'state': item.state,
                'dependencies': item.dependencies,
                'correlated': item.correlated,
                'independence_weight': item.independence_weight,
                'reason': item.reason,
            } for name, item in sorted(self.graph.evidence.items())
        }, separators=(',', ':'), sort_keys=True)
        self.graph_pub.publish(graph)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TrustEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
