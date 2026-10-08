#!/usr/bin/env python3
"""Detection-independent staged GNSS recovery policy manager."""

from dataclasses import asdict, dataclass
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@dataclass(frozen=True)
class RecoveryDecision:
    """Current GNSS proxy command."""

    forward: bool
    covariance_factor: float
    state: str
    reason: str


class GnssRecoveryPolicy:
    """Stage deweighting, gating, probation, and normal re-entry."""

    def __init__(self, degraded_covariance_factor: float = 10.0,
                 untrusted_covariance_factor: float = 100.0,
                 untrusted_observations_before_gate: int = 5,
                 probation_observations: int = 10,
                 probation_covariance_factor: float = 5.0,
                 probation_high_covariance_observations: Optional[int] = None,
                 probation_high_covariance_factor: Optional[float] = None,
                 startup_forward: bool = True,
                 probation_after_ungated_degraded: bool = True) -> None:
        if probation_high_covariance_factor is None:
            probation_high_covariance_factor = degraded_covariance_factor
        if probation_high_covariance_observations is None:
            probation_high_covariance_observations = min(
                5, probation_observations - 1)
        if not 1.0 <= probation_covariance_factor <= (
                degraded_covariance_factor):
            raise ValueError('probation factor must be between 1 and degraded')
        if degraded_covariance_factor < 1.0:
            raise ValueError('degraded factor must be at least one')
        if not probation_covariance_factor <= (
                probation_high_covariance_factor) <= (
                    degraded_covariance_factor):
            raise ValueError(
                'high probation factor must be between medium and degraded')
        if untrusted_covariance_factor < degraded_covariance_factor:
            raise ValueError('untrusted factor must be at least degraded')
        if min(untrusted_observations_before_gate,
               probation_observations) < 1:
            raise ValueError('persistence must be at least one')
        if not 0 <= probation_high_covariance_observations < (
                probation_observations):
            raise ValueError(
                'high covariance observations must be within probation')
        self.degraded_factor = degraded_covariance_factor
        self.untrusted_factor = untrusted_covariance_factor
        self.probation_factor = probation_covariance_factor
        self.probation_high_factor = probation_high_covariance_factor
        self.probation_high_observations = (
            probation_high_covariance_observations)
        self.gate_after = untrusted_observations_before_gate
        self.probation_required = probation_observations
        self.startup_guard = not startup_forward
        self.gate_latched = not startup_forward
        self.probation_after_ungated_degraded = (
            probation_after_ungated_degraded)
        self.state = 'NORMAL' if startup_forward else 'GATED'
        self.trust_state = 'UNAVAILABLE'
        self.untrusted_count = 0
        self.probation_count = 0
        self.decision = RecoveryDecision(
            startup_forward, 1.0, self.state,
            ('startup passthrough; trust unavailable' if startup_forward else
             'startup gated pending attributed healthy evidence'))

    def update_trust(
        self,
        trust_state: str,
        attribution: str = 'INSUFFICIENT_EVIDENCE',
    ) -> RecoveryDecision:
        """Advance policy using debounced trust observations, not GNSS data."""
        if trust_state not in (
                'HEALTHY', 'DEGRADED', 'UNTRUSTED', 'UNAVAILABLE'):
            raise ValueError(f'unsupported GNSS trust state: {trust_state}')
        self.trust_state = trust_state
        if self.startup_guard and trust_state != 'HEALTHY':
            self.untrusted_count = 0
            self.probation_count = 0
            self.state = 'GATED'
            self.decision = RecoveryDecision(
                False, self.untrusted_factor, self.state,
                'startup gate retained pending healthy initialization')
            return self.decision
        if trust_state == 'UNTRUSTED':
            self.probation_count = 0
            if self.gate_latched:
                self.state = 'GATED'
                self.decision = RecoveryDecision(
                    False, self.untrusted_factor, self.state,
                    'latched gate retained until consecutive healthy evidence')
                return self.decision
            if attribution != 'LIKELY_GNSS_FAULT':
                self.untrusted_count = 0
                self.state = 'DEGRADED'
                self.decision = RecoveryDecision(
                    True, self.untrusted_factor, self.state,
                    f'GNSS UNTRUSTED but attribution {attribution} does not '
                    'authorize GNSS gating')
                return self.decision
            self.untrusted_count += 1
            if self.untrusted_count >= self.gate_after:
                self.state = 'GATED'
                self.gate_latched = True
                self.decision = RecoveryDecision(
                    False, self.untrusted_factor, self.state,
                    'persistent GNSS UNTRUSTED evidence')
            else:
                self.state = 'DEGRADED'
                self.decision = RecoveryDecision(
                    True, self.untrusted_factor, self.state,
                    'UNTRUSTED persistence accumulating before gate')
            return self.decision
        self.untrusted_count = 0
        if trust_state == 'HEALTHY':
            if self.state == 'NORMAL':
                self.decision = RecoveryDecision(
                    True, 1.0, 'NORMAL', 'GNSS trust healthy')
                return self.decision

            if (
                self.state == 'DEGRADED'
                and not self.gate_latched
                and not self.probation_after_ungated_degraded
            ):
                self.state = 'NORMAL'
                self.probation_count = 0
                self.decision = RecoveryDecision(
                    True, 1.0, 'NORMAL',
                    'ungated degradation recovered directly to normal')
                return self.decision

            self.state = 'PROBATION'
            self.probation_count += 1
            if self.probation_count >= self.probation_required:
                self.state = 'NORMAL'
                self.startup_guard = False
                self.gate_latched = False
                self.probation_count = 0
                self.decision = RecoveryDecision(
                    True, 1.0, 'NORMAL', 'probation completed')
            else:
                covariance_factor = (
                    self.probation_high_factor
                    if self.probation_count <=
                    self.probation_high_observations
                    else self.probation_factor)
                phase = (
                    'high covariance'
                    if covariance_factor == self.probation_high_factor
                    else 'medium covariance')
                self.decision = RecoveryDecision(
                    True, covariance_factor, 'PROBATION',
                    f'healthy evidence accumulating during {phase} probation')
            return self.decision
        self.probation_count = 0
        if self.gate_latched:
            if trust_state == 'DEGRADED':
                self.state = 'PROBATION'
                self.decision = RecoveryDecision(
                    True, self.probation_high_factor, self.state,
                    'fresh degraded evidence admitted at high covariance; '
                    'healthy probation not yet advancing')
                return self.decision
            self.state = 'GATED'
            self.decision = RecoveryDecision(
                False, self.untrusted_factor, self.state,
                'latched gate retained until consecutive healthy evidence')
            return self.decision
        self.state = 'DEGRADED'
        factor = self.degraded_factor
        reason = ('GNSS consistency degraded' if trust_state == 'DEGRADED'
                  else 'GNSS evidence unavailable; deweighted, not gated')
        self.decision = RecoveryDecision(True, factor, self.state, reason)
        return self.decision


class RecoveryManager(Node):
    """Turn GNSS trust and attribution context into a proxy policy."""

    def __init__(self) -> None:
        super().__init__('recovery_manager')
        self.policy = GnssRecoveryPolicy(
            degraded_covariance_factor=float(self.declare_parameter(
                'degraded_covariance_factor', 10.0).value),
            untrusted_covariance_factor=float(self.declare_parameter(
                'untrusted_covariance_factor', 100.0).value),
            untrusted_observations_before_gate=int(self.declare_parameter(
                'untrusted_observations_before_gate', 5).value),
            probation_observations=int(self.declare_parameter(
                'probation_observations', 10).value),
            probation_covariance_factor=float(self.declare_parameter(
                'probation_covariance_factor', 5.0).value),
            probation_high_covariance_observations=int(self.declare_parameter(
                'probation_high_covariance_observations', 5).value),
            probation_high_covariance_factor=float(self.declare_parameter(
                'probation_high_covariance_factor', 10.0).value),
            startup_forward=bool(self.declare_parameter(
                'startup_forward', True).value),
            probation_after_ungated_degraded=bool(
                self.declare_parameter(
                    'probation_after_ungated_degraded', True).value),
        )
        self.attribution = 'INSUFFICIENT_EVIDENCE'
        self.received_available_trust = False
        self.create_subscription(String, '/ares/trust/gnss',
                                 self.trust_callback, 10)
        self.create_subscription(String, '/ares/attribution',
                                 self.attribution_callback, 10)
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command_pub = self.create_publisher(
            String, '/ares/recovery/gnss_policy', qos)
        self.state_pub = self.create_publisher(
            String, '/ares/recovery_state', 10)
        self._publish()

    def attribution_callback(self, message: String) -> None:
        try:
            self.attribution = str(json.loads(message.data).get(
                'hypothesis', 'INSUFFICIENT_EVIDENCE'))
        except (json.JSONDecodeError, TypeError):
            self.attribution = 'INSUFFICIENT_EVIDENCE'
        # Attribution updates authorization context only. They are not new GNSS
        # observations and must never advance gate/probation persistence.
        self._publish()

    def trust_callback(self, message: String) -> None:
        if (message.data == 'UNAVAILABLE' and
                not self.received_available_trust and
                self.policy.state == 'NORMAL'):
            # Startup ordering can publish UNAVAILABLE before the first GNSS
            # sample. Preserve configured passthrough without manufacturing a
            # DEGRADED -> PROBATION cycle. Later dropouts are still actionable.
            self._publish()
            return
        if message.data != 'UNAVAILABLE':
            self.received_available_trust = True
        try:
            self.policy.update_trust(message.data, self.attribution)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._publish()

    def _publish(self) -> None:
        payload = asdict(self.policy.decision)
        payload.update({
            'gnss_trust_state': self.policy.trust_state,
            'attribution_context': self.attribution,
            'untrusted_count': self.policy.untrusted_count,
            'probation_count': self.policy.probation_count,
            'probation_high_covariance_observations':
                self.policy.probation_high_observations,
            'gate_latched': self.policy.gate_latched,
            'integration_note': (
                '/ares/gps_trusted never feeds the production EKF; Week 5 may '
                'route it only to /odometry/trust_fused.'),
        })
        command = String()
        command.data = json.dumps(
            payload, separators=(',', ':'), sort_keys=True)
        self.command_pub.publish(command)
        state = String()
        state.data = self.policy.state
        self.state_pub.publish(state)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RecoveryManager()
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
