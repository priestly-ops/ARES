"""Dependency-aware evidence representation and cautious attribution."""

from dataclasses import asdict, dataclass
from typing import Dict, Iterable, Optional, Tuple


EVIDENCE_STATES = (
    'CONSISTENT', 'DEGRADED', 'INCONSISTENT', 'UNAVAILABLE'
)


@dataclass(frozen=True)
class Evidence:
    """One test result with provenance and independence metadata."""

    test_id: str
    state: str
    dependencies: Tuple[str, ...]
    reason: str
    correlated: bool = False
    independence_weight: float = 1.0
    residual: Optional[float] = None

    def __post_init__(self) -> None:
        if self.state not in EVIDENCE_STATES:
            raise ValueError(f'unsupported evidence state: {self.state}')
        if not 0.0 < self.independence_weight <= 1.0:
            raise ValueError('independence weight must be in (0, 1]')


@dataclass(frozen=True)
class AttributionAssessment:
    """Explainable system hypothesis, not a claim of attack intent."""

    consistency: str
    hypothesis: str
    confidence: float
    confidence_label: str
    supporting_evidence: Tuple[str, ...]
    contradicting_evidence: Tuple[str, ...]
    unavailable_evidence: Tuple[str, ...]
    reason: str
    effective_evidence_weight: float

    def as_dict(self) -> dict:
        """Return JSON-ready fields with a compatibility attribution key."""
        result = asdict(self)
        result['attribution'] = self.hypothesis
        return result


class EvidenceDependencyGraph:
    """Store latest evidence and prevent correlated tests becoming votes."""

    def __init__(self) -> None:
        self.evidence: Dict[str, Evidence] = {}

    def update(self, evidence: Evidence) -> None:
        """Replace the latest result for one named test."""
        self.evidence[evidence.test_id] = evidence

    def update_payload(self, payload: dict) -> Evidence:
        """Validate and store a monitor JSON payload."""
        item = Evidence(
            test_id=str(payload['test_id']),
            state=str(payload.get('evidence_state', 'UNAVAILABLE')),
            dependencies=tuple(str(value) for value in
                               payload.get('dependencies', [])),
            reason=str(payload.get('reason', 'unspecified')),
            correlated=bool(payload.get('correlated', False)),
            independence_weight=float(payload.get(
                'independence_weight',
                0.5 if payload.get('correlated', False) else 1.0)),
            residual=_optional_float(payload.get('residual')),
        )
        self.update(item)
        return item

    def assess(self) -> AttributionAssessment:
        """Apply deterministic rules with explicit evidence limitations."""
        unavailable = tuple(sorted(
            evidence.test_id for evidence in self.evidence.values()
            if evidence.state == 'UNAVAILABLE'))
        available = {name: item for name, item in self.evidence.items()
                     if item.state != 'UNAVAILABLE'}
        weight = sum(item.independence_weight for item in available.values())
        direct_fault = self._direct_freshness_fault(available.values())
        if direct_fault is not None:
            sensor, support = direct_fault
            other_bad = tuple(sorted(
                item.test_id for item in available.values()
                if item.test_id not in support and
                item.state in ('DEGRADED', 'INCONSISTENT')))
            if other_bad:
                return self._assessment(
                    'MULTI_SENSOR_INCONSISTENCY', 'INSUFFICIENT_EVIDENCE',
                    0.35, support + other_bad, (), unavailable,
                    f'{sensor} has a direct validity failure while other '
                    'independent inconsistencies are also active', weight)
            return self._assessment(
                'SENSOR_INCONSISTENCY', f'LIKELY_{sensor.upper()}_FAULT',
                0.85, support, (), unavailable,
                f'{sensor} freshness failed independently of pair residuals',
                weight)

        def state(name: str) -> str:
            item = available.get(name)
            return 'UNAVAILABLE' if item is None else item.state

        gnss = state('gnss_motion')
        imu = state('imu_wheel')
        localization = state('localization_motion')
        bad = {'DEGRADED', 'INCONSISTENT'}
        good = 'CONSISTENT'

        if gnss == good and imu == good and localization in (
                good, 'UNAVAILABLE'):
            support = ('gnss_motion', 'imu_wheel') + (
                ('localization_motion',) if localization == good else ())
            return self._assessment(
                'CONSISTENT', 'SYSTEM_HEALTHY',
                min(0.95, 0.65 + 0.1 * weight), support, (), unavailable,
                'independent GNSS-motion and IMU-wheel checks agree', weight)
        if gnss in bad and imu == good and localization in (
                good, 'UNAVAILABLE'):
            gnss_support = ['gnss_motion', 'imu_wheel']
            confidence = 0.72
            if localization == good:
                gnss_support.append('localization_motion')
                confidence += 0.08  # correlated AMCL evidence has low weight
            contradict = ('localization_motion',) if localization in bad else ()
            return self._assessment(
                'SENSOR_INCONSISTENCY', 'LIKELY_GNSS_FAULT', confidence,
                tuple(gnss_support), contradict, unavailable,
                'GNSS disagrees while the independent IMU-wheel relation is '
                'consistent; AMCL support is correlation-discounted', weight)
        if imu in bad and gnss == good and localization in bad:
            return self._assessment(
                'MULTI_SENSOR_INCONSISTENCY', 'LIKELY_ODOMETRY_FAULT', 0.65,
                ('imu_wheel', 'localization_motion', 'gnss_motion'), (),
                unavailable,
                'IMU and correlated localization relations share raw wheel '
                'odometry while filtered-motion GNSS remains consistent',
                weight)
        if imu in bad and gnss == good:
            imu_support = ['imu_wheel', 'gnss_motion']
            confidence = 0.72
            if localization == good:
                imu_support.append('localization_motion')
                confidence += 0.08
            return self._assessment(
                'SENSOR_INCONSISTENCY', 'LIKELY_IMU_FAULT', confidence,
                tuple(imu_support), (), unavailable,
                'IMU disagrees with wheel rate while GNSS-motion remains '
                'consistent', weight)
        if localization in bad and gnss == good and imu == good:
            return self._assessment(
                'SENSOR_INCONSISTENCY', 'LIKELY_LOCALIZATION_FAULT', 0.68,
                ('localization_motion', 'gnss_motion', 'imu_wheel'), (),
                unavailable,
                'AMCL relative motion disagrees while two other relations '
                'remain consistent', weight)
        if gnss in bad and localization in bad and imu == good:
            return self._assessment(
                'MULTI_SENSOR_INCONSISTENCY', 'INSUFFICIENT_EVIDENCE', 0.38,
                ('gnss_motion', 'localization_motion', 'imu_wheel'), (),
                unavailable,
                'GNSS and correlated localization both disagree with odometry; '
                'linear odometry error and simultaneous faults are ambiguous',
                weight)
        if gnss in bad and imu in bad and localization in bad:
            odom_support = ['gnss_motion', 'imu_wheel']
            if localization in bad:
                odom_support.append('localization_motion')
            confidence = 0.65
            return self._assessment(
                'MULTI_SENSOR_INCONSISTENCY', 'LIKELY_ODOMETRY_FAULT',
                confidence, tuple(odom_support), (), unavailable,
                'independent GNSS and IMU tests share wheel/motion evidence; '
                'the AMCL branch is not counted as independent', weight)
        if gnss in bad and imu in bad:
            return self._assessment(
                'MULTI_SENSOR_INCONSISTENCY', 'INSUFFICIENT_EVIDENCE', 0.35,
                ('gnss_motion', 'imu_wheel'), (), unavailable,
                'two inconsistent relations share odometry, but simultaneous '
                'GNSS and IMU faults cannot be excluded', weight)
        inconsistent = tuple(sorted(name for name, item in available.items()
                                    if item.state in bad))
        if inconsistent:
            return self._assessment(
                'MULTI_SENSOR_INCONSISTENCY', 'INSUFFICIENT_EVIDENCE', 0.30,
                inconsistent, (), unavailable,
                'an inconsistency exists without enough independent '
                'cross-checks to isolate a source', weight)
        return self._assessment(
            'INSUFFICIENT_EVIDENCE', 'INSUFFICIENT_EVIDENCE', 0.10,
            (), (), unavailable,
            'fewer than two useful independent consistency relations exist',
            weight)

    @staticmethod
    def _direct_freshness_fault(
        evidence: Iterable[Evidence],
    ) -> Optional[tuple[str, tuple[str, ...]]]:
        for item in evidence:
            if item.test_id == 'gnss_initialization' and (
                    item.state == 'INCONSISTENT'):
                return 'gnss', (item.test_id,)
            if (item.test_id.endswith('_freshness') and
                    item.state == 'INCONSISTENT' and
                    len(item.dependencies) == 1):
                return item.dependencies[0], (item.test_id,)
        return None

    @staticmethod
    def _assessment(consistency: str, hypothesis: str, confidence: float,
                    support: tuple[str, ...], contradict: tuple[str, ...],
                    unavailable: tuple[str, ...], reason: str,
                    weight: float) -> AttributionAssessment:
        label = ('HIGH' if confidence >= 0.8 else
                 'MODERATE' if confidence >= 0.5 else 'LOW')
        return AttributionAssessment(
            consistency, hypothesis, confidence, label, support, contradict,
            unavailable, reason, weight)


def _optional_float(value: object) -> Optional[float]:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return result
