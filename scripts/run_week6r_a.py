#!/usr/bin/env python3
"""Freeze, dry validate, and execute exactly one healthy attempt per frozen pair."""
import argparse
import hashlib
import json
import os
import secrets
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
C = ROOT / 'results/week6r_a'
SPEC = C / 'spec'
ENV = dict(os.environ, RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
           ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')


def now():
    return datetime.now(timezone.utc).isoformat()


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def write(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


def hashes():
    paths = set()
    for base in ('src', 'install'):
        for p in (ROOT / base).rglob('*'):
            if p.is_file() and '__pycache__' not in str(p) and p.suffix != '.pyc':
                paths.add(p)
    for name in ('run_week6_navigation.py', 'week6_runtime_isolation.py',
                 'run_week6r_a.py', 'analyze_week6r_a.py', 'week6r_a_observer.py'):
        paths.add(ROOT / 'scripts' / name)
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(paths)}


def evidence_hashes():
    return {str(p.relative_to(ROOT)): sha(p)
            for p in sorted((ROOT / 'results/week6').rglob('*')) if p.is_file()}


def verify():
    manifest = json.loads((SPEC / 'implementation_fingerprint.json').read_text())
    current = hashes()
    if current != manifest['sha256']:
        raise RuntimeError('Implementation changed since freeze')
    for line in (SPEC / 'pre_run_evidence_fingerprints.sha256').read_text().splitlines():
        digest, name = line.split('  ', 1)
        if sha(ROOT / name) != digest:
            raise RuntimeError('Frozen pre-run evidence changed: ' + name)
    seeds = json.loads((SPEC / 'healthy_seed_list.json').read_text())['seeds']
    assert len(seeds) == len(set(seeds)) == 30
    protocol = json.loads((SPEC / 'healthy_qualification_spec.json').read_text())
    assert protocol['run_order'] == [dict(seed=s, configuration=m)
                                    for s in seeds for m in ('baseline', 'ares')]
    return protocol, manifest


def freeze():
    if C.exists():
        raise FileExistsError('Campaign already exists; refuses overwrite')
    for name in ('spec', 'canonical', 'analysis', 'diagnostics'):
        (C / name).mkdir(parents=True)
    generated = now()
    seeds = []
    while len(seeds) < 30:
        candidate = 10000 + secrets.randbelow(2147473647)
        if candidate not in seeds:
            seeds.append(candidate)
    write(SPEC / 'healthy_seed_list.json', dict(seeds=seeds, generated_utc=generated,
          method='OS cryptographic randomness via secrets.randbelow; uniform integers '
                 '[10000,2147483646]; duplicates rejected before outcomes; generated once'))
    criteria = dict(
        frozen_utc=generated, hypotheses={
            'H1': 'Healthy completion non-inferiority, ARES minus baseline >= -0.05',
            'H2': 'Zero healthy gates and probations',
            'H3': 'No systematic protected-only controller instability',
            'H4': 'Matched path non-degradation', 'H5': 'No systematic runtime collapse'},
        gate_a='60 attempts exactly once, 30 pairs, consistent hashes, complete metrics, '
               'ready isolation and teardown and observer evidence, no infrastructure contamination',
        completion_margin=0.05,
        completion_ci='Conservative exact paired interval: simultaneous 97.5% two-sided '
                      'Clopper-Pearson intervals for ARES-only successes and baseline-only '
                      'successes; difference bounds [L_ares-U_baseline,U_ares-L_baseline]. '
                      'Bonferroni joint coverage >=95%. PASS if lower >=-0.05; FAIL if '
                      'observed difference <-0.05; otherwise INCONCLUSIVE.',
        false_gate_limit=0, false_probation_limit=0,
        controller_metrics=['controller_failure_count', 'collision_ahead_error_count',
                            'nav_recovery_count', 'controller_command_gap_count', 'nav2_abort_count'],
        systematic_rule='FAIL if >=2 seeds have ARES controller or collision-ahead events '
                        'and zero baseline events of that type, OR any controller metric '
                        'has a positive paired-mean bootstrap CI lower bound. Five metrics: '
                        '99% per-metric percentile bootstrap CIs (Bonferroni family 95%). '
                        'Otherwise PASS when complete; missing data INCONCLUSIVE.',
        path_thresholds={'mean_cross_track_error_m': 0.10,
                         'p95_cross_track_error_m': 0.10, 'max_cross_track_error_m': 0.15},
        path_rule='Use median paired relative degradation with baseline floor 0.01 m. '
                  'Also report absolute differences and floor-affected pairs. PASS if '
                  '95% bootstrap CI upper bounds all strictly below thresholds; FAIL if '
                  'any lower bound >=threshold; otherwise INCONCLUSIVE.',
        bootstrap=dict(resamples=20000, random_seed=640601),
        runtime_rule='Report paired metrics and Spearman correlations of controller burden '
                     'with median RTF, severe collapse counts, and rate misses; low RTF '
                     '<0.50, healthy RTF >=0.90. Positive 95% CI of paired severe-collapse '
                     'difference plus negative 95% CI of median-RTF difference flags systematic '
                     'runtime deterioration. Runtime never excuses controller failure.',
        final_rule='FAIL for failed scientific gates; otherwise INCONCLUSIVE for insufficient '
                   'support or incomplete/contaminated evidence. PASS requires every gate '
                   'and clean infrastructure. Proceed to Week6R-B only on PASS.')
    write(SPEC / 'acceptance_criteria.json', criteria)
    versions = subprocess.run(['dpkg-query', '-W', '-f=${Package} ${Version}\n'],
                              capture_output=True, text=True, check=True).stdout
    (SPEC / 'installed_package_versions.txt').write_text(versions)
    write(SPEC / 'implementation_fingerprint.json', dict(
        frozen_utc=generated, sha256=hashes(), ros_distro=os.environ.get('ROS_DISTRO'),
        gazebo_version=subprocess.check_output(['gz', 'sim', '--version'], text=True).strip(),
        python=sys.version, runtime_environment={k: ENV[k] for k in
                            ('RMW_IMPLEMENTATION', 'ROS_AUTOMATIC_DISCOVERY_RANGE')},
        discovery='Existing Week6 helper; unsupported unique-network-flows env unset',
        clock_boundary='Existing Week6 raw_clock -> /clock; 100 Hz maximum'))
    write(SPEC / 'healthy_qualification_spec.json', dict(
        frozen_utc=generated, stage='Week6R-A Healthy Controller Qualification',
        expected_runs=60, expected_pairs=30, fault_injection=False, retries=0,
        mode_mapping={'baseline': 'baseline', 'ares': 'protected'},
        baseline='Existing normal AMCL navigation stack',
        ares='Existing trust-gated GNSS fusion protected stack',
        configuration_note='Baseline and protected localization architectures differ as '
                           'in the existing stack; shared mission/map/robot/Nav2/runtime. '
                           'This qualifies the operational configurations, not an isolated '
                           'single-variable causal toggle.',
        mission='src/ares_reliability/config/week6_mission.yaml',
        world='src/ares_simulation/worlds/ares_test_world.sdf',
        map='src/ares_simulation/maps/ares_warehouse.yaml',
        robot='src/ares_simulation/models/ares_jackal/model.sdf',
        nav2='src/ares_reliability/config/nav2_week6_navigation.yaml',
        ekf=['src/ares_localization/config/ekf_baseline.yaml',
             'src/ares_reliability/config/ekf_week6_navigation.yaml'],
        ares_config='src/ares_reliability/config/week4_trust.yaml',
        observer='Passive trust transitions and /rosout; retained JSONL and launch log. '
                 'Collision-ahead counts from launch log, controller recovery diagnostics '
                 'from rosout. No tuning; no diagnostic replacement runs.',
        global_stop='Stop if teardown not ready, implementation mismatch, or two consecutive '
                    'infrastructure failures. Never retry. Resumption can only attempt '
                    'previously unattempted frozen entries after infrastructure resolution.',
        run_order=[dict(seed=s, configuration=m) for s in seeds for m in ('baseline', 'ares')]))
    # Capture every old Week6 file, plus all new specification files, before execution.
    old = evidence_hashes()
    write(SPEC / 'week6_evidence_inventory.json', old)
    frozen = old | {str(p.relative_to(ROOT)): sha(p) for p in sorted(SPEC.iterdir()) if p.is_file()}
    with (SPEC / 'pre_run_evidence_fingerprints.sha256').open('x') as f:
        f.writelines(f'{digest}  {name}\n' for name, digest in frozen.items())
    print('PRE_RUN_FREEZE_WRITTEN', generated, flush=True)


def dry():
    protocol, manifest = verify()
    assert not any((C / 'canonical').iterdir())
    from week6_runtime_isolation import establish_world_exclusivity
    from run_week6_navigation import week6_runtime_environment
    env = week6_runtime_environment(ENV, C / 'diagnostics/dry', 'week6r-a-dry')
    isolation = establish_world_exclusivity(C / 'diagnostics/dry_isolation.json', environment=env)
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('ares_reliability'))
    assert sha(share / 'launch/week6_navigation.launch.py') == sha(
        ROOT / 'src/ares_reliability/launch/week6_navigation.launch.py')
    for entry in protocol['run_order']:
        assert not (C / 'canonical' / f"seed_{entry['seed']}" / entry['configuration']).exists()
    subprocess.run([sys.executable, str(ROOT / 'scripts/run_week6_navigation.py'), '--help'],
                   check=True, stdout=subprocess.DEVNULL)
    write(C / 'diagnostics/dry_validation.json', dict(validated_utc=now(), passed=True,
          canonical_seeds_consumed=0, seed_parsing=True, output_paths=True,
          mode_mapping=protocol['mode_mapping'], source_installed_launch_matches=True,
          hashes_captured=len(manifest['sha256']), isolation=isolation['world_exclusivity_ready']))
    print('DRY_VALIDATION_PASS; CANONICAL_ATTEMPTS=0', flush=True)


def execute():
    protocol, manifest = verify()
    assert json.loads((C / 'diagnostics/dry_validation.json').read_text())['passed']
    lock = C / 'campaign_execution_lock.json'
    write(lock, dict(started_utc=now(), pid=os.getpid(), run_order=protocol['run_order']))
    consecutive_infrastructure = 0
    for index, entry in enumerate(protocol['run_order'], 1):
        verify()
        directory = C / 'canonical' / f"seed_{entry['seed']}" / entry['configuration']
        directory.mkdir(parents=True, exist_ok=False)
        command = [sys.executable, str(ROOT / 'scripts/run_week6_navigation.py'),
                   '--navigation-mode', protocol['mode_mapping'][entry['configuration']],
                   '--scenario', 'healthy', '--seed', str(entry['seed']),
                   '--run-directory', str(directory)]
        write(directory / 'attempt.json', dict(**entry, order=index, started_utc=now(),
              command=command, implementation_sha256=manifest['sha256']))
        started = time.monotonic()
        print(f"ATTEMPT {index}/60 seed={entry['seed']} mode={entry['configuration']}", flush=True)
        with (directory / 'observer.log').open('x') as log:
            observer = subprocess.Popen([sys.executable, str(ROOT / 'scripts/week6r_a_observer.py'),
                                         str(directory / 'observer.jsonl')], env=ENV,
                                        stdout=log, stderr=subprocess.STDOUT)
            with (directory / 'runner.log').open('x') as runner_log:
                completed = subprocess.run(command, env=ENV, stdout=runner_log,
                                           stderr=subprocess.STDOUT, check=False)
            observer.terminate()
            try:
                observer.wait(timeout=10)
            except subprocess.TimeoutExpired:
                observer.kill()
                observer.wait()
        result_file = directory / 'result.json'
        result = json.loads(result_file.read_text()) if result_file.exists() else {}
        teardown_file = directory / 'post_run_teardown.json'
        teardown = json.loads(teardown_file.read_text()) if teardown_file.exists() else {}
        isolation_file = directory / 'runtime_isolation.json'
        isolation = json.loads(isolation_file.read_text()) if isolation_file.exists() else {}
        infrastructure = (not result or result.get('failure_class') == 'ORCHESTRATION_RUNTIME'
                          or not teardown.get('teardown_ready')
                          or not isolation.get('world_exclusivity_ready') or observer.returncode != 0)
        write(directory / 'attempt_outcome.json', dict(exit_code=completed.returncode,
              observer_exit_code=observer.returncode, finished_utc=now(),
              wall_time_sec=time.monotonic()-started, infrastructure_failure=infrastructure,
              mission_completed=result.get('mission_completed'),
              teardown_ready=teardown.get('teardown_ready', False),
              implementation_hashes_consistent=hashes() == manifest['sha256']))
        print(f"FINISHED {index}/60 completion={result.get('mission_completed')} "
              f"infrastructure_failure={infrastructure} teardown={teardown.get('teardown_ready')}", flush=True)
        consecutive_infrastructure = consecutive_infrastructure + 1 if infrastructure else 0
        if not teardown.get('teardown_ready') or consecutive_infrastructure >= 2:
            write(C / 'global_infrastructure_stop.json', dict(stopped_utc=now(), order=index,
                  reason='Teardown not ready or two consecutive infrastructure failures'))
            break
    subprocess.run([sys.executable, str(ROOT / 'scripts/analyze_week6r_a.py')], check=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['freeze', 'dry', 'execute'])
    args = parser.parse_args()
    globals()[args.action]()
