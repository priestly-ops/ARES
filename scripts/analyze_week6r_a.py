#!/usr/bin/env python3
"""Matched healthy qualification statistics under the frozen decision rules."""
import json
import math
import re
from pathlib import Path

import numpy as np

from run_week6r_a import C, SPEC, ROOT, sha, hashes, evidence_hashes

CONTINUOUS = ['final_goal_error_m', 'ground_reference_final_goal_error_m',
              'mission_completion_time_sec', 'mean_cross_track_error_m',
              'median_cross_track_error_m', 'p95_cross_track_error_m',
              'max_cross_track_error_m', 'path_length_ratio', 'mean_localization_error_m',
              'max_localization_error_m', 'max_controller_command_gap_sec',
              'rtf_median', 'rtf_p10', 'rtf_p90', 'clock_mean_hz', 'wall_time_sec']
EVENTS = ['planner_failure_count', 'controller_failure_count', 'collision_ahead_error_count',
          'nav_recovery_count', 'nav2_abort_count', 'controller_command_gap_count',
          'controller_rate_miss_count', 'costmap_clear_event_count', 'tf_error_count',
          'estimator_restart_count', 'lifecycle_failure_count', 'costmap_error_count',
          'severe_rtf_collapse_count', 'clock_backward_jump_count',
          'ares_gate_count', 'ares_probation_count', 'degraded_transition_count']


def read(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def save(name, value):
    path = C / 'analysis' / name
    with path.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


def valid(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


def binom_cdf(k, n, p):
    return sum(math.comb(n, j)*p**j*(1-p)**(n-j) for j in range(k+1))


def cp(k, n, alpha):
    if n == 0:
        return [0.0, 1.0]
    def solve(target, index):
        lo, hi = 0.0, 1.0
        for _ in range(70):
            mid = (lo+hi)/2
            if binom_cdf(index, n, mid) > target:
                lo = mid
            else:
                hi = mid
        return (lo+hi)/2
    return [0.0 if k == 0 else solve(1-alpha/2, k-1),
            1.0 if k == n else solve(alpha/2, k)]


def boot(values, rng, resamples, alpha=0.05):
    values = np.asarray(values, dtype=float)
    if len(values) == 0:
        return dict(mean_ci=None, median_ci=None)
    samples = values[rng.integers(0, len(values), (resamples, len(values)))]
    bounds = [alpha/2, 1-alpha/2]
    return dict(mean_ci=np.quantile(samples.mean(axis=1), bounds).tolist(),
                median_ci=np.quantile(np.median(samples, axis=1), bounds).tolist())


def statistic(pairs, metric, rng, resamples):
    complete = [(p['baseline'].get(metric), p['ares'].get(metric)) for p in pairs]
    complete = [(b, a) for b, a in complete if valid(b) and valid(a)]
    if not complete:
        return dict(valid_pairs=0, missing_pairs=len(pairs))
    b, a = np.asarray(complete, dtype=float).T
    d = a-b
    sd = float(np.std(d, ddof=1)) if len(d)>1 else 0
    return dict(valid_pairs=len(d), missing_pairs=len(pairs)-len(d),
                baseline_mean=float(b.mean()), ares_mean=float(a.mean()),
                baseline_median=float(np.median(b)), ares_median=float(np.median(a)),
                paired_mean_difference=float(d.mean()), paired_median_difference=float(np.median(d)),
                paired_difference_95_bootstrap=boot(d, rng, resamples),
                ares_numerically_greater_fraction=float(np.mean(d>0)),
                ares_worse_fraction=float(np.mean(d<0 if metric.startswith('rtf_') else d>0)),
                standardized_paired_mean_effect=d.mean()/sd if sd>0 else None)


def event_stat(pairs, metric, rng, resamples):
    s = statistic(pairs, metric, rng, resamples)
    complete = [(p['baseline'].get(metric), p['ares'].get(metric)) for p in pairs]
    complete = [(b, a) for b, a in complete if valid(b) and valid(a)]
    if complete:
        b, a = np.asarray(complete).T
        s.update(baseline_total=int(b.sum()), ares_total=int(a.sum()),
                 baseline_runs_with_event=int(sum(b>0)), ares_runs_with_event=int(sum(a>0)),
                 paired_contingency=dict(neither=int(sum((b==0)&(a==0))),
                     baseline_only=int(sum((b>0)&(a==0))), ares_only=int(sum((a>0)&(b==0))),
                     both=int(sum((a>0)&(b>0)))),
                 exposure_note='Counts per canonical run; unequal mission durations also reported')
    return s


def load_run(seed, mode):
    directory = C / 'canonical' / f'seed_{seed}' / mode
    attempt = read(directory / 'attempt.json', {})
    outcome = read(directory / 'attempt_outcome.json', {})
    raw = read(directory / 'result.json', {})
    row = {key: raw.get(key) for key in CONTINUOUS+EVENTS}
    row.update(seed=seed, configuration=mode, attempted=bool(attempt),
               mission_completed=raw.get('mission_completed'),
               waypoints_reached=raw.get('waypoints_reached'), waypoints_total=raw.get('waypoints_total'),
               result_path=str((directory / 'result.json').relative_to(ROOT)),
               raw_failure_class=raw.get('failure_class'),
               infrastructure_failure=outcome.get('infrastructure_failure', True),
               wall_time_sec=outcome.get('wall_time_sec'), exit_code=outcome.get('exit_code'),
               teardown_ready=outcome.get('teardown_ready', False),
               hashes_consistent=outcome.get('implementation_hashes_consistent', False))
    runtime = raw.get('runtime_quality', {})
    for key in ('rtf_median', 'rtf_p10', 'rtf_p90', 'clock_mean_hz',
                'severe_rtf_collapse_count', 'clock_backward_jump_count'):
        row[key] = runtime.get(key)
    text = (directory / 'launch.log').read_text(errors='replace') if (directory / 'launch.log').exists() else ''
    lines = text.splitlines()
    diagnostic_events = []
    patterns = {'collision_ahead': 'RegulatedPurePursuitController detected collision ahead!',
                'costmap_clear': 'Received request to clear entirely all layers',
                'planner_failure': '[planner_server]: GridBasedplugin failed to plan from',
                'controller_failure': '[controller_server]: [follow_path] [ActionServer] Aborting handle',
                'navigation_abort': '[bt_navigator]: Goal failed'}
    counts = {name: 0 for name in patterns}
    for index, line in enumerate(lines, 1):
        for name, pattern in patterns.items():
            if pattern in line:
                counts[name] += 1
                diagnostic_events.append(dict(kind=name, line=index, message=line))
    row['collision_ahead_error_count'] = counts['collision_ahead'] if text else None
    row['costmap_clear_event_count'] = counts['costmap_clear'] if text else None
    row['mission_reported_controller_failure_count'] = raw.get('controller_failure_count')
    row['launch_controller_abort_count'] = counts['controller_failure']
    # Max across overlapping sources avoids counting the same failure twice.
    for key, derived in (('controller_failure_count', max(counts['collision_ahead'], counts['controller_failure'])),
                         ('planner_failure_count', counts['planner_failure']),
                         ('nav2_abort_count', counts['navigation_abort'])):
        if key in raw:
            row[key] = max(raw[key] or 0, derived)
    transitions = []
    rosout_events = []
    observer_ready = observer_stopped = False
    observer_path = directory / 'observer.jsonl'
    if observer_path.exists():
        for line in observer_path.read_text().splitlines():
            entry = json.loads(line)
            if entry['kind'] == 'transition':
                transitions.append(entry)
            elif entry['kind'] == 'observer':
                observer_ready |= entry['value'] == 'ready'
                observer_stopped |= entry['value'] == 'stopped'
            elif entry['kind'] == 'rosout' and re.search(
                    r'collision|failed|Aborting|clear entirely|Running (spin|wait|backup)',
                    entry['value']['message'], re.I):
                rosout_events.append(entry)
    row['observer_capture_complete'] = observer_ready and observer_stopped
    row['trust_transitions'] = transitions
    row['degraded_transitions'] = [t for t in transitions if 'DEGRADED' in t['value']['state']]
    row['degraded_transition_count'] = len(row['degraded_transitions']) if observer_ready else None
    recovery = [t for t in transitions if t['topic'] == '/ares/recovery_state']
    if mode == 'ares':
        for key, state in (('ares_gate_count', 'GATED'), ('ares_probation_count', 'PROBATION')):
            if key in raw:
                row[key] = max(raw[key] or 0, sum(t['value']['state'] == state for t in recovery))
    row['unexpected_protection_state_changes'] = [t for t in recovery
               if t['value']['state'] not in ('NORMAL', 'UNAVAILABLE')]
    row['diagnostic_events'] = diagnostic_events
    row['controller_collision_rosout_diagnostics'] = rosout_events
    required = CONTINUOUS+EVENTS+['waypoints_reached', 'waypoints_total', 'mission_completed']
    row['missing_metrics'] = [k for k in required if row.get(k) is None]
    # An infrastructure run retains missing values rather than manufacturing zeros.
    return row


def rank(values):
    result = np.empty(len(values))
    order = np.argsort(values)
    i = 0
    while i < len(values):
        j = i+1
        while j < len(values) and values[order[j]] == values[order[i]]:
            j += 1
        result[order[i:j]] = (i+j-1)/2
        i = j
    return result


def main():
    criteria = read(SPEC / 'acceptance_criteria.json')
    seeds = read(SPEC / 'healthy_seed_list.json')['seeds']
    rng = np.random.default_rng(criteria['bootstrap']['random_seed'])
    nboot = criteria['bootstrap']['resamples']
    pairs = [dict(seed=s, baseline=load_run(s, 'baseline'), ares=load_run(s, 'ares')) for s in seeds]
    rows = [p[m] for p in pairs for m in ('baseline', 'ares')]
    attempted = sum(r['attempted'] for r in rows)
    represented = sum(p['baseline']['attempted'] and p['ares']['attempted'] for p in pairs)
    complete = [p for p in pairs if p['baseline']['mission_completed'] is not None
                and p['ares']['mission_completed'] is not None]
    table = {key: 0 for key in ('both_complete', 'baseline_only_complete', 'ares_only_complete', 'neither_complete')}
    for p in complete:
        b, a = p['baseline']['mission_completed'], p['ares']['mission_completed']
        key = 'both_complete' if b and a else 'baseline_only_complete' if b else 'ares_only_complete' if a else 'neither_complete'
        table[key] += 1
    n = len(complete)
    discordant = table['baseline_only_complete']+table['ares_only_complete']
    b_success = table['both_complete']+table['baseline_only_complete']
    a_success = table['both_complete']+table['ares_only_complete']
    a_ci = cp(table['ares_only_complete'], n, 0.025)
    b_ci = cp(table['baseline_only_complete'], n, 0.025)
    diff_ci = [a_ci[0]-b_ci[1], a_ci[1]-b_ci[0]]
    observed = (a_success-b_success)/n if n else None
    completion_gate = 'INCONCLUSIVE'
    if n == 30:
        if observed < -criteria['completion_margin']:
            completion_gate = 'FAIL'
        elif diff_ci[0] >= -criteria['completion_margin']:
            completion_gate = 'PASS'
    completion = dict(valid_pairs=n, table=table, baseline_completed=b_success,
                      ares_completed=a_success, baseline_rate_ci_95=cp(b_success,n,0.05),
                      ares_rate_ci_95=cp(a_success,n,0.05), ares_minus_baseline=observed,
                      paired_difference_ci_95_conservative_exact=diff_ci,
                      method=criteria['completion_ci'], gate=completion_gate,
                      mcnemar_exact_two_sided_p=min(1.0, 2*binom_cdf(min(
                          table['baseline_only_complete'],table['ares_only_complete']), discordant, 0.5))
                          if discordant else 1.0)
    continuous = {key: statistic(pairs, key, rng, nboot) for key in CONTINUOUS}
    events = {key: event_stat(pairs, key, rng, nboot) for key in EVENTS}
    controller = dict(frozen_rule=criteria['systematic_rule'], metrics={}, failing_metrics=[])
    repeated_seeds = []
    for p in pairs:
        for key in ('controller_failure_count', 'collision_ahead_error_count'):
            b, a = p['baseline'].get(key), p['ares'].get(key)
            if valid(a) and valid(b) and a>0 and b==0:
                repeated_seeds.append(dict(seed=p['seed'], metric=key, ares=a, baseline=b))
    for key in criteria['controller_metrics']:
        s = events[key].copy()
        d = [p['ares'][key]-p['baseline'][key] for p in pairs
             if valid(p['ares'].get(key)) and valid(p['baseline'].get(key))]
        s['paired_difference_99_bootstrap'] = boot(d, rng, nboot, alpha=0.01)
        controller['metrics'][key] = s
        ci = s['paired_difference_99_bootstrap']['mean_ci']
        if ci and ci[0] > 0:
            controller['failing_metrics'].append(key)
    repeated = len(set(r['seed'] for r in repeated_seeds)) >= 2
    controller['protected_only_failure_seeds'] = repeated_seeds
    controller_gate = 'FAIL' if repeated or controller['failing_metrics'] else (
        'PASS' if all(events[k]['valid_pairs']==30 for k in criteria['controller_metrics']) else 'INCONCLUSIVE')
    controller['gate'] = controller_gate
    path = {}
    for key, threshold in criteria['path_thresholds'].items():
        relative = [(p['ares'][key]-p['baseline'][key])/max(p['baseline'][key],0.01)
                    for p in pairs if valid(p['ares'].get(key)) and valid(p['baseline'].get(key))]
        ci = boot(relative, rng, nboot)['median_ci']
        gate = 'INCONCLUSIVE'
        if len(relative)==30:
            if ci[1] < threshold:
                gate = 'PASS'
            elif ci[0] >= threshold:
                gate = 'FAIL'
        path[key] = dict(threshold=threshold, valid_pairs=len(relative),
                         median_relative_degradation=float(np.median(relative)) if relative else None,
                         median_relative_degradation_95_bootstrap_ci=ci,
                         baseline_floor_m=0.01, floor_affected_pairs=sum(
                             valid(p['baseline'].get(key)) and p['baseline'][key]<0.01 for p in pairs),
                         absolute_differences=continuous[key], gate=gate)
    path_gate = 'FAIL' if any(s['gate']=='FAIL' for s in path.values()) else (
        'PASS' if all(s['gate']=='PASS' for s in path.values()) else 'INCONCLUSIVE')
    ares_rows = [p['ares'] for p in pairs]
    gates = sum(r['ares_gate_count'] or 0 for r in ares_rows)
    probations = sum(r['ares_probation_count'] or 0 for r in ares_rows)
    false = dict(false_gate_count=gates, false_probation_count=probations,
                 runs_with_gate=sum((r['ares_gate_count'] or 0)>0 for r in ares_rows),
                 runs_with_probation=sum((r['ares_probation_count'] or 0)>0 for r in ares_rows),
                 denominator_attempted=sum(r['attempted'] for r in ares_rows),
                 degraded_transitions=[dict(seed=r['seed'],transitions=r['degraded_transitions'])
                                       for r in ares_rows if r['degraded_transitions']],
                 unexpected_protection_state_changes=[dict(seed=r['seed'],transitions=r['unexpected_protection_state_changes'])
                                       for r in ares_rows if r['unexpected_protection_state_changes']],
                 gate='FAIL' if gates or probations else 'PASS')
    denom = false['denominator_attempted']
    false['gate_run_rate'] = false['runs_with_gate']/denom if denom else None
    false['probation_run_rate'] = false['runs_with_probation']/denom if denom else None
    correlations = {}
    affected = [r for r in rows if any((r.get(k) or 0)>0 for k in
                           ('controller_failure_count','collision_ahead_error_count','nav2_abort_count'))]
    for key in ('rtf_median','severe_rtf_collapse_count','controller_rate_miss_count'):
        xy = [(r[key],sum(r.get(k) or 0 for k in criteria['controller_metrics']))
              for r in rows if valid(r.get(key))]
        rho = None
        if len(xy)>1:
            x,y=np.asarray(xy).T
            if np.std(x)>0 and np.std(y)>0:
                rho=float(np.corrcoef(rank(x),rank(y))[0,1])
        correlations[key]=dict(valid_runs=len(xy),spearman_rho=rho,
                              causal_inference='Association only; shared scheduling confounds remain')
    rtf_ci=continuous['rtf_median'].get('paired_difference_95_bootstrap',{}).get('mean_ci')
    collapse_ci=events['severe_rtf_collapse_count'].get('paired_difference_95_bootstrap',{}).get('mean_ci')
    runtime_worse=bool(rtf_ci and collapse_ci and rtf_ci[1]<0 and collapse_ci[0]>0)
    runtime = dict(matched_metrics={k: continuous[k] for k in ('rtf_median','rtf_p10','rtf_p90','clock_mean_hz','wall_time_sec')},
                   matched_events={k: events[k] for k in ('severe_rtf_collapse_count','controller_rate_miss_count','clock_backward_jump_count')},
                   burden_correlations=correlations, systematic_runtime_deterioration=runtime_worse,
                   affected_runs=[{k:r.get(k) for k in ('seed','configuration','rtf_median','rtf_p10','severe_rtf_collapse_count',
                       'controller_rate_miss_count','controller_failure_count','collision_ahead_error_count')} for r in affected],
                   controller_failure_runs_at_healthy_median_rtf=[dict(seed=r['seed'],configuration=r['configuration'])
                        for r in affected if valid(r['rtf_median']) and r['rtf_median']>=0.90],
                   interpretation='Runtime does not excuse controller failures; whole-run median RTF '
                                  'does not prove RTF at the instant of failure.')
    current_week6=evidence_hashes()
    unchanged=current_week6==read(SPEC/'week6_evidence_inventory.json')
    manifest=read(SPEC/'implementation_fingerprint.json')
    hashes_ok=hashes()==manifest['sha256']
    contamination=[dict(seed=r['seed'],mode=r['configuration'],missing=r['missing_metrics'],
                        infrastructure_failure=r['infrastructure_failure']) for r in rows
                   if r['attempted'] and (r['infrastructure_failure'] or r['missing_metrics']
                        or not r['observer_capture_complete'] or not r['hashes_consistent'])]
    evidence_pass=attempted==60 and represented==30 and not contamination and unchanged and hashes_ok
    acceptance=dict(gate_a_evidence='PASS' if evidence_pass else 'INCONCLUSIVE',
        gate_b_completion=completion_gate, gate_c_false_intervention=false['gate'],
        gate_d_controller=controller_gate, gate_e_path=path_gate,
        gate_f_runtime='FAIL' if runtime_worse else 'PASS', evidence_contamination=contamination,
        week6_evidence_modified=not unchanged, implementation_hashes_consistent=hashes_ok,
        selective_reruns=0, fault_campaign_started=False)
    science_gates=[completion_gate,false['gate'],controller_gate,path_gate,acceptance['gate_f_runtime']]
    decision='FAIL' if 'FAIL' in science_gates else 'PASS' if evidence_pass and all(g=='PASS' for g in science_gates) else 'INCONCLUSIVE'
    acceptance.update(decision='WEEK6R_A: '+decision,
                      ready_for_week6r_b_fault_qualification=decision=='PASS',
                      healthy_controller_issue_resolved=decision=='PASS')
    anomalies=[r for r in rows if r['attempted'] and (not r['mission_completed'] or r['infrastructure_failure']
              or r['degraded_transitions'] or any((r.get(k) or 0)>0 for k in
                      ('controller_failure_count','collision_ahead_error_count','nav_recovery_count',
                       'controller_command_gap_count','nav2_abort_count','ares_gate_count','ares_probation_count')))]
    summary=dict(canonical_runs_expected=60,canonical_runs_attempted=attempted,matched_seed_pairs=represented,
                 completion=completion,false_intervention=false,continuous_metrics=continuous,
                 event_metrics=events,path_non_degradation=path,acceptance=acceptance,
                 limitations=['Only 30 pairs; conservative confidence intervals may be inconclusive.',
                     'Existing baseline AMCL and protected fusion differ; isolated causality is not proven.',
                     'Path metrics reference the active plan; they are not a collision-clearance guarantee.',
                     'Bootstrap point masses for zero events are not population-wide absence claims.'])
    save('healthy_qualification_summary.json',summary)
    save('matched_pair_results.json',dict(pairs=pairs,continuous_statistics=continuous,event_statistics=events))
    save('controller_stability_analysis.json',controller)
    save('runtime_quality_analysis.json',runtime)
    save('acceptance.json',acceptance)
    names=[('CONTROLLER_FAILURES','controller_failure_count'),('COLLISION_AHEAD_ERRORS','collision_ahead_error_count'),
           ('RECOVERIES','nav_recovery_count'),('COMMAND_GAPS','controller_command_gap_count'),('ABORTS','nav2_abort_count')]
    terminal=['WEEK6R_A_HEALTHY_CONTROLLER_QUALIFICATION','=========================================','',
              'CANONICAL_RUNS_EXPECTED: 60',f'CANONICAL_RUNS_ATTEMPTED: {attempted}',f'MATCHED_SEED_PAIRS: {represented}/30','',
              f'BASELINE_COMPLETION: {b_success}/30',f'ARES_COMPLETION: {a_success}/30','',
              f'ARES_FALSE_GATES: {gates}',f'ARES_FALSE_PROBATIONS: {probations}','']
    for label,key in names:
        terminal += [f"BASELINE_{label}: {events[key].get('baseline_total','UNAVAILABLE')}",
                     f"ARES_{label}: {events[key].get('ares_total','UNAVAILABLE')}",'']
    terminal += [f"PAIRED_MEAN_CROSS_TRACK_DIFFERENCE_M: {continuous['mean_cross_track_error_m'].get('paired_mean_difference','UNAVAILABLE')}",
                 f"PAIRED_P95_CROSS_TRACK_DIFFERENCE_M: {continuous['p95_cross_track_error_m'].get('paired_mean_difference','UNAVAILABLE')}",'',
                 'CONTROLLER_INSTABILITY_SYSTEMATICALLY_WORSE_WITH_ARES: '+('YES' if controller_gate=='FAIL' else 'NO' if controller_gate=='PASS' else 'INCONCLUSIVE'),
                 'PATH_QUALITY_NONDEGRADATION: '+path_gate,'COMPLETION_NONDEGRADATION: '+completion_gate,
                 'FALSE_INTERVENTION_GATE: '+false['gate'],'','WEEK6R_A: '+decision,
                 'READY_FOR_WEEK6R_B_FAULT_QUALIFICATION: '+('YES' if decision=='PASS' else 'NO'),'',
                 'WEEK6_EVIDENCE_MODIFIED: '+('NO' if unchanged else 'YES'),
                 'SELECTIVE_RERUNS: 0','FAULT_CAMPAIGN_STARTED: NO']
    terminal_text='\n'.join(terminal)+'\n'
    (C/'analysis/terminal_summary.txt').write_text(terminal_text)
    md=['# WEEK6R-A Healthy Controller Qualification','',
        '## 1. Experiment design','30 matched healthy pairs; baseline then ARES per frozen seed. '
        'No faults, no retries, no outcome-based tuning. Existing stabilized runtime and clock boundary. '
        'Passive observer preserves trust and controller diagnostics.',
        '## 2. Frozen seed list','`'+', '.join(map(str,seeds))+'`',
        '## 3. Implementation fingerprints',f"Frozen {len(manifest['sha256'])} source/installed files. "
        f"Manifest SHA256 `{sha(SPEC/'implementation_fingerprint.json')}`. Consistent: {hashes_ok}. "
        f"Week6 evidence unchanged: {unchanged}. See spec/pre_run_evidence_fingerprints.sha256 and implementation_fingerprint.json.",
        '## 4. Run completeness',f'{attempted}/60 attempts; {represented}/30 pairs. '
        f'Evidence gate: {acceptance["gate_a_evidence"]}; contamination records: {len(contamination)}. '
        'Missing values remain missing and are never replaced by success or zeros.',
        '## 5. Healthy completion comparison','```json',json.dumps(completion,indent=2),'```',
        '## 6. False gate/probation analysis',f'{gates} gates; {probations} probations. '
        f'DEGRADED transitions retained in {len(false["degraded_transitions"])} ARES runs. '
        'Initial DEGRADED observations are retained too. Full transitions are in matched_pair_results.json.',
        '## 7. Controller/collision analysis',criteria['systematic_rule'],
        f'Controller gate: {controller_gate}. ARES-only failure evidence: {json.dumps(repeated_seeds)}. '
        'Launch-log controller aborts/collision errors and mission counters are combined by maximum '
        'to avoid duplicate counting. Exact event logs and rosout diagnostics are retained per run.',
        '## 8. Path-quality comparison','```json',json.dumps(path,indent=2),'```',
        '## 9. Runtime-quality comparison','```json',json.dumps(runtime,indent=2),'```',
        '## 10. Anomalous-run table','| Seed | Mode | Completed | Controller | Collision | Recoveries | Gaps | Median RTF | Infrastructure |',
        '|---|---|---|---|---|---|---|---|---|']
    for r in anomalies:
        md.append('| '+' | '.join(str(r.get(k)) for k in ('seed','configuration','mission_completed',
                  'controller_failure_count','collision_ahead_error_count','nav_recovery_count',
                  'controller_command_gap_count','rtf_median','infrastructure_failure'))+' |')
    if not anomalies:
        md.append('| none | | | | | | | | |')
    md += ['## 11. Matched-pair statistics','Continuous metrics: means, medians, paired differences, '
           '95% bootstrap intervals, worse fractions and standardized paired effects. Event metrics: '
           'totals, affected runs, exact matched event-presence tables and paired bootstrap differences.',
           '| Metric | Baseline mean | ARES mean | Paired mean difference | 95% mean CI | Worse fraction |',
           '|---|---|---|---|---|---|']
    for k,s in (continuous|events).items():
        md.append('| '+k+' | '+' | '.join(str(s.get(field)) for field in
                   ('baseline_mean','ares_mean','paired_mean_difference'))+' | '+
                   str(s.get('paired_difference_95_bootstrap',{}).get('mean_ci'))+' | '+str(s.get('ares_worse_fraction'))+' |')
    md += ['## 12. Acceptance decision','```json',json.dumps(acceptance,indent=2),'```',
           '## 13. Whether the healthy-controller issue is resolved',
           'YES' if decision=='PASS' else 'NO. Qualification has not established all prerequisite gates.',
           '## 14. Whether proceeding to the 120-run fault campaign is justified',
           'YES; separately authorized future stage only.' if decision=='PASS' else 'NO. Do not start Week6R-B.',
           '## Limits',*summary['limitations'],'','```',terminal_text.rstrip(),'```','']
    with (C/'analysis/WEEK6R_A_HEALTHY_QUALIFICATION_REPORT.md').open('x') as f:
        f.write('\n\n'.join(md))
    print(terminal_text,end='')


if __name__=='__main__':
    main()
