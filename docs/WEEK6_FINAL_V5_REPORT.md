# Week 6 Final Immutable V5 Campaign Report

## Verdict

**WEEK6: BLOCKED**  
**NAVIGATION_READINESS: NOT_READY**

The v5 campaign contains all 15 canonical `result.json` files in the required matched-seed order. The frozen analyzer did not produce a normal acceptance verdict: the required command exited with code 1 because startup-failure results do not contain `max_localization_error_m`. The analyzer was not modified or rerun. Accordingly, the frozen-analyzer verdict and the Week 6 verdict are **BLOCKED**.

## Freeze audit

`results/week6/final_campaign_v5/freeze_audit.json` records `audit_verdict: PASS` before campaign execution and zero scientific runs created before audit passage. The following preflight conditions were confirmed:

- no ROS, Gazebo, or Nav2 processes were active;
- the validated A2 startup barrier and B2 mission fixes were installed;
- the Week 6 startup readiness barrier was enabled;
- Week 6 selected `rmw_fastrtps_cpp`;
- the `/ares/week6/raw_clock` to `/clock` boundary remained limited to 100 Hz;
- the frozen waypoint acceptance threshold remained 0.30 m;
- the analyzer SHA-256 remained `f95f28b180e324e21575fab0e23b809a532ebaa8ab2a44623cc4e10aefce0d48`;
- every current implementation hash matched the freeze audit;
- the preserved v2, v3, and v4 file counts and root mtimes remained identical to their audited values; and
- the focused regression suite passed again: 29 passed.

Scientific criteria were not changed. The analyzer was not changed. No completed canonical run was rerun.

## Execution and interruption handling

The orchestration resumed an interrupted v5 campaign. The initial pre-run environment interruption is documented by `interruption_001.json`. A later session interruption occurred during the seed-2530 protected-fault invocation after its canonical result had been produced; `interruption_002.json` documents the resume state. Five seed-2530 canonical results already existed and were skipped by the existing campaign runner. The remaining ten canonical runs were executed once in the required order for seeds 2531 and 2532. All ten produced canonical results and were therefore preserved without reruns.

Those ten resumed runs failed during launch because the execution environment denied Fast DDS network/socket operations (`Error creating socket: Operation not permitted` and `getifaddrs: Operation not permitted`). Each records `runner_error: launch exited early with code 0`, failed startup diagnostics, five missing lifecycle confirmations, and both missing behavior-tree creations. These are orchestration-environment failures rather than valid navigation experiments, but the canonical-result rule makes them completed failures and prohibits rerunning them.

## All canonical run outcomes

| Seed | Scenario / mode | Mission | Waypoints | Gates | Probations | Outcome |
|---:|---|:---:|:---:|---:|---:|---|
| 2530 | healthy / baseline | FAIL | — | — | — | Week 6 readiness timeout; empty lifecycle state set |
| 2530 | healthy / unprotected | PASS | 5/5 | 0 | 0 | Mission completed |
| 2530 | healthy / protected | PASS | 5/5 | 0 | 0 | Mission completed; no false gating |
| 2530 | gnss_step_5m / unprotected | FAIL | 1/2 | 1 | 1 | Incomplete waypoints; Nav2 goal aborted |
| 2530 | gnss_step_5m / protected | FAIL | 2/2 | 1 | 0 | Timer creation failed after waypoint evidence; mission not completed |
| 2531 | healthy / baseline | FAIL | — | — | — | Launch exited early; startup endpoints and BTs unavailable |
| 2531 | healthy / unprotected | FAIL | — | — | — | Launch exited early; startup endpoints and BTs unavailable |
| 2531 | healthy / protected | FAIL | — | — | — | Launch exited early; startup endpoints and BTs unavailable |
| 2531 | gnss_step_5m / unprotected | FAIL | — | — | — | Launch exited early; startup endpoints and BTs unavailable |
| 2531 | gnss_step_5m / protected | FAIL | — | — | — | Launch exited early; startup endpoints and BTs unavailable |
| 2532 | healthy / baseline | FAIL | — | — | — | Launch exited early; startup endpoints and BTs unavailable |
| 2532 | healthy / unprotected | FAIL | — | — | — | Launch exited early; startup endpoints and BTs unavailable |
| 2532 | healthy / protected | FAIL | — | — | — | Launch exited early; startup endpoints and BTs unavailable |
| 2532 | gnss_step_5m / unprotected | FAIL | — | — | — | Launch exited early; startup endpoints and BTs unavailable |
| 2532 | gnss_step_5m / protected | FAIL | — | — | — | Launch exited early; startup endpoints and BTs unavailable |

## Aggregate outcome

- Canonical results: 15/15
- Healthy baseline completion: 0/3
- Healthy unprotected completion: 1/3
- Healthy protected completion: 1/3
- Fault unprotected completion: 0/3
- Fault protected completion: 0/3
- Protected fault runs with at least one gate: 1/3
- Protected fault runs with at least one probation: 0/3
- False healthy protected gates: 0
- Explicit BT discovery/creation failures: 10
- Endpoint-readiness failures: 11 (one seed-2530 readiness timeout and ten early-launch startup failures)
- Nav2-success/evaluator mismatches: 0

## Frozen analyzer outcome

The required command was run exactly:

```text
python3 scripts/analyze_week6_campaign.py --campaign-dir results/week6/final_campaign_v5
```

It exited with code 1 at `protected_fault_localization_peak_improves`, where the frozen analyzer indexed `max_localization_error_m` in the startup-failure records. The exception was `KeyError: 'max_localization_error_m'`. Because execution stopped before the write phase, this campaign has no analyzer-generated `summary.json`, `acceptance.json`, or `matched_seed_comparison.json`; no substitute analyzer output was fabricated. `analyzer_failure.json` preserves the command, frozen hash, exception, exit code, and absent-output state. The campaign metadata and freeze audit remain preserved.

Failed checks/outcomes are therefore reported as:

- frozen analyzer execution failed before acceptance checks were emitted;
- all healthy missions did not complete;
- protected fault missions did not complete consistently;
- protected fault completion did not exceed unprotected completion;
- protected fault gate-and-probation behavior was not demonstrated across 3/3 seeds; and
- the campaign does not establish navigation readiness.
