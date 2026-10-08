# Week 6 V5 Runtime Regression Investigation

## Executive finding

`final_campaign_v5` contains two distinct failure mechanisms and is therefore classified **MIXED**:

1. The healthy baseline launch had a real, mode-specific orchestration defect: `nav2_week6_upstream` did not launch the `bt_navigator` process. Its barrier discovered all ten required actions/services and found every upstream node plus AMCL active, but `/bt_navigator/get_state` was unavailable. This reproduced in the required mini-campaign and was fixed without changing scientific configuration.
2. All ten seed-2531/2532 launches ran under a restricted execution profile that denied network interface and UDP socket system calls. Fast DDS reported `getifaddrs: Operation not permitted` and `Error creating socket: Operation not permitted` in every run. These failures occurred before endpoint readiness could be measured and are not DDS cleanup or GNSS-protection failures.

The first socket-denied launch followed a 12,676-second gap. A later controlled protected startup sequence passed 3/3 with no stale process or external ROS graph nodes after shutdown. The catastrophic ten-run block is therefore not sequence-dependent.

## 1. V5 failure matrix

The machine-readable matrix is preserved at `results/week6/final_campaign_v5/startup_failure_matrix.json`. Original v5 did not take a contemporaneous process snapshot after each shutdown, so the matrix records that field as unknown rather than inventing a result. Structured launch logs provide start/end events, shutdown durations, and forced-exit evidence.

| # | Seed | Mode | Scenario | Runner | Mission started | Barrier started | Endpoint readiness | Missing endpoint/lifecycle evidence | BT activation | Shutdown (s) | Gap from prior last event (s) |
|---:|---:|---|---|---:|:---:|:---:|:---:|---|:---:|---:|---:|
| 1 | 2530 | baseline | healthy | 1 | YES | YES | FAIL | No action/service missing; `/bt_navigator/get_state` unavailable | NO/NO | 10.019 | — |
| 2 | 2530 | unprotected | healthy | 0 | YES | YES | READY | None | YES/YES | 10.240 | 65.180 |
| 3 | 2530 | protected | healthy | 0 | YES | YES | READY | None | YES/YES | 10.114 | 0.835 |
| 4 | 2530 | unprotected | gnss_step_5m | 1 | YES | YES | READY | None | YES/YES | 10.317 | 0.885 |
| 5 | 2530 | protected | gnss_step_5m | 1 | YES | YES | READY | None | YES/YES | not logged | 0.800 |
| 6 | 2531 | baseline | healthy | 1 | NO | YES | FAIL | Unobserved: barrier emitted no record | NO/NO | 0.202 | 12676.198 |
| 7 | 2531 | unprotected | healthy | 1 | NO | NO | FAIL | Unobserved: barrier emitted no record | NO/NO | 0.423 | 1.215 |
| 8 | 2531 | protected | healthy | 1 | NO | NO | FAIL | Unobserved: barrier emitted no record | NO/NO | 2.050 | 1.334 |
| 9 | 2531 | unprotected | gnss_step_5m | 1 | NO | NO | FAIL | Unobserved: barrier emitted no record | NO/NO | 0.640 | 1.361 |
| 10 | 2531 | protected | gnss_step_5m | 1 | NO | NO | FAIL | Unobserved: barrier emitted no record | NO/NO | 0.469 | 1.317 |
| 11 | 2532 | baseline | healthy | 1 | NO | YES | FAIL | Unobserved: barrier emitted no record | NO/NO | 0.220 | 1.436 |
| 12 | 2532 | unprotected | healthy | 1 | NO | NO | FAIL | Unobserved: barrier emitted no record | NO/NO | 0.425 | 1.456 |
| 13 | 2532 | protected | healthy | 1 | NO | NO | FAIL | Unobserved: barrier emitted no record | NO/NO | 1.955 | 1.253 |
| 14 | 2532 | unprotected | gnss_step_5m | 1 | NO | NO | FAIL | Unobserved: barrier emitted no record | NO/NO | 2.307 | 1.341 |
| 15 | 2532 | protected | gnss_step_5m | 1 | NO | NO | FAIL | Unobserved: barrier emitted no record | NO/NO | 4.842 | 1.628 |

For runs 6–15, all five lifecycle confirmations were absent because ROS participants could not initialize. This is not evidence that a particular server was inactive or that a particular action/service failed discovery; the barrier never produced a readiness sample. Runs 6 and 11 started the baseline barrier immediately, but it also failed Fast DDS initialization. The non-baseline barrier is intentionally delayed three seconds and was never started before those launches collapsed.

## 2. Sequence dependence

**FAILURE_IS_SEQUENCE_DEPENDENT: NO.**

- The first v5 run had the baseline-only missing-process defect; the next four seed-2530 launches reached endpoint readiness despite gaps below one second between runs 2–5.
- The first socket-denied failure was run 6 after a 12,676-second gap, which excludes a short inter-run gap or accumulated participant count as its cause.
- Runs 6–15 failed in every mode and scenario across two seeds with the same operating-system permission error.
- The controlled protected startup sequence later passed 3/3 sequentially under the same ROS configuration.
- The clock-boundary process failed alongside every other ROS participant in the restricted runs; no stale clock boundary was found afterward.

Launch count, seed, scenario, and GNSS protection mode do not explain the ten-run failure block. Execution isolation does.

## 3. Stale-process analysis

**STALE_PROCESSES_FOUND: NO.**

The original v5 runner did not record post-return `ps` snapshots, so exact after-each-run survivor PIDs cannot be reconstructed. Its structured logs show bounded teardown, and later preflight found no active ROS/Gazebo/Nav2 processes. The new controlled checks inspected:

- `gz` and `parameter_bridge`;
- planner, controller, behavior, and BT servers;
- lifecycle managers and EKF;
- trust/recovery nodes;
- startup barrier and clock boundary.

No listed process survived runner return after any controlled attempt, the mini-campaign, or the targeted baseline-fix validation.

`planner_server` and `controller_server` require the runner's bounded escalation to SIGKILL during shutdown. This occurred in all three passing A2 runs, all three passing repeated-startup attempts, every mini-campaign run, and the passing baseline-fix check. It is controlled teardown, not a stale-process difference: the launch process observes their final exits before `stop_process()` returns.

## 4. Fast DDS participant cleanup

The cleanup measurements are preserved at `results/week6/post_v5_runtime_diagnostics/repeated_startup_3x/cleanup_measurements.json`.

| Sample | External graph nodes | Critical processes |
|---|---:|---:|
| Immediately after attempt 1 | 0 | 0 |
| Immediately after attempt 2 | 0 | 0 |
| After attempt 3, first observation | 0 | 0 |
| 1-second target | 0 | 0 |
| 2-second target | 0 | 0 |
| 5-second target | 0 | 0 |
| 10-second target | 0 | 0 |

The raw delayed samples contain one `/_ros2cli_*` node belonging to the observer itself; external node count is zero. The ROS daemon was not running. No next launch began while a prior critical node remained visible.

**DDS_CLEANUP_LATENCY_CAUSAL: NO.** No settle delay or arbitrary sleep was added.

## 5. A2/B2 versus V5 invocation

| Property | A2 startup | B2 full run | V5 campaign |
|---|---|---|---|
| Per-run implementation | `run_week6_navigation.py` | Same | Same child runner via `run_week6_campaign.py` |
| Working directory | Workspace root | Workspace root | Workspace root |
| RMW | `rmw_fastrtps_cpp` | Same | Same |
| ROS domain | Not recorded; no differing value evidenced | Same limitation | Same limitation |
| Namespace | None | None | None |
| Launch arguments | Mode/scenario/seed, RViz false | Same shape | Same shape for each matrix case |
| Startup barrier | Same installed A2 implementation | Same | Same |
| Lifecycle order | Upstream active → endpoint barrier → BT lifecycle manager | Same | Same; baseline defect omitted BT process |
| Clock boundary | 100 Hz Week 6 boundary | Same | Same |
| Cleanup | Same `stop_process()` process-group escalation | Same | Same per-run child cleanup |
| ROS daemon dependence | None required | None required | None required; later controlled sequence passed with daemon stopped |
| Result path | A2 diagnostic directory | B2 diagnostic directory | Campaign case directory only |
| Parent execution | Direct startup-only runner | Direct full runner | Sequential wrapper using inherited interpreter/environment |
| Material runtime difference | Network/DDS operations permitted | Network/DDS operations permitted | Runs 6–15 executed under a profile denying interface/UDP socket syscalls |

The initial v5 wrapper invocation without `install/setup.bash` is separately preserved as `interruption_001.json`; it created no canonical runs and is not one of the 15 outcomes. The subsequent canonical invocations had the workspace sourced.

## 6. Endpoint failure distribution

No required action or service is the recurring missing endpoint:

- Seed-2530 baseline: all seven actions and three services were discovered. The missing prerequisite was the owner process's lifecycle service, `/bt_navigator/get_state`, because baseline orchestration never launched `bt_navigator`.
- Runs 6–15: exact endpoint names are unknowable because the barrier emitted no readiness JSON. Fast DDS participant construction failed first.
- Controlled protected attempts: all ten endpoints ready 3/3.
- Post-fix baseline startup: all ten endpoints ready, including both costmap clear services and `/is_path_valid`.

**MOST_COMMON_MISSING_ENDPOINT: NONE_OBSERVED.** Treating all ten unobserved endpoints as missing would overstate the evidence.

## 7. Analyzer execution failure

**ANALYZER_EXECUTION_BUG_FOUND: YES.** The result files are valid JSON. Startup-failure results intentionally omit navigation metrics that cannot exist before mission start. The frozen analyzer directly indexed `run['max_localization_error_m']` while evaluating `protected_fault_localization_peak_improves`, raising:

```text
KeyError: 'max_localization_error_m'
```

The execution-only fix adds complete finite-vector and completed-waypoint
evidence guards, plus null-safe aggregation for numeric runtime fields. If a
required localization or waypoint metric is unavailable, its unchanged check
fails instead of crashing. Missing optional runtime metrics remain optional.
No criterion or threshold was weakened.

Analyzer integration tests now cover six requested result shapes: a fully
accepted campaign, a scientific navigation failure, a startup-readiness
failure, a lifecycle failure, null waypoint metrics, and missing optional
runtime metrics. A seventh helper test retains NaN/infinity coverage. Failed
evidence returns `BLOCKED`; valid accepted evidence returns `PASS`.

After the fix, the analyzer executes and returns `BLOCKED` with these existing checks failed:

- `all_healthy_missions_complete`
- `protected_fault_missions_complete_consistently`
- `protected_fault_completion_exceeds_unprotected`
- `protected_fault_localization_peak_improves`
- `protected_fault_executes_gate_and_probation`

## 8. Repeated-startup reproduction

The exact protected healthy seed-2530 startup-only configuration ran three times sequentially under `results/week6/post_v5_runtime_diagnostics/repeated_startup_3x/`.

| Attempt | Startup | Barrier elapsed (wall s) | Missing endpoints | Missing BTs | Stale processes |
|---:|:---:|---:|---:|---:|---:|
| 1 | PASS | 1.239 | 0 | 0 | 0 |
| 2 | PASS | 1.700 | 0 | 0 | 0 |
| 3 | PASS | 1.630 | 0 | 0 | 0 |

Every attempt observed planner/controller/behavior actions, `/is_path_valid`, both costmap clear services, upstream ACTIVE lifecycle states, and `bt_navigator` unconfigured before activation. All then activated `bt_navigator` and created both BTs.

**REPEATED_STARTUP_3X_PASS: 3/3.** The v5 ten-run discovery pattern was not reproduced.

## 9. Orchestration fix

The repeated protected sequence required no cleanup or graph-settle change. The real baseline defect reproduced independently and received the minimal fix in `ares_baseline.launch.py`:

- launch `bt_navigator` in `nav2_week6_upstream`;
- keep it out of the upstream lifecycle manager's `node_names`;
- leave activation exclusively to the existing Week 6 barrier-controlled BT lifecycle manager.

No sleep was added. No mission, estimator, trust, fault, map, controller,
planner, or analyzer acceptance parameter changed. Both affected packages
rebuilt successfully. The final targeted startup/lifecycle/mode/campaign/
analyzer/navigation/Week4/Week5 suite passed 73/73 tests.

A separate targeted baseline startup-only validation passed after the fix. Barrier readiness took 5.149 wall seconds, all upstream nodes plus AMCL were ACTIVE, `bt_navigator` was unconfigured before activation, all ten endpoints were ready, activation succeeded, and both BTs were created. No critical process or external ROS graph node remained after shutdown.

## 10. Mini-campaign validation

The required single five-run mini-campaign is preserved under `results/week6/post_v5_runtime_diagnostics/mini_campaign_seed2530/`. It was not rerun or replaced after the baseline fix.

| Run | Endpoint readiness | BT creation | Mission started | Scientific result (not acceptance evidence here) |
|---|:---:|:---:|:---:|---|
| healthy / baseline | FAIL | FAIL | YES | readiness failure |
| healthy / unprotected | READY | PASS | YES | 5/5 complete |
| healthy / protected | READY | PASS | YES | 5/5 complete |
| gnss_step_5m / unprotected | READY | PASS | YES | 1/2, expected unprotected fault failure shape |
| gnss_step_5m / protected | READY | PASS | YES | 5/5 complete |

The baseline failure is the defect fixed in section 9. The other four launches ran back-to-back with sub-second transitions, reached readiness, created both BTs, and started their missions. No socket-permission failure occurred.

**MINI_CAMPAIGN_LAUNCHES_PASS: 4/5**  
**MINI_CAMPAIGN_ENDPOINT_FAILURES: 1**  
**MINI_CAMPAIGN_BT_FAILURES: 1**

## 11. Exact failed mini case and causal timeline

The only failed launch in the original mini-campaign was
`healthy/healthy_baseline_s2530`.

| Field | Evidence |
|---|---|
| Mode / scenario | baseline / healthy |
| Runner exit | 1 |
| Barrier | started; failed after 60.004 wall seconds |
| Missing action/service | none |
| Missing lifecycle service | `/bt_navigator/get_state` |
| Upstream states | map, planner, controller, behavior, and AMCL active |
| BT Navigator process | absent |
| BT Navigator expected | YES |
| BT activation attempted | NO |
| BT creation | NO |
| Failure text | `Week 6 Nav2 startup readiness barrier failed; bt_navigator will remain inactive` |
| Mission result | `runner_error: Week 6 readiness timed out; lifecycle states={}` |

The evidence timeline is:

1. Launch created PIDs 16848-16864, including the barrier, but there was no
   BT Navigator process entry.
2. At ROS time 1791328918.750, the upstream manager began configuration.
3. At ROS time 1791328923.163, map, AMCL, planner, controller, and behavior
   were active.
4. Barrier-relative endpoint discovery completed between 1.471 and 4.573
   wall seconds; all seven actions and three services were ready.
5. At barrier elapsed 60.004 seconds, `bt_navigator` remained
   `SERVICE_UNAVAILABLE`, so the barrier returned failure.
6. At ROS time 1791328978.039, launch teardown began. The separate BT manager
   had never started, no BT activation was attempted, and neither BT was
   created.
7. At ROS time 1791329052.504, the mission runner preserved its readiness
   timeout result.

The endpoint failure and BT failure are the same root event: the baseline
stage omitted the BT process. **MINI_FAILURE_SINGLE_ROOT_CAUSE: YES.**

## 12. Baseline fix and mode-aware readiness

Baseline is supposed to launch BT Navigator. This was launch-configuration
case E: not an intentional omission, namespace difference, early query, or
pre-service crash. The fix launches `bt_navigator` in
`nav2_week6_upstream`, excludes it from that stage's upstream managed list,
and leaves activation to the existing post-barrier BT lifecycle manager.

Readiness now takes an explicit `--navigation-mode` manifest. Baseline
requires AMCL plus map/planner/controller/behavior active; unprotected and
protected require map/planner/controller/behavior. All three use the same
frozen ten BT endpoints because they share the same Nav2 parameters and BTs.
The complete launch and endpoint comparison is in
`docs/WEEK6_MODE_STARTUP_MATRIX.md`.

**BT_NAVIGATOR_EXPECTED_IN_BASELINE: YES**  
**BASELINE_BT_GET_STATE_ROOT_CAUSE: BT process omitted by the baseline Week 6 upstream launch stage**  
**MODE_SPECIFIC_READINESS_IMPLEMENTED: YES**

## 13. Analyzer robustness validation

The original exception was `KeyError: 'max_localization_error_m'`: startup
failure results validly lacked metrics that cannot exist before mission
execution. The analyzer now treats absent/null required metrics as unavailable
failed evidence and never promotes a failed run to success. It also avoids
`float(None)` for waypoint evidence and null integer aggregation failures.

The analyzer completed on the original 15 v5 results and returned `BLOCKED`
with the same five failed acceptance checks. It also consumed the fresh mixed
mini-campaign results without error. The acceptance check identifiers,
thresholds, completion logic, and failed-check semantics are unchanged.

**ANALYZER_EXECUTION_FIXED: YES**  
**ANALYZER_ACCEPTANCE_LOGIC_CHANGED: NO**

## 14. Controlled mode startup matrix

The three fresh startup-only runs are preserved under
`results/week6/post_v5_runtime_diagnostics/mode_startup_matrix/`.

| Mode | Result | Barrier elapsed (wall s) | Endpoint failures | BT failures |
|---|:---:|---:|---:|---:|
| baseline | PASS | 5.354 | 0 | 0 |
| unprotected | PASS | 1.546 | 0 | 0 |
| protected | PASS | 1.245 | 0 | 0 |

Every mode matched its manifest, observed BT Navigator unconfigured before
activation, then activated it and created both BTs. No failure was rerun.

**MODE_STARTUP_MATRIX_PASS: 3/3**  
**MODE_STARTUP_ENDPOINT_FAILURES: 0**  
**MODE_STARTUP_BT_FAILURES: 0**

## 15. Fresh mini-campaign v2

The new five-run sequence is preserved separately under
`results/week6/post_v5_runtime_diagnostics/mini_campaign_seed2530_v2/`.

| Run | Mission started | Readiness | BT creation | Lifecycle startup | Scientific outcome (not the runtime gate) |
|---|:---:|:---:|:---:|:---:|---|
| healthy / baseline | YES | READY | PASS | PASS | 5/5 complete |
| healthy / unprotected | YES | READY | PASS | PASS | 5/5 complete |
| healthy / protected | YES | READY | PASS | PASS | 5/5 complete |
| gnss_step_5m / unprotected | YES | READY | PASS | PASS | 1/2, mission incomplete |
| gnss_step_5m / protected | YES | READY | PASS | PASS | 5/5 complete |

All five launch logs contain a successful readiness record and both BT
creation records. No startup barrier failure, BT activation failure, or
lifecycle startup failure is present. The unprotected fault result is a
scientific outcome after successful startup and was not rerun. The hardened
analyzer consumed all five schemas and completed; its PASS is diagnostic for
this one-seed result set, not Week 6 scientific acceptance.

**MINI_CAMPAIGN_V2_LAUNCHES_PASS: 5/5**  
**MINI_CAMPAIGN_V2_ENDPOINT_FAILURES: 0**  
**MINI_CAMPAIGN_V2_BT_FAILURES: 0**  
**MINI_CAMPAIGN_V2_LIFECYCLE_FAILURES: 0**

## 16. V6 readiness decision

The runtime prerequisites for a future v6 campaign are now satisfied:

- mode startup matrix: 3/3;
- sequential mini-campaign launch reliability: 5/5;
- endpoint, BT, and lifecycle startup failures: zero;
- successful and failed result schemas execute through the analyzer; and
- scientific criteria remain unchanged.

Therefore a separately authorized v6 campaign is justified, but none was run
in this investigation. Week 6 remains blocked pending that scientific
campaign and may not be declared passed from these runtime diagnostics.

**SCIENTIFIC_CRITERIA_CHANGED: NO**  
**READY_FOR_V6_CAMPAIGN: YES**  
**WEEK6_STATUS: BLOCKED**
