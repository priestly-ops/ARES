# Week 6 Post-v7 Teardown Validation

## Scope and immutability

This work addresses only runtime ownership and campaign transitions. It does
not change scientific criteria or analyzer acceptance logic. No v8 campaign
was run. The existing v7 campaign tree was not modified.

## v7 transition failure evidence

The two reported v7 failures were:

- `healthy/healthy_unprotected_s2530`
- `gnss_step_5m/gnss_step_5m_protected_s2530`

For both pre-run isolation failures, the report found no registered
`ares_world`, no `/clock` publisher, and one `/ares/week6/raw_clock`
publisher. Neither report recorded stale process PIDs or remaining critical
ROS nodes. The elapsed time between the preceding canonical result and the
failed next isolation check was 10.082 seconds for
`healthy_unprotected_s2530` and 10.493 seconds for
`gnss_step_5m_protected_s2530`. The v7 run artifacts did not preserve a
post-run teardown timeline or the previous launch parent's exit status.

The preceding launch logs recorded `ros_gz_bridge` exiting with code `-11`
and `planner_server` and `controller_server` requiring SIGKILL (`-9`). These
facts are consistent with an incomplete shutdown / DDS graph-release tail;
the artifacts do not establish whether Gazebo itself required a signal, and
they do not establish that a duplicate Gazebo world remained registered.

## Post-run teardown architecture

`scripts/week6_runtime_isolation.py` now has a post-run teardown barrier.
`RunProcessTracker` records process identities, including PID start-time
identity, from the launch process group and the ARES run-id marker. Teardown
signals only those tracked processes, in order: SIGINT, bounded wait, SIGTERM,
bounded wait, then SIGKILL for any remaining owned identities. It then polls
critical ROS nodes, Gazebo world services, and `/clock` and raw-clock
publisher counts until it observes two consecutive clean snapshots or
reaches its bounded deadline.

Every runner invocation writes `post_run_teardown.json`, with result and
launch timing, tracked PIDs, survivors after each signal stage, clock
publisher samples, critical nodes, world presence, duration, and the final
ready status. A campaign cannot advance to another case unless both the
pre-run isolation report and prior post-run teardown report pass.

The pre-run isolation check retains fail-closed behavior and cleans only
processes attributed to the ARES Week 6 runtime. Based on the stress-cycle
evidence below, it now also polls for two consecutive clean process, node,
world, and publisher snapshots after cleanup, instead of treating one
immediate graph observation as final.

## Controlled teardown stress

The original failed stress attempt is preserved under
`results/week6/post_v7_runtime_diagnostics/teardown_stress_5x/`. It did not
reach startup readiness and remains a 0/5 historical attempt.

The initial isolation snapshot found 15 attributable processes from an old
`healthy_baseline_s2531` runtime, including its Week 6 launch, Gazebo,
bridge, Nav2, and clock-boundary processes; 10 critical ROS nodes were also
visible. The world probe did not find a registered `ares_world`, and
`/clock` had zero publishers. The gate issued 12 SIGINT cleanup actions but
failed closed because one raw-clock publisher was still visible in its
immediate verification snapshot.

The failed runner did not launch the requested protected stress stack. Its
post-run teardown report showed no tracked survivors, no critical-node
survivors, no world, and two later raw-clock samples at zero; the first
sample had one publisher. This exposed delayed raw-clock graph convergence
as the immediate cause of the failed pre-run gate.

## Why v2 is a new validation revision

The v2 validation used a distinct directory,
`results/week6/post_v7_runtime_diagnostics/teardown_stress_v2_5x/`; it did not
overwrite or rerun the original attempt. The runtime revision now records
bounded pre-run convergence duration and the first and second clean-snapshot
timestamps. Post-run reports likewise record both clean-snapshot timestamps
and per-sample `/clock` and raw-clock counts. The campaign gate still fails
closed when bounded convergence cannot be established.

Cycle 1 of v2 completed startup, shutdown, and post-run teardown successfully.
Its raw-clock samples were `1, 1, 0, 0`; the teardown barrier recorded two
consecutive clean snapshots and returned READY. The initial v2 aggregate
incorrectly rejected this successful convergence because it required every
sample to be zero. The validator was corrected to require that the last two
samples are zero and that both consecutive-clean timestamps exist. Cycle 1's
per-run evidence was not rerun or replaced; its original classifier outcome
is retained in the aggregate for auditability. Only cycles 2–5 were launched
after that correction.

## Teardown stress v2 evidence

The new stress validation passed **5/5**:

- world failures: 0
- `/clock` convergence failures: 0
- raw-clock convergence failures: 0
- stale critical-node failures: 0
- endpoint readiness failures: 0
- behavior-action discovery failures: 0
- lifecycle startup failures: 0

All five cycles started the protected Week 6 runtime, reached startup
readiness, sent no navigation goal, shut down, and passed the two-clean-
snapshot teardown barrier. Each cycle observed raw-clock publisher counts
`1, 1, 0, 0` and `/clock` counts `0, 0, 0, 0` during cleanup. Thus a briefly
visible DDS endpoint did not block the next run once it converged within the
bounded poll.

| Cycle | Pre-run convergence (s) | Teardown (s) | First / second clean snapshots (UTC) |
|---|---:|---:|---|
| 1 | 15.925 | 32.071 | Recorded in cycle 1 `post_run_teardown.json` |
| 2 | 16.497 | 31.777 | Recorded in cycle 2 `post_run_teardown.json` |
| 3 | 15.573 | 32.083 | Recorded in cycle 3 `post_run_teardown.json` |
| 4 | 15.589 | 32.305 | Recorded in cycle 4 `post_run_teardown.json` |
| 5 | 15.526 | 31.917 | Recorded in cycle 5 `post_run_teardown.json` |

For all cycles, survivors after SIGINT, SIGTERM, and SIGKILL were zero;
remaining critical nodes were empty and `ares_world` was absent. Per-cycle
reports contain the exact timestamps, publisher samples, detected stale
resources, and cleanup actions.

## Mixed-mode handoff v2

The gated startup-only sequence passed **5/5**, in the required order:

1. baseline / seed 2530
2. unprotected / seed 2530
3. protected / seed 2530
4. unprotected / seed 2530
5. protected / seed 2530

No navigation goals were sent. World-exclusivity failures, stale clock and
raw-clock failures, endpoint failures, behavior-action discovery failures,
and lifecycle startup failures were all zero. Every transition recorded
successful prior teardown, clean runtime release, next-run exclusivity, and
startup readiness.

## Mini campaign v2

After the 5/5 handoff pass, the exact five seed-2530 cases ran once:
healthy baseline, healthy unprotected, healthy protected, `gnss_step_5m`
unprotected, and `gnss_step_5m` protected. Runtime validation passed **5/5**
launches and **4/4** inter-run teardowns. World-exclusivity, clock/raw-clock,
endpoint, behavior-action, lifecycle, and duplicate-world failures were all
zero. No failed case was rerun.

Mission outcomes were recorded diagnostically: all three healthy missions
completed, the unprotected fault mission did not complete, and the protected
fault mission completed. These scientific outcomes were not used to tune or
change the system.

## Fail-closed campaign sequencing

An explicit campaign-runner test injects a teardown failure after case N and
verifies that the campaign stops, case N's result and teardown evidence are
preserved, and case N+1 is neither launched nor given a synthetic
`result.json`. **NEXT_CASE_BLOCKED_ON_TEARDOWN_FAILURE: YES.**

## Regression results

The targeted runtime isolation, teardown ownership, startup barrier,
mode-specific readiness, campaign runner, analyzer robustness, and Week 4 /
Week 5 regression tests passed: **100 passed**. Python syntax checks passed
for the modified runtime orchestration scripts and tests.

## v8 readiness

All v8 runtime prerequisites specified for this validation passed:
teardown stress 5/5, mixed-mode handoff 5/5, mini-campaign launches 5/5 and
inter-run teardowns 4/4, with zero isolation, clock, raw-clock, endpoint,
behavior-action, lifecycle, and duplicate-world failures. The fail-closed
test passed. Scientific criteria and analyzer acceptance logic remain
unchanged; v7 and the original failed stress evidence remain immutable.

**READY_FOR_V8_CAMPAIGN: YES.** No v8 campaign was run. **WEEK6_STATUS:
BLOCKED** pending the separately authorized final scientific acceptance
campaign.
