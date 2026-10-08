# Week 6 Post-v6 Runtime Fix

Date: 2026-10-06  
Status: **Runtime smoke checks passed; Week 6 remains BLOCKED pending a new
final campaign.**

This report addresses only the two startup/runtime failures in immutable
`results/week6/final_campaign_v6/`. It does not change scientific acceptance
criteria or analyzer logic. No v6 run was rerun, and no full campaign was
started.

## 1. Baseline seed-2531 v6 failure

The v6 healthy/baseline/seed-2531 run never started its mission. Its result
records:

```text
runner_error: Week 6 readiness timed out; lifecycle states={}
```

The launch log directly confirms that Gazebo rejected the attempted world:

```text
Another world of the same name is running
```

The same run then logged seven TF-buffer backward-time resets and repeated
`Invalid frame ID "map"` errors (120 occurrences). Its eventual readiness
sample had inactive controller and behavior servers, a planner lifecycle
query timeout, and a missing `/is_path_valid` service. Those downstream
failures are consequences of the contaminated simulation startup, not a
scientific baseline result.

**DUPLICATE_WORLD_CONFIRMED: YES.** The duplicate-world message is explicit
Gazebo evidence; the backward clock and map-frame errors are consistent with
two simulation/world lifetimes overlapping.

## 2. World exclusivity and pre-run isolation

Every invocation of `scripts/run_week6_navigation.py` now runs
`scripts/week6_runtime_isolation.py` before starting `ros2 launch`. It writes
`runtime_isolation.json` in that run's output directory and prints
`WORLD_EXCLUSIVITY_READY: YES` only after all checks pass.

The gate records:

- processes tied to a Week 6 launch root, its descendants, the per-run
  `ARES_WEEK6_RUN_ID`, an ARES `ares_world` SDF, or the Week 6 raw-clock bridge;
- critical ROS nodes, using a daemon-free node-graph query;
- `/clock` and `/ares/week6/raw_clock` publisher counts;
- Gazebo services for a world named `ares_world`;
- cleanup actions, before/after observations, errors, and the final
  exclusivity decision.

Cleanup first sends SIGINT to an isolated Week 6 launch process group, or to
the specifically identified owned PIDs when there is no isolated group. It
verifies disappearance and escalates only for those same identified PIDs.
Unrelated process groups are never signalled. If a critical ROS node or clock
publisher remains without attributable Week 6 ownership, or if a probe cannot
establish the required state, the gate fails closed before Gazebo is launched.
No arbitrary sleep is used as the readiness criterion.

## 3. Protected seed-2531 v6 behavior-action failure

The immutable protected/fault/seed-2531 v6 launch log records
`behavior_server` as ACTIVE and its dedicated ROS log records all four
plugins—`spin`, `backup`, `drive_on_heading`, and `wait`—created, configured,
and activated. However, for the complete existing 60-second barrier interval,
the correctly typed clients did not discover `/spin`, `/wait`, `/backup`, or
`/drive_on_heading`. The other required navigation actions and services were
discoverable. `bt_navigator` correctly remained inactive.

**BEHAVIOR_SERVER_ACTIVE_BUT_ACTIONS_MISSING_CONFIRMED: YES.**

The evidence does not indicate a missing plugin or failed plugin activation.
It demonstrates that lifecycle ACTIVE is not an endpoint-readiness guarantee:
the lifecycle transition can succeed while the action endpoints are not
available to the ROS graph. The v6 artifacts establish the failure at the
action registration/discovery boundary but do not identify a unique
historical DDS cause. In particular, the v6 run did not retain enough action
service graph information to distinguish absent action services from a
namespace/type discovery discrepancy.

The barrier now uses actual `rclpy.action.ActionClient` instances for these
exact Nav2 action types:

| Endpoint | Action type |
|---|---|
| `/spin` | `nav2_msgs/action/Spin` |
| `/backup` | `nav2_msgs/action/BackUp` |
| `/drive_on_heading` | `nav2_msgs/action/DriveOnHeading` |
| `/wait` | `nav2_msgs/action/Wait` |

The Week 6-only Nav2 file explicitly lists the same four behavior plugin names
and types observed in v6. This makes plugin configuration auditable without
changing the production Week 4/5 configuration or behavior tuning.

## 4. Startup sequencing and failure diagnostics

The existing bounded 60-second readiness timeout is unchanged. The barrier
now advances through explicit readiness phases:

1. Verify all mode-required upstream lifecycle nodes are ACTIVE while
   `bt_navigator` remains inactive.
2. Verify all four behavior plugins are listed with their expected plugin
   types, the behavior server is ACTIVE, and all four typed behavior action
   clients can discover their servers.
3. Verify planner/controller actions and costmap/path-validity services.
4. Return success; only then does the launch start BT Navigator activation.

On timeout, BT Navigator stays inactive and the barrier records the failed
phase. Per behavior action it records lifecycle state, endpoint YES/NO, first
discovery wall time, first discovery simulation time when `/clock` is
available, and plugin configured/activated YES/NO. It also records discovered
action graph names/types and the plugin parameter values. An unavailable
diagnostic query is recorded as unknown rather than treated as success.

## 5. Baseline seed-2531 smoke

The one authorized healthy/baseline/seed-2531 smoke is preserved in
`results/week6/post_v6_runtime_diagnostics/baseline_s2531_smoke/`.

- Isolation passed with no stale Week 6 process/node, duplicate world, or
  `/clock`/raw-clock publisher.
- Map server, planner, controller, behavior server, AMCL, and BT Navigator
  activated; required actions and services were discovered before BT
  activation.
- The mission sent its first goal and completed **5/5 waypoints**.
- The launch log contains no duplicate-world message and no backward clock
  reset. One transient initial `map` frame lookup warning occurred; map and
  planner subsequently became ready and navigation completed.

**BASELINE_S2531_SMOKE: PASS**  
**BASELINE_FIRST_GOAL_SENT: YES**  
**BASELINE_MISSION_COMPLETE: YES**

The saved baseline readiness record predates the subsequent best-effort
`/clock` subscription adjustment, so its first-discovery simulation-time fields
are null. The protected smoke below records those simulation times.

## 6. Protected fault seed-2531 smoke

Because the baseline smoke passed isolation and startup, exactly one
protected/`gnss_step_5m`/seed-2531 smoke was run and preserved in
`results/week6/post_v6_runtime_diagnostics/protected_fault_s2531_smoke/`.

- World exclusivity passed; no duplicate-world message or backward clock
  reset was observed.
- `behavior_server` was ACTIVE; all four plugins were explicitly confirmed
  configured and activated.
- All four behavior actions, the planner/controller actions, and all required
  services were ready before BT activation. The typed behavior action clients
  recorded first discovery at approximately 0.008–0.050 wall seconds after
  the barrier began and at simulation time 1.096–1.115 seconds.
- The mission sent its first goal and completed **5/5 waypoints**.
- The fault ran from mission time 15.004 to 45.007 seconds. The preserved
  logs contain nine pre-fusion rejection events. Recorded trust/recovery
  progression includes `HEALTHY -> DEGRADED -> UNTRUSTED`, then `GATED` at
  simulation time 35.901, `PROBATION` at 66.401, and `NORMAL` at 69.298.
  Final recovery state was NORMAL and final GNSS trust was HEALTHY.
- Localization error was 0.151 m at mission start, at most 0.286 m during the
  fault window, and at most 0.414 m over the mission. These are smoke
  diagnostics only and do not modify or replace the frozen acceptance
  evaluation.

**PROTECTED_FAULT_S2531_SMOKE: PASS**  
**PROTECTED_BEHAVIOR_ACTIONS_DISCOVERED: 4/4**  
**PROTECTED_FIRST_GOAL_SENT: YES**  
**PROTECTED_MISSION_COMPLETE: YES**

The post-run isolation check also passed with zero stale Week 6 processes and
zero critical ROS nodes remaining.

## 7. Regression tests and campaign readiness

The targeted regression set passed:

- runtime isolation, startup barrier, behavior-action readiness, and
  mode-specific readiness;
- campaign runner and analyzer robustness; and
- Week 4/Week 5 analysis, fusion configuration, and recovery-manager
  regressions.

**Regression result: 90 passed.**

The two requested runtime failures did not recur in their one-shot smokes.
World exclusivity is enforced before every future Week 6 launch, and BT
activation is blocked until real typed behavior-action readiness has passed.
This supports runtime readiness for a separately authorized final campaign;
it is not evidence that such a campaign passes.

**SCIENTIFIC_CRITERIA_CHANGED: NO**  
**ANALYZER_ACCEPTANCE_LOGIC_CHANGED: NO**  
**V6_EVIDENCE_IMMUTABLE: YES**  
**READY_FOR_NEXT_FINAL_CAMPAIGN: YES**  
**WEEK6_STATUS: BLOCKED**
