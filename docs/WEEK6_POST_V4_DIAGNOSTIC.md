# Week 6 post-v4 diagnostic

**Status: BLOCKED.** No v5 campaign was run, and these results do not establish
Week 6 acceptance.

## 1. v4 analyzer result

`final_campaign_v4` contains 15 runs. Its analyzer verdict was `BLOCKED`;
`all_healthy_missions_complete` was the only failed check. Healthy completion
rates were baseline 3/3, unprotected 2/3, and protected 2/3. Protected GNSS
fault completion (3/3), protected-over-unprotected completion, localization
improvement, no false healthy protected gating, gate/probation behavior,
system-failure checks, and the frozen completed-waypoint threshold all passed.
The acceptance file is [acceptance.json](../results/week6/final_campaign_v4/acceptance.json).

## 2. Exact healthy failures

* `healthy_unprotected_s2531`: mission readiness timed out although map,
  planner, controller, and behavior servers were active; `bt_navigator` was
  inactive. The error was the `is_path_valid` service being unavailable after
  the configured 5.00-second wait.
* `healthy_protected_s2530`: Nav2 returned action status 4 for the final
  waypoint, but the evaluator marked it unreached at 0.3531943063 m from the
  goal, above the unchanged 0.30 m threshold. GNSS trust was HEALTHY, gates and
  probation were zero, and Nav2, planner, controller, and lifecycle failure
  counts were zero. The failed waypoint and its result are retained in
  [result.json](../results/week6/final_campaign_v4/healthy/healthy_protected_s2530/result.json).

## 3–5. Issue A: startup/discovery race and implementation

The local Nav2 Lyrical BT navigator's 5-second `wait_for_service_timeout`
diagnostic is direct evidence that lifecycle activation did not guarantee
that the `bt_navigator` client had discovered `/is_path_valid`. A lifecycle
manager's bond/active transition confirms node lifecycle state; it does not
wait for every remote Fast DDS action/service endpoint match.

The Week 6 launch now stages Nav2 lifecycle activation before BT activation.
In protected and unprotected modes it activates `map_server`,
`planner_server`, `controller_server`, and `behavior_server` first. Baseline
mode uses a Week 6-only upstream stage that also activates AMCL; the barrier
requires AMCL to be active before allowing BT activation in that mode. A
Week 6-only barrier waits up to 60 seconds of wall time for the required
lifecycle states and every action/service below to be discoverable. It
requires `bt_navigator` to remain unconfigured or inactive while checking.
Only a successful barrier exit starts the separate `bt_navigator` lifecycle
manager. A failed barrier emits an error and shuts the launch down rather than
activating a partially ready BT navigator. Readiness is based on explicit
lifecycle and ROS graph checks, not a blind readiness sleep. The existing
5000 ms Nav2 wait is unchanged and remains a bounded secondary margin between
the successful graph check and BT construction.

The checked endpoints are those referenced by both installed Lyrical default
trees, plus the navigator's path-validation service:

| Kind | Endpoint | Nav2 interface |
|---|---|---|
| Action | `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` |
| Action | `/compute_path_through_poses` | `nav2_msgs/action/ComputePathThroughPoses` |
| Action | `/follow_path` | `nav2_msgs/action/FollowPath` |
| Action | `/spin` | `nav2_msgs/action/Spin` |
| Action | `/wait` | `nav2_msgs/action/Wait` |
| Action | `/backup` | `nav2_msgs/action/BackUp` |
| Action | `/drive_on_heading` | `nav2_msgs/action/DriveOnHeading` |
| Service | `/is_path_valid` | `nav2_msgs/srv/IsPathValid` |
| Service | `/global_costmap/clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` |
| Service | `/local_costmap/clear_entirely_local_costmap` | `nav2_msgs/srv/ClearEntireCostmap` |

The default BTs also name planner/controller selector topics; those are topics,
not action/service servers, and are not readiness prerequisites because the
trees provide their default plugin selections. The Lyrical files inspected
were `navigate_to_pose_w_replanning_and_recovery.xml` and
`navigate_through_poses_w_replanning_and_recovery.xml` under
`/opt/ros/lyrical/share/nav2_bt_navigator/behavior_trees/`.

The required set is **10 endpoints**: 7 actions and 3 services. The frozen
trees call `ClearEntireCostmap` for the global and local costmaps only; they
do not call `ClearCostmapAroundRobot` or `ClearCostmapAroundPose`. Those
registered Nav2 service APIs therefore are not discovery prerequisites for
these trees. `/drive_on_heading` is included as requested even though neither
selected tree invokes it. The barrier records each endpoint's first observed
monotonic wall timestamp and elapsed time in its READY record. Startup-only
runner artifacts from the current implementation are named
`endpoint_readiness.json`; the earlier three attempts retain their original
names and contents.

### Controlled startup attempts

Exactly three fresh startup-only launches were attempted and preserved:

| Attempt | Outcome | Detail |
|---|---|---|
| Healthy/unprotected seed 2531 | FAIL | The barrier executable rejected launch-injected `--ros-args -r __node:=...`; launch shutdown handling also logged an invalid `EmitEvent(Shutdown)` construction. |
| Healthy/protected seed 2530 | FAIL | Same startup-validation integration errors; no barrier endpoint result was produced. |
| `gnss_step_5m`/protected seed 2532 | FAIL | Same startup-validation integration errors; no barrier endpoint result was produced. |

The preserved records are under
[post_v4_diagnostics](../results/week6/post_v4_diagnostics/). These failed
attempts were not rerun. Thus the requested A1 pass count is **0/3**; these
failures do not indicate missing Nav2 endpoints because the barrier did not
run.

The one required B1 launch exercised an earlier corrected version of the
startup barrier. Its launch log records the then-configured nine endpoints
ready, all four upstream lifecycle nodes active, and `bt_navigator` still
unconfigured at the check, in 1.573 wall seconds. Both BTs were then created,
and the BT lifecycle manager reported its bond. This verifies the protected
healthy launch's earlier synchronization path once, but it does not turn the
three A1 attempts into passes. That barrier version predated
`/drive_on_heading`; it did not verify the current tenth endpoint. No
controlled launch was rerun with the current ten-endpoint barrier.

## 6. Issue B: pose source and time comparison

The installed Nav2 Lyrical **1.5.1** source confirms that the controller calls
`Costmap2DROS::getRobotPose()` when checking the goal, transforms the planned
goal into the costmap global frame, and passes that TF-derived pose and goal
to the configured goal-checker plugin. `SimpleGoalChecker` checks XY distance
against the squared XY tolerance and yaw against the yaw tolerance. The
controller costmap obtains its pose through `nav2_util::getCurrentPose()`
using its TF buffer, global frame, base frame, and transform tolerance. This
does not use either odometry topic as the goal check's position source.

`Week6Mission._sample_pose()` independently obtains the latest
`map -> base_footprint` TF and the frozen acceptance calculation uses that
pose. Therefore the Nav2 goal checker and mission evaluator use the same
pose source and coordinate frame, but not necessarily the same sample or
effective timestamp: the controller checks on its control cycle, while the
mission reads its latest TF sample after receiving the action result. A
`NavigateToPose` result has no server-side completion timestamp. The new
diagnostic records the action-result receipt time, evaluator sample time,
both odometry messages and their frames/stamps, map-frame TF pose/age,
distance/yaw per source, queried goal-checker tolerances, and an existing-time
window pose trace. It does not change the evaluator source or use ground truth
for runtime gating.

The installed Lyrical plugin header identifies
`nav2_controller::SimpleGoalChecker`; Week 6 config sets XY tolerance
**0.20 m**, yaw tolerance **0.30 rad**, and `stateful: false`. The global
costmap/controller costmap uses `global_frame: map` and
`robot_base_frame: base_footprint`. The controller obtains the robot pose in
that costmap frame via its `getRobotPose()` API (backed by
`Costmap2DROS::getRobotPose()`), which uses map-to-base TF; the odometry input
is used for velocity handling, not as the goal check's XY pose. Explicit
Week 6 tolerances are `bt_navigator.transform_tolerance: 0.2` s,
`behavior_server.transform_tolerance: 0.2` s, and
`FollowPath.transform_tolerance: 0.3` s. The installed Nav2 1.5.1
`Costmap2DROS` source sets the costmap `transform_tolerance` default to
**0.3 s**; Week 6 does not override that parameter in its YAML.

For v4 waypoint 4, the controller logged “Reached the goal!” at
1791307225.343439845, the BT logged “Goal succeeded” at 1791307225.387459486,
and the mission logged the result at 1791307225.391269. The final evaluator
distance was 0.3531943063 m and yaw error 0.2395997011 rad. Week 6's
goal-checker configuration was `xy_goal_tolerance=0.20`,
`yaw_goal_tolerance=0.30`, `stateful=false`; the independent mission limits
remain 0.30 m and 0.35 rad. A 0.353 m sample cannot be the same XY sample
that satisfied a 0.20 m checker. The controller evaluates the goal at a
control-cycle TF sample; the mission evaluates its latest `map ->
base_footprint` TF after receiving the action result. They therefore use the
same pose source and frame but not necessarily the same TF sample timestamp.
This rules out an intentionally larger XY tolerance and an odometry-position
source mismatch. A pose/timestamp difference or transient between samples is
consistent with the discrepancy, but v4 did not retain synchronized pose
samples and therefore does not establish which occurred. The available logs
also cannot establish whether low RTF, TF age, or localization motion caused
the discrepancy.

The v4 runtime had clock mean 99.159 Hz, RTF median 0.4423 (p10 0.3974,
p90 0.4754), 164/166 severe-collapse windows, and zero controller-rate
misses. Low RTF could increase the wall-time age of samples, but the evidence
does not establish it as the cause. `Path validation failed. Invalid pose
indices: [119]` occurred on the last leg; the planner supplied a replacement
path and the mission recorded one replan. It affected the approach path, but
there is no evidence tying that replan to the final pose discrepancy.

The installed trees route `ValidatePath` through `/is_path_valid`. The retained
warning proves that the service marked path pose 119 invalid and replanning
followed. The v4 run did not retain the invalid cell's costmap value, layer,
or full costmap snapshot, so it cannot distinguish an obstacle/lethal or
inflated cell from another validity condition. The warning occurs only in the
failed protected healthy seed-2530 v4 launch among the v4 healthy launches;
none of the clean healthy v4 launch logs contains it.

The updated mission records action-result receipt and evaluator timestamps,
current `/odometry/filtered` and `/odometry/trust_fused` poses, the
map-frame TF pose when available, goal pose, per-source distance and yaw
error, pose ages, signed timestamp deltas, and queryable controller goal
checker settings. It also records delivered simulation-clock/wall-clock
pairs and diagnostic RTF for each waypoint leg and the final five wall
seconds before each action result, plus an existing-time pose trace around
completion. These are diagnostic only; the evaluator still uses its existing
TF-based acceptance source and ground truth remains evaluation-only. v4 has
aggregate run RTF only and B1 ended before its first goal; B2 below is the
first complete synchronized diagnostic comparison.

### B1 controlled protected healthy attempt

Exactly one protected healthy seed-2530 navigation attempt was made and
preserved at
[healthy_protected_s2530_attempt1](../results/week6/post_v4_diagnostics/healthy_protected_s2530_attempt1/).
The corrected barrier passed and both BTs loaded, but the mission stopped in
readiness before sending its first goal: the optional goal-checker diagnostic
treated ROS `ParameterValue` entries as named parameters. The result records
that runner error. The response decoding was corrected afterward and covered
by a unit test, but B1 was not rerun. Thus B1 reached **0/5** waypoints; there
is no new completion sample, no controlled reproduction measurement for Issue
B, and no navigation-based false-gate/abort/controller/lifecycle result.

## 7. Follow-up validation

### A1 exact failure cause

All three A1 launch logs show that the readiness executable started, then
exited with code 2 before `rclpy.init()` or any endpoint/lifecycle query. ROS
launch appended `--ros-args -r __node:=week6_nav2_startup_barrier`; that
earlier executable rejected those arguments. No A1 endpoint check was
performed, and `readiness_barrier` is null in all three preserved results.
The process-exit handler then raised
`EmitEvent() expected an event instance ... Shutdown ...`, so the intended
fail-closed launch shutdown also failed. The launch runtime subsequently
stopped child processes; the lifecycle manager's signal/context exceptions
and aborts happened during that teardown, not during a completed Nav2
bringup.

The lifecycle managers and Nav2 nodes used the root namespace, matching the
barrier's absolute endpoint names; endpoint naming was not exercised in A1.
In the protected healthy attempt the upstream manager began configuring the
map, planner, controller, and behavior servers. Across all three attempts
there is no `Server ... connected with bond` confirmation for those servers
and no evidence that they reached ACTIVE. `bt_navigator` was launched but
logged that it was waiting on external lifecycle transitions. The separate
BT lifecycle manager is started only by a successful barrier-exit callback,
so BT activation was deferred correctly. There was no competing lifecycle
manager owning the same nodes. The root cause was barrier argument parsing,
with the malformed shutdown event as a second launch-handler defect; not
incorrect endpoint names, namespace mismatch, or lifecycle-manager ownership.

The corrected installed entry point was checked with the same appended ROS
remap arguments and accepted them. Its failure handler now returns a
`Shutdown` launch action directly. No further barrier redesign was made.

### Barrier smoke test

Exactly one unprotected healthy seed-2531 startup smoke test passed and is
preserved in
[a2_smoke](../results/week6/post_v4_diagnostics/a2_smoke/). The barrier
process started, observed `map_server`, `planner_server`, `controller_server`,
and `behavior_server` ACTIVE, and observed `bt_navigator` still unconfigured.
It checked all 10 endpoints, each discovered, and returned READY in **1.740
wall seconds**. The event log then started the BT lifecycle manager; Nav2
reported the BT server connected and both
`NavigateToPoseWReplanningAndRecovery` and
`NavigateThroughPosesWReplanningAndRecovery` were created. No endpoint timed
out.

### A2 startup validation

Exactly three fresh attempts were run in
[a2_startup_validation](../results/week6/post_v4_diagnostics/a2_startup_validation/).
All **3/3 passed** the barrier, endpoint, lifecycle, ordering, and BT creation
checks. Each readiness record contains per-endpoint first-discovery wall
time; the values below are elapsed wall seconds from barrier startup.

| Endpoint | Healthy unprotected 2531 | Healthy protected 2530 | GNSS step protected 2532 |
|---|---:|---:|---:|
| `/compute_path_to_pose` | 0.051 | 1.574 | 0.051 |
| `/compute_path_through_poses` | 0.051 | 1.574 | 0.051 |
| `/follow_path` | 0.051 | 0.906 | 0.634 |
| `/spin` | 0.051 | 0.052 | 0.572 |
| `/backup` | 0.051 | 0.052 | 0.572 |
| `/drive_on_heading` | 0.051 | 0.052 | 0.572 |
| `/wait` | 0.051 | 0.052 | 0.572 |
| `/is_path_valid` | 0.683 | 1.574 | 0.051 |
| `/global_costmap/clear_entirely_global_costmap` | 0.051 | 1.574 | 0.051 |
| `/local_costmap/clear_entirely_local_costmap` | 0.051 | 0.900 | 0.623 |

In every attempt, readiness recorded all four upstream lifecycle nodes
ACTIVE and `bt_navigator` unconfigured before BT activation; all 10 actions
and services were discoverable, `bt_navigator` subsequently connected with a
bond, and both BTs were created. There were **0 endpoint timeouts, 0 BT
discovery failures, 0 BT activation failures, and 0 lifecycle bringup
aborts**. The launch runner's later SIGINT/forced process shutdown is
controlled teardown after its startup assertions, not a bringup failure.

### B2 controlled healthy protected seed 2530

Exactly one full B2 navigation attempt was run and preserved at
[b2_healthy_protected_s2530_attempt1](../results/week6/post_v4_diagnostics/b2_healthy_protected_s2530_attempt1/).
It completed **5/5** waypoints. All action statuses were Nav2 success (status
4); all evaluator distances were at or below **0.1979941 m**, unchanged
against the frozen **0.30 m** acceptance threshold. False gates, probation,
Nav2 aborts, planner failures, controller failures, and lifecycle failures
were all zero.

| WP | Evaluator distance (m) | Evaluator yaw error (rad) | TF distance at action-result receipt (m) | TF sample age at receipt (sim s, derived) | Trust-fused sample age (sim s) | Final-5-wall-second RTF |
|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0.190238 | 0.018313 | 0.186456 | 0.019 | 0.019 | 0.741 |
| 1 | 0.197994 | 0.291098 | 0.197994 | 0.025 | 0.025 | 0.687 |
| 2 | 0.181316 | 0.299704 | 0.181316 | 0.026 | 0.026 | 0.614 |
| 3 | 0.081710 | 0.211931 | 0.079565 | 0.008 | 0.008 | 0.581 |
| 4 | 0.114439 | 0.277404 | 0.114439 | 0.027 | 0.027 | 0.565 |

The evaluator and action-result TF distances differed by at most **0.003783
m**. The action-result transform and trust-fused odometry poses/distances
matched for each waypoint (the configured EKF output is remapped to
`/odometry/trust_fused`). `/odometry/filtered` had no samples in protected
mode, and is explicitly recorded as `no_sample_received`; this topic is not
the protected EKF output. Evaluator TF sample ages were **0.026–0.065 sim
seconds**. The queried goal checker remained XY **0.20 m**, yaw **0.30 rad**,
`stateful: false`.

The action receipt and evaluator times were recorded in ROS, simulation, and
wall-clock fields. One diagnostic field,
`transform_age_at_action_result_sec`, incorrectly subtracts a simulation-time
TF stamp from the system-time ROS clock and is therefore not a meaningful
age. The table derives the action-result TF age in the matching simulation
time domain (`sim_clock_time_sec - tf_stamp_sec`); it does not alter the
acceptance result. B2 did **not** reproduce a Nav2-success/evaluator-distance
mismatch. It cannot reveal the exact controller-cycle pose at the instant
Nav2 internally accepted the goal, but both the evaluator and near-result TF
samples were within the unchanged evaluator threshold.

B2 runtime diagnostics showed median RTF **0.618**, p10 **0.539**, 8 severe
RTF windows out of 161, and zero controller-rate misses. Each final-five-wall
second window before success had RTF **0.565–0.741**. There were no new
systematic runtime or navigation failures. The `AMCL_NOT_LAUNCHED` diagnostic
also appears in the clean v4 protected seed-2531 run; AMCL is not part of this
protected mode and the diagnostic did not cause a gate or mission failure.

### Controlled validation outcome and v5 readiness

The startup barrier is validated at **3/3** after the passing smoke test;
B2 is **5/5** with the frozen acceptance criteria. No startup discovery
failures occurred in smoke/A2, and no scientific settings or analyzer
criteria changed. **READY_FOR_FINAL_CAMPAIGN_V5: YES** means only that the
controlled validation supports proceeding to a separately authorized
immutable campaign; it does not mean Week 6 passed. No v5 campaign was run.

## 8. Regression and integrity

The `ares_simulation` and `ares_reliability` packages built successfully.
The focused Week 4/5/6, navigation-metrics, recovery-manager, and
startup-barrier tests were rerun after controlled validation: **84 passed**.
A direct mypy check of the four Week 6 modules
passed before the latest baseline-stage edit. The later check including
`ares_baseline.launch.py` was blocked only by unavailable YAML typing stubs;
the package `test_mypy.py` wrapper could not complete because its workspace
scan finds a
duplicate `reliability_test.launch` module under both `build/` and `install/`.
Focused import-order and new-file line-length checks passed; full flake8 still
reports legacy broad-exception and long-line violations in the existing
mission module outside the new diagnostic code.

No scientific setting was changed in this task. The 0.30 m evaluator
threshold, mission goals and GNSS fault magnitude/timing remain in
`week6_mission.yaml`; trust and pre-fusion thresholds remain in the Week 6
launch; EKF, controller/planner tuning, robot/inflation radii, map origin, and
Week 4/5 defaults remain unchanged. Analyzer code and criteria were not
modified. Pre/post integrity checks confirm byte hashes, sizes, and
modification times match for all **1,411** files in
`final_campaign_v2`, `final_campaign_v3`, `final_campaign_v4`, and
`runtime_stabilization`. Of the nine separately tracked protected files, the
only intentional change is `ares_baseline.launch.py`, which adds the opt-in
`nav2_week6_upstream` stage for Week 6; its existing stage/default paths are
unchanged. The other eight protected files match their pre-change hashes and
modification times.

## 9. Another immutable campaign

**Controlled validation justifies readiness for a separately authorized
`final_campaign_v5`; it does not justify declaring Week 6 PASS.** A2 passed
3/3, B2 passed 5/5, there were no startup discovery failures, and the frozen
criteria/analyzer were unchanged. The known limitation in the action-result
TF-age diagnostic is documented above; it did not affect the mission
evaluator or B2 outcome. Do not run v5 as part of this validation task.
