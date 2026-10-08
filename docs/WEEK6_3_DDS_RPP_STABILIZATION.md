# Week 6.3 DDS and RPP Stabilization

## Scope and v8 blockers

This diagnostic revision preserves [final_campaign_v8](../results/week6/final_campaign_v8)
and its analyzer evidence. The reported v8 blockers were:

- `healthy/unprotected/seed2530` failed before mission execution: world
  isolation passed and the upstream map, planner, controller, and behavior
  lifecycle nodes were active, but the planner actions and both costmap-clear
  services were not discovered. The BT navigator remained unconfigured.
- `healthy/protected/seed2531` completed all five waypoints but recorded six
  controller error-level `RegulatedPurePursuitController detected collision
  ahead!` events, with Nav2 recoveries and controller loop misses.

The first failure is treated as a startup/orchestration discovery failure, not
a scientific navigation result. The second is retained as a collision
diagnostic; collision detection remains enabled.

## Prechange freeze

Before source changes, [prechange_freeze.json](../results/week6/week6_3_diagnostics/prechange_freeze.json)
recorded SHA256 and mtime data for the requested implementation and scientific
files, plus timestamps, sizes, and SHA256 hashes for all 636 entries in the
v8 evidence tree. Reverification after implementation found no changed v8
entry.

## Fast DDS discovery configuration

Week 6 continues to use `rmw_fastrtps_cpp`. The runner sets
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in the environment passed to the
Week 6 launch and mission subprocesses, and the Week 6 launch sets the same
value for its child nodes. The ROS `rcl` discovery API in this installation
supports the local-host range.

The requested `RMW_FASTRTPS_ROS_DISCOVERY_INFO_UNIQUE_NETWORK_FLOWS=DISABLED`
environment variable is not read by the installed `rmw_fastrtps` library.
The library does expose a Fast DDS XML property named
`fastdds.unique_network_flows`; it does not establish that the requested
environment variable is supported. The variable is therefore not injected
or reported as effective. The supported, Week-6-only local discovery control
is the `LOCALHOST` automatic discovery range. Details are preserved in
[dds_discovery_support.json](../results/week6/week6_3_diagnostics/dds_discovery_support.json).
Week 4/5 launch defaults and scientific configuration are unchanged.

Every run's `run_metadata.json` records the applied localhost range and the
requested-but-unsupported network-flow setting with its unapplied status.

## Startup evidence and RPP/costmap consistency

The startup barrier keeps its existing bounded wait and lifecycle sequence.
Failure output distinguishes planner action discovery, planner service
discovery, costmap service discovery, controller action discovery, behavior
action discovery, lifecycle activation, and BT activation failures.
Pre-activation `bt_navigator=unconfigured` is explicitly distinguished from
the post-barrier active confirmation.

The installed RPP plugin contains
`inflation_cost_scaling_factor`. The Week-6-only RPP value is now explicitly
`2.0`, matching both the local and global costmap inflation scaling factors;
other controller and costmap tuning values were not changed for this
consistency correction. A startup-only live query confirmed:

- `FollowPath.inflation_cost_scaling_factor = 2.0`
- `FollowPath.use_cost_regulated_linear_velocity_scaling = true`
- `controller_server.use_realtime_priority = false`

The probe also discovered the RPP collision arc topic
`/lookahead_collision_arc` (`nav_msgs/Path`), `/local_costmap/costmap_raw`,
`/ares/scan`, `/cmd_vel`, filtered odometry, plan topics, and lookahead-point
topics. The live parameter output is in
[rpp_parameter_probe](../results/week6/week6_3_diagnostics/rpp_parameter_probe).

## Collision-event diagnostics

Goal-sending diagnostic runs can enable a separate telemetry node that
captures `/rosout` collision errors and snapshots the latest command, pose,
TF, scan, local costmap, RPP collision arc, plans, current waypoint estimate,
and sensor/costmap ages. Costmap cell samples are labeled as center-cell
observations and are not represented as a reproduction of Nav2's full
footprint collision check. The waypoint estimate is derived from the active
plan endpoint and mission waypoint list; it does not use ground truth.

Each event records the controller error, arc samples when available, local
costmap cell/cost when frames permit, scan proximity, odometry/TF, command,
mission-level recovery/completion, and timing context. If event-specific
recovery cannot be correlated from available Nav2 evidence, the artifact
marks that limitation rather than inferring an event-level recovery.

## Controller scheduling

The `controller_server` build exposes `use_realtime_priority`, and live
parameter inspection reported it false. The host kernel identifies as WSL2;
`RLIMIT_RTPRIO` is `(0, 0)` and `CAP_SYS_NICE` is absent. The live controller
threads observed in the startup-only probe all used `SCHED_OTHER`. Thus
realtime priority cannot be made effective with the current privileges and
kernel setup. No `use_realtime_priority: true` experiment was run.

The v8 run recorded two controller missed-rate warnings, both mentioning
costmap-update waits, a maximum controller command gap of about `0.528 s`,
and eight Nav2 recoveries. See
[controller_scheduling_audit.json](../results/week6/week6_3_diagnostics/controller_scheduling_audit.json).

## DDS startup stress and later diagnostics

The exact ten-cycle mixed-mode startup stress is stored at
[dds_startup_stress](../results/week6/week6_3_diagnostics/dds_startup_stress).
It has no navigation goals. Its result and endpoint-specific failure counts
are:

- Startup-ready cycles: **9/10**.
- World-isolation and post-run teardown failures: **0**.
- One actual startup failure: cycle 9, protected/healthy/seed 2530.
- Navigation goals sent: **0**.
- Cycle-9 readiness deficits: two planner actions, `/follow_path`, four
  behavior actions, `/is_path_valid`, two costmap-clear services, four
  upstream lifecycle nodes, and BT activation.

Cycle 9 passed the pre-run isolation gate, but its startup barrier found the
map server inactive, planner/controller/behavior lifecycle nodes
unconfigured, and the required planner, controller, behavior actions and
costmap/planner services absent. The launch log records
`RuntimeError: Unable to convert call argument '0' to Python object` in
`wheel_fault_injector` and `localization_consistency_monitor`; the map-server
lifecycle change-state request timed out, and the upstream lifecycle manager
did not exit until SIGKILL. These are concrete startup/runtime failures; the
evidence does not establish a single deeper cause, so none is inferred.
Cleanup and post-run teardown completed.

The initial aggregate incorrectly required `bt_navigator=active` in the
startup-barrier snapshot, which is intentionally recorded before BT activation.
The persisted endpoint artifact separately records post-activation lifecycle
bond confirmation and BT-tree construction. The classifier was corrected to
use those confirmations and to inspect upstream lifecycle states. Existing
evidence was reclassified in place without rerunning any cycle; the report
retains the pre-reclassification verdict and the true cycle-9 failure.

Because the startup gate did not reach 10/10, the protected healthy seed-2531
collision diagnostic and the matched healthy baseline/unprotected/protected
sequence were **not run**. The conditional collision-horizon experiment was
also not run. Their absence is a gate-enforced stop, not a skipped failure.

## Matched healthy comparison

Not run: the DDS stress did not pass 10/10, so the protected healthy
diagnostic prerequisite did not pass. The matched sequence remains gated.

## Conditional collision-horizon experiment

No collision-horizon change is planned by default. It is justified only if
the captured arc and costmap evidence shows the projected collision arc
substantially exceeds the immediately commanded safe trajectory. Collision
detection, robot radius, inflation radius, map, planner, and mission remain
unchanged. If evidence is insufficient, the optional experiment is skipped.

## Scientific integrity and v9 decision

The Week 6 trust/fault/recovery logic, EKF scientific configuration, mission
geometry, map, frozen `0.30 m` threshold, and analyzer acceptance logic are
outside this stabilization scope. No v9 campaign is run. The readiness
decision is **NOT READY**: the DDS gate is 9/10, so no collision diagnostic,
matched mission, or v9 campaign is authorized by this validation sequence.

The focused Week 4/5/6 regression set, including the corrected
pre-activation/post-activation BT classifier test, completed **138 passed**.
The first package-wide pytest invocation was made without sourcing the ROS
workspace and failed test collection with `ModuleNotFoundError:
ares_reliability`; rerunning the requested Week 4/5/6 regression files after
`source install/setup.bash` passed.
The repository-wide `test_flake8` guard reports 4,235 findings while scanning
generated `build/`, `install/`, and backup trees. A direct flake8 run on the
two changed Python files passes.
