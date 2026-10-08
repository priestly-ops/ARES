# ARES Week 6 Closed-Loop Navigation Reliability Report

Date: 2026-10-05  
Workspace: `/home/priestly/ares_ws`  
Final campaign: `results/week6/final_campaign`  
Seeds: 2530, 2531, 2532

## 1. Objective

Week 6 compares `baseline`, `unprotected`, and `protected` closed-loop Nav2
operation in healthy conditions, then compares `unprotected` and `protected`
under a matched-seed `gnss_step_5m` fault. The frozen mission acceptance limit
remains `final_distance_m <= 0.30 m`.

The final verdict is **BLOCKED**. Protection was decisively better than the
unprotected path, but protected fault completion was 2/3 rather than the
required consistent 3/3. The failed protected seed is retained in the final
campaign and is the demonstrated blocker.

## 2. Architecture

The Week 6 experimental localization chain is:

```text
/ares/gps_raw -> fault injector -> /ares/gps
                                      |
                 +--------------------+--------------------+
                 |                                         |
       unprotected covariance normalizer          protected trusted proxy
       /ares/gps_unprotected_normalized           /ares/gps_trusted
                 |                                         |
                 +-------------- selected GNSS ------------+
                                      |
                         navsat_transform_node
                                      |
                         /odometry/gps_trusted
                                      |
                         Week 6 fused EKF (TF owner)
                                      |
                         /odometry/trust_fused
                                      |
                         Nav2 controller + BT navigator
```

Protected and unprotected modes use the same world, robot, map, motion limits,
Nav2 stack, Week 6 EKF, fixed datum, mission, and normalized GNSS covariance.
Only the protected GNSS path receives actual recovery-policy gating commands.
The unprotected proxy listens on the deliberately unused policy topic
`/ares/week6/unprotected_unused_policy` and publishes status on the separate
`/ares/week6/unprotected_gnss_state` topic.

`controller_server` and `bt_navigator` remap `/odometry/filtered` to
`/odometry/trust_fused` in both fused modes. Baseline retains AMCL and the
production EKF, but the Week 6 launcher passes the same Week 6 Nav2 parameters
through a new optional baseline-launch argument. The baseline launch argument's
default remains the original shared `ares_localization/config/nav2_params.yaml`,
so production, Week 4, and Week 5 defaults are unchanged.

Ground truth is the Gazebo model pose bridged to `/ares/ground_truth`. It is
used only for recorded evaluation, never for readiness, gating, planning,
control, or recovery.

## 3. Final frozen implementation

The campaign implementation was frozen before seed 2530 began. Every one of
the 15 `run_metadata.json` files contains an identical SHA-256 map for the
runner, mission, metric code, Week 6 mission/Nav2/NavSat/EKF configurations,
Week 6 launch, and baseline launch. The analyzer verified one unique hash set.

Final implementation changes were deliberately narrow:

- `nav2_week6_navigation.yaml` is an exact copy of the shared Nav2 config except
  `goal_checker.stateful: false`.
- The baseline launcher accepts an optional `nav2_params` argument whose default
  is unchanged; only Week 6 supplies the Week 6 config.
- Readiness uses fixed simulation time, localization sample count, localization
  history duration, lifecycle state, action availability, and required sensor
  services. It no longer waits for a low-drift localization window.
- Readiness records, but does not use, mission-start ground truth and
  localization error.
- Run metadata fingerprints the complete runtime implementation.
- Final aggregation validates matched cases, frozen hashes, mission reliability,
  localization improvement, false gating, system health, and the 0.30 m limit.
- Planner/controller failures are also derived from preserved launch logs,
  because the Nav2 diagnostic topic did not report failures that were explicit
  in process logs.

## 4. Fairness controls

Each seed was executed sequentially in this fixed order:

1. baseline healthy
2. unprotected healthy
3. protected healthy
4. unprotected `gnss_step_5m`
5. protected `gnss_step_5m`

The fault starts 15 simulated seconds after the first accepted mission goal and
lasts 30 simulated seconds. No failed run was retried, removed, or overwritten.
All result directories are immutable under the runner.

Readiness is identical across modes where applicable:

- all required Nav2 lifecycle nodes active;
- NavigateToPose action server available;
- localization available;
- five simulated seconds settled after lifecycle readiness;
- absolute simulation time at least 20.0 seconds;
- at least 50 localization samples;
- at least five simulated seconds of localization history;
- GNSS injector service available for non-baseline fault cases.

Final campaign mission starts were tightly bounded at 20.002–20.035 simulated
seconds. The readiness decision never checks `latest_reference`, localization
error, or closeness to the known spawn. Every result records
`readiness_ground_truth_used: false`.

## 5. Covariance normalization

Both fused modes convert UNKNOWN Gazebo GNSS covariance to 1.0 m² diagonal
covariance before NavSat conversion:

- protected: `/ares/gps_trusted`;
- unprotected: `/ares/gps_unprotected_normalized`.

The unprotected normalizer is not connected to the real gating command topic.
This removes the earlier covariance-weighting confound without protecting the
unprotected measurement path.

## 6. Fixed-datum rationale

Only `navsat_week6_navigation.yaml` uses:

```yaml
wait_for_datum: true
datum: [39.7391356028, -104.9903000000, 0.0]
```

The robot spawns at world `(0, -7, +pi/2)`. The SDF GNSS antenna is 0.15 m
behind the chassis, which rotates to approximately world `(0, -7.15)` at the
spawn heading. The datum is the corresponding geographic coordinate and makes
the local ENU origin deterministic. The shared Week 5 config remains
`wait_for_datum: false` with no datum entry.

## 7. Readiness strategy and startup evidence

The earlier 0.10 m drift gate was removed. It proved only that the estimator
was temporarily quiet and could wait for a particular noise window. It also
depended indirectly on the ground-reference stream because pose sampling
returned early before ground truth was available.

The final gate uses fixed history rather than estimator accuracy. It reports:

- `mission_start_sim_time_sec`;
- `localization_at_mission_start`;
- `ground_reference_at_mission_start`;
- `localization_error_at_mission_start_m`;
- readiness history duration and sample count;
- localization span over retained history;
- latest estimator covariance;
- `readiness_ground_truth_used: false`.

Baseline startup localization error was effectively zero. Fused-mode startup
error varied because the 1 m² GNSS stream is noisy; this variation is recorded
but did not control mission start. It remains a source of path-quality variance,
not a selected startup advantage.

## 8. Nav2 goal checker investigation

The shared Nav2 config already used a strict 0.20 m XY tolerance and 0.30 rad
yaw tolerance, so increasing or decreasing the frozen 0.30 m evaluator limit
was neither necessary nor permitted.

The actual cause of Nav2 success at fused distances of 0.307–0.388 m was the
stateful SimpleGoalChecker. The installed Lyrical header states that, when
stateful, XY is not checked again after it first becomes true. The robot could
therefore enter the 0.20 m radius, rotate for yaw, drift back outside it, and
still return success.

Week 6 now uses `stateful: false`, requiring XY and yaw simultaneously. The
shared file remains `stateful: true`. Runtime validation on seed 2522 produced
five successful protected waypoint fused distances with a maximum of 0.194 m;
the baseline validation maximum was 0.199 m. Across all completed final
campaign waypoints, the maximum fused error was 0.244 m, below the frozen
0.30 m evaluator threshold.

Ground-reference waypoint distances can still exceed 0.30 m in fused modes
because Nav2 controls the robot from the fused pose, not ground truth. That is
a localization limitation, not a goal-checker semantic mismatch. Terminal
ground-reference error is reported separately.

## 9. Matched-seed methodology

Three new seeds (2530–2532) were selected after implementation freeze. The 15
runs are all under `results/week6/final_campaign`; no development or
repeatability run is included. Each run preserves:

- `result.json`;
- `launch.log`;
- `mission.log`;
- `run_metadata.json`;
- per-process ROS logs.

The machine-readable products are:

- `results/week6/final_campaign/summary.json`;
- `results/week6/final_campaign/matched_seed_comparison.json`;
- `results/week6/final_campaign/acceptance.json`.

## 10. Per-seed results

`Final` is fused final goal error. `GT final` is independent terminal
ground-reference error. Errors and CTE are metres; time is simulated seconds.

| Seed | Scenario | Mode | Waypoints | Complete | Final | GT final | Time | Mean / p95 CTE | Max loc. | Recoveries | Abort | Gates / probation |
|---:|---|---|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2530 | healthy | baseline | 5/5 | yes | 0.195 | 0.098 | 70.602 | 0.052 / 0.144 | 0.152 | 0 | 0 | 0 / 0 |
| 2530 | healthy | unprotected | 5/5 | yes | 0.110 | 0.090 | 100.440 | 0.145 / 0.313 | 0.423 | 0 | 0 | 0 / 0 |
| 2530 | healthy | protected | 5/5 | yes | 0.195 | 0.243 | 71.221 | 0.149 / 0.320 | 0.469 | 0 | 0 | 0 / 0 |
| 2530 | fault | unprotected | 1/2 | no | 4.722 | 1.988 | 27.842 | 0.862 / 2.126 | 4.921 | 10 | 1 | 1 / 1* |
| 2530 | fault | protected | 1/2 | no | 2.246 | 2.189 | 27.443 | 0.112 / 0.290 | 0.970 | 10 | 1 | 1 / 1 |
| 2531 | healthy | baseline | 5/5 | yes | 0.184 | 0.132 | 70.020 | 0.051 / 0.136 | 0.170 | 0 | 0 | 0 / 0 |
| 2531 | healthy | unprotected | 5/5 | yes | 0.202 | 0.046 | 73.795 | 0.138 / 0.358 | 0.548 | 0 | 0 | 0 / 0 |
| 2531 | healthy | protected | 5/5 | yes | 0.189 | 0.164 | 79.226 | 0.157 / 0.417 | 0.547 | 0 | 0 | 0 / 0 |
| 2531 | fault | unprotected | 1/2 | no | 5.276 | 2.840 | 31.498 | 0.999 / 3.379 | 5.204 | 9 | 1 | 1 / 1* |
| 2531 | fault | protected | 5/5 | yes | 0.098 | 0.106 | 84.786 | 0.163 / 0.384 | 0.423 | 0 | 0 | 1 / 1 |
| 2532 | healthy | baseline | 5/5 | yes | 0.195 | 0.136 | 71.514 | 0.064 / 0.164 | 0.164 | 0 | 0 | 0 / 0 |
| 2532 | healthy | unprotected | 5/5 | yes | 0.092 | 0.135 | 95.123 | 0.097 / 0.222 | 0.524 | 1 | 0 | 0 / 0 |
| 2532 | healthy | protected | 5/5 | yes | 0.089 | 0.030 | 80.004 | 0.095 / 0.235 | 0.446 | 0 | 0 | 0 / 0 |
| 2532 | fault | unprotected | 1/2 | no | 5.006 | 2.766 | 30.054 | 0.689 / 2.612 | 5.320 | 9 | 1 | 1 / 1* |
| 2532 | fault | protected | 5/5 | yes | 0.102 | 0.095 | 75.601 | 0.177 / 0.465 | 0.597 | 0 | 0 | 1 / 1 |

`*` The common recovery graph detects and publishes recovery states in
unprotected mode, so the recorder observes GATED/PROBATION. The unprotected
GNSS proxy is intentionally disconnected from those commands; these are not
actual unprotected measurement gates.

## 11. Aggregate results

| Scenario / mode | Completion | Waypoints | Final error mean | Mean / p95 / max CTE mean | Path ratio mean | Time mean | Mean / max localization mean |
|---|---:|---:|---:|---:|---:|---:|---:|
| healthy / baseline | 3/3 | 15/15 | 0.191 | 0.056 / 0.148 / 0.223 | 0.969 | 70.712 | 0.059 / 0.162 |
| healthy / unprotected | 3/3 | 15/15 | 0.134 | 0.127 / 0.297 / 0.396 | 1.011 | 89.786 | 0.198 / 0.498 |
| healthy / protected | 3/3 | 15/15 | 0.158 | 0.133 / 0.324 / 0.418 | 0.986 | 76.817 | 0.201 / 0.487 |
| fault / unprotected | 0/3 | 3/6 attempted | 5.001 | 0.850 / 2.705 / 2.750 | 0.675 | 29.798* | 2.209 / 5.148 |
| fault / protected | 2/3 | 11/12 attempted | 0.815* | 0.151 / 0.379 / 0.461 | 0.892 | 62.610* | 0.357 / 0.663 |

`*` Failed missions end early, so group means for final error, path ratio, and
completion time combine completed and early-aborted missions. Distribution
details (mean, median, minimum, maximum, and count) are retained in
`summary.json`.

Reliability/failure aggregates:

| Scenario / mode | Nav2 aborts | Planner failures | Controller failures | Nav recoveries | Command gaps / max gap | Gates / probation |
|---|---:|---:|---:|---:|---:|---:|
| healthy / baseline | 0 | 0 | 0 | 0 | 0 / 0.000 s | 0 / 0 |
| healthy / unprotected | 0 | 0 | 0 | 1 | 0 / 0.000 s | 0 / 0 |
| healthy / protected | 0 | 0 | 0 | 0 | 0 / 0.000 s | 0 / 0 |
| fault / unprotected | 3 | 16 | 7 | 28 | 6 / 5.574 s | 3 / 3* |
| fault / protected | 1 | 8 | 1 | 10 | 2 / 5.682 s | 3 / 3 |

Planner/controller failure counts come from explicit launch-log events. The
mission diagnostic-topic fields were zero for these same runs, so the analyzer
preserves both sources and uses the log-derived counts in final aggregates.

## 12. Protected versus unprotected fault comparison

Protection was strongly beneficial:

- completion improved from 0/3 to 2/3;
- reached-waypoint fraction improved from 3/6 attempted to 11/12 attempted;
- mean of each run's maximum localization error fell from 5.148 m to 0.663 m
  (87.1% lower);
- mean CTE fell from 0.850 m to 0.151 m;
- mean p95 CTE fell from 2.705 m to 0.379 m;
- aborts fell from 3 to 1;
- log-derived planner failures fell from 16 to 8;
- controller collision failures fell from 7 to 1;
- Nav2 recoveries fell from 28 to 10.

Protected seeds 2531 and 2532 followed
`NORMAL -> DEGRADED -> GATED -> PROBATION -> NORMAL`, completed 5/5, and
returned to stable NORMAL 4.499 s and 4.385 s after fault clear respectively.

Protected seed 2530 is the blocker. It reached GATED approximately 1.01 s after
DEGRADED and limited maximum localization error to 0.970 m, far below the
unprotected 4.921 m. However, corrupt GNSS samples fused before gating had
already shifted the EKF pose. The controller reported collision ahead, and the
global planner repeatedly rejected the shifted start pose near `(3.75, -2.41)`.
Eight planning failures and ten Nav2 recoveries exhausted the behavior tree,
causing one abort before the 30-second fault cleared. The recovery state later
entered PROBATION during post-mission observation, but mission continuation was
already impossible.

This is not a covariance, topic-collision, TF, lifecycle, estimator-restart, or
goal-tolerance failure. It is residual protected-state corruption during FDIR
latency combined with Nav2's inability to plan from the shifted/costly pose.

## 13. Healthy-mode degradation

All healthy modes completed 3/3 with 15/15 waypoints. Protected healthy mode
had zero gates, zero probation, zero false recoveries, zero estimator restarts,
zero TF errors, zero lifecycle failures, zero planner/controller failures, and
zero command gaps.

Relative to baseline, protected healthy mean completion time increased from
70.712 s to 76.817 s (+8.6%). Mean CTE increased from 0.056 m to 0.133 m, and
mean run-maximum localization error increased from 0.162 m to 0.487 m. These
are measurable costs of the GNSS-fused estimator, but healthy mission
reliability and the 0.30 m acceptance threshold were preserved.

## 14. False-positive gating analysis

Protected healthy runs had 0 gates and 0 probation transitions. Seeds 2530 and
2531 each had late transient `DEGRADED -> NORMAL` transitions without gating;
seed 2532 remained NORMAL. This matches the Week 6-only
`probation_after_ungated_degraded = false` requirement. No protected healthy
run restarted the estimator or lost TF/lifecycle health.

## 15. Known limitations

- Three matched seeds meet the requested minimum but not the preferred five.
- The protected failure shows that gating after two persistent observations can
  still allow enough corrupt GNSS influence to leave the EKF pose invalid for
  planning. Fixing this requires a new, predeclared protection mechanism and a
  completely new final campaign; the present campaign must not be relabeled.
- The SDF and URDF GNSS antenna transforms differ slightly. The fixed datum is
  based on the simulated SDF antenna; NavSat lever-arm compensation uses TF.
  This mismatch may contribute a small systematic localization offset.
- Ground-reference distances sometimes exceed fused goal distances because
  Nav2 correctly acts on the fused pose. Ground truth is evaluation-only.
- Collision count remains unavailable as a direct physics contact metric.
- WSL scheduling affects wall time even with a fixed Gazebo seed. Simulation
  time and deterministic startup requirements reduce, but do not eliminate,
  runtime variability.
- Nav2 did not expose all planner/controller failures through the subscribed
  diagnostic stream. Final analysis supplements result fields with log-derived
  counts and records that provenance.
- Each final run logged two Nav2 SIGKILL exits during teardown (30 per campaign)
  after `result.json` had been written. These are reported in `summary.json` as
  `shutdown_sigkill_after_result_count` and are not mission failures.

## 16. Week 4/5 regression and configuration results

Builds completed successfully for `ares_reliability` and `ares_simulation`
with symlink install.

Focused Week 4/5/6 configuration, analysis, recovery, and navigation tests:

```text
68 passed
```

Full non-linter `ares_reliability` suite:

```text
148 passed, 1 skipped
```

The tests verify:

- frozen Week 5 EKF settings except the already documented Week 6 relative-yaw
  delta;
- Week 5 automatic datum and Week 6-only fixed datum;
- equal 1.0 m² covariance normalization;
- separate unprotected command/status topics;
- protected/unprotected odometry remaps;
- fixed-history, ground-truth-independent readiness;
- Week 6 stateless goal checker as the only Nav2 config delta;
- unchanged baseline Nav2 default;
- unchanged 0.30 m mission threshold.

The repository-wide lint-wrapper invocation had three environmental or
pre-existing failures and is not hidden: flake8 multiprocessing could not bind
its sandbox forkserver socket; mypy scanned duplicate build/install modules;
and repository-wide pep257 found unrelated existing docstring violations.
Direct flake8/pep257 checks on the new campaign/analyzer/test files pass after
the final formatting fix.

## 17. Exact regeneration commands

From `/home/priestly/ares_ws`:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select ares_simulation ares_reliability --symlink-install
source install/setup.bash

python3 -m pytest -q \
  src/ares_reliability/test/test_week6_navigation_config.py \
  src/ares_reliability/test/test_navigation_metrics.py \
  src/ares_reliability/test/test_week5_fusion_config.py \
  src/ares_reliability/test/test_recovery_manager.py \
  src/ares_reliability/test/test_week4_analysis.py \
  src/ares_reliability/test/test_week5_analysis.py

python3 -m pytest -q src/ares_reliability/test \
  -k 'not flake8 and not mypy and not pep257'

# Use a new directory; the existing final campaign is immutable.
python3 scripts/run_week6_campaign.py \
  --seeds 2530 2531 2532 \
  --results-dir results/week6/final_campaign_reproduction

python3 scripts/analyze_week6_campaign.py \
  --campaign-dir results/week6/final_campaign_reproduction

graphify update .
```

The original final artifacts are regenerated analytically, without rerunning
simulation, by:

```bash
python3 scripts/analyze_week6_campaign.py \
  --campaign-dir results/week6/final_campaign
```

## 18. Final Week 6 verdict

The hypothesis is supported directionally and causally: all matched
unprotected fault missions failed, while two protected missions completed and
protection reduced localization and cross-track error dramatically. Healthy
protected behavior was reliable and free of false gates/probation.

Week 6 cannot be declared ready because protected fault completion was 2/3,
not consistent, and the failed protected run contained one controller
collision failure, eight planner failures, ten recoveries, a command gap, and
a Nav2 abort. The exact blocker is the pre-gate EKF displacement that leaves
Nav2 unable to plan before fault clear.

`WEEK6: BLOCKED`

`NAVIGATION_RELIABILITY: NOT_READY`
