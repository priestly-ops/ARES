# Week 6 Runtime Stabilization

Status: **implementation complete; controlled validation failed; Week 6 remains
BLOCKED**.

This document records a Week 6-only runtime-performance change. It does not
change scientific acceptance criteria and it does not declare Week 6 PASS.
`final_campaign_v2` and `final_campaign_v3` are immutable and are not modified.

## 1. Pre-change architecture audit

This section was written before the runtime implementation was changed.

### Clock ownership and flow

The source clock is Gazebo Sim's `/clock`. The world physics configuration is:

```xml
<max_step_size>0.001</max_step_size>
<real_time_factor>1.0</real_time_factor>
```

`ares_baseline.launch.py` loads `bridge_baseline.yaml` and generates a private
bridge configuration. That configuration gives `ros_gz_bridge`'s
`parameter_bridge` a direct, unthrottled Gazebo-to-ROS mapping:

```text
Gazebo /clock (gz.msgs.Clock)
  -> ros_gz_bridge parameter_bridge
  -> ROS /clock (rosgraph_msgs/msg/Clock)
  -> every node with use_sim_time=true
```

The baseline launch appends `use_sim_time=true` to every ROS node it creates.
The Week 6 launch also explicitly enables simulation time on its own estimator,
Nav2, state-publisher, health-monitor, and adapter nodes. It includes the Week
4 reliability launch with `use_sim_time=true`; that launch applies the common
parameter to the fault injectors, consistency monitors, trust engine, recovery
manager, and GNSS trusted proxy.

The bridge owns the only pre-change ROS `/clock` publisher. Sensor bridges are
independent mappings and retain the timestamps contained in their messages.

### In-situ pre-change measurement

A protected/healthy seed-2530 stack was launched without starting the mission
or creating a campaign result. The current unbounded clock path showed:

- one `/clock` publisher, owned by the bare-DDS `ros_gz_bridge` endpoint;
- 30 `/clock` subscriptions;
- observed wall-delivery rate varying from approximately 336 to 355 Hz;
- Gazebo still producing 1 ms simulation increments, so wall delivery scaled
  with the loaded simulator RTF rather than changing the physics step;
- reliability Python processes at approximately 45--57% CPU;
- `controller_server` at approximately 20% in this particular snapshot;
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in the bridge, controller, and every
  sampled reliability process.

The controller snapshot does not replace the earlier reproduced CycloneDDS
100--110% idle observation. Both are retained as runtime evidence because host
load and simulator RTF differed.

The pre-change launch printed the bridge creation explicitly:

```text
Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock)
  -> /clock (rosgraph_msgs/msg/Clock)]
```

### Existing RMW override chain

The Week 6 runner sets CycloneDDS in its subprocess environment. The outer
Week 6 launch sets CycloneDDS again. The included Week 4 launch also sets
CycloneDDS before creating reliability processes. Therefore changing only the
calling shell cannot deterministically move the complete Week 6 graph to Fast
DDS.

### Selected Week 6-only insertion boundary

The selected architecture keeps physics and sensor production untouched:

```text
Gazebo genuine /clock at native simulation-step cadence
  -> ros_gz_bridge
  -> /ares/week6/raw_clock             (Week 6 private)
  -> C++ latest-sample clock boundary  (100 Hz maximum, wall scheduled)
  -> /clock                            (exactly one consumer publisher)
  -> Week 6 use_sim_time consumers
```

The boundary forwards the most recent genuine Gazebo `Clock` message without
altering its timestamp. It publishes only when a newer sample exists, rejects
regressions, and never synthesizes time. Sensor message headers do not traverse
this boundary and are untouched.

Outside Week 6, the baseline launch keeps its existing default direct mapping
to `/clock`. Week 4 and Week 5 keep their existing default RMW and simulation
time behavior.

## 2. Established diagnostic evidence

The detailed experiment is retained in `docs/WEEK6_CPU_DIAGNOSTIC.md`:

- plain 30 Hz rclpy subscriber: 4.78% CPU;
- `message_filters.Subscriber`: 4.58%;
- Subscriber plus Cache: 4.58%;
- TF at 30 Hz: 4.38%;
- 10 Hz timer: 1.00%;
- implicit simulation time at approximately 1 kHz clock: 56.58%;
- production-shaped Cache plus odometry plus 1 kHz clock: 60.77%;
- same isolated workload with 100 Hz clock: 13.95%.

This excludes `message_filters.Cache` as the cause and identifies clock fan-out
as the dominant Python-node load.

## 3. Runtime-quality metrics policy

The implementation records consumer clock frequency and period bounds, plus
RTF samples measured over one-second wall-time windows from simulation-clock
advance divided by wall-time advance. The reported distribution is median,
p10, and p90. A severe RTF collapse is a complete one-second measurement
window with RTF below **0.50**. Controller missed-rate warnings are counted
from the retained launch log.

These are diagnostic/runtime-quality fields only. They do not silently alter
mission classification, analyzer thresholds, waypoint acceptance, fault
handling, or recovery semantics.

## 4. Final Week 6-only implementation

`ares_baseline.launch.py` now accepts an opt-in `clock_ros_topic` argument.
Its default is still `/clock`. The Week 6 launch alone passes
`/ares/week6/raw_clock`, so the generated `ros_gz_bridge` configuration maps
the genuine Gazebo clock to that private ROS topic. No sensor mapping passes
through the clock boundary and no sensor header stamp is modified.

The new compiled `ares_simulation/week6_clock_boundary` node:

- retains the most recent genuine raw `Clock` sample;
- publishes that sample from one 100 Hz wall timer;
- never changes the contained Gazebo timestamp;
- publishes nothing if no new genuine timestamp is available;
- rejects duplicate and backward timestamps;
- exposes source/output counts, mean rates, and drop counts on
  `/ares/week6/clock_boundary_status`;
- rejects identical input/output topics, preventing a feedback configuration;
- owns the only consumer-facing `/clock` publisher in the Week 6 graph.

A Gazebo reset within one launch is treated as a backward jump and is dropped,
so consumers never observe time moving backward. The stream resumes only when
genuine source time exceeds the last published time. A new run starts a new
boundary process and therefore starts with clean state.

Both `scripts/run_week6_navigation.py` and
`week6_navigation.launch.py` explicitly select `rmw_fastrtps_cpp`. The included
Week 4 launch receives that value through its existing launch argument. The
Week 4 default remains `rmw_cyclonedds_cpp`; Week 5 was not changed. Every new
run metadata file records `rmw_implementation` and the clock-boundary topics
and rate.

The mission result now includes a `runtime_quality` object containing consumer
clock rate and observed period bounds, one-second-window RTF median/p10/p90,
the count of windows below RTF 0.50, clock regressions, boundary status, and
controller missed-rate warnings. None of these fields participates in mission
classification or existing acceptance analysis.

## 5. Clock-boundary development probes

All failed probes were retained rather than overwritten:

1. The first launch was rejected by the Codex network sandbox before DDS could
   initialize (`getifaddrs` and UDP socket `EPERM`). Its log is retained under
   `results/week6/runtime_stabilization/test_a_idle_healthy_protected_s2530/`.
2. A source-callback implementation removed the original double limiter but
   quantized output against the approximately 450 Hz loaded raw source. It
   produced approximately 91 Hz and was retained under
   `test_a_attempt2_idle_healthy_protected_s2530/`.
3. The final single-timer implementation converged to 99.6 Hz and is retained
   under `test_a_attempt3_idle_healthy_protected_s2530/`.

The original double limiter was specifically rejected: a 100 Hz timer plus a
second strict 10 ms state check skipped normally jittered timer callbacks and
produced only about 63.7 Hz. The final node has one rate authority: its 100 Hz
wall timer.

## 6. Test A -- idle protected stack

Protected, healthy, seed 2530 passed the final controlled idle check:

- consumer `/clock`: one publisher (`week6_clock_boundary`), 30 subscribers;
- raw clock: one publisher and one boundary subscriber;
- consumer rate: 99.6 Hz at the end of the direct sample;
- directly observed raw rate: approximately 407 Hz under the loaded RTF;
- boundary drop counts: zero backward and zero duplicate;
- all managed Nav2 nodes became active;
- every sampled critical process had
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`;
- `controller_server` idle CPU: 13.1%;
- critical Python reliability-node CPU: 30.2--38.6%;
- controller missed-rate warning count: zero.

The Python range is lower than the retained pre-change 45--57% stack snapshot,
although it remains higher than the isolated 100 Hz probe because the full
nodes also process their sensor and TF workloads. The controller result is
well below both the reproduced CycloneDDS 100--110% hot path and the earlier
Fast DDS comparison range of roughly 17--20%.

## 7. Test B -- healthy navigation

The single protected/healthy seed-2530 navigation attempt passed and is
retained under `runtime_stabilization/test_b_healthy/`:

- 5/5 waypoints reached;
- final localization distance: 0.190137 m (limit remains 0.30 m);
- GNSS gates: zero;
- probation events: zero;
- Nav2 aborts: zero;
- lifecycle and controller failures: zero;
- controller missed-rate warnings: zero;
- consumer clock mean: 99.296 Hz;
- clock backward jumps: zero.

## 8. Test C -- protected GNSS fault

The single protected `gnss_step_5m`, seed-2532 attempt **failed** and was not
rerun. It is retained under `runtime_stabilization/test_c_fault/`.

The failure occurred before navigation and before fault onset. During lifecycle
bring-up, `bt_navigator` could not find the `compute_path_through_poses` action
server, failed configuration, and remained inactive. The other four managed
nodes were active. The mission then reached the unchanged 120 s readiness
wall-time limit and recorded:

```text
runner_error: Week 6 readiness timed out; ... 'bt_navigator': 'inactive'
```

Consequently this attempt provides no evidence about +5 m fault detection,
pre-fusion rejection, trust transition, protected gating, or recovery. It is a
startup failure, but under the one-attempt rule it makes `FAULT_2532: FAIL` and
prevents readiness for `final_campaign_v4`. Controller missed-rate warnings
were zero. No launch delay, acceptance threshold, or scientific parameter was
changed after this failure.

## 9. RTF and timing evidence

The pre-change seed-2530 result did not record an RTF distribution. Its mission
times imply only a coarse whole-mission average of 88.207 simulated seconds /
162.501 wall seconds = **0.543**. The new healthy result records the required
distribution directly:

- RTF median: 0.426412;
- RTF p10: 0.389350;
- RTF p90: 0.500311;
- severe-collapse threshold: RTF below 0.50 for a complete one-second window;
- severe-collapse count: 190 of 212 windows.

The new whole-mission average is 74.496 / 180.307 = 0.413. Thus this host was
more heavily loaded during the controlled run even though middleware and clock
CPU improved. RTF is diagnostic evidence, not a silently added acceptance
criterion. The retained old and new seed-2530 logs both contain zero controller
missed-rate warnings under the documented warning matcher.

## 10. Regression and integrity checks

- 92 targeted Week 4, Week 5, Week 6, recovery-manager, navigation-metric, and
  launch/configuration tests passed.
- Six `ares_simulation` tests passed, including five clock-boundary tests for
  rate configuration, latest-timestamp forwarding, monotonicity, duplicate and
  backward/reset rejection, and high-rate input bounding.
- The four scientific Week 6 configuration hashes are byte-for-byte identical
  to the hashes recorded by immutable `final_campaign_v3`.
- Configuration tests confirm the Week 4 RMW default remains CycloneDDS and the
  baseline clock default remains the direct `/clock` mapping.
- `final_campaign_v2` and `final_campaign_v3` directory modification times are
  unchanged.

The aggregate package lint harness also exposed unrelated environment/existing
issues: flake8 cannot create its multiprocessing forkserver socket in the
sandbox, xmllint cannot download the ROS package schema, and mypy reports an
existing annotation error in `consistency_monitor.py`. These do not invalidate
the passing targeted regressions, but they are not hidden as a clean aggregate
lint pass.

## 11. Scientific-integrity conclusion

The runtime boundary does not alter Gazebo physics, sensor generation/rates,
sensor timestamps, fault scheduling in simulation time, mission waypoints,
the 0.30 m success threshold, navigation tuning, geometry, trust/fault/recovery
semantics, EKF fusion semantics, ground-truth restrictions, or analyzer
criteria. The healthy mission shows that the bounded clock preserves navigation
behavior under Fast DDS. The fault attempt did not reach its scientific
stimulus, so no protection or recovery claim can be made and Week 6 remains
**BLOCKED**.

## 12. Week 6 BT discovery startup stabilization

The original controlled protected `gnss_step_5m`, seed-2532 Test C remains
preserved under `results/week6/runtime_stabilization/test_c_fault/`. It failed
before navigation or fault onset because `bt_navigator` could not discover the
`compute_path_through_poses` action server within its original
`wait_for_service_timeout` of 1000 ms. The lifecycle manager aborted bring-up,
left `bt_navigator` inactive, and the mission later recorded its readiness
timeout. That failed attempt has not been overwritten or reclassified.

A live successful-stack probe accepted
`ros2 param set /bt_navigator wait_for_service_timeout 5000` with `Set parameter
successful`. The controlled fix therefore adds only
`wait_for_service_timeout: 5000` beneath `bt_navigator.ros__parameters` in the
Week 6-only `nav2_week6_navigation.yaml`. `default_server_timeout` and every
controller, planner, costmap, mission, estimator, trust, fault, recovery,
clock-boundary, and RMW setting remain unchanged. The installed configuration
was rebuilt from `ares_reliability`, and live inspection during the retry
reported `Integer value is: 5000`.

The exactly-one retry is retained at
`results/week6/runtime_stabilization/test_c_fault_retry1/gnss_step_5m/gnss_step_5m_protected_s2532/`.
Its startup and scientific results were:

- all five managed Nav2 nodes were active before and after the mission;
- `bt_navigator` created both `NavigateToPoseWReplanningAndRecovery` and
  `NavigateThroughPosesWReplanningAndRecovery` behavior trees;
- Fast DDS was recorded as `rmw_fastrtps_cpp`, with one consumer-facing
  `/clock` publisher;
- the +5 m fault ran from mission time 15.001 s to 45.004 s;
- 18 `GNSS PRE-FUSION REJECT` events rejected the injected bad GNSS;
- GNSS trust transitioned `HEALTHY -> DEGRADED -> UNTRUSTED`, protected state
  entered `GATED`, then recovery traversed `PROBATION -> NORMAL` and trust
  returned through `DEGRADED -> HEALTHY`;
- 5/5 waypoints were reached, final distance was 0.105205 m, and Nav2 aborts
  and controller-rate missed warnings were both zero;
- consumer clock mean was 98.9291 Hz (minimum/maximum observed periods
  0.000520/0.054120 s); and
- RTF median/p10/p90 were 0.441323/0.375930/0.471904. There were 220 severe
  collapse windows at the already documented diagnostic threshold RTF < 0.50.

The 51 targeted Week 4/5/6 configuration, navigation-metric, and
recovery-manager regression tests passed. The retry satisfies the controlled
Test C requirements, but the RTF distribution remains diagnostic evidence and
does not alter scientific acceptance criteria. No `final_campaign_v4` was run;
Week 6 remains **BLOCKED** until a clean immutable campaign is executed and
independently analyzed.
