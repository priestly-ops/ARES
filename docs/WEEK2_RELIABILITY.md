# ARES Week 2: GNSS Consistency and Trust-State Monitoring

## Objective

Week 2 detects sensor inconsistency and GNSS deception before any recovery
behavior is added. It preserves the qualified Week 1 Cyclone DDS, llvmpipe,
LiDAR, IMU, odometry, EKF, and physics configuration.

The initial threat model is GNSS spoofing. Thresholds in this document are
experimental defaults to be tuned from recorded data; they are not claimed to
be theoretically optimal.

## Architecture and topic ownership

~~~text
Gazebo /ares/gps (gz.msgs.NavSat)
             |
             | GZ_TO_ROS bridge
             v
ROS /ares/gps_raw (sensor_msgs/NavSatFix)
             |
             | gnss_fault_injector
             v
ROS /ares/gps (sensor_msgs/NavSatFix)
             |
             +------------------------------+
                                            v
/odometry/filtered ----------------> consistency_monitor
                                            |
                          +-----------------+------------------+
                          v                 v                  v
              instantaneous residual   rolling mean      trust state
~~~

**/ares/gps_raw** is untouched simulator ground truth for validating the
injector. The consistency monitor never subscribes to it and never compares
**/ares/gps** with **/ares/gps_raw**. Detection compares the potentially
corrupted **/ares/gps** with independent **/odometry/filtered**.

The reliability launch starts only the injector, monitor, and optional
recorder. It does not start Gazebo, a bridge, EKF, RViz, or a second copy of
anything in the baseline stack.

## Fault injection

The injector keeps the verified static meter-offset behavior and provides:

| Mode | Behavior |
| --- | --- |
| none | Exact passthrough, including when enabled=false |
| step | Constant east, north, and altitude offsets |
| drift | Static offset plus deterministic east/north rate × elapsed time |
| dropout | Drops every Nth raw message; dropout_every_n=1 is full dropout |

Changing any fault parameter resets the drift epoch and dropout message
counter. **/ares/gnss_fault_status** reports the effective mode, offsets, and
whether each raw input was dropped. This keeps dropout observable even while
**/ares/gps** is silent.

Profiles are in
**src/ares_reliability/config/fault_profiles.yaml**: healthy, east_1m,
east_3m, east_5m, east_10m, slow_east_drift, north_step, and dropout.

## Residual and trust state

The monitor takes one initial GNSS reference and one initial odometry
reference. For each later GNSS message it converts latitude/longitude changes
to local east/north, reads the latest relative odometry, and computes:

~~~text
dx = gnss_east  - odometry_x
dy = gnss_north - odometry_y
residual = hypot(dx, dy)
~~~

Residual computation runs only in the GNSS callback, once per GNSS message.
The odometry callback only updates the latest odometry sample.

Experimental defaults:

- Rolling window: 5 GNSS residuals
- mean < 1.5 m: HEALTHY
- 1.5 m <= mean < 3.0 m: SUSPECT
- mean >= 3.0 m: FAULT
- A candidate must persist for 5 consecutive evaluations
- Missing GNSS becomes a FAULT candidate after 1.0 s and uses the same
  persistence logic on a 0.2 s timer

Transitions are logged once, for example
TRUST STATE: HEALTHY -> FAULT. Residual diagnostics are rate-limited by
diagnostic_log_every_n.

## Topics

| Topic | Type | Producer | Meaning |
| --- | --- | --- | --- |
| /ares/gps_raw | sensor_msgs/msg/NavSatFix | bridge | Untouched raw simulator GNSS |
| /ares/gps | sensor_msgs/msg/NavSatFix | injector | GNSS consumed by reliability logic |
| /ares/gnss_fault_status | std_msgs/msg/String | injector | JSON fault/dropout status |
| /ares/gnss_odom_residual | std_msgs/msg/Float64 | monitor | Instantaneous residual in meters |
| /ares/gnss_odom_rolling_mean | std_msgs/msg/Float64 | monitor | Rolling residual mean in meters |
| /ares/gnss_odom_diagnostics | std_msgs/msg/Float64MultiArray | monitor | east, north, odom x/y, residual, mean |
| /ares/trust_state | std_msgs/msg/String | monitor | HEALTHY, SUSPECT, or FAULT |

## Existing evidence before the recorder

The previously observed stationary healthy samples were:

~~~text
0.087, 0.152, 0.299, 0.390, 0.470, 0.520, 0.600, 0.930,
0.970, 1.010, 1.100, 1.140, 1.210, 1.380, 1.730 m
~~~

For this manually transcribed 15-sample set:

| Statistic | Value |
| --- | ---: |
| Mean | 0.799 m |
| Median | 0.930 m |
| Population standard deviation | 0.465 m |
| 95th percentile (linear interpolation) | 1.485 m |
| Maximum | 1.730 m |

This evidence rejects a single 1 m alarm threshold. It also shows why the
rolling window and persistence are important.

The previous validated +5 m step experiment, applied after healthy reference
initialization without restarting the monitor, produced ten residuals from
4.169 m to 5.645 m (mean 4.731 m). That is evidence of strong separation for
this attack, but it did not include recorder-derived detection latency.

The +1 m east injector validation used matching timestamps and measured
east=0.999 m, north=0.000 m, and altitude=0.000 m.

## Build and launch

Every terminal must use Cyclone DDS. The Gazebo terminal must also keep the
software-rendering workaround:

~~~bash
cd ~/ares_ws
source /opt/ros/lyrical/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GALLIUM_DRIVER=llvmpipe
colcon build --packages-select ares_reliability --symlink-install
source install/setup.bash
~~~

Start the existing frozen baseline in terminal A:

~~~bash
ros2 launch ares_simulation ares_baseline.launch.py \
  stage:=ekf rviz:=false camera:=false seed:=42
~~~

Start reliability healthy in terminal B:

~~~bash
ros2 launch ares_reliability reliability_test.launch.py \
  profile:=healthy record:=true \
  result_name:=manual_healthy results_dir:=$PWD/results/week2
~~~

Do not start a profile such as east_5m before monitor initialization when
testing step detection. A pre-existing constant spoof becomes the reference.
For valid step experiments, launch healthy, wait for references, and then
change the running injector.

Example +5 m attack and recovery:

~~~bash
ros2 param set /gnss_fault_injector mode step
ros2 param set /gnss_fault_injector east_offset_m 5.0
ros2 param set /gnss_fault_injector enabled true
# Observe the attack without restarting consistency_monitor.
ros2 param set /gnss_fault_injector enabled false
ros2 param set /gnss_fault_injector mode none
~~~

Use ordinary parameter commands. If parameter services are unresponsive,
check /gnss_fault_injector, its set_parameters service, and discovery; only
restart the injector if necessary. A timeout wrapper can kill the CLI during
rclpy shutdown and print !rclpy.ok(); that is not evidence of an injector
algorithm failure.

## Deterministic experiments

With the baseline already running, run each scenario in a fresh terminal. The
script always starts the monitor and injector healthy, waits for the trusted
reference period, applies the fault without restarting the monitor, removes
it, and records recovery:

~~~bash
cd ~/ares_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GALLIUM_DRIVER=llvmpipe

python3 scripts/run_week2_experiment.py healthy
python3 scripts/run_week2_experiment.py east_1m
python3 scripts/run_week2_experiment.py east_3m
python3 scripts/run_week2_experiment.py east_5m
python3 scripts/run_week2_experiment.py east_10m
python3 scripts/run_week2_experiment.py slow_east_drift --attack-sec 30
python3 scripts/run_week2_experiment.py north_step
python3 scripts/run_week2_experiment.py dropout
~~~

Every non-healthy scenario includes spoof-enabled to disabled recovery. To
measure repeatability, run strong attacks at least three times rather than
inferring consistency from one sample.

Analyze recorded files:

~~~bash
ros2 run ares_reliability analyze_week2_results \
  results/week2/*.csv \
  --output results/week2/summary.json
~~~

Each CSV contains simulator timestamp, event type, profile, fault mode,
enabled/drop status, effective offsets, local GNSS east/north, odometry x/y,
instantaneous residual, rolling mean, and public trust state. The analyzer
reports healthy and fault statistics, peak rolling mean, false healthy-period
transitions, FAULT detection latency, and recovery latency. It leaves
unobserved values as null; it does not invent passes.

## Limitations

**A relative GNSS-vs-odometry detector cannot reliably detect a constant spoof
already present before trusted reference initialization.** The initial GNSS
reference would include the spoofed offset, making that constant offset
invisible. Week 2 therefore accepts step attacks applied after trusted
initialization.

Other limitations:

- GNSS and odometry use latest-sample comparison rather than timestamp
  synchronization.
- Geographic conversion is a local tangent approximation suitable for these
  short experiments, not global navigation.
- Wheel odometry drift can eventually resemble GNSS inconsistency.
- A FAULT state identifies inconsistency or missing GNSS, not which sensor is
  malicious.
- The baseline does not yet contain a trusted absolute map anchor.
- Threshold tuning requires larger, timestamped healthy and attack datasets.

## Results and acceptance status

Live experiments were run on 2026-09-25 with the frozen baseline, Cyclone DDS,
llvmpipe, camera off, and RViz off. Raw GNSS averaged 8.95 Hz and filtered
odometry averaged 26.28 Hz in wall time while Gazebo was running below real
time. Both streams remained continuous.

The dedicated 40-second stationary healthy recording contained 280 residuals:

| Statistic | Value |
| --- | ---: |
| Mean | 0.842 m |
| Median | 0.820 m |
| Population standard deviation | 0.372 m |
| 95th percentile | 1.493 m |
| Maximum | 2.060 m |
| False SUSPECT transitions | 0 |
| False FAULT transitions | 0 |

Across all pre-attack healthy intervals there were 2,114 residual samples:
mean 0.813 m, median 0.773 m, population standard deviation 0.414 m, 95th
percentile 1.565 m, and maximum 2.774 m. One long pre-drift interval produced
five transient SUSPECT transitions. No pre-attack interval produced a false
FAULT transition.

Recorded attack results:

| Scenario | Fault residual evidence | FAULT latency | Recovery latency | Result |
| --- | --- | ---: | ---: | --- |
| +1 m east | mean 1.525 m; peak rolling mean 1.882 m | No FAULT | 0.403 s from transient state | Characterized |
| +3 m east | mean 3.100 m; range 2.066–4.123 m | 2.001 s | 0.703 s | Detected |
| +5 m east, run 1 | mean 5.404 m; range 4.253–6.122 m | 0.601 s | 0.802 s | Detected |
| +5 m east, run 2 | mean 4.447 m; range 3.449–5.458 m | 0.703 s | 0.702 s | Detected |
| +5 m east, run 3 | mean 5.250 m; range 4.398–6.278 m | 0.602 s | 0.802 s | Detected |
| +10 m east | mean 9.838 m; range 8.885–10.649 m | 0.501 s | 0.803 s | Detected |
| 0.2 m/s east drift | mean 2.790 m; max 5.104 m | 11.801 s | 0.801 s | Detected |
| +5 m north | mean 5.228 m; range 4.096–6.612 m | 0.601 s | 0.800 s | Detected |
| Full dropout | 77 dropped-message status rows | 1.899 s | 0.400 s | Detected |

All three +5 m repeats reached FAULT and recovered to HEALTHY. Recovery
latency is measured from the first injector status showing the fault disabled.
Dropout has no fault-period residuals by definition, so its residual
statistics are null rather than fabricated.

### Threshold analysis

The 1.5 m HEALTHY boundary is close to the dedicated healthy p95 (1.493 m) and
below the combined healthy p95 (1.565 m). It therefore allows occasional
SUSPECT chatter, which was observed. The 3.0 m FAULT boundary plus
five-evaluation persistence produced zero false healthy-period FAULT
transitions and detected +5 m in all three repeats.

Keep 1.5 m / 3.0 m as the experimental Week 2 defaults for reproducibility;
the dataset is not broad enough to silently retune them. A future candidate
is a modestly higher HEALTHY boundary or asymmetric state-exit hysteresis to
reduce SUSPECT chatter, but it must be evaluated against moving-robot data and
slow attacks before adoption. The +3 m result is borderline by construction
and should not be used alone to optimize the FAULT threshold.

### Acceptance decision

Week 2 passes the stated baseline acceptance criteria with recorded evidence:
raw GNSS remained separate, injected GNSS and odometry were live, the existing
+1 m conversion remains covered by the prior timestamp-matched 0.999 m
measurement and new unit tests, strong +5 m attacks reached FAULT in 3/3
repeats, all injected faults recovered, drift/dropout were represented, and
the package test suite passed. The Week 1 source/configuration was not changed;
its baseline launch received a live EKF-stage smoke test. The full Week 1
20-trial campaign was not rerun.

Machine-readable per-run details are in results/week2/summary.json.

## Week 3 starting point

Start Week 3 by separating detection from recovery policy. Preserve the
recorder and add a trust-aware estimator input gate or covariance inflation
strategy. In parallel, design trusted initialization using an absolute map
anchor or independent localization, and add timestamp-aligned IMU/odometry and
LiDAR/localization residual channels. Do not use /ares/gps_raw as a runtime
detector input.
