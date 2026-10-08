# ARES Week 1 completion report

Date: 2026-09-25 (updated; original audit 2026-09-23)

## Status

**BLOCKED.** The current, explicitly approved Week 1 simulation profile uses a
5 Hz GPU LiDAR. A complete 20-run WSL mission suite was executed on
2026-09-24/25 after three clean preflights, but only 9/20 runs passed all gates.
Twelve runs met the unchanged effective-RTF gate of 0.5, fourteen shut down
cleanly, and six ended with `parameter_bridge` signal 11. The observed RTF
ranged from 0.188 to 0.865 under unchanged source and configuration.

The completed WSL suite is a failed acceptance result, not an incomplete pass.
It shows two independent blockers: unstable WSL performance and an intermittent
Fast DDS bridge teardown race. It also cannot satisfy the separate requirement
for final acceptance on a native-Ubuntu host.

Earlier partial-run and diagnostic evidence remains preserved below for audit
history. The trial runner records child-process crashes in each result, updates
the CSV and aggregate status, and returns nonzero when any run fails.

### Native final-acceptance attempt (2026-09-23)

The requested native-Ubuntu final acceptance could not start because the
execution environment supplied for the attempt is still WSL2. The mandatory
host check reported kernel
`6.18.33.2-microsoft-standard-WSL2`; `/proc/version` contains the same WSL2
identifier. This directly fails the requirement that neither check contain
Microsoft/WSL. `glxinfo -B` also could not open display `:0`, and `lspci` is
not installed in the session.

The run stopped at the environment gate. No build, preflight, or 20-run suite
was started, and no prior result was overwritten. The retained evidence is
`results/week1/native_final_acceptance_attempt_20260923_2200/environment_gate.txt`.
Week 1 therefore remains **BLOCKED**, not PASS. Resume only from a shell whose
kernel and `/proc/version` both prove native Ubuntu Linux.

## Configuration under test

The current Week 1 navigation profile is:

- GPU LiDAR at 5 Hz, 360 samples, resolution 1, full 360-degree FOV;
- range 0.12-20.0 m with 0.01 m resolution;
- Gaussian range noise mean 0.0 and standard deviation 0.01 m;
- `/ares/scan` in `lidar_link`;
- IMU at 100 Hz and raw wheel odometry at 50 Hz;
- EKF output at 30 Hz;
- 1 ms physics step, camera off, RViz off;
- unchanged RTF threshold (0.5), position tolerance (0.30 m), yaw tolerance
  (0.35 rad), and rate tolerance (25%).

The original 10 Hz profile did not pass on this WSL/Intel Iris Xe runtime. It
remains the target profile for validation on a stronger native graphics path;
this report does not claim that 10 Hz passed.

## Source integrity

The current ARES package source matches the manifest recorded by the last complete
5 Hz batch run. Relative to the passing 10 Hz rotation-fix trial, exactly two
source files differ:

- `src/ares_simulation/models/ares_jackal/model.sdf`: LiDAR update rate 10 Hz
  to 5 Hz;
- `src/ares_benchmark/config/mission.yaml`: expected scan rate 10 Hz to 5 Hz.

No samples, FOV, range, resolution, noise, bridge, mission tolerance, physics,
or unrelated ARES source was changed for the 5 Hz rerun.

## Performance isolation

The controlled measurements show that scan rendering, not the ROS/Gazebo
bridge itself, is the dominant cost:

| Stage | Effective RTF |
|---|---:|
| World only | approximately 0.991 |
| ARES Gazebo stage | approximately 0.987 |
| Minimal bridge | approximately 0.904 |
| Odometry-only bridge | approximately 0.946 |
| IMU-only bridge | approximately 0.937 |
| 10 Hz GPU LiDAR, native Gazebo scan subscriber | approximately 0.415 |
| 10 Hz GPU LiDAR, full sensor bridge | approximately 0.408 |
| 5 Hz GPU LiDAR, native Gazebo scan subscriber | 0.800 |
| 5 Hz GPU LiDAR, full sensor bridge | 0.744 |
| 5 Hz full-sensor preflight before the partial batch | 0.711 |
| 5 Hz Nav2 diagnostic before the partial batch | 0.901 |

Native Gazebo subscription reproduced the 10 Hz collapse without
`ros_gz_bridge`, ruling out bridge transport as the primary bottleneck. A CPU
LiDAR experiment using SDF sensor type `lidar` preserved the sensing parameters
but never advertised `/ares/scan` in the installed Gazebo Sim 10.5.0 runtime;
that invalid experiment was reverted.

## Partial 5 Hz mission batch

Evidence is in
`results/week1/week1_5hz_acceptance_fresh_domains_20260923_154636/`.

| Metric | Result |
|---|---:|
| Complete runs | 11 |
| Runtime mission passes | 11/11 |
| Requested-suite progress | 11/20 |
| RTF min / mean / median / max | 0.577 / 0.650 / 0.650 / 0.825 |
| RTF population standard deviation | 0.069 |
| All-waypoint position error min / mean / max | 0.163 / 0.192 / 0.202 m |
| All-waypoint yaw error min / mean / max | 0.001 / 0.151 / 0.302 rad |
| Final position error min / mean / max | 0.193 / 0.195 / 0.198 m |
| Final yaw error min / mean / max | 0.077 / 0.145 / 0.216 rad |

Observed simulation-time sensor rates across the 11 runs were:

| Stream | Minimum | Mean | Maximum |
|---|---:|---:|---:|
| `/ares/scan` | 5.000 | 5.001 | 5.009 Hz |
| `/ares/imu` | 99.509 | 99.748 | 100.000 Hz |
| `/ares/odom` | 49.799 | 49.921 | 50.000 Hz |
| `/odometry/filtered` | 29.963 | 29.990 | 30.004 Hz |

Every complete run recorded a valid bag, all three Nav2 goals succeeded, and no
launch or bag process exited before controlled shutdown. The twelfth directory,
`run_20260923_162920_012`, ended during its first goal and contains a zero-byte
`metrics.json`; it is retained as interruption evidence.

## Resume-session blocker

The resumed WSL instance booted on kernel
`6.6.123.2-microsoft-standard-WSL2+`. GLX reports direct, accelerated D3D12
rendering on Intel Iris Xe with Mesa 26.0.8. Nevertheless, `dxg` adapter-query
failures appeared six seconds after WSL boot and recurred at Gazebo launches.

Fresh measurements in this session were:

| Test | Effective RTF | Result |
|---|---:|---|
| Sensors only | 0.554 | PASS |
| Nav2, no bag | 0.409 | FAIL |
| Nav2, agent proxy isolated to CPUs 0-1 | 0.534 | PASS |
| Full health + bag, normal scheduling | 0.315 | FAIL |
| Full health + bag, proxy CPUs 0-1 / benchmark CPUs 2-11 | 0.494 | FAIL |
| Full health + bag, proxy CPU 0 / benchmark CPUs 1-11 | 0.475 | FAIL |

The full health checks otherwise passed: scan 5.0 Hz, IMU 100 Hz, raw odometry
50 Hz, filtered odometry approximately 30 Hz, TF connectivity, single dynamic
TF authority, lifecycle state, localization, clock advancement, and bag
validation. Only effective RTF failed.

The affinity experiment shows that the active agent proxy materially affects
this marginal workload, but it is not a sufficient explanation: both isolated
full-stack trials remained below 0.5. No cgroup CPU quota is active, memory and
swap are healthy, and accelerated rendering is reported. The recurring `dxg`
errors and sensitivity to host load make the current WSL graphics/runtime state
the remaining performance blocker.

## Full-stack nondeterminism investigation

The 20-run suite was not run. A known-good mission
(`run_20260923_161108_001`, RTF 0.825) was compared with the resumed failing
health trial (`run_20260923_211651_001`, RTF 0.315).

### Known-good versus failing run

| Item | Known-good | Failing | Finding |
|---|---|---|---|
| ARES package manifest | 58 files | 58 files | Byte-for-byte identical |
| Kernel | 6.6.123.2 WSL2 custom kernel | 6.6.123.2 WSL2 custom kernel | Same kernel build |
| `.wslconfig` | custom 6.6 kernel | same file | File predates both runs |
| Gazebo | 10.5.0 | 10.5.0 | No package change |
| Mesa / renderer | Mesa 26.0.8, D3D12 Intel Iris Xe | same | No package change |
| Trial affinity | default CPUs 0-11 | default CPUs 0-11 | No taskset restriction |
| LiDAR / IMU / odometry rates | 5.0 / 99.6 / 49.9 Hz | 5.0 / 100.0 / 50.0 Hz | Healthy in both |
| Filtered odometry | 30.0 Hz | 30.0 Hz | Healthy in both |
| Bridge and launch ordering | standard baseline | standard baseline | Same files and order |
| Nav2 activation from first ROS log | 7.10 s | 14.70 s | Slow run took 2.1x longer |
| Stale Gazebo / bridge process | none | none | Both clean baselines |
| Bag | valid | valid | Not the functional failure |
| Teardown | Gazebo and bridge -11 | Gazebo and bridge -11 | Same independent defect |

The exact hashes shared by the comparison and diagnostic matrix are:

- model SDF: `48278e388ded9e07857ab3f0b5083618bb43d0221cf14386eb1e9cd10c1dfcf2`;
- mission config: `2c0c01ebcfe15a8ddd943cf6aed8e04bac8e80547eb9d18a8e3ab1b1c603ffe9`;
- baseline bridge: `cd9b1cb8d78870419b376abde95748e0c73264a2094580e706870c5984426558`;
- baseline launch: `c78de2b0095f284b493c364ca92565586dd7c87b4b087a773ae1feddf44b6115`.

The closest retained process snapshots are the 0.901 pre-batch Nav2 probe and
the 0.409 resumed Nav2 probe. Gazebo used 179% versus 164% CPU and 33 threads in
both; the bridge used 48.8% versus 47.1% CPU and 26 threads in both. The major
observable difference was the active headroom proxy: 26.8% CPU in the fast
snapshot versus 77.2% in the slow snapshot. Its memory was approximately
25.4% versus 22.6%, so memory capacity was not the differentiator. Historical
process environments were not saved; the common runner supplied the same
environment except for the intentionally unique `ROS_DOMAIN_ID`,
`GZ_PARTITION`, and log directory.

WSL exposes neither per-core frequency nor thermal-zone sensors, so frequency
and temperature cannot be compared directly. No CPU cgroup quota is active.
The new matrix recorded zero memory-pressure PSI and no evidence of swapping or
memory exhaustion. CPU scheduling pressure was measurable.

### Controlled A-B-C-D preflight matrix

Each run used a fresh ROS domain, Gazebo partition, output directory, full Nav2
stack, bag recorder, and health check. Source hashes, versions, renderer, bridge,
launch order, and relevant process environment were identical.

| Run | Scheduling | RTF | Mean Gazebo CPU | Mean bridge CPU | Mean proxy CPU | Result before teardown |
|---|---|---:|---:|---:|---:|---|
| A | normal, CPUs 0-11 | 0.808 | 146% | 49.6% | 47.7% | PASS |
| B | proxy 0-1, trial 2-11 | 0.887 | 137% | 46.3% | 46.1% | PASS |
| C | proxy 0-1, trial 2-11 | 0.822 | 143% | 45.9% | 44.4% | PASS |
| D | normal, CPUs 0-11 | 0.767 | 149% | 50.2% | 43.1% | PASS |

Normal mean RTF was 0.787; isolated mean RTF was 0.854, an improvement of
0.067. CPU isolation therefore changes performance materially, but it is not
the sole determinant: both normal runs passed and the two isolated runs still
differed by 0.065. The earlier low state coincided with a much busier proxy and
slower Nav2 activation. The matrix supports running acceptance outside the
CPU-heavy agent path or with an explicit affinity partition, but it does not
prove that host scheduling is the only source of variance.

All A-D sensor, TF, localization, lifecycle, clock, and bag checks passed. All
four exceeded RTF 0.5. `dxg` adapter-query errors recurred at every launch,
including all four fast runs, so those boot/launch errors alone do not predict
low RTF.

Raw evidence is under `results/week1/diagnostics/`. The reusable read-only
instrumentation is `scripts/run_fullstack_diagnostic.py`.

## Shutdown SIGSEGV investigation

Gazebo and `parameter_bridge` were launched independently from mission
acceptance with an owned world, bridge, scan subscriber, ROS domain, and Gazebo
partition. This allowed direct signaling and `waitpid` return codes without ROS
launch automatically hiding the first failure.

| First process / method | Gazebo exit | Bridge exit | Kernel result |
|---|---:|---:|---|
| Bridge first, SIGINT | -11 | 0 | `gz-sim-main` SIGSEGV |
| Gazebo first, SIGINT | -11 | 0 | `gz-sim-main` SIGSEGV |
| Bridge first, SIGTERM | -11 | 0 | `gz-sim-main` SIGSEGV |
| Gazebo first, SIGTERM | -11 | 0 | `gz-sim-main` SIGSEGV |
| Gazebo `/server_control stop` | -11 | 0 | `gz-sim-main` SIGSEGV |
| Pause world, then server stop | -11 | not started | `gz-sim-main` SIGSEGV |
| Remove robot model, then server stop | -11 | not started | `gz-sim-main` SIGSEGV |
| Software-rendering request, server stop | -11 | not started | `gz-sim-main` SIGSEGV |
| Ogre 1 generated world, server stop | -11 | not started | `gz-sim-main` SIGSEGV |
| Stop bridge, wait, then server stop | -11 | 0 | `gz-sim-main` SIGSEGV |
| Subscriber-driven LiDAR, stop bridge, wait, then server stop | -11 | 0 | `gz-sim-main` SIGSEGV |
| Same generated world with LiDAR removed | 0 | not started | no kernel fault |

For the corrected pause and model-removal tests, both Gazebo services returned
`data: true` before the crash. Signal choice, shutdown order, server-control
shutdown, pausing, early entity removal, and requested software rendering do
not avoid the fault. In every crashing case, dmesg records `gz-sim-main`
faulting in `libgcc_s.so.1`; the reported CPU varies, ruling out one defective
logical core. The bridge exited cleanly in every independent reproduction, so
its occasional signal 11 under ROS launch is a concurrent-teardown symptom,
not a separately reproduced bridge defect.

The no-LiDAR control localizes the crash to destruction of Gazebo's
GPU-LiDAR/render-sensor path under this WSLg runtime. This is consistent with
Gazebo's open WSLg GPU-LiDAR crash report
([gz-sim issue 3335](https://github.com/gazebosim/gz-sim/issues/3335)) and
Gazebo Rendering's documented WSLg/D3D12 GPU-ray and LiDAR test failures
([gz-rendering issue 852](https://github.com/gazebosim/gz-rendering/issues/852)).
No ARES functional source was changed. Raw evidence is under
`results/week1/shutdown_diagnostics/`; the reproducer is
`scripts/diagnose_shutdown.py`.

A final diagnostic changed only the generated test world's LiDAR
`always_on` value to `false`, verified approximately 4.9 Hz scan delivery with
the bridge subscribed, stopped the bridge cleanly, waited five seconds, and
then requested server stop. Gazebo still exited -11. Subscriber-driven sensor
activation is therefore not a viable teardown workaround and was not applied
to the ARES model.

The installed Ogre 1 backend was also tested in a generated world with the same
5 Hz LiDAR parameters and without modifying the ARES source. It produced the
same Gazebo -11 / `libgcc_s.so.1` kernel fault. Switching between the installed
Ogre 1 and Ogre 2 backends therefore does not supply a clean teardown path.

The installed rendering stack is Gazebo Rendering 10.0.2 through
`ros-lyrical-gz-rendering-vendor` 0.4.4, Ogre Next vendor 0.2.1, and
`libgcc-s1` / `libstdc++6` 16-20260322. Core dumps are disabled and no debugger
is installed, so the retained crash-level evidence is the direct wait status,
per-process logs, and repeatable kernel fault records.

## WSL llvmpipe qualification and 20-run result (2026-09-24/25)

Software rendering with `GALLIUM_DRIVER=llvmpipe` eliminated the Gazebo
GPU-LiDAR teardown crash in focused tests. Adding lifecycle shutdown, a
five-second DDS consumer-settle interval, and explicit `parameter_bridge`
shutdown produced three valid consecutive clean health preflights at RTF
0.536, 0.506, and 0.550. A separate attempted run is excluded because the
command sandbox denied `getifaddrs` and DDS UDP socket creation before ARES
could initialize.

The gated 20-run mission suite then **FAILED 9/20**. Twelve runs met RTF >=
0.5, fourteen shut down cleanly, and six ended with `parameter_bridge` SIGSEGV
(`-11`). Five runs failed only RTF, three failed only bridge teardown, and
three failed both. RTF ranged from 0.188 to 0.865 (mean 0.548, median 0.528),
including a consecutive jump from 0.188 to 0.865 under identical source and
configuration.

The llvmpipe workaround therefore isolates and avoids the Gazebo D3D12 crash,
but it does not yield a stable acceptance configuration. The Fast DDS bridge
teardown race and WSL performance variability remain independent blockers.
Full evidence and the per-run table are in
`results/week1/diagnostics/acceptance_wsl_finalcfg_20run_20260924/ACCEPTANCE_REPORT.md`.
This WSL batch also does not satisfy the separate native-Ubuntu host gate.

## Gate status after diagnostics

1. Three consecutive clean preflights: **FAIL**. Four consecutive runtime
   preflights passed, but all four had an unexpected teardown crash.
2. Every preflight RTF >= 0.500: **PASS** for A-D (minimum 0.767).
3. No contaminated baseline: **PASS**.
4. No unexpected Gazebo or bridge shutdown crash: **FAIL** (Gazebo -11).
5. Sensor, TF, localization, and Nav2 health: **PASS**.

The 20-run suite remains prohibited until the GPU-LiDAR teardown fault is fixed
in the Gazebo/rendering/WSLg stack or an explicit acceptance waiver defines the
upstream post-mission crash as out of scope. Performance should then be
revalidated with at least three consecutive instrumented preflights outside the
active agent proxy or with a fixed CPU partition.

## Acceptance checklist

- [PASS] Simulation starts from a fresh owned process set.
- [PASS] ROS/Gazebo bridge functions during operation.
- [PASS] LiDAR publishes the documented 5 Hz / 360-sample profile.
- [PASS] IMU and raw odometry publish at their configured rates.
- [PASS] EKF publishes filtered odometry at approximately 30 Hz.
- [PASS] TF tree and single-dynamic-authority checks pass.
- [PASS] AMCL/localization initializes.
- [PASS] Nav2 lifecycle nodes activate and the fixed mission succeeds.
- [PASS] Rosbag and metrics pipelines produce valid evidence.
- [PASS] Repeated-run automation starts each trial in a fresh ROS domain and
  Gazebo partition.
- [BLOCKED] Full-stack RTF passed A-D but remains sensitive to host scheduling.
- [FAIL] Gazebo and usually the bridge report signal 11 during controlled
  teardown.
- [BLOCKED] Required 20/20 clean acceptance runs.

## Required completion run

Do not run the 20-run acceptance yet. First repair or update the
Gazebo/rendering/WSLg GPU-LiDAR teardown path, then rerun three instrumented
preflights outside the CPU-heavy agent proxy or with a fixed CPU partition.
Use new result names and do not delete any existing evidence:

```bash
cd ~/ares_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
python3 scripts/week1_probe.py --name final_5hz_sensors --stage sensors
python3 scripts/week1_probe.py --name final_5hz_nav2 --stage nav2
scripts/run_healthy_trials.sh --runs 1 --health-only \
  --output results/week1/final_5hz_health
```

Proceed to 20 runs only after three consecutive full-stack health trials meet
RTF >= 0.5 with clean child shutdown. Week 1 remains **BLOCKED**, not PASS.



