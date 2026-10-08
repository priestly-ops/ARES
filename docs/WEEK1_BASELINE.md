# ARES Week 1 healthy reference

Status and measured acceptance are recorded separately in `WEEK1_COMPLETION_REPORT.md`. Never infer a pass from this operating guide.

## Preserved architecture

The initial audit is in `WEEK1_AUDIT.md`; original files are archived under `backups/week1_initial/`. The workspace was not a Git repository. Existing simulation geometry, warehouse/map, ROS description, Nav2 plugins and command adapter were reused. Unrelated source packages were not changed.

```text
Gazebo warehouse + four-wheel DiffDrive
  /ares/odom (50 Hz) ─┐
  /ares/imu (100 Hz) ─┼─ ros_gz_bridge ─ EKF (30 Hz) ─ /odometry/filtered
  /ares/scan (5 Hz) ──┘             └─ AMCL + Nav2 costmaps
  /clock ─────────────────────────── simulation time for all ROS components
Nav2 /cmd_vel (TwistStamped) ─ cmd_vel_adapter.py ─ /ares/cmd_vel (Twist) ─ Gazebo
```

`ares_simulation` owns the simulation adapter and launch. `ares_localization` owns estimator/navigation configuration. `ares_jackal_description` owns robot geometry/transforms. `ares_benchmark` contains a configurable ROS interface benchmark, independent of Gazebo resets. The workspace trial orchestrator starts a fresh simulation and estimator for each run.

## Build and launch

Historical commands below were developed in a terminal with WSLg graphics
access. Final Week 1 acceptance must instead run on native Ubuntu Linux. Before
sourcing or launching, verify that neither `uname -a` nor `/proc/version`
contains `microsoft` or `WSL`; stop without starting acceptance if either does.

From the validated host terminal:

```bash
cd ~/ares_ws
source /opt/ros/lyrical/setup.bash
colcon build --base-paths src/ares_simulation src/ares_jackal_description src/ares_localization src/ares_benchmark_worlds src/ares_benchmark
source install/setup.bash
ros2 launch ares_simulation ares_baseline.launch.py rviz:=false camera:=false
```

The explicit build scope avoids unrelated legacy ROS packages. Gazebo runs its server using the available graphics display for GPU LiDAR. No GUI is required. `rviz:=true` adds RViz; `camera:=true` adds the existing reduced depth sensor and its bridges. No software renderer is forced. Use Ctrl-C to stop the launch; do not run the old `start_ares.sh` concurrently.

Launch includes map server, AMCL, planner, controller, behavior server, BT navigator, and lifecycle manager. AMCL starts at the existing map's recorded spawn pose (0,0,0). A fixed seed defaults to 42. Temporary world/bridge files are unique per launch. `map:=/absolute/path/map.yaml` is supported but requires a matching mission/initial pose.

The launch derives the warehouse from the existing SDF: it embeds the existing robot and removes the three animated actors. Static obstacles, including `r1_test_obstacle`, remain. Camera-off removes the camera sensor element, retaining its physical link/mass. It does not rely on `always_on=false`. The original dynamic world remains available for later experiments.

## Sensor and TF configuration

| Stream | Sim-time rate | Frame | Configuration |
|---|---:|---|---|
| `/ares/scan` | 5 Hz | lidar_link | 360 samples, ±pi, 0.12–20 m, Gaussian sigma 0.01 m |
| `/ares/imu` | 100 Hz | imu_link | angular velocity sigma 0.0002 rad/s; acceleration sigma 0.01 m/s² |
| `/ares/odom` | 50 Hz | odom / base_footprint | Wheel-integrated DiffDrive odometry |
| `/odometry/filtered` | 30 Hz | odom / base_footprint | Planar EKF |

GNSS remains simulated but is not a navigation input or baseline bridge. Optional depth camera remains 320×240 at 5 Hz. Physics remains 1 ms. The GPU LiDAR rate was intentionally reduced from 10 Hz to 5 Hz after native Gazebo subscription isolated the 10 Hz renderer workload below the 0.5 RTF gate; samples, FOV, range, resolution, and noise are unchanged. The original 10 Hz target remains pending on a stronger native graphics path.

```text
map                       AMCL authority
 └─ odom                  EKF authority below this frame
     └─ base_footprint
         └─ base_link     robot_state_publisher fixed z=0.18
             └─ chassis_link
                 ├─ imu_link       (0,0,0.10)
                 ├─ lidar_link     (0.10,0,0.22)
                 ├─ camera_link    (0.20,0,0.12)
                 ├─ gps_link
                 └─ four wheel links (z=-0.082 relative to chassis)
```

The Gazebo DiffDrive `/ares/tf` stream is deliberately not bridged. Wheel links retain the existing fixed ROS visualization; physical wheel joints are revolute. There is one dynamic authority per child. Camera link can exist even when its sensor is off.

## EKF and localization

`ekf_baseline.yaml` fuses wheel forward/lateral velocity and IMU yaw rate. Wheel pose and wheel yaw rate are excluded to avoid correlated wheel pose/twist fusion and skid-steer heading errors. IMU absolute orientation is not fused because it is Gazebo-world referenced. Acceleration is not fused. EKF owns odom -> base_footprint; AMCL owns map -> odom. Unobserved global pose covariance grows during dead reckoning; that alone is not a localization failure.

Raw Gazebo odometry covariance is zero (unspecified); robot_localization applies its numerical covariance floor. This idealized simulator input is not a calibrated real-robot uncertainty model. IMU angular/acceleration covariances are populated from simulated noise. Later fault comparisons must preserve these choices or version the baseline.

The saved map is reused byte-for-byte at 0.05 m resolution. Its origin is not grid-aligned, which Nav2 warns about. The added static test obstacle is sensed by the obstacle layers even though absent from the older saved map.

## Mission and acceptance

Mission parameters are in `src/ares_benchmark/config/mission.yaml`. A fresh launch starts at world (0,-7,0.10), yaw +pi/2; localization starts at map (0,0,0). The route sends map goals (2,0,0), (6,-2,0), and (6,2,pi/2). It includes straight travel, turns and obstacle-aware planning.

In another sourced terminal, on the same ROS domain as the manually launched baseline:

```bash
ros2 run ares_benchmark health_check --output results/week1/manual_health.json --ros-args -p use_sim_time:=true
ros2 run ares_benchmark healthy_mission --output results/week1/manual_mission.json --ros-args -p use_sim_time:=true
```

Output paths must be new. `health_check` does not reset localization. The mission initializes localization and therefore must start only from the fixed fresh spawn; it is not a reset tool for a robot already moved elsewhere.

Checks require publishers and fresh valid data, advancing stamps, expected frames, sim-time rates within 25%, TF connectivity, one dynamic TF authority, active lifecycle states, initialized AMCL, and effective RTF >=0.5. Mission acceptance additionally requires all actions succeeded, each XY error <=0.30 m and yaw error <=0.35 rad, with bounded wall-time waits. Goal errors use map->base TF and are not independent ground-truth localization errors.

## Recorded trials

Stop any existing Gazebo/bridge instance first. The unattended runner refuses contaminated process baselines; it never broadly kills unrelated processes.

```bash
cd ~/ares_ws
scripts/run_healthy_trials.sh --runs 20
```

Each trial starts a fresh Gazebo/ROS launch, runs preflight and the fixed mission, records a bag, checks its metadata, and cleans up owned processes. It assigns a fresh ROS domain in the 100–199 range and a unique Gazebo partition to each run; avoid concurrent ROS workloads in that range during a batch. Results go to `results/week1/healthy/run_<timestamp>_<index>/`:

* `metrics.json`: result, failures, wall/sim duration, goal errors, rates, RTF, path length, commands, estimator diagnostics and configuration.
* `bag/`, `bag_info.txt`, `bag.log`: reference recording and validation.
* `launch.log`, `mission.log`, `ros_logs/`: runtime evidence.
* `manifest.json`: SHA256 hashes of source inputs.

`baseline_summary.csv` contains trial summaries. Failed attempts remain visible. `--runs 2 --output results/week1/my_trials` selects a smaller independent batch. `--health-only` performs recording and stationary health acceptance without sending goals.

For a manual recording on the currently active ROS domain:

```bash
scripts/record_healthy_bag.sh results/week1/manual_reference
```

Recordings include raw sensors, filtered odometry, AMCL, TF/static TF, commands, plan, clock, diagnostics and Nav2 action status/feedback. Ctrl-C finalizes the bag. Results, bags, logs and backups are ignored by the new `.gitignore`; no giant bag is intended for Git.

## Metrics interpretation

Rates use `(message_count-1)/(last_header_stamp-first_header_stamp)` in simulated seconds. RTF uses simulation-time progress divided by monotonic wall-time progress; performance probes independently use long-window Gazebo statistics deltas. They do not average instantaneous RTF samples.

Path length integrates filtered planar odometry; maximum planned path length is recorded separately. Velocity metrics report mean/max absolute forward and yaw commands. AMCL covariance samples and asynchronous AMCL-to-TF discrepancy provide future comparison diagnostics, not independent accuracy. No ground-truth accuracy claim is made.

## Performance investigation

All measured tests used fresh owned processes, unique Gazebo partitions, and ~30-second statistics windows after startup. Raw statistics and process snapshots are under `results/week1/performance/`.

| Test | Effective RTF |
|---|---:|
| Original world, Gazebo only | 0.991 |
| Original world, command/odom/IMU bridge | 0.992 |
| Original world, full navigation sensors/clock bridge | 0.175 |
| Original world, native Gazebo scan subscriber, no ROS bridge | 0.182 |
| Actors retained, camera removed, full sensor bridge | 0.190 |
| Static warehouse, camera removed, full sensor bridge | 0.685 |
| Same static case, forced software rendering | 0.652 |
| Static baseline + RSP/EKF | 0.646 |
| + map server/AMCL | 0.613 |
| + full Nav2 (stationary, no bag) | 0.553 |

The reproduced bottleneck is subscriber-activated rendering of the animated scene, predominantly the actors. Gazebo-only without a scan subscriber is not equivalent to a functioning navigation sensor workload. Basic ROS bridge transport did not reproduce the slowdown. Camera and RViz are excluded from the reference workload. The machine is WSL with 12 logical CPUs, ~7.7 GiB RAM and D3D12 Intel Iris Xe acceleration. Results are machine/load-specific; the headroom above 0.5 is limited.

Probe examples (source workspace first):

```bash
python3 scripts/week1_probe.py --name new_ekf_measurement --stage ekf
python3 scripts/week1_probe.py --name new_nav_measurement --stage nav2
python3 scripts/week1_probe.py --name new_camera_measurement --stage nav2 --camera
```

Names must be unique. Supported stages: gazebo, bridge (clock/command), sensors, ekf, localization, nav2. Refer to the completion report for actual optional-camera and repeated mission results, plus unresolved limitations.
