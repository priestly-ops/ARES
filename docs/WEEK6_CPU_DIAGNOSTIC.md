# Week 6 Python CPU Diagnostic

Status: **BLOCKED**. This is a diagnostic report, not a Week 6 acceptance
result. No campaign was run and no production monitor, acceptance threshold,
mission geometry, map, collision setting, or immutable result was changed.

## Conclusion

The reproducible CPU source is high-rate simulation-clock delivery to every
Python node with `use_sim_time=true`, followed by the normal rclpy executor
ready-callback/wait-set cycle. It is not `message_filters.Subscriber`,
`message_filters.Cache`, `CacheTimestampAligner`, TF, the 10 Hz timeout timer,
or `MonitorDiagnostics` by itself.

On isolated ROS domain 97 under Fast DDS, a no-op direct `/clock` subscriber
used 57.13% CPU and an otherwise empty node with the implicit
`use_sim_time` subscription used 56.58% CPU when `/clock` was driven at 1 kHz.
The production-shaped Cache + 30 Hz odometry + implicit 1 kHz clock probe used
60.77%. The same Cache probe used 4.58% with 30 Hz odometry and no clock, and
13.95% when clock delivery was reduced to 100 Hz.

This also explains why a Python sample often lands in
`event_handler.add_to_wait_set`: after each high-rate clock callback the
single-threaded executor reconstructs and re-enters its wait set. That frame is
a consequence/hot-path location; the isolated tests do not support treating
the event handler or Cache as an independently spinning entity.

## Environment

- Ubuntu 26.04 LTS under WSL2, kernel
  `6.18.33.2-microsoft-standard-WSL2`
- ROS 2 Lyrical
- Python 3.14.4
- `rclpy` 10.0.10
- `message-filters` 7.4.2
- `diagnostic-updater` 4.4.7
- RMW for every probe: `rmw_fastrtps_cpp`
- Synthetic traffic was confined to nonzero `ROS_DOMAIN_ID=97` and never
  reached the normal ROS graph.
- No probe subscribed to `/ares/ground_truth`; no command topic was published.

The Week 6 stack was no longer running when measurements began. `ros2 node
list` was empty and `/ares/odom_operational`, `/clock`, and `/amcl_pose` were
all unknown. Therefore the reported 54.8% production monitor value and the
28--34 Hz odometry value remain prior evidence supplied with this task. The
controlled replay results below are not mislabeled as in-situ campaign data.

## Code and launch inspection

All three consistency monitors construct an ordinary
`message_filters.Subscriber` and a `message_filters.Cache`, then wrap the Cache
with `CacheTimestampAligner`:

- localization: odometry Cache sized from 5 seconds at 50 Hz, a TF2
  `Buffer`/`TransformListener`, `/amcl_pose`, publishers, diagnostics, and a
  0.5 second timer;
- IMU: odometry Cache sized from 2 seconds at 50 Hz, `/ares/imu_operational`,
  publishers, diagnostics, and a 0.1 second timer;
- GNSS: odometry Cache sized from 5 seconds at 50 Hz, the operational GNSS
  subscription, publishers, diagnostics, and a configurable 0.2 second timer.

`time_sync.py` does not create ROS entities. Its lookup takes a snapshot of
the Cache lists, sorts it, applies deterministic timestamp selection, and is
only called by application callbacks. `diagnostics.py` creates a compatibility
publisher and a `diagnostic_updater.Updater`; it does not contain a busy loop.

Week 6 includes the Week 4 reliability launch with `use_sim_time=true`. The
included launch applies that common parameter to all three monitors, all three
fault injectors, the trust engine, recovery manager, and trusted proxy. The
Gazebo world has `max_step_size=0.001`, and the unthrottled bridge maps Gazebo
`/clock` directly to ROS `/clock`. That is a plausible 1 kHz fan-out and is
consistent with the isolated reproduction.

There is also checked-in RMW configuration drift that must be resolved before
another campaign:

- `scripts/run_week6_navigation.py` sets `rmw_cyclonedds_cpp`;
- `week6_navigation.launch.py` sets `rmw_cyclonedds_cpp`;
- included `week4_reliability.launch.py` also sets `rmw_cyclonedds_cpp`.

Consequently, an external `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` prefix alone
does not prove every launched process used Fast DDS. No RMW source file was
changed during this diagnosis.

## Original Cache probe exit

The installed constructor signatures are:

```text
Subscriber(node, msg_type, topic, qos_profile=..., ...)
Cache(filter, cache_size=1, allow_headerless=False)
```

Thus `Subscriber(n, Odometry, topic, 50)` validly uses integer depth 50 as the
QoS profile, and `Cache(s, 102, allow_headerless=False)` is valid for the
installed versions. There was no construction exception.

The one-line command first reproduced an immediate exit inside the restricted
diagnostic shell, but it failed before node or filter construction:

```text
RuntimeError: Failed opening file
/home/priestly/.ros/log/python3_...log for writing: Read-only file system
```

With `ROS_LOG_DIR=/tmp/ares_cache_probe_logs` on the host it printed
`CACHE_PROBE_READY` and remained alive until the five-second `timeout` sent a
signal. The resulting `ExternalShutdownException` was timeout-induced. The
observed immediate exit was therefore environmental ROS logging permission,
not incorrect `message_filters` API usage, QoS misuse, or a Python/ROS
constructor incompatibility.

Representative reproduction:

```bash
timeout 5s env \
  ROS_LOG_DIR=/tmp/ares_cache_probe_logs \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  python3 -c "import rclpy,message_filters; from rclpy.node import Node; from nav_msgs.msg import Odometry; rclpy.init(); n=Node('ares_cache_probe'); s=message_filters.Subscriber(n,Odometry,'/ares/odom_operational',50); c=message_filters.Cache(s,102,allow_headerless=False); print('CACHE_PROBE_READY', flush=True); rclpy.spin(n)"
```

## Diagnostic probe

The temporary diagnostic is `scripts/diagnose_rclpy_waitset.py`. Its target
modes are `idle`, `plain_sub`, `message_filter_sub`,
`message_filter_cache`, `timer`, `tf`, `clock_sub`, and `diagnostics`.
`--use-sim-time` independently enables rclpy's implicit `/clock`
subscription. Every mode prints `READY mode=<mode> pid=<pid>` and handles
Ctrl+C/shutdown without changing production code.

The script also has a guarded `source` mode used only for controlled replay.
It refuses to run on domain 0. It can publish synthetic odometry, clock, or TF
on an isolated domain; it never publishes robot commands.

Examples:

```bash
ROS_DOMAIN_ID=97 ROS_LOG_DIR=/tmp/ares_cpu_diag_ros_logs \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
python3 scripts/diagnose_rclpy_waitset.py message_filter_cache --use-sim-time

ROS_DOMAIN_ID=97 ROS_LOG_DIR=/tmp/ares_cpu_diag_ros_logs \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
python3 scripts/diagnose_rclpy_waitset.py source \
  --odom-rate-hz 30 --clock-rate-hz 1000
```

CPU was sampled with `top -b -d 1 -n 6 -p PID`. The reported averages were
then recorded more precisely from the delta of `/proc/PID/stat` user+system
jiffies over five wall seconds (`getconf CLK_TCK`), which is equivalent to the
per-process CPU calculation made by `top`/`pidstat`.

## Measurements

All values are percent of one logical CPU and are five-second averages unless
noted.

| Probe condition | CPU | Interpretation |
|---|---:|---|
| empty node, no traffic | 0.000% | executor blocks normally |
| plain odometry subscriber, no traffic | 0.000% | no idle spin |
| `message_filters.Subscriber`, no traffic | 0.000% | no idle spin |
| Subscriber + Cache, no traffic | 0.000% | Cache adds no idle spin |
| implicit sim time, no clock traffic | 0.0% by `top` | extra entity alone does not spin |
| 10 Hz timer only | 0.996% | small expected timer cost |
| TF listener, no TF traffic | 0.000% | listener alone blocks |
| `MonitorDiagnostics` only | 0.199% | small updater cost |
| plain subscriber, 30 Hz odometry | 4.783% | matches prior minimal probe |
| `message_filters.Subscriber`, 30 Hz odometry | 4.584% | no penalty versus plain |
| Subscriber + Cache, 30 Hz odometry | 4.582% | no measurable Cache penalty |
| TF listener, 30 Hz TF | 4.381% | traffic-proportional, not 55% |
| empty node, 1 kHz clock present, sim time off | 0.000% | control; clock is ignored |
| no-op direct `/clock` subscriber, 1 kHz source | 57.133% | clock delivery reproduces jump |
| empty node with implicit sim time, 1 kHz source | 56.584% | production mechanism reproduced |
| Cache + 30 Hz odom + implicit 1 kHz clock | 60.770% | production-shaped high-CPU case |
| Cache + 30 Hz odom + implicit 100 Hz clock | 13.949% | proposed clock-rate mitigation |

The 1 kHz source delivered approximately 9,976 clock messages and 305 odometry
messages over the combined probe lifetime. The 100 Hz source delivered 804
clock messages and 241 odometry messages over its shorter lifetime. Counts
match the requested rates closely enough for attribution.

## Profile evidence

The requested attachment commands were:

```bash
/home/priestly/.local/bin/py-spy dump --pid PID
/home/priestly/.local/bin/py-spy record --pid PID --duration 5 \
  --rate 100 --format raw --output /tmp/ares_clock1000_pyspy.raw
timeout 5s strace -f -c -o /tmp/ares_clock1000_strace.txt -p PID
```

`sudo -n` reported `interactive authentication required`; same-user ptrace
worked without sudo. A one-shot dump landed in the native wait path:

```text
rclpy.spin
  spin_once
    _spin_once_impl
      wait_for_ready_callbacks
        _wait_for_ready_callbacks (executors.py:872)
```

The five-second, 388-sample raw profile additionally captured:

```text
rclpy.spin
  _wait_for_ready_callbacks
    event_handler.add_to_wait_set

rclpy.spin
  handler -> _execute -> await_or_execute
    time_source.clock_callback
      Time.from_msg

rclpy.spin
  handler -> _execute -> await_or_execute
    message_filters.callback -> signalMessage -> Cache.add
```

The Cache callback appeared as an occasional odometry path. Clock
deserialization/callbacks and repeated executor wait-set traversal occurred at
the much higher clock rate. The syscall summary was dominated by 14,977
`futex` calls (63.17%), 14 `recvfrom` calls (29.32% of aggregated blocked
thread time), and wait restarts; 29 middleware/runtime threads were attached.

## Recommended fix (proposal only)

1. Add a **Week 6-only** clock-rate boundary. Bridge Gazebo clock to a raw
   Week 6 topic, then relay the latest monotonically increasing sample to the
   shared `/clock` at 100 Hz. Keep Gazebo physics at its existing 1 ms step.
   Do not change `message_filters`, `time_sync.py`, or monitor callbacks.
2. Before adopting 100 Hz, run only a short non-campaign diagnostic launch and
   verify clock monotonicity, timer firing, fault onset timing, timeout
   behavior, odometry/header alignment, and Nav2 lifecycle behavior. Then
   repeat the process CPU measurements. Do not run `final_campaign_v2` or
   `final_campaign_v3` until this diagnostic gate is accepted.
3. Make Fast DDS explicit and auditable for Week 6 across the runner, outer
   launch, and included reliability processes. Preserve the existing defaults
   and behavior of Week 4/5. The supplied controller evidence supports Fast
   DDS as the required Week 6 RMW, but Fast DDS does not eliminate the Python
   clock fan-out cost.
4. On the next diagnostic launch, record `ros2 topic hz /clock`, each target
   PID's environment, and in-situ CPU. This is required because the live stack
   had stopped before this session could measure its actual clock rate.

Risks of clock decimation are coarser simulated-time timer resolution, delayed
timeout/fault transitions by up to one clock period, accidental split time
bases if only some nodes are remapped, and launch-order/latched-clock startup
issues. At 100 Hz the nominal resolution is 10 ms, below the monitors' 50 ms
minimum sync tolerance and 100 ms fastest timer, but that is a validation
argument, not permission to change or accept behavior.

A Week 6-only fix is feasible by isolating the raw clock bridge/relay and RMW
selection in Week 6 launch wiring. Week 4/5 compatibility risk is low only if
their frozen default launch/config paths remain behaviorally unchanged. No
such fix has been applied here.

## Required status

`WEEK6_STATUS: BLOCKED`

Blockers are: no in-situ confirmation from the stopped Week 6 stack, unresolved
checked-in RMW selection drift, and no approved/validated Week 6-only clock
rate fix. This report does not declare Week 6 PASS.
