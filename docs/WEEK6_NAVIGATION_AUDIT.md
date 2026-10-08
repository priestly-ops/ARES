# ARES Week 6 Navigation Architecture Audit

Date: 2026-09-30

This audit records the navigation architecture before any Week 6 runtime
changes. Week 5 trust thresholds, attribution, recovery policy, estimator
state selection, and fault profiles remain frozen.

## Production baseline

The production entry point is
`src/ares_simulation/launch/ares_baseline.launch.py`. Its default `nav2` stage
starts Gazebo, `ros_gz_bridge`, `robot_state_publisher`, the production
`robot_localization` EKF, map server, AMCL, planner server, controller server,
behavior server, BT navigator, the command adapter, and one lifecycle manager.
It loads the static warehouse map from
`src/ares_simulation/maps/ares_warehouse.yaml` (or the launch `map` argument).
SLAM Toolbox has configuration and a standalone launch file in the workspace,
but it is not part of the production navigation launch.

The production localization chain is:

```text
map --AMCL--> odom --production EKF--> base_footprint
```

The production EKF consumes planar velocity from `/ares/odom` and yaw rate
from `/ares/imu`, publishes `/odometry/filtered`, and is the sole dynamic
`odom -> base_footprint` authority. AMCL consumes `/ares/scan`, publishes
`map -> odom`, and provides `/amcl_pose`. Both the controller server and BT
navigator are configured with `odom_topic: /odometry/filtered`.

## Nav2 configuration

`src/ares_localization/config/nav2_params.yaml` defines the current stack:

| Item | Current value |
|---|---|
| Localization | AMCL on a static map |
| Planner | `nav2_navfn_planner::NavfnPlanner`, A* enabled |
| Controller | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| Controller rate | 10 Hz |
| Behavior server | Nav2 behavior server |
| Loaded behaviors | spin, backup, drive-on-heading, wait (confirmed in runtime logs) |
| Velocity smoother | none |
| Goal checker | `SimpleGoalChecker`, 0.20 m XY / 0.30 rad yaw, stateful |
| Progress checker | `SimpleProgressChecker`, 0.20 m in 10.0 s |
| Global costmap | static + scan obstacle + inflation layers |
| Local costmap | rolling scan obstacle + inflation layers |
| Observation source | `/ares/scan` (`LaserScan`) |
| Robot radius | 0.30 m |
| Inflation radius | 0.70 m |

The default BT navigator provides both NavigateToPose and
NavigateThroughPoses action navigators. No separate Nav2 waypoint-follower
server is launched. Two sequential-goal clients exist:

- `src/ares_benchmark/scripts/healthy_mission.py`, the bounded Week 1
  acceptance mission and health monitor.
- `src/ares_localization/ares_localization/waypoint_mission.py`, an older
  five-waypoint client with CSV timing but unbounded lifecycle.

There was no Week 6 navigation runner, recorder, result tree, or A/B analyzer
at the time of this audit.

## Command and recovery path

Nav2 publishes stamped `/cmd_vel`. `cmd_vel_adapter.py` converts it to the
unstamped `/ares/cmd_vel` consumed by the Gazebo differential-drive plugin.
No velocity-smoother node is present. Nav2 recovery behavior is provided by
the behavior server and the default BT; there is no ARES-specific planner or
controller modification.

## Experimental Week 5 estimator

`src/ares_reliability/launch/week5_trust_fusion.launch.py` starts the frozen
reliability layer, `navsat_transform_node`, and a parallel EKF. The parallel
EKF publishes `/odometry/trust_fused` with:

- `header.frame_id: odom` and `child_frame_id: base_footprint`;
- planar position and orientation estimate;
- planar twist;
- pose and twist covariance;
- ROS-native timestamps at a configured 30 Hz.

Its source selection is wheel planar velocity, IMU yaw rate, and converted
GNSS planar position. In `unprotected` mode the converter consumes
`/ares/gps`; in `protected` mode it consumes `/ares/gps_trusted` after the
frozen ARES policy.

The experimental EKF deliberately has `publish_tf: false`. Consequently,
`/odometry/trust_fused` has the message fields Nav2 needs for velocity
feedback, but it cannot by itself provide the pose transform Nav2 uses for
planning and control. Remapping only `odom_topic` would leave Nav2 following
AMCL and the production EKF, invalidating a protected/unprotected localization
comparison.

## Week 6 isolation requirement

Mode A must continue to use the production launch unchanged. Modes B and C
must use an explicit experimental launch which:

1. starts the same world, bridge, robot description, static map, Nav2 planner,
   controller, behavior server, BT, motion limits, and command adapter;
2. does not start AMCL or the production EKF;
3. makes the experimental EKF the only `odom -> base_footprint` authority;
4. supplies one explicit identity `map -> odom` transform because the fixed
   spawn, map initial pose, and local ENU estimator origin are aligned;
5. points controller and BT odometry feedback to `/odometry/trust_fused`;
6. differs between Modes B and C only in the frozen Week 5 fusion mode
   (`unprotected` versus `protected`).

This preserves `/odometry/filtered` and the production TF chain. Experimental
results must be rejected if duplicate TF authorities, missing transforms,
timestamp regressions, estimator restarts, or lifecycle failures are observed.

## Measurement gaps to close in Week 6

The Week 1 mission records filtered path length, maximum plan length, command
magnitudes, lifecycle state, and TF connectivity, but it does not provide the
Week 6 mission schema. Week 6 still needs a recorder and deterministic metric
functions for ground-truth trajectory, localization error, cross-track error,
command gaps, fault/recovery transitions, per-waypoint outcomes, Nav2 errors,
and matched A/B comparison.

Gazebo's `/ares/odom` is generated by the differential-drive system and is
independent of the experimental GNSS fusion, but it is wheel-model odometry,
not a separately bridged physics pose. It may be used as the initial
deterministic reference while the recorder labels that limitation explicitly;
an independent Gazebo model-pose bridge is preferred before making final
ground-truth localization claims.

## Audit decision

The production navigation baseline is internally complete and previously
qualified. Week 6 implementation is safe only through an opt-in experimental
launch; the frozen production launch, `/odometry/filtered`, Week 5 thresholds,
recovery policy, and fault profiles must remain unchanged.
