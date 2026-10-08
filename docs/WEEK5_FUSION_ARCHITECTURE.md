# ARES Week 5 Experimental Trust-Fusion Architecture

The Week 5 estimator is parallel to the qualified production estimator. The
production `/odometry/filtered` topic and its `odom -> base_footprint` transform
remain untouched.

```text
/ares/gps_raw -> GNSS fault injector -> /ares/gps
                                          |
                         +----------------+----------------+
                         |                                 |
                    unprotected                       ARES proxy
                         |                                 |
                         |                         /ares/gps_trusted
                         +----------------+----------------+
                                          |
                              navsat_trust_transform
                                          |
                              /odometry/gps_trusted
                                          |
/ares/odom_operational -------------------+--- ekf_trust_fusion_node
/ares/imu_operational --------------------+             |
                                                   /odometry/trust_fused
```

The launch supports three controlled modes:

- `odom_imu_only`: wheel forward velocity and IMU yaw rate; no GNSS converter.
- `unprotected`: `/ares/gps` is fused without ARES recovery action.
- `protected`: `/ares/gps_trusted` is fused after covariance inflation, gating,
  and probation decisions.

State contributions are intentionally narrow:

| Source | Fused state dimensions |
|---|---|
| `/ares/odom_operational` | planar body velocity `vx`, `vy` (including the nonholonomic zero-`vy` constraint) |
| `/ares/imu_operational` | yaw rate `vyaw` |
| `/odometry/gps_trusted` | planar position `x`, `y` |

The experimental EKF does not publish TF. This prevents duplicate production
transforms while still exposing a complete `nav_msgs/Odometry` estimate for
measurement and later opt-in navigation experiments.
