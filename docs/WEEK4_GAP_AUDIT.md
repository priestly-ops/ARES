# ARES Week 4 Runtime-Evidence Gap Audit

Audit date: 2026-09-28

This audit distinguishes implementation from recorded runtime evidence. A
scenario is not marked passed merely because it exists in the matrix or has
unit tests. Runtime-tested means an exact matching `profile` exists in a Week 4
CSV. The healthy calibration sweep covers its labeled motion regimes, but it
does not create separate runtime evidence for every individually named healthy
scenario.

## Qualified baseline

- `healthy_calibration_sweep_20260928_131355.csv`: passed. No false DEGRADED or
  UNTRUSTED transitions, no gate activation, and no probation entry.
- `imu_bias_0p20_20260928_134041.csv`: passed the expected DEGRADED response and
  `LIKELY_IMU_FAULT` attribution. It did not reach UNTRUSTED, which is correct
  for its configured expected severity.
- `gnss_step_5m_20260928_065922.csv`: partial evidence only. Detection reached
  UNTRUSTED in about 0.7 s and attribution was correct, but no recovery or
  probation was recorded. It is not a complete recovery-scenario pass.

## Gap table

| Scenario | Implemented | Runtime tested | Passed | Evidence |
|---|---:|---:|---:|---|
| healthy_stationary | Yes | Yes | Yes | `healthy_stationary_20260928_065050.csv` |
| healthy_straight_slow | Yes | No | Not evaluated | Covered only as a regime inside the final calibration sweep |
| healthy_straight_fast | Yes | No | Not evaluated | Covered only as a regime inside the final calibration sweep |
| healthy_gentle_left | Yes | Yes | No | `healthy_gentle_left_20260928_065509.csv`; attribution was insufficient and one probation entry occurred |
| healthy_gentle_right | Yes | No | Not evaluated | Covered only as a regime inside the final calibration sweep |
| healthy_sharp_left | Yes | No | Not evaluated | Covered only as a regime inside the final calibration sweep |
| healthy_sharp_right | Yes | No | Not evaluated | Covered only as a regime inside the final calibration sweep |
| healthy_stop_start | Yes | No | Not evaluated | Covered only as a regime inside the final calibration sweep |
| healthy_accel_decel | Yes | No | Not evaluated | Covered only as a regime inside the final calibration sweep |
| healthy_figure8 | Yes | No | Not evaluated | Covered only as a regime inside the final calibration sweep |
| healthy_mixed | Yes | No | Not evaluated | No matching CSV |
| healthy_calibration_sweep | Yes | Yes | Yes | `healthy_calibration_sweep_20260928_131355.csv` |
| gnss_step_5m | Yes | Yes | No (partial) | `gnss_step_5m_20260928_065922.csv`; detection/attribution passed, recovery/probation absent |
| gnss_drift_0p02 | Yes | No | Not evaluated | No matching CSV |
| gnss_drift_0p05 | Yes | No | Not evaluated | No matching CSV |
| gnss_drift_0p10 | Yes | No | Not evaluated | No matching CSV |
| gnss_noise_x3 | Yes | No | Not evaluated | No matching CSV |
| gnss_freeze | Yes | No | Not evaluated | No matching CSV |
| gnss_freeze_timestamp | Yes | No | Not evaluated | No matching CSV |
| gnss_delay_50ms | Yes | No | Not evaluated | No matching CSV |
| gnss_delay_100ms | Yes | No | Not evaluated | No matching CSV |
| gnss_delay_250ms | Yes | No | Not evaluated | No matching CSV |
| gnss_timestamp_jitter | Yes | No | Not evaluated | No matching CSV |
| gnss_out_of_order | Yes | No | Not evaluated | No matching CSV |
| gnss_spike | Yes | No | Not evaluated | No matching CSV |
| gnss_plausible_spoof | Yes | No | Not evaluated | No matching CSV |
| imu_bias_0p05 | Yes | No | Not evaluated | No matching CSV |
| imu_bias_0p10 | Yes | No | Not evaluated | No matching CSV |
| imu_bias_0p20 | Yes | Yes | Yes | `imu_bias_0p20_20260928_134041.csv` |
| imu_drift | Yes | No | Not evaluated | No matching CSV |
| imu_dropout | Yes | No | Not evaluated | No matching CSV |
| imu_freeze | Yes | No | Not evaluated | No matching CSV |
| imu_spike | Yes | No | Not evaluated | No matching CSV |
| wheel_scale_1p05 | Yes | No | Not evaluated | No matching CSV |
| wheel_scale_1p10 | Yes | No | Not evaluated | No matching CSV |
| wheel_yaw_bias | Yes | No | Not evaluated | No matching CSV |
| wheel_slip | Yes | No | Not evaluated | No matching CSV |
| gnss_plus_wheel_fault | Yes | No | Not evaluated | No matching CSV |
| gnss_dropout_plus_imu_bias | Yes | No | Not evaluated | No matching CSV |
| gnss_fault_sensor_unavailable | Yes | No | Not evaluated | No matching CSV |
| repeated_gnss_cycles | Yes | No | Not evaluated | No matching CSV |
| preinit_gnss_5m | Yes | No | Not evaluated | No matching CSV |
| preinit_gnss_slow_offset | Yes | No | Not evaluated | No matching CSV |
| preinit_odom_bias | Yes | No | Not evaluated | No matching CSV |
| localization_covariance_degradation | No | No | Not evaluated | Matrix marks this manual and blocked because no operational/raw AMCL split exists |

## Truthful Week 4 status

Week 4 passes only the core healthy calibration and the 0.20 rad/s IMU-bias
attribution case. The recorded +5 m GNSS step supports detection and attribution
claims, but not recovery. Slow GNSS drift, GNSS noise, freeze, timing faults,
other IMU cases, wheel faults, combined faults, pre-initialization faults,
repeated probation, and ambiguous attribution remain untested at runtime.
