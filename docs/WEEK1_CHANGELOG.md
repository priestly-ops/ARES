# Week 1 change log

* Qualified an llvmpipe plus ordered-bridge-shutdown WSL configuration with
  three consecutive clean health preflights, then ran the complete gated
  20-trial mission suite on 2026-09-24/25. The suite failed 9/20: 12/20 met
  RTF, 14/20 shut down cleanly, and six `parameter_bridge` processes exited
  `-11`. Added a retained acceptance report and multi-run aggregation support
  to the diagnostic wrapper; Week 1 remains blocked.

* Completed read-only audit and host process/graphics inspection; archived original ARES inputs before edits.
* Added fresh-process performance probe, isolated Gazebo partitions/ROS domains, raw statistics and integrated RTF calculations. Corrected JSON camelCase parser and recovered first result from retained raw samples.
* Reproduced slowdown with full scan bridge, isolated actor rendering with camera removed, compared software rendering. No sensor-rate or physics-step reductions.
* Follow-up isolation on the repaired 6.6 WSL kernel showed the remaining bottleneck was the subscribed 10 Hz GPU LiDAR itself: native Gazebo scan subscription reproduced approximately 0.415 RTF without the ROS bridge. An equivalent CPU LiDAR type did not instantiate in the installed runtime and was reverted. The documented Week 1 simulation profile now uses an intentional 5 Hz GPU LiDAR rate with 360 samples, full FOV, range, resolution, and noise unchanged; native/full-sensor probes reached approximately 0.800/0.744 RTF.
* Added baseline launch deriving a static world from existing warehouse, optional camera and RViz, explicit stage selection, one lifecycle manager, process-exit shutdown propagation and temporary-file cleanup.
* Added production bridge with clock/scan/odom/IMU/command and separate optional camera bridge. Original experimental bridges preserved.
* Added intentional EKF baseline (wheel x/y velocity plus IMU yaw rate; no correlated wheel pose or absolute world IMU yaw). Existing EKF config retained; corrected legacy launch output topic.
* Corrected URDF vertical geometry and camera extrinsic to agree with simulation. Existing fixed wheel visualization retained.
* Corrected LiDAR noise schema using installed SDFormat specification. Removed duplicate local-costmap YAML keys without changing values.
* Installed map resources and runtime dependencies. Added benchmark package with bounded waits, lifecycle checks, sensor validation, TF checks, metrics and configurable mission.
* Added fresh-launch trials with bag recording, source hashes, independent logs, process cleanup and CSV results. Added output ignores.
* Corrected final trial accounting so child-process crashes discovered during teardown update `success`, the CSV summary, printed status, and the runner's final exit code.
* Added instrumented A-B-C-D full-stack preflights and an independent shutdown reproducer. The matrix passed the runtime RTF/health gate but isolated a repeatable Gazebo GPU-LiDAR teardown SIGSEGV under WSLg; the equivalent generated world without LiDAR exits cleanly.
* Attempted the requested native-Ubuntu final acceptance on 2026-09-23. The mandatory environment gate identified kernel `6.18.33.2-microsoft-standard-WSL2`, so the session was still WSL2. Stopped before sourcing, build, preflight, or acceptance launches and retained the blocker evidence under `results/week1/native_final_acceptance_attempt_20260923_2200/`.
