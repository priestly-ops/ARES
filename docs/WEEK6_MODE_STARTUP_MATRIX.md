# Week 6 mode startup matrix

## Finding

Baseline, unprotected, and protected navigation all require BT Navigator, but
they do not have identical upstream lifecycle topology. Baseline owns AMCL and
therefore requires AMCL to be active; unprotected and protected use the
experimental trust-fused localization path and do not launch AMCL.

The original mini-campaign baseline failure was not an intentional topology
difference. `nav2_week6_upstream` launched the planner, controller, and behavior
servers but omitted the `bt_navigator` process. The barrier therefore found all
ten action/service endpoints, yet `/bt_navigator/get_state` remained absent.
Baseline is supposed to launch BT Navigator, so this was launch configuration
case E, not cases A-D.

## Launch topology

| Property | Baseline | Unprotected | Protected |
|---|---|---|---|
| Nav2 nodes launched | map server, AMCL, planner, controller, behavior, BT Navigator | map server, planner, controller, behavior, BT Navigator | map server, planner, controller, behavior, BT Navigator |
| Upstream lifecycle manager | `lifecycle_manager_week6_upstream` from the baseline include | explicit `lifecycle_manager_week6_upstream` | explicit `lifecycle_manager_week6_upstream` |
| Upstream managed nodes | map, AMCL, planner, controller, behavior | map, planner, controller, behavior | map, planner, controller, behavior |
| Separate BT manager | `lifecycle_manager_week6_bt` | `lifecycle_manager_week6_bt` | `lifecycle_manager_week6_bt` |
| BT Navigator launched | YES | YES | YES |
| `/bt_navigator/get_state` expected | YES | YES | YES |
| Planner lifecycle service | YES | YES | YES |
| Controller lifecycle service | YES | YES | YES |
| Behavior lifecycle service | YES | YES | YES |
| Readiness barrier | YES | YES | YES |
| BT activation | Barrier exit 0 starts the separate BT manager | Same | Same |
| Navigation odometry | `/odometry/filtered` | `/odometry/trust_fused` | `/odometry/trust_fused` |

The baseline upstream manager deliberately excludes BT Navigator from its
managed-node list. The BT process exists in an unconfigured state while the
barrier checks lifecycle and endpoint readiness; only the separate BT manager
configures and activates it after the barrier passes.

## Mode-specific readiness manifests

The barrier now receives `--navigation-mode` and records the selected manifest
in its readiness JSON.

| Mode | Required ACTIVE lifecycle nodes |
|---|---|
| baseline | map server, planner, controller, behavior, AMCL |
| unprotected | map server, planner, controller, behavior |
| protected | map server, planner, controller, behavior |

All three modes use the same frozen Week 6 Nav2 parameters and the same
NavigateToPose/NavigateThroughPoses BTs, so their required endpoint sets are
legitimately identical:

| Kind | Endpoint | Owner / justification |
|---|---|---|
| Action | `/compute_path_to_pose` | planner server; NavigateToPose planning |
| Action | `/compute_path_through_poses` | planner server; NavigateThroughPoses planning |
| Action | `/follow_path` | controller server; path execution |
| Action | `/spin` | behavior server; recovery BT behavior |
| Action | `/wait` | behavior server; recovery BT behavior |
| Action | `/backup` | behavior server; recovery BT behavior |
| Action | `/drive_on_heading` | configured frozen behavior-server action retained by the frozen ten-endpoint Week 6 barrier |
| Service | `/is_path_valid` | planner server; BT path-validity condition |
| Service | `/global_costmap/clear_entirely_global_costmap` | planner/global costmap recovery |
| Service | `/local_costmap/clear_entirely_local_costmap` | controller/local costmap recovery |

No unrelated clear-around-pose or clear-around-robot services are checked.
Mode specificity applies to lifecycle ownership, not by weakening the common
BT endpoint coverage.

## Controlled startup validation

Artifacts are under
`results/week6/post_v5_runtime_diagnostics/mode_startup_matrix/`.

| Mode | Result | Barrier elapsed (wall s) | Pre-BT lifecycle state | Endpoint failures | BT failures |
|---|:---:|---:|---|---:|---:|
| baseline | PASS | 5.354 | upstream + AMCL active; BT unconfigured | 0 | 0 |
| unprotected | PASS | 1.546 | upstream active; BT unconfigured | 0 | 0 |
| protected | PASS | 1.245 | upstream active; BT unconfigured | 0 | 0 |

Each launch then activated BT Navigator, established its lifecycle bond, and
created both configured BTs. No failed case was rerun.

**MODE_STARTUP_MATRIX_PASS: 3/3**  
**MODE_STARTUP_ENDPOINT_FAILURES: 0**  
**MODE_STARTUP_BT_FAILURES: 0**
