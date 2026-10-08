#!/usr/bin/env python3
"""Consolidate the controlled WSL GPU-LiDAR shutdown-order experiments."""

import csv
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results/week1/wsl_shutdown_order_20260924"


def topic_state(summary):
    for event in summary.get("timeline", []):
        if event.get("event") == "gz_topic_info_after_consumer_removal":
            return event.get("stdout", "").strip()
    return "not measured"


rows = []
for summary_path in sorted(RESULTS.glob("*/summary.json")):
    summary = json.loads(summary_path.read_text())
    codes = summary.get("exit_codes", {})
    rows.append(
        {
            "name": summary.get("name", summary_path.parent.name),
            "sequence": summary.get("sequence"),
            "valid": bool(summary.get("scan_received"))
            and not summary.get("forced_cleanup")
            and not summary.get("stale_processes_after"),
            "lazy_bridge": bool(summary.get("lazy_bridge", False)),
            "subscriber_driven_lidar": bool(
                summary.get("subscriber_driven_lidar", False)
            ),
            "gazebo_exit": codes.get("gazebo"),
            "bridge_exit": codes.get("bridge"),
            "subscriber_exit": codes.get("scan_subscriber"),
            "scan_received": bool(summary.get("scan_received")),
            "clean_shutdown": bool(summary.get("clean_shutdown")),
            "crash": bool(summary.get("crash")),
            "forced_cleanup": summary.get("forced_cleanup", []),
            "stale_processes_after": summary.get("stale_processes_after", []),
            "native_topic_after_consumer_removal": topic_state(summary),
            "result_dir": str(summary_path.parent.relative_to(ROOT)),
        }
    )

matrix = {
    "schema_version": 1,
    "experiment": "ARES 5 Hz GPU-LiDAR shutdown ordering and lifecycle",
    "sensor_fidelity_changed": False,
    "rows": rows,
    "findings": {
        "proven": [
            "All six valid A-F shutdown-order cases ended with Gazebo exit -11.",
            "Stopping the ROS scan consumer before the bridge allowed the bridge to exit 0.",
            "Stopping the bridge while the ROS scan consumer remained active reproduced bridge exit -11.",
            "SIGTERM did not prevent the Gazebo crash and did not produce a clean subscriber exit.",
            "The lazy-bridge/subscriber-driven-lidar case had no native scan subscribers before shutdown, yet Gazebo still exited -11.",
        ],
        "likely": [
            "The Gazebo teardown fault is below the ROS scan-consumer lifetime and is consistent with the D3D12 rendering teardown path identified in the core dump.",
            "Bridge teardown reliability depends on removing its ROS-side scan consumer before stopping the bridge.",
        ],
        "unknown": [
            "The exact symbol at the original libd3d12core.so fault address because the WSL D3D12 library is stripped and matching debug symbols are unavailable.",
        ],
    },
}

(RESULTS / "shutdown_matrix.json").write_text(json.dumps(matrix, indent=2) + "\n")

columns = [
    "name",
    "sequence",
    "valid",
    "lazy_bridge",
    "subscriber_driven_lidar",
    "gazebo_exit",
    "bridge_exit",
    "subscriber_exit",
    "scan_received",
    "clean_shutdown",
    "crash",
    "result_dir",
]
with (RESULTS / "shutdown_matrix.csv").open("w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
    writer.writeheader()
    writer.writerows(rows)

valid_rows = [row for row in rows if row["valid"]]
table = [
    "| Case | Sequence | Lazy | Subscriber-driven | Gazebo | Bridge | Subscriber | Clean |",
    "|---|---:|---:|---:|---:|---:|---:|---:|",
]
for row in valid_rows:
    table.append(
        "| {name} | {sequence} | {lazy_bridge} | {subscriber_driven_lidar} | "
        "{gazebo_exit} | {bridge_exit} | {subscriber_exit} | {clean_shutdown} |".format(
            **row
        )
    )

report = """# WSL GPU-LiDAR Shutdown and Lifecycle Matrix

The exact 5 Hz / 360-sample / 360-degree ARES GPU-LiDAR workload was retained in every valid case. These are teardown diagnostics, not Week 1 acceptance runs.

## Results

{table}

## PROVEN

- All six requested shutdown orders reproduced Gazebo `SIGSEGV` / exit `-11`.
- Cases A, B, D, and E show that removing the ROS subscriber before stopping the bridge gives the bridge a clean exit. Case C, where the bridge stopped while the subscriber remained active, reproduced bridge exit `-11`.
- SIGTERM did not solve the Gazebo crash; it also left the `ros2 topic hz` process with exit `1`.
- In the lifecycle case, the bridge was lazy and the LiDAR was subscriber-driven. After the ROS consumer stopped, `gz topic -i -t /ares/scan` reported **no subscribers**. Gazebo still exited `-11` after five seconds of sensor settling, a clean bridge exit, and another three seconds of settling.
- No valid case required forced cleanup or left stale Gazebo / bridge processes.

## LIKELY

- The Gazebo crash is independent of an active ROS or native scan consumer. Combined with the captured core, it is most consistent with teardown of the WSL D3D12 rendering resource path.
- ROS subscriber-first ordering is still the correct bridge shutdown policy because it eliminates the bridge-specific crash.

## UNKNOWN

- The precise function at the original fault address in `libd3d12core.so`; the installed WSL library is stripped and no matching debug symbols were available.

## Acceptance impact

No case is a clean preflight because Gazebo exited `-11`. The five-consecutive-preflight gate remains closed, so the 20-run suite must not start.
""".format(table="\n".join(table))
(RESULTS / "REPORT.md").write_text(report)

print(json.dumps({"valid_cases": len(valid_rows), "clean_cases": sum(row["clean_shutdown"] for row in valid_rows), "output": str(RESULTS)}, indent=2))
