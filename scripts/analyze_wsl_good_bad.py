#!/usr/bin/env python3
"""Build the ARES Week 1 known-good / bad WSL evidence comparison."""

import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics


root = Path(__file__).resolve().parents[1]
output = root / "results/week1/wsl_good_bad_comparison_20260923"
output.mkdir(parents=True, exist_ok=True)

good_root = root / "results/week1/week1_5hz_acceptance_fresh_domains_20260923_154636"
bad_paths = [
    root / "results/week1/resume_5hz_health_preflight_20260923_2127/run_20260923_211651_001",
    root / "results/week1/resume_5hz_health_affinity_20260923_2125/run_20260923_212127_001",
    root / "results/week1/resume_5hz_health_affinity1_20260923_2123/run_20260923_212227_001",
]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    return json.loads(path.read_text())


def launch_facts(run):
    text = (run / "launch.log").read_text(errors="replace")
    started = [
        {"ordinal": i + 1, "process": match.group(1), "pid": int(match.group(2))}
        for i, match in enumerate(
            re.finditer(r"\[INFO\] \[([^]]+)\]: process started with pid \[(\d+)\]", text)
        )
    ]
    died = [
        {"process": match.group(1), "pid": int(match.group(2)), "exit_code": int(match.group(3))}
        for match in re.finditer(
            r"\[ERROR\] \[([^]]+)\]: process has died \[pid (\d+), exit code (-?\d+),", text
        )
    ]
    return {
        "startup_order": started,
        "launch_child_count": len(started),
        "unexpected_child_exits": died,
        "server_only": "cmd 'gz sim -s -r --seed 42" in text,
        "sigint_shutdown": "user interrupted with ctrl-c (SIGINT)" in text,
        "scan_subscriptions_in_log": len(re.findall(r"Subscribed to Topics: scan", text)),
    }


def sensor_rates(metrics):
    return {
        topic: details.get("rate_sim_hz")
        for topic, details in metrics.get("sensors", {}).items()
        if isinstance(details, dict)
    }


def entry(run, category, scheduling, workload):
    metrics = read_json(run / "metrics.json")
    manifest = read_json(run / "manifest.json")
    launch = launch_facts(run)
    return {
        "category": category,
        "run_id": run.name,
        "path": str(run.relative_to(root)),
        "timestamp_local_from_directory": run.name.removeprefix("run_").rsplit("_", 1)[0],
        "runtime_success_recorded": metrics.get("success"),
        "mission_waypoint_count": len(metrics.get("waypoints", [])),
        "mission_waypoints_successful": sum(
            bool(item.get("success")) for item in metrics.get("waypoints", [])
        ),
        "effective_rtf": metrics.get("effective_rtf"),
        "duration_wall_s": metrics.get("duration_wall_s"),
        "duration_sim_s": metrics.get("duration_sim_s"),
        "sensor_rates_sim_hz": sensor_rates(metrics),
        "ros_domain_id": metrics.get("ros_domain_id"),
        "scheduling": scheduling,
        "workload": workload,
        "bag_valid": metrics.get("bag_valid"),
        "no_unexpected_process_exit_before_shutdown": metrics.get(
            "no_unexpected_process_exit"
        ),
        "launch": launch,
        "source_hashes": {
            key: manifest.get(key)
            for key in (
                "src/ares_simulation/worlds/ares_test_world.sdf",
                "src/ares_simulation/models/ares_jackal/model.sdf",
                "src/ares_simulation/config/bridge_baseline.yaml",
                "src/ares_simulation/launch/ares_baseline.launch.py",
                "src/ares_benchmark/config/mission.yaml",
            )
        },
        "manifest_file_count": len(manifest),
        "manifest_sha256": sha256(run / "manifest.json"),
    }


good_runs = []
for run in sorted(good_root.glob("run_*")):
    metrics_path = run / "metrics.json"
    if not metrics_path.exists() or metrics_path.stat().st_size == 0:
        continue
    metrics = read_json(metrics_path)
    if (
        metrics.get("success")
        and len(metrics.get("waypoints", [])) == 3
        and all(item.get("success") for item in metrics["waypoints"])
        and metrics.get("effective_rtf", 0) >= 0.5
    ):
        good_runs.append(entry(run, "known_good_runtime_mission", "default 0-11", "mission+bag"))

bad_runs = [
    entry(bad_paths[0], "known_bad_rtf", "default 0-11", "stationary health-only+bag"),
    entry(bad_paths[1], "known_bad_rtf", "agent 0-1; trial 2-11", "stationary health-only+bag"),
    entry(bad_paths[2], "known_bad_rtf", "agent 0; trial 1-11", "stationary health-only+bag"),
]

if len(good_runs) != 11:
    raise RuntimeError(f"Expected 11 known-good mission runs, found {len(good_runs)}")

good_rtfs = [item["effective_rtf"] for item in good_runs]
indices = list(range(1, len(good_rtfs) + 1))
mean_index = statistics.mean(indices)
mean_rtf = statistics.mean(good_rtfs)
pearson_numerator = sum(
    (index - mean_index) * (rtf - mean_rtf)
    for index, rtf in zip(indices, good_rtfs)
)
pearson_denominator = math.sqrt(
    sum((index - mean_index) ** 2 for index in indices)
    * sum((rtf - mean_rtf) ** 2 for rtf in good_rtfs)
)

common_manifest = good_runs[0]["source_hashes"]
all_source_hashes_equal = all(
    item["source_hashes"] == common_manifest for item in good_runs + bad_runs
)
startup_names = [item["process"] for item in good_runs[0]["launch"]["startup_order"]]
all_startup_orders_equal = all(
    [child["process"] for child in item["launch"]["startup_order"]] == startup_names
    for item in good_runs + bad_runs
)

current_files = {
    str(path.relative_to(root)): sha256(path)
    for path in (
        root / "scripts/run_healthy_trials.py",
        root / "scripts/run_healthy_trials.sh",
        root / "src/ares_simulation/worlds/ares_test_world.sdf",
        root / "src/ares_simulation/models/ares_jackal/model.sdf",
        root / "src/ares_simulation/config/bridge_baseline.yaml",
        root / "src/ares_simulation/launch/ares_baseline.launch.py",
        root / "src/ares_benchmark/config/mission.yaml",
    )
}

comparison = {
    "schema_version": 1,
    "generated_for": "ARES Week 1 WSL good/bad comparison 2026-09-23",
    "classification_definitions": {
        "PROVEN": "directly present in retained per-run or host artifacts",
        "LIKELY": "supported by common runner/report evidence but not retained per-run",
        "UNKNOWN": "not recoverable from retained evidence",
    },
    "known_good_selection_rule": (
        "metrics success true, 3/3 waypoint success, effective_rtf >= 0.500, "
        "under the fresh-domains 5 Hz batch"
    ),
    "known_good": good_runs,
    "known_bad": bad_runs,
    "statistics": {
        "good_count": len(good_runs),
        "good_rtf_min": min(good_rtfs),
        "good_rtf_mean": statistics.mean(good_rtfs),
        "good_rtf_median": statistics.median(good_rtfs),
        "good_rtf_max": max(good_rtfs),
        "good_rtf_population_stddev": statistics.pstdev(good_rtfs),
        "good_run_index_vs_rtf_pearson": pearson_numerator / pearson_denominator,
        "bad_rtfs": [item["effective_rtf"] for item in bad_runs],
    },
    "common_facts": {
        "source_hashes_equal_good_and_bad": all_source_hashes_equal,
        "startup_order_equal_good_and_bad": all_startup_orders_equal,
        "source_hashes": common_manifest,
        "startup_order": startup_names,
        "launch_command": "ros2 launch ares_simulation ares_baseline.launch.py rviz:=false camera:=false",
        "gazebo_command": "gz sim -s -r --seed 42 <generated-world.sdf>",
        "shutdown": "Nav2 lifecycle command 4, then launch SIGINT",
        "ros_scan_subscriber": True,
        "native_gz_scan_subscriber": False,
        "gui_running": False,
        "camera_enabled": False,
        "gazebo_version": "10.5.0",
        "ros_distribution": "Lyrical",
        "historical_kernel": "6.6.123.2-microsoft-standard-WSL2+",
        "historical_renderer": "D3D12 Intel Iris Xe, Mesa 26.0.8",
    },
    "evidence_assessment": [
        {
            "classification": "PROVEN",
            "finding": "All 11 selected runs completed 3/3 mission goals, recorded valid bags, and had RTF >= 0.576709.",
            "sources": ["per-run metrics.json", "baseline_summary.csv"],
        },
        {
            "classification": "PROVEN",
            "finding": "Every good and bad run has the same retained ARES world/model/bridge/launch/mission hashes.",
            "sources": ["per-run manifest.json"],
        },
        {
            "classification": "PROVEN",
            "finding": "Every good and bad run used the same 12-child launch startup order, server-only Gazebo (-s), RViz false, and camera false.",
            "sources": ["launch.log", "ares_baseline.launch.py", "runner command"],
        },
        {
            "classification": "PROVEN",
            "finding": "The 11 good-run RTF values decline with run index (Pearson r about -0.870), despite fresh Gazebo/ROS processes per run.",
            "sources": ["per-run metrics.json"],
        },
        {
            "classification": "PROVEN",
            "finding": "The normal 0.315 bad run had the same default 0-11 affinity class as the good runs; the 0.494 and 0.475 bad runs used explicit partitions.",
            "sources": ["completion report", "retained result names and commands"],
        },
        {
            "classification": "PROVEN",
            "finding": "All 11 runtime-good launches ended with Gazebo -11 and parameter_bridge -11 under the old accounting; none is a clean shutdown pass.",
            "sources": ["per-run launch.log"],
        },
        {
            "classification": "PROVEN",
            "finding": "The good runs executed the moving three-goal mission; the three low-RFT comparisons were stationary health-only preflights.",
            "sources": ["metrics.json", "mission.log"],
        },
        {
            "classification": "LIKELY",
            "finding": "Host scheduling load is a material contributor: retained nearby snapshots show much higher proxy CPU in the slow state, while later same-boot A-D trials recovered 0.767-0.887 without ARES source changes.",
            "sources": ["diagnostics samples", "completion report", "diagnostics summaries"],
        },
        {
            "classification": "UNKNOWN",
            "finding": "Exact per-run inherited graphics environment, per-core frequencies, thermals, and Windows host load were not retained for the 11 good runs.",
            "sources": [],
        },
        {
            "classification": "UNKNOWN",
            "finding": "The exact historical runner hash and workspace Git hash are unavailable: the workspace root was not a Git repository and the runner was edited after the good batch.",
            "sources": ["runner mtime", "completion report"],
        },
    ],
    "current_file_hashes": current_files,
    "runner_version_note": {
        "historical_good_runner_hash": None,
        "historical_bad_runner_hash": None,
        "current_runner_hash": current_files["scripts/run_healthy_trials.py"],
        "workspace_git_hash": None,
        "reason": "workspace root has no Git metadata; runner was modified after both batches",
    },
    "environment_evidence": {
        "historical_per_run_environment": "UNKNOWN except variables assigned by runner",
        "assigned_by_runner": ["ROS_DOMAIN_ID", "GZ_PARTITION", "ROS_LOG_DIR"],
        "inherited_likely": {
            "DISPLAY": ":0",
            "WAYLAND_DISPLAY": "wayland-0",
            "GALLIUM_DRIVER": "d3d12",
            "ROS_DISTRO": "lyrical",
        },
        "wsl_settings": {
            "historical_wslconfig": "kernel=C:\\\\wsl-kernels\\\\bzImage-wsl-6.6",
            "logical_cpus": 12,
            "memory_visible_gib": 7.7,
            "swap_visible_gib": 2.0,
            "classification": "LIKELY for good batch; PROVEN for retained later same-day host snapshot",
        },
    },
}

(output / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")

with (output / "runs.csv").open("w", newline="") as stream:
    fields = [
        "category",
        "run_id",
        "timestamp_local_from_directory",
        "effective_rtf",
        "duration_wall_s",
        "duration_sim_s",
        "workload",
        "scheduling",
        "mission_waypoint_count",
        "mission_waypoints_successful",
        "bag_valid",
        "server_only",
        "gui_running",
        "camera_enabled",
        "launch_child_count",
        "gazebo_exit",
        "parameter_bridge_exit",
    ]
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader()
    for item in good_runs + bad_runs:
        exits = {
            entry["process"].split("-", 1)[0]: entry["exit_code"]
            for entry in item["launch"]["unexpected_child_exits"]
        }
        writer.writerow(
            {
                "category": item["category"],
                "run_id": item["run_id"],
                "timestamp_local_from_directory": item[
                    "timestamp_local_from_directory"
                ],
                "effective_rtf": item["effective_rtf"],
                "duration_wall_s": item["duration_wall_s"],
                "duration_sim_s": item["duration_sim_s"],
                "workload": item["workload"],
                "scheduling": item["scheduling"],
                "mission_waypoint_count": item["mission_waypoint_count"],
                "mission_waypoints_successful": item[
                    "mission_waypoints_successful"
                ],
                "bag_valid": item["bag_valid"],
                "server_only": item["launch"]["server_only"],
                "gui_running": False,
                "camera_enabled": False,
                "launch_child_count": item["launch"]["launch_child_count"],
                "gazebo_exit": exits.get("gz"),
                "parameter_bridge_exit": exits.get("parameter_bridge"),
            }
        )

good_table = "\n".join(
    f"| {i} | `{item['run_id']}` | {item['effective_rtf']:.6f} | "
    f"{item['duration_sim_s']:.3f} | {item['duration_wall_s']:.3f} | "
    f"{item['sensor_rates_sim_hz'].get('/ares/scan', float('nan')):.3f} | -11 | -11 |"
    for i, item in enumerate(good_runs, 1)
)
bad_table = "\n".join(
    f"| `{item['run_id']}` | {item['scheduling']} | {item['effective_rtf']:.6f} | "
    f"{item['sensor_rates_sim_hz'].get('/ares/scan', float('nan')):.3f} | "
    f"{', '.join(x['process'] + '=' + str(x['exit_code']) for x in item['launch']['unexpected_child_exits'])} |"
    for item in bad_runs
)

report = f"""# ARES Week 1 WSL known-good / bad comparison

Generated: 2026-09-23. This report separates runtime mission success from clean
Week 1 acceptance. The 11 historical runs passed the mission and RTF checks but
did **not** exit cleanly.

## Identified 11 known-good runtime mission runs

| # | Run | Effective RTF | Sim s | Wall s | Scan Hz | Gazebo exit | Bridge exit |
|---:|---|---:|---:|---:|---:|---:|---:|
{good_table}

RTF min / mean / median / max / population stddev:
`{min(good_rtfs):.6f} / {statistics.mean(good_rtfs):.6f} / {statistics.median(good_rtfs):.6f} / {max(good_rtfs):.6f} / {statistics.pstdev(good_rtfs):.6f}`.

## Three retained low-RTF preflights

| Run | Scheduling | RTF | Scan Hz | Teardown failures |
|---|---|---:|---:|---|
{bad_table}

## PROVEN

- The 11 selected runs are exactly the complete 5 Hz fresh-domain runs with
  `success=true`, 3/3 successful waypoints, valid bags, and RTF >= 0.500.
- The ARES inputs match across all 11 good and all three bad runs: source-world
  `{common_manifest['src/ares_simulation/worlds/ares_test_world.sdf']}`, robot / LiDAR
  model `{common_manifest['src/ares_simulation/models/ares_jackal/model.sdf']}`, bridge
  `{common_manifest['src/ares_simulation/config/bridge_baseline.yaml']}`, launch
  `{common_manifest['src/ares_simulation/launch/ares_baseline.launch.py']}`, and mission
  `{common_manifest['src/ares_benchmark/config/mission.yaml']}`.
- Launch startup order is identical: `{', '.join(startup_names)}` (12 launch
  children), followed externally by bag recording and mission/health checking.
- Gazebo was server-only (`-s -r`), RViz / GUI was off, camera was off, the ROS
  scan bridge was active, and there was no native Gazebo scan subscriber.
- The launch command was `ros2 launch ares_simulation ares_baseline.launch.py
  rviz:=false camera:=false`; Gazebo used `gz sim -s -r --seed 42` with the
  generated temporary world.
- All 11 good runs used ROS scan consumers (Nav2 costmaps, localization,
  benchmark, and bag recording). Scan was therefore subscriber-activated.
- Good-run RTF strongly fell with sequential run index (Pearson
  `r={pearson_numerator / pearson_denominator:.3f}`), from 0.825095 on run 1 to
  0.576709 on run 11. This proves time/order correlation, not its cause.
- The normal 0.315 bad preflight used the same default CPU set as the good batch.
  Affinity improved the two other low-state trials only to 0.494 and 0.475.
- The good runs performed the moving three-goal mission; the three low-RTF runs
  were stationary `--health-only` preflights. This is a real workload difference,
  but later stationary A-D preflights reached 0.767-0.887, so health-only mode
  does not explain the low state by itself.
- Every good launch ended with both Gazebo and `parameter_bridge` at `-11`.
  Their historical `success=true` values predate corrected teardown accounting.

## LIKELY

- CPU / host scheduling pressure materially affects RTF. The nearest retained
  fast/slow process snapshots show the agent proxy around 26.8% versus 77.2% CPU,
  while Gazebo and bridge thread counts were unchanged. Later A-D trials in the
  same 6.6 boot recovered to 0.767-0.887 with unchanged ARES hashes.
- The 11 good runs and the three low preflights used ROS 2 Lyrical, Gazebo 10.5.0,
  D3D12 Intel Iris Xe through Mesa 26.0.8, and the 6.6.123.2 WSL2 kernel. These
  are directly captured near the runs and documented as common, but the good
  batch did not retain a per-run host snapshot.
- The historical WSL allocation was 12 logical CPUs, about 7.7 GiB RAM, 2 GiB
  swap, and a `.wslconfig` selecting the custom 6.6 kernel. The same-day host
  snapshot proves this later; it is not embedded in each good run.

## UNKNOWN

- Exact inherited environment for each good run (beyond runner-assigned
  `ROS_DOMAIN_ID`, `GZ_PARTITION`, and `ROS_LOG_DIR`) was not saved.
- Exact per-run CPU frequency, thermal state, Windows host load, and WSL graphics
  scheduler state were not captured.
- Exact historical runner hash and workspace Git hash cannot be recovered. The
  workspace root has no Git history and `run_healthy_trials.py` was edited after
  both batches. The current runner hash is
  `{current_files['scripts/run_healthy_trials.py']}`.
- Generated temporary-world file hashes and exact historical domain / partition
  values were not retained. Their source inputs and generation code are retained.
- Exact ROS subscriber count was not recorded; subscriber presence is proven.

## Correlation conclusion

No ARES source, sensor profile, bridge, launch order, renderer family, Gazebo
version, or kernel difference separates the 0.577-0.825 runs from the
0.315-0.494 runs. CPU affinity helps but is insufficient. The strongest retained
correlates are session/order and external scheduling load; the later same-boot
recovery without source changes proves the low state was transient. The new
controlled matrix must therefore measure load and process CPU on every run and
must preserve teardown failures independently of functional health.
"""

(output / "REPORT.md").write_text(report)
print(
    json.dumps(
        {
            "output": str(output),
            "good_count": len(good_runs),
            "good_rtf_min": min(good_rtfs),
            "good_rtf_mean": statistics.mean(good_rtfs),
            "good_rtf_max": max(good_rtfs),
            "all_source_hashes_equal": all_source_hashes_equal,
            "all_startup_orders_equal": all_startup_orders_equal,
        }
    )
)
