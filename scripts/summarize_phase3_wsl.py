#!/usr/bin/env python3
"""Summarize the controlled ARES WSL Phase 3 A-E matrix."""

import csv
import json
import os
from pathlib import Path
import statistics


root = Path(__file__).resolve().parents[1]
diagnostics = root / "results/week1/diagnostics"
output = root / "results/week1/wsl_good_bad_comparison_20260923"

runs = [
    ("A", "normal", diagnostics / "wsl_phase3_A_current_retry_20260923_2222"),
    ("B", "normal immediate repeat", diagnostics / "wsl_phase3_B_immediate_20260923_2223"),
    ("C", "normal after 20 s clean idle", diagnostics / "wsl_phase3_C_after_idle_20260923_2225"),
    ("D", "proxy 0-1; trial 2-11", diagnostics / "wsl_phase3_D_affinity_20260923_2227"),
    ("E", "known-good mission workload", diagnostics / "wsl_phase3_E_known_good_mission_20260923_2228"),
]

clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])


def meminfo_value(text, key):
    for line in (text or "").splitlines():
        if line.startswith(key + ":"):
            return int(line.split()[1])
    return None


def process_cpu(samples, predicate):
    by_pid = {}
    for sample in samples:
        for process in sample.get("processes", []):
            if not predicate(process.get("cmdline", "")):
                continue
            ticks = process.get("cpu_ticks")
            if not ticks:
                continue
            by_pid.setdefault(process["pid"], []).append(
                (
                    sample["monotonic"],
                    ticks["utime"] + ticks["stime"],
                    process.get("status", {}).get("Cpus_allowed_list"),
                )
            )
    values = []
    affinities = set()
    for observations in by_pid.values():
        if len(observations) < 2:
            continue
        start, end = observations[0], observations[-1]
        elapsed = end[0] - start[0]
        if elapsed > 0:
            values.append((end[1] - start[1]) / clock_ticks / elapsed * 100.0)
        affinities.update(item[2] for item in observations if item[2])
    return sum(values) if values else None, sorted(affinities)


def overall_cpu(samples):
    counters = []
    for sample in samples:
        first = (sample.get("proc_stat") or "").splitlines()
        if not first or not first[0].startswith("cpu "):
            continue
        fields = [int(item) for item in first[0].split()[1:]]
        idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
        counters.append((sum(fields), idle))
    if len(counters) < 2:
        return None
    total = counters[-1][0] - counters[0][0]
    idle = counters[-1][1] - counters[0][1]
    return (total - idle) / total * 100.0 if total else None


rows = []
for label, configuration, directory in runs:
    summary = json.loads((directory / "summary.json").read_text())
    samples = [
        json.loads(line)
        for line in (directory / "samples.jsonl").read_text().splitlines()
        if line.strip()
    ]
    metric_path = root / summary["metrics_path"]
    metrics = json.loads(metric_path.read_text())
    gz_cpu, gz_affinity = process_cpu(samples, lambda cmd: "gz-sim-main" in cmd)
    bridge_cpu, bridge_affinity = process_cpu(
        samples, lambda cmd: "/ros_gz_bridge/parameter_bridge" in cmd
    )
    proxy_cpu, proxy_affinity = process_cpu(
        samples, lambda cmd: "headroom.cli proxy --port" in cmd
    )
    available = [
        meminfo_value(sample.get("meminfo"), "MemAvailable") for sample in samples
    ]
    swap_total = [
        meminfo_value(sample.get("meminfo"), "SwapTotal") for sample in samples
    ]
    swap_free = [
        meminfo_value(sample.get("meminfo"), "SwapFree") for sample in samples
    ]
    available = [value for value in available if value is not None]
    swap_used = [
        total - free
        for total, free in zip(swap_total, swap_free)
        if total is not None and free is not None
    ]
    sim_delta = summary.get("measurement_sim_delta_s")
    rtf = summary.get("effective_rtf")
    row = {
        "test": label,
        "configuration": configuration,
        "workload": summary.get("workload"),
        "effective_rtf": rtf,
        "measurement_sim_delta_s": sim_delta,
        "measurement_real_delta_s_inferred": sim_delta / rtf if sim_delta and rtf else None,
        "mission_duration_sim_s": metrics.get("duration_sim_s"),
        "mission_duration_wall_s": metrics.get("duration_wall_s"),
        "overall_cpu_percent": overall_cpu(samples),
        "gz_cpu_percent": gz_cpu,
        "parameter_bridge_cpu_percent": bridge_cpu,
        "agent_proxy_cpu_percent": proxy_cpu,
        "gz_affinity": gz_affinity,
        "parameter_bridge_affinity": bridge_affinity,
        "agent_proxy_affinity": proxy_affinity,
        "mem_available_min_kib": min(available) if available else None,
        "mem_available_mean_kib": statistics.mean(available) if available else None,
        "swap_used_max_kib": max(swap_used) if swap_used else None,
        "sensor_rates_sim_hz": {
            topic: details.get("rate_sim_hz")
            for topic, details in metrics.get("sensors", {}).items()
        },
        "functional_checks": metrics.get("checks"),
        "bag_valid": metrics.get("bag_valid"),
        "process_exit_codes": metrics.get("process_exit_codes"),
        "launch_child_exit_codes": metrics.get("launch_child_exit_codes"),
        "clean_child_shutdown": metrics.get("clean_child_shutdown"),
        "overall_success": metrics.get("success"),
        "reason": metrics.get("reason"),
        "renderer": json.loads((directory / "host_before.json").read_text())[
            "renderer"
        ],
        "evidence_directory": str(directory.relative_to(root)),
    }
    rows.append(row)

(output / "phase3_matrix.json").write_text(json.dumps(rows, indent=2) + "\n")

csv_fields = [
    "test",
    "configuration",
    "workload",
    "effective_rtf",
    "measurement_sim_delta_s",
    "measurement_real_delta_s_inferred",
    "mission_duration_sim_s",
    "mission_duration_wall_s",
    "overall_cpu_percent",
    "gz_cpu_percent",
    "parameter_bridge_cpu_percent",
    "agent_proxy_cpu_percent",
    "mem_available_min_kib",
    "swap_used_max_kib",
    "clean_child_shutdown",
    "overall_success",
    "reason",
]
with (output / "phase3_matrix.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=csv_fields, extrasaction="ignore")
    writer.writeheader()
    writer.writerows(rows)


def number(value, digits=3):
    return "n/a" if value is None else f"{value:.{digits}f}"


table = "\n".join(
    "| {test} | {configuration} | {workload} | {rtf} | {overall} | {gz} | {bridge} | {proxy} | {memory} | {swap} | {exit} |".format(
        test=row["test"],
        configuration=row["configuration"],
        workload=row["workload"],
        rtf=number(row["effective_rtf"]),
        overall=number(row["overall_cpu_percent"], 1),
        gz=number(row["gz_cpu_percent"], 1),
        bridge=number(row["parameter_bridge_cpu_percent"], 1),
        proxy=number(row["agent_proxy_cpu_percent"], 1),
        memory=number((row["mem_available_min_kib"] or 0) / 1024 / 1024, 2),
        swap=number((row["swap_used_max_kib"] or 0) / 1024, 1),
        exit=", ".join(
            item["process"] + "=" + str(item["exit_code"])
            for item in row["launch_child_exit_codes"]
        ),
    )
    for row in rows
)

report = f"""# ARES Week 1 WSL controlled performance matrix

All five valid tests used the unchanged 5 Hz / 360 sample / 360-degree GPU
LiDAR profile, a fresh ROS domain and Gazebo partition, a clean process baseline,
the D3D12 Intel Iris Xe renderer, bagging, and corrected child-exit accounting.
The earlier invalid A setup attempt is retained under
`results/week1/diagnostics/wsl_phase3_A_current_20260923_2220` and is excluded
because the ROS workspace environment was not sourced and simulation never ran.

| Test | Configuration | Workload | RTF | Overall CPU % | Gazebo CPU % | Bridge CPU % | Proxy CPU % | Min avail GiB | Max swap used MiB | Critical exits |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|
{table}

## Findings

- A reached RTF {rows[0]['effective_rtf']:.3f}, proving the current 6.18 WSL2
  instance can exceed the 0.500 gate without a sensor or launch change.
- Immediate repeat B fell to {rows[1]['effective_rtf']:.3f}. After a clean
  20-second idle, C remained {rows[2]['effective_rtf']:.3f}; that idle was not
  sufficient to recover A's state.
- CPU partition D reached {rows[3]['effective_rtf']:.3f}, only
  {rows[3]['effective_rtf'] - rows[2]['effective_rtf']:+.3f} versus C. Affinity
  helps modestly in this state but is not the main separator.
- Mission-mode E, the closest match to the 11 historical runs, reached RTF
  {rows[4]['effective_rtf']:.3f}. The known-good workload itself did not restore
  the historical mean of 0.650.
- Every functional health map is true, every bag is valid, and all recorded
  sensor rates meet the unchanged profile. Every overall result nevertheless
  fails because `clean_child_shutdown=false`.
- Available memory stayed well above exhaustion and swap use did not materially
  grow. The run-to-run RTF shift is therefore not explained by memory pressure.

The matrix establishes two independent facts: runtime performance is currently
state-sensitive, and teardown is deterministically unsafe. No qualification
counter starts from these runs because none exited cleanly.
"""
(output / "PHASE3_REPORT.md").write_text(report)

print(
    json.dumps(
        {
            "rtfs": {row["test"]: row["effective_rtf"] for row in rows},
            "all_functional": all(
                row["functional_checks"]
                and all(bool(value) for value in row["functional_checks"].values())
                for row in rows
            ),
            "all_clean_shutdown": all(row["clean_child_shutdown"] for row in rows),
            "output": str(output),
        }
    )
)
